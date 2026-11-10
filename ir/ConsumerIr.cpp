/*
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ConsumerIr.h"

#include <fcntl.h>
#include <linux/lirc.h>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>
#include <sys/ioctl.h>
#include <unistd.h>

#include <android-base/logging.h>
#include <android-base/unique_fd.h>
#include <ir.sysprop.h>

using namespace ::vendor::lineage::ir;

namespace aidl {
namespace android {
namespace hardware {
namespace ir {

static constexpr uint32_t kSpiClockHz = 1920000; // 1.92 MHz on SM4450 QUPv3 SE6

static const std::string kIrSpiDevice = "/dev/ir_spi";
static const std::string kLircDevice = "/dev/lirc0";

static std::vector<uint8_t> modulatePatternToSpi(int32_t carrierFreqHz,
                                                const std::vector<int32_t>& pattern,
                                                size_t entries) {
    const double bit_period_us = 1000000.0 / static_cast<double>(kSpiClockHz);
    const double carrier_period_us = 1000000.0 / static_cast<double>(carrierFreqHz);
    // 33.3% duty cycle matches Vishay TSOP and industry-standard IR demodulator bandpass AGC,
    // avoiding saturation gain reduction and maximizing room-wide reception range.
    const double carrier_high_us = carrier_period_us * 0.333333;

    std::vector<uint8_t> spi_bytes;
    uint8_t current_byte = 0;
    int bit_count = 0;

    for (size_t i = 0; i < entries; ++i) {
        const bool is_pulse = (i % 2 == 0);
        const double duration_us = static_cast<double>(pattern[i]);
        double elapsed_us = 0.0;

        while (elapsed_us < duration_us) {
            uint8_t bit = 0;
            if (is_pulse) {
                double phase = fmod(elapsed_us, carrier_period_us);
                bit = (phase < carrier_high_us) ? 1 : 0;
            }
            current_byte = (current_byte << 1) | bit;
            bit_count++;
            if (bit_count == 8) {
                spi_bytes.push_back(current_byte);
                current_byte = 0;
                bit_count = 0;
            }
            elapsed_us += bit_period_us;
        }
    }

    if (bit_count > 0) {
        spi_bytes.push_back(static_cast<uint8_t>(current_byte << (8 - bit_count)));
    }

    // Trailing silence padding (16 bytes = 66.7 us of carrier off) to guarantee
    // the SPI MOSI line settles low (IR LED completely OFF) before transfer ends.
    for (int k = 0; k < 16; ++k) {
        spi_bytes.push_back(0x00);
    }

    // ir-spi driver sets bits_per_word = 32 (4 bytes).
    // Linux kernel __spi_validate() enforces len % (bits_per_word / 8) == 0.
    // If not divisible by 4, write fails with error 22 (-EINVAL).
    while (spi_bytes.size() % 4 != 0) {
        spi_bytes.push_back(0x00);
    }

    // Qualcomm QUPv3 Serial Engine in SPI master mode (spi-msm-geni) transmits 32-bit words
    // (bits_per_word = 32) MSB first (bit 31 down to bit 0).
    // On little-endian ARM64 architecture, writing sequential bytes [B0, B1, B2, B3] into memory
    // causes the hardware to emit [B3, B2, B1, B0] on the wire, reversing time every 16.6 us.
    // Swap the 4 bytes within each 32-bit word so that bytes are clocked out in exact chronological order:
    // [B0, B1, B2, B3] on the physical wire.
    for (size_t i = 0; i < spi_bytes.size(); i += 4) {
        std::swap(spi_bytes[i], spi_bytes[i + 3]);
        std::swap(spi_bytes[i + 1], spi_bytes[i + 2]);
    }

    return spi_bytes;
}

ConsumerIr::ConsumerIr() {
    auto carrier_freqs = IrProperties::carrier_freqs();

    if (carrier_freqs.size() >= 2) {
        for (size_t i = 0; i < carrier_freqs.size() - 1; i += 2) {
            if (!carrier_freqs[i] || !carrier_freqs[i + 1]) {
                continue;
            }

            kRangeVec.push_back({
                    .minHz = carrier_freqs[i].value(),
                    .maxHz = carrier_freqs[i + 1].value(),
            });
        }
    } else {
        kRangeVec.push_back({.minHz = 30000, .maxHz = 60000});
    }

    // Start background worker thread for non-blocking asynchronous transmission
    mWorkerThread = std::thread(&ConsumerIr::workerLoop, this);
}

ConsumerIr::~ConsumerIr() {
    {
        std::lock_guard<std::mutex> lock(mQueueMutex);
        mStopWorker = true;
    }
    mCv.notify_all();

    if (mWorkerThread.joinable()) {
        mWorkerThread.join();
    }
}

void ConsumerIr::workerLoop() {
    while (true) {
        IrTxJob job;
        {
            std::unique_lock<std::mutex> lock(mQueueMutex);
            mCv.wait(lock, [this] { return mStopWorker || !mJobQueue.empty(); });

            if (mStopWorker && mJobQueue.empty()) {
                break;
            }

            job = std::move(mJobQueue.front());
            mJobQueue.pop();
        }

        if (job.isSpi) {
            ::android::base::unique_fd fd(open(kIrSpiDevice.c_str(), O_WRONLY | O_CLOEXEC));
            if (!fd.ok()) {
                LOG(ERROR) << "workerLoop: Failed to open " << kIrSpiDevice << ": " << strerror(errno);
                continue;
            }

            // In ir-spi.ko, write() dynamically allocates kmalloc(count), performs spi_sync,
            // frees the buffer, and resets buffer_size to 0. Closing the FD decrements refcount
            // back to 0. This avoids driver krealloc pointer corruption and -EINVAL errors.
            ssize_t written = write(fd.get(), job.spiData.data(), job.spiData.size());
            if (written < 0 || static_cast<size_t>(written) != job.spiData.size()) {
                LOG(ERROR) << "workerLoop: write SPI failed (" << job.spiData.size()
                           << " bytes), written=" << written << ", error: " << errno;
            }

            // 15ms quiet gap between transmissions ensures TV IR receiver TSOP AGC filter
            // and decoder state machine have completely reset before transmitting repeat frames.
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        } else {
            ::android::base::unique_fd fd(open(kLircDevice.c_str(), O_WRONLY | O_CLOEXEC));
            if (!fd.ok()) {
                LOG(ERROR) << "workerLoop: Failed to open " << kLircDevice << ": " << strerror(errno);
                continue;
            }

            int rc = ioctl(fd.get(), LIRC_SET_SEND_CARRIER, &job.carrierFreqHz);
            if (rc < 0) {
                LOG(ERROR) << "workerLoop: set LIRC carrier failed: " << strerror(errno);
                continue;
            }

            rc = write(fd.get(), job.rawPattern.data(), job.rawPattern.size() * sizeof(int32_t));
            if (rc < 0) {
                LOG(ERROR) << "workerLoop: write LIRC pattern failed: " << strerror(errno);
                continue;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    }
}

::ndk::ScopedAStatus ConsumerIr::getCarrierFreqs(std::vector<ConsumerIrFreqRange>* _aidl_return) {
    *_aidl_return = kRangeVec;

    return ::ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus ConsumerIr::transmit(int32_t carrierFreqHz,
                                          const std::vector<int32_t>& pattern) {
    size_t entries = pattern.size();

    if (entries == 0) {
        return ::ndk::ScopedAStatus::ok();
    }

    if (!isInRange(carrierFreqHz)) {
        LOG(ERROR) << "Unsupported carrier " << carrierFreqHz;
        return ::ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    // Pattern must end with an on (pulse) period
    if (entries % 2 == 0) {
        entries--;
    }

    bool is_spi = (access(kIrSpiDevice.c_str(), F_OK) == 0);

    IrTxJob job;
    job.isSpi = is_spi;
    job.carrierFreqHz = carrierFreqHz;

    if (is_spi) {
        job.spiData = modulatePatternToSpi(carrierFreqHz, pattern, entries);
    } else {
        job.rawPattern.assign(pattern.begin(), pattern.begin() + entries);
    }

    {
        std::lock_guard<std::mutex> lock(mQueueMutex);
        // Allow up to 6 jobs queued (preserves multi-frame remote commands like repeat bursts
        // while bounding backlog if user holds a button down)
        while (mJobQueue.size() >= 6) {
            mJobQueue.pop();
        }
        mJobQueue.push(std::move(job));
    }
    mCv.notify_one();

    // Return immediately to Android framework (releases system_server mHalLock and avoids caller UI freeze)
    return ::ndk::ScopedAStatus::ok();
}

bool ConsumerIr::isInRange(int32_t carrierFreqHz) {
    for (const auto& range : kRangeVec) {
        if (carrierFreqHz >= range.minHz && carrierFreqHz <= range.maxHz) {
            return true;
        }
    }
    return false;
}

}  // namespace ir
}  // namespace hardware
}  // namespace android
}  // namespace aidl
