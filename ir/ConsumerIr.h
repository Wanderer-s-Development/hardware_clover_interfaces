/*
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/ir/BnConsumerIr.h>
#include <android-base/unique_fd.h>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace aidl {
namespace android {
namespace hardware {
namespace ir {

struct IrTxJob {
    std::vector<uint8_t> spiData;
    std::vector<int32_t> rawPattern;
    int32_t carrierFreqHz;
    bool isSpi;
};

class ConsumerIr : public BnConsumerIr {
  public:
    ConsumerIr();
    ~ConsumerIr() override;

    ::ndk::ScopedAStatus getCarrierFreqs(
            ::std::vector<::aidl::android::hardware::ir::ConsumerIrFreqRange>* _aidl_return)
            override;
    ::ndk::ScopedAStatus transmit(int32_t carrierFreqHz,
                                  const ::std::vector<int32_t>& pattern) override;

  private:
    std::vector<ConsumerIrFreqRange> kRangeVec;
    bool isInRange(int32_t carrierFreqHz);

    // Asynchronous background worker for non-blocking rapid transmissions
    std::thread mWorkerThread;
    std::mutex mQueueMutex;
    std::condition_variable mCv;
    std::queue<IrTxJob> mJobQueue;
    bool mStopWorker = false;

    void workerLoop();
};

}  // namespace ir
}  // namespace hardware
}  // namespace android
}  // namespace aidl
