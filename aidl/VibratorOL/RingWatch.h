/*
 * SPDX-FileCopyrightText: The Yasvara Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

// The kernel ends a stream on its own when it reads the chip in standby (its FIFO ran dry, or a
// fault stopped it) and tells no one: buffers queued after that are never taken. A queue that
// stops moving is the only sign.
class RingWatch {
  public:
    explicit RingWatch(int64_t stallNs) : mStallNs(stallNs) {}

    // At (re)start, once the ring is empty.
    void reset(int64_t now) {
        mQueued = 0;
        mSinceNs = now;
    }
    // Before filling, with the buffers still queued: true once none has been taken for the
    // stall time.
    bool stalled(int pending, int64_t now) {
        if (pending == 0 || pending < mQueued) mSinceNs = now;
        mQueued = pending;
        return pending > 0 && now - mSinceNs > mStallNs;
    }
    // After queueing one.
    void queued() { mQueued++; }

  private:
    const int64_t mStallNs;
    int64_t mSinceNs = 0;
    int mQueued = 0;
};

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
