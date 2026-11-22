/*
 * SPDX-FileCopyrightText: The Yasvara Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "HentakFollower.h"

#include "StreamPlayer.h"

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

namespace {

class Follower : public IVibratorDefault {
  public:
    explicit Follower(StreamPlayer* stream) : mStream(stream) {}

    ndk::ScopedAStatus getCapabilities(int32_t* caps) override {
        *caps = IVibrator::CAP_AMPLITUDE_CONTROL;
        return ndk::ScopedAStatus::ok();
    }
    ndk::ScopedAStatus setAmplitude(float level) override {
        mStream->follow(level);
        return ndk::ScopedAStatus::ok();
    }
    ndk::ScopedAStatus off() override {
        mStream->follow(0.f);
        return ndk::ScopedAStatus::ok();
    }
    ndk::ScopedAStatus getInterfaceVersion(int32_t* version) override {
        *version = IVibrator::version;
        return ndk::ScopedAStatus::ok();
    }
    ndk::ScopedAStatus getInterfaceHash(std::string* hash) override {
        *hash = IVibrator::hash;
        return ndk::ScopedAStatus::ok();
    }

  private:
    StreamPlayer* const mStream;
};

}  // namespace

std::shared_ptr<BnVibrator> makeHentakFollower() {
    StreamPlayer* stream = StreamPlayer::instance();
    if (stream == nullptr) return nullptr;
    return ndk::SharedRefBase::make<IVibratorDelegator>(
            ndk::SharedRefBase::make<Follower>(stream));
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
