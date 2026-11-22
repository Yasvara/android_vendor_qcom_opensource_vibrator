/*
 * SPDX-FileCopyrightText: The Yasvara Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/vibrator/BnVibrator.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "TextureBank.h"

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

// Plays synthesized waveforms through the sih688x RTP stream (/dev/si_haptic, s8 PCM at 24 kHz).
//
// The LRA saturates mechanically at about a quarter of full scale at f0, so waveforms sustain at
// that knee and spend full scale only on the first cycle (attack) and an antiphase cycle (stop).
// A coil temperature model, integrated from the samples actually played, scales output down
// before the coil overheats.
//
// Textures are the chip RAM bank's waves played as they are, at any scale: private primitive ids
// kTextureBase + wave. A standard primitive can be reskinned to play a texture instead of its
// synthesized shape.
class StreamPlayer {
  public:
    using Done = std::function<void()>;

    // Sustain level, set through alwaysOnEnable(kDriveId, CLICK, LIGHT|MEDIUM|STRONG).
    enum class Drive { NORMAL, FIRM, STRONG };
    static constexpr int32_t kDriveId = 0x4D;
    static constexpr int32_t kTextureBase = 0x100;
    // alwaysOnEnable(kReskinId + primitive, effect = wave, _); wave 0 or alwaysOnDisable restores.
    static constexpr int32_t kReskinId = 0x5200;
    // alwaysOnEnable(kBoostId, effect = 0..100, _): TextureBank::boosted() for every texture.
    static constexpr int32_t kBoostId = 0x5300;
    static constexpr int kReskinCount = static_cast<int>(CompositePrimitive::LOW_TICK) + 1;

    StreamPlayer();
    ~StreamPlayer();

    bool ready() const { return mFd >= 0; }

    // Replaces whatever plays; done runs once the last sample has left the motor, or after a
    // stop or a newer job.
    void playComposition(const std::vector<CompositeEffect>& composite, Done done);
    // f0 carrier for ms at the live amplitude from setAmplitude().
    void playCarrier(int32_t ms, Done done);
    // Texture wave (from 1) at scale; false when the bank has no such wave.
    bool playTexture(int wave, float scale, Done done);
    void setAmplitude(float amplitude);
    // Returns once the stream is stopped. Follow stays off for a moment so the caller can use the
    // chip; holdFollow() extends that.
    void stop();

    // Live envelope from game audio (the "hentak" instance): an f0 carrier tracks level, 0..1 of
    // the sustain knee, with overdrive on rises and antiphase braking on falls. Anything played
    // through the default instance takes over while it lasts. Updates must keep coming: a level
    // older than the watchdog counts as 0.
    void follow(float level);
    void holdFollow(int32_t ms);
    static StreamPlayer* instance();

    int32_t primitiveDurationMs(CompositePrimitive primitive) const;
    const TextureBank& textures() const { return mTextures; }
    static bool isTexture(CompositePrimitive primitive) {
        return static_cast<int32_t>(primitive) > kTextureBase;
    }
    void setReskin(int primitive, int wave);
    void setTextureBoost(float boost);
    void setDrive(Drive drive);
    // Estimated coil rise over ambient, °C.
    float coilRiseC() const;

  private:
    struct Job {
        std::vector<float> wave;  // empty: carrier
        int64_t carrierNs = 0;
        Done done;
        uint64_t generation = 0;
        // Bank waves at most full scale: what HyperOS plays from RAM, with no thermal limit.
        bool stock = false;
    };

    void submit(Job job);
    void loop();
    // 0: follow now; > 0: ns until it may; < 0: nothing to follow.
    int64_t followWaitLocked(int64_t now) const;
    void runFollow(uint64_t generation);
    float driveScale();
    void renderPrimitive(CompositePrimitive primitive, float scale, float knee,
                         std::vector<float>& out) const;
    void renderTexture(int wave, float scale, std::vector<float>& out) const;
    bool isBankWave(CompositePrimitive primitive) const;
    void renderCarrier(float* out, size_t n, float knee);
    // Adds n samples to the coil model and returns the output scale it allows.
    float heat(const float* x, size_t n);
    void coolLocked(int64_t now);

    int mFd = -1;
    void* mRing = nullptr;
    float mF0 = 170.f;
    float mBoostVolts = 9.2f;
    TextureBank mTextures;
    std::array<std::atomic<int>, kReskinCount> mReskin{};
    std::atomic<float> mBoost{0.f};
    mutable std::atomic<bool> mBoostShed{false};

    std::mutex mLock;
    std::condition_variable mWake;
    std::condition_variable mIdle;
    Job mJob;
    bool mHasJob = false;
    bool mExit = false;
    bool mWorkerIdle = true;
    uint64_t mGeneration = 0;
    float mDriveScale;
    std::atomic<float> mTargetAmplitude{1.f};
    std::atomic<float> mFollowLevel{0.f};
    std::atomic<int64_t> mFollowStampNs{0};
    int64_t mFollowHoldUntilNs = 0;

    // Worker thread only.
    float mAmplitude = 0.f;
    double mPhase = 0;
    int mCarrierSamples = 0;

    mutable std::mutex mCoilLock;
    float mCoilC = 0.f;
    int64_t mCoilStampNs = 0;

    std::thread mThread;
};

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
