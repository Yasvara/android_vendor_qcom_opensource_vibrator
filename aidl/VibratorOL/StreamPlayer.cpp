/*
 * SPDX-FileCopyrightText: The Yasvara Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.qti.vibrator.stream"

#include "StreamPlayer.h"
#include "RingWatch.h"

#include <fcntl.h>
#include <log/log.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

namespace {

// drivers/input/misc/si_haptic: haptic_misc.h, haptic.h.
constexpr unsigned kIoctlGroup = 0x52;
constexpr unsigned long kIoctlSetGain = _IO(kIoctlGroup, 0x05);
constexpr unsigned long kIoctlStream = _IO(kIoctlGroup, 0x0A);
constexpr unsigned long kIoctlGetF0 = _IO(kIoctlGroup, 0x11);
constexpr unsigned long kIoctlStop = _IO(kIoctlGroup, 0x12);
constexpr uint8_t kBufValid = 0x55;
constexpr int kBufCount = 16;
constexpr int kBufData = 1000;
constexpr int kUnityGain = 0x80;

#pragma pack(push, 4)
struct MmapBuf {
    volatile uint8_t status;
    uint8_t bit;
    int16_t length;
    uint32_t reserve;
    uint64_t kernelNext;
    uint64_t userNext;
    uint8_t data[kBufData];
};
#pragma pack(pop)
static_assert(sizeof(MmapBuf) == 1024);

constexpr char kDevice[] = "/dev/si_haptic";
constexpr char kBoostNode[] = "/sys/bus/i2c/drivers/sih_haptic_688X/5-006b/drv_vboost";
// The bank the kernel loads into the chip; same file on stock HyperOS.
const std::vector<std::string> kBankPaths = {
        "/odm/firmware/aw8697_haptic.bin",
        "/vendor/firmware/aw8697_haptic.bin",
};
constexpr int kRate = 24000;

// Bench, 2026-11-03: linear to ~2 V of drive at f0, flat above. Full scale is the boost voltage.
constexpr float kKneeNormal = 0.28f;
constexpr float kKneeFirm = 0.40f;
constexpr float kKneeStrong = 0.55f;

// Carrier refill: 20 ms chunks, at most two queued beyond what the kernel already took.
constexpr size_t kCarrierChunk = 480;
constexpr int kCarrierAhead = 2;

// Coil: 6 × 150 ms of full-scale f0 at 9.2 V raised R by 10.8 % (≈ +28 °C); cools in ~15 s.
constexpr float kCoilOhm = 10.4f;
constexpr float kCoilJoulePerC = 0.13f;
constexpr float kCoilTauS = 15.f;
constexpr float kCoilSoftC = 25.f;
constexpr float kCoilHardC = 45.f;
constexpr float kCoilFloor = 0.35f;
// A boosted texture's added cycles cost ~3× the texture's own heat (~1 vs 0.3 °C a key at 150 %):
// they give way from here to the soft limit, so a long run of fast typing settles on the texture
// as HyperOS plays it instead of a dimmed one.
constexpr float kBoostShedC = 10.f;

// Follow: 5 ms chunks, two queued, so a level reaches the chip FIFO within ~10 ms.
constexpr size_t kFollowChunk = 120;
constexpr int kFollowAhead = 2;
constexpr int64_t kFollowWatchdogNs = 150'000'000;
constexpr int64_t kFollowIdleNs = 40'000'000;   // silent this long: brake and release the chip
constexpr int64_t kStopHoldNs = 30'000'000;
constexpr int64_t kStopWaitNs = 500'000'000;
// The kernel takes a follow chunk every 5 ms and a job chunk every ~42 ms while it streams.
constexpr int64_t kFollowStallNs = 50'000'000;
constexpr int64_t kJobStallNs = 120'000'000;
// The ring runs ~10 ms ahead of the chip in follow; a nice-0 thread under a game's load oversleeps
// that, the chip FIFO runs dry and the kernel ends the stream. Same as ANDROID_PRIORITY_URGENT_AUDIO.
constexpr int kWorkerNice = -19;
// Follow drive: a rise of more than kKickRise of the knee lands as a kick, kKickCycles at up to
// full scale sized by the rise (a round, a hit); the body then plays the level as it is, and a
// fall to rest gets one antiphase cycle. No feedback on the level: correcting toward a model of
// the mass overshoots after a kick and brakes into a dip and a climb back, a stutter per round.
constexpr float kKickRise = 0.25f;
constexpr int kKickCycles = 2;

StreamPlayer* sInstance = nullptr;

int64_t nowNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

float readBoostVolts() {
    FILE* f = fopen(kBoostNode, "r");
    if (!f) return 9.2f;
    char line[64] = {};
    size_t n = fread(line, 1, sizeof(line) - 1, f);
    fclose(f);
    line[n] = 0;
    const char* eq = strchr(line, '=');
    int tenths = atoi(eq ? eq + 1 : line);
    return tenths >= 60 && tenths <= 110 ? tenths / 10.f : 9.2f;
}

}  // namespace

StreamPlayer::StreamPlayer() : mDriveScale(kKneeNormal) {
    mFd = open(kDevice, O_RDWR | O_CLOEXEC);
    if (mFd < 0) {
        ALOGI("%s unavailable (%d): primitives stay off", kDevice, errno);
        return;
    }
    mRing = mmap(nullptr, sizeof(MmapBuf) * kBufCount, PROT_READ | PROT_WRITE, MAP_SHARED, mFd, 0);
    if (mRing == MAP_FAILED) {
        ALOGE("mmap %s failed (%d)", kDevice, errno);
        mRing = nullptr;
        close(mFd);
        mFd = -1;
        return;
    }
    uint32_t f0 = 0;
    if (ioctl(mFd, kIoctlGetF0, &f0) == 0 && f0 >= 1500 && f0 <= 1900) mF0 = f0 / 10.f;
    mBoostVolts = readBoostVolts();
    mTextures = TextureBank::load(kBankPaths);
    mCoilStampNs = nowNs();
    mThread = std::thread([this] { loop(); });
    sInstance = this;
    ALOGI("stream ready: f0 %.1f Hz, boost %.1f V", mF0, mBoostVolts);
}

StreamPlayer::~StreamPlayer() {
    if (sInstance == this) sInstance = nullptr;
    {
        std::lock_guard<std::mutex> lock(mLock);
        mExit = true;
        mGeneration++;
    }
    mWake.notify_all();
    if (mThread.joinable()) mThread.join();
    if (mRing) munmap(mRing, sizeof(MmapBuf) * kBufCount);
    if (mFd >= 0) close(mFd);
}

void StreamPlayer::setDrive(Drive drive) {
    std::lock_guard<std::mutex> lock(mLock);
    mDriveScale = drive == Drive::STRONG ? kKneeStrong
                  : drive == Drive::FIRM ? kKneeFirm
                                         : kKneeNormal;
    ALOGI("drive %d (sustain %.2f of full scale)", static_cast<int>(drive), mDriveScale);
}

float StreamPlayer::driveScale() {
    std::lock_guard<std::mutex> lock(mLock);
    return mDriveScale;
}

void StreamPlayer::submit(Job job) {
    Done dropped;
    std::unique_lock<std::mutex> lock(mLock);
    if (mHasJob) dropped = std::move(mJob.done);
    job.generation = ++mGeneration;
    mJob = std::move(job);
    mHasJob = true;
    lock.unlock();
    mWake.notify_all();
    if (dropped) dropped();
}

void StreamPlayer::playComposition(const std::vector<CompositeEffect>& composite, Done done) {
    const float knee = driveScale();
    std::vector<float> wave;
    bool stock = true;
    for (const CompositeEffect& e : composite) {
        wave.insert(wave.end(), static_cast<size_t>(e.delayMs) * kRate / 1000, 0.f);
        renderPrimitive(e.primitive, std::clamp(e.scale, 0.f, 1.f), knee, wave);
        stock &= isBankWave(e.primitive);
    }
    Job job;
    job.wave = std::move(wave);
    job.stock = stock;
    job.done = std::move(done);
    submit(std::move(job));
}

void StreamPlayer::playCarrier(int32_t ms, Done done) {
    Job job;
    job.carrierNs = static_cast<int64_t>(ms) * 1000000;
    job.done = std::move(done);
    submit(std::move(job));
}

bool StreamPlayer::playTexture(int wave, float scale, Done done) {
    if (!mTextures.has(wave)) return false;
    Job job;
    renderTexture(wave, std::clamp(scale, 0.f, 1.f), job.wave);
    job.stock = true;
    job.done = std::move(done);
    submit(std::move(job));
    return true;
}

void StreamPlayer::setReskin(int primitive, int wave) {
    if (primitive <= 0 || primitive >= kReskinCount) return;
    if (wave != 0 && !mTextures.has(wave)) return;
    if (mReskin[primitive].exchange(wave) != wave) ALOGI("primitive %d plays texture %d", primitive, wave);
}

void StreamPlayer::setTextureBoost(float boost) {
    boost = std::isfinite(boost) ? std::clamp(boost, 0.f, 1.f) : 0.f;
    if (mBoost.exchange(boost) != boost) ALOGI("texture boost %.2f", boost);
}

void StreamPlayer::setAmplitude(float amplitude) {
    mTargetAmplitude.store(std::clamp(amplitude, 0.f, 1.f));
}

StreamPlayer* StreamPlayer::instance() {
    return sInstance;
}

void StreamPlayer::follow(float level) {
    level = std::isfinite(level) ? std::clamp(level, 0.f, 1.f) : 0.f;
    const int64_t now = nowNs();
    const bool start = level > 0 &&
            (mFollowLevel.load() <= 0 || now - mFollowStampNs.load() > kFollowWatchdogNs);
    mFollowLevel.store(level);
    mFollowStampNs.store(now);
    if (start) {
        // Under the lock, so the worker can't miss it between its check and its wait.
        { std::lock_guard<std::mutex> lock(mLock); }
        mWake.notify_all();
    }
}

void StreamPlayer::holdFollow(int32_t ms) {
    std::lock_guard<std::mutex> lock(mLock);
    mFollowHoldUntilNs = std::max(mFollowHoldUntilNs, nowNs() + static_cast<int64_t>(ms) * 1000000);
}

int64_t StreamPlayer::followWaitLocked(int64_t now) const {
    if (mFollowLevel.load() <= 0 || now - mFollowStampNs.load() > kFollowWatchdogNs) return -1;
    return now < mFollowHoldUntilNs ? mFollowHoldUntilNs - now : 0;
}

void StreamPlayer::stop() {
    Done dropped;
    std::unique_lock<std::mutex> lock(mLock);
    if (mHasJob) dropped = std::move(mJob.done);
    mGeneration++;
    mHasJob = false;
    mFollowHoldUntilNs = std::max(mFollowHoldUntilNs, nowNs() + kStopHoldNs);
    mWake.notify_all();
    // Synchronous: the FF path may touch the chip right after this returns. Bounded, so a wedged
    // driver can't take system_server's VibrationThread (and its watchdog) down with it.
    if (!mIdle.wait_for(lock, std::chrono::nanoseconds(kStopWaitNs),
                        [this] { return mWorkerIdle || mExit; }))
        ALOGE("stop: worker still busy after %lld ms", static_cast<long long>(kStopWaitNs / 1000000));
    lock.unlock();
    if (dropped) dropped();
}

float StreamPlayer::coilRiseC() const {
    std::lock_guard<std::mutex> lock(mCoilLock);
    const float dt = (nowNs() - mCoilStampNs) / 1e9f;
    return dt > 0 ? mCoilC * std::exp(-dt / kCoilTauS) : mCoilC;
}

void StreamPlayer::coolLocked(int64_t now) {
    float dt = (now - mCoilStampNs) / 1e9f;
    if (dt > 0) mCoilC *= std::exp(-dt / kCoilTauS);
    mCoilStampNs = now;
}

float StreamPlayer::heat(const float* x, size_t n) {
    double sq = 0;
    for (size_t i = 0; i < n; i++) sq += x[i] * x[i];
    std::lock_guard<std::mutex> lock(mCoilLock);
    coolLocked(nowNs());
    // Worst case: no back-EMF, all of V²/R in the coil.
    mCoilC += static_cast<float>(sq / kRate) * mBoostVolts * mBoostVolts / kCoilOhm / kCoilJoulePerC;
    if (mCoilC <= kCoilSoftC) return 1.f;
    float t = (mCoilC - kCoilSoftC) / (kCoilHardC - kCoilSoftC);
    return std::max(kCoilFloor, 1.f - t * (1.f - kCoilFloor));
}

// Waveforms run at the tracked f0, where a volt buys the most acceleration. `knee` is the
// sustain level; the attack cycle goes to full scale and the stop cycle is in antiphase.
void StreamPlayer::renderPrimitive(CompositePrimitive primitive, float s, float knee,
                                   std::vector<float>& out) const {
    const int id = static_cast<int>(primitive);
    if (isTexture(primitive)) {
        renderTexture(id - kTextureBase, s, out);
        return;
    }
    if (id > 0 && id < kReskinCount) {
        const int wave = mReskin[id].load();
        if (wave > 0) {
            renderTexture(wave, s, out);
            return;
        }
    }
    const double w = 2 * M_PI * mF0 / kRate;
    const int cycle = static_cast<int>(std::lround(kRate / mF0));
    size_t i0 = out.size();
    auto tone = [&](double cycles, auto amp) {
        int n = static_cast<int>(cycles * cycle);
        for (int i = 0; i < n; i++) {
            size_t k = out.size();
            out.push_back(static_cast<float>(amp(i / double(n)) * std::sin(w * (k - i0))));
        }
    };
    auto flat = [](float a) { return [a](double) { return a; }; };

    switch (primitive) {
        case CompositePrimitive::NOOP:
            break;
        case CompositePrimitive::CLICK:
            tone(1, flat(s));
            tone(1, flat(knee * s));
            tone(1, flat(-0.8f * s));
            break;
        case CompositePrimitive::LIGHT_TICK:
            tone(1, flat(0.6f * s));
            tone(0.5, flat(-0.5f * s));
            break;
        case CompositePrimitive::LOW_TICK:
            tone(2, [&](double t) { return 0.5 * s * std::sin(M_PI * t); });
            tone(0.5, flat(-0.4f * s));
            break;
        case CompositePrimitive::THUD:
            tone(1, flat(s));
            tone(7, [&](double t) { return knee * s * std::exp(-2.3 * t); });
            tone(1, flat(-0.6f * s));
            break;
        case CompositePrimitive::SPIN:
            tone(1, flat(0.7f * s));
            tone(13, [&](double t) { return knee * s * std::sin(M_PI * t); });
            break;
        case CompositePrimitive::QUICK_RISE:
            tone(5, [&](double t) { return std::min(1.0, 1.2 * knee * s * (0.2 + t * t)); });
            tone(1, flat(-0.8f * s));
            break;
        case CompositePrimitive::SLOW_RISE:
            tone(20, [&](double t) { return knee * s * t; });
            tone(1, flat(-0.6f * s));
            break;
        case CompositePrimitive::QUICK_FALL:
            tone(1, flat(s));
            tone(6, [&](double t) { return knee * s * (1 - t); });
            break;
    }
}

// Bank waves are full-scale bursts already shaped for the motor (attack, two or three cycles,
// no tail), so they play as they are, scaled.
void StreamPlayer::renderTexture(int wave, float s, std::vector<float>& out) const {
    if (!mTextures.has(wave)) return;
    float boost = mBoost.load();
    if (boost > 0) {
        const float rise = coilRiseC();
        const float keep = std::clamp((kCoilSoftC - rise) / (kCoilSoftC - kBoostShedC), 0.f, 1.f);
        if (mBoostShed.exchange(keep < 1.f) != (keep < 1.f)) {
            ALOGI("coil +%.0f °C: texture boost %s", rise, keep < 1.f ? "giving way" : "back");
        }
        boost *= keep;
    }
    if (boost <= 0) {
        for (float x : mTextures.wave(wave)) out.push_back(x * s);
        return;
    }
    for (float x : TextureBank::boosted(mTextures.wave(wave), boost, mF0)) out.push_back(x * s);
}

bool StreamPlayer::isBankWave(CompositePrimitive primitive) const {
    const int id = static_cast<int>(primitive);
    if (isTexture(primitive)) return mTextures.has(id - kTextureBase);
    return id > 0 && id < kReskinCount && mReskin[id].load() > 0;
}

int32_t StreamPlayer::primitiveDurationMs(CompositePrimitive primitive) const {
    std::vector<float> wave;
    if (isTexture(primitive)) {
        renderTexture(static_cast<int>(primitive) - kTextureBase, 1.f, wave);
        return static_cast<int32_t>((wave.size() * 1000 + kRate - 1) / kRate);
    }
    renderPrimitive(primitive, 1.f, kKneeNormal, wave);
    return static_cast<int32_t>((wave.size() * 1000 + kRate - 1) / kRate);
}

// Fills n samples of the live carrier; amplitude slews over ~3 ms so steps don't click.
void StreamPlayer::renderCarrier(float* out, size_t n, float knee) {
    const double w = 2 * M_PI * mF0 / kRate;
    const int cycle = static_cast<int>(std::lround(kRate / mF0));
    const float target = mTargetAmplitude.load() * knee;
    const float slew = 1.f - std::exp(-1.f / (0.003f * kRate));
    for (size_t i = 0; i < n; i++) {
        float a;
        if (mCarrierSamples < cycle) {
            a = std::min(1.f, 2.5f * target);  // attack
        } else {
            mAmplitude += (target - mAmplitude) * slew;
            a = mAmplitude;
        }
        out[i] = static_cast<float>(a * std::sin(mPhase));
        mPhase += w;
        if (mPhase > 2 * M_PI) mPhase -= 2 * M_PI;
        mCarrierSamples++;
    }
}

static int pendingIn(const MmapBuf* ring) {
    int pending = 0;
    for (int i = 0; i < kBufCount; i++) pending += ring[i].status == kBufValid;
    return pending;
}

void StreamPlayer::loop() {
    MmapBuf* ring = static_cast<MmapBuf*>(mRing);
    std::vector<float> chunk(kBufData);
    if (setpriority(PRIO_PROCESS, 0, kWorkerNice) != 0) ALOGW("nice %d: errno %d", kWorkerNice, errno);
    for (;;) {
        Job job;
        uint64_t followGeneration = 0;
        {
            std::unique_lock<std::mutex> lock(mLock);
            mWorkerIdle = true;
            mIdle.notify_all();
            for (;;) {
                if (mExit) return;
                if (mHasJob) break;
                const int64_t wait = followWaitLocked(nowNs());
                if (wait == 0) {
                    followGeneration = mGeneration;
                    break;
                }
                if (wait > 0) {
                    mWake.wait_for(lock, std::chrono::nanoseconds(wait));
                } else {
                    // Wakes on a job, or on follow() going from 0 to a level.
                    mWake.wait_for(lock, std::chrono::milliseconds(200));
                }
            }
            if (!followGeneration) {
                job = std::move(mJob);
                mHasJob = false;
            }
            mWorkerIdle = false;
        }
        if (followGeneration) {
            runFollow(followGeneration);
            continue;
        }
        const float knee = driveScale();
        auto current = [&] {
            std::lock_guard<std::mutex> lock(mLock);
            return mGeneration == job.generation && !mExit;
        };

        ioctl(mFd, kIoctlSetGain, kUnityGain);
        ioctl(mFd, kIoctlStream, 0);
        RingWatch watch(kJobStallNs);
        watch.reset(nowNs());
        const bool carrier = job.wave.empty();
        const size_t carrierSamples = static_cast<size_t>(job.carrierNs * kRate / 1000000000LL);
        mCarrierSamples = 0;
        mAmplitude = 0;
        mPhase = 0;

        size_t pos = 0;      // composition samples handed over
        size_t written = 0;  // all samples handed over
        int idx = 0;
        int64_t firstTaken = 0;
        bool braked = false;
        bool live = true;
        bool stalled = false;
        while (live) {
            live = current();
            if (!live) break;
            if (!firstTaken && written && ring[0].status != kBufValid) firstTaken = nowNs();

            const int pending = pendingIn(ring);
            // Late is worse than dropped for a UI effect; the next one restarts the stream.
            if ((stalled = watch.stalled(pending, nowNs()))) {
                ALOGW("stream stalled with %d queued: effect dropped", pending);
                break;
            }
            MmapBuf& b = ring[idx];
            if (b.status == kBufValid || (carrier && pending >= kCarrierAhead)) {
                usleep(1000);
                continue;
            }

            size_t n;
            if (!carrier) {
                n = std::min<size_t>(kBufData, job.wave.size() - pos);
                if (n == 0) break;
                std::copy_n(job.wave.data() + pos, n, chunk.data());
                pos += n;
            } else if (written < carrierSamples) {
                // Counted in samples: the queue runs ahead of the motor, a clock would overshoot.
                n = std::min(kCarrierChunk, carrierSamples - written);
                renderCarrier(chunk.data(), n, knee);
            } else if (!braked) {
                // Stop cycle in antiphase, from the amplitude it had.
                const double w = 2 * M_PI * mF0 / kRate;
                n = static_cast<size_t>(std::lround(kRate / mF0));
                for (size_t i = 0; i < n; i++) {
                    chunk[i] = static_cast<float>(-0.8f * mAmplitude * std::sin(mPhase));
                    mPhase += w;
                }
                braked = true;
            } else {
                break;
            }

            // A stock job's heat still counts against the game stream; its boost already gave way.
            const float heatGain = heat(chunk.data(), n);
            const float g = job.stock ? 1.f : heatGain;
            for (size_t i = 0; i < n; i++)
                b.data[i] = static_cast<uint8_t>(static_cast<int8_t>(
                        std::clamp(std::lround(chunk[i] * g * 127), -127L, 127L)));
            b.length = static_cast<int16_t>(n);
            __atomic_thread_fence(__ATOMIC_RELEASE);
            b.status = kBufValid;
            watch.queued();
            written += n;
            idx = (idx + 1) % kBufCount;
        }

        // Let the queued samples and the chip FIFO play out, unless something replaced us.
        if (live && !stalled) {
            for (int spin = 0; spin < 1000 && current(); spin++) {
                const int pending = pendingIn(ring);
                if (!pending || watch.stalled(pending, nowNs())) break;
                if (!firstTaken && ring[0].status != kBufValid) firstTaken = nowNs();
                usleep(1000);
            }
            if (!firstTaken) firstTaken = nowNs();
            const int64_t end = firstTaken + static_cast<int64_t>(written) * 1000000000LL / kRate;
            while (current() && nowNs() < end) usleep(1000);
        }
        // A newer job restarts the stream itself; stop only when nothing follows.
        bool next;
        {
            std::lock_guard<std::mutex> lock(mLock);
            next = mHasJob;
        }
        if (!next) ioctl(mFd, kIoctlStop, 0);
        if (job.done) job.done();
    }
}

void StreamPlayer::runFollow(uint64_t generation) {
    MmapBuf* ring = static_cast<MmapBuf*>(mRing);
    std::vector<float> chunk(kFollowChunk);
    const float knee = driveScale();
    const double w = 2 * M_PI * mF0 / kRate;
    const int cycle = static_cast<int>(std::lround(kRate / mF0));
    auto current = [&] {
        std::lock_guard<std::mutex> lock(mLock);
        return mGeneration == generation && !mExit;
    };

    ioctl(mFd, kIoctlSetGain, kUnityGain);
    ioctl(mFd, kIoctlStream, 0);
    RingWatch watch(kFollowStallNs);
    const int64_t startNs = nowNs();
    watch.reset(startNs);
    int64_t firstTakeNs = 0;
    double phase = 0;
    float wanted = 0;     // level at the end of the last chunk
    int kickLeft = 0, brakeLeft = 0;
    float kickAmp = 0, brakeAmp = 0;
    int64_t silentNs = 0;
    int idx = 0;
    int restarts = 0;
    bool live = true;
    while ((live = current())) {
        const int pending = pendingIn(ring);
        const int64_t now = nowNs();
        if (watch.stalled(pending, now)) {
            // The chip stopped with the mass at rest; start over from buffer 0.
            if (++restarts <= 3 || restarts % 100 == 0)
                ALOGW("stream stalled with %d queued at %lld ms (first take %lld ms): restart %d",
                      pending, static_cast<long long>((now - startNs) / 1000000),
                      static_cast<long long>(firstTakeNs ? (firstTakeNs - startNs) / 1000000 : -1),
                      restarts);
            ioctl(mFd, kIoctlStream, 0);
            watch.reset(nowNs());
            idx = 0;
            phase = 0;
            wanted = 0;
            kickLeft = brakeLeft = 0;
            continue;
        }
        if (!firstTakeNs && idx > pending) firstTakeNs = now;
        MmapBuf& b = ring[idx];
        if (b.status == kBufValid || pending >= kFollowAhead) {
            usleep(1000);
            continue;
        }
        float level = now - mFollowStampNs.load() > kFollowWatchdogNs ? 0.f : mFollowLevel.load();
        const float target = level * knee;
        if (target - wanted > kKickRise * knee) {
            kickLeft = kKickCycles * cycle;
            kickAmp = std::clamp(0.4f + 0.6f * (target - wanted) / knee, 0.4f, 1.f);
        }
        if (target <= 0 && wanted > 0.05f * knee) {
            brakeLeft = cycle;
            brakeAmp = wanted;
        }
        // Rises land at once (that's the hit); falls are spread over the chunk, as the audio
        // envelope decays between updates.
        const float from = target > wanted ? target : wanted;
        for (size_t i = 0; i < kFollowChunk; i++) {
            const float l = from + (target - from) * (i + 1) / kFollowChunk;
            float d = l;
            if (kickLeft > 0) {
                kickLeft--;
                d = kickAmp;
            } else if (brakeLeft > 0) {
                brakeLeft--;
                d = -brakeAmp;
            }
            chunk[i] = static_cast<float>(d * std::sin(phase));
            phase += w;
            if (phase > 2 * M_PI) phase -= 2 * M_PI;
        }
        wanted = target;
        silentNs = target <= 0 && kickLeft == 0 && brakeLeft == 0
                ? silentNs + static_cast<int64_t>(kFollowChunk) * 1000000000LL / kRate : 0;
        if (silentNs >= kFollowIdleNs) break;

        const float g = heat(chunk.data(), kFollowChunk);
        for (size_t i = 0; i < kFollowChunk; i++)
            b.data[i] = static_cast<uint8_t>(static_cast<int8_t>(
                    std::clamp(std::lround(chunk[i] * g * 127), -127L, 127L)));
        b.length = static_cast<int16_t>(kFollowChunk);
        __atomic_thread_fence(__ATOMIC_RELEASE);
        b.status = kBufValid;
        watch.queued();
        idx = (idx + 1) % kBufCount;
    }
    if (live) {
        // Already braked to rest: let the queue drain, then release the chip.
        for (int spin = 0; spin < 100 && current(); spin++) {
            if (!pendingIn(ring)) break;
            usleep(1000);
        }
    }
    bool next;
    {
        std::lock_guard<std::mutex> lock(mLock);
        next = mHasJob;
    }
    if (!next) ioctl(mFd, kIoctlStop, 0);
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
