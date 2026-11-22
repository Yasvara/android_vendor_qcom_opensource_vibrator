/*
 * SPDX-FileCopyrightText: The Yasvara Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

// The waves of the haptic chip's RAM bank (aw8697_haptic.bin, the file the kernel loads into the
// chip): what prebaked effect N plays as wave N + 1. Kept as s8 PCM at 24 kHz, scaled to ±1, so
// the same shapes can be streamed at any scale.
//
// Layout: [0:2] checksum, [2:4] RAM base address (BE), [4] 0x55 at the base, then (start, end)
// BE u16 per wave, contiguous; RAM address A is file offset 4 + A - base.
class TextureBank {
  public:
    // Empty when no file parses.
    static TextureBank load(const std::vector<std::string>& paths);
    static TextureBank parse(const std::vector<uint8_t>& file);

    // Waves are numbered from 1, as the chip does.
    int count() const { return static_cast<int>(mWaves.size()); }
    bool has(int wave) const { return wave >= 1 && wave <= count(); }
    const std::vector<float>& wave(int wave) const { return mWaves[wave - 1]; }
    int32_t durationMs(int wave) const;

    // A texture made stronger, boost 0..1: its whole cycles as designed, then round(3 × boost)
    // sine cycles at the motor's f0 at 0.6 + 0.4 × boost of full scale and one antiphase cycle to
    // stop. The bank's waves sit off this motor's resonance (220-260 or ~115 Hz), so their own
    // cycles repeated or driven harder only buzz; nothing here clips. 0 returns the wave unchanged.
    static std::vector<float> boosted(const std::vector<float>& wave, float boost, float f0);

  private:
    std::vector<std::vector<float>> mWaves;
};

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
