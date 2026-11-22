/*
 * SPDX-FileCopyrightText: The Yasvara Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/vibrator/BnVibrator.h>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

// "IVibrator/hentak": audioserver streams a game's haptic envelope here as setAmplitude()
// (0 allowed) and off(); it plays through StreamPlayer::follow(). Null without the stream device.
std::shared_ptr<BnVibrator> makeHentakFollower();

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
