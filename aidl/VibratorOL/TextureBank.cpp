/*
 * SPDX-FileCopyrightText: The Yasvara Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.qti.vibrator.textures"

#include "TextureBank.h"

#include <log/log.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

namespace {

constexpr int kRate = 24000;
constexpr size_t kTable = 5;
constexpr uint8_t kBaseMark = 0x55;
// 64 KiB of chip RAM at most; a wave longer than this is a parse error, not a texture.
constexpr size_t kMaxFile = 1 << 16;
constexpr size_t kMaxWave = kRate / 5;

uint16_t be16(const std::vector<uint8_t>& d, size_t i) {
    return static_cast<uint16_t>(d[i] << 8 | d[i + 1]);
}

}  // namespace

TextureBank TextureBank::parse(const std::vector<uint8_t>& d) {
    TextureBank bank;
    if (d.size() < kTable + 4 || d.size() > kMaxFile || d[4] != kBaseMark) return bank;
    const uint32_t base = be16(d, 2);
    const uint32_t end = base + static_cast<uint32_t>(d.size()) - 4;  // one past the last address
    uint32_t first = 0, last = 0;
    for (size_t i = kTable; i + 4 <= d.size(); i += 4) {
        const uint32_t s = be16(d, i);
        const uint32_t e = be16(d, i + 2);
        if (s <= base || e < s || e >= end) break;
        if (last && s != last + 1) break;
        if (e - s + 1 > kMaxWave) break;
        std::vector<float> w;
        w.reserve(e - s + 1);
        for (uint32_t a = s; a <= e; a++) w.push_back(static_cast<int8_t>(d[4 + a - base]) / 127.f);
        bank.mWaves.push_back(std::move(w));
        if (!first) first = s;
        last = e;
    }
    // The table ends where the first wave starts; entries read past that are wave data.
    if (first) bank.mWaves.resize(std::min<size_t>(bank.mWaves.size(), (first - base - 1) / 4));
    return bank;
}

TextureBank TextureBank::load(const std::vector<std::string>& paths) {
    for (const std::string& path : paths) {
        FILE* f = fopen(path.c_str(), "rbe");
        if (!f) continue;
        std::vector<uint8_t> d(kMaxFile + 1);
        size_t n = fread(d.data(), 1, d.size(), f);
        fclose(f);
        d.resize(n);
        TextureBank bank = parse(d);
        if (bank.count() > 0) {
            ALOGI("%d textures from %s", bank.count(), path.c_str());
            return bank;
        }
        ALOGW("%s: no wave table", path.c_str());
    }
    return {};
}

std::vector<float> TextureBank::boosted(const std::vector<float>& x, float boost, float f0) {
    boost = std::clamp(boost, 0.f, 1.f);
    const int cycles = static_cast<int>(std::floor(3.f * boost + 0.5f));
    size_t splice = 0;  // the last positive-going zero crossing: whole cycles end there
    for (size_t i = 1; i < x.size(); i++) {
        if (x[i - 1] < 0 && x[i] >= 0) splice = i;
    }
    if (cycles == 0 || splice == 0 || f0 <= 0) return x;
    const float level = 0.6f + 0.4f * boost;
    const double w = 2 * M_PI * f0 / kRate;
    const size_t body = static_cast<size_t>(std::lround(cycles * kRate / f0));
    const size_t end = static_cast<size_t>(std::lround((cycles + 1) * kRate / f0));
    std::vector<float> y(x.begin(), x.begin() + splice);
    y.reserve(splice + end);
    for (size_t k = 0; k < end; k++) {
        y.push_back(static_cast<float>((k < body ? level : -level) * std::sin(w * k)));
    }
    return y;
}

int32_t TextureBank::durationMs(int wave) const {
    if (!has(wave)) return 0;
    return static_cast<int32_t>((mWaves[wave - 1].size() * 1000 + kRate - 1) / kRate);
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
