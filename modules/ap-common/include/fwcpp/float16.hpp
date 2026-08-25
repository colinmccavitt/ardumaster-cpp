#pragma once

// Port of AP_Common/float16.h + float16.cpp. CPP-012.
//
// IEEE half-precision (binary16), NOT bfloat16 - upstream's own comment is
// explicit about this, worth repeating since the two are easy to confuse
// and are bit-incompatible.
//
// UNSAFE REINTERPRETATION: upstream reads/writes a `union FP32 { uint32_t
// u; float f; }` through whichever member wasn't last written - reading a
// non-active union member is undefined behavior in C++ (not C; this is a
// common point of confusion, and upstream is C++). It works everywhere in
// practice because every real compiler treats it as a supported extension,
// but ADR-0012's no-unsafe-reinterpretation stance (already applied to
// ap-param's is_sentinel, Matrix3::zero(), and Vector3::xy() being
// deferred) argues against reproducing it here too - and unlike xy(),
// there's no design question to defer: std::bit_cast<To>(from) (C++20) is
// a direct, well-defined, zero-cost replacement for exactly this pattern.
// Every union access below is written as an explicit bit_cast instead,
// preserving upstream's read/write SEQUENCE exactly (which values are read
// as bits vs. as float, and in what order) since that sequence is the
// algorithm - only the mechanism for the reinterpretation changed.

#include <bit>
#include <cstdint>

namespace fwcpp {

struct Float16 {
    std::uint16_t v16 = 0;

    [[nodiscard]] float get() const {
        constexpr std::uint32_t magic_bits = (254u - 15u) << 23u;
        constexpr std::uint32_t was_inf_nan_bits = (127u + 16u) << 23u;
        const float magic_f = std::bit_cast<float>(magic_bits);
        const float was_inf_nan_f = std::bit_cast<float>(was_inf_nan_bits);

        std::uint32_t out_bits = (static_cast<std::uint32_t>(v16) & 0x7FFFu) << 13u;
        const float out_f_scaled = std::bit_cast<float>(out_bits) * magic_f;
        out_bits = std::bit_cast<std::uint32_t>(out_f_scaled);
        if (out_f_scaled >= was_inf_nan_f) {
            out_bits |= 255u << 23u;
        }
        out_bits |= (static_cast<std::uint32_t>(v16) & 0x8000u) << 16u;
        return std::bit_cast<float>(out_bits);
    }

    void set(float value) {
        constexpr std::uint32_t f32inf_bits = 255u << 23u;
        constexpr std::uint32_t f16inf_bits = 31u << 23u;
        constexpr std::uint32_t magic_bits = 15u << 23u;
        constexpr std::uint32_t sign_mask = 0x80000000u;
        constexpr std::uint32_t round_mask = 0xFFFFF000u;
        const float magic_f = std::bit_cast<float>(magic_bits);

        std::uint32_t in_bits = std::bit_cast<std::uint32_t>(value);
        const std::uint32_t sign = in_bits & sign_mask;
        in_bits ^= sign; // clear sign -> bits of |value|

        v16 = 0;

        if (in_bits >= f32inf_bits) {
            v16 = (in_bits > f32inf_bits) ? std::uint16_t{0x7FFFu} : std::uint16_t{0x7C00u};
        } else {
            in_bits &= round_mask;
            const float in_f_scaled = std::bit_cast<float>(in_bits) * magic_f;
            in_bits = std::bit_cast<std::uint32_t>(in_f_scaled);
            in_bits -= round_mask;
            if (in_bits > f16inf_bits) {
                in_bits = f16inf_bits;
            }
            v16 = static_cast<std::uint16_t>(in_bits >> 13u);
        }
        v16 |= static_cast<std::uint16_t>(sign >> 16u);
    }
};

} // namespace fwcpp
