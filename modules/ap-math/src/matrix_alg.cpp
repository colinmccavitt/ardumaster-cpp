// Definitions for matrix_alg.hpp. Compiled under fwcpp_upstream_flags for
// consistency with the rest of ap-math (see scalar.hpp's file banner) even
// though nothing in this file currently has an ambiguous bare double
// literal - matrix code is exactly the kind of place a future addition
// might introduce one, and getting the flag applied is then free.
//
// Explicitly instantiated for float and double only, matching upstream's
// own instantiation list at the bottom of matrix_alg.cpp - nothing upstream
// calls these with any other T.

#include <fwcpp/math/matrix_alg.hpp>
#include <fwcpp/math/scalar.hpp>

#include <cmath>

namespace fwcpp::math {

// mat_mul/mat_identity are header-only (see matrix_alg.hpp) - no ambiguous
// literal, so no reason to duplicate them here.

template <typename T>
bool inverse3x3(const std::array<T, 9>& m, std::array<T, 9>& inv_out) {
    const T det = m[0] * (m[4] * m[8] - m[7] * m[5])
                - m[1] * (m[3] * m[8] - m[5] * m[6])
                + m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (is_zero(det) || std::isinf(det)) {
        return false;
    }
    const T invdet = T(1) / det;

    inv_out[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
    inv_out[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    inv_out[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    inv_out[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
    inv_out[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    inv_out[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
    inv_out[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
    inv_out[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
    inv_out[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;
    return true;
}

template <typename T>
bool inverse4x4(const std::array<T, 16>& m, std::array<T, 16>& inv_out) {
    std::array<T, 16> inv{};

    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15]
           + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15]
           - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15]
           + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14]
            - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];

    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15]
           - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15]
           + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15]
           - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14]
            + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15]
           + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15]
           - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15]
            + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14]
            - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11]
           - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11]
           + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11]
            - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10]
            + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

    T det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    if (is_zero(det) || std::isinf(det)) {
        return false;
    }
    det = T(1) / det;
    for (std::size_t i = 0; i < 16; ++i) {
        inv_out[i] = inv[i] * det;
    }
    return true;
}

template <typename T>
bool mat_inverse(const T* x, T* y, uint16_t dim) {
    switch (dim) {
        case 3: {
            std::array<T, 9> m{};
            std::array<T, 9> out{};
            for (int i = 0; i < 9; ++i) {
                m[i] = x[i];
            }
            if (!inverse3x3(m, out)) {
                return false;
            }
            for (int i = 0; i < 9; ++i) {
                y[i] = out[i];
            }
            return true;
        }
        case 4: {
            std::array<T, 16> m{};
            std::array<T, 16> out{};
            for (int i = 0; i < 16; ++i) {
                m[i] = x[i];
            }
            if (!inverse4x4(m, out)) {
                return false;
            }
            for (int i = 0; i < 16; ++i) {
                y[i] = out[i];
            }
            return true;
        }
        default:
            // General-N LU path deliberately not ported - see matrix_alg.hpp's
            // file banner (ADR-0012 decision 4: upstream's mat_inverseN
            // heap-allocates, which the flight path may not do).
            return false;
    }
}

template bool inverse3x3<float>(const std::array<float, 9>&, std::array<float, 9>&);
template bool inverse3x3<double>(const std::array<double, 9>&, std::array<double, 9>&);
template bool inverse4x4<float>(const std::array<float, 16>&, std::array<float, 16>&);
template bool inverse4x4<double>(const std::array<double, 16>&, std::array<double, 16>&);
template bool mat_inverse<float>(const float*, float*, uint16_t);
template bool mat_inverse<double>(const double*, double*, uint16_t);

} // namespace fwcpp::math
