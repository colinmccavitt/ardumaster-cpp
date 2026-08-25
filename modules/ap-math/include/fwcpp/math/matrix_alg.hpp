#pragma once

// Port of AP_Math/matrix_alg.cpp's free functions - mat_mul, mat_identity,
// mat_inverse. CPP-004 continuation.
//
// SCOPE DECISION, worth flagging loudly: upstream's mat_inverse(x, y, dim)
// dispatches to a closed-form inverse3x3/inverse4x4 for those two sizes and
// falls back to a general LU-decomposition mat_inverseN for any other
// dimension. mat_inverseN heap-allocates seven n*n scratch buffers
// (new[]/delete[]) - which ADR-0012 decision 4 forbids anywhere reachable
// from the scheduler's per-loop call graph.
//
// Fixed-wing's own scope (AP_AHRS_DCM's 3x3 rotation matrix; nothing in the
// fixed-wing control stack this port has read so far needs an arbitrary-N
// inverse) only ever needs 3x3. This slice ports mat_mul/mat_identity (which
// take caller-owned storage, no allocation regardless of n) and
// mat_inverse's 3x3/4x4 closed-form paths in full. The general-N path is
// deliberately NOT ported: mat_inverse() returns false for any dimension
// other than 3 or 4, which is a real, honest "unsupported" rather than a
// silent wrong answer. If a later ticket needs arbitrary-N (EKF3, FW-009,
// is the likely candidate), that is the point to decide between a bounded
// stack buffer (cap N at whatever the EKF's actual state size is) and
// accepting heap allocation there specifically with ADR-0012 amended to say
// so explicitly - not a decision to make speculatively here.

#include <array>
#include <cstdint>

namespace fwcpp::math {

// C = A * B, all n x n, row-major, caller-owned storage. No allocation for
// any n - this one upstream function never needed it.
template <typename T>
void mat_mul(const T* a, const T* b, T* c, uint16_t n) {
    for (uint16_t i = 0; i < n * n; ++i) {
        c[i] = T(0);
    }
    for (uint16_t i = 0; i < n; ++i) {
        for (uint16_t j = 0; j < n; ++j) {
            for (uint16_t k = 0; k < n; ++k) {
                c[i * n + j] += a[i * n + k] * b[k * n + j];
            }
        }
    }
}

template <typename T>
void mat_identity(T* a, uint16_t n) {
    for (uint16_t i = 0; i < n * n; ++i) {
        a[i] = T(0);
    }
    for (uint16_t i = 0; i < n; ++i) {
        a[i * n + i] = T(1);
    }
}

// Closed-form 3x3 inverse. Returns false (leaves invOut unspecified) if the
// matrix is singular or the determinant is non-finite - upstream's own
// contract, unchanged.
template <typename T>
bool inverse3x3(const std::array<T, 9>& m, std::array<T, 9>& inv_out);

// Closed-form 4x4 inverse (cofactor expansion, upstream credits the
// gluInvertMatrix OpenGL implementation as its source).
template <typename T>
bool inverse4x4(const std::array<T, 16>& m, std::array<T, 16>& inv_out);

// Dispatch matching upstream's mat_inverse(x, y, dim) call shape (used by
// callers, like AP_AHRS_DCM, that carry dim as a runtime value even though
// it's 3 in practice). See the file banner: only 3 and 4 are supported;
// anything else returns false rather than allocating.
template <typename T>
[[nodiscard]] bool mat_inverse(const T* x, T* y, uint16_t dim);

} // namespace fwcpp::math
