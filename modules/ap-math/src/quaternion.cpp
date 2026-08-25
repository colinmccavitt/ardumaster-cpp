// QuaternionT<T>::from_rotation(Rotation) - CPP-019 continuation. Compiled
// under fwcpp_upstream_flags: every constant here is a bare non-exact
// literal (precomputed sin/cos values for 45-degree-increment rotations),
// same treatment as scalar.cpp's wrap_* family and vector3.cpp's
// rotate(Rotation).
//
// CUSTOM_1/CUSTOM_2 upstream delegate to AP::custom_rotations() (a
// singleton, AP_CustomRotations not ported) - left as a no-op (identity
// quaternion), matching MAX/CUSTOM_OLD/CUSTOM_END's own fallthrough rather
// than inventing behavior for an unported subsystem. Same choice
// vector3.cpp's rotate(Rotation) already made for the same enum values.

#include <fwcpp/math/quaternion.hpp>

namespace fwcpp::math {

namespace {
constexpr double kHalfSqrt2 = 0.70710678118654752440084436210485;
constexpr double kHalfSqrt2PlusSqrt2 = 0.92387953251128673848313610506011;
constexpr double kHalfSqrt2MinusSqrt2 = 0.38268343236508972626808144923416;
constexpr double kHalfSqrtHalfTimesTwoPlusSqrtTwo = 0.65328148243818828788676000840496;
constexpr double kHalfSqrtHalfTimesTwoMinusSqrtTwo = 0.27059805007309845059637609665515;
}

template <typename T>
void QuaternionT<T>::from_rotation(Rotation rotation) {
    const T s2 = static_cast<T>(kHalfSqrt2);
    const T s2p = static_cast<T>(kHalfSqrt2PlusSqrt2);
    const T s2m = static_cast<T>(kHalfSqrt2MinusSqrt2);
    const T shp = static_cast<T>(kHalfSqrtHalfTimesTwoPlusSqrtTwo);
    const T shm = static_cast<T>(kHalfSqrtHalfTimesTwoMinusSqrtTwo);

    switch (rotation) {
    case Rotation::NONE:
        q1 = T(1); q2 = q3 = q4 = T(0);
        return;
    case Rotation::YAW_45:
        q1 = s2p; q2 = q3 = T(0); q4 = s2m;
        return;
    case Rotation::YAW_90:
        q1 = s2; q2 = q3 = T(0); q4 = s2;
        return;
    case Rotation::YAW_135:
        q1 = s2m; q2 = q3 = T(0); q4 = s2p;
        return;
    case Rotation::YAW_180:
        q1 = q2 = q3 = T(0); q4 = T(1);
        return;
    case Rotation::YAW_225:
        q1 = -s2m; q2 = q3 = T(0); q4 = s2p;
        return;
    case Rotation::YAW_270:
        q1 = s2; q2 = q3 = T(0); q4 = -s2;
        return;
    case Rotation::YAW_315:
        q1 = s2p; q2 = q3 = T(0); q4 = -s2m;
        return;
    case Rotation::ROLL_180:
        q1 = q3 = q4 = T(0); q2 = T(1);
        return;
    case Rotation::ROLL_180_YAW_45:
        q1 = q4 = T(0); q2 = s2p; q3 = s2m;
        return;
    case Rotation::ROLL_180_YAW_90:
    case Rotation::PITCH_180_YAW_270:
        q1 = q4 = T(0); q2 = q3 = s2;
        return;
    case Rotation::ROLL_180_YAW_135:
        q1 = q4 = T(0); q2 = s2m; q3 = s2p;
        return;
    case Rotation::PITCH_180:
        q1 = q2 = q4 = T(0); q3 = T(1);
        return;
    case Rotation::ROLL_180_YAW_225:
        q1 = q4 = T(0); q2 = -s2m; q3 = s2p;
        return;
    case Rotation::ROLL_180_YAW_270:
    case Rotation::PITCH_180_YAW_90:
        q1 = q4 = T(0); q2 = -s2; q3 = s2;
        return;
    case Rotation::ROLL_180_YAW_315:
        q1 = q4 = T(0); q2 = s2p; q3 = -s2m;
        return;
    case Rotation::ROLL_90:
        q1 = q2 = s2; q3 = q4 = T(0);
        return;
    case Rotation::ROLL_90_YAW_45:
        q1 = shp; q2 = shp; q3 = shm; q4 = shm;
        return;
    case Rotation::ROLL_90_YAW_90:
        q1 = q2 = q3 = q4 = T(0.5);
        return;
    case Rotation::ROLL_90_YAW_135:
        q1 = shm; q2 = shm; q3 = shp; q4 = shp;
        return;
    case Rotation::ROLL_270:
        q1 = s2; q2 = -s2; q3 = q4 = T(0);
        return;
    case Rotation::ROLL_270_YAW_45:
        q1 = shp; q2 = -shp; q3 = -shm; q4 = shm;
        return;
    case Rotation::ROLL_270_YAW_90:
        q1 = q4 = T(0.5); q2 = q3 = T(-0.5);
        return;
    case Rotation::ROLL_270_YAW_135:
        q1 = shm; q2 = -shm; q3 = -shp; q4 = shp;
        return;
    case Rotation::PITCH_90:
        q1 = q3 = s2; q2 = q4 = T(0);
        return;
    case Rotation::PITCH_270:
        q1 = s2; q2 = q4 = T(0); q3 = -s2;
        return;
    case Rotation::ROLL_90_PITCH_90:
        q1 = q2 = q3 = T(-0.5); q4 = T(0.5);
        return;
    case Rotation::ROLL_180_PITCH_90:
        q1 = q3 = T(0); q2 = -s2; q4 = s2;
        return;
    case Rotation::ROLL_270_PITCH_90:
        q1 = q3 = q4 = T(0.5); q2 = T(-0.5);
        return;
    case Rotation::ROLL_90_PITCH_180:
        q1 = q2 = T(0); q3 = -s2; q4 = s2;
        return;
    case Rotation::ROLL_270_PITCH_180:
        q1 = q2 = T(0); q3 = q4 = s2;
        return;
    case Rotation::ROLL_90_PITCH_270:
        q1 = q2 = q4 = T(0.5); q3 = T(-0.5);
        return;
    case Rotation::ROLL_180_PITCH_270:
        q1 = q3 = T(0); q2 = q4 = s2;
        return;
    case Rotation::ROLL_270_PITCH_270:
        q1 = T(-0.5); q2 = q3 = q4 = T(0.5);
        return;
    case Rotation::ROLL_90_PITCH_180_YAW_90:
        q1 = q3 = T(-0.5); q2 = q4 = T(0.5);
        return;
    case Rotation::ROLL_90_YAW_270:
        q1 = q2 = T(-0.5); q3 = q4 = T(0.5);
        return;
    case Rotation::ROLL_90_PITCH_68_YAW_293:
        q1 = static_cast<T>(0.26774500501681575137524760066299);
        q2 = static_cast<T>(0.70698804688952421315661922562867);
        q3 = static_cast<T>(0.012957683254962659713527273197542);
        q4 = static_cast<T>(-0.65445596665363614530264158020145);
        return;
    case Rotation::PITCH_315:
        q1 = s2p; q2 = q4 = T(0); q3 = -s2m;
        return;
    case Rotation::ROLL_90_PITCH_315:
        q1 = shp; q2 = shp; q3 = -shm; q4 = shm;
        return;
    case Rotation::PITCH_7:
        q1 = static_cast<T>(0.99813479842186692003735970502021);
        q2 = q4 = T(0);
        q3 = static_cast<T>(0.061048539534856872956769535676358);
        return;
    case Rotation::ROLL_45:
        q1 = s2p; q2 = s2m; q3 = q4 = T(0);
        return;
    case Rotation::ROLL_315:
        q1 = s2p; q2 = -s2m; q3 = q4 = T(0);
        return;
    case Rotation::CUSTOM_1:
    case Rotation::CUSTOM_2:
    case Rotation::MAX:
    case Rotation::CUSTOM_OLD:
    case Rotation::CUSTOM_END:
        // See file banner: left as a no-op (whatever this quaternion
        // already held - identity, if freshly constructed) rather than
        // upstream's INTERNAL_ERROR. No InternalError* threaded through
        // this call today; a genuine no-op is a safe, honest default.
        return;
    }
}

template void QuaternionT<float>::from_rotation(Rotation);
template void QuaternionT<double>::from_rotation(Rotation);

} // namespace fwcpp::math
