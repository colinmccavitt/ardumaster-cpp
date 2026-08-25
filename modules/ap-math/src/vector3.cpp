// Vector3<T>::rotate(Rotation) - CPP-019. Compiled under
// fwcpp_upstream_flags: HALF_SQRT_2 and several rotation-specific constants
// (ROLL_90_PITCH_68_YAW_293, PITCH_7) are bare non-exact literals, same
// treatment as scalar.cpp's wrap_* family.
//
// CUSTOM_1/CUSTOM_2 upstream delegate to AP::custom_rotations() (a
// singleton, and AP_CustomRotations isn't ported) - left as a no-op here,
// matching MAX/CUSTOM_OLD/CUSTOM_END's own no-op fallthrough rather than
// inventing behavior for a subsystem this port doesn't have.
//
// Explicitly instantiated for float and double only - matches every other
// caller of this vector (rotation only makes sense for the two floating
// instantiations upstream actually uses it with).

#include <fwcpp/math/vector3.hpp>

namespace fwcpp::math {

namespace {
constexpr double kHalfSqrt2 = 0.70710678118654752440084436210485;
}

template <typename T>
static void rotate_impl(T& x, T& y, T& z, Rotation rotation) {
    const T h = static_cast<T>(kHalfSqrt2);
    T tmp;
    switch (rotation) {
    case Rotation::NONE:
        return;
    case Rotation::YAW_45:
        tmp = h * (x - y);
        y = h * (x + y);
        x = tmp;
        return;
    case Rotation::YAW_90:
        tmp = x; x = -y; y = tmp;
        return;
    case Rotation::YAW_135:
        tmp = -h * (x + y);
        y = h * (x - y);
        x = tmp;
        return;
    case Rotation::YAW_180:
        x = -x; y = -y;
        return;
    case Rotation::YAW_225:
        tmp = h * (y - x);
        y = -h * (x + y);
        x = tmp;
        return;
    case Rotation::YAW_270:
        tmp = x; x = y; y = -tmp;
        return;
    case Rotation::YAW_315:
        tmp = h * (x + y);
        y = h * (y - x);
        x = tmp;
        return;
    case Rotation::ROLL_180:
        y = -y; z = -z;
        return;
    case Rotation::ROLL_180_YAW_45:
        tmp = h * (x + y);
        y = h * (x - y);
        x = tmp; z = -z;
        return;
    case Rotation::ROLL_180_YAW_90:
    case Rotation::PITCH_180_YAW_270:
        tmp = x; x = y; y = tmp; z = -z;
        return;
    case Rotation::ROLL_180_YAW_135:
        tmp = h * (y - x);
        y = h * (y + x);
        x = tmp; z = -z;
        return;
    case Rotation::PITCH_180:
        x = -x; z = -z;
        return;
    case Rotation::ROLL_180_YAW_225:
        tmp = -h * (x + y);
        y = h * (y - x);
        x = tmp; z = -z;
        return;
    case Rotation::ROLL_180_YAW_270:
    case Rotation::PITCH_180_YAW_90:
        tmp = x; x = -y; y = -tmp; z = -z;
        return;
    case Rotation::ROLL_180_YAW_315:
        tmp = h * (x - y);
        y = -h * (x + y);
        x = tmp; z = -z;
        return;
    case Rotation::ROLL_90:
        tmp = z; z = y; y = -tmp;
        return;
    case Rotation::ROLL_90_YAW_45:
        tmp = z; z = y; y = -tmp;
        tmp = h * (x - y);
        y = h * (x + y);
        x = tmp;
        return;
    case Rotation::ROLL_90_YAW_90:
        tmp = z; z = y; y = -tmp;
        tmp = x; x = -y; y = tmp;
        return;
    case Rotation::ROLL_90_YAW_135:
        tmp = z; z = y; y = -tmp;
        tmp = -h * (x + y);
        y = h * (x - y);
        x = tmp;
        return;
    case Rotation::ROLL_270:
        tmp = z; z = -y; y = tmp;
        return;
    case Rotation::ROLL_270_YAW_45:
        tmp = z; z = -y; y = tmp;
        tmp = h * (x - y);
        y = h * (x + y);
        x = tmp;
        return;
    case Rotation::ROLL_270_YAW_90:
        tmp = z; z = -y; y = tmp;
        tmp = x; x = -y; y = tmp;
        return;
    case Rotation::ROLL_270_YAW_135:
        tmp = z; z = -y; y = tmp;
        tmp = -h * (x + y);
        y = h * (x - y);
        x = tmp;
        return;
    case Rotation::PITCH_90:
        tmp = z; z = -x; x = tmp;
        return;
    case Rotation::PITCH_270:
        tmp = z; z = x; x = -tmp;
        return;
    case Rotation::ROLL_90_PITCH_90:
        tmp = z; z = y; y = -tmp;
        tmp = z; z = -x; x = tmp;
        return;
    case Rotation::ROLL_180_PITCH_90:
        y = -y; z = -z;
        tmp = z; z = -x; x = tmp;
        return;
    case Rotation::ROLL_270_PITCH_90:
        tmp = z; z = -y; y = tmp;
        tmp = z; z = -x; x = tmp;
        return;
    case Rotation::ROLL_90_PITCH_180:
        tmp = z; z = y; y = -tmp;
        x = -x; z = -z;
        return;
    case Rotation::ROLL_270_PITCH_180:
        tmp = z; z = -y; y = tmp;
        x = -x; z = -z;
        return;
    case Rotation::ROLL_90_PITCH_270:
        tmp = z; z = y; y = -tmp;
        tmp = z; z = x; x = -tmp;
        return;
    case Rotation::ROLL_180_PITCH_270:
        y = -y; z = -z;
        tmp = z; z = x; x = -tmp;
        return;
    case Rotation::ROLL_270_PITCH_270:
        tmp = z; z = -y; y = tmp;
        tmp = z; z = x; x = -tmp;
        return;
    case Rotation::ROLL_90_PITCH_180_YAW_90:
        tmp = z; z = y; y = -tmp;
        x = -x; z = -z;
        tmp = x; x = -y; y = tmp;
        return;
    case Rotation::ROLL_90_YAW_270:
        tmp = z; z = y; y = -tmp;
        tmp = x; x = y; y = -tmp;
        return;
    case Rotation::ROLL_90_PITCH_68_YAW_293: {
        const T tmpx = x, tmpy = y, tmpz = z;
        x = static_cast<T>(0.14303897231223747232853327204793) * tmpx
          + static_cast<T>(0.36877648650320382639478111741482) * tmpy
          + static_cast<T>(-0.91844638134308709265241077446262) * tmpz;
        y = static_cast<T>(-0.33213277779664740485543461545603) * tmpx
          + static_cast<T>(-0.85628942146641884303193137384369) * tmpy
          + static_cast<T>(-0.39554550256296522325882847326284) * tmpz;
        z = static_cast<T>(-0.93232380121551217122544130688766) * tmpx
          + static_cast<T>(0.36162457008209242248497616856184) * tmpy
          + static_cast<T>(0.00000000000000002214311861220361) * tmpz;
        return;
    }
    case Rotation::PITCH_315:
        tmp = h * (x - z);
        z = h * (x + z);
        x = tmp;
        return;
    case Rotation::ROLL_90_PITCH_315:
        tmp = z; z = y; y = -tmp;
        tmp = h * (x - z);
        z = h * (x + z);
        x = tmp;
        return;
    case Rotation::PITCH_7: {
        const T sin_pitch = static_cast<T>(0.1218693434051474899781908334262);
        const T cos_pitch = static_cast<T>(0.99254615164132198312785249072476);
        const T tmpx = x, tmpz = z;
        x = cos_pitch * tmpx + sin_pitch * tmpz;
        z = -sin_pitch * tmpx + cos_pitch * tmpz;
        return;
    }
    case Rotation::ROLL_45:
        tmp = h * (y - z);
        z = h * (y + z);
        y = tmp;
        return;
    case Rotation::ROLL_315:
        tmp = h * (y + z);
        z = h * (z - y);
        y = tmp;
        return;
    case Rotation::CUSTOM_1:
    case Rotation::CUSTOM_2:
    case Rotation::MAX:
    case Rotation::CUSTOM_OLD:
    case Rotation::CUSTOM_END:
        // See file banner: CUSTOM_1/2 need AP_CustomRotations (not
        // ported); the rest are upstream's own "invalid" sentinels. All
        // left as a no-op rather than the INTERNAL_ERROR upstream reaches
        // for - no InternalError* is threaded through this call today, and
        // "leaves the vector unchanged" is itself a safe, honest default.
        return;
    }
}

template <typename T>
void Vector3<T>::rotate(Rotation rotation) {
    rotate_impl(x, y, z, rotation);
}

template void Vector3<float>::rotate(Rotation);
template void Vector3<double>::rotate(Rotation);

} // namespace fwcpp::math
