// Vector2<T>::angle(const Vector2&) - the one function in vector2.hpp's
// slice with a bare double literal (M_PI on the antiparallel path).
// Compiled under fwcpp_upstream_flags for the same reason scalar.cpp's
// wrap_* family is - see scalar.hpp's file banner.
//
// Explicitly instantiated for float and double, matching upstream's own
// vector2.cpp instantiation list (there is no int/short instantiation of
// this specific function upstream, unlike wrap_180 - angle() is only ever
// called on floating vectors).

#include <fwcpp/math/vector2.hpp>

namespace fwcpp::math {

static constexpr double kPi = 3.141592653589793238462643383279502884;

template <typename T>
static T vector2_angle(const Vector2<T>& a, const Vector2<T>& b) {
    const T len = a.length() * b.length();
    if (len <= T(0)) {
        return T(0);
    }
    const T cosv = (a * b) / len;
    if (cosv >= T(1)) {
        return T(0);
    }
    if (cosv <= T(-1)) {
        return static_cast<T>(kPi);
    }
    return std::acos(cosv);
}

template <>
float Vector2<float>::angle(const Vector2<float>& v2) const {
    return vector2_angle(*this, v2);
}

template <>
double Vector2<double>::angle(const Vector2<double>& v2) const {
    return vector2_angle(*this, v2);
}

} // namespace fwcpp::math
