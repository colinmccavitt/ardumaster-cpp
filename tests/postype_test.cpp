// Tests for fwcpp::math::postype_t/Vector2p/Vector3p (AP_Math/control.h).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/postype.hpp>

#include <type_traits>

using namespace fwcpp::math;

TEST_CASE("postype_t/Vector2p/Vector3p resolve to the double-precision variants under FWCPP_POSTYPE_DOUBLE", "[postype]") {
    // This port's CMakeLists.txt defaults FWCPP_POSTYPE_DOUBLE ON to match
    // HAL_WITH_POSTYPE_DOUBLE's real value on this port's target (SITL,
    // which has no HAL_PROGRAM_SIZE_LIMIT_KB constraint) - so this is
    // pinning the actual build configuration, not an arbitrary choice.
#if FWCPP_POSTYPE_DOUBLE
    STATIC_REQUIRE(std::is_same_v<postype_t, double>);
    STATIC_REQUIRE(std::is_same_v<Vector2p, Vector2d>);
    STATIC_REQUIRE(std::is_same_v<Vector3p, Vector3d>);
#else
    STATIC_REQUIRE(std::is_same_v<postype_t, float>);
    STATIC_REQUIRE(std::is_same_v<Vector2p, Vector2f>);
    STATIC_REQUIRE(std::is_same_v<Vector3p, Vector3f>);
#endif
}

TEST_CASE("Vector2p/Vector3p behave as ordinary Vector2<T>/Vector3<T> instances", "[postype]") {
    Vector2p v2(1.0, 2.0);
    Vector3p v3(1.0, 2.0, 3.0);
    REQUIRE(v2.x == static_cast<postype_t>(1.0));
    REQUIRE(v3.z == static_cast<postype_t>(3.0));
}
