add_library(ap-control INTERFACE)
target_include_directories(ap-control INTERFACE include)
target_compile_features(ap-control INTERFACE cxx_std_20)

# ap-math is PUBLIC: attitude_kinematics.hpp's four functions take/return
# fwcpp::math::Quaternion/Vector3f/Vector2<float> directly in their own
# public signatures and call math::is_zero()/is_positive()/
# constrain_value() inline - consumers of this header-only module need
# ap-math's include path too, same shape as ap-motors's own PUBLIC ap-math
# dependency (see modules/ap-motors/CMakeLists.txt).
target_link_libraries(ap-control INTERFACE ap-math)

# Not linking fwcpp_upstream_flags: every literal in attitude_kinematics.hpp
# is already explicitly float-suffixed (0.1f, 1.0f), so
# -fsingle-precision-constant has no observable effect on this module's own
# arithmetic - matches ap-motors's own motors_matrix.hpp/
# thrust_linearization.hpp precedent of not linking it for header-only
# INTERFACE modules with no ambiguously-typed literals.
