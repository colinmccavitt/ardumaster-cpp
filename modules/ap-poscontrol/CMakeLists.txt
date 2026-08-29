add_library(ap-poscontrol INTERFACE)
target_include_directories(ap-poscontrol INTERFACE include)
target_compile_features(ap-poscontrol INTERFACE cxx_std_20)

# ap-math is PUBLIC: pos_control_lean.hpp uses fwcpp::math::Vector3f,
# radians(), is_positive(), and local tan/atan lean conversions matching
# upstream AP_Math/control.cpp angle/accel helpers (not yet in control.hpp).
target_link_libraries(ap-poscontrol INTERFACE ap-math ap-pid)

# Not linking fwcpp_upstream_flags: every literal in this slice is explicitly
# float-suffixed, matching ap-control/ap-motors header-only INTERFACE precedent.