add_library(ap-motors INTERFACE)
target_include_directories(ap-motors INTERFACE include)
target_compile_features(ap-motors INTERFACE cxx_std_20)

# ap-math is PUBLIC: motors_matrix.hpp's public add_motor() overloads call
# math::radians()/is_zero() directly, and both are used inline in this
# header-only module - consumers need ap-math's include path too, same
# shape as ap-compass/ap-gps's own PUBLIC ap-math dependency.
target_link_libraries(ap-motors INTERFACE ap-math)

# ap-filter is PUBLIC (CCP-010): thrust_linearization.hpp's
# ThrustLinearization holds a fwcpp::filter::LowPassFilterFloat member
# (battery-voltage filtering, upstream batt_voltage_filt) and calls it
# inline - consumers of this header-only module need ap-filter's include
# path too, same PUBLIC-dependency shape as ap-math just above.
target_link_libraries(ap-motors INTERFACE ap-filter)

# Not linking fwcpp_upstream_flags: every literal in motors_matrix.hpp/
# thrust_linearization.hpp is already explicitly float-suffixed (0.5f,
# 0.0f, 90.0f), so -fsingle-precision-constant has no observable effect on
# this module's own arithmetic (see motors_matrix.hpp's file banner).
# Matches ap-compass/ap-gps's own precedent of not linking it for
# header-only INTERFACE modules with no ambiguously-typed literals.
