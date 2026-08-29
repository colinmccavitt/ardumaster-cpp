# Modules land here as they are ported, one directory per module, mirroring
# fw-rust's crates/ layout (module names deliberately parallel: ap-math here
# is the same upstream scope as ap-math in ports/plane-fw-rust/crates/).

add_subdirectory(fwcpp-result)
add_subdirectory(ap-internal-error)
add_subdirectory(ap-math)
add_subdirectory(ap-common)
add_subdirectory(ap-filter)
add_subdirectory(ap-pid)
add_subdirectory(ap-nav)
add_subdirectory(ap-param)
add_subdirectory(ap-scheduler)
add_subdirectory(ap-hal)
add_subdirectory(ap-rc-channel)
add_subdirectory(ap-srv-channel)
add_subdirectory(ap-ahrs)
add_subdirectory(ap-compass)
add_subdirectory(ap-gps)
add_subdirectory(ap-airspeed)
add_subdirectory(ap-sim)
add_subdirectory(ap-tecs)
add_subdirectory(ap-fw-control)
add_subdirectory(ap-steer-control)
add_subdirectory(ap-vehicle)
add_subdirectory(ap-ekf)
add_subdirectory(ap-hal-sitl)

# copter-cpp effort (see /srv/ardumaster/tracker/efforts/copter-cpp.md):
# ArduCopter-specific modules, reusing the shared modules above unchanged.
# ap-motors (CCP-001): AP_MotorsMatrix core factor storage/arithmetic.
add_subdirectory(ap-motors)

# ap-control (CCP-018): AC_AttitudeControl frame-conversion/limit
# kinematics - the first ticket of this effort's own AC_AttitudeControl
# phase, following AP_Motors's completion (CCP-001 through CCP-017).
add_subdirectory(ap-control)

# ap-poscontrol (CCP-027): AC_PosControl port — slice 1 lean-angle helpers.
add_subdirectory(ap-poscontrol)

# ap-wpnav (CCP-028): AC_WPNav, AC_Loiter, AC_Circle.
add_subdirectory(ap-wpnav)

# ap-quadplane (VCP-001): QuadPlane slice 1 scaffold.
add_subdirectory(ap-quadplane)

# ap-vtol-assist (VCP-002): VTOL_Assist slice 1 scaffold.
add_subdirectory(ap-vtol-assist)
