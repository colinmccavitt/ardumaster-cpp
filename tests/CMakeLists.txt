# Parity/unit tests land here as modules are ported, one target per module
# under test - mirrors fw-rust's per-crate tests/ directories.
#
# Framework: Catch2 3.x (system package `catch2`, found via find_package
# below - no FetchContent, no network access needed to configure or build).
# Chosen over doctest/GoogleTest for BDD-style SECTION blocks, which suit the
# scenario-table fixture format tools/parity already emits for fw-rust and
# that this effort intends to read directly where a fixture is
# upstream-behavior data rather than Rust- or C++-specific (see the fw-cpp
# effort charter). Amendable per-ticket if a module's test shape argues
# otherwise; record the reason in that ticket's notes if so.

find_package(Catch2 3 REQUIRED)
include(CTest)
include(Catch)

add_executable(result_test result_test.cpp)
target_link_libraries(result_test PRIVATE fwcpp-result Catch2::Catch2WithMain)
catch_discover_tests(result_test)

add_executable(internal_error_test internal_error_test.cpp)
target_link_libraries(internal_error_test PRIVATE ap-internal-error Catch2::Catch2WithMain)
catch_discover_tests(internal_error_test)

add_executable(scalar_test scalar_test.cpp)
target_link_libraries(scalar_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(scalar_test)

add_executable(matrix_alg_test matrix_alg_test.cpp)
target_link_libraries(matrix_alg_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(matrix_alg_test)

add_executable(vector2_test vector2_test.cpp)
target_link_libraries(vector2_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(vector2_test)

add_executable(vector3_test vector3_test.cpp)
target_link_libraries(vector3_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(vector3_test)

add_executable(matrix3_test matrix3_test.cpp)
target_link_libraries(matrix3_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(matrix3_test)

add_executable(quaternion_test quaternion_test.cpp)
target_link_libraries(quaternion_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(quaternion_test)

add_executable(bitmask_test bitmask_test.cpp)
target_link_libraries(bitmask_test PRIVATE ap-common Catch2::Catch2WithMain)
catch_discover_tests(bitmask_test)

add_executable(location_test location_test.cpp)
target_link_libraries(location_test PRIVATE ap-common Catch2::Catch2WithMain)
catch_discover_tests(location_test)

add_executable(float16_test float16_test.cpp)
target_link_libraries(float16_test PRIVATE ap-common Catch2::Catch2WithMain)
catch_discover_tests(float16_test)

add_executable(low_pass_filter_2p_test low_pass_filter_2p_test.cpp)
target_link_libraries(low_pass_filter_2p_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(low_pass_filter_2p_test)

add_executable(low_pass_filter_test low_pass_filter_test.cpp)
target_link_libraries(low_pass_filter_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(low_pass_filter_test)

add_executable(slew_limiter_test slew_limiter_test.cpp)
target_link_libraries(slew_limiter_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(slew_limiter_test)

add_executable(ac_pid_test ac_pid_test.cpp)
target_link_libraries(ac_pid_test PRIVATE ap-pid Catch2::Catch2WithMain)
catch_discover_tests(ac_pid_test)

add_executable(l1_control_test l1_control_test.cpp)
# CPP-048: l1_control.hpp now also includes fwcpp/param/* headers (the
# new top-level AP_Param Info[] table for L1Control::Gains) - ap-nav's
# own CMakeLists.txt does not link ap-param (out of this ticket's touch
# scope: only l1_control.hpp, its own new test file, and this file are
# allowed), so ap-param is linked directly here instead, for this
# pre-existing test target to keep resolving those headers.
target_link_libraries(l1_control_test PRIVATE ap-nav ap-param Catch2::Catch2WithMain)
catch_discover_tests(l1_control_test)

# CPP-048: storage/persistence integration test for L1Control::Gains's
# real top-level AP_Param Info[] table - no ap-sim needed (not a
# closed-loop test, see the ticket's own "NOT a SimPlane closed-loop
# test" instruction, matching CPP-043's own plane_aparm_param_test).
add_executable(l1_control_param_test l1_control_param_test.cpp)
target_link_libraries(l1_control_param_test PRIVATE ap-nav ap-param Catch2::Catch2WithMain)
catch_discover_tests(l1_control_param_test)

add_executable(notch_filter_test notch_filter_test.cpp)
target_link_libraries(notch_filter_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(notch_filter_test)

add_executable(vector2_geometry_test vector2_geometry_test.cpp)
target_link_libraries(vector2_geometry_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(vector2_geometry_test)

add_executable(rotations_test rotations_test.cpp)
target_link_libraries(rotations_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(rotations_test)

add_executable(quaternion_rotation_test quaternion_rotation_test.cpp)
target_link_libraries(quaternion_rotation_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(quaternion_rotation_test)

add_executable(vector3_matops_test vector3_matops_test.cpp)
target_link_libraries(vector3_matops_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(vector3_matops_test)

add_executable(postype_test postype_test.cpp)
target_link_libraries(postype_test PRIVATE ap-math Catch2::Catch2WithMain)
catch_discover_tests(postype_test)

add_executable(storage_test storage_test.cpp)
target_link_libraries(storage_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(storage_test)

add_executable(param_test param_test.cpp)
target_link_libraries(param_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(param_test)

add_executable(group_info_test group_info_test.cpp)
target_link_libraries(group_info_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(group_info_test)

add_executable(name_lookup_test name_lookup_test.cpp)
target_link_libraries(name_lookup_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(name_lookup_test)

add_executable(persistence_test persistence_test.cpp)
target_link_libraries(persistence_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(persistence_test)

add_executable(defaults_test defaults_test.cpp)
target_link_libraries(defaults_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(defaults_test)

add_executable(top_level_test top_level_test.cpp)
target_link_libraries(top_level_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(top_level_test)

add_executable(native_value_test native_value_test.cpp)
target_link_libraries(native_value_test PRIVATE ap-param Catch2::Catch2WithMain)
catch_discover_tests(native_value_test)

add_executable(notch_filter_params_test notch_filter_params_test.cpp)
target_link_libraries(notch_filter_params_test PRIVATE ap-filter ap-param Catch2::Catch2WithMain)
catch_discover_tests(notch_filter_params_test)

add_executable(filter_registry_test filter_registry_test.cpp)
target_link_libraries(filter_registry_test PRIVATE ap-filter Catch2::Catch2WithMain)
catch_discover_tests(filter_registry_test)

add_executable(scheduler_test scheduler_test.cpp)
target_link_libraries(scheduler_test PRIVATE ap-scheduler Catch2::Catch2WithMain)
catch_discover_tests(scheduler_test)

add_executable(rc_test rc_test.cpp)
target_link_libraries(rc_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(rc_test)

add_executable(rc_channel_test rc_channel_test.cpp)
target_link_libraries(rc_channel_test PRIVATE ap-rc-channel Catch2::Catch2WithMain)
catch_discover_tests(rc_channel_test)

add_executable(uart_driver_test uart_driver_test.cpp)
target_link_libraries(uart_driver_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(uart_driver_test)

add_executable(analog_in_test analog_in_test.cpp)
target_link_libraries(analog_in_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(analog_in_test)

add_executable(srv_channel_test srv_channel_test.cpp)
target_link_libraries(srv_channel_test PRIVATE ap-srv-channel Catch2::Catch2WithMain)
catch_discover_tests(srv_channel_test)

add_executable(hal_context_test hal_context_test.cpp)
target_link_libraries(hal_context_test PRIVATE ap-hal Catch2::Catch2WithMain)
catch_discover_tests(hal_context_test)

add_executable(rc_channels_test rc_channels_test.cpp)
target_link_libraries(rc_channels_test PRIVATE ap-rc-channel Catch2::Catch2WithMain)
catch_discover_tests(rc_channels_test)

add_executable(srv_channels_test srv_channels_test.cpp)
target_link_libraries(srv_channels_test PRIVATE ap-srv-channel Catch2::Catch2WithMain)
catch_discover_tests(srv_channels_test)

add_executable(ahrs_dcm_test ahrs_dcm_test.cpp)
target_link_libraries(ahrs_dcm_test PRIVATE ap-ahrs Catch2::Catch2WithMain)
catch_discover_tests(ahrs_dcm_test)

add_executable(sim_plane_test sim_plane_test.cpp)
target_link_libraries(sim_plane_test PRIVATE ap-sim Catch2::Catch2WithMain)
catch_discover_tests(sim_plane_test)

add_executable(tecs_test tecs_test.cpp)
# ap-param added by CPP-049: tecs.hpp itself now #includes several
# fwcpp/param/ headers for its own "CPP-049 ADDENDUM" (tecs_group_info/
# tecs_param_info/apply_tecs_defaults/load_tecs_parameters/
# save_tecs_parameters) - any translation unit that includes tecs.hpp,
# including this pre-existing test, needs ap-param's include path too.
# modules/ap-tecs/CMakeLists.txt itself is intentionally left untouched
# (out of this ticket's scope - see tecs_param_test's own target below
# for the fuller note); adding the dependency here, at every consuming
# test target, is the fix available within this ticket's touched files.
target_link_libraries(tecs_test PRIVATE ap-tecs ap-param Catch2::Catch2WithMain)
catch_discover_tests(tecs_test)

# CPP-049: storage/persistence integration test for Tecs::Gains (TECS_
# prefix) - no ap-sim needed (not a closed-loop test, see the ticket's
# own "NOT a SimPlane closed-loop test" instruction). Links ap-param
# directly (in addition to ap-tecs) because tecs_param_test.cpp itself,
# not just tecs.hpp, uses fwcpp::param:: types (GroupInfo, ParamHeader,
# storage::StorageAccess, top_level::find) directly - ap-tecs's own
# CMakeLists.txt is intentionally left untouched by this ticket (see the
# ticket's own scope note restricting changes to tecs.hpp/this test
# file/this CMakeLists.txt only), so this test target picks up
# fwcpp/param/'s include path via its own direct ap-param link instead.
add_executable(tecs_param_test tecs_param_test.cpp)
target_link_libraries(tecs_param_test PRIVATE ap-tecs ap-param Catch2::Catch2WithMain)
catch_discover_tests(tecs_param_test)

add_executable(fw_control_test fw_control_test.cpp)
# ap-param: CPP-044 added real AP_Param Info-table support directly to
# roll_controller.hpp (included here), which now unconditionally
# #includes fwcpp/param/* headers - ap-fw-control itself doesn't link
# ap-param (see roll_controller_param_test's own link line below for why
# that dependency isn't added to the module's own CMakeLists), so this
# test target needs the include path directly.
target_link_libraries(fw_control_test PRIVATE ap-fw-control ap-param Catch2::Catch2WithMain)
catch_discover_tests(fw_control_test)

# ap-sim is a TEST-ONLY dependency here (CPP-031's closed-loop
# integration test) - ap-vehicle itself does not link it, matching how
# ap-sim deliberately shares no code with ap-ahrs (see sim_plane.hpp's
# own file banner).
add_executable(vehicle_test vehicle_test.cpp)
target_link_libraries(vehicle_test PRIVATE ap-vehicle ap-sim Catch2::Catch2WithMain)
catch_discover_tests(vehicle_test)

# CPP-043: storage/persistence integration test for Plane::aparm - no
# ap-sim needed (not a closed-loop test, see the ticket's own "NOT a
# SimPlane closed-loop test" instruction).
add_executable(plane_aparm_param_test plane_aparm_param_test.cpp)
target_link_libraries(plane_aparm_param_test PRIVATE ap-vehicle Catch2::Catch2WithMain)
catch_discover_tests(plane_aparm_param_test)

# CPP-044: storage/persistence integration test for
# fw_control::RollController::Gains - no ap-sim needed (not a closed-loop
# test, matching CPP-043's own plane_aparm_param_test above). Links
# ap-param directly (in addition to ap-fw-control) for the same reason
# fw_control_test does above.
add_executable(roll_controller_param_test roll_controller_param_test.cpp)
target_link_libraries(roll_controller_param_test PRIVATE ap-fw-control ap-param Catch2::Catch2WithMain)
catch_discover_tests(roll_controller_param_test)

add_executable(gps_test gps_test.cpp)
target_link_libraries(gps_test PRIVATE ap-gps Catch2::Catch2WithMain)
catch_discover_tests(gps_test)

add_executable(compass_test compass_test.cpp)
target_link_libraries(compass_test PRIVATE ap-compass Catch2::Catch2WithMain)
catch_discover_tests(compass_test)

add_executable(steer_controller_test steer_controller_test.cpp)
target_link_libraries(steer_controller_test PRIVATE ap-steer-control Catch2::Catch2WithMain)
catch_discover_tests(steer_controller_test)

# CPP-045: storage/persistence integration test for
# PitchController::Gains - no ap-sim needed (not a closed-loop test, see
# the ticket's own "NOT a SimPlane closed-loop test" instruction).
# ap-fw-control's own transitive link chain (ap-pid -> ap-filter -> a
# PUBLIC ap-param dependency) already brings in ap-param's include path,
# so no module CMakeLists.txt needs editing for this ticket.
add_executable(pitch_controller_param_test pitch_controller_param_test.cpp)
target_link_libraries(pitch_controller_param_test PRIVATE ap-fw-control Catch2::Catch2WithMain)
catch_discover_tests(pitch_controller_param_test)

# CPP-046: storage/persistence integration test for
# fw_control::YawController::Gains - no ap-sim/ap-vehicle needed (not a
# closed-loop test, see the ticket's own "NOT a SimPlane closed-loop
# test" instruction); ap-fw-control alone (which transitively pulls in
# ap-param via ap-pid -> ap-filter -> ap-param) is enough.
add_executable(yaw_controller_param_test yaw_controller_param_test.cpp)
target_link_libraries(yaw_controller_param_test PRIVATE ap-fw-control Catch2::Catch2WithMain)
catch_discover_tests(yaw_controller_param_test)

# CPP-047: storage/persistence integration test for
# steer_control::SteerController::Gains - no ap-sim needed (not a
# closed-loop test, matching CPP-043's own plane_aparm_param_test
# precedent above). Links ap-param directly (in addition to
# ap-steer-control, whose own module CMakeLists.txt does not depend on
# ap-param) for the same reason notch_filter_params_test does above.
add_executable(steer_controller_param_test steer_controller_param_test.cpp)
target_link_libraries(steer_controller_param_test PRIVATE ap-steer-control ap-param Catch2::Catch2WithMain)
catch_discover_tests(steer_controller_param_test)

# CPP-052 phase 1: strapdown INS mechanization + covariance prediction
# unit tests, no fusion (see fwcpp/ekf/ekf_core.hpp's scope banner).
add_executable(ekf_core_test ekf_core_test.cpp)
target_link_libraries(ekf_core_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_core_test)

# CPP-056 phase 2: fuse_direct_state_observation() primitive + GPS
# velocity/position fusion unit tests (see fwcpp/ekf/ekf_core.hpp's
# "CPP-056, PHASE 2" scope banner section).
add_executable(ekf_fusion_test ekf_fusion_test.cpp)
target_link_libraries(ekf_fusion_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_fusion_test)

# CPP-059 phase 5: 3-axis magnetometer fusion (fuse_magnetometer()) unit
# tests (see fwcpp/ekf/ekf_core.hpp's "CPP-059, PHASE 5" scope banner
# section).
add_executable(ekf_mag_fusion_test ekf_mag_fusion_test.cpp)
target_link_libraries(ekf_mag_fusion_test PRIVATE ap-ekf Catch2::Catch2WithMain)
catch_discover_tests(ekf_mag_fusion_test)

# CPP-061 phase 7: closed-loop EkfCore validation against SimPlane's
# ground truth (see fwcpp/ekf/ekf_core.hpp's phase banners for phases
# 1-6 - this is the first test to run the WHOLE assembled pipeline
# together, continuously, against a realistic simulated flight). ap-sim,
# ap-compass, and ap-vehicle are TEST-ONLY dependencies here (same
# precedent as vehicle_test.cpp's own ap-sim dependency) - ap-ekf itself
# links none of them.
add_executable(ekf_closed_loop_test ekf_closed_loop_test.cpp)
target_link_libraries(ekf_closed_loop_test PRIVATE ap-ekf ap-sim ap-compass ap-vehicle Catch2::Catch2WithMain)
catch_discover_tests(ekf_closed_loop_test)
