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
target_link_libraries(l1_control_test PRIVATE ap-nav Catch2::Catch2WithMain)
catch_discover_tests(l1_control_test)

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
target_link_libraries(tecs_test PRIVATE ap-tecs Catch2::Catch2WithMain)
catch_discover_tests(tecs_test)

add_executable(fw_control_test fw_control_test.cpp)
target_link_libraries(fw_control_test PRIVATE ap-fw-control Catch2::Catch2WithMain)
catch_discover_tests(fw_control_test)

# ap-sim is a TEST-ONLY dependency here (CPP-031's closed-loop
# integration test) - ap-vehicle itself does not link it, matching how
# ap-sim deliberately shares no code with ap-ahrs (see sim_plane.hpp's
# own file banner).
add_executable(vehicle_test vehicle_test.cpp)
target_link_libraries(vehicle_test PRIVATE ap-vehicle ap-sim Catch2::Catch2WithMain)
catch_discover_tests(vehicle_test)

add_executable(gps_test gps_test.cpp)
target_link_libraries(gps_test PRIVATE ap-gps Catch2::Catch2WithMain)
catch_discover_tests(gps_test)

add_executable(compass_test compass_test.cpp)
target_link_libraries(compass_test PRIVATE ap-compass Catch2::Catch2WithMain)
catch_discover_tests(compass_test)

add_executable(steer_controller_test steer_controller_test.cpp)
target_link_libraries(steer_controller_test PRIVATE ap-steer-control Catch2::Catch2WithMain)
catch_discover_tests(steer_controller_test)
