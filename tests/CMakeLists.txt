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
