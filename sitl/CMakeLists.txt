# CPP-085: this port's first real, standalone runnable executable (not a
# Catch2 test) - a genuine main() driving Plane+SimPlane via SitlHarness
# (CPP-084). Mirrors modules/'s and tests/'s own top-level convention: one
# new directory, wired into the root CMakeLists.txt via add_subdirectory.
#
# ap-hal-sitl already links ap-vehicle and ap-sim publicly (see
# modules/ap-hal-sitl/CMakeLists.txt), so linking it alone is enough for
# main.cpp's own direct use of fwcpp::vehicle::/fwcpp::sim:: types - same
# precedent tests/CMakeLists.txt's own sitl_harness_test target follows.
add_executable(sitl_run main.cpp)
target_link_libraries(sitl_run PRIVATE ap-hal-sitl)

# CCP-044 slice 1: Copter analogue of sitl_run (CPP-085). Same
# ap-hal-sitl link — SitlCopterHarness + LeftoverCopter come through
# ap-hal-sitl's public ap-copter dependency (CCP-043).
add_executable(copter_sitl_run copter_main.cpp)
target_link_libraries(copter_sitl_run PRIVATE ap-hal-sitl)
