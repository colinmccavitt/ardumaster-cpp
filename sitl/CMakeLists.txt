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
