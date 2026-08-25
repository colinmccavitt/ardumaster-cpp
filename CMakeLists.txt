cmake_minimum_required(VERSION 3.25)

project(plane_fw_cpp
    VERSION 0.1.0
    DESCRIPTION "ArduPlane fixed-wing port to modern C++ (CPP effort). See ADR-0012."
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# ADR-0012 decision 7: precision is a build option, not a template parameter.
option(FWCPP_EKF_DOUBLE "Use double precision for ftype (matches upstream HAL_WITH_EKF_DOUBLE)" OFF)

# ADR-0012 decision 1: match upstream's behavior-affecting compiler flags exactly.
# -fno-exceptions / -fno-rtti are enforced per-target below (test binaries need RTTI-free
# too, but the test *framework* linked in tests/ is exempt - see tests/CMakeLists.txt).
add_compile_options(
    -fno-threadsafe-statics
    -fsingle-precision-constant
    -fno-signed-zeros
    -fno-trapping-math
    -fno-math-errno
    -Wall
    -Wextra
)

enable_testing()

add_subdirectory(modules)
add_subdirectory(tests)
