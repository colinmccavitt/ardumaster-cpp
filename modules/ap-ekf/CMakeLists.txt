add_library(ap-ekf STATIC
    src/ekf_core.cpp
)
target_include_directories(ap-ekf PUBLIC include)
target_compile_features(ap-ekf PUBLIC cxx_std_20)

# See top-level CMakeLists.txt: FWCPP_EKF_DOUBLE selects ekf_core.hpp's
# ftype (mirrors upstream's HAL_WITH_EKF_DOUBLE, AP_Math/ftype.h) - PUBLIC
# because ekf_core.hpp is a public header and consumers need the same
# definition this module itself was built with (same pattern as ap-math's
# FWCPP_POSTYPE_DOUBLE in modules/ap-math/CMakeLists.txt).
target_compile_definitions(ap-ekf PUBLIC FWCPP_EKF_DOUBLE=$<BOOL:${FWCPP_EKF_DOUBLE}>)

target_link_libraries(ap-ekf PUBLIC ap-math)
