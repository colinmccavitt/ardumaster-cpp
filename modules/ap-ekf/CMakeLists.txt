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

# CPP-080: fwcpp/ekf/ekf_backend.hpp (EkfCoreBackend) is a new PUBLIC
# header in this module's own include tree that implements
# fwcpp::ahrs::AhrsBackend and includes ahrs_dcm.hpp for the complete
# AhrsBackend/GyroSample/AccelSample/CompassSample/GpsSample definitions
# (ahrs_backend.hpp itself only forward-declares the sample structs) - any
# consumer of ap-ekf's public headers needs ap-ahrs's include path too.
# ap-ahrs is a header-only INTERFACE library (modules/ap-ahrs/CMakeLists.txt)
# so this adds no new compiled translation unit, only an include path -
# ekf_core.hpp/ekf_core.cpp (this module's pre-existing, unrelated content)
# are completely unaffected and still know nothing about ap-ahrs.
target_link_libraries(ap-ekf PUBLIC ap-ahrs)
