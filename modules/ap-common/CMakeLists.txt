add_library(ap-common STATIC
    src/location.cpp
)
target_include_directories(ap-common PUBLIC include)
target_compile_features(ap-common PUBLIC cxx_std_20)
target_link_libraries(ap-common PUBLIC ap-internal-error ap-math)
target_link_libraries(ap-common PRIVATE fwcpp_upstream_flags)
