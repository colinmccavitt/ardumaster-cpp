add_library(ap-filter STATIC
    src/low_pass_filter_2p.cpp
)
target_include_directories(ap-filter PUBLIC include)
target_compile_features(ap-filter PUBLIC cxx_std_20)
target_link_libraries(ap-filter PUBLIC ap-math ap-param)
target_link_libraries(ap-filter PRIVATE fwcpp_upstream_flags)
