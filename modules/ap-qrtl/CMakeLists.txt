add_library(ap-qrtl INTERFACE)
target_include_directories(ap-qrtl INTERFACE include)
target_compile_features(ap-qrtl INTERFACE cxx_std_20)
target_link_libraries(ap-qrtl INTERFACE ap-math)
