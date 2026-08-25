add_library(ap-ahrs INTERFACE)
target_include_directories(ap-ahrs INTERFACE include)
target_compile_features(ap-ahrs INTERFACE cxx_std_20)
target_link_libraries(ap-ahrs INTERFACE ap-math)
