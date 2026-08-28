add_library(ap-tecs INTERFACE)
target_include_directories(ap-tecs INTERFACE include)
target_compile_features(ap-tecs INTERFACE cxx_std_20)
target_link_libraries(ap-tecs INTERFACE ap-math ap-ahrs ap-filter)
