add_library(ap-rc-channel INTERFACE)
target_include_directories(ap-rc-channel INTERFACE include)
target_compile_features(ap-rc-channel INTERFACE cxx_std_20)
target_link_libraries(ap-rc-channel INTERFACE ap-math ap-hal)
