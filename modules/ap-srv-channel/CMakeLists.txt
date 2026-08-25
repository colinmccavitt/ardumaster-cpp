add_library(ap-srv-channel INTERFACE)
target_include_directories(ap-srv-channel INTERFACE include)
target_compile_features(ap-srv-channel INTERFACE cxx_std_20)
target_link_libraries(ap-srv-channel INTERFACE ap-math ap-hal)
