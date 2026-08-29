add_library(ap-copter INTERFACE)
target_include_directories(ap-copter INTERFACE include)
target_compile_features(ap-copter INTERFACE cxx_std_20)
target_link_libraries(ap-copter INTERFACE ap-scheduler)
