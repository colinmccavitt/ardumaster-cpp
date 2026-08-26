add_library(ap-steer-control INTERFACE)
target_include_directories(ap-steer-control INTERFACE include)
target_compile_features(ap-steer-control INTERFACE cxx_std_20)
target_link_libraries(ap-steer-control INTERFACE ap-math ap-pid)
