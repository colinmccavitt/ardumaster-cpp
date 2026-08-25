add_library(ap-fw-control INTERFACE)
target_include_directories(ap-fw-control INTERFACE include)
target_compile_features(ap-fw-control INTERFACE cxx_std_20)
target_link_libraries(ap-fw-control INTERFACE ap-math ap-pid)
