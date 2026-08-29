add_library(ap-tiltrotor INTERFACE)
target_include_directories(ap-tiltrotor INTERFACE include)
target_compile_features(ap-tiltrotor INTERFACE cxx_std_20)
target_link_libraries(ap-tiltrotor INTERFACE ap-math ap-quadplane-transition)
