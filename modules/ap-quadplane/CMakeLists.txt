add_library(ap-quadplane INTERFACE)
target_include_directories(ap-quadplane INTERFACE include)
target_compile_features(ap-quadplane INTERFACE cxx_std_20)
target_link_libraries(ap-quadplane INTERFACE ap-wpnav ap-math ap-quadplane-transition ap-tailsitter ap-tiltrotor)
