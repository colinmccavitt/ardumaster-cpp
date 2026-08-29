add_library(ap-tailsitter INTERFACE)
target_include_directories(ap-tailsitter INTERFACE include)
target_compile_features(ap-tailsitter INTERFACE cxx_std_20)
target_link_libraries(ap-tailsitter INTERFACE ap-math ap-quadplane-transition)
