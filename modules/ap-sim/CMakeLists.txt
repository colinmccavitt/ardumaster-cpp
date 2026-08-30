add_library(ap-sim INTERFACE)
target_include_directories(ap-sim INTERFACE include)
target_compile_features(ap-sim INTERFACE cxx_std_20)
target_link_libraries(ap-sim INTERFACE ap-math ap-common ap-filter)
