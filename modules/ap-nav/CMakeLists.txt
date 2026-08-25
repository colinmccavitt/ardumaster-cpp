add_library(ap-nav INTERFACE)
target_include_directories(ap-nav INTERFACE include)
target_compile_features(ap-nav INTERFACE cxx_std_20)
target_link_libraries(ap-nav INTERFACE ap-common ap-math)
