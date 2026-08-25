add_library(ap-param INTERFACE)
target_include_directories(ap-param INTERFACE include)
target_compile_features(ap-param INTERFACE cxx_std_20)
target_link_libraries(ap-param INTERFACE ap-math)
