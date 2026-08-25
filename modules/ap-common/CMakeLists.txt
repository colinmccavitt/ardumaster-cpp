add_library(ap-common INTERFACE)
target_include_directories(ap-common INTERFACE include)
target_compile_features(ap-common INTERFACE cxx_std_20)
target_link_libraries(ap-common INTERFACE ap-internal-error ap-math)
