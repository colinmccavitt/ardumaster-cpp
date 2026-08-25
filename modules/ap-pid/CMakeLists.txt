add_library(ap-pid INTERFACE)
target_include_directories(ap-pid INTERFACE include)
target_compile_features(ap-pid INTERFACE cxx_std_20)
target_link_libraries(ap-pid INTERFACE ap-filter ap-internal-error)
