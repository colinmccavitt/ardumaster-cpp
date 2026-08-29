add_library(ap-logger INTERFACE)
target_include_directories(ap-logger INTERFACE include)
target_compile_features(ap-logger INTERFACE cxx_std_20)
