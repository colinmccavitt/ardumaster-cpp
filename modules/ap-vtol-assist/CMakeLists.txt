add_library(ap-vtol-assist INTERFACE)
target_include_directories(ap-vtol-assist INTERFACE include)
target_compile_features(ap-vtol-assist INTERFACE cxx_std_20)
target_link_libraries(ap-vtol-assist INTERFACE ap-math)