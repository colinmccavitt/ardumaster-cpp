add_library(ap-wpnav INTERFACE)
target_include_directories(ap-wpnav INTERFACE include)
target_compile_features(ap-wpnav INTERFACE cxx_std_20)
target_link_libraries(ap-wpnav INTERFACE ap-math)
