add_library(ap-q-loiter INTERFACE)
target_include_directories(ap-q-loiter INTERFACE include)
target_compile_features(ap-q-loiter INTERFACE cxx_std_20)
target_link_libraries(ap-q-loiter INTERFACE ap-math ap-q-modes)
