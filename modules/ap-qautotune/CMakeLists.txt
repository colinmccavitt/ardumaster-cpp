add_library(ap-qautotune INTERFACE)
target_include_directories(ap-qautotune INTERFACE include)
target_compile_features(ap-qautotune INTERFACE cxx_std_20)
target_link_libraries(ap-qautotune INTERFACE ap-math ap-q-modes ap-q-loiter)
