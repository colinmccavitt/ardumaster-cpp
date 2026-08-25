add_library(ap-hal INTERFACE)
target_include_directories(ap-hal INTERFACE include)
target_compile_features(ap-hal INTERFACE cxx_std_20)
target_link_libraries(ap-hal INTERFACE ap-param ap-scheduler)
