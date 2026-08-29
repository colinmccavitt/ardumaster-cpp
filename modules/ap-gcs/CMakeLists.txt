# CPP-087 slice 1: GCS_MAVLink wire seam (INTERFACE, no singleton).
add_library(ap-gcs INTERFACE)
target_include_directories(ap-gcs INTERFACE include)
target_compile_features(ap-gcs INTERFACE cxx_std_20)
target_link_libraries(ap-gcs INTERFACE fwcpp-result)
