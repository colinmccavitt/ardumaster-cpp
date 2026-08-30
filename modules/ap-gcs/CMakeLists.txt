# CPP-087 slice 5: GCS_MAVLink wire seam + COMMAND_LONG + PARAM + MISSION + Plane ATTITUDE (INTERFACE, no singleton).
add_library(ap-gcs INTERFACE)
target_include_directories(ap-gcs INTERFACE include)
target_compile_features(ap-gcs INTERFACE cxx_std_20)
target_link_libraries(ap-gcs INTERFACE fwcpp-result)
