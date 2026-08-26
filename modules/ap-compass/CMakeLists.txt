add_library(ap-compass INTERFACE)
target_include_directories(ap-compass INTERFACE include)
target_compile_features(ap-compass INTERFACE cxx_std_20)

# ap-ahrs is PUBLIC: compass.hpp's own public interface (Compass::sample())
# returns fwcpp::ahrs::CompassSample by const reference, so consumers need
# ap-ahrs's include path too - same shape as ap-gps's own dependency on
# ap-ahrs for GpsSample. ap-math comes in transitively through ap-ahrs, but
# is also depended on directly since compass.hpp uses math::Vector3f/
# Matrix3f itself.
target_link_libraries(ap-compass INTERFACE ap-math ap-ahrs)
