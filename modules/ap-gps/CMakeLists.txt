add_library(ap-gps INTERFACE)
target_include_directories(ap-gps INTERFACE include)
target_compile_features(ap-gps INTERFACE cxx_std_20)

# ap-ahrs is PUBLIC: gps.hpp's own public interface (Gps::sample()) returns
# fwcpp::ahrs::GpsSample by const reference, so consumers need ap-ahrs's
# include path too. ap-math comes in transitively through ap-ahrs, but is
# also depended on directly since gps.hpp uses math::Vector3f/wrap_360/
# degrees() itself.
target_link_libraries(ap-gps INTERFACE ap-math ap-ahrs)
