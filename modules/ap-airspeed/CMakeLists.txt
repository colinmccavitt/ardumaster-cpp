add_library(ap-airspeed INTERFACE)
target_include_directories(ap-airspeed INTERFACE include)
target_compile_features(ap-airspeed INTERFACE cxx_std_20)

# No dependencies: AirspeedSensor's public interface is plain floats/bools
# (no fwcpp::ahrs::*Sample-style shared type the way Compass/Gps expose),
# and its implementation needs only <cmath> - see airspeed_sensor.hpp's
# own file banner.
