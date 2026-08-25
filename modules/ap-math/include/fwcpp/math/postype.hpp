#pragma once

// Port of AP_Math/control.h's postype_t/Vector2p/Vector3p typedefs.
// CPP-011/CPP-007 continuation - unblocks Location::get_vector_from_
// origin_*/get_distance_*_postype and the ekf_offset constructors, all of
// which upstream expresses in terms of this type rather than a fixed
// float/double.
//
// Upstream:
//   #if HAL_WITH_POSTYPE_DOUBLE
//   typedef double postype_t;
//   typedef Vector2d Vector2p;
//   typedef Vector3d Vector3p;
//   #else
//   typedef float postype_t;
//   typedef Vector2f Vector2p;
//   typedef Vector3f Vector3p;
//   #endif
//
// FWCPP_POSTYPE_DOUBLE (set via CMake, see top-level CMakeLists.txt) plays
// the role of HAL_WITH_POSTYPE_DOUBLE - ADR-0012 decision 7's "precision
// is a build option, not a template parameter", already used for
// FWCPP_EKF_DOUBLE, applied to this separate upstream precision switch.
//
// Vector2p/Vector3p are ordinary aliases for this port's own Vector2<T>/
// Vector3<T> template (no new type, unlike upstream's own typedef which is
// likewise just an alias) - nothing upstream-specific needed porting here
// beyond the alias itself.

#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::math {

#if FWCPP_POSTYPE_DOUBLE
using postype_t = double;
using Vector2p = Vector2d;
using Vector3p = Vector3d;
#else
using postype_t = float;
using Vector2p = Vector2f;
using Vector3p = Vector3f;
#endif

} // namespace fwcpp::math
