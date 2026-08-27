#pragma once

// CPP-043: byte-level (memcpy-based) read/write of a scalar AP_Param
// value at `ptr`, for objects whose fields are plain native C++ types
// (float, bool, an unsigned-8-bit-backed enum class, ...) rather than
// this port's own ParamValue<T>/ParamFloat wrapper classes (param.hpp).
//
// WHY THIS EXISTS, SEPARATE FROM set_value/cast_to_float (CPP-022
// slices 6-7, defaults.hpp/persistence.hpp): those two functions
// reinterpret `ptr` as `ParamInt8*`/`ParamInt16*`/`ParamInt32*`/
// `ParamFloat*` and call that wrapper class's own member functions -
// valid ONLY when an object of that EXACT wrapper type genuinely lives
// at `ptr` (true for NotchFilterParams, CPP-024, whose fields are
// declared as param::ParamFloat). Plane's FixedWingTunables
// (fwcpp/vehicle/plane.hpp) is not like that: its AP_Param-backed
// fields are declared as plain `float`/`bool` (see plane.hpp's own
// "CPP-043 ADDENDUM" banner for why retrofitting FixedWingTunables to
// ParamValue<T> field types was considered and rejected - roughly 50
// already-verified fields, read at hundreds of call sites across
// CPP-031 through CPP-042, for zero behavior change). Reinterpreting a
// `bool`/`float` object as a `ParamInt8`/`ParamFloat*` (a DIFFERENT
// class type) to call its member functions would itself be exactly the
// kind of unsafe reinterpretation ADR-0012 forbids (Float16's bit_cast
// precedent) - even though the two classes happen to share layout on
// every real compiler this port targets, the object at `ptr` is not
// actually of that type.
//
// std::memcpy is the standard-blessed alternative: for a
// TriviallyCopyable object, copying its underlying bytes via memcpy (or
// reading FROM a byte buffer into a local object of the correct size)
// is well-defined regardless of the exact static type on either side,
// as long as the resulting bytes form a valid representation of the
// destination type ([basic.types.general]) - which they do here, since
// every value ever written through set_native_value is itself produced
// by a static_cast to the destination width (int8_t/int16_t/int32_t) or
// is already a float, and this port's target (SITL/Linux/GCC) gives
// `bool` exactly two valid single-byte representations (0/1), which is
// all set_native_value/native_cast_to_float ever produce or consume for
// an Int8-classed `bool` field (AP_Param booleans, upstream and here,
// are never anything other than 0 or 1).
//
// Scope: only the four scalar VarTypes get a case, matching set_value/
// cast_to_float exactly (Vector3f/Group/None have no single scalar to
// read/write here either).

#include <cmath>
#include <cstdint>
#include <cstring>

#include <fwcpp/param/param.hpp> // VarType

namespace fwcpp::param {

// Reads the scalar at `ptr` (known, by the caller, to hold a value of
// `type`, in ITS OWN native C++ representation - not a ParamValue<T>
// wrapper) as a float. NaN for None/Group/Vector3f, matching
// cast_to_float's own fallback.
[[nodiscard]] inline float native_cast_to_float(VarType type, const void* ptr) {
    switch (type) {
        case VarType::Int8: {
            std::int8_t v = 0;
            std::memcpy(&v, ptr, sizeof(v));
            return static_cast<float>(v);
        }
        case VarType::Int16: {
            std::int16_t v = 0;
            std::memcpy(&v, ptr, sizeof(v));
            return static_cast<float>(v);
        }
        case VarType::Int32: {
            std::int32_t v = 0;
            std::memcpy(&v, ptr, sizeof(v));
            return static_cast<float>(v);
        }
        case VarType::Float: {
            float v = 0.0f;
            std::memcpy(&v, ptr, sizeof(v));
            return v;
        }
        default:
            return std::nanf("");
    }
}

// Writes `value` into the scalar at `ptr` (known, by the caller, to
// hold a value of `type` in its own native C++ representation),
// converting as needed - matches set_value's own implicit float->T
// narrowing for the integer cases.
inline void set_native_value(VarType type, void* ptr, float value) {
    switch (type) {
        case VarType::Int8: {
            auto v = static_cast<std::int8_t>(value);
            std::memcpy(ptr, &v, sizeof(v));
            break;
        }
        case VarType::Int16: {
            auto v = static_cast<std::int16_t>(value);
            std::memcpy(ptr, &v, sizeof(v));
            break;
        }
        case VarType::Int32: {
            auto v = static_cast<std::int32_t>(value);
            std::memcpy(ptr, &v, sizeof(v));
            break;
        }
        case VarType::Float:
            std::memcpy(ptr, &value, sizeof(value));
            break;
        default:
            break;
    }
}

} // namespace fwcpp::param
