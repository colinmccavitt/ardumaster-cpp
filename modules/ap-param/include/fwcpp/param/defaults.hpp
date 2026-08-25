#pragma once

// Port of AP_Param::set_value, AP_Param::get_default_value, and
// AP_Param::setup_object_defaults (AP_Param.cpp). CPP-022, slice 6. See
// ADR-0013 for the AP_Param sub-effort's overall scoping.
//
// setup_object_defaults is deliberately NOT recursive (matching
// upstream exactly - its own comment: "load default values for scalars
// in a group. This does not recurse into subgroups"): it only applies
// defaults to the scalars directly listed in the ONE GroupInfo table
// passed to it. A nested sub-object's own defaults need their own
// separate setup_object_defaults call against that sub-object's own
// table - this is upstream's real, intentional scope for this function,
// not a limitation introduced here.
//
// get_default_value's param_overrides mechanism (a runtime table
// letting board-specific code override a parameter's compiled-in
// default, keyed by object pointer identity) is deliberately NOT
// reproduced - a board-configuration feature layered on top of the core
// default-value lookup, not part of it; every override table upstream
// ships is empty unless a specific board's hwdef opts in, so this is a
// real but currently-inert-for-this-port feature, tracked as follow-on
// work rather than silently dropped from the algorithm's shape.
//
// Vector3f defaults NOT reproduced: this port has no Vector3f-typed
// param wrapper yet (param.hpp only has scalar ParamValue<T>/ParamFloat)
// and AC_PID/AP_Filter - what's actually driving this sub-effort - have
// no Vector3f parameters, so this isn't dropping functionality currently
// needed; setup_object_defaults below silently skips (not crashes on,
// not mishandles) any VarType::Vector3f entry it encounters, which is
// honest given there's no wrapper type to write a default into yet.
//
// reinterpret_cast usage here is the same ordinary, well-defined address
// arithmetic already used in group_info.hpp's get_base/
// adjust_group_offset and name_lookup.hpp's find_group - not the union/
// type-punning kind ADR-0012 forbids. set_value's casts to ParamInt8*/
// ParamFloat*/etc are valid because the resolved address genuinely holds
// an object of that exact type (the caller already matched VarType
// before calling), the same guarantee upstream's own AP_Int8*/AP_Float*
// casts rely on.

#include <cstdint>

#include <fwcpp/param/group_info.hpp>
#include <fwcpp/param/param.hpp>

namespace fwcpp::param {

// Sets the scalar value wrapper at `ptr` (already known, by the caller,
// to hold a value of `type`) to `value`, converting as needed - matches
// upstream's own implicit float->T narrowing at each ParamValue<T>::set
// call (int8_t/int16_t/int32_t truncate toward zero, same as a C-style
// or static_cast would - both are the identical standard conversion).
inline void set_value(VarType type, void* ptr, float value) {
    switch (type) {
        case VarType::Int8:
            static_cast<ParamInt8*>(ptr)->set(value);
            break;
        case VarType::Int16:
            static_cast<ParamInt16*>(ptr)->set(value);
            break;
        case VarType::Int32:
            static_cast<ParamInt32*>(ptr)->set(value);
            break;
        case VarType::Float:
            static_cast<ParamFloat*>(ptr)->set(value);
            break;
        default:
            break;
    }
}

// Resolves a GroupInfo/Info entry's default value for the object at
// `vp`: either a plain compile-time constant (info.def_value), or - if
// kFlagDefaultPointer is set - a float read from a SIBLING field,
// info.def_value_offset bytes before `vp` (used when the "default" is
// itself a shared/runtime value rather than a fixed constant; see
// AP_GROUPINFO_FLAGS_DEFAULT_POINTER in upstream's macros). See file
// banner for why the param_overrides table isn't checked here.
[[nodiscard]] inline float get_default_value(const void* vp, const GroupInfo& info) {
    if (info.flags & kFlagDefaultPointer) {
        return *reinterpret_cast<const float*>(reinterpret_cast<std::ptrdiff_t>(vp) - info.def_value_offset);
    }
    return info.def_value;
}

[[nodiscard]] inline float get_default_value(const void* vp, const Info& info) {
    if (info.flags & kFlagDefaultPointer) {
        return *reinterpret_cast<const float*>(reinterpret_cast<std::ptrdiff_t>(vp) - info.def_value_offset);
    }
    return info.def_value;
}

// Applies every scalar (Int8/Int16/Int32/Float) default in `group_info`
// to the corresponding field of the object at `object_pointer` - NOT
// recursive, see file banner.
inline void setup_object_defaults(void* object_pointer, const GroupInfo* group_info) {
    const auto base = reinterpret_cast<std::ptrdiff_t>(object_pointer);
    for (std::uint8_t i = 0; group_info[i].type != static_cast<std::uint8_t>(VarType::None); ++i) {
        const auto type = static_cast<VarType>(group_info[i].type);
        if (type == VarType::Int8 || type == VarType::Int16 || type == VarType::Int32 || type == VarType::Float) {
            void* ptr = reinterpret_cast<void*>(base + group_info[i].offset);
            set_value(type, ptr, get_default_value(ptr, group_info[i]));
        }
        // VarType::Vector3f: see file banner - deliberately skipped.
    }
}

} // namespace fwcpp::param
