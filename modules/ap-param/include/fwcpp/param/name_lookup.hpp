#pragma once

// Port of AP_Param::find_group (AP_Param.cpp) - the recursive by-name
// search through a GroupInfo tree. CPP-022, slice 3. See ADR-0013 and
// group_info.hpp's own banner for the AP_Param sub-effort's scoping.
//
// RETURN TYPE: upstream returns `AP_Param *` - a pointer to the live
// object at the resolved address, reinterpreted through AP_Param's role
// as a common (non-virtual) base class purely so generic tree-walking
// code can pass an address around before the caller casts it to the
// concrete type `*ptype` says it actually is. This port's value types
// (param.hpp's ParamValue<T>/ParamFloat) don't share such a base -
// nothing in this slice's scope needs one yet (that only matters for
// find_var_info's BY-POINTER-IDENTITY mode - a live object discovering
// its own address - which is separate, larger, deferred work). Returning
// `void*` + a VarType out-param here is the honest equivalent of what
// upstream's AP_Param* actually buys a caller in this by-name mode: an
// address, plus what's really there, with the caller responsible for
// casting to the matching concrete type - exactly upstream's own
// contract (`AP_Float *v = (AP_Float *)ap;`), just without dressing a
// void* up as false type safety.
//
// SLICE BOUNDARY: covers the ordinary-scalar and nested-group cases.
// Deliberately NOT in this slice: the Vector3f `_X`/`_Y`/`_Z` suffix
// special case (upstream's find_group has a third branch matching a
// name like "FOO_X" against a Vector3f field named "FOO" and returning
// a pointer to the individual float) - AP_Filter/AC_PID, the actual
// consumers driving this sub-effort, have no Vector3f parameters, so
// this isn't silently dropped functionality this port currently needs;
// tracked in CPP-022's notes as real remaining scope. Also deliberately
// NOT in this slice: AP_Param::find (the top-level Info-table walk this
// wraps) - upstream's version needs a real vehicle-wide Info table this
// port doesn't have yet (see ADR-0013's scope boundary); find_group
// itself takes its starting GroupInfo table as an explicit parameter, so
// it's independently testable and usable right now for exactly the
// nested-object case (AC_PID inside a parent, AP_Filter's var_info)
// this sub-effort actually needs.
//
// CASE SENSITIVITY, preserved not "fixed": nested-group name matching
// here is case-INsensitive (strncasecmp/strcasecmp) - upstream's own
// top-level AP_Param::find, by contrast, uses case-SENSITIVE strncmp for
// its equivalent group-prefix check. That's a genuine inconsistency
// between the two levels in upstream itself, not a transcription choice
// made here - reproduced as found, not smoothed over, since "fix bugs"
// (ADR-0007) means bugs in the PORT, not upstream's own already-shipped
// behavior.

#include <cstdint>
#include <cstring>
#include <strings.h> // POSIX strcasecmp/strncasecmp - this port targets SITL/Linux, same as upstream's own SITL build

#include <fwcpp/param/group_info.hpp>
#include <fwcpp/param/param.hpp> // VarType

namespace fwcpp::param {

// Recursively searches `group_info` (and any nested groups within it)
// for `name`, starting from `base + group_offset`. Returns nullptr if
// not found. On success, sets `ptype` to the VarType found and returns
// the resolved address as `void*` - see file banner for why not a typed
// pointer.
[[nodiscard]] inline void* find_group(const char* name, std::ptrdiff_t base, std::ptrdiff_t group_offset, const GroupInfo* group_info, VarType& ptype) {
    for (std::uint8_t i = 0; group_info[i].type != static_cast<std::uint8_t>(VarType::None); ++i) {
        const std::uint8_t type = group_info[i].type;
        if (type == static_cast<std::uint8_t>(VarType::Group)) {
            const std::size_t prefix_len = std::strlen(group_info[i].name);
            if (strncasecmp(name, group_info[i].name, prefix_len) != 0) {
                continue;
            }
            // kFlagInfoPointer (dynamically-defined group tables, used by
            // AP_Scripting) is not reproduced here - group_info is always
            // read directly, matching the common (non-scripting) case.
            const GroupInfo* ginfo = group_info[i].group_info;
            if (ginfo == nullptr) {
                continue;
            }
            std::ptrdiff_t new_offset = group_offset;
            if (!adjust_group_offset(base, group_info[i], new_offset)) {
                continue;
            }
            void* found = find_group(name + prefix_len, base, new_offset, ginfo, ptype);
            if (found != nullptr) {
                return found;
            }
        } else if (strcasecmp(name, group_info[i].name) == 0) {
            ptype = static_cast<VarType>(type);
            return reinterpret_cast<void*>(base + group_info[i].offset + group_offset);
        }
        // Vector3f suffix case (name == "FOO_X"/"FOO_Y"/"FOO_Z" against a
        // Vector3f field named "FOO") deliberately not handled - see file
        // banner.
    }
    return nullptr;
}

} // namespace fwcpp::param
