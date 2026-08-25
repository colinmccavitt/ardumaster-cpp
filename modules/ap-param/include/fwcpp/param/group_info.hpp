#pragma once

// Port of AP_Param/AP_Param.h's GroupInfo/Info descriptor structs and
// AP_Param.cpp's group_id() token encoding. CPP-022, slice 1. See
// ADR-0013 for the AP_Param sub-effort's overall scoping.
//
// SLICE BOUNDARY: this is the descriptor SHAPE (the structs a var_info
// table is built from) plus group_id (the one small, fully self-
// contained function needed to encode a nested parameter's position
// within its object). Deliberately NOT in this slice: the tree-
// traversal functions built on top of these structs (find_var_info's two
// modes - by name, and by a live object's own pointer identity - see
// upstream's find_var_info_group; they're a materially bigger and more
// intricate piece than this slice, tracked as CPP-022's own remaining
// work, not silently folded in here), get_base/adjust_group_offset,
// load_object_from_eeprom/save (need CPP-020's ParamStorage wired to a
// resolved address, which needs the traversal above), and
// setup_object_defaults.
//
// GroupInfo/Info's union of {group_info pointer, group_info_ptr,
// def_value float, def_value_offset ptrdiff_t} is reproduced as a plain
// union, matching upstream's own tagged-union pattern (which member is
// active is determined by `flags`, exactly as upstream reads it) -
// unlike Result<T,E>'s union, every member here is trivially
// constructible/destructible, so there's no lifetime management to get
// wrong the way Result<T,E>'s did; a plain C-style union is the honest,
// direct equivalent, not a simplification.
//
// LITERAL SAFETY: no bare ambiguous double literals - group_id is pure
// integer arithmetic.

#include <cstddef>
#include <cstdint>

namespace fwcpp::param {

// Matches upstream's AP_PARAM_FLAG_* #defines exactly - values are part
// of var_info table declarations (GROUPINFO macros bake them in), not
// arbitrary.
enum ParamFlag : std::uint16_t {
    kFlagNestedOffset = 1 << 0,
    kFlagPointer = 1 << 1,
    kFlagEnable = 1 << 2,
    kFlagNoShift = 1 << 3,
    kFlagInfoPointer = 1 << 4,
    kFlagInternalUseOnly = 1 << 5,
    kFlagHidden = 1 << 6,
    kFlagDefaultPointer = 1 << 7,
};

// Nesting depth encoding: group_element (ParamHeader's 18-bit field,
// param.hpp) packs up to 3 levels of 6-bit indices - matches upstream's
// own _group_level_shift/_group_bits exactly (18 = 3*6).
inline constexpr std::uint8_t kGroupLevelShift = 6;
inline constexpr std::uint8_t kGroupBits = 18;

struct GroupInfo {
    const char* name;
    std::ptrdiff_t offset; // offset within the containing object
    union {
        const GroupInfo* group_info;                // nested group table (kFlagNestedOffset)
        const GroupInfo* const* group_info_ptr;      // kFlagInfoPointer: dynamic group table
        float def_value;                             // ordinary scalar default
        std::ptrdiff_t def_value_offset;              // kFlagDefaultPointer: offset to a sibling field holding the default
    };
    std::uint16_t flags;
    std::uint8_t idx;  // identifier within the group (0-63, 6 bits)
    std::uint8_t type; // ap_var_type (param.hpp's VarType), stored raw
                        // as upstream does - AP_PARAM_NONE (0) terminates
                        // a var_info table, checked as a raw uint8_t by
                        // the traversal loop, not through VarType's enum
                        // class (which would need a cast at every
                        // comparison site for no benefit here).
};

struct Info {
    const char* name;
    const void* ptr; // pointer to the variable in memory
    union {
        const GroupInfo* group_info;
        const GroupInfo* const* group_info_ptr;
        float def_value;
        std::ptrdiff_t def_value_offset;
    };
    std::uint16_t flags;
    std::uint16_t key; // k_param_* - top-level parameter identifier
    std::uint8_t type;
};

// Encodes group_info[i]'s position at nesting level `shift/kGroupLevelShift`
// into `base`'s group_element accumulator. Reproduces a genuine upstream
// workaround, not a bug of this port's own: idx 0 shifted by a nonzero
// amount is still 0, indistinguishable from "no nesting at this level" -
// upstream's own comment calls this "a bug in the original design" that
// would otherwise let two different parameters alias to the same
// group_element and create load/save loops. The fix (use 63, the max
// 6-bit value, in place of a shifted 0) is upstream's own, reproduced
// exactly - not something to "clean up" per ADR-0007 (fix bugs in the
// PORT, register divergences from upstream; this isn't the port's bug).
[[nodiscard]] inline constexpr std::uint32_t group_id(const GroupInfo* grpinfo, std::uint32_t base, std::uint8_t i, std::uint8_t shift) {
    if (grpinfo[i].idx == 0 && shift != 0 && !(grpinfo[i].flags & kFlagNoShift)) {
        return base + (63U << shift);
    }
    return base + (static_cast<std::uint32_t>(grpinfo[i].idx) << shift);
}

} // namespace fwcpp::param
