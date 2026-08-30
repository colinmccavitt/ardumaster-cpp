#pragma once

// CPP-023 slice 1–5: AP_Param ConversionInfo + convert_old_parameters_scaled
// leftover scaffold (Plane-4.7.0 AP_Param.cpp ~2125-2160, convert_old_parameter
// ~2068-2121), _convert_parameter_width leftover (~2222+), convert_class
// leftover (~2143-2193), and convert_g2 / convert_toplevel thin leftovers
// (~2197-2218). ADR-0012: no EEPROM / no AP_Param singleton — injected
// OldParamStore / NewParamStore and WidthConvertInputs stand in for storage
// scan, find_var_info, and save.
//
// Slice 1: table loop, inject lookup, scaler apply into NewParamStore by
// new_name. Slice 2: leftover_convert_parameter_width (configured skip, inject
// old value, scale or bitmask stub). Slice 3: CONVERT_FLAG_REVERSE / FORCE on
// convert_old_parameter (inject new_configured). Slice 4: leftover_convert_class
// (old class key → ClassConversionInfo table → convert_old_parameter).
// Slice 5 (close): leftover_convert_g2 / leftover_convert_toplevel (loop →
// leftover_convert_class); thin centi/bitmask width wrappers; EEPROM scan +
// flush catalogued kOutOfScope.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <fwcpp/math/scalar.hpp> // math::is_equal

namespace fwcpp::param {

// Upstream AP_Param::ConversionInfo (AP_Param.h ~232-237). Slice shape uses
// uint8_t old_group_element / type (ticket); upstream old_group_element is
// uint32_t and type is ap_var_type. Widen in a later slice if vehicle tables
// need the full 18-bit group_element packing.
struct ConversionInfo {
    std::uint16_t old_key = 0;
    std::uint8_t old_group_element = 0;
    std::uint8_t type = 0; // VarType as uint8_t
    const char* new_name = nullptr;
};

// Upstream CONVERT_FLAG_* (AP_Param.h ~514-517).
inline constexpr std::uint8_t kConvertFlagReverse = 1; // _REV → _REVERSED
inline constexpr std::uint8_t kConvertFlagForce = 2;   // ignore configured_in_storage

inline constexpr std::size_t kConversionStoreCap = 16;

struct OldParamEntry {
    std::uint16_t old_key = 0;
    std::uint8_t old_group_element = 0;
    std::uint8_t type = 0;
    float value = 0.0f;
    bool present = false;
};

// Injected stand-in for EEPROM contents find_old_parameter would scan.
struct OldParamStore {
    OldParamEntry entries[kConversionStoreCap]{};
    std::size_t count = 0;
};

struct NewParamEntry {
    char name[17]{};
    float value = 0.0f;
    bool present = false;
};

// Injected stand-in for find(new_name) + set_value + save.
struct NewParamStore {
    NewParamEntry entries[kConversionStoreCap]{};
    std::size_t count = 0;
};

[[nodiscard]] inline bool conversion_name_eq(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return a == b;
    }
    return std::strcmp(a, b) == 0;
}

// Inject old scalar (float-cast form). Returns false if store is full.
[[nodiscard]] inline bool old_store_put(OldParamStore& store, std::uint16_t old_key,
                                        std::uint8_t old_group_element, std::uint8_t type,
                                        float value) {
    if (store.count >= kConversionStoreCap) {
        return false;
    }
    auto& e = store.entries[store.count++];
    e.old_key = old_key;
    e.old_group_element = old_group_element;
    e.type = type;
    e.value = value;
    e.present = true;
    return true;
}

[[nodiscard]] inline bool new_store_put(NewParamStore& store, const char* name, float value) {
    if (name == nullptr || store.count >= kConversionStoreCap) {
        return false;
    }
    for (std::size_t i = 0; i < store.count; ++i) {
        if (conversion_name_eq(store.entries[i].name, name)) {
            store.entries[i].value = value;
            store.entries[i].present = true;
            return true;
        }
    }
    auto& e = store.entries[store.count++];
    std::size_t i = 0;
    for (; i < 16 && name[i] != '\0'; ++i) {
        e.name[i] = name[i];
    }
    e.name[i] = '\0';
    e.value = value;
    e.present = true;
    return true;
}

[[nodiscard]] inline bool new_store_find(const NewParamStore& store, const char* name,
                                         float& out_value) {
    if (name == nullptr) {
        return false;
    }
    for (std::size_t i = 0; i < store.count; ++i) {
        if (store.entries[i].present && conversion_name_eq(store.entries[i].name, name)) {
            out_value = store.entries[i].value;
            return true;
        }
    }
    return false;
}

// Leftover find_old_parameter: inject lookup, not EEPROM scan
// (AP_Param.cpp ~2047-2062).
[[nodiscard]] inline bool find_old_parameter(const ConversionInfo& info, const OldParamStore& old,
                                             float& out_value) {
    for (std::size_t i = 0; i < old.count; ++i) {
        const auto& e = old.entries[i];
        if (!e.present) {
            continue;
        }
        if (e.old_key == info.old_key && e.old_group_element == info.old_group_element &&
            e.type == info.type) {
            out_value = e.value;
            return true;
        }
    }
    return false;
}

// Leftover convert_old_parameter (AP_Param.cpp ~2068-2121): if old found,
// apply REVERSE/FORCE + scaler and store by new_name. No type-memcpy path
// (same-type flags==0 memcpy) — scalar float path only. new_configured
// injects ap2->configured_in_storage().
inline void convert_old_parameter(const ConversionInfo& info, float scaler, std::uint8_t flags,
                                  const OldParamStore& old, NewParamStore& neu,
                                  bool new_configured = false) {
    if (info.new_name == nullptr) {
        return;
    }
    float v = 0.0f;
    if (!find_old_parameter(info, old, v)) {
        return;
    }
    // Upstream: if (!(flags & FORCE) && configured_in_storage()) return;
    if ((flags & kConvertFlagForce) == 0 && new_configured) {
        return;
    }
    // Upstream CONVERT_FLAG_REVERSE: _REV (-1/other) → _REVERSED (1/0).
    if ((flags & kConvertFlagReverse) != 0) {
        v = math::is_equal(v, -1.0f) ? 1.0f : 0.0f;
    }
    const float scaled = v * scaler;
    float existing = 0.0f;
    if (new_store_find(neu, info.new_name, existing) && math::is_equal(existing, scaled)) {
        return;
    }
    (void)new_store_put(neu, info.new_name, scaled);
}

// Upstream convert_old_parameters_scaled (AP_Param.cpp ~2131-2139): loop calling
// convert_old_parameter. flush() not reproduced (no EEPROM). new_configured is
// applied uniformly (inject stand-in for per-name configured_in_storage).
inline void convert_old_parameters_scaled(const ConversionInfo* table, std::uint8_t table_size,
                                         float scaler, std::uint8_t flags, const OldParamStore& old,
                                         NewParamStore& neu, bool new_configured = false) {
    if (table == nullptr) {
        return;
    }
    for (std::uint8_t i = 0; i < table_size; ++i) {
        convert_old_parameter(table[i], scaler, flags, old, neu, new_configured);
    }
}

// Upstream convert_old_parameters → convert_old_parameters_scaled(..., 1.0f, flags).
inline void convert_old_parameters(const ConversionInfo* table, std::uint8_t table_size,
                                  std::uint8_t flags, const OldParamStore& old, NewParamStore& neu,
                                  bool new_configured = false) {
    convert_old_parameters_scaled(table, table_size, 1.0f, flags, old, neu, new_configured);
}

// Inject stand-in for one GroupInfo scalar field in AP_Param::convert_class
// (AP_Param.cpp ~2143-2193). Upstream walks group_info[], packs
// old_group_element from idx/group_shift, find_old_parameter, then
// memcpy+save into object_pointer+offset. This leftover uses new_name +
// NewParamStore instead of object_pointer; no nested AP_PARAM_GROUP recurse
// (convert_g2 / convert_toplevel are thin loops over leftover_convert_class).
//
// Table rows are {type, new_name} (+ old_group_element for inject matching);
// shared old class key is the leftover_convert_class param_key argument.
struct ClassConversionInfo {
    std::uint8_t old_group_element = 0;
    std::uint8_t type = 0; // VarType as uint8_t
    const char* new_name = nullptr;
};

// Leftover convert_class: shared old class key → walk ClassConversionInfo
// table → convert_old_parameter(scaler=1, flags=0) per field.
// new_configured injects configured_in_storage skip. flush() not reproduced.
inline void leftover_convert_class(std::uint16_t param_key, const ClassConversionInfo* table,
                                   std::uint8_t table_size, const OldParamStore& old,
                                   NewParamStore& neu, bool new_configured = false) {
    if (table == nullptr) {
        return;
    }
    for (std::uint8_t i = 0; i < table_size; ++i) {
        ConversionInfo info{param_key, table[i].old_group_element, table[i].type,
                            table[i].new_name};
        convert_old_parameter(info, 1.0f, 0, old, neu, new_configured);
    }
}

// Inject stand-in for one G2ObjectConversion row (AP_Param.h ~481-485).
// Upstream carries object_pointer + GroupInfo*; this leftover supplies the
// ClassConversionInfo field table already (same shape as convert_class).
// old_group_element packing (old_index / group_shift) is assumed done by the
// caller — ADR-0012 inject, no find_top_level_key_by_pointer.
struct G2ConversionEntry {
    const ClassConversionInfo* table = nullptr;
    std::uint8_t table_size = 0;
};

// Leftover convert_g2_objects (AP_Param.cpp ~2197-2208): injected g2_old_key
// → walk G2ConversionEntry[] → leftover_convert_class per entry.
inline void leftover_convert_g2(std::uint16_t g2_old_key, const G2ConversionEntry* entries,
                                std::uint8_t num_entries, const OldParamStore& old,
                                NewParamStore& neu, bool new_configured = false) {
    if (entries == nullptr) {
        return;
    }
    for (std::uint8_t i = 0; i < num_entries; ++i) {
        leftover_convert_class(g2_old_key, entries[i].table, entries[i].table_size, old, neu,
                               new_configured);
    }
}

// Inject stand-in for one TopLevelObjectConversion row (AP_Param.h ~490-494).
// Upstream old_index is the former top-level key passed to convert_class.
struct ToplevelConversionEntry {
    std::uint16_t old_key = 0;
    const ClassConversionInfo* table = nullptr;
    std::uint8_t table_size = 0;
};

// Leftover convert_toplevel_objects (AP_Param.cpp ~2210-2216): each entry's
// old_key → leftover_convert_class (same as convert_class is_top_level path).
inline void leftover_convert_toplevel(const ToplevelConversionEntry* entries,
                                      std::uint8_t num_entries, const OldParamStore& old,
                                      NewParamStore& neu, bool new_configured = false) {
    if (entries == nullptr) {
        return;
    }
    for (std::uint8_t i = 0; i < num_entries; ++i) {
        leftover_convert_class(entries[i].old_key, entries[i].table, entries[i].table_size, old,
                               neu, new_configured);
    }
}

// Injected inputs for leftover_convert_parameter_width (no find_var_info /
// EEPROM scan). Caller supplies configured_in_storage and any already-found
// old float value (stand-in for cast_to_float(old_ptype) after scan).
struct WidthConvertInputs {
    bool configured_in_storage{false};
    bool old_value_found{false};
    float old_value{};
    float scale_factor{1.f};
    bool bitmask{false};
};

struct WidthConvertEffects {
    bool skipped_configured{false};
    bool skipped_missing{false};
    bool converted{false};
    float new_value{};
};

// Leftover AP_Param::_convert_parameter_width (AP_Param.cpp ~2222+).
// Upstream: if configured_in_storage return false; find old type in store;
// scale (or bitmask via uint32); set_value + save(true). This inject path
// returns the computed new float only — no EEPROM write.
//
// Non-bitmask: new_value = old_value * scale_factor.
// Bitmask: simple truncating cast through uint32 (float→u32→float). Typed
// AP_Int8/16/32 unsigned widen (int8 -1 → 255) is not reproduced — inject
// already holds a float-cast old value.
[[nodiscard]] inline bool leftover_convert_parameter_width(const WidthConvertInputs& in,
                                                           WidthConvertEffects& fx) {
    fx = WidthConvertEffects{};
    if (in.configured_in_storage) {
        fx.skipped_configured = true;
        return false;
    }
    if (!in.old_value_found) {
        fx.skipped_missing = true;
        return false;
    }
    if (!in.bitmask) {
        fx.new_value = in.old_value * in.scale_factor;
    } else {
        // Simple cast stub — not the full unsigned mask widen path.
        const auto mask = static_cast<std::uint32_t>(in.old_value);
        fx.new_value = static_cast<float>(mask);
    }
    fx.converted = true;
    return true;
}

// Thin leftover for convert_centi_parameter (AP_Param.h ~505-507): width
// convert with scale_factor 0.01f, bitmask false.
[[nodiscard]] inline bool leftover_convert_centi_parameter(const WidthConvertInputs& in,
                                                           WidthConvertEffects& fx) {
    WidthConvertInputs scaled = in;
    scaled.scale_factor = 0.01f;
    scaled.bitmask = false;
    return leftover_convert_parameter_width(scaled, fx);
}

// Thin leftover for convert_bitmask_parameter_width (AP_Param.h ~508-511):
// width convert with scale_factor 1.0f, bitmask true.
[[nodiscard]] inline bool leftover_convert_bitmask_parameter_width(const WidthConvertInputs& in,
                                                                   WidthConvertEffects& fx) {
    WidthConvertInputs bm = in;
    bm.scale_factor = 1.0f;
    bm.bitmask = true;
    return leftover_convert_parameter_width(bm, fx);
}

} // namespace fwcpp::param
