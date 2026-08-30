#pragma once

// CPP-023 slice 1: AP_Param ConversionInfo + convert_old_parameters_scaled
// leftover scaffold (Plane-4.7.0 AP_Param.cpp ~2125-2160, convert_old_parameter
// ~2068-2121). ADR-0012: no EEPROM / no AP_Param singleton — injected
// OldParamStore / NewParamStore maps stand in for find_old_parameter's
// storage scan and find()+save of the new name.
//
// THIS SLICE: table loop, inject lookup, scaler apply into NewParamStore
// by new_name. Remaining (_convert_parameter_width, bitmasks, convert_class,
// CONVERT_FLAG_REVERSE/FORCE, EEPROM find_old_parameter) in
// conversion_leftover.hpp.

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

// Upstream CONVERT_FLAG_* (AP_Param.h ~514-517). Scaffold ignores flags;
// full REVERSE/FORCE behavior is cataloged as remaining.
inline constexpr std::uint8_t kConvertFlagReverse = 1;
inline constexpr std::uint8_t kConvertFlagForce = 2;

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
// apply scaler and store by new_name. No type-memcpy path, no REVERSE/FORCE,
// no configured_in_storage skip — those remain.
inline void convert_old_parameter(const ConversionInfo& info, float scaler, std::uint8_t /*flags*/,
                                  const OldParamStore& old, NewParamStore& neu) {
    if (info.new_name == nullptr) {
        return;
    }
    float v = 0.0f;
    if (!find_old_parameter(info, old, v)) {
        return;
    }
    const float scaled = v * scaler;
    float existing = 0.0f;
    if (new_store_find(neu, info.new_name, existing) && math::is_equal(existing, scaled)) {
        return;
    }
    (void)new_store_put(neu, info.new_name, scaled);
}

// Upstream convert_old_parameters_scaled (AP_Param.cpp ~2131-2139): loop calling
// convert_old_parameter. flush() not reproduced (no EEPROM).
inline void convert_old_parameters_scaled(const ConversionInfo* table, std::uint8_t table_size,
                                         float scaler, std::uint8_t flags, const OldParamStore& old,
                                         NewParamStore& neu) {
    if (table == nullptr) {
        return;
    }
    for (std::uint8_t i = 0; i < table_size; ++i) {
        convert_old_parameter(table[i], scaler, flags, old, neu);
    }
}

// Upstream convert_old_parameters → convert_old_parameters_scaled(..., 1.0f, flags).
inline void convert_old_parameters(const ConversionInfo* table, std::uint8_t table_size,
                                  std::uint8_t flags, const OldParamStore& old, NewParamStore& neu) {
    convert_old_parameters_scaled(table, table_size, 1.0f, flags, old, neu);
}

} // namespace fwcpp::param
