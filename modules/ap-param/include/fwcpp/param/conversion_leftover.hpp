#pragma once

// CPP-023 leftover completeness catalog — AP_Param conversion/upgrade
// (Plane-4.7.0 AP_Param.cpp convert_old_* / _convert_parameter_width /
// convert_class). Nested under fwcpp::param::conversion so remaining_count()
// does not collide with other param helpers.
//
// Slice 1: ConversionInfo + convert_old_parameters_scaled leftover scaffold
// (inject OldParamStore / NewParamStore; no EEPROM).
// Slice 2: leftover_convert_parameter_width inject (no EEPROM / find_var_info).
// Slice 3: convert_old_parameter CONVERT_FLAG_REVERSE / FORCE (inject
// new_configured).
// Slice 4: leftover_convert_class (old class key → ClassConversionInfo table →
// convert_old_parameter). Remaining rows are later CPP-023 work.

#include <cstddef>
#include <cstdint>

namespace fwcpp::param::conversion {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PortItem kCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"ConversionInfo", PortStatus::kThisSlice,
     "AP_Param.h ~232-237; uint8_t old_group_element/type this slice"},
    {"convert_old_parameters_scaled", PortStatus::kThisSlice,
     "AP_Param.cpp ~2131-2139; inject find → scaler → NewParamStore"},
    {"convert_old_parameters", PortStatus::kThisSlice,
     "AP_Param.cpp ~2125-2128; scaled with 1.0f"},
    {"find_old_parameter inject", PortStatus::kThisSlice,
     "inject OldParamStore; no EEPROM scan"},
    {"convert_old_parameter REVERSE/FORCE", PortStatus::kThisSlice,
     "CONVERT_FLAG_REVERSE (_REV→_REVERSED) / FORCE + inject new_configured"},
    {"find_old_parameter EEPROM scan", PortStatus::kRemaining,
     "AP_Param.cpp ~2047-2062 scan()+read_block"},
    {"convert_class", PortStatus::kThisSlice,
     "AP_Param.cpp ~2143-2193; leftover_convert_class inject; no object_pointer/flush"},
    {"convert_g2 / convert_toplevel objects", PortStatus::kRemaining,
     "AP_Param.cpp ~2197-2218"},
    {"_convert_parameter_width", PortStatus::kThisSlice,
     "AP_Param.cpp ~2222+; leftover_convert_parameter_width inject; no EEPROM"},
    {"bitmask / centi width helpers", PortStatus::kRemaining,
     "convert_bitmask_parameter_width / convert_centi_parameter; typed mask widen"},
    {"flush after convert", PortStatus::kRemaining,
     "AP_Param.cpp ~2137-2139 / ~2190-2192 StorageManager flush"},
    {"AP_Param singleton / EEPROM", PortStatus::kOutOfScope,
     "ADR-0012 inject stores; no hal.storage singleton"},
};

[[nodiscard]] inline constexpr std::size_t completeness_size() {
    return sizeof(kCompleteness) / sizeof(kCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

} // namespace fwcpp::param::conversion
