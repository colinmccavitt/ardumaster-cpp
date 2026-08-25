#pragma once

// Port of Filter/AP_Filter.h's AP_Filters (the up-to-8-slot filter
// registry AC_PID's set_notch_sample_rate looks a configured filter up
// in by index). CPP-024, slice 2. See ADR-0013 for the AP_Param
// sub-effort's overall scoping, and notch_filter_params.hpp for slice 1.
//
// DIVERGENCE, deliberate and registered (matching ADR-0007's "fix bugs,
// register divergences" for the port itself, extended here to a
// deliberate scope simplification, not a bug fix): upstream's AP_Filters
// dynamically allocates each slot's concrete filter backend on first use
// (AP_Filters::update(), heap-allocating an AP_NotchFilter_params via
// NEW_NOTHROW once that slot's own AP_Filter_params::_type selector
// param is set to FILTER_NOTCH), addressed through AP_SUBGROUPVARPTR's
// runtime-varying group_info pointer (kFlagInfoPointer) - a materially
// bigger piece this port's find_group doesn't support yet (see
// name_lookup.hpp's banner).
//
// This port statically embeds one NotchFilterParams per slot instead,
// and drops the per-slot FILT{n}_TYPE selector parameter entirely -
// FILTER_NOTCH is the only FilterType upstream itself currently defines
// (see Filter/AP_Filter.h's own FilterType enum: FILTER_NONE=0,
// FILTER_NOTCH=1, nothing else), so there is no OTHER filter kind a
// user could actually select via that parameter today; a slot with
// NOTCH_FREQ left at its zero default already behaves identically to an
// upstream slot with _type==FILTER_NONE (setup_notch_filter's own zero-
// check treats zero center_freq_hz as "not configured" either way - see
// notch_filter_params.hpp). The user-visible parameter surface this
// simplifies away is real (one fewer GCS-configurable parameter per
// slot than a real vehicle would show) and is called out here plainly,
// not silently dropped - if a second FilterType is ever added upstream,
// this simplification stops being valid and this file needs revisiting,
// which is exactly why it's documented instead of just done.
//
// Slot indexing matches upstream's own 1-based get_filter(index)
// contract: index 0 is never valid (AC_PID only calls this when its own
// _notch_T_filter/_notch_E_filter index param is nonzero), matching
// upstream's `filters[index-1]` exactly.

#include <array>
#include <cstdint>

#include <fwcpp/filter/notch_filter_params.hpp>

namespace fwcpp::filter {

class FilterRegistry {
public:
    static constexpr std::uint8_t kNumFilters = 8; // AP_FILTER_NUM_FILTERS for HAL_PROGRAM_SIZE_LIMIT_KB > 1024 (this port's SITL target)

    FilterRegistry(const FilterRegistry&) = delete;
    FilterRegistry& operator=(const FilterRegistry&) = delete;
    FilterRegistry() = default;

    // Returns the notch-filter parameters for the given 1-based slot
    // index, or nullptr if index is 0 or out of range - matches upstream
    // AP_Filters::get_filter's own contract exactly (see file banner for
    // why every returned slot is a NotchFilterParams unconditionally,
    // rather than only when a _type selector says so).
    [[nodiscard]] NotchFilterParams* get_notch_filter(std::uint8_t index) {
        if (index == 0 || index > kNumFilters) {
            return nullptr;
        }
        return &slots_[index - 1];
    }
    [[nodiscard]] const NotchFilterParams* get_notch_filter(std::uint8_t index) const {
        if (index == 0 || index > kNumFilters) {
            return nullptr;
        }
        return &slots_[index - 1];
    }

private:
    std::array<NotchFilterParams, kNumFilters> slots_;
};

} // namespace fwcpp::filter
