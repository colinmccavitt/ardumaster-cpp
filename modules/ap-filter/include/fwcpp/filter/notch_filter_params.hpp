#pragma once

// Port of Filter/AP_Filter.h's FilterType enum and AP_NotchFilter_params
// (var_info table + constructor + setup_notch_filter), plus the
// GCS-configurable NOTCH_FREQ/NOTCH_Q/NOTCH_ATT parameters themselves.
// CPP-024, slice 1. See ADR-0013 for the AP_Param sub-effort's scoping.
//
// SLICE BOUNDARY: this is ONE filter's worth of parameters and the logic
// to apply them to a NotchFilter - the piece AC_PID actually calls.
// Deliberately NOT in this slice: AP_Filters (the up-to-8-slot registry,
// AP_Filter_params' _type selector, and AP_Filters::update()'s dynamic
// per-slot backend allocation via AP_SUBGROUPVARPTR's runtime-varying
// group_info pointer - a materially bigger piece needing kFlagInfoPointer
// support this port's find_group doesn't have yet). AC_PID's own
// set_notch_sample_rate/notch application (CPP-024's other half) is
// also separate follow-on work, layered on top of this.
//
// var_info AS A STATIC MEMBER FUNCTION, not a static data member: the
// table's initializer needs offsetof(NotchFilterParams, center_freq_hz_)
// etc, which requires NotchFilterParams to be a COMPLETE type - true for
// an inline member FUNCTION body (member function bodies are parsed in
// the class's "complete-class context", after the whole class has been
// seen) but not for an in-class static DATA member initializer (which
// upstream itself avoids by defining var_info out-of-line in a .cpp,
// after the class's own header has already been fully parsed - this is
// the header-only equivalent of that same ordering constraint).
//
// GroupInfo names/keys/defaults below are transcribed directly from
// upstream's own AP_NotchFilter_params::var_info (Filter/
// AP_NotchFilter_params.cpp): NOTCH_FREQ (idx 1, default 0), NOTCH_Q
// (idx 2, default 2), NOTCH_ATT (idx 3, default 40) - idx 0 is skipped
// upstream too (probably historical, matching group_id's own aliasing
// workaround for idx 0 - not reproduced as a guess, transcribed as
// found).

#include <cstddef>
#include <cstdint>

#include <fwcpp/filter/notch_filter.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/param/defaults.hpp>
#include <fwcpp/param/group_info.hpp>
#include <fwcpp/param/param.hpp>

namespace fwcpp::filter {

enum class FilterType : std::uint8_t {
    None = 0,
    Notch = 1,
};

class NotchFilterParams {
private:
    // Declared before var_info() deliberately: a constexpr evaluation
    // (var_info()'s own table initializer) needs these two functions'
    // COMPLETE definitions already parsed at that point - unlike an
    // ordinary (non-constexpr-evaluated) member call, which tolerates a
    // forward reference to a later-declared member via the class's
    // "complete-class context".
    [[nodiscard]] static constexpr param::GroupInfo make_entry(const char* name, std::ptrdiff_t offset, std::uint8_t idx, float def_value) {
        param::GroupInfo g{};
        g.name = name;
        g.offset = offset;
        g.idx = idx;
        g.type = static_cast<std::uint8_t>(param::VarType::Float);
        g.def_value = def_value;
        return g;
    }

    [[nodiscard]] static constexpr param::GroupInfo make_sentinel() {
        param::GroupInfo g{};
        g.type = static_cast<std::uint8_t>(param::VarType::None);
        return g;
    }

public:
    NotchFilterParams() { param::setup_object_defaults(this, var_info()); }

    [[nodiscard]] static const param::GroupInfo* var_info() {
        static constexpr param::GroupInfo table[] = {
            make_entry("NOTCH_FREQ", offsetof(NotchFilterParams, center_freq_hz_), 1, 0.0f),
            make_entry("NOTCH_Q", offsetof(NotchFilterParams, quality_), 2, 2.0f),
            make_entry("NOTCH_ATT", offsetof(NotchFilterParams, attenuation_db_), 3, 40.0f),
            make_sentinel(),
        };
        return table;
    }

    // Applies the currently-configured parameters to `filt`, re-
    // initializing it only when the sample rate or center frequency has
    // actually changed (matches upstream's own is_equal-guarded re-init,
    // avoiding NotchFilter::init's own reset-detection churn on every
    // call). Returns false (leaving `filt` untouched) if quality, center
    // frequency, or attenuation is zero - "not configured", matching
    // upstream exactly.
    [[nodiscard]] bool setup_notch_filter(NotchFilter<float>& filt, float sample_rate) const {
        if (math::is_zero(quality_.get()) || math::is_zero(center_freq_hz_.get()) || math::is_zero(attenuation_db_.get())) {
            return false;
        }
        if (!math::is_equal(sample_rate, filt.sample_freq_hz()) || !math::is_equal(center_freq_hz_.get(), filt.center_freq_hz())) {
            filt.init(sample_rate, center_freq_hz_.get(), center_freq_hz_.get() / quality_.get(), attenuation_db_.get());
        }
        return true;
    }

    [[nodiscard]] const param::ParamFloat& center_freq_hz() const { return center_freq_hz_; }
    [[nodiscard]] const param::ParamFloat& quality() const { return quality_; }
    [[nodiscard]] const param::ParamFloat& attenuation_db() const { return attenuation_db_; }

    param::ParamFloat center_freq_hz_;
    param::ParamFloat quality_;
    param::ParamFloat attenuation_db_;
};

} // namespace fwcpp::filter
