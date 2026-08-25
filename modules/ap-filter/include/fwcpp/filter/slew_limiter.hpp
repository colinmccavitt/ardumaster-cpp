#pragma once

// Port of Filter/SlewLimiter.h + SlewLimiter.cpp. CPP-015.
//
// Reduces a PID controller's P+D output when the actuator demand is
// changing faster than the actuator can physically move, preventing the
// demand and achieved rate from getting out of phase and oscillating.
// Written for fixed-wing by Paul Riseborough, later generalized - directly
// relevant to this effort's own scope.
//
// AP_HAL::millis() REPLACED WITH AN EXPLICIT PARAMETER: upstream's
// modifier() reaches AP_HAL::millis() internally to timestamp slew-rate
// exceedance events. That's a HAL singleton call, which ADR-0012 decision 6
// forbids reproducing - and this port has no HAL time abstraction built yet
// regardless (AP_HAL itself is untouched). modifier() takes `now_ms`
// explicitly instead; the caller (eventually the vehicle scheduler) is
// responsible for supplying it, the same way constrain_value takes an
// InternalError* instead of reaching AP::internalerror().
//
// slew_rate_max/slew_rate_tau kept as upstream declares them - const
// float& references, not owned values - so a caller backed by a live
// parameter (once AP_Param exists in this port) can update them without
// reconstructing the limiter. This is an aliasing choice upstream itself
// makes explicitly (not a singleton), so ADR-0012 decision 6 doesn't argue
// against reproducing it.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/filter/low_pass_filter.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::filter {

class SlewLimiter {
public:
    static constexpr std::uint8_t kNumEvents = 2; // matches upstream's SLEWLIMITER_N_EVENTS

    SlewLimiter(const float& slew_rate_max, const float& slew_rate_tau)
        : slew_rate_max_(slew_rate_max), slew_rate_tau_(slew_rate_tau) {
        slew_filter_.set_cutoff_frequency(kDerivativeCutoffFreq);
        slew_filter_.reset(0.0f);
    }

    SlewLimiter(const SlewLimiter&) = delete;
    SlewLimiter& operator=(const SlewLimiter&) = delete;

    // Returns a multiplier in [0, 1] to scale P+D output down by, keeping
    // the actuator's demanded rate of change within slew_rate_max.
    // `now_ms` replaces upstream's internal AP_HAL::millis() call - see
    // file banner.
    float modifier(float sample, float dt, std::uint32_t now_ms) {
        if (!math::is_positive(dt)) {
            return 1.0f;
        }

        const float slew_rate = slew_filter_.apply((sample - last_sample_) / dt, dt);
        last_sample_ = sample;

        const float decay_alpha = std::min(dt, slew_rate_tau_) / slew_rate_tau_;
        const float attack_alpha = std::min(2.0f * decay_alpha, 1.0f);

        if (slew_rate > max_pos_slew_rate_) {
            max_pos_slew_rate_ = slew_rate;
            max_pos_slew_event_ms_ = now_ms;
        } else if (now_ms - max_pos_slew_event_ms_ > kWindowMs) {
            max_pos_slew_rate_ *= (1.0f - decay_alpha);
        }

        if (-slew_rate > max_neg_slew_rate_) {
            max_neg_slew_rate_ = -slew_rate;
            max_neg_slew_event_ms_ = now_ms;
        } else if (now_ms - max_neg_slew_event_ms_ > kWindowMs) {
            max_neg_slew_rate_ *= (1.0f - decay_alpha);
        }

        const float raw_slew_rate = 0.5f * (max_pos_slew_rate_ + max_neg_slew_rate_);
        output_slew_rate_ = (1.0f - attack_alpha) * output_slew_rate_ + attack_alpha * raw_slew_rate;
        output_slew_rate_ = std::min(output_slew_rate_, raw_slew_rate);

        if (slew_rate_max_ <= 0.0f) {
            return 1.0f;
        }

        const float limited_raw_slew_rate =
            0.5f * (std::min(max_pos_slew_rate_, 10.0f * slew_rate_max_)
                  + std::min(max_neg_slew_rate_, 10.0f * slew_rate_max_));

        if (!pos_event_stored_ && slew_rate > slew_rate_max_) {
            if (pos_event_index_ >= kNumEvents) {
                pos_event_index_ = 0;
            }
            pos_event_ms_[pos_event_index_] = now_ms;
            ++pos_event_index_;
            pos_event_stored_ = true;
            neg_event_stored_ = false;
        }

        if (!neg_event_stored_ && -slew_rate > slew_rate_max_) {
            if (neg_event_index_ >= kNumEvents) {
                neg_event_index_ = 0;
            }
            neg_event_ms_[neg_event_index_] = now_ms;
            ++neg_event_index_;
            neg_event_stored_ = true;
            pos_event_stored_ = false;
        }

        std::uint32_t oldest_ms = now_ms;
        for (std::uint8_t i = 0; i < kNumEvents; ++i) {
            oldest_ms = std::min(oldest_ms, pos_event_ms_[i]);
            oldest_ms = std::min(oldest_ms, neg_event_ms_[i]);
        }

        float modifier_input = limited_raw_slew_rate;
        if (now_ms - oldest_ms > (kNumEvents + 1u) * kWindowMs) {
            const float oldest_time_from_window =
                0.001f * static_cast<float>(now_ms - oldest_ms - (kNumEvents + 1u) * kWindowMs);
            modifier_input *= std::exp(-oldest_time_from_window / slew_rate_tau_);
        }

        modifier_slew_rate_ = (1.0f - attack_alpha) * modifier_slew_rate_ + attack_alpha * modifier_input;
        modifier_slew_rate_ = std::min(modifier_slew_rate_, modifier_input);

        float mod = 1.0f;
        if (modifier_slew_rate_ > slew_rate_max_) {
            mod = slew_rate_max_ / (slew_rate_max_ + kModifierGain * (modifier_slew_rate_ - slew_rate_max_));
        }
        return mod;
    }

    [[nodiscard]] float get_slew_rate() const { return output_slew_rate_; }

private:
    static constexpr std::uint32_t kWindowMs = 300;
    static constexpr float kModifierGain = 1.5f;
    static constexpr float kDerivativeCutoffFreq = 25.0f;

    const float& slew_rate_max_;
    const float& slew_rate_tau_;
    LowPassFilterFloat slew_filter_;
    float output_slew_rate_ = 0.0f;
    float modifier_slew_rate_ = 0.0f;
    float last_sample_ = 0.0f;
    float max_pos_slew_rate_ = 0.0f;
    float max_neg_slew_rate_ = 0.0f;
    std::uint32_t max_pos_slew_event_ms_ = 0;
    std::uint32_t max_neg_slew_event_ms_ = 0;
    std::uint8_t pos_event_index_ = 0;
    std::uint8_t neg_event_index_ = 0;
    std::uint32_t pos_event_ms_[kNumEvents] = {};
    std::uint32_t neg_event_ms_[kNumEvents] = {};
    bool pos_event_stored_ = false;
    bool neg_event_stored_ = false;
};

} // namespace fwcpp::filter
