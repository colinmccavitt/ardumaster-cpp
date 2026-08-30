#pragma once

// Port of libraries/SITL/SIM_Battery.h + SIM_Battery.cpp (Copter-4.7.0).
// SoC table, IR sag, 10 Hz voltage filter, first-order temperature model.
// ADR-0012: consume_energy takes explicit now_us (no AP_HAL::micros64).

#include <cfloat>
#include <cstdint>

#include <fwcpp/filter/low_pass_filter.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::sim {

class Battery {
public:
    Battery() : voltage_filter_(10.0f) {}
    explicit Battery(float cutoff_hz) : voltage_filter_(cutoff_hz) {}

    void setup(float capacity_ah, float resistance_ohm, float max_voltage, float ambient_temperature_degc) {
        capacity_ah_ = capacity_ah;
        resistance_ohm_ = resistance_ohm;
        max_voltage_ = max_voltage;
        ambient_temperature_degc_ = ambient_temperature_degc;
        voltage_set_ = max_voltage;
        voltage_filter_.reset(voltage_set_);
        remaining_ah_ = compute_remaining_ah(voltage_set_);
        last_us_ = 0;
        temperature_degc_ = 0.0f;
    }

    void maybe_reset(float desired_voltage, float desired_capacity_ah, float desired_resistance_ohm = -1.0f) {
        if (!math::is_negative(desired_resistance_ohm)) {
            resistance_ohm_ = desired_resistance_ohm;
        }
        const bool reset_not_needed =
            math::is_equal(voltage_set_, desired_voltage) && math::is_equal(capacity_ah_, desired_capacity_ah);
        if (reset_not_needed) {
            return;
        }
        capacity_ah_ = desired_capacity_ah;
        voltage_set_ = std::fmin(desired_voltage, max_voltage_);
        voltage_filter_.reset(voltage_set_);
        remaining_ah_ = compute_remaining_ah(voltage_set_);
    }

    void consume_energy(float attempted_current_amp, std::uint64_t now_us) {
        constexpr float kMicrosecToSec = 1.0e-6f;
        constexpr float kMaxDt = 0.1f;
        const float dt = static_cast<float>(now_us - last_us_) * kMicrosecToSec;
        if (dt <= 0.0f) {
            return;
        }
        last_us_ = now_us;
        if (dt > kMaxDt) {
            return;
        }
        constexpr float kHoursPerSecond = 1.0f / 3600.0f;
        const float dt_hr = dt * kHoursPerSecond;
        const float delta_ah = std::fmin(attempted_current_amp * dt_hr, remaining_ah_);
        if (!capacity_is_unlimited()) {
            remaining_ah_ -= delta_ah;
        }
        const float current_amp = delta_ah / dt_hr;
        const float voltage_delta = current_amp * resistance_ohm_;
        const float sagged_voltage = get_resting_voltage() - voltage_delta;
        voltage_filter_.apply(sagged_voltage, dt);
        update_temperature(current_amp, dt);
    }

    [[nodiscard]] float get_voltage() const { return voltage_filter_.get(); }
    [[nodiscard]] float get_capacity() const { return capacity_ah_; }
    [[nodiscard]] float get_temperature_degC() const { return temperature_degc_; }
    [[nodiscard]] float remaining_ah() const { return remaining_ah_; }
    [[nodiscard]] bool capacity_is_unlimited() const { return !(math::is_positive(capacity_ah_)); }

private:
    struct SocRow {
        float volt_per_cell;
        float soc_pct;
    };

    static constexpr SocRow kSocTable[] = {
        {4.173f, 100.0f},  {4.112f, 96.15f}, {4.085f, 92.31f}, {4.071f, 88.46f}, {4.039f, 84.62f},
        {3.987f, 80.77f},  {3.943f, 76.92f}, {3.908f, 73.08f}, {3.887f, 69.23f}, {3.854f, 65.38f},
        {3.833f, 61.54f},  {3.801f, 57.69f}, {3.783f, 53.85f}, {3.742f, 50.0f},  {3.715f, 46.15f},
        {3.679f, 42.31f},  {3.636f, 38.46f}, {3.588f, 34.62f}, {3.543f, 30.77f}, {3.503f, 26.92f},
        {3.462f, 23.08f},  {3.379f, 19.23f}, {3.296f, 15.38f}, {3.218f, 11.54f}, {3.165f, 7.69f},
        {3.091f, 3.85f},   {2.977f, 2.0f},   {2.8f, 1.5f},    {2.7f, 1.3f},    {2.5f, 1.2f},
        {2.3f, 1.1f},      {2.1f, 1.0f},     {1.9f, 0.9f},    {1.6f, 0.8f},    {1.3f, 0.7f},
        {1.0f, 0.6f},      {0.6f, 0.4f},     {0.3f, 0.2f},    {0.01f, 0.01f},  {0.001f, 0.001f},
    };

    static constexpr std::size_t kSocTableSize = sizeof(kSocTable) / sizeof(kSocTable[0]);

    [[nodiscard]] float get_resting_voltage() const {
        if (capacity_is_unlimited()) {
            return voltage_set_;
        }
        const float charge_pct = 100.0f * remaining_ah_ / capacity_ah_;
        const float max_cell_voltage = kSocTable[0].volt_per_cell;
        const float min_cell_voltage = kSocTable[kSocTableSize - 1].volt_per_cell;
        for (std::size_t i = 1; i < kSocTableSize; ++i) {
            if (charge_pct >= kSocTable[i].soc_pct) {
                const float dv1 = charge_pct - kSocTable[i].soc_pct;
                const float dv2 = kSocTable[i - 1].soc_pct - kSocTable[i].soc_pct;
                const float vpc1 = kSocTable[i].volt_per_cell;
                const float vpc2 = kSocTable[i - 1].volt_per_cell;
                const float cell_volt = vpc1 + (dv1 / dv2) * (vpc2 - vpc1);
                return (cell_volt / max_cell_voltage) * max_voltage_;
            }
        }
        return min_cell_voltage;
    }

    [[nodiscard]] float compute_remaining_ah(float voltage) const {
        if (capacity_is_unlimited()) {
            return FLT_MAX;
        }
        const float max_cell_voltage = kSocTable[0].volt_per_cell;
        const float cell_volt = (voltage / max_voltage_) * max_cell_voltage;
        for (std::size_t i = 1; i < kSocTableSize; ++i) {
            if (cell_volt >= kSocTable[i].volt_per_cell) {
                const float dv1 = cell_volt - kSocTable[i].volt_per_cell;
                const float dv2 = kSocTable[i - 1].volt_per_cell - kSocTable[i].volt_per_cell;
                const float soc1 = kSocTable[i].soc_pct;
                const float soc2 = kSocTable[i - 1].soc_pct;
                const float soc = soc1 + (dv1 / dv2) * (soc2 - soc1);
                return capacity_ah_ * (soc * 0.01f);
            }
        }
        return 0.0f;
    }

    void update_temperature(float current_amp, float dt) {
        constexpr float kInverseThermalCapacity = 1.0f / 500.0f;
        constexpr float kTemperatureDecay = 5.6e-4f;
        const float temp_increase =
            (current_amp * current_amp) * resistance_ohm_ * kInverseThermalCapacity * dt;
        const float temp_decrease = (temperature_degc_ - ambient_temperature_degc_) * kTemperatureDecay * dt;
        temperature_degc_ += (temp_increase - temp_decrease);
    }

    float capacity_ah_{0.0f};
    float resistance_ohm_{0.01f};
    float max_voltage_{12.6f};
    float ambient_temperature_degc_{25.0f};
    float voltage_set_{12.6f};
    float remaining_ah_{0.0f};
    std::uint64_t last_us_{0};
    float temperature_degc_{0.0f};
    filter::LowPassFilterFloat voltage_filter_{};
};

}  // namespace fwcpp::sim
