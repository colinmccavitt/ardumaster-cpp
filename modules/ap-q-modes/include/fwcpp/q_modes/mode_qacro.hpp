#pragma once

#include <fwcpp/q_modes/q_run_common.hpp>

#include <cstdint>

namespace fwcpp::q_modes {

struct QAcroEnterResult {
    bool entered{true};
    bool throttle_wait{false};
    bool force_transition_complete{true};
    bool relax_attitude_controllers{true};
    bool disable_yaw_rate_time_constant{true};
};

[[nodiscard]] inline QAcroEnterResult qacro_enter() {
    return QAcroEnterResult{};
}

enum class QAcroRunPhase : std::uint8_t {
    kFwTransitionControllers = 0,
    kThrottleWait = 1,
    kAcroRates = 2,
};

enum class QAcroRateInputVariant : std::uint8_t {
    kNoLocking2 = 0,
    kLocking3 = 1,
};

struct QAcroRunInputs {
    bool tailsitter_in_vtol_transition{false};
    bool throttle_wait{false};
    bool acro_locking{false};
    float roll_norm{0.0f};
    float pitch_norm{0.0f};
    float yaw_norm{0.0f};
    float acro_roll_rate{0.0f};
    float acro_pitch_rate{0.0f};
    float acro_yaw_rate{0.0f};
    bool tailsitter_enabled{false};
    float pilot_throttle_scaled{0.0f};
};

[[nodiscard]] inline QAcroRateInputVariant qacro_rate_input_variant(bool acro_locking) {
    return acro_locking ? QAcroRateInputVariant::kLocking3 : QAcroRateInputVariant::kNoLocking2;
}

[[nodiscard]] inline QAcroRunPhase qacro_run_phase(const QAcroRunInputs& in) {
    if (run_delegates_to_fw_controllers(in.tailsitter_in_vtol_transition)) {
        return QAcroRunPhase::kFwTransitionControllers;
    }
    if (in.throttle_wait) {
        return QAcroRunPhase::kThrottleWait;
    }
    return QAcroRunPhase::kAcroRates;
}

struct QAcroBodyRates {
    float roll_cds{0.0f};
    float pitch_cds{0.0f};
    float yaw_cds{0.0f};
};

struct QAcroRunActions {
    bool ground_idle_spool{false};
    bool relax_attitude{false};
    bool throttle_unlimited_spool{false};
    bool input_rate_bf{false};
    QAcroRateInputVariant rate_input{QAcroRateInputVariant::kNoLocking2};
    bool throttle_out_no_angle_boost{false};
    float throttle_out{0.0f};
    bool run_mode_acro_fw{false};
    QAcroBodyRates rates{};
};

[[nodiscard]] inline QAcroBodyRates qacro_body_rates_from_sticks(float roll_norm, float pitch_norm,
                                                                 float yaw_norm, float acro_roll_rate,
                                                                 float acro_pitch_rate, float acro_yaw_rate,
                                                                 bool tailsitter_enabled) {
    QAcroBodyRates out{};
    if (tailsitter_enabled) {
        out.roll_cds = yaw_norm * acro_yaw_rate * 100.0f;
        out.pitch_cds = pitch_norm * acro_pitch_rate * 100.0f;
        out.yaw_cds = -roll_norm * acro_roll_rate * 100.0f;
    } else {
        out.roll_cds = roll_norm * acro_roll_rate * 100.0f;
        out.pitch_cds = pitch_norm * acro_pitch_rate * 100.0f;
        out.yaw_cds = yaw_norm * acro_yaw_rate * 100.0f;
    }
    return out;
}

[[nodiscard]] inline QAcroRunActions qacro_run_actions(QAcroRunPhase phase, const QAcroRunInputs& in) {
    QAcroRunActions out{};
    switch (phase) {
        case QAcroRunPhase::kFwTransitionControllers:
            break;
        case QAcroRunPhase::kThrottleWait:
            out.ground_idle_spool = true;
            out.relax_attitude = true;
            break;
        case QAcroRunPhase::kAcroRates:
            out.throttle_unlimited_spool = true;
            out.input_rate_bf = true;
            out.rate_input = qacro_rate_input_variant(in.acro_locking);
            out.rates = qacro_body_rates_from_sticks(in.roll_norm, in.pitch_norm, in.yaw_norm, in.acro_roll_rate,
                                                     in.acro_pitch_rate, in.acro_yaw_rate, in.tailsitter_enabled);
            out.throttle_out_no_angle_boost = true;
            out.throttle_out = in.pilot_throttle_scaled;
            out.run_mode_acro_fw = true;
            break;
    }
    return out;
}

struct QAcroRunResult {
    QAcroRunPhase phase{QAcroRunPhase::kAcroRates};
    QAcroRunActions actions{};
    bool delegate_mode_run{false};
};

/// Port of ModeQAcro::run (throttle wait vs body-frame rates + mode_acro FW stabilize).
[[nodiscard]] inline QAcroRunResult qacro_run(const QAcroRunInputs& in) {
    QAcroRunResult out{};
    out.phase = qacro_run_phase(in);
    if (out.phase == QAcroRunPhase::kFwTransitionControllers) {
        out.delegate_mode_run = true;
        return out;
    }
    out.actions = qacro_run_actions(out.phase, in);
    return out;
}

}  // namespace fwcpp::q_modes
