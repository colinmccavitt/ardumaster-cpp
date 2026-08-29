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

struct QAcroRunInputs {
    bool tailsitter_in_vtol_transition{false};
    bool throttle_wait{false};
};

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
    bool throttle_out_no_angle_boost{false};
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

[[nodiscard]] inline QAcroRunActions qacro_run_actions(QAcroRunPhase phase) {
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
            out.throttle_out_no_angle_boost = true;
            out.run_mode_acro_fw = true;
            break;
    }
    return out;
}

}  // namespace fwcpp::q_modes
