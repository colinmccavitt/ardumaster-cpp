#pragma once

#include <cstdint>
#include <optional>

#include <fwcpp/quadplane/quadplane_defaults.hpp>
#include <fwcpp/quadplane/quadplane_frame.hpp>
#include <fwcpp/quadplane/quadplane_motors_init.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

namespace fwcpp::quadplane {

struct QuadPlaneSetupInputs {
    bool soft_armed{false};
    std::uint32_t available_memory_bytes{kSetupMinMemoryBytes};
};

class QuadPlane {
public:
    QuadPlane() = default;
    explicit QuadPlane(std::int8_t enable) : enable_{enable} {}

    void set_enable(std::int8_t enable) { enable_ = enable; }
    [[nodiscard]] std::int8_t enable() const { return enable_; }
    void set_frame_class(std::uint8_t v) { frame_class_ = v; }
    [[nodiscard]] std::uint8_t frame_class() const { return frame_class_; }
    void set_frame_type(std::uint8_t v) { frame_type_ = v; }
    [[nodiscard]] std::uint8_t frame_type() const { return frame_type_; }
    void set_tailsit_enable(std::int8_t v) { tailsit_enable_ = v; }
    [[nodiscard]] std::int8_t tailsit_enable() const { return tailsit_enable_; }
    void set_tilt_enable(std::int8_t v) { tilt_enable_ = v; }
    [[nodiscard]] std::int8_t tilt_enable() const { return tilt_enable_; }
    void set_options(std::int32_t v) { options_ = v; }
    [[nodiscard]] std::int32_t options() const { return options_; }
    [[nodiscard]] bool option_is_set(QOption option) const {
        return fwcpp::quadplane::option_is_set(options_, option);
    }

    [[nodiscard]] bool enabled() const { return enable_ != 0; }
    [[nodiscard]] bool initialised() const { return initialised_; }
    [[nodiscard]] bool available() const { return initialised_; }
    [[nodiscard]] bool motors_inited() const { return motors_inited_; }
    [[nodiscard]] const MotorsInitParams& motors_init_params() const { return motors_init_; }
    [[nodiscard]] std::optional<MotorsInitParams> motors_init_params_if_inited() const {
        if (!motors_inited_) return std::nullopt;
        return motors_init_;
    }

    [[nodiscard]] std::optional<MotorsKind> motors_kind() const {
        if (!motors_inited_) return std::nullopt;
        return motors_kind_;
    }

    [[nodiscard]] std::optional<VtolAirframe> vtol_airframe() const {
        if (!motors_inited_) return std::nullopt;
        const auto sel = classify_frame(frame_class_, tailsit_enable_, tilt_enable_);
        if (!sel) return std::nullopt;
        return sel->airframe;
    }

    bool setup(const QuadPlaneSetupInputs& inputs = {}) {
        if (initialised_) return true;
        if (inputs.soft_armed) return false;
        if (inputs.available_memory_bytes < kSetupMinMemoryBytes) return false;
        if (!enabled()) return false;
        const auto sel = classify_frame(frame_class_, tailsit_enable_, tilt_enable_);
        if (!sel) return false;
        motors_kind_ = sel->motors_kind;
        motors_init_ = make_motors_init_params(frame_class_, frame_type_);
        motors_inited_ = true;
        attitude_control_inited_ = true;
        pos_control_inited_ = true;
        weathervane_inited_ = true;
        initialised_ = true;
        return true;
    }

    [[nodiscard]] bool attitude_control_inited() const { return attitude_control_inited_; }
    [[nodiscard]] bool pos_control_inited() const { return pos_control_inited_; }
    [[nodiscard]] bool weathervane_inited() const { return weathervane_inited_; }

    [[nodiscard]] bool q_assist_force_enable_from_options() const {
        return option_is_set(QOption::kQAssistForceEnable);
    }

    void set_lean_angle_max_cd(std::int32_t v) { lean_angle_max_cd_ = v; }
    [[nodiscard]] std::int32_t lean_angle_max_cd() const { return lean_angle_max_cd_; }

    [[nodiscard]] const PosControlState& poscontrol() const { return poscontrol_; }
    PosControlState& poscontrol_mut() { return poscontrol_; }

    void set_guided_wait_takeoff(bool v) { guided_wait_takeoff_ = v; }
    [[nodiscard]] bool guided_wait_takeoff_on_mode_enter() const {
        return guided_wait_takeoff_on_mode_enter_;
    }

    void mode_enter() {
        if (available()) {
            lean_angle_max_cd_ = 0;
        }
        poscontrol_.reset_on_mode_enter();
        guided_wait_takeoff_on_mode_enter_ = guided_wait_takeoff_;
        guided_wait_takeoff_ = false;
    }

private:
    std::int8_t enable_{kQEnableDefault};
    std::uint8_t frame_class_{kQFrameClassDefault};
    std::uint8_t frame_type_{kQFrameTypeDefault};
    std::int8_t tailsit_enable_{kQTailsitEnableDefault};
    std::int8_t tilt_enable_{kQTiltEnableDefault};
    std::int32_t options_{kQOptionsDefault};
    bool initialised_{false};
    bool motors_inited_{false};
    MotorsKind motors_kind_{MotorsKind::kMatrix};
    MotorsInitParams motors_init_{};
    bool attitude_control_inited_{false};
    bool pos_control_inited_{false};
    bool weathervane_inited_{false};
    std::int32_t lean_angle_max_cd_{0};
    PosControlState poscontrol_{};
    bool guided_wait_takeoff_{false};
    bool guided_wait_takeoff_on_mode_enter_{false};
};

}  // namespace fwcpp::quadplane
