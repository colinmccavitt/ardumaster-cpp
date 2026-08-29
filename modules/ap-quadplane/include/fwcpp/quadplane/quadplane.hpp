#pragma once

#include <cstdint>
#include <optional>

#include <fwcpp/quadplane/quadplane_air_mode.hpp>
#include <fwcpp/quadplane/quadplane_defaults.hpp>
#include <fwcpp/quadplane/quadplane_mode_predicates.hpp>
#include <fwcpp/quadplane/quadplane_vtol_position_controller.hpp>
#include <fwcpp/quadplane/quadplane_frame.hpp>
#include <fwcpp/quadplane/quadplane_motors_init.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_approach.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>
#include <fwcpp/quadplane/quadplane_setup_channels.hpp>
#include <fwcpp/quadplane/quadplane_setup_navigators.hpp>
#include <fwcpp/quadplane/quadplane_update.hpp>
#include <fwcpp/quadplane_transition/transition_fsm.hpp>

namespace fwcpp::quadplane {

struct QuadPlaneSetupInputs {
    bool soft_armed{false};
    std::uint32_t available_memory_bytes{kSetupMinMemoryBytes};
    AhrsViewCreateInputs ahrs_view{};
    SetupChannelsSink channels_sink{};
    WpNavSetupInputs wp_nav{};
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

    [[nodiscard]] const SetupChannelsSink& setup_channels() const { return setup_channels_; }
    [[nodiscard]] const AhrsViewSetup& ahrs_view() const { return ahrs_view_; }
    [[nodiscard]] bool ahrs_view_inited() const { return ahrs_view_inited_; }

    [[nodiscard]] bool wp_nav_inited() const { return wp_nav_inited_; }
    [[nodiscard]] bool loiter_nav_inited() const { return loiter_nav_inited_; }
    [[nodiscard]] const fwcpp::wpnav::WpNav& wp_nav() const { return wp_nav_; }
    [[nodiscard]] const LoiterNavStub& loiter_nav() const { return loiter_nav_; }

    bool setup(const QuadPlaneSetupInputs& inputs = {}) {
        if (initialised_) return true;
        if (inputs.soft_armed) return false;
        if (inputs.available_memory_bytes < kSetupMinMemoryBytes) return false;
        if (!enabled()) return false;
        const auto sel = classify_frame(frame_class_, tailsit_enable_, tilt_enable_);
        if (!sel) return false;

        setup_channels_ = inputs.channels_sink;
        wire_setup_channels(frame_class_, setup_channels_);

        AhrsViewCreateInputs ahrs_in = inputs.ahrs_view;
        ahrs_in.tailsit_enable = tailsit_enable_;
        ahrs_view_ = make_ahrs_view_setup(ahrs_in);
        ahrs_view_inited_ = ahrs_view_.created;

        motors_kind_ = sel->motors_kind;
        motors_init_ = make_motors_init_params(frame_class_, frame_type_);
        motors_inited_ = true;
        attitude_control_inited_ = true;
        pos_control_inited_ = true;

        const NavigatorDeps nav_deps{
            .ahrs_view_created = ahrs_view_.created,
            .attitude_control_inited = attitude_control_inited_,
            .pos_control_inited = pos_control_inited_,
        };
        const auto nav = wire_setup_navigators(nav_deps, inputs.wp_nav);
        if (!nav.ok) {
            return false;
        }
        wp_nav_ = nav.wp_nav;
        loiter_nav_ = nav.loiter_nav;
        wp_nav_inited_ = nav.wp_and_spline_inited;
        loiter_nav_inited_ = nav.loiter_nav.created;

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

    void set_assisted_flight(bool v) { assisted_flight_ = v; }
    void set_air_mode(AirMode v) { air_mode_ = v; }
    [[nodiscard]] AirMode air_mode() const { return air_mode_; }
    [[nodiscard]] bool air_mode_active() const {
        return fwcpp::quadplane::air_mode_active(air_mode_, assisted_flight_);
    }
    void apply_air_mode_aux(AirModeAuxPos pos) {
        fwcpp::quadplane::apply_air_mode_aux(pos, air_mode_, throttle_wait_);
    }
    void apply_armdisarm_airmode_latch(bool armed) {
        fwcpp::quadplane::apply_armdisarm_airmode_latch(armed, air_mode_, throttle_wait_);
    }
    [[nodiscard]] bool throttle_wait() const { return throttle_wait_; }
    void set_throttle_wait(bool v) { throttle_wait_ = v; }

    [[nodiscard]] bool in_vtol_mode(const InVtolModeInputs& view) const {
        InVtolModeInputs in = view;
        in.available = available();
        in.pos_state = poscontrol_.state;
        return compute_in_vtol_mode(in);
    }

    VtolPositionControllerTick vtol_position_controller(const VtolPositionControllerInputs& in) {
        InVtolModeInputs vtol_in = in.in_vtol;
        vtol_in.available = available();
        vtol_in.pos_state = poscontrol_.state;
        VtolPositionControllerInputs wired = in;
        wired.in_vtol = vtol_in;
        return run_vtol_position_controller(poscontrol_, poscontrol_land_, vtol_pos_sink_, wired);
    }
    [[nodiscard]] const PosControlSetStateSink& vtol_pos_sink() const { return vtol_pos_sink_; }


    [[nodiscard]] bool assisted_flight() const { return assisted_flight_; }

    [[nodiscard]] const MotorsOutputState& motors_output_state() const { return motors_output_state_; }

    MotorsOutputTick motors_output(MotorsOutputView view) {
        return run_motors_output(view, options_, assisted_flight_, motors_output_state_);
    }

    void set_approach_distance_m(float v) { approach_distance_m_ = v; }
    [[nodiscard]] float approach_distance_m() const { return approach_distance_m_; }
    [[nodiscard]] const PosControlLandStub& poscontrol_land() const { return poscontrol_land_; }
    [[nodiscard]] const PosControlTransitionPrep& last_transition_prep() const {
        return last_transition_prep_;
    }
    PoscontrolApproachInitResult poscontrol_init_approach(const ApproachInitView& view,
                                                          const PosControlSetStateInputs& set_state) {
        PoscontrolApproachInitInputs in{
            .options = options_,
            .approach_distance_m = approach_distance_m_,
            .view = view,
            .set_state = set_state,
        };
        const auto result = poscontrol_init_approach_prep(poscontrol_, poscontrol_land_, in);
        last_transition_prep_ = result.transition_prep;
        last_set_state_sink_ = result.set_state_sink;
        return result;
    }
    [[nodiscard]] const PosControlSetStateSink& last_set_state_sink() const {
        return last_set_state_sink_;
    }
    [[nodiscard]] const fwcpp::quadplane_transition::SltTransition& slt_transition() const {
        return slt_transition_;
    }
    fwcpp::quadplane_transition::SltTransition& slt_transition_mut() { return slt_transition_; }

    QuadPlaneUpdateTick update(const QuadPlaneUpdateView& view) {
        return run_quadplane_update(slt_transition_, available(), assisted_flight_, options_, view);
    }

    void mode_enter() {
        if (available()) {
            lean_angle_max_cd_ = 0;
        }
        poscontrol_.reset_on_mode_enter();
        guided_wait_takeoff_on_mode_enter_ = guided_wait_takeoff_;
        guided_wait_takeoff_ = false;
        last_transition_prep_ = {};
        last_set_state_sink_ = {};
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
    SetupChannelsSink setup_channels_{};
    AhrsViewSetup ahrs_view_{};
    bool ahrs_view_inited_{false};
    bool attitude_control_inited_{false};
    bool pos_control_inited_{false};
    bool weathervane_inited_{false};
    bool wp_nav_inited_{false};
    bool loiter_nav_inited_{false};
    fwcpp::wpnav::WpNav wp_nav_{};
    LoiterNavStub loiter_nav_{};
    bool assisted_flight_{false};
    AirMode air_mode_{AirMode::kOff};
    bool throttle_wait_{false};
    PosControlSetStateSink vtol_pos_sink_{};
    MotorsOutputState motors_output_state_{};
    std::int32_t lean_angle_max_cd_{0};
    PosControlState poscontrol_{};
    bool guided_wait_takeoff_{false};
    bool guided_wait_takeoff_on_mode_enter_{false};
    float approach_distance_m_{kApproachDistanceDefaultM};
    PosControlLandStub poscontrol_land_{};
    PosControlTransitionPrep last_transition_prep_{};
    PosControlSetStateSink last_set_state_sink_{};
    fwcpp::quadplane_transition::SltTransition slt_transition_{
        fwcpp::quadplane_transition::SltTransition::with_defaults()};
};

}  // namespace fwcpp::quadplane
