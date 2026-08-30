#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>

#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_leftover.hpp>
#include <fwcpp/copter/mode_reason.hpp>

using Catch::Approx;

using fwcpp::copter::FlightModeContext;
using fwcpp::copter::FlightModeTable;
using fwcpp::copter::Mode;
using fwcpp::copter::ModeAuto;
using fwcpp::copter::ModePortStatus;
using fwcpp::copter::ModeReason;
using fwcpp::copter::SetModeInputs;
using fwcpp::copter::mode_completeness_has;
using fwcpp::copter::mode_completeness_size;
using fwcpp::copter::mode_from_mode_num;
using fwcpp::copter::mode_on_main_count;
using fwcpp::copter::mode_out_of_scope_count;
using fwcpp::copter::mode_this_slice_count;
using fwcpp::copter::remaining_count;
using fwcpp::copter::gcs_mode_enabled;
using fwcpp::copter::set_mode;

namespace {

class TestPosMode : public Mode {
public:
    [[nodiscard]] Number mode_number() const override { return Number::LOITER; }
    [[nodiscard]] bool init(bool /*ignore_checks*/) override { return true; }
    void run() override {}
    [[nodiscard]] bool requires_position() const override { return true; }
    [[nodiscard]] bool has_manual_throttle() const override { return false; }
};

class TestManualMode : public Mode {
public:
    [[nodiscard]] Number mode_number() const override { return Number::ACRO; }
    [[nodiscard]] bool init(bool /*ignore_checks*/) override { return true; }
    void run() override {}
    [[nodiscard]] bool requires_position() const override { return false; }
    [[nodiscard]] bool has_manual_throttle() const override { return true; }
};

struct Fixture {
    FlightModeTable table;
    FlightModeContext ctx;

    Fixture() {
        ctx.current = &table.stabilize;
        ctx.reason = ModeReason::INITIALISED;
    }
};

void require_submode_runs(const ModeAuto& m, bool takeoff, bool wp, bool land, bool rtl, bool circle,
                          bool nav_guided, bool loiter, bool loiter_to_alt, bool nav_att) {
    REQUIRE(m.takeoff_run == takeoff);
    REQUIRE(m.wp_run == wp);
    REQUIRE(m.land_run == land);
    REQUIRE(m.rtl_run == rtl);
    REQUIRE(m.circle_run == circle);
    REQUIRE(m.nav_guided_run == nav_guided);
    REQUIRE(m.loiter_run == loiter);
    REQUIRE(m.loiter_to_alt_run == loiter_to_alt);
    REQUIRE(m.nav_attitude_time_run == nav_att);
}

void require_wp_run_flying(const ModeAuto& m) {
    REQUIRE(m.wp_run);
    REQUIRE(m.desired_spool_unlimited);
    REQUIRE(m.update_wpnav);
    REQUIRE(m.terrain_failsafe_status);
    REQUIRE(m.pos_D_update);
    REQUIRE(m.input_thrust_vector_heading);
    REQUIRE_FALSE(m.make_safe_ground_handling);
    REQUIRE_FALSE(m.land_run_normal_or_precland);
    REQUIRE_FALSE(m.leftover_mode_rtl_run);
    REQUIRE_FALSE(m.leftover_circle_nav_update);
    REQUIRE_FALSE(m.leftover_reached_wp_destination_ne);
    REQUIRE_FALSE(m.leftover_loiter_to_alt_rest);
}

void require_no_wp_run_leftovers(const ModeAuto& m) {
    REQUIRE_FALSE(m.make_safe_ground_handling);
    REQUIRE_FALSE(m.desired_spool_unlimited);
    REQUIRE_FALSE(m.update_wpnav);
    REQUIRE_FALSE(m.terrain_failsafe_status);
    REQUIRE_FALSE(m.pos_D_update);
    REQUIRE_FALSE(m.input_thrust_vector_heading);
    REQUIRE_FALSE(m.land_run_normal_or_precland);
    REQUIRE_FALSE(m.leftover_mode_rtl_run);
    REQUIRE_FALSE(m.leftover_circle_nav_update);
    REQUIRE_FALSE(m.leftover_reached_wp_destination_ne);
    REQUIRE_FALSE(m.leftover_loiter_to_alt_rest);
}

void require_land_run_flying(const ModeAuto& m) {
    REQUIRE(m.land_run);
    REQUIRE(m.desired_spool_unlimited);
    REQUIRE(m.land_run_normal_or_precland);
    REQUIRE_FALSE(m.make_safe_ground_handling);
    REQUIRE_FALSE(m.leftover_mode_rtl_run);
    REQUIRE_FALSE(m.leftover_circle_nav_update);
    REQUIRE_FALSE(m.leftover_reached_wp_destination_ne);
    REQUIRE_FALSE(m.leftover_loiter_to_alt_rest);
}

void require_loiter_run_flying(const ModeAuto& m) {
    REQUIRE(m.loiter_run);
    REQUIRE(m.desired_spool_unlimited);
    REQUIRE(m.update_wpnav);
    REQUIRE(m.terrain_failsafe_status);
    REQUIRE(m.pos_D_update);
    REQUIRE(m.input_thrust_vector_heading);
    REQUIRE_FALSE(m.make_safe_ground_handling);
    REQUIRE_FALSE(m.wp_run);
    REQUIRE_FALSE(m.loiter_to_alt_run);
    REQUIRE_FALSE(m.land_run_normal_or_precland);
    REQUIRE_FALSE(m.leftover_mode_rtl_run);
    REQUIRE_FALSE(m.leftover_circle_nav_update);
    REQUIRE_FALSE(m.leftover_reached_wp_destination_ne);
    REQUIRE_FALSE(m.leftover_loiter_to_alt_rest);
}

void require_circle_run(const ModeAuto& m) {
    REQUIRE(m.circle_run);
    REQUIRE(m.leftover_circle_nav_update);
    REQUIRE(m.terrain_failsafe_status);
    REQUIRE(m.pos_D_update);
    REQUIRE(m.input_thrust_vector_heading);
    REQUIRE_FALSE(m.update_wpnav);
    REQUIRE_FALSE(m.desired_spool_unlimited);
    REQUIRE_FALSE(m.make_safe_ground_handling);
    REQUIRE_FALSE(m.wp_run);
    REQUIRE_FALSE(m.land_run_normal_or_precland);
    REQUIRE_FALSE(m.leftover_mode_rtl_run);
    REQUIRE_FALSE(m.leftover_reached_wp_destination_ne);
    REQUIRE_FALSE(m.leftover_loiter_to_alt_rest);
}

void require_loiter_to_alt_wp_run_reuse(const ModeAuto& m) {
    REQUIRE(m.loiter_to_alt_run);
    REQUIRE(m.leftover_reached_wp_destination_ne);
    REQUIRE(m.desired_spool_unlimited);
    REQUIRE(m.update_wpnav);
    REQUIRE(m.terrain_failsafe_status);
    REQUIRE(m.pos_D_update);
    REQUIRE(m.input_thrust_vector_heading);
    REQUIRE_FALSE(m.make_safe_ground_handling);
    REQUIRE_FALSE(m.wp_run);
    REQUIRE_FALSE(m.loiter_run);
    REQUIRE_FALSE(m.leftover_loiter_to_alt_rest);
    REQUIRE_FALSE(m.land_run_normal_or_precland);
    REQUIRE_FALSE(m.leftover_mode_rtl_run);
    REQUIRE_FALSE(m.leftover_circle_nav_update);
}

}  // namespace

TEST_CASE("Mode::Number values match mode.h including gaps", "[copter][mode]") {
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::STABILIZE) == 0);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::ACRO) == 1);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::ALT_HOLD) == 2);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::LAND) == 9);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::DRIFT) == 11);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::SPORT) == 13);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::AUTO_RTL) == 27);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::TURTLE) == 28);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::CIRCLE) == 7);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::LAND) - static_cast<std::uint8_t>(Mode::Number::CIRCLE) ==
            2);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::DRIFT) - static_cast<std::uint8_t>(Mode::Number::LAND) == 2);
    REQUIRE(static_cast<std::uint8_t>(Mode::Number::SPORT) - static_cast<std::uint8_t>(Mode::Number::DRIFT) ==
            2);
}

TEST_CASE("ModeReason values match ModeReason.h", "[copter][mode]") {
    REQUIRE(static_cast<std::uint8_t>(ModeReason::UNKNOWN) == 0);
    REQUIRE(static_cast<std::uint8_t>(ModeReason::GCS_COMMAND) == 2);
    REQUIRE(static_cast<std::uint8_t>(ModeReason::INITIALISED) == 26);
    REQUIRE(static_cast<std::uint8_t>(ModeReason::FENCE_REENABLE) == 55);
}

TEST_CASE("mode_from_mode_num returns Stabilize AltHold and AUTO", "[copter][mode]") {
    FlightModeTable table;
    REQUIRE(mode_from_mode_num(Mode::Number::STABILIZE, table) == &table.stabilize);
    REQUIRE(mode_from_mode_num(Mode::Number::ALT_HOLD, table) == &table.althold);
    REQUIRE(mode_from_mode_num(Mode::Number::AUTO, table) == &table.mode_auto);
    REQUIRE(mode_from_mode_num(Mode::Number::ACRO, table) == nullptr);
    REQUIRE(mode_from_mode_num(Mode::Number::LAND, table) == nullptr);
    REQUIRE(mode_from_mode_num(Mode::Number::AUTO_RTL, table) == nullptr);
    REQUIRE(table.stabilize.requires_position() == false);
    REQUIRE(table.stabilize.has_manual_throttle() == true);
    REQUIRE(table.stabilize.allows_entry_in_rc_failsafe() == false);
    REQUIRE(table.althold.requires_position() == false);
    REQUIRE(table.althold.has_manual_throttle() == false);
    REQUIRE(table.althold.allows_entry_in_rc_failsafe() == true);
    REQUIRE(table.mode_auto.auto_RTL == false);
    REQUIRE(table.mode_auto.mode_number() == Mode::Number::AUTO);
    REQUIRE(table.mode_auto.requires_position() == true);
    REQUIRE(table.mode_auto.has_manual_throttle() == false);
    table.mode_auto.auto_RTL = true;
    REQUIRE(table.mode_auto.mode_number() == Mode::Number::AUTO_RTL);
}

TEST_CASE("already-in-mode returns true and updates reason", "[copter][mode]") {
    Fixture f;
    const SetModeInputs in{};
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.ctx.reason == ModeReason::GCS_COMMAND);
}

TEST_CASE("unknown mode returns false", "[copter][mode]") {
    Fixture f;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::ACRO, ModeReason::RC_COMMAND, {}));
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::LAND, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.ctx.reason == ModeReason::INITIALISED);
}

TEST_CASE("AUTO_RTL fails when both mission jumps are false", "[copter][mode]") {
    Fixture f;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.table.mode_auto.auto_RTL == false);
    REQUIRE(f.ctx.force_resume == false);
}

TEST_CASE("AUTO_RTL succeeds when closest mission jump only", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.jump_to_closest_mission_leg = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE(f.table.mode_auto.auto_RTL == true);
    REQUIRE(f.ctx.current->mode_number() == Mode::Number::AUTO_RTL);
    REQUIRE(f.ctx.force_resume == true);
}

TEST_CASE("AUTO_RTL succeeds when landing sequence jump only", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.jump_to_landing_sequence = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE(f.table.mode_auto.auto_RTL == true);
    REQUIRE(f.ctx.current->mode_number() == Mode::Number::AUTO_RTL);
    REQUIRE(f.ctx.force_resume == true);
}

TEST_CASE("AUTO_RTL succeeds when both mission jumps are true", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.jump_to_closest_mission_leg = true;
    in.jump_to_landing_sequence = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE(f.table.mode_auto.auto_RTL == true);
    REQUIRE(f.ctx.reason == ModeReason::GCS_COMMAND);
}

TEST_CASE("GCS_COMMAND blocked before AUTO_RTL special case", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.gcs_mode_enabled = false;
    in.jump_to_closest_mission_leg = true;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.table.mode_auto.auto_RTL == false);
}

TEST_CASE("AUTO_RTL when already in AUTO sets auto_RTL", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE(f.table.mode_auto.mode_number() == Mode::Number::AUTO);

    SetModeInputs in{};
    in.jump_to_landing_sequence = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE(f.table.mode_auto.auto_RTL == true);
    REQUIRE(f.ctx.current->mode_number() == Mode::Number::AUTO_RTL);
    REQUIRE(f.ctx.force_resume == true);
}

TEST_CASE("AUTO_RTL fails when set_mode AUTO fails", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = true;
    in.position_ok = false;
    in.jump_to_closest_mission_leg = true;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.table.mode_auto.auto_RTL == false);
    REQUIRE(f.ctx.force_resume == false);
}

TEST_CASE("GCS_COMMAND blocked when gcs_mode_enabled is false", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.gcs_mode_enabled = false;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND, in));
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
}

TEST_CASE("FLTMODE_GCSBLOCK mask 0 allows GCS ALT_HOLD and STABILIZE", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    REQUIRE(in.fltmode_gcsblock == 0);
    REQUIRE(in.gcs_mode_enabled);
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
}

TEST_CASE("FLTMODE_GCSBLOCK bit 2 blocks GCS ALT_HOLD; RC_COMMAND succeeds", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.fltmode_gcsblock = 1U << 2;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE_FALSE(set_mode(f.ctx, f.table.althold, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
}

TEST_CASE("FLTMODE_GCSBLOCK bit 0 blocks GCS STABILIZE", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, {}));
    SetModeInputs in{};
    in.fltmode_gcsblock = 1U << 0;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
}

TEST_CASE("FLTMODE_GCSBLOCK AUTO_RTL bit blocks GCS even with jump flags", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.jump_to_closest_mission_leg = true;
    in.jump_to_landing_sequence = true;
    // Upstream mode_list: AUTO_RTL is index 22 (AUTOROTATE is 21).
    in.fltmode_gcsblock = 1U << 22;
    REQUIRE_FALSE(gcs_mode_enabled(Mode::Number::AUTO_RTL, in.fltmode_gcsblock));
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.table.mode_auto.auto_RTL == false);
}

TEST_CASE("LAND and RTL are never blocked by FLTMODE_GCSBLOCK", "[copter][mode]") {
    REQUIRE(gcs_mode_enabled(Mode::Number::LAND, 0xFFFFFFFFu));
    REQUIRE(gcs_mode_enabled(Mode::Number::RTL, 0xFFFFFFFFu));
}

TEST_CASE("disarmed ignore_checks allows requires_position when !position_ok", "[copter][mode]") {
    Fixture f;
    TestPosMode pos;
    SetModeInputs in{};
    in.armed = false;
    in.position_ok = false;
    REQUIRE(set_mode(f.ctx, pos, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &pos);
    REQUIRE(f.ctx.current->requires_position());
}

TEST_CASE("armed + requires_position + !position_ok is false", "[copter][mode]") {
    Fixture f;
    TestPosMode pos;
    SetModeInputs in{};
    in.armed = true;
    in.position_ok = false;
    REQUIRE_FALSE(set_mode(f.ctx, pos, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
}

TEST_CASE("throttle too high blocks manual-throttle entry", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, {}));

    SetModeInputs in{};
    in.armed = true;
    in.land_complete = true;
    in.pilot_desired_throttle = 0.40f;
    in.non_takeoff_throttle = 0.25f;
    REQUIRE_FALSE(in.is_drift);
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
}

TEST_CASE("is_drift forces user_throttle on a non-manual next mode", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, {}));

    TestPosMode pos;
    REQUIRE_FALSE(pos.has_manual_throttle());

    SetModeInputs in{};
    in.armed = true;
    in.land_complete = true;
    in.pilot_desired_throttle = 0.40f;
    in.non_takeoff_throttle = 0.25f;
    in.is_drift = true;
    REQUIRE_FALSE(set_mode(f.ctx, pos, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
}

TEST_CASE("is_drift false does not treat non-manual stub as user_throttle", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, {}));

    TestPosMode pos;
    REQUIRE_FALSE(pos.has_manual_throttle());

    SetModeInputs in{};
    in.armed = true;
    in.land_complete = true;
    in.pilot_desired_throttle = 0.40f;
    in.non_takeoff_throttle = 0.25f;
    REQUIRE_FALSE(in.is_drift);
    REQUIRE(set_mode(f.ctx, pos, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &pos);
}

TEST_CASE("throttle at or below non_takeoff_throttle succeeds", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, {}));

    SetModeInputs in{};
    in.armed = true;
    in.land_complete = true;
    in.pilot_desired_throttle = 0.25f;
    in.non_takeoff_throttle = 0.25f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
}

TEST_CASE("alt estimate required when leaving manual throttle", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = true;
    in.ekf_alt_ok = false;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);

    in.ekf_alt_ok = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
}

TEST_CASE("rc_failsafe blocks Stabilize even when disarmed", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, {}));

    SetModeInputs in{};
    in.armed = false;
    in.rc_failsafe = true;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);

    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RADIO_FAILSAFE, in));
    REQUIRE(f.ctx.reason == ModeReason::RADIO_FAILSAFE);
}

TEST_CASE("rc_failsafe allows AltHold (default allows_entry true)", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = false;
    in.rc_failsafe = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RADIO_FAILSAFE, in));
    REQUIRE(f.ctx.current == &f.table.althold);
}

TEST_CASE("manual-to-auto sets accel throttle I", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = true;
    in.land_complete = false;
    in.pilot_desired_throttle = 0.70f;
    in.throttle_hover = 0.50f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE(f.ctx.accel_throttle_I_set);
    REQUIRE(f.ctx.accel_throttle_I == Approx(-(0.70f - 0.50f)));
}

TEST_CASE("accel throttle I skipped when disarmed", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = false;
    in.land_complete = false;
    in.pilot_desired_throttle = 0.70f;
    in.throttle_hover = 0.50f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE_FALSE(f.ctx.accel_throttle_I_set);
}

TEST_CASE("accel throttle I skipped when land_complete", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = true;
    in.land_complete = true;
    in.pilot_desired_throttle = 0.70f;
    in.throttle_hover = 0.50f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE_FALSE(f.ctx.accel_throttle_I_set);
}

TEST_CASE("accel throttle I skipped when staying in manual", "[copter][mode]") {
    Fixture f;
    TestManualMode acro;
    SetModeInputs in{};
    in.armed = true;
    in.land_complete = false;
    in.pilot_desired_throttle = 0.70f;
    in.throttle_hover = 0.50f;
    REQUIRE(set_mode(f.ctx, acro, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &acro);
    REQUIRE_FALSE(f.ctx.accel_throttle_I_set);
}

TEST_CASE("accel throttle I skipped when already auto", "[copter][mode]") {
    Fixture f;
    f.ctx.current = &f.table.althold;
    SetModeInputs in{};
    in.armed = true;
    in.land_complete = false;
    in.pilot_desired_throttle = 0.70f;
    in.throttle_hover = 0.50f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE_FALSE(f.ctx.accel_throttle_I_set);
}

TEST_CASE("Stabilize to AltHold records Write_Mode and notify", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE(f.ctx.write_mode);
    REQUIRE(f.ctx.written_mode_number == Mode::Number::ALT_HOLD);
    REQUIRE(f.ctx.written_reason == ModeReason::RC_COMMAND);
    REQUIRE(f.ctx.notify_flight_mode);
    REQUIRE(f.ctx.notify_flight_mode_number == Mode::Number::ALT_HOLD);
    REQUIRE_FALSE(f.ctx.notify_autopilot_mode);
    REQUIRE_FALSE(f.ctx.gcs_heartbeat);
}

TEST_CASE("failed set_mode does not Write_Mode", "[copter][mode]") {
    Fixture f;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::ACRO, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE_FALSE(f.ctx.write_mode);
    REQUIRE_FALSE(f.ctx.notify_flight_mode);
}

TEST_CASE("already-in-mode does not Write_Mode", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::GCS_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.ctx.reason == ModeReason::GCS_COMMAND);
    REQUIRE_FALSE(f.ctx.write_mode);
    REQUIRE_FALSE(f.ctx.notify_flight_mode);
}

TEST_CASE("AUTO_RTL success writes AUTO_RTL after auto_RTL is set", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.jump_to_closest_mission_leg = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.table.mode_auto.auto_RTL);
    REQUIRE(f.ctx.current->mode_number() == Mode::Number::AUTO_RTL);
    REQUIRE(f.ctx.write_mode);
    REQUIRE(f.ctx.written_mode_number == Mode::Number::AUTO_RTL);
    REQUIRE(f.ctx.written_reason == ModeReason::GCS_COMMAND);
}

TEST_CASE("fence recovery blocks set_mode when all gate conditions hold", "[copter][mode]") {
    Fixture f;
    f.ctx.reason = ModeReason::FENCE_BREACHED;
    SetModeInputs in{};
    in.armed = true;
    in.land_complete = false;
    in.fence_enabled = true;
    in.fence_disable_mode_change = true;
    in.fence_breaches = true;
    in.fence_present = true;
    in.fence_action_report_only = false;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.ctx.reason == ModeReason::FENCE_BREACHED);
    REQUIRE_FALSE(f.ctx.fence_manual_recovery_start);
    REQUIRE_FALSE(f.ctx.write_mode);
}

TEST_CASE("fence recovery allows when any gate condition is false", "[copter][mode]") {
    auto blocking = []() {
        SetModeInputs in{};
        in.armed = true;
        in.land_complete = false;
        in.fence_enabled = true;
        in.fence_disable_mode_change = true;
        in.fence_breaches = true;
        return in;
    };

    {
        Fixture f;
        f.ctx.reason = ModeReason::FENCE_BREACHED;
        SetModeInputs in = blocking();
        in.land_complete = true;
        REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
        REQUIRE(f.ctx.current == &f.table.althold);
    }
    {
        Fixture f;
        SetModeInputs in = blocking();
        REQUIRE(f.ctx.reason != ModeReason::FENCE_BREACHED);
        REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::FENCE_BREACHED, in));
        REQUIRE(f.ctx.current == &f.table.althold);
    }
    {
        Fixture f;
        f.ctx.reason = ModeReason::FENCE_BREACHED;
        SetModeInputs in = blocking();
        in.armed = false;
        REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
        REQUIRE(f.ctx.current == &f.table.althold);
    }
    {
        Fixture f;
        f.ctx.reason = ModeReason::FENCE_BREACHED;
        SetModeInputs in = blocking();
        in.fence_enabled = false;
        REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
        REQUIRE(f.ctx.current == &f.table.althold);
    }
    {
        Fixture f;
        f.ctx.reason = ModeReason::FENCE_BREACHED;
        SetModeInputs in = blocking();
        in.fence_disable_mode_change = false;
        REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
        REQUIRE(f.ctx.current == &f.table.althold);
    }
    {
        Fixture f;
        f.ctx.reason = ModeReason::FENCE_BREACHED;
        SetModeInputs in = blocking();
        in.fence_breaches = false;
        REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
        REQUIRE(f.ctx.current == &f.table.althold);
    }
}

TEST_CASE("successful enter records fence_manual_recovery_start", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.fence_present = true;
    in.fence_action_report_only = false;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE(f.ctx.fence_manual_recovery_start);
}

TEST_CASE("fence_present false does not record fence_manual_recovery_start", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.fence_present = false;
    in.fence_action_report_only = false;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE_FALSE(f.ctx.fence_manual_recovery_start);
}

TEST_CASE("fence_action_report_only does not record fence_manual_recovery_start", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.fence_present = true;
    in.fence_action_report_only = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE_FALSE(f.ctx.fence_manual_recovery_start);
}

TEST_CASE("AUTO init fails when armed landed without takeoff cmd", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = true;
    in.land_complete = true;
    in.mission_present = true;
    in.starts_with_takeoff = false;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE_FALSE(f.table.mode_auto.waiting_to_start);
    REQUIRE_FALSE(f.ctx.write_mode);
}

TEST_CASE("AUTO init succeeds when landed with takeoff cmd", "[copter][mode]") {
    Fixture f;
    f.table.mode_auto.auto_RTL = true;
    SetModeInputs in{};
    in.armed = true;
    in.land_complete = true;
    in.starts_with_takeoff = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE(f.table.mode_auto.waiting_to_start);
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    REQUIRE(f.table.mode_auto.submode_loiter);
    REQUIRE(f.table.mode_auto.wp_spline_init);
    REQUIRE(f.table.mode_auto.speed_override_cleared);
    REQUIRE(f.table.mode_auto.guided_limit_clear);
    REQUIRE(f.table.mode_auto.land_repo_active_cleared);
    REQUIRE_FALSE(f.table.mode_auto.auto_yaw_roi_to_hold);
}

TEST_CASE("AUTO init fails when no mission and armed", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = true;
    in.land_complete = false;
    in.mission_present = false;
    REQUIRE_FALSE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE_FALSE(f.table.mode_auto.waiting_to_start);
}

TEST_CASE("AUTO init succeeds without mission when ignore_checks", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.armed = false;
    in.mission_present = false;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE(f.table.mode_auto.waiting_to_start);
}

TEST_CASE("AUTO init ROI leftover sets HOLD", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.yaw_mode_is_roi = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, in));
    REQUIRE(f.table.mode_auto.auto_yaw_roi_to_hold);
    REQUIRE(f.table.mode_auto.waiting_to_start);
}

TEST_CASE("leave AUTO with mission running records mission_stop", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.mission_running = true;
    f.table.mode_auto.auto_RTL = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE(f.table.mode_auto.mission_stop);
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    REQUIRE_FALSE(f.table.mode_auto.camera_mount_default);
}

TEST_CASE("leave AUTO without mission running clears auto_RTL", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.mission_running = false;
    f.table.mode_auto.auto_RTL = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE_FALSE(f.table.mode_auto.mission_stop);
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    REQUIRE_FALSE(f.table.mode_auto.camera_mount_default);
}

TEST_CASE("AUTO_RTL then leave clears auto_RTL", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.jump_to_closest_mission_leg = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.table.mode_auto.auto_RTL);
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    REQUIRE_FALSE(f.table.mode_auto.mission_stop);
}

TEST_CASE("Stabilize and AltHold exit are no-ops", "[copter][mode]") {
    Fixture f;
    REQUIRE_FALSE(f.table.mode_auto.mission_stop);
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.althold);
    REQUIRE_FALSE(f.table.mode_auto.mission_stop);
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::STABILIZE, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.ctx.current == &f.table.stabilize);
    REQUIRE_FALSE(f.table.mode_auto.mission_stop);
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
}

TEST_CASE("AUTO_RTL jump path still calls AUTO init", "[copter][mode]") {
    Fixture f;
    SetModeInputs in{};
    in.jump_to_closest_mission_leg = true;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO_RTL, ModeReason::GCS_COMMAND, in));
    REQUIRE(f.ctx.current == &f.table.mode_auto);
    REQUIRE(f.table.mode_auto.auto_RTL);
    REQUIRE(f.table.mode_auto.waiting_to_start);
    REQUIRE(f.table.mode_auto.submode_loiter);
}

TEST_CASE("ModeAuto run with origin starts mission leftover", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.table.mode_auto.waiting_to_start);
    REQUIRE_FALSE(f.table.mode_auto.has_origin);
    REQUIRE_FALSE(f.table.mode_auto.start_or_resume);
    REQUIRE_FALSE(f.table.mode_auto.mis_change_check_init);
    f.table.mode_auto.has_origin = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.start_or_resume);
    REQUIRE_FALSE(f.table.mode_auto.waiting_to_start);
    REQUIRE(f.table.mode_auto.mis_change_check_init);
    REQUIRE_FALSE(f.table.mode_auto.mission_update);
    REQUIRE_FALSE(f.table.mode_auto.restart_nav_cmd);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_restarted);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_failed);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
}

TEST_CASE("ModeAuto run without origin stays waiting", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.table.mode_auto.waiting_to_start);
    REQUIRE_FALSE(f.table.mode_auto.has_origin);
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.waiting_to_start);
    REQUIRE_FALSE(f.table.mode_auto.start_or_resume);
    REQUIRE_FALSE(f.table.mode_auto.mis_change_check_init);
    REQUIRE_FALSE(f.table.mode_auto.mission_update);
    REQUIRE_FALSE(f.table.mode_auto.restart_nav_cmd);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
}

TEST_CASE("ModeAuto run when not waiting records mission_update", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.table.mode_auto.waiting_to_start);
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.has_origin = true;
    f.table.mode_auto.run();
    REQUIRE_FALSE(f.table.mode_auto.waiting_to_start);
    REQUIRE_FALSE(f.table.mode_auto.start_or_resume);
    REQUIRE_FALSE(f.table.mode_auto.mis_change_check_init);
    REQUIRE(f.table.mode_auto.mission_update);
    REQUIRE_FALSE(f.table.mode_auto.restart_nav_cmd);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_restarted);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_failed);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
}

TEST_CASE("ModeAuto run else-path restart leftover when changed running wp ok", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.mission_changed = true;
    f.table.mode_auto.mission_running = true;
    f.table.mode_auto.submode_is_wp = true;
    f.table.mode_auto.restart_nav_ok = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.restart_nav_cmd);
    REQUIRE(f.table.mode_auto.gcs_mission_changed_restarted);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_failed);
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
}

TEST_CASE("ModeAuto run else-path restart leftover when changed running wp fail", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.mission_changed = true;
    f.table.mode_auto.mission_running = true;
    f.table.mode_auto.submode_is_wp = true;
    f.table.mode_auto.restart_nav_ok = false;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.restart_nav_cmd);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_restarted);
    REQUIRE(f.table.mode_auto.gcs_mission_changed_failed);
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
}

TEST_CASE("ModeAuto run else-path no restart when not running", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.mission_changed = true;
    f.table.mode_auto.mission_running = false;
    f.table.mode_auto.submode_is_wp = true;
    f.table.mode_auto.restart_nav_ok = true;
    f.table.mode_auto.run();
    REQUIRE_FALSE(f.table.mode_auto.restart_nav_cmd);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_restarted);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_failed);
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
}

TEST_CASE("ModeAuto run else-path no restart when not wp", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.mission_changed = true;
    f.table.mode_auto.mission_running = true;
    f.table.mode_auto.submode_is_wp = false;
    f.table.mode_auto.restart_nav_ok = true;
    f.table.mode_auto.run();
    REQUIRE_FALSE(f.table.mode_auto.restart_nav_cmd);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_restarted);
    REQUIRE_FALSE(f.table.mode_auto.gcs_mission_changed_failed);
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
}

TEST_CASE("ModeAuto SubMode values match Copter-4.7.0 declaration order", "[copter][mode]") {
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::TAKEOFF) == 0);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::WP) == 1);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::LAND) == 2);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::RTL) == 3);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::CIRCLE_MOVE_TO_EDGE) == 4);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::CIRCLE) == 5);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::NAVGUIDED) == 6);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::LOITER) == 7);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::LOITER_TO_ALT) == 8);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::NAV_SCRIPT_TIME) == 9);
    REQUIRE(static_cast<std::uint8_t>(ModeAuto::SubMode::NAV_ATTITUDE_TIME) == 10);
}

TEST_CASE("ModeAuto run SubMode TAKEOFF leftover takeoff_run only", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::TAKEOFF;
    REQUIRE_FALSE(f.table.mode_auto.allow_takeoff_without_raising_throttle);
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, true, false, false, false, false, false, false, false, false);
    REQUIRE(f.table.mode_auto.takeoff_run);
    REQUIRE(f.table.mode_auto.auto_takeoff_run);
    REQUIRE_FALSE(f.table.mode_auto.set_auto_armed);
    require_no_wp_run_leftovers(f.table.mode_auto);
}

TEST_CASE("ModeAuto takeoff_run leftover sets auto_armed when option enabled", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::TAKEOFF;
    f.table.mode_auto.allow_takeoff_without_raising_throttle = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.takeoff_run);
    REQUIRE(f.table.mode_auto.auto_takeoff_run);
    REQUIRE(f.table.mode_auto.set_auto_armed);
    require_no_wp_run_leftovers(f.table.mode_auto);
}

TEST_CASE("ModeAuto run SubMode WP leftover wp_run flying", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::WP;
    REQUIRE_FALSE(f.table.mode_auto.disarmed_or_landed);
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, true, false, false, false, false, false, false, false);
    require_wp_run_flying(f.table.mode_auto);
}

TEST_CASE("ModeAuto run SubMode WP leftover wp_run when disarmed_or_landed", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::WP;
    f.table.mode_auto.disarmed_or_landed = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.wp_run);
    REQUIRE(f.table.mode_auto.make_safe_ground_handling);
    REQUIRE_FALSE(f.table.mode_auto.desired_spool_unlimited);
    REQUIRE_FALSE(f.table.mode_auto.update_wpnav);
    REQUIRE_FALSE(f.table.mode_auto.terrain_failsafe_status);
    REQUIRE_FALSE(f.table.mode_auto.pos_D_update);
    REQUIRE_FALSE(f.table.mode_auto.input_thrust_vector_heading);
    REQUIRE_FALSE(f.table.mode_auto.land_run_normal_or_precland);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_run);
    REQUIRE_FALSE(f.table.mode_auto.leftover_circle_nav_update);
}

TEST_CASE("ModeAuto run SubMode CIRCLE_MOVE_TO_EDGE leftover wp_run flying", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::CIRCLE_MOVE_TO_EDGE;
    REQUIRE_FALSE(f.table.mode_auto.disarmed_or_landed);
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, true, false, false, false, false, false, false, false);
    require_wp_run_flying(f.table.mode_auto);
    REQUIRE(f.table.mode_auto.update_wpnav);
    REQUIRE_FALSE(f.table.mode_auto.leftover_circle_nav_update);
    REQUIRE_FALSE(f.table.mode_auto.circle_run);
}

TEST_CASE("ModeAuto run SubMode LAND leftover land_run flying", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::LAND;
    REQUIRE_FALSE(f.table.mode_auto.disarmed_or_landed);
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, false, true, false, false, false, false, false, false);
    require_land_run_flying(f.table.mode_auto);
}

TEST_CASE("ModeAuto run SubMode LAND leftover land_run when disarmed_or_landed", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::LAND;
    f.table.mode_auto.disarmed_or_landed = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.land_run);
    REQUIRE(f.table.mode_auto.make_safe_ground_handling);
    REQUIRE_FALSE(f.table.mode_auto.desired_spool_unlimited);
    REQUIRE_FALSE(f.table.mode_auto.land_run_normal_or_precland);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_run);
    REQUIRE_FALSE(f.table.mode_auto.leftover_circle_nav_update);
}

TEST_CASE("ModeAuto run SubMode RTL leftover rtl_run", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::RTL;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, false, false, true, false, false, false, false, false);
    REQUIRE(f.table.mode_auto.rtl_run);
    REQUIRE(f.table.mode_auto.leftover_mode_rtl_run);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_disarm_on_land);
    REQUIRE_FALSE(f.table.mode_auto.land_run);
    REQUIRE_FALSE(f.table.mode_auto.land_run_normal_or_precland);
    REQUIRE_FALSE(f.table.mode_auto.leftover_circle_nav_update);
}

TEST_CASE("ModeAuto run SubMode CIRCLE leftover circle_run", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::CIRCLE;
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, false, false, true, false, false, false, false);
    require_circle_run(f.table.mode_auto);
}

TEST_CASE("ModeAuto run SubMode LOITER leftover loiter_run flying", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::LOITER;
    f.table.mode_auto.allow_takeoff_without_raising_throttle = true;
    REQUIRE_FALSE(f.table.mode_auto.disarmed_or_landed);
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
    require_loiter_run_flying(f.table.mode_auto);
    REQUIRE_FALSE(f.table.mode_auto.auto_takeoff_run);
    REQUIRE_FALSE(f.table.mode_auto.set_auto_armed);
}

TEST_CASE("ModeAuto run SubMode LOITER leftover loiter_run when disarmed_or_landed", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::LOITER;
    f.table.mode_auto.disarmed_or_landed = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.loiter_run);
    REQUIRE(f.table.mode_auto.make_safe_ground_handling);
    REQUIRE_FALSE(f.table.mode_auto.desired_spool_unlimited);
    REQUIRE_FALSE(f.table.mode_auto.update_wpnav);
    REQUIRE_FALSE(f.table.mode_auto.terrain_failsafe_status);
    REQUIRE_FALSE(f.table.mode_auto.pos_D_update);
    REQUIRE_FALSE(f.table.mode_auto.input_thrust_vector_heading);
    REQUIRE_FALSE(f.table.mode_auto.wp_run);
    REQUIRE_FALSE(f.table.mode_auto.land_run_normal_or_precland);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_run);
    REQUIRE_FALSE(f.table.mode_auto.leftover_circle_nav_update);
}

TEST_CASE("ModeAuto run SubMode LOITER_TO_ALT leftover loiter_to_alt_run", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::LOITER_TO_ALT;
    REQUIRE_FALSE(f.table.mode_auto.disarmed_or_landed);
    REQUIRE(f.table.mode_auto.motors_interlock);
    REQUIRE_FALSE(f.table.mode_auto.leftover_loiter_to_alt_reached_xy);
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, false, true, false);
    require_loiter_to_alt_wp_run_reuse(f.table.mode_auto);
}

TEST_CASE("ModeAuto run SubMode LOITER_TO_ALT leftover when disarmed_or_landed", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::LOITER_TO_ALT;
    f.table.mode_auto.disarmed_or_landed = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.loiter_to_alt_run);
    REQUIRE(f.table.mode_auto.make_safe_ground_handling);
    REQUIRE_FALSE(f.table.mode_auto.desired_spool_unlimited);
    REQUIRE_FALSE(f.table.mode_auto.update_wpnav);
    REQUIRE_FALSE(f.table.mode_auto.terrain_failsafe_status);
    REQUIRE_FALSE(f.table.mode_auto.pos_D_update);
    REQUIRE_FALSE(f.table.mode_auto.input_thrust_vector_heading);
    REQUIRE_FALSE(f.table.mode_auto.leftover_reached_wp_destination_ne);
    REQUIRE_FALSE(f.table.mode_auto.leftover_loiter_to_alt_rest);
    REQUIRE_FALSE(f.table.mode_auto.wp_run);
    REQUIRE_FALSE(f.table.mode_auto.loiter_run);
}

TEST_CASE("ModeAuto run SubMode LOITER_TO_ALT leftover when motors interlock off", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::LOITER_TO_ALT;
    REQUIRE_FALSE(f.table.mode_auto.disarmed_or_landed);
    f.table.mode_auto.motors_interlock = false;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.loiter_to_alt_run);
    REQUIRE(f.table.mode_auto.make_safe_ground_handling);
    REQUIRE_FALSE(f.table.mode_auto.desired_spool_unlimited);
    REQUIRE_FALSE(f.table.mode_auto.update_wpnav);
    REQUIRE_FALSE(f.table.mode_auto.leftover_reached_wp_destination_ne);
    REQUIRE_FALSE(f.table.mode_auto.leftover_loiter_to_alt_rest);
}

TEST_CASE("ModeAuto run SubMode LOITER_TO_ALT leftover skips wp_run when reached_xy", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::LOITER_TO_ALT;
    REQUIRE_FALSE(f.table.mode_auto.disarmed_or_landed);
    REQUIRE(f.table.mode_auto.motors_interlock);
    f.table.mode_auto.leftover_loiter_to_alt_reached_xy = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.loiter_to_alt_run);
    REQUIRE_FALSE(f.table.mode_auto.leftover_reached_wp_destination_ne);
    REQUIRE_FALSE(f.table.mode_auto.leftover_loiter_to_alt_rest);
    REQUIRE_FALSE(f.table.mode_auto.wp_run);
    REQUIRE_FALSE(f.table.mode_auto.loiter_run);
    require_no_wp_run_leftovers(f.table.mode_auto);
}

TEST_CASE("ModeAuto run SubMode NAV_ATTITUDE_TIME leftover nav_attitude_time_run", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::NAV_ATTITUDE_TIME;
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, false, false, true);
}

TEST_CASE("ModeAuto run SubMode NAVGUIDED closed does not set nav_guided_run", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::NAVGUIDED;
    REQUIRE_FALSE(f.table.mode_auto.nav_guided_or_scripting);
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, false, false, false);
}

TEST_CASE("ModeAuto run SubMode NAVGUIDED open leftover nav_guided_run", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::NAVGUIDED;
    f.table.mode_auto.nav_guided_or_scripting = true;
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, true, false, false, false);
}

TEST_CASE("ModeAuto run SubMode NAV_SCRIPT_TIME open leftover nav_guided_run", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::NAV_SCRIPT_TIME;
    f.table.mode_auto.nav_guided_or_scripting = true;
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, true, false, false, false);
}

TEST_CASE("ModeAuto run SubMode switch fires while waiting_to_start", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    REQUIRE(f.table.mode_auto.waiting_to_start);
    REQUIRE_FALSE(f.table.mode_auto.has_origin);
    f.table.mode_auto.submode = ModeAuto::SubMode::TAKEOFF;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.waiting_to_start);
    REQUIRE_FALSE(f.table.mode_auto.start_or_resume);
    REQUIRE_FALSE(f.table.mode_auto.mission_update);
    require_submode_runs(f.table.mode_auto, true, false, false, false, false, false, false, false, false);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_run);
}

TEST_CASE("ModeAuto run SubMode switch clears previous leftover flags", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.submode = ModeAuto::SubMode::TAKEOFF;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.takeoff_run);
    REQUIRE(f.table.mode_auto.auto_takeoff_run);
    require_no_wp_run_leftovers(f.table.mode_auto);
    f.table.mode_auto.submode = ModeAuto::SubMode::WP;
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, true, false, false, false, false, false, false, false);
    REQUIRE_FALSE(f.table.mode_auto.auto_takeoff_run);
    REQUIRE_FALSE(f.table.mode_auto.set_auto_armed);
    require_wp_run_flying(f.table.mode_auto);
    f.table.mode_auto.submode = ModeAuto::SubMode::LAND;
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, true, false, false, false, false, false, false);
    require_land_run_flying(f.table.mode_auto);
    REQUIRE_FALSE(f.table.mode_auto.update_wpnav);
    REQUIRE_FALSE(f.table.mode_auto.terrain_failsafe_status);
    REQUIRE_FALSE(f.table.mode_auto.pos_D_update);
    REQUIRE_FALSE(f.table.mode_auto.input_thrust_vector_heading);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_run);
    REQUIRE_FALSE(f.table.mode_auto.leftover_circle_nav_update);
    f.table.mode_auto.submode = ModeAuto::SubMode::RTL;
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, false, true, false, false, false, false, false);
    REQUIRE(f.table.mode_auto.leftover_mode_rtl_run);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_disarm_on_land);
    REQUIRE_FALSE(f.table.mode_auto.land_run);
    REQUIRE_FALSE(f.table.mode_auto.land_run_normal_or_precland);
    REQUIRE_FALSE(f.table.mode_auto.leftover_circle_nav_update);
    f.table.mode_auto.submode = ModeAuto::SubMode::CIRCLE;
    f.table.mode_auto.run();
    require_submode_runs(f.table.mode_auto, false, false, false, false, true, false, false, false, false);
    require_circle_run(f.table.mode_auto);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_run);
    f.table.mode_auto.submode = ModeAuto::SubMode::LAND;
    f.table.mode_auto.run();
    require_land_run_flying(f.table.mode_auto);
    REQUIRE_FALSE(f.table.mode_auto.leftover_mode_rtl_run);
    REQUIRE_FALSE(f.table.mode_auto.leftover_circle_nav_update);
}

TEST_CASE("ModeAuto run auto_RTL with no landing flags clears and writes AUTO_RTL_EXIT", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.auto_RTL = true;
    REQUIRE_FALSE(f.table.mode_auto.in_landing_sequence);
    REQUIRE_FALSE(f.table.mode_auto.in_return_path);
    REQUIRE_FALSE(f.table.mode_auto.mission_complete);
    f.table.mode_auto.run();
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    REQUIRE(f.table.mode_auto.write_mode_auto_rtl_exit);
    REQUIRE(f.table.mode_auto.written_mode_number == Mode::Number::AUTO);
    REQUIRE(f.table.mode_auto.written_reason == ModeReason::AUTO_RTL_EXIT);
    REQUIRE(f.table.mode_auto.mode_number() == Mode::Number::AUTO);
}

TEST_CASE("ModeAuto run auto_RTL with in_landing_sequence stays without write", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.auto_RTL = true;
    f.table.mode_auto.in_landing_sequence = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.auto_RTL);
    REQUIRE_FALSE(f.table.mode_auto.write_mode_auto_rtl_exit);
    REQUIRE(f.table.mode_auto.mode_number() == Mode::Number::AUTO_RTL);
}

TEST_CASE("ModeAuto run auto_RTL with in_return_path stays without write", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.auto_RTL = true;
    f.table.mode_auto.in_return_path = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.auto_RTL);
    REQUIRE_FALSE(f.table.mode_auto.write_mode_auto_rtl_exit);
}

TEST_CASE("ModeAuto run auto_RTL with mission_complete stays without write", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.auto_RTL = true;
    f.table.mode_auto.mission_complete = true;
    f.table.mode_auto.run();
    REQUIRE(f.table.mode_auto.auto_RTL);
    REQUIRE_FALSE(f.table.mode_auto.write_mode_auto_rtl_exit);
}

TEST_CASE("ModeAuto run without auto_RTL does not write AUTO_RTL_EXIT leftover", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    f.table.mode_auto.run();
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    REQUIRE_FALSE(f.table.mode_auto.write_mode_auto_rtl_exit);
}

TEST_CASE("ModeAuto run auto_RTL leftover keeps LOITER SubMode dispatch", "[copter][mode]") {
    Fixture f;
    REQUIRE(set_mode(f.ctx, f.table, Mode::Number::AUTO, ModeReason::RC_COMMAND, {}));
    f.table.mode_auto.waiting_to_start = false;
    f.table.mode_auto.auto_RTL = true;
    f.table.mode_auto.submode = ModeAuto::SubMode::LOITER;
    f.table.mode_auto.run();
    REQUIRE_FALSE(f.table.mode_auto.auto_RTL);
    REQUIRE(f.table.mode_auto.write_mode_auto_rtl_exit);
    require_submode_runs(f.table.mode_auto, false, false, false, false, false, false, true, false, false);
}

TEST_CASE("leftover remaining_count matches catalog", "[copter][mode][leftover]") {
    REQUIRE(remaining_count() == 1);
    REQUIRE(remaining_count() > 0);
    REQUIRE(mode_this_slice_count() == 2);
    REQUIRE(mode_on_main_count() == 27);
    REQUIRE(mode_out_of_scope_count() == 3);
    REQUIRE(mode_completeness_size() ==
            mode_on_main_count() + mode_this_slice_count() + remaining_count() + mode_out_of_scope_count());
    REQUIRE(mode_completeness_has("leftover catalog", ModePortStatus::kThisSlice));
    REQUIRE(mode_completeness_has("Mode::Number", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeReason", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("Mode base virtuals", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("mode_from_mode_num stabilize+althold", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("set_mode checks", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("stabilize_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("AUTO_RTL", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("acro_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("althold_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("remaining mode bodies", ModePortStatus::kRemaining));
    REQUIRE(mode_completeness_has("ModeAuto::init", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::exit", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::run else-path", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::run SubMode switch", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::run auto_RTL landing-sequence", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::takeoff_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::wp_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::land_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::rtl_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::loiter_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::circle_run", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("ModeAuto::loiter_to_alt_run", ModePortStatus::kThisSlice));
    REQUIRE(mode_completeness_has("FLTMODE_GCSBLOCK param", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("fence recovery", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("update_flight_mode FAST_TASK", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("Write_Mode/notify", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("Drift-as-manual-throttle", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("set_accel_throttle_I", ModePortStatus::kOnMain));
    REQUIRE(mode_completeness_has("HELI runup/flybar", ModePortStatus::kOutOfScope));
    REQUIRE(mode_completeness_has("AP:: singletons", ModePortStatus::kOutOfScope));
    REQUIRE(mode_completeness_has("AP_Notify sounds", ModePortStatus::kOutOfScope));
}
