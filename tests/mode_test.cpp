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

TEST_CASE("leftover remaining_count matches catalog", "[copter][mode][leftover]") {
    REQUIRE(remaining_count() == 1);
    REQUIRE(remaining_count() > 0);
    REQUIRE(mode_this_slice_count() == 2);
    REQUIRE(mode_on_main_count() == 16);
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
    REQUIRE(mode_completeness_has("ModeAuto::exit", ModePortStatus::kThisSlice));
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
