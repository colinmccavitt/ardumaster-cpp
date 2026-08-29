#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_land_detector.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>

using fwcpp::quadplane::CheckLandCompleteInputs;
using fwcpp::quadplane::CheckLandFinalInputs;
using fwcpp::quadplane::LandDetectorInputs;
using fwcpp::quadplane::PosControlLandStub;
using fwcpp::quadplane::PosControlState;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::ShouldRelaxInputs;
using fwcpp::quadplane::check_land_complete;
using fwcpp::quadplane::check_land_final;
using fwcpp::quadplane::kCheckLandCompleteTimeoutMs;
using fwcpp::quadplane::kCheckLandFinalTimeoutMs;
using fwcpp::quadplane::kDetectAltChangeDefaultM;
using fwcpp::quadplane::kLandFinalAltDefaultM;
using fwcpp::quadplane::kShouldRelaxLowerLimitMs;
using fwcpp::quadplane::land_detector;
using fwcpp::quadplane::should_relax;

static ShouldRelaxInputs at_lower_limit(std::uint32_t now_ms) {
    return ShouldRelaxInputs{
        .now_ms = now_ms,
        .throttle_lower = true,
        .throttle_mix_min = true,
        .throttle = 0.2f,
    };
}

static LandDetectorInputs detector_at_rest(std::uint32_t now_ms, float height_m = 1.0f) {
    return LandDetectorInputs{
        .now_ms = now_ms,
        .throttle_lower = true,
        .throttle_mix_min = true,
        .throttle = 0.0f,
        .height_m = height_m,
        .pilot_correction_active = false,
    };
}

static void arm_detector(PosControlLandStub& land, std::uint32_t lower_ms, std::uint32_t land_ms,
                         float vpos_m) {
    land.lower_limit_start_ms = lower_ms;
    land.land_start_ms = land_ms;
    land.vpos_start_m = vpos_m;
}

TEST_CASE("should_relax latches then true after 1000ms", "[quadplane][land]") {
    PosControlLandStub land{};
    REQUIRE_FALSE(should_relax(land, at_lower_limit(100)));
    REQUIRE(land.lower_limit_start_ms == 100);
    REQUIRE(land.land_start_ms == 0);

    REQUIRE_FALSE(should_relax(land, at_lower_limit(100 + kShouldRelaxLowerLimitMs)));
    REQUIRE(land.lower_limit_start_ms == 100);

    REQUIRE(should_relax(land, at_lower_limit(100 + kShouldRelaxLowerLimitMs + 1)));
    REQUIRE(land.lower_limit_start_ms == 100);

    QuadPlane qp{1};
    REQUIRE_FALSE(qp.should_relax(at_lower_limit(50)));
    REQUIRE(qp.poscontrol_land().lower_limit_start_ms == 50);
    REQUIRE(qp.should_relax(at_lower_limit(50 + kShouldRelaxLowerLimitMs + 1)));
}

TEST_CASE("should_relax throttle rest forces lower limit", "[quadplane][land]") {
    PosControlLandStub land{};
    ShouldRelaxInputs in{.now_ms = 1, .throttle_lower = false, .throttle_mix_min = false, .throttle = 0.009f};
    REQUIRE_FALSE(should_relax(land, in));
    REQUIRE(land.lower_limit_start_ms == 1);
    in.now_ms = 1 + kShouldRelaxLowerLimitMs + 1;
    REQUIRE(should_relax(land, in));
}

TEST_CASE("should_relax not-at-limit clears both timers", "[quadplane][land]") {
    PosControlLandStub land{.land_start_ms = 40, .lower_limit_start_ms = 20};
    ShouldRelaxInputs in{
        .now_ms = 5000,
        .throttle_lower = true,
        .throttle_mix_min = false,
        .throttle = 0.2f,
    };
    REQUIRE_FALSE(should_relax(land, in));
    REQUIRE(land.lower_limit_start_ms == 0);
    REQUIRE(land.land_start_ms == 0);
}

TEST_CASE("land_detector height change aborts", "[quadplane][land]") {
    PosControlLandStub land{};
    REQUIRE_FALSE(land_detector(kCheckLandCompleteTimeoutMs, land, detector_at_rest(10, 2.0f)));
    REQUIRE(land.lower_limit_start_ms == 10);

    const std::uint32_t relax_ms = 10 + kShouldRelaxLowerLimitMs + 1;
    REQUIRE_FALSE(land_detector(kCheckLandCompleteTimeoutMs, land, detector_at_rest(relax_ms, 2.0f)));
    REQUIRE(land.land_start_ms == relax_ms);
    REQUIRE(land.vpos_start_m == 2.0f);

    auto jumped = detector_at_rest(relax_ms + 1, 2.0f + kDetectAltChangeDefaultM + 0.01f);
    REQUIRE_FALSE(land_detector(kCheckLandCompleteTimeoutMs, land, jumped));
    REQUIRE(land.land_start_ms == 0);
}

TEST_CASE("land_detector timeout_ms and timeout+1000 lower-limit gates", "[quadplane][land]") {
    PosControlLandStub land{};
    arm_detector(land, 1000, 2001, 1.0f);
    auto in = detector_at_rest(6000, 1.0f);
    REQUIRE((6000 - 2001) < kCheckLandCompleteTimeoutMs);
    REQUIRE((6000 - 1000) >= (kCheckLandCompleteTimeoutMs + kShouldRelaxLowerLimitMs));
    REQUIRE_FALSE(land_detector(kCheckLandCompleteTimeoutMs, land, in));
    REQUIRE(land.land_start_ms == 2001);

    arm_detector(land, 4000, 1000, 1.0f);
    in = detector_at_rest(5500, 1.0f);
    REQUIRE((5500 - 1000) >= kCheckLandCompleteTimeoutMs);
    REQUIRE((5500 - 4000) < (kCheckLandCompleteTimeoutMs + kShouldRelaxLowerLimitMs));
    REQUIRE_FALSE(land_detector(kCheckLandCompleteTimeoutMs, land, in));
    REQUIRE(land.land_start_ms == 1000);

    arm_detector(land, 1000, 2000, 1.0f);
    in = detector_at_rest(7000, 1.0f);
    REQUIRE((7000 - 2000) >= kCheckLandCompleteTimeoutMs);
    REQUIRE((7000 - 1000) >= (kCheckLandCompleteTimeoutMs + kShouldRelaxLowerLimitMs));
    REQUIRE(land_detector(kCheckLandCompleteTimeoutMs, land, in));
}

TEST_CASE("land_detector pilot correction aborts", "[quadplane][land]") {
    PosControlLandStub land{};
    arm_detector(land, 10, 50, 1.0f);
    auto in = detector_at_rest(2000, 1.0f);
    in.pilot_correction_active = true;
    REQUIRE_FALSE(land_detector(kCheckLandCompleteTimeoutMs, land, in));
    REQUIRE(land.land_start_ms == 0);
}

TEST_CASE("check_land_complete only in LAND_FINAL", "[quadplane][land]") {
    PosControlState pc{};
    PosControlLandStub land{};
    arm_detector(land, 1000, 1000, 0.5f);
    pc.state = PositionControlState::kLandDescend;
    CheckLandCompleteInputs in{.detector = detector_at_rest(7000, 0.5f)};
    auto out = check_land_complete(pc, land, in);
    REQUIRE_FALSE(out.complete);
    REQUIRE(pc.state == PositionControlState::kLandDescend);

    pc.state = PositionControlState::kLandFinal;
    arm_detector(land, 1000, 1000, 0.5f);
    out = check_land_complete(pc, land, in);
    REQUIRE(out.complete);
    REQUIRE(out.landed_text);
    REQUIRE(out.disarm);
    REQUIRE_FALSE(out.spool_shut_down);
    REQUIRE(pc.state == PositionControlState::kLandComplete);
}

TEST_CASE("check_land_complete payload-place shuts down without complete", "[quadplane][land]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandFinal;
    PosControlLandStub land{};
    arm_detector(land, 1000, 1000, 0.4f);
    CheckLandCompleteInputs in{
        .detector = detector_at_rest(7000, 0.4f),
        .in_payload_place = true,
    };
    const auto out = check_land_complete(pc, land, in);
    REQUIRE_FALSE(out.complete);
    REQUIRE(out.spool_shut_down);
    REQUIRE_FALSE(out.disarm);
    REQUIRE(out.landed_text);
    REQUIRE(pc.state == PositionControlState::kLandComplete);
}

TEST_CASE("check_land_complete disarm vs continue_after_land", "[quadplane][land]") {
    const auto run = [](bool mode_auto, bool continue_after_land) {
        PosControlState pc{};
        pc.state = PositionControlState::kLandFinal;
        PosControlLandStub land{};
        arm_detector(land, 1000, 1000, 0.3f);
        CheckLandCompleteInputs in{
            .detector = detector_at_rest(7000, 0.3f),
            .mode_auto = mode_auto,
            .continue_after_land = continue_after_land,
        };
        return check_land_complete(pc, land, in);
    };

    {
        const auto out = run(false, false);
        REQUIRE(out.complete);
        REQUIRE(out.disarm);
    }
    {
        const auto out = run(true, false);
        REQUIRE(out.complete);
        REQUIRE(out.disarm);
    }
    {
        const auto out = run(true, true);
        REQUIRE(out.complete);
        REQUIRE_FALSE(out.disarm);
    }

    QuadPlane qp{1};
    qp.poscontrol_mut().state = PositionControlState::kLandFinal;
    arm_detector(qp.poscontrol_land_mut(), 1000, 1000, 0.3f);
    const auto wired = qp.check_land_complete(
        {.detector = detector_at_rest(7000, 0.3f), .mode_auto = true, .continue_after_land = true});
    REQUIRE(wired.complete);
    REQUIRE_FALSE(wired.disarm);
    REQUIRE(qp.poscontrol().state == PositionControlState::kLandComplete);
}

TEST_CASE("check_land_final 5m glitch filter", "[quadplane][land]") {
    PosControlLandStub land{};
    CheckLandFinalInputs in{
        .detector = detector_at_rest(10, 10.0f),
        .height_above_ground_m = 10.0f,
        .land_final_alt_m = kLandFinalAltDefaultM,
    };
    REQUIRE_FALSE(check_land_final(land, in));
    REQUIRE(land.last_land_final_agl_m == 10.0f);

    in.height_above_ground_m = 3.0f;
    REQUIRE_FALSE(check_land_final(land, in));
    REQUIRE(land.last_land_final_agl_m == 3.0f);

    in.height_above_ground_m = 3.1f;
    REQUIRE(check_land_final(land, in));
    REQUIRE(land.last_land_final_agl_m == 3.0f);
}

TEST_CASE("check_land_final falls through to 6000ms detector", "[quadplane][land]") {
    PosControlLandStub land{};
    arm_detector(land, 1000, 1000, 8.0f);
    land.last_land_final_agl_m = 20.0f;
    CheckLandFinalInputs in{
        .detector = detector_at_rest(1000 + kCheckLandFinalTimeoutMs - 1, 8.0f),
        .height_above_ground_m = 20.0f,
        .land_final_alt_m = kLandFinalAltDefaultM,
    };
    REQUIRE_FALSE(check_land_final(land, in));

    arm_detector(land, 1000, 1000, 8.0f);
    land.last_land_final_agl_m = 20.0f;
    in.detector.now_ms = 1000 + kCheckLandFinalTimeoutMs + kShouldRelaxLowerLimitMs;
    REQUIRE(check_land_final(land, in));

    QuadPlane qp{1};
    qp.set_land_final_alt_m(6.0f);
    qp.poscontrol_land_mut().last_land_final_agl_m = 0.0f;
    REQUIRE(qp.check_land_final({.detector = detector_at_rest(10, 2.0f), .height_above_ground_m = 2.0f}));
}
