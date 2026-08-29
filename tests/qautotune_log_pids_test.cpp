#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <fwcpp/qautotune/qautotune_log_pids.hpp>

using fwcpp::qautotune::QAutotuneRatePidSnapshot;
using fwcpp::qautotune::kLogPiqpIdentity;
using fwcpp::qautotune::kLogPiqrIdentity;
using fwcpp::qautotune::kLogPiqyIdentity;
using fwcpp::qautotune::kQAutotuneLogPidHooks;
using fwcpp::qautotune::kRatePitchPidInfo;
using fwcpp::qautotune::kRateRollPidInfo;
using fwcpp::qautotune::kRateYawPidInfo;
using fwcpp::qautotune::qautotune_log_pid_hook_count;
using fwcpp::qautotune::resolve_qautotune_log_pids;

TEST_CASE("qautotune log_pids names PIQR PIQP PIQY sources", "[qautotune][log]") {
    REQUIRE(qautotune_log_pid_hook_count() == 3);
    REQUIRE(std::strcmp(kQAutotuneLogPidHooks[0].identity, kLogPiqrIdentity) == 0);
    REQUIRE(std::strcmp(kQAutotuneLogPidHooks[0].pid_info_source, kRateRollPidInfo) == 0);
    REQUIRE(std::strcmp(kQAutotuneLogPidHooks[1].identity, kLogPiqpIdentity) == 0);
    REQUIRE(std::strcmp(kQAutotuneLogPidHooks[1].pid_info_source, kRatePitchPidInfo) == 0);
    REQUIRE(std::strcmp(kQAutotuneLogPidHooks[2].identity, kLogPiqyIdentity) == 0);
    REQUIRE(std::strcmp(kQAutotuneLogPidHooks[2].pid_info_source, kRateYawPidInfo) == 0);
}

TEST_CASE("qautotune log_pids gated by HAL logging", "[qautotune][log]") {
    QAutotuneRatePidSnapshot pids{};
    pids.rate_roll.p = 1.0f;
    pids.rate_pitch.i = 2.0f;
    pids.rate_yaw.d = 3.0f;

    const auto off = resolve_qautotune_log_pids(false, pids);
    REQUIRE_FALSE(off.piqr.write);
    REQUIRE_FALSE(off.piqp.write);
    REQUIRE_FALSE(off.piqy.write);
    REQUIRE(off.piqr.pid.p == 0.0f);

    const auto on = resolve_qautotune_log_pids(true, pids);
    REQUIRE(on.piqr.write);
    REQUIRE(on.piqp.write);
    REQUIRE(on.piqy.write);
    REQUIRE(std::strcmp(on.piqr.identity, "PIQR") == 0);
    REQUIRE(std::strcmp(on.piqp.identity, "PIQP") == 0);
    REQUIRE(std::strcmp(on.piqy.identity, "PIQY") == 0);
    REQUIRE(std::strcmp(on.piqr.pid_info_source, "rate_roll") == 0);
    REQUIRE(std::strcmp(on.piqp.pid_info_source, "rate_pitch") == 0);
    REQUIRE(std::strcmp(on.piqy.pid_info_source, "rate_yaw") == 0);
    REQUIRE(on.piqr.pid.p == 1.0f);
    REQUIRE(on.piqp.pid.i == 2.0f);
    REQUIRE(on.piqy.pid.d == 3.0f);
}
