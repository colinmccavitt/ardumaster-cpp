#include <catch2/catch_test_macros.hpp>
#include <fwcpp/vtol_assist/assist_recovery.hpp>
#include <fwcpp/vtol_assist/assist_recovery_options.hpp>

using fwcpp::vtol_assist::AssistOption;
using fwcpp::vtol_assist::AssistRecoveryInputs;
using fwcpp::vtol_assist::RecoveryPath;
using fwcpp::vtol_assist::VtolAssist;
using fwcpp::vtol_assist::allow_fw_vtol_recovery;
using fwcpp::vtol_assist::classify_recovery_path;
using fwcpp::vtol_assist::spin_recovery_option_allows;

TEST_CASE("Q_ASSIST_OPTIONS recovery path classification", "[vtol_assist][recovery][options]") {
    VtolAssist assist = VtolAssist::with_defaults();
    AssistRecoveryInputs in{};

    REQUIRE(classify_recovery_path(assist, in) == RecoveryPath::kFwRecoveryAllowed);
    REQUIRE(allow_fw_vtol_recovery(assist, in));
    REQUIRE(spin_recovery_option_allows(assist));

    assist.set_options(static_cast<std::int16_t>(AssistOption::kFwForceDisabled));
    REQUIRE(classify_recovery_path(assist, in) == RecoveryPath::kBlockedFwForceDisabled);
    REQUIRE_FALSE(allow_fw_vtol_recovery(assist, in));

    assist.set_options(0);
    in.tailsitter_enabled = true;
    REQUIRE(classify_recovery_path(assist, in) == RecoveryPath::kBlockedTailsitter);
    REQUIRE_FALSE(allow_fw_vtol_recovery(assist, in));

    in.tailsitter_enabled = false;
    in.in_qacro_mode = true;
    REQUIRE(classify_recovery_path(assist, in) == RecoveryPath::kBlockedQacro);
    REQUIRE_FALSE(allow_fw_vtol_recovery(assist, in));

    assist.set_options(static_cast<std::int16_t>(AssistOption::kSpinDisabled));
    in.in_qacro_mode = false;
    REQUIRE(spin_recovery_option_allows(assist) == false);
}
