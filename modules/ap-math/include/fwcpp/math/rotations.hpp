#pragma once

// Port of AP_Math/rotations.h - just the enum. CPP-019.
//
// "these rotations form a full set - every rotation in the following list
// when combined with another in the list forms an entry which is also in
// the list" - upstream's own comment. Values are stored to EEPROM upstream
// (not yet a concern here - this port has no AP_Param), so the numbering
// is preserved exactly regardless.

#include <cstdint>

namespace fwcpp::math {

enum class Rotation : std::uint8_t {
    NONE = 0,
    YAW_45 = 1,
    YAW_90 = 2,
    YAW_135 = 3,
    YAW_180 = 4,
    YAW_225 = 5,
    YAW_270 = 6,
    YAW_315 = 7,
    ROLL_180 = 8,
    ROLL_180_YAW_45 = 9,
    ROLL_180_YAW_90 = 10,
    ROLL_180_YAW_135 = 11,
    PITCH_180 = 12,
    ROLL_180_YAW_225 = 13,
    ROLL_180_YAW_270 = 14,
    ROLL_180_YAW_315 = 15,
    ROLL_90 = 16,
    ROLL_90_YAW_45 = 17,
    ROLL_90_YAW_90 = 18,
    ROLL_90_YAW_135 = 19,
    ROLL_270 = 20,
    ROLL_270_YAW_45 = 21,
    ROLL_270_YAW_90 = 22,
    ROLL_270_YAW_135 = 23,
    PITCH_90 = 24,
    PITCH_270 = 25,
    PITCH_180_YAW_90 = 26,  // same as ROLL_180_YAW_270
    PITCH_180_YAW_270 = 27, // same as ROLL_180_YAW_90
    ROLL_90_PITCH_90 = 28,
    ROLL_180_PITCH_90 = 29,
    ROLL_270_PITCH_90 = 30,
    ROLL_90_PITCH_180 = 31,
    ROLL_270_PITCH_180 = 32,
    ROLL_90_PITCH_270 = 33,
    ROLL_180_PITCH_270 = 34,
    ROLL_270_PITCH_270 = 35,
    ROLL_90_PITCH_180_YAW_90 = 36,
    ROLL_90_YAW_270 = 37,
    ROLL_90_PITCH_68_YAW_293 = 38, // actually roll 90, pitch 68.8, yaw 293.3
    PITCH_315 = 39,
    ROLL_90_PITCH_315 = 40,
    PITCH_7 = 41,
    ROLL_45 = 42,
    ROLL_315 = 43,
    MAX = 44,
    CUSTOM_OLD = 100,
    CUSTOM_1 = 101,
    CUSTOM_2 = 102,
    CUSTOM_END = 103,
};

} // namespace fwcpp::math
