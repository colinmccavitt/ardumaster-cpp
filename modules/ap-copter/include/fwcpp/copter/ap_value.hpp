#pragma once

// Copter::ap_value leftover. Upstream Copter.cpp ~762-777 and
// Copter.h ~361-389 PACKED ap. Field order is the bit index (logging
// depends on it). 27 bools, bits 0-26. No PACKED overlay — named
// fields copied into an explicit bool array, then 1U<<i. Cap at 32
// bits (ignore extra). Empty / all-false → 0.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter {

inline constexpr std::size_t kApValueFieldCount = 27;

struct ApValueInputs {
    bool unused1{false};                                 //  0
    bool unused_was_simple_mode_byte1{false};            //  1
    bool unused_was_simple_mode_byte2{false};            //  2
    bool pre_arm_rc_check{false};                        //  3
    bool pre_arm_check{false};                           //  4
    bool auto_armed{false};                              //  5
    bool unused_log_started{false};                      //  6
    bool land_complete{false};                           //  7
    bool new_radio_frame{false};                         //  8
    bool unused_usb_connected{false};                    //  9
    bool unused_receiver_present{false};                 // 10
    bool compass_mot{false};                             // 11
    bool motor_test{false};                              // 12
    bool initialised{false};                             // 13
    bool land_complete_maybe{false};                     // 14
    bool throttle_zero{false};                           // 15
    bool system_time_set_unused{false};                  // 16
    bool gps_glitching{false};                           // 17
    bool using_interlock{false};                         // 18
    bool land_repo_active{false};                        // 19
    bool motor_interlock_switch{false};                  // 20
    bool in_arming_delay{false};                         // 21
    bool initialised_params{false};                      // 22
    bool unused_compass_init_location{false};            // 23
    bool unused2_aux_switch_rc_override_allowed{false};  // 24
    bool armed_with_airmode_switch{false};               // 25
    bool prec_land_active{false};                        // 26
};

[[nodiscard]] inline std::uint32_t ap_value(const ApValueInputs& in = {}) {
    const bool bits[] = {
        in.unused1,
        in.unused_was_simple_mode_byte1,
        in.unused_was_simple_mode_byte2,
        in.pre_arm_rc_check,
        in.pre_arm_check,
        in.auto_armed,
        in.unused_log_started,
        in.land_complete,
        in.new_radio_frame,
        in.unused_usb_connected,
        in.unused_receiver_present,
        in.compass_mot,
        in.motor_test,
        in.initialised,
        in.land_complete_maybe,
        in.throttle_zero,
        in.system_time_set_unused,
        in.gps_glitching,
        in.using_interlock,
        in.land_repo_active,
        in.motor_interlock_switch,
        in.in_arming_delay,
        in.initialised_params,
        in.unused_compass_init_location,
        in.unused2_aux_switch_rc_override_allowed,
        in.armed_with_airmode_switch,
        in.prec_land_active,
    };
    static_assert((sizeof(bits) / sizeof(bits[0])) == kApValueFieldCount);

    std::uint32_t ret = 0;
    const std::size_t n = sizeof(bits) / sizeof(bits[0]);
    const std::size_t cap = n < 32U ? n : 32U;
    for (std::size_t i = 0; i < cap; ++i) {
        if (bits[i]) {
            ret |= 1U << static_cast<unsigned>(i);
        }
    }
    return ret;
}

}  // namespace fwcpp::copter
