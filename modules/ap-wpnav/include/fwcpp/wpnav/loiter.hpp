#pragma once

// AC_Loiter port placeholder (CCP-028). Upstream:
// libraries/AC_WPNav/AC_Loiter.{h,cpp} (~538 loc).
//
// DEFERRED TO A LATER CCP-028 SLICE (Rust spec: crates/ap-wpnav/src/loiter.rs):
//   Loiter::init_target_m / init_target
//   Loiter::update (+ calc_desired_velocity leftover)
//   Loiter::set_pilot_desired_acceleration_rad
//   PosControl leftovers: NE_set_correction_speed_accel_m, NE_set_pos_error_max_m,
//     NE_init_controller_stopping_point, NE_relax_velocity_controller,
//     set_pos_desired_NE_m, set_pos_vel_accel_NE_m, NE_update_controller
//   Fence/obstacle velocity adjust (COP-026 upstream scope)
//
// Parity tests to port: tests/loiter_init.rs, loiter_pilot_accel.rs
