#pragma once

// AC_Circle port placeholder (CCP-028). Upstream:
// libraries/AC_WPNav/AC_Circle.{h,cpp} (~708 loc).
//
// DEFERRED TO A LATER CCP-028 SLICE (Rust spec: crates/ap-wpnav/src/circle.rs):
//   Circle::init_ned_m / init
//   Circle::update_ms (+ calc_velocities, angular ramp, terrain D leftover)
//   Circle::set_center
//   Circle::get_closest_point_on_circle_ned_m
//   PosControl leftovers: NE/D init stopping point, input_pos_vel_accel_* ,
//     D_set_pos_target_from_climb_rate_ms, NE_update_controller
//
// Parity tests to port: tests/circle_init.rs, circle_leftover.rs
