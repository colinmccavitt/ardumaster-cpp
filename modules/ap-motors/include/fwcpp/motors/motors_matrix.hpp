#pragma once

// Port of libraries/AP_Motors/AP_MotorsMatrix.cpp's (Copter-4.7.0) CORE
// factor storage/arithmetic - CCP-001, the first ticket of the copter-cpp
// effort (see /srv/ardumaster/tracker/efforts/copter-cpp.md) - PLUS
// (CCP-002) the first real consumer of that infrastructure,
// setup_quad_matrix (AP_MotorsMatrix.cpp ~line 576-711) - PLUS (CCP-003)
// the second, setup_hexa_matrix (AP_MotorsMatrix.cpp line 775-851) - PLUS
// (CCP-004) the third, setup_octa_matrix (AP_MotorsMatrix.cpp line
// 854-970) - PLUS (CCP-005) the fourth, setup_octaquad_matrix
// (AP_MotorsMatrix.cpp line 973-1140). Deliberately NOT the remaining
// three frame-class setup functions (setup_dodecahexa_matrix,
// setup_y6_matrix, setup_deca_matrix), NOT setup_motors' own
// dispatcher, and NOT the output stage (output_to_motors/
// output_armed_stabilizing/check_for_failed_motor/thrust_compensation/
// disable_yaw_torque) - all real, substantial, separately-scoped future
// phases. See "DEFERRED FUTURE PHASES" below.
//
// CCP-002 ADDITION (setup_quad_matrix) - upstream AP_MotorsMatrix.cpp,
// real function at line 576 (re-verified directly against the pinned
// worktree, not trusted from the ticket's own transcription; the ticket's
// guessed ~576-711 span matched exactly). The real switch(frame_type) has
// exactly these cases, in this order:
//   PLUS (581), X (592), NYT_PLUS (604, EXCLUDED - see below),
//   NYT_X (615, EXCLUDED), BF_X (627), BF_X_REV (640), DJI_X (652),
//   CW_X (665), V (678), H (689), VTAIL (701), ATAIL (724),
//   PLUSREV (744), Y4 (756), default (767).
// This is exhaustive - re-counted directly against the real file, matching
// the ticket's own list exactly (PLUS/X/BF_X/BF_X_REV/DJI_X/CW_X/V/H/
// VTAIL/ATAIL/PLUSREV/Y4, twelve in-scope cases).
//
// EXCLUDED: MOTOR_FRAME_TYPE_NYT_PLUS/MOTOR_FRAME_TYPE_NYT_X (lines
// 604-625) sit behind a real
// `#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)`
// gate (confirmed directly, line 603/626) - QuadPlane-specific frame
// layouts, not real ArduCopter build scope. copter-rust's own COP-005
// independently found and excluded the exact same two cases for the exact
// same reason ("they are quadplane layouts and belong to that effort" -
// COP-005 notes, 2026-08-25). Matching that precedent, both are excluded
// here too - a disclosed exclusion, not an oversight.
//
// DEFAULT BRANCH - confirmed the SIMPLE kind: line 767-769 is
// `default: // quad frame class does not support this frame type
//          return false;` - a genuine "not supported" fallback with no
// case logic hidden in it, UNLIKE setup_y6_matrix's own real `default:`
// (a separate, later-ticket concern; copter-rust's own COP-005 found that
// one answers for 24 of 64 upstream frame configurations). Quad's own
// default carries no such risk - confirmed by reading it directly, not
// assumed by analogy.
//
// REAL CONSTANTS used below (AP_MotorsMatrix.h lines 10-11,
// AP_Motors_Class.h lines 14-17, both re-verified against the real
// headers): AP_MOTORS_MATRIX_YAW_FACTOR_CW = -1, _CCW = 1 (ported as
// kYawFactorCw/kYawFactorCcw, float to match MotorDef::yaw_factor's real
// type); AP_MOTORS_MOT_1..4 = 0U,1U,2U,3U (zero-indexed motor numbers,
// used directly as VTAIL/ATAIL's own add_motor() motor_num argument).
//
// VTAIL vs ATAIL (lines 701-743) - REAL, RE-VERIFIED ASYMMETRY: both call
// add_motor() with the IDENTICAL four (motor_num, roll_deg, pitch_deg,
// testing_order) tuples - (0,60,60,1), (1,0,-160,3), (2,-60,-60,4),
// (3,0,160,2) - so roll_factor/pitch_factor/test_order are pairwise equal
// between the two frames. The ONE real difference is motors 1 and 3's yaw
// factor, which is the OPPOSITE sign in ATAIL vs VTAIL: VTAIL motor 1 =
// CW, motor 3 = CCW; ATAIL motor 1 = CCW, motor 3 = CW. Upstream's own
// comments (reproduced in full at each case below) explain why: an
// A-shaped VTail's rear motors face the opposite direction from a
// V-shaped VTail's for the same physical arm geometry, so the same
// physical rotor spin direction produces opposite yaw torque. Motors 0
// and 2 (the front motors) carry yaw_factor 0 in BOTH frames - "no yaw in
// front motors" per VTAIL's own comment, and ATAIL's own comment confirms
// "Yaw control is entirely in the rear motors" still holds. A port that
// collapsed VTAIL and ATAIL into one shared implementation would be
// wrong for both; motors_matrix_test.cpp tests this pair distinctly.
//
// PLUS vs PLUSREV (lines 581-591, 744-755) - REAL, RE-VERIFIED
// SIGN-NEGATION: PLUSREV's own comment says "plus with reversed motor
// directions". Confirmed directly: PLUSREV's four MotorDef entries have
// the EXACT SAME angle_degrees/testing_order as PLUS's own four entries,
// in the same order, with EVERY yaw_factor sign-flipped (PLUS: CCW, CCW,
// CW, CW for motors at 90/-90/0/180 degrees respectively; PLUSREV: CW,
// CW, CCW, CCW for the same four angles) - i.e. PLUSREV is exactly
// PLUS's own yaw_factor column negated, motor-for-motor, not an
// independent frame definition. motors_matrix_test.cpp confirms this
// exact negation relationship directly rather than merely checking
// PLUSREV's absolute values in isolation.
//
// V (line 678) - REAL, non-+-1 yaw factors, transcribed exactly as
// upstream's own float literals (NOT derived from a cos/sin formula):
// 0.7981f, 1.0000f, -0.7981f, -1.0000f for motors at 45/-135/-45/135
// degrees respectively.
//
// Y4 (line 756) - uses add_motors_raw with real explicit roll/pitch/yaw
// factors (no angle-based conversion at all): four MotorDefRaw entries,
// transcribed exactly from the real static array.
//
// CCP-003 ADDITION (setup_hexa_matrix) - upstream AP_MotorsMatrix.cpp,
// real function at line 775 (re-verified directly against the pinned
// worktree; the ticket's own guessed ~775-847 span was close but the real
// function, including its closing brace and `} //hexa` comment, runs
// 775-851). The real switch(frame_type) has exactly these cases, in this
// order: PLUS (780), X (793), H (806), DJI_X (820), CW_X (833),
// default (846). Five in-scope cases - exhaustive, re-counted directly
// against the real file, matching both the ticket's own list and
// AP_MOTORS_FRAME_HEXA_ENABLED's single #if/#endif region (774/852) that
// brackets the whole function - no other gated cases (no QuadPlane-only
// #if block like setup_quad_matrix's own NYT_PLUS/NYT_X) appear anywhere
// inside it, confirmed by reading the full function body line-by-line.
//
// DEFAULT BRANCH - confirmed the SIMPLE kind: line 846-848 is
// `default: // hexa frame class does not support this frame type
//          return false;` - matching setup_quad_matrix's own simple
// default exactly, NOT setup_y6_matrix's own real productive fallback
// (still a separate, later-ticket concern per COP-005 - see
// "DEFERRED FUTURE PHASES" below).
//
// FrameType-ENUM-SHARING INVESTIGATION (this ticket's own explicit
// question, answered directly from the real upstream header, not
// assumed): AP_Motors_Class.h declares TWO separate real enums.
// `motor_frame_class` (line 54-72: MOTOR_FRAME_QUAD=1, MOTOR_FRAME_HEXA=2,
// MOTOR_FRAME_OCTA=3, ... MOTOR_FRAME_DECA=14, etc.) selects which
// setup_*_matrix FUNCTION runs - that dispatch lives in setup_motors()
// (a separate, deferred future phase, not ported by this ticket).
// `motor_frame_type` (line 78-99: MOTOR_FRAME_TYPE_PLUS=0,
// MOTOR_FRAME_TYPE_X=1, ..., MOTOR_FRAME_TYPE_DJI_X=13,
// MOTOR_FRAME_TYPE_CW_X=14, etc.) is the SINGLE shared parameter type
// passed into WHICHEVER setup_*_matrix function frame_class selected -
// both setup_quad_matrix(motor_frame_type) and
// setup_hexa_matrix(motor_frame_type) take the literal same C++ type, and
// MOTOR_FRAME_TYPE_PLUS/X/H/DJI_X/CW_X are the literal same enumerator
// values (not merely same-named siblings in different enums) whichever
// switch dispatches on them. Confirmed directly: comparing setup_hexa_
// matrix's five cases against setup_quad_matrix's own twelve (see
// "CCP-002 ADDITION" above), all five of PLUS/X/H/DJI_X/CW_X are cases
// setup_quad_matrix ALREADY handles - hexa introduces ZERO frame types
// unseen by quad. DECISION: this port's own FrameType enum (below) is
// therefore NOT extended with any new enumerators for this ticket -
// setup_hexa_matrix(FrameType) below reuses the existing
// Plus/X/H/DjiX/CwX enumerators verbatim, exactly mirroring upstream's
// real single-shared-enum design. This is a real surprise relative to
// the ticket text's own suggestion ("extend... with five new
// enumerators") - the investigation the ticket itself asked for turned
// up that no extension is needed at all, only a second switch reusing
// the enum CCP-002 already built. Reported honestly, not silently
// corrected without comment.
//
// Hexa's own five frame types, transcribed exactly from the real source
// (all five are plain add_motors()/add_motors_raw() calls over a static
// six-motor array - hexa = six motors - unlike setup_quad_matrix's own
// VTAIL/ATAIL, there are no direct add_motor() calls and no per-motor
// sign-reversal subtleties here):
//   PLUS (780-791): angle/yaw/order sextuple
//     (0,CW,1) (180,CCW,4) (-120,CW,5) (60,CCW,2) (-60,CCW,6) (120,CW,3).
//   X (793-804): (90,CW,2) (-90,CCW,5) (-30,CW,6) (150,CCW,3) (30,CCW,1)
//     (-150,CW,4).
//   H (806-818) - upstream's own comment: "H is same as X except middle
//     motors are closer to center" - uses add_motors_raw with REAL
//     explicit (roll_fac, pitch_fac) pairs, not angle degrees:
//     (-1,0,CW,2) (1,0,CCW,5) (1,1,CW,6) (-1,-1,CCW,3) (-1,1,CCW,1)
//     (1,-1,CW,4).
//   DJI_X (820-831): (30,CCW,1) (-30,CW,6) (-90,CCW,5) (-150,CW,4)
//     (150,CCW,3) (90,CW,2).
//   CW_X (833-844): (30,CCW,1) (90,CW,2) (150,CCW,3) (-150,CW,4)
//     (-90,CCW,5) (-30,CW,6).
//
// NOT ported (same disclosed omission class as setup_quad_matrix's own
// _mav_type write, see below): the real upstream also sets
// `_mav_type = MAV_TYPE_HEXAROTOR;` unconditionally at the top of
// setup_hexa_matrix (line 777, before the switch) - pure MAVLink/GCS
// metadata, out of scope for the same reason as setup_quad_matrix's own
// MAV_TYPE_QUADROTOR write.
//
// NOT ported (this ticket's own explicit scope, per the file banner
// above and the ticket's acceptance criteria): the real upstream also
// sets `_mav_type = MAV_TYPE_QUADROTOR;` unconditionally at the top of
// setup_quad_matrix, before the switch - pure MAVLink/GCS metadata this
// port has no GCS to report to and no _mav_type-typed member for (see
// "NO-OP"/GCS-adjacent precedents in CCP-001's own banner above for the
// same class of disclosed omission). frame_type_string_/
// frame_class_string_ ARE ported (see class body) since the ticket
// explicitly asks for them as useful test/debug state even without a GCS.
//
// CCP-004 ADDITION (setup_octa_matrix) - upstream AP_MotorsMatrix.cpp,
// real function at line 854 (re-verified directly against the pinned
// worktree; the ticket's own guessed ~854-972 span was close - the real
// function, including its closing brace, runs 854-970, with the real
// AP_MOTORS_FRAME_OCTA_ENABLED #if/#endif bracketing it at 853/971).
// The real switch(frame_type) has exactly these cases, in this order:
// PLUS (859), X (875), V (890), H (905), I (920), DJI_X (935),
// CW_X (950), default (965). Seven in-scope cases - exhaustive,
// re-counted directly against the real file; no gated #if blocks appear
// anywhere inside the function body (confirmed by reading it
// line-by-line, same check CCP-003's own hexa banner section performed).
//
// DEFAULT BRANCH - confirmed the SIMPLE kind: line 965-966 is
// `default: // octa frame class does not support this frame type
//          return false;` - matching setup_quad_matrix's/setup_hexa_
// matrix's own simple defaults exactly, NOT setup_y6_matrix's own real
// productive fallback (still a separate, later-ticket concern per
// COP-005 - see "DEFERRED FUTURE PHASES" below).
//
// FrameType-ENUM INVESTIGATION (this ticket's own explicit question,
// answered by re-reading the CURRENT enum below directly before writing
// anything, not by trusting the ticket's own guess): of octa's seven
// real frame types, SIX (PLUS, X, V, H, DJI_X, CW_X) were CONFIRMED
// ALREADY PRESENT in this port's own FrameType enum, left over from
// CCP-002's quad work (the existing Plus, X, V, H, DjiX, CwX
// enumerators) - re-checked one by one against the enum's real
// declaration, not assumed from the ticket text. Exactly ONE,
// MOTOR_FRAME_TYPE_I (AP_Motors_Class.h line 91, re-verified directly:
// "MOTOR_FRAME_TYPE_I = 15, // (sideways H) octo only"), is genuinely
// new and is the only enumerator this ticket adds - as `I`, appended
// after the existing `Y4` to preserve the enum's own incremental-growth
// order rather than reordering any existing enumerator. This confirms
// the ticket's own guess exactly (unlike CCP-003's own hexa
// investigation, which found the ticket's guess of "five new
// enumerators" wrong - zero were actually needed there); here the
// ticket's guess of "one new enumerator" checked out, but was still
// independently re-verified against the real enum and the real switch
// cases rather than accepted on trust.
//
// Octa's own seven frame types, transcribed exactly from the real
// source (octa = eight motors; PLUS/X/DJI_X/CW_X are plain add_motors()
// calls over angle-based MotorDef tables; V/H/I are add_motors_raw()
// calls over explicit-factor MotorDefRaw tables - unlike quad/hexa,
// EVERY octa case uses one of these two forms, no direct add_motor()
// calls like quad's own VTAIL/ATAIL):
//   PLUS (859-871): angle/yaw/order octuple (0,CW,1) (180,CW,5)
//     (45,CCW,2) (135,CCW,4) (-45,CCW,8) (-135,CCW,6) (-90,CW,7)
//     (90,CW,3).
//   X (875-886): (22.5,CW,1) (-157.5,CW,5) (67.5,CCW,2) (157.5,CCW,4)
//     (-22.5,CCW,8) (-112.5,CCW,6) (-67.5,CW,7) (112.5,CW,3).
//   V (890-901) - REAL, RE-VERIFIED non-round explicit raw
//     (roll_fac, pitch_fac) pairs, NOT derived from any angle formula -
//     transcribed exactly, matching the ticket's own explicit warning
//     (roll_fac, pitch_fac, yaw, testing_order):
//     (0.83,0.34,CW,7) (-0.67,-0.32,CW,3) (0.67,-0.32,CCW,6)
//     (-0.50,-1.00,CCW,4) (1.00,1.00,CCW,8) (-0.83,0.34,CCW,2)
//     (-1.00,1.00,CW,1) (0.50,-1.00,CW,5).
//   H (905-916) - REAL, RE-VERIFIED raw (roll_fac, pitch_fac) pairs; six
//     of the eight entries use plain +-1.0f pitch factors, but the TWO
//     at testing_order 2 and 6 use +-0.333f pitch instead - transcribed
//     exactly, NOT a typo and NOT rounded to +-1.0f or +-0.33f, matching
//     the ticket's own explicit warning:
//     (-1.0,1.0,CW,1) (1.0,-1.0,CW,5) (-1.0,0.333,CCW,2)
//     (-1.0,-1.0,CCW,4) (1.0,1.0,CCW,8) (1.0,-0.333,CCW,6)
//     (1.0,0.333,CW,7) (-1.0,-0.333,CW,3).
//   I (920-931) - the genuinely new frame type (see enum investigation
//     above); upstream's own comment on its enumerator calls it
//     "(sideways H) octo only". REAL raw (roll_fac, pitch_fac) pairs,
//     same +-1.0f/+-0.333f value vocabulary as H's own but arranged
//     differently - here ROLL carries the four 0.333f-magnitude values,
//     not pitch:
//     (0.333,-1.0,CW,5) (-0.333,1.0,CW,1) (1.0,-1.0,CCW,6)
//     (0.333,1.0,CCW,8) (-0.333,-1.0,CCW,4) (-1.0,1.0,CCW,2)
//     (-1.0,-1.0,CW,3) (1.0,1.0,CW,7).
//   DJI_X (935-946): (22.5,CCW,1) (-22.5,CW,8) (-67.5,CCW,7)
//     (-112.5,CW,6) (-157.5,CCW,5) (157.5,CW,4) (112.5,CCW,3)
//     (67.5,CW,2).
//   CW_X (950-961): (22.5,CCW,1) (67.5,CW,2) (112.5,CCW,3) (157.5,CW,4)
//     (-157.5,CCW,5) (-112.5,CW,6) (-67.5,CCW,7) (-22.5,CW,8).
//
// NOT ported (same disclosed omission class as setup_quad_matrix's/
// setup_hexa_matrix's own _mav_type writes above): the real upstream
// also sets `_mav_type = MAV_TYPE_OCTOROTOR;` unconditionally at the
// top of setup_octa_matrix (line 857, before the switch) - pure
// MAVLink/GCS metadata, out of scope for the same reason as the other
// two _mav_type writes.
//
// Upstream (Copter-4.7.0, read directly from the pinned worktree at
// /srv/ardumaster/upstream/plane-4.7.0 - confirmed byte-identical to
// Copter-4.7.0 for this file by the copter-cpp effort charter, since both
// tags share the same commit 1511f27194f1dcc3728270883047bdf022b3fd53):
//   - AP_MotorsMatrix.cpp: add_motor_raw (line 502), add_motor 3-arg
//     (line 528), add_motor 4-arg (line 534), remove_motor (line 545),
//     add_motors (line 558), add_motors_raw (line 564),
//     normalise_rpy_factors (line 1353) - every line number re-verified
//     directly against the real file, not copied from the ticket.
//   - AP_MotorsMatrix.h: MotorDef (line 82), MotorDefRaw (line 96).
//   - AP_MotorsMatrix.cpp: setup_hexa_matrix (line 775, CCP-003),
//     setup_octa_matrix (line 854, CCP-004).
//   - AP_Motors_Class.h: initialised_ok()/set_initialised_ok() (lines
//     113-114), add_motor_num() declaration (line 310),
//     motor_frame_class enum (lines 54-72, CCP-003), motor_frame_type
//     enum (lines 78-99, CCP-003) - see "CCP-003 ADDITION" above for the
//     full enum-sharing investigation these two line ranges settled -
//     MOTOR_FRAME_TYPE_I specifically at line 91 (CCP-004, see
//     "CCP-004 ADDITION" above).
//   - AP_Motors_Class.cpp: add_motor_num()'s real body (line 214).
//   - AP_MotorsMulticopter.h: motor_enabled[AP_MOTORS_MAX_NUM_MOTORS]
//     (line 207) - the real array this ticket's functions operate on
//     lives in the multicopter base class, not AP_MotorsMatrix itself.
//   - AP_Motors_config.h (lines 7-29): AP_MOTORS_MAX_NUM_MOTORS's real
//     conditional definition - see kMaxNumMotors below.
//
// REAL FORMULAS, reproduced exactly (re-verified against the source above,
// not transcribed from the ticket text):
//
//   add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, testing_order,
//                 throttle_factor=1.0f):
//     if (initialised_ok()) return;                    // see GUARD below
//     if (motor_num in [0, kMaxNumMotors)) {
//       motor_enabled[motor_num] = true;
//       _roll_factor/_pitch_factor/_yaw_factor/_throttle_factor[motor_num]
//           = roll_fac/pitch_fac/yaw_fac/throttle_factor;
//       _test_order[motor_num] = testing_order;
//       add_motor_num(motor_num);                      // see NO-OP below
//     }
//
//   add_motor(motor_num, angle_degrees, yaw_factor, testing_order):
//     add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor,
//               testing_order);   // roll angle == pitch angle
//
//   add_motor(motor_num, roll_factor_in_degrees, pitch_factor_in_degrees,
//             yaw_factor, testing_order):
//     add_motor_raw(motor_num,
//                   cosf(radians(roll_factor_in_degrees + 90)),   // +90 !
//                   cosf(radians(pitch_factor_in_degrees)),       // no +90
//                   yaw_factor, testing_order);
//     REAL, RE-VERIFIED ASYMMETRY: the "+ 90" offset is applied to the
//     roll angle ONLY, before the cosine, never to pitch. Upstream's own
//     comment above add_motor_raw's caller ("assumes that for each motor,
//     roll and pitch factors are equal") refers to the INPUT angle being
//     shared between the two params, not to the two output factors being
//     equal - they are NOT equal even when roll_factor_in_degrees ==
//     pitch_factor_in_degrees, because only one of the two angles gets the
//     +90 shift before the cosine. motors_matrix_test.cpp's angle tests
//     are built specifically to catch a port that applies +90 to both
//     angles, or to neither - see that file's own comments.
//
//   remove_motor(motor_num): if in range, motor_enabled[motor_num] = false
//     and all four factors (roll/pitch/yaw/throttle) zeroed.
//
//   add_motors(MotorDef[], n) / add_motors_raw(MotorDefRaw[], n): trivial
//     index-order loops calling add_motor/add_motor_raw per element.
//
//   normalise_rpy_factors(): TWO passes over all kMaxNumMotors slots,
//     motor_enabled[i] ones only.
//     Pass 1 (find maxima): roll_fac = MAX(0, max(|roll[i]|)); same for
//       pitch_fac/yaw_fac (all three seeded at 0.0f, so a max is always
//       >= 0 even with no enabled motors); throttle_fac =
//       MAX(throttle_fac, MAX(0.0f, throttle[i])) - throttle's OWN maximum
//       already excludes negative per-motor values from consideration,
//       independently of the clamp applied again in pass 2.
//     Pass 2 (rescale), each guarded by is_zero(<axis>_fac) to avoid
//       dividing by a zero range:
//       roll[i]  = 0.5f * roll[i]  / roll_fac    (no floor - CAN go negative)
//       pitch[i] = 0.5f * pitch[i] / pitch_fac   (no floor)
//       yaw[i]   = 0.5f * yaw[i]   / yaw_fac     (no floor)
//       throttle[i] = MAX(0.0f, throttle[i] / throttle_fac)   (FLOORS AT 0)
//     REAL, RE-VERIFIED ASYMMETRY: throttle is the only one of the four
//     axes whose final rescaled value is clamped non-negative; roll/pitch/
//     yaw legitimately end up negative for motors on the "negative" side
//     of an axis. motors_matrix_test.cpp exercises this directly with a
//     motor set whose throttle factor is entered negative and confirms it
//     floors to exactly 0.0f while an equally-negative roll factor does not.
//
// GUARD (initialised_ok) - DESIGN DECISION, investigated not assumed:
// AP_Motors_Class.h's real _initialised_ok bool (line 408) is set true by
// AP_MotorsMatrix::init() (AP_MotorsMatrix.cpp ~line 1348,
// set_initialised_ok(success)) once a real setup_motors() frame-table call
// has completed. Its real purpose: once a frame has been set up
// successfully, add_motor_raw refuses to mutate the (now-live) motor
// table out from under an armed/spooling vehicle - re-running
// setup_motors() or an errant scripting add_motor call is a no-op after
// that point, not a re-initialization. This ticket does not port init()/
// setup_motors() (deferred - see below), so there is no real code path
// that would ever set this true YET. Reasonable minimal equivalent
// (matching the ticket's own suggested approach, verified against the
// real semantics above before accepting it): a plain bool
// initialised_ok_ this class owns directly, defaulting false (matching
// upstream's own real default - _initialised_ok is never explicitly
// initialized in AP_Motors's constructor list, but every real call path
// reaches it through set_initialised_ok(false) at the top of
// AP_MotorsMatrix::init() before any setup_*_matrix call, so "starts
// false" is upstream's real effective behavior, not a guess), with a
// public set_initialised_ok()/initialised_ok() pair mirroring the real
// upstream pair exactly so a future frame-table phase's init() can flip
// it the same way, and so this ticket's own tests can exercise the real
// guard behavior (add_motor_raw becoming a no-op once flipped true)
// without needing setup_motors() to exist yet.
//
// NO-OP (add_motor_num) - DESIGN DECISION, investigated not assumed:
// AP_Motors_Class.cpp's real add_motor_num() (line 214) does exactly two
// things, both real SRV_Channels output-registration plumbing this port
// has not built yet: SRV_Channels::get_motor_function(motor_num) (maps a
// motor index to an SRV_Channel::Function enumerator) and
// SRV_Channels::set_aux_channel_default(function, motor_num) (registers
// that function's default output channel assignment). Neither touches
// _roll_factor/_pitch_factor/_yaw_factor/_throttle_factor/motor_enabled/
// _test_order - the state this ticket's own tests actually verify. This
// port's ap-srv-channel module (modules/ap-srv-channel) exists and is
// almost certainly the eventual real target for this call (per the
// copter-cpp effort charter's own note that ap-srv-channel is "almost
// certainly reusable... confirm directly when first needed"), but wiring
// it up is real output-stage work explicitly out of THIS ticket's scope
// (see "DEFERRED FUTURE PHASES" below) - add_motor_num() is therefore a
// disclosed no-op here (add_motor_raw does not call anything in its
// place), not silently dropped: this comment IS the disclosure, and the
// future output-stage phase is the one that must add the real call.
//
// AP_MOTORS_MAX_NUM_MOTORS -> kMaxNumMotors - DESIGN DECISION, investigated
// not assumed: AP_Motors_config.h (lines 7-29) defines this conditionally:
// 32 if AP_SCRIPTING_ENABLED, else 12, then clamps down to
// NUM_SERVO_CHANNELS if that is smaller (with a further floor back up to
// 12 if that clamp would go below it). SITL builds (this port's own
// target, and every existing precedent module's own stated target) take
// the AP_SCRIPTING_ENABLED=1 branch, giving 32 before the NUM_SERVO_CHANNELS
// clamp. This port's OWN existing ap-srv-channel module
// (modules/ap-srv-channel/include/fwcpp/srv/srv_channels.hpp) already
// investigated and recorded upstream's real NUM_SERVO_CHANNELS resolution
// for this exact SITL target as 32 (kNumServoChannels, matching
// fwcpp::hal::kNumRcOutputChannels = 32, "matches SITL_NUM_CHANNELS") - so
// the NUM_SERVO_CHANNELS clamp does not reduce 32 (32 is not < 32), and
// upstream's real, fully-resolved value for this port's target is 32, not
// 12. This is also the exact number copter-rust's own COP-005 investigation
// found (and initially got wrong, at 12, from reading only the #else
// branch) - re-verified independently here against the real header rather
// than trusted from that note, and confirmed to agree with it AND with
// this port's own already-established kNumServoChannels precedent, so 32
// is used directly (kMaxNumMotors below), not invented.
//
// CCP-005 ADDITION (setup_octaquad_matrix) - upstream AP_MotorsMatrix.cpp,
// real function at line 973 (re-verified directly against the pinned
// worktree; the ticket's own guessed ~973-1140 span matched exactly - the
// real function, including its closing brace, runs 973-1140, with the
// real AP_MOTORS_FRAME_OCTAQUAD_ENABLED #if/#endif bracketing it at
// 972/1141). The real switch(frame_type) has exactly these cases, in this
// order: PLUS (977), X (992), V (1009), H (1028), CW_X (1050),
// BF_X (1069), BF_X_REV (1085), X_COR (1102), CW_X_COR (1122),
// default (1136). Nine in-scope cases - exhaustive, re-counted directly
// against the real file; no gated #if blocks appear anywhere inside the
// function body (same check CCP-003's/CCP-004's own banner sections
// performed).
//
// DEFAULT BRANCH - confirmed the SIMPLE kind: line 1136-1137 is
// `default: // octaquad frame class does not support this frame type
//          return false;` - matching setup_quad_matrix's/setup_hexa_
// matrix's/setup_octa_matrix's own simple defaults exactly, NOT
// setup_y6_matrix's own real productive fallback (still a separate,
// later-ticket concern per COP-005 - see "DEFERRED FUTURE PHASES" below).
//
// FrameType-ENUM INVESTIGATION (this ticket's own explicit question,
// answered by re-reading the CURRENT enum below directly before writing
// anything): of octaquad's nine real frame types, SEVEN (PLUS, X, V, H,
// CW_X, BF_X, BF_X_REV) were CONFIRMED ALREADY PRESENT in this port's own
// FrameType enum, left over from CCP-002's/CCP-004's own quad/octa work
// (the existing Plus, X, V, H, CwX, BfX, BfXRev enumerators) - re-checked
// one by one against the enum's real declaration, not assumed from the
// ticket text. Exactly TWO, MOTOR_FRAME_TYPE_X_COR and
// MOTOR_FRAME_TYPE_CW_X_COR, are genuinely new and are the only
// enumerators this ticket adds - named `XCor`/`CwXCor` below (appended
// after the existing `I`), following this enum's own established
// abbreviation convention exactly (`CwX` for CW_X, `BfX` for BF_X, `DjiX`
// for DJI_X - each upstream underscore-word boundary becomes a
// capitalized run with no separator). This confirms the ticket's own
// guess of "exactly two new enumerators" exactly.
//
// Octaquad's own seven "plain" frame types, transcribed exactly from the
// real source (octaquad = eight motors; all seven are plain add_motors()
// calls over a static MotorDef table - no add_motors_raw()/direct
// add_motor() calls anywhere in this function, unlike octa's own V/H/I):
//   PLUS (977-989): angle/yaw/order octuple (0,CCW,1) (-90,CW,7)
//     (180,CCW,5) (90,CW,3) (-90,CCW,8) (0,CW,2) (90,CCW,4) (180,CW,6).
//   X (992-1004): (45,CCW,1) (-45,CW,7) (-135,CCW,5) (135,CW,3)
//     (-45,CCW,8) (45,CW,2) (135,CCW,4) (-135,CW,6).
//   V (1009-1021) - REAL, RE-VERIFIED: this is a MotorDef table (angle
//     degrees + yaw_factor), matching setup_quad_matrix's own V case
//     exactly in SHAPE (angle-derived roll/pitch, non-+-1 explicit
//     yaw_factor) - NOT a MotorDefRaw (roll_fac, pitch_fac) table like
//     octa's own V. Same real non-round 0.7981f/1.0000f literals as
//     quad's V: (45,0.7981f,1) (-45,-0.7981f,7) (-135,1.0000f,5)
//     (135,-1.0000f,3) (-45,0.7981f,8) (45,-0.7981f,2) (135,1.0000f,4)
//     (-135,-1.0000f,6).
//   H (1028-1040) - upstream's own comment: "same as X but motors spin in
//     opposite directions": (45,CW,1) (-45,CCW,7) (-135,CW,5) (135,CCW,3)
//     (-45,CW,8) (45,CCW,2) (135,CW,4) (-135,CCW,6).
//   CW_X (1050-1062): (45,CCW,1) (45,CW,2) (135,CW,3) (135,CCW,4)
//     (-135,CCW,5) (-135,CW,6) (-45,CW,7) (-45,CCW,8).
//   BF_X (1069-1081) - upstream's own comment: betaflight octa-quad X
//     order using two 4-in-1 ESCs: (135,CW,3) (45,CCW,1) (-135,CCW,5)
//     (-45,CW,7) (135,CCW,4) (45,CW,2) (-135,CW,6) (-45,CCW,8).
//   BF_X_REV (1085-1097) - same table as BF_X with every yaw_factor
//     sign-flipped, matching quad's own BfX/BfXRev sign-negation
//     relationship: (135,CCW,3) (45,CW,1) (-135,CW,5) (-45,CCW,7)
//     (135,CW,4) (45,CCW,2) (-135,CCW,6) (-45,CW,8).
//
// THE REAL X8 CO-ROTATING PITFALL (X_COR/CW_X_COR, lines 1102-1134) -
// copter-rust's own COP-005 investigation found this first (see that
// ticket's 2026-08-25 note, finding #2/#3); re-verified independently
// here against the real source rather than trusted from that note. Both
// cases call add_motors() with their own 8-motor MotorDef table, THEN
// apply a real, SEPARATE per-motor rescaling `for` loop AFTER the table
// is applied - ported below as its own explicit second step, exactly
// mirroring this real two-step structure, NOT folded into the table:
//   X_COR (1102-1120): table is IDENTICAL to X's own table above except
//     motors at testing_order 8/2/4/6 use CW/CCW/CW/CCW yaw instead of
//     CCW/CW/CCW/CW (re-verify directly, do not assume it equals X's
//     table verbatim) - (45,CCW,1) (-45,CW,7) (-135,CCW,5) (135,CW,3)
//     (-45,CW,8) (45,CCW,2) (135,CW,4) (-135,CCW,6). THEN:
//     `for (uint8_t i=0; i<4; i++)` - scales array indices 0,1,2,3 (the
//     FIRST FOUR motors in array/table order) by
//     AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR.
//   CW_X_COR (1122-1134): table is (45,CCW,1) (45,CCW,2) (135,CW,3)
//     (135,CW,4) (-135,CCW,5) (-135,CCW,6) (-45,CW,7) (-45,CW,8). THEN:
//     `for (uint8_t i=0; i<8; i+=2)` - scales array indices 0,2,4,6
//     (EVERY OTHER motor, starting from index 0) by the SAME scale
//     constant - a genuinely DIFFERENT subset of motors from X_COR's own
//     first-four, re-verified directly against the real source (not
//     assumed to match).
//   BOTH loops scale ALL FOUR factor arrays for every affected index -
//     `_roll_factor[i] *=`, `_pitch_factor[i] *=`, `_yaw_factor[i] *=`,
//     AND `_throttle_factor[i] *=` - re-verified directly; neither
//     X_COR's nor CW_X_COR's own MotorDef table sets an explicit
//     throttle_factor (MotorDef has no such field - only add_motor_raw's
//     own default parameter does), so the value the throttle scaling
//     multiplies is add_motor_raw's own default `throttle_factor=1.0f`
//     (CCP-001) for every affected motor - i.e. the scaled result is
//     exactly `kOctaquadCorotatingScaleFactor` itself for throttle, not
//     some other pre-existing value. A port that only scaled
//     roll/pitch/yaw and left throttle untouched would silently under-
//     mix the top rotor layer's real throttle contribution - confirmed
//     this is NOT the case here, and motors_matrix_test.cpp tests this
//     directly for both frame types.
//
// THE REAL FLOAT-VS-DOUBLE PITFALL, DESIGN DECISION MADE HERE:
// AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR is defined in
// AP_Motors_config.h (re-verified directly against the pinned worktree,
// lines 64-66) as `#define AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_
// FACTOR 0.9` - a bare, UNSUFFIXED floating-point literal, confirmed NOT
// `0.9f` in the real header. Upstream's real effective numeric behavior
// is nonetheless `float`-precision `0.9f * factor`, because upstream's
// ENTIRE build applies `-fsingle-precision-constant`, which coerces every
// bare floating literal in the translation unit to `float` regardless of
// its own spelling - this is exactly the pitfall copter-rust's own
// COP-005 investigation found and wrote up as ADR-0011 ("applies to
// every bare literal in the tree, not just this one"). THIS PORT'S OWN
// `ap-motors` module does NOT link `fwcpp_upstream_flags` (re-verified
// directly against modules/ap-motors/CMakeLists.txt - still true as of
// this ticket, unchanged since CCP-001), so a bare unsuffixed `0.9`
// written in this port's own code would compute in `double` precision
// and produce a result numerically DIFFERENT (by a few ULP after the
// narrowing store into a `float` factor array) from upstream's real
// `float`-precision `0.9f * factor` - the exact discrepancy COP-005
// measured ("two ulp"). DECISION MADE (the ticket's own recommended,
// simplest fix, adopted as-is after independent judgment - the
// alternative of linking `fwcpp_upstream_flags` into this module was
// considered and rejected: it would coerce EVERY bare literal in this
// entire header, not just this one constant, which is a much larger and
// less-targeted behavior change than this ticket's own narrow scope
// warrants, for a module whose only other literals are already all
// explicitly float-suffixed): `kOctaquadCorotatingScaleFactor` below is
// defined as an explicit `0.9f` `float` literal - matching this whole
// file's own already-established convention of explicit `f`-suffixed
// literals everywhere else (e.g. V's own `0.7981f`) - which reproduces
// upstream's real EFFECTIVE numeric behavior without needing to change
// `ap-motors`'s own CMake link configuration. motors_matrix_test.cpp
// verifies this decision was correct with a test that compares the
// scaled result against a reference computed with an explicitly
// `float`-typed `0.9f * value` multiplication, tight enough (exact
// equality) to catch a few-ULP discrepancy, not just "approximately 0.9
// times the original" (see that file's own comments).
//
// Real upstream also has a compile-time check immediately before X_COR's
// own case body (line 1102, re-verified directly):
// `static_assert(AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR < 1.0,
// "...")` - ported below as a `static_assert` on `kOctaquadCorotating
// ScaleFactor` at namespace scope, since it is a real `constexpr` this
// port can assert against directly (cheap, real, worth keeping exactly
// as upstream does).
//
// NOT ported (same disclosed omission class as setup_quad_matrix's/
// setup_hexa_matrix's/setup_octa_matrix's own _mav_type writes above):
// the real upstream also sets `_mav_type = MAV_TYPE_OCTOROTOR;`
// unconditionally at the top of setup_octaquad_matrix (line 975, before
// the switch) - pure MAVLink/GCS metadata, out of scope for the same
// reason as the other three _mav_type writes.
//
// CCP-006 ADDITION (setup_y6_matrix) - upstream AP_MotorsMatrix.cpp, real
// function at line 1191 (re-verified directly against the pinned
// worktree; the ticket's own guessed ~1191-1241 span was close - the
// real function body, INCLUDING its closing brace, runs 1191-1239, with
// the real AP_MOTORS_FRAME_Y6_ENABLED #if/#endif bracketing it at
// 1190/1240 - one line shorter at the end than the ticket's own guess,
// re-counted directly, not assumed). The real switch(frame_type) has
// exactly TWO explicit cases plus a real, PRODUCTIVE default - a
// genuine, disclosed structural departure from every setup_*_matrix
// this port has built so far (setup_quad_matrix/setup_hexa_matrix/
// setup_octa_matrix/setup_octaquad_matrix, CCP-002 through CCP-005),
// each of which has exactly one "not supported, return false" default.
//
// THE REAL PRODUCTIVE default: PITFALL, CONFIRMED DIRECTLY (this is
// exactly the pitfall copter-rust's own COP-005 investigation found
// first - see that ticket's 2026-08-25 note, finding #1 - and this
// effort's own charter named explicitly; re-verified independently here
// against the real source, not trusted from that note): setup_y6_matrix's
// own switch names only MOTOR_FRAME_TYPE_Y6B (line 1195) and
// MOTOR_FRAME_TYPE_Y6F (line 1207) as explicit cases. Its own default:
// (line 1219-1236) is NOT "unsupported, return false" - it is a real,
// working six-motor MotorDefRaw table, transcribed exactly below, that
// upstream reaches for EVERY OTHER motor_frame_type value: every other
// named enumerator this port's own Plus/X/BfX/BfXRev/DjiX/CwX/V/H/VTail/
// ATail/PlusRev/Y4/I/XCor/CwXCor already model (none of which Y6 itself
// declares a case for), plus the ~5-7 other real upstream enumerators
// this port has not yet added (e.g. MOTOR_FRAME_TYPE_DECA,
// MOTOR_FRAME_TYPE_Y6B/_Y6F's own siblings like PLUS/X on frame classes
// this ticket does not touch), plus any truly out-of-range value. A
// switch-by-switch transcription that only handled Y6B/Y6F explicitly
// and made every other value `return false` would be WRONG - it would
// silently break every real Y6-frame-class vehicle not specifically
// configured as Y6B/Y6F, which real upstream never does. See
// motors_matrix_test.cpp's own dedicated fallback tests below, the
// single most important tests this ticket writes.
//
// THE SECOND, RELATED REAL CONSEQUENCE, CONFIRMED DIRECTLY: unlike
// setup_quad_matrix/setup_hexa_matrix/setup_octa_matrix/
// setup_octaquad_matrix (each of which has exactly one `return false;`,
// in its own default: case), setup_y6_matrix's own body has NO
// `return false;` ANYWHERE - re-read line-by-line to confirm this. Every
// path (Y6B, Y6F, and the productive default) falls through to the SAME
// unconditional `return true;` at line 1238. Ported below exactly: the
// switch has no `default: return false;` arm at all, and the function's
// own final statement is an unconditional `return true;` - a genuine,
// disclosed departure from every prior setup_*_matrix this port has
// built, not an oversight.
//
// Y6's own two explicit frame types plus its productive default,
// transcribed exactly from the real source (Y6 = six motors; all three
// cases are add_motors_raw() calls over a static MotorDefRaw table, no
// angle-based MotorDef anywhere in this function):
//   Y6B (1195-1206) - upstream's own comment: "Y6 motor definition with
//     all top motors spinning clockwise, all bottom motors counter
//     clockwise": (roll_fac,pitch_fac,yaw,order)
//     (-1.0,0.500,CW,1) (-1.0,0.500,CCW,2) (0.0,-1.000,CW,3)
//     (0.0,-1.000,CCW,4) (1.0,0.500,CW,5) (1.0,0.500,CCW,6).
//   Y6F (1207-1218) - upstream's own comment: "Y6 motor layout for
//     FireFlyY6": (0.0,-1.000,CCW,3) (-1.0,0.500,CCW,1) (1.0,0.500,CCW,5)
//     (0.0,-1.000,CW,4) (-1.0,0.500,CW,2) (1.0,0.500,CW,6). Note the
//     array's own testing_order is NOT in ascending order (3,1,5,4,2,6) -
//     transcribed exactly in that real array order, since add_motors_raw
//     uses array INDEX as motor_num, not testing_order.
//   default (1219-1236) - THE REAL PRODUCTIVE FALLBACK, upstream's own
//     case has no comment on it, just the table:
//     (-1.0,0.666,CCW,2) (1.0,0.666,CW,5) (1.0,0.666,CCW,6)
//     (0.0,-1.333,CW,4) (-1.0,0.666,CW,1) (0.0,-1.333,CCW,3).
//
// FrameType-ENUM INVESTIGATION (this ticket's own explicit question,
// answered by re-reading the CURRENT enum below directly before writing
// anything, and independently against the real header
// AP_Motors_Class.h lines 86-87): MOTOR_FRAME_TYPE_Y6B = 10 and
// MOTOR_FRAME_TYPE_Y6F = 11 are BOTH genuinely new - neither appears in
// any switch this port has ported so far (CCP-002 through CCP-005).
// Named `Y6B`/`Y6F` below (appended after the existing `CwXCor`),
// matching this enum's own established convention for names that are
// already a single capitalized-letter-run in upstream (e.g. `Y4` kept
// verbatim, `V`/`H`/`I` kept verbatim) - Y6B/Y6F have no internal
// underscore word-boundary to collapse (unlike `CW_X` -> `CwX`), so they
// are ported character-for-character as `Y6B`/`Y6F`.
//
// NOT ported (same disclosed omission class as every other
// setup_*_matrix's own _mav_type write above): the real upstream also
// sets `_mav_type = MAV_TYPE_HEXAROTOR;` unconditionally at the top of
// setup_y6_matrix (line 1193, before the switch) - pure MAVLink/GCS
// metadata, out of scope for the same reason as the other four
// _mav_type writes.
//
// CCP-007 ADDITION (setup_dodecahexa_matrix) - upstream AP_MotorsMatrix.cpp,
// real function at line 1140 (re-verified directly against the pinned
// worktree; the ticket's own guessed 1140-1188 span matched exactly, with
// the real AP_MOTORS_FRAME_DODECAHEXA_ENABLED #if/#endif bracketing it at
// 1139/1189). The real switch(frame_type) has exactly TWO explicit cases -
// PLUS (1145), X (1164) - plus a real default (1183). A genuine RETURN to
// every setup_*_matrix's own structure from BEFORE setup_y6_matrix
// (CCP-002 through CCP-005), NOT a continuation of Y6's own
// productive-default departure - re-verified directly, not assumed from
// "the function right after Y6 in the file". Confirmed directly: lines
// 1183-1185 are `default: // dodeca-hexa frame class does not support
// this frame type\n return false;` - the SIMPLE kind, matching
// setup_quad_matrix's/setup_hexa_matrix's/setup_octa_matrix's/
// setup_octaquad_matrix's own defaults exactly. Both explicit cases are
// plain add_motors() calls over a static 12-entry MotorDef table -
// dodecahexa = 12 motors (six physical positions, each with a top/bottom
// motor pair) - no add_motors_raw()/direct add_motor() calls anywhere in
// this function, a genuinely simpler shape than several earlier tickets.
//
// Dodecahexa's own two real frame types, transcribed exactly from the real
// source (angle/yaw/order triples, in real array order - upstream's own
// per-motor comments name each as a top/bottom pair at one of six physical
// positions: forward, forward-right, back-right, back, back-left,
// forward-left):
//   PLUS (1145-1163): (0,CCW,1) (0,CW,2) (60,CW,3) (60,CCW,4) (120,CCW,5)
//     (120,CW,6) (180,CW,7) (180,CCW,8) (-120,CCW,9) (-120,CW,10)
//     (-60,CW,11) (-60,CCW,12).
//   X (1164-1182): (30,CCW,1) (30,CW,2) (90,CW,3) (90,CCW,4) (150,CCW,5)
//     (150,CW,6) (-150,CW,7) (-150,CCW,8) (-90,CCW,9) (-90,CW,10)
//     (-30,CW,11) (-30,CCW,12) - same repeated-pair-of-six-positions
//     structure as PLUS, rotated 30 degrees.
//
// THE ONE REAL PITFALL THIS TICKET GUARDS AGAINST, RE-VERIFIED
// MOTOR-BY-MOTOR: both tables repeat each angle TWICE (a top/bottom pair
// per physical position) with ALTERNATING yaw factors - CCW,CW,CW,CCW,
// CCW,CW,CW,CCW,CCW,CW,CW,CCW for both PLUS and X (identical alternation
// pattern, only the angles differ). A copy-paste error collapsing a pair
// into two IDENTICAL yaw factors (e.g. both CCW instead of CCW-then-CW)
// would still compile and still populate all 12 motor slots, but would
// silently produce a frame with no differential top/bottom yaw authority
// at that physical position. No such bug is present here (re-verified
// against the real source above); motors_matrix_test.cpp's own tests
// check every pair's two yaw factors are opposite, not merely that 12
// motors exist with plausible-looking angles.
//
// FrameType-ENUM INVESTIGATION (this ticket's own explicit question,
// answered by re-reading the CURRENT enum below directly before writing
// anything): BOTH of dodecahexa's real frame types, PLUS and X, were
// CONFIRMED ALREADY PRESENT in this port's own FrameType enum, left over
// from CCP-002's own setup_quad_matrix work - re-checked directly against
// the enum's real declaration, not assumed from the ticket text. This
// ticket therefore adds ZERO new enumerators to FrameType.
//
// CORRECTION TO THE TICKET'S OWN CLAIM (found by independently checking
// this file's own history rather than trusting the ticket's summary, per
// this ticket's explicit instruction not to trust it as complete): the
// ticket asserts this would be "the first setup_*_matrix ticket in this
// arc" to add zero new enumerators. That is NOT correct - CCP-003's own
// setup_hexa_matrix already added zero new enumerators (see "CCP-003
// ADDITION" above: "CCP-003 added ZERO new enumerators: all five of
// setup_hexa_matrix's real frame types... were already present from
// CCP-002's setup_quad_matrix work"). CCP-007 is therefore the SECOND
// setup_*_matrix ticket in this arc to add zero new enumerators, not the
// first - stated here as a positive, independently-verified confirmation
// (real, checked evidence, not an omission), with the ticket's own
// over-claim explicitly corrected rather than silently repeated.
//
// NOT ported (same disclosed omission class as every other
// setup_*_matrix's own _mav_type write above): the real upstream also
// sets `_mav_type = MAV_TYPE_DODECAROTOR;` unconditionally at the top of
// setup_dodecahexa_matrix (line 1142, before the switch) - pure
// MAVLink/GCS metadata, out of scope for the same reason as the other
// five _mav_type writes.
//
// CCP-008 ADDITION (setup_deca_matrix) - upstream AP_MotorsMatrix.cpp, real
// function at line 1242 (re-verified directly against the pinned worktree;
// the ticket's own guessed 1242-1287 span matched exactly, with the real
// AP_MOTORS_FRAME_DECA_ENABLED #if/#endif bracketing it at 1241/1288).
// **THIS IS THE LAST OF THE SEVEN REAL setup_*_matrix FRAME-CLASS
// FUNCTIONS IN THIS PORT'S SCOPE** - see updated "DEFERRED FUTURE PHASES"
// below. The real switch(frame_type) has exactly TWO explicit case
// labels covering THREE enumerator values - PLUS (1245), and a combined
// `case MOTOR_FRAME_TYPE_X: case MOTOR_FRAME_TYPE_CW_X:` fall-through
// (1258-1259) - plus a real SIMPLE default (1281-1282: `default: // deca
// frame class does not support this frame type\n return false;`),
// re-verified directly, matching setup_quad_matrix's/setup_hexa_matrix's/
// setup_octa_matrix's/setup_octaquad_matrix's/setup_dodecahexa_matrix's
// own simple defaults exactly, NOT setup_y6_matrix's own real productive
// fallback.
//
// THE REAL X/CW_X FALL-THROUGH - A GENUINELY DISTINCT STRUCTURAL PATTERN
// FROM EVERY PRIOR TICKET, RE-VERIFIED DIRECTLY AGAINST THE REAL SWITCH:
// unlike every earlier setup_*_matrix in this arc (where X and CW_X, when
// both present, are always two SEPARATE case labels with two DIFFERENT
// tables - e.g. setup_octaquad_matrix's own X_COR/CW_X_COR in CCP-005 are
// a genuinely different subset-scaling pair, and setup_quad_matrix's/
// setup_hexa_matrix's/setup_octa_matrix's/setup_octaquad_matrix's own
// plain X/CW_X cases each have their own distinct MotorDef table),
// setup_deca_matrix's own real source has X and CW_X share the EXACT SAME
// case body and the EXACT SAME 10-motor table, with a single combined
// `_frame_type_string = "X/CW_X"` label (not two separate strings) - a
// real, deliberate C++ fall-through
// (`case MOTOR_FRAME_TYPE_X: case MOTOR_FRAME_TYPE_CW_X: { ... }`), not a
// transcription error and not something that "should" be split into two
// differently-valued cases. Ported below as the direct, faithful C++
// equivalent: `case FrameType::X: case FrameType::CwX: { ... }`.
// motors_matrix_test.cpp's own dedicated test below constructs the port's
// own MotorsMatrix twice - once via FrameType::X, once via FrameType::CwX
// - and confirms both produce EXACTLY the same per-motor roll/pitch/yaw/
// test_order values, proving the shared table was faithfully reproduced
// rather than accidentally split into two different (wrong) tables.
//
// Deca's own frame types, transcribed exactly from the real source (deca
// = 10 motors; both PLUS and the shared X/CW_X table are plain
// add_motors() calls over a static 10-entry MotorDef table - no
// add_motors_raw()/direct add_motor() calls anywhere in this function,
// same simple shape as setup_dodecahexa_matrix's own):
//   PLUS (1245-1257): angles evenly spaced 36 degrees apart around the
//     full circle starting at 0, alternating CCW/CW yaw factors, testing
//     orders 1 through 10 in strict array order:
//     (0,CCW,1) (36,CW,2) (72,CCW,3) (108,CW,4) (144,CCW,5) (180,CW,6)
//     (-144,CCW,7) (-108,CW,8) (-72,CCW,9) (-36,CW,10).
//   X/CW_X (1258-1276, SHARED by both FrameType::X and FrameType::CwX):
//     same evenly-spaced-36-degrees pattern as PLUS, rotated 18 degrees,
//     same alternating CCW/CW yaw factors, testing orders 1 through 10 in
//     strict array order:
//     (18,CCW,1) (54,CW,2) (90,CCW,3) (126,CW,4) (162,CCW,5) (-162,CW,6)
//     (-126,CCW,7) (-90,CW,8) (-54,CCW,9) (-18,CW,10).
//
// FrameType-ENUM INVESTIGATION (this ticket's own explicit question,
// answered by re-reading the CURRENT enum directly before writing
// anything, and independently against the real header
// AP_Motors_Class.h): all three real enumerators this ticket's switch
// names - MOTOR_FRAME_TYPE_PLUS, MOTOR_FRAME_TYPE_X, MOTOR_FRAME_TYPE_
// CW_X - were CONFIRMED ALREADY PRESENT in this port's own FrameType enum
// (Plus/X from CCP-002's own setup_quad_matrix work, CwX likewise from
// CCP-002 - re-checked one by one against the enum's real declaration,
// not assumed from the ticket text). This ticket therefore adds ZERO new
// enumerators to FrameType - CONFIRMED, matching the ticket's own
// expectation.
//
// THE RUNNING ZERO-GROWTH COUNT, INDEPENDENTLY RE-VERIFIED AGAINST THIS
// FILE'S OWN BANNER HISTORY (per this ticket's own explicit instruction
// not to trust the ticket's summary as complete): CCP-003's own
// setup_hexa_matrix added zero new enumerators (see "CCP-003 ADDITION"
// above), and CCP-007's own setup_dodecahexa_matrix added zero new
// enumerators (see "CCP-007 ADDITION" above, including that ticket's own
// correction of an earlier miscount). CCP-008 (this ticket) is therefore
// the THIRD setup_*_matrix ticket in this arc to add zero new
// enumerators - re-counted directly against this file's own history
// (CCP-003, CCP-007, CCP-008), matching the ticket's own claim exactly
// this time (unlike CCP-007's own ticket text, which miscounted itself as
// "the first").
//
// NOT ported (same disclosed omission class as every other
// setup_*_matrix's own _mav_type write above): the real upstream also
// sets `_mav_type = MAV_TYPE_DECAROTOR;` unconditionally at the top of
// setup_deca_matrix (line 1244, before the switch) - pure MAVLink/GCS
// metadata, out of scope for the same reason as the other six
// _mav_type writes.
//
// CCP-009 ADDITION (setup_motors) - upstream AP_MotorsMatrix::setup_motors
// (AP_MotorsMatrix.cpp, real function lines 1290-1349, re-verified
// directly against the pinned worktree; the ticket's own guessed span
// matched exactly). **THIS CLOSES OUT AP_MotorsMatrix's OWN
// CONSTRUCTION-TIME CONFIGURATION SURFACE ENTIRELY** - see the updated
// "DEFERRED FUTURE PHASES" below. setup_motors is the dispatcher that
// routes a (motor_frame_class, motor_frame_type) pair to the correct one
// of the seven already-ported setup_*_matrix functions (CCP-002 through
// CCP-008) - see FrameClass's own class-body comment above and
// setup_motors()'s own method-level comment below for the full seven-step
// structure and the exhaustiveness investigation of every real
// motor_frame_class enumerator (confirming exactly seven of the real
// eighteen are genuine cases in this switch, and that the other eleven -
// UNDEFINED/HELI/TRI/SINGLE/COAX/TAILSITTER/HELI_DUAL/HELI_QUAD/
// SCRIPTING_MATRIX/6DOF_SCRIPTING/DYNAMIC_SCRIPTING_MATRIX - are handled
// by entirely separate AP_Motors-family classes, never by
// AP_MotorsMatrix::setup_motors itself).
//
// THE THREE REAL STRUCTURAL SURPRISES THIS TICKET RE-VERIFIED DIRECTLY
// (not trusted from the ticket's own summary): (1) the real `for (int8_t
// i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) remove_motor(i);` loop and
// `set_initialised_ok(false);` both run UNCONDITIONALLY before the
// switch, with no guard on any prior state - this is what makes
// re-running setup_motors with a different frame class/type a genuine
// full reset rather than an incremental merge; motors_matrix_test.cpp's
// own re-configuration test is built specifically to catch a port that
// forgot this and leaked stale motor slots from a prior call. (2)
// `normalise_rpy_factors();` runs UNCONDITIONALLY too, confirmed NOT
// inside an `if (success)` guard - on failure this normalises an
// all-zero, all-disabled motor set (harmless, since step 1 already
// cleared everything), and is ported faithfully rather than skipped as
// "obviously a no-op". (3) a genuine, disclosed QUIRK found while writing
// the re-configuration test: remove_motor's own real body (AP_MotorsMatrix.cpp
// lines 545-552, re-verified directly) clears motor_enabled and the four
// RPYT factor arrays ONLY - it never resets _test_order. This means the
// "no stale state" guarantee setup_motors's own remove-all-then-rebuild
// structure provides is real and complete for motor_enabled/roll/pitch/
// yaw/throttle, but test_order on a slot the new frame table does not
// repopulate can legitimately retain a stale value from a PRIOR
// configuration - upstream's own real behavior (test_order only matters
// for motor-test sequencing among ENABLED motors, so this is harmless in
// practice), not a port gap. See setup_motors()'s own method-level
// comment below and motors_matrix_test.cpp's own re-configuration test
// for the full detail.
//
// NOT ported (same disclosed omission class as every setup_*_matrix's
// own _mav_type write above): the real upstream also writes `_mav_type =
// MAV_TYPE_GENERIC;` inside setup_motors's own `default:` branch - pure
// MAVLink/GCS metadata, out of scope for the same reason as the other
// seven _mav_type writes.
//
// CCP-011 ADDITION (check_for_failed_motor) - upstream
// AP_MotorsMatrix::check_for_failed_motor, real function body lines
// 414-461 (re-verified directly against the pinned worktree; the
// ticket's own guessed span, 414-457, undershot the real end by 4 lines
// - the real closing brace is at line 461, not 457). The real contract
// comment immediately above (lines 406-413) documents this is intended
// to run immediately after output_armed_stabilizing (not yet built in
// this port - see updated "DEFERRED FUTURE PHASES" below), with
// throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj.
//
// THE REAL SIX-STEP STRUCTURE, TRANSCRIBED EXACTLY (re-verified line by
// line against the real source, not trusted from the ticket's own
// summary):
//   1. alpha = dt_s / (dt_s + 0.5f) - a real, non-standard low-pass
//      filter alpha formula specific to THIS function, deliberately NOT
//      substituted with this port's own generic
//      math::calc_lowpass_alpha_dt helper (used elsewhere in this port,
//      e.g. fwcpp::filter::LowPassFilter) even though the two serve a
//      similar purpose.
//   2. For every enabled motor, in a first standalone loop: a plain
//      exponential filter update, thrust_rpyt_out_filt_[i] += alpha *
//      (thrust_rpyt_out_[i] - thrust_rpyt_out_filt_[i]).
//   3. A SECOND, separate loop over enabled motors computes rpyt_high
//      (the max filtered value seen so far), rpyt_sum (running total),
//      and number_motors (count). While a NEW rpyt_high is recorded,
//      motor_lost_index_ is updated to the CURRENT motor's index, but
//      ONLY if !thrust_boost_ - re-verified this exact gating condition
//      directly, matching upstream's own comment ("hold motor lost
//      index constant while thrust boost is active").
//   4. thrust_balance = (rpyt_sum > 0.1f) ? (rpyt_high * number_motors /
//      rpyt_sum) : 1.0f - re-verified the exact 0.1f threshold and the
//      1.0f fallback.
//   5. Real hysteresis, confirmed to be TWO SEPARATE SEQUENTIAL `if`
//      statements, NOT an if/else (re-verified directly - transcribed
//      faithfully rather than "cleaned up"): is_corotating =
//      (active_frame_type_ == FrameType::XCor || active_frame_type_ ==
//      FrameType::CwXCor) (reusing CCP-005's own enumerators verbatim).
//      First `if`: if (number_motors >= 6 && thrust_balance >= 1.5f &&
//      thrust_balanced_ && !is_corotating) thrust_balanced_ = false.
//      Second, independent `if`: if (thrust_balance <= 1.25f &&
//      !thrust_balanced_) thrust_balanced_ = true.
//   6. Final: if ((throttle_thrust_max * thr_lin_.get_compensation_gain(
//      air_density_ratio) > throttle_thrust_best_plus_adj) && (rpyt_high
//      < 0.9f) && thrust_balanced_) thrust_boost_ = false - re-verified
//      this exact three-way AND and the 0.9f threshold, and that this is
//      the ONLY place in the real function thrust_boost is ever
//      assigned - NEVER set true here (see motors_matrix_test.cpp's own
//      dedicated "never re-arms" test).
//
// A REAL, CONFIRMED UPSTREAM BUG - FIXED, NOT REPRODUCED, PER THIS
// PROJECT'S OWN "port fixes bugs, not upstream, register every
// divergence" POLICY: real upstream's own `_motor_lost_index`
// (AP_MotorsMatrix.h line 150) has NO in-class default member
// initializer, and AP_MotorsMatrix's own constructor (AP_MotorsMatrix.h
// lines 17-24, re-verified directly) only delegates to
// AP_MotorsMulticopter(speed_hz) - it never assigns `_motor_lost_index`
// either. Real upstream therefore reads this uint8_t as
// indeterminate/uninitialized memory until check_for_failed_motor itself
// first writes it - a real, latent bug (an uninitialized-read), not an
// intentional design choice. This port's own motor_lost_index_ is
// instead given a real, DEFINED initial value of 0 (see the private
// member declaration below) - confirmed harmless for this ticket's own
// scope, since every real use of _motor_lost_index elsewhere in upstream
// (AP_MotorsMatrix::get_lost_motor(), read by a higher-level thrust-boost
// consumer this port has not built) is itself gated behind thrust_boost
// being true first, and thrust_boost_ starts false (AP_Motors_Class.cpp
// lines 54-55, re-verified directly) - so a defined-but-arbitrary initial
// motor_lost_index_ value of 0 can never be observed as a "real"
// lost-motor index before check_for_failed_motor itself has run at least
// once and set thrust_boost_ true via some future caller. Disclosed here
// and in the commit message, per policy - NOT a silent fix.
//
// active_frame_type_ TRACKING - DESIGN DECISION: real upstream's
// _active_frame_type is written by AP_MotorsMatrix::init() (line 27) and
// AP_MotorsMatrix::set_frame_class_and_type() (line 137) - re-verified
// directly that the ALREADY-PORTED setup_motors (CCP-009, real function
// lines 1290-1349) does NOT itself write _active_frame_type anywhere;
// only those two other, still-unported functions do. Modifying the
// already-verified, faithfully-transcribed setup_motors to ALSO record
// active_frame_type_ would therefore not match what real upstream's own
// setup_motors does. This ticket instead adds a standalone
// set_active_frame_type() setter (see public accessors below) - tests
// call it directly, matching upstream's own real init()/
// set_frame_class_and_type() as the true (still-unported) setters of
// this state. active_frame_type_ itself is given a defined default of
// FrameType::Plus (upstream's own _active_frame_type field has no
// in-class initializer either - a second, lower-stakes instance of the
// same indeterminate-member pattern as _motor_lost_index above, though
// not one the ticket specifically asked to fix; giving it a real default
// here just continues this class's own established convention of never
// leaving a member indeterminate, matching every other field in this
// class).
//
// thr_lin_ MEMBER - this port's own MotorsMatrix had no
// ThrustLinearization (CCP-010) dependency before this ticket; real
// upstream's own AP_MotorsMulticopter (the base class) owns `thr_lin`,
// and AP_MotorsMatrix inherits access to it. This port has no such base
// class yet, so a ThrustLinearization instance is added directly as a
// MotorsMatrix member instead (thr_lin_ below) - the smallest change
// that lets this ticket's own get_compensation_gain() call be real
// rather than stubbed. ThrustLinearization's own deleted copy
// constructor/assignment (see thrust_linearization.hpp) makes
// MotorsMatrix itself non-copyable as a result - confirmed harmless: no
// existing test or caller copies a MotorsMatrix instance.
//
// PARAMETER SHAPE - check_for_failed_motor(throttle_thrust_best_plus_adj,
// throttle_thrust_max, dt_s, air_density_ratio): dt_s and
// air_density_ratio are explicit parameters per this ticket's own
// instruction and CCP-010's own established explicit-parameter
// convention (no AHRS/dt-source singleton - ADR-0012). throttle_thrust_max
// is ALSO taken as an explicit parameter, beyond what the ticket's own
// suggested signature named: real upstream's step 6 reads
// `_throttle_thrust_max`, a real AP_MotorsMulticopter-level member this
// port has not built yet (part of the deferred output-stage
// infrastructure - see updated "DEFERRED FUTURE PHASES" below) -
// surfacing it as an explicit caller-supplied parameter here, rather
// than inventing a placeholder member or hardcoding a value, matches
// this file's own established pattern (see ThrustLinearization's own
// explicit-parameter treatment of AP::battery()/AP::ahrs() state it does
// not own either).
//
// TEST-ONLY GAP, DISCLOSED: output_armed_stabilizing (not yet built -
// see updated "DEFERRED FUTURE PHASES" below) is upstream's real, sole
// writer of thrust_rpyt_out_ before check_for_failed_motor's own real
// call-order contract expects it to run. Since this port has no
// output_armed_stabilizing yet, a bounds-checked set_thrust_rpyt_out()
// setter is added purely so tests can populate thrust_rpyt_out_ directly
// - a real, temporary gap a LATER ticket (the one that actually builds
// output_armed_stabilizing) closes by computing and writing these values
// for real before calling this function, exactly matching upstream's own
// call-order contract.
//
// CCP-012 ADDITION (set_actuator_with_slew + actuator_spin_up_to_ground_idle)
// - upstream AP_MotorsMulticopter.cpp (re-verified directly against the
// pinned worktree): set_actuator_with_slew's real function body runs lines
// 480-503 (ticket's own guessed 480-503 matched exactly), and
// actuator_spin_up_to_ground_idle's real body runs lines 511-513 (ticket's
// own guessed span matched exactly too). Both are real, self-contained pure
// functions of AP_MotorsMulticopter, the still-not-built base class this
// port has no equivalent of yet - added directly to MotorsMatrix instead,
// matching CCP-011's own precedent of adding output-stage methods with
// nowhere else to live.
//
// REAL FORMULAS, TRANSCRIBED EXACTLY (re-verified directly against the
// source above, not trusted from the ticket's own transcription):
//   set_actuator_with_slew(actuator_output&, input, slew_up_time,
//   slew_dn_time, dt_s):
//     output_slew_limit_up = 1.0f;                 // "no slew limit" default
//     output_slew_limit_dn = 0.0f;                 // "no slew limit" default
//     if (is_positive(slew_up_time)) {
//       output_delta_up_max = dt_s / constrain_value(slew_up_time, 0, 0.5);
//       output_slew_limit_up = constrain_value(actuator_output +
//                                               output_delta_up_max, 0, 1);
//     }
//     if (is_positive(slew_dn_time)) {
//       output_delta_dn_max = dt_s / constrain_value(slew_dn_time, 0, 0.5);
//       output_slew_limit_dn = constrain_value(actuator_output -
//                                               output_delta_dn_max, 0, 1);
//     }
//     actuator_output = constrain_value(input, output_slew_limit_dn,
//                                        output_slew_limit_up);
//   actuator_spin_up_to_ground_idle(spin_up_ratio, spin_min):
//     return constrain_value(spin_up_ratio, 0, 1) * spin_min;
//
// THE CURRENT-OUTPUT-VS-DESTINATION BOUNDING PITFALL - copter-rust's own
// COP-004 investigation found this first (crates/ap-motors/src/output.rs:
// "the limits are computed from the *current* output, so they bound the
// step rather than the destination"; COP-004's own tracker notes, 2026-08-25
// entry: "the sweep runs six iterations per case because a port that
// bounded the destination agrees on iteration one and diverges on iteration
// two"). Independently re-verified here directly against the real C++
// source: both output_slew_limit_up/_dn above are computed from
// `actuator_output` - the value of the OUTPUT parameter BEFORE this call
// mutates it - never from `input`, the destination this call is trying to
// reach. A caller that runs this once per control-loop tick, feeding the
// same variable back in as actuator_output each time, gets a genuine
// per-step ramp: the "current" value call N reads is exactly what call N-1
// itself wrote. A port that instead computed either limit relative to
// `input` would produce a DIFFERENT trajectory toward a distant target -
// motors_matrix_test.cpp's own dedicated multi-iteration test (several
// calls toward one large fixed target, the trajectory checked call-by-call
// against the real step formula, not just the final value) is built
// specifically to prove this - see that test's own comments; a single-call
// test cannot tell the two interpretations apart as reliably as watching
// the actual step size hold constant across repeated calls.
//
// THE RESOLVED "NO SHUT_DOWN CHECK" NON-BUG - also from COP-004's own
// investigation ("Resolved, not a bug: set_actuator_with_slew has no
// SHUT_DOWN check even though the comment above it says slew limiting is
// skipped in that state. output_to_motors assigns zero to the actuator
// directly in SHUT_DOWN and never calls the function..."), independently
// re-verified here directly: the real comment immediately above
// set_actuator_with_slew (AP_MotorsMulticopter.cpp lines 476-479) does say
// "no slew limiting while in SHUT_DOWN to allow immediate motor
// de-energisation", but the real function BODY (lines 480-503) has no
// _spool_state/SpoolState reference anywhere - re-confirmed by reading
// every line. The comment documents set_actuator_with_slew's CALL-SITE
// CONTRACT (real upstream output_to_motors's own SHUT_DOWN case sets
// `_actuator[i] = 0.0f` directly and skips calling this function at all in
// that state), not something the function itself has to enforce. Since
// output_to_motors and the SpoolState machine are still unbuilt in this
// port (see updated "DEFERRED FUTURE PHASES" below - output_to_pwm's own
// real SpoolState dependency is the same reason it is excluded from this
// ticket too), set_actuator_with_slew below is ported as the real,
// unconditional pure function it is - NO SpoolState parameter, NO internal
// check added. A future ticket that builds output_to_motors is the one
// responsible for skipping this call entirely in SHUT_DOWN, exactly as real
// upstream does.
//
// spin_min PARAMETER CORRECTION (this ticket's own investigation
// overriding an earlier, wrong assumption of its own ticket text): real
// upstream's actuator_spin_up_to_ground_idle calls
// `thr_lin.get_spin_min()`. This port's own ThrustLinearization (CCP-010,
// thrust_linearization.hpp) does NOT have any get_spin_min() accessor and
// does NOT own spin_min as internal state at all - confirmed directly by
// reading that file: ThrustLinParams::spin_min is a plain public field on a
// separate, caller-owned struct, passed as an explicit parameter to every
// ThrustLinearization method (thrust_to_actuator, actuator_to_thrust,
// etc.). The correct port of this one-line formula therefore takes
// spin_min directly as its own second explicit float parameter and never
// touches thr_lin_ at all - it needs no ThrustLinearization dependency,
// unlike check_for_failed_motor's real thr_lin_.get_compensation_gain()
// call (CCP-011).
//
// PARAMETER SHAPE, BOTH FUNCTIONS - explicit parameters throughout, per
// ADR-0012 and every prior output-stage ticket's own established
// convention: real upstream reads _slew_up_time/_slew_dn_time/_dt_s as
// AP_MotorsMulticopter member state (GCS-tunable AP_Float parameters this
// port has not built, and a dt source ADR-0012 forbids reading from a
// singleton) and _spin_up_ratio as spool-state-machine output this port has
// not built either - all four become explicit parameters instead.
// set_actuator_with_slew(actuator_output&, input, slew_up_time,
// slew_dn_time, dt_s) mutates its first parameter BY REFERENCE, exactly
// matching upstream's own `float&` reference parameter (the alternative -
// returning the new value and requiring the caller to assign it - was
// considered and rejected: it would silently allow a caller to compute the
// new value and then forget to write it back, whereas the reference form
// makes the mutation happen unconditionally, matching upstream's real
// contract exactly and matching COP-004's own `&mut f32` choice on the Rust
// side).
//
// STATIC METHODS - DESIGN DECISION: neither function reads or writes any
// MotorsMatrix instance state (no motor_enabled_/roll_factor_/thr_lin_/etc.
// - re-confirmed against the real formulas above), so both are added as
// `static` member functions rather than ordinary instance methods. This
// still satisfies the ticket's own instruction to add them "to
// fwcpp::motors::MotorsMatrix" (matching CCP-011's own precedent of adding
// output-stage methods directly to MotorsMatrix, in the absence of a
// not-yet-built AP_MotorsMulticopter-equivalent base class) without forcing
// a caller to construct a live MotorsMatrix instance just to call a
// function that would never touch one.
//
// AP_MOTORS_SLEW_TIME_DEFAULT - real default, re-verified directly
// (AP_MotorsMulticopter.h line 21): 0.0f, for BOTH MOT_SLEW_UP_TIME and
// MOT_SLEW_DN_TIME - i.e. slew limiting is OFF in both directions by
// default. No AP_Param-style tunable is added for this (this port has
// never used AP_Param for pure numeric tuning constants - see
// thrust_linearization.hpp's own "NO AP_Param" precedent, CCP-010) - the
// real default value is only asserted directly in a test.
//
// CCP-013 ADDITION (output_logic, PART 1 ONLY) - upstream
// AP_MotorsMulticopter::output_logic, real function body lines 591-884
// (~294 lines), re-verified directly against the pinned worktree. This
// ticket ports ONLY lines 591-768 (~60%): the safety preamble
// (597-622), SpoolState::SHUT_DOWN (633-663), and SpoolState::GROUND_IDLE
// (665-768, including its own nested switch on DesiredSpoolState). The
// remaining three SpoolState cases (SPOOLING_UP/THROTTLE_UNLIMITED/
// SPOOLING_DOWN, real lines 769-884) are explicitly OUT OF SCOPE - a
// separate, deliberately deferred future ticket (CCP-014 or similar),
// since they depend on real, not-yet-built current-limiting
// (get_current_limit_max_throttle) and filtered-throttle (get_throttle)
// infrastructure this ticket's own scope does not need. Fourth ticket of
// the copter-cpp effort's AP_Motors OUTPUT-STAGE phase, after CCP-010
// (ThrustLinearization), CCP-011 (check_for_failed_motor), and CCP-012
// (set_actuator_with_slew / actuator_spin_up_to_ground_idle). This is,
// by a wide margin, the most complex single function this whole effort
// has ported so far - a genuine, safety-critical real-time state
// machine with many external dependencies this port has never built
// (vehicle-level armed()/get_interlock(), an attitude-controller `limit`
// flags object, a pre-takeoff `spoolup_block` gate) - every one of them
// is an explicit function parameter or explicit output below, per
// ADR-0012, rather than any attempt to build the real vehicle-level
// infrastructure behind them.
//
// REUSED, INDEPENDENTLY RE-VERIFIED INVESTIGATION FROM copter-rust'S
// COP-004
//
// COP-004 (crates/ap-motors/src/spool.rs, plane-fw-rust) already ported
// this exact 294-line function to Rust with an unusually thorough
// verification methodology (8 scripted flights, 25,200 iterations at
// 400 Hz, all bit-exact against upstream). Three findings reused here
// after independently re-checking each against the real C++ source:
//   1. Step-by-step, not endpoint, testing is essential for a state
//      machine (COP-004's own words: "A state machine can agree at the
//      endpoints and disagree in the middle... only a per-step
//      comparison sees that.") - motors_matrix_test.cpp's own new tests
//      below drive the machine through many successive calls and assert
//      intermediate state at each step, not just final outcomes.
//   2. The real spool_up_time write-back: real upstream's own safety
//      preamble clamps a too-short _spool_up_time back into the AP_Float
//      PARAMETER ITSELF (`_spool_up_time.set(...)`, real line 620), not
//      a local copy - reproduced here by taking spool_up_time as `float&`
//      and genuinely mutating it when the clamp fires, matching
//      COP-004's own `&mut f32` choice on the Rust side.
//   3. get_throttle()'s own filtered-vs-raw distinction is a Part 2
//      concern (SPOOLING_UP/THROTTLE_UNLIMITED both call it, real lines
//      791/831) - not called anywhere in this ticket's own Part 1 scope,
//      noted here for context only.
//
// TWO SUSPECTED UPSTREAM BUGS - ONE CONFIRMED FALSE, ONE CONFIRMED TRUE
// (see the private member declarations below for the full writeup of
// each): this ticket's own text suspected `_spool_state`/`_spool_desired`
// had the same uninitialized-until-first-write bug class as CCP-011's
// own `_motor_lost_index` finding. Re-verified directly and found FALSE:
// real `AP_Motors::AP_Motors(uint16_t)` (AP_Motors_Class.cpp lines
// 31-38) explicitly initializes BOTH via its own member-initializer list
// (`_spool_desired(DesiredSpoolState::SHUT_DOWN),
// _spool_state(SpoolState::SHUT_DOWN)`), and AP_MotorsMulticopter's own
// constructor delegates to that base constructor - so both are correctly
// SHUT_DOWN from construction in real upstream, contrary to this
// ticket's own suspicion. A DIFFERENT, genuinely confirmed instance of
// the same bug class turned up instead while investigating: six OTHER
// members (`_disarm_safe_timer`/`_spin_up_ratio`/`_throttle_thrust_max`/
// `_idle_time`/`_spin_up_complete`, AP_MotorsMulticopter.h, plus
// `_thrust_boost_ratio`, AP_Motors_Class.h) have NO in-class initializer
// and are assigned by NEITHER constructor - confirmed low-severity for
// the same reason CCP-011's own finding was (each is written before any
// real read reaches it, in the realistic call order), but fixed and
// disclosed the same way: all six get real, DEFINED initial values in
// this port rather than being left indeterminate.
//
// OUTPUT SHAPE - real upstream's own `limit.set_all(true)` (a five-flag
// `AP_Motors_limit` struct roll/pitch/yaw/throttle_lower/throttle_upper
// this port has not built) becomes a single explicit `bool&
// limits_all_engaged` output parameter instead - this ticket's own Part
// 1 scope only ever calls `set_all(true)` (never per-axis, never
// `false` - that only happens in the three out-of-scope states), so a
// single bool captures every real behavior Part 1 exercises; a future
// ticket porting the SPOOLING_UP/THROTTLE_UNLIMITED/SPOOLING_DOWN cases
// (which call `set_all(false)`) is the natural place to decide whether
// the full five-flag struct is worth building then. Real upstream's own
// `set_spoolup_block(true)` (a stateful setter on a vehicle-level flag
// this port does not own) becomes a `bool& should_set_spoolup_block`
// output instead, raised to `true` only on the exact call where
// `_spin_up_complete` first transitions to true - `get_spoolup_block()`
// (the corresponding getter) becomes the plain `bool spoolup_block`
// INPUT parameter.
//
// THE REAL SAME-CALL spoolup_block READ-AFTER-WRITE - a genuine, easy-to-
// miss finding independently re-verified here (AP_Motors_Class.h lines
// 127-128, re-verified directly: get_spoolup_block()/set_spoolup_block()
// are a plain bool getter/setter pair with no other side effect): real
// upstream calls `set_spoolup_block(true)` and then, a few lines below
// IN THE SAME output_logic CALL, reads `get_spoolup_block()` to decide
// the SPOOLING_UP transition - so that transition NEVER fires on the
// very call that first raises the block, it always observes the
// just-written `true`. A naive port that read the `spoolup_block`
// PARAMETER (the value as it was going INTO this call) for that same
// check would take the transition one call too early. This is exactly
// the "transition taken one iteration early" failure mode COP-004's own
// tracker notes warn a state machine can hide at the endpoints and only
// show step-by-step - fixed here with a local `spoolup_block_now`
// variable that mirrors the synchronous write, matching real upstream's
// actual observed behavior exactly (see output_logic's own inline
// comment at the point of use).
//
// PARAMETER/OUTPUT SHAPE, FULL SIGNATURE - flat explicit parameters
// throughout (not a bundling struct like ThrustLinParams/BatteryVoltage,
// CCP-010), matching check_for_failed_motor's/set_actuator_with_slew's
// own precedent of flat parameter lists and this ticket's own text,
// which itself enumerates the dependencies as a flat list: armed,
// interlock, disarm_disable_pwm, safe_time, spool_up_time (float&,
// genuinely mutated - see finding #2 above), spool_down_time, spin_arm,
// idle_time_delay_s, spin_min, spoolup_block, PLUS dt_s (needed
// throughout for every ramp/timer but not itself named in the ticket's
// own dependency list), PLUS the two explicit bool& outputs
// (limits_all_engaged, should_set_spoolup_block) appended at the end,
// mirroring set_actuator_with_slew's own established "mutate an output
// parameter by reference" convention rather than returning a small
// struct. There is no single obviously-correct shape for an
// eleven-input/two-output function - this is the chosen one, stated
// explicitly per this ticket's own request.
//
// DEFERRED FUTURE PHASES (named explicitly, not silently omitted):
//   1. Remaining frame tables - setup_quad_matrix (line 576) is DONE as of
//      CCP-002, setup_hexa_matrix (line 775) is DONE as of CCP-003,
//      setup_octa_matrix (line 854) is DONE as of CCP-004,
//      setup_octaquad_matrix (line 973) is DONE as of CCP-005,
//      setup_y6_matrix (line 1191) is DONE as of CCP-006,
//      setup_dodecahexa_matrix (line 1140) is DONE as of CCP-007,
//      setup_deca_matrix (line 1242) is DONE as of CCP-008, and
//      setup_motors (line 1290) is DONE as of CCP-009 (see
//      "CCP-002 ADDITION"/"CCP-003 ADDITION"/"CCP-004 ADDITION"/
//      "CCP-005 ADDITION"/"CCP-006 ADDITION"/"CCP-007 ADDITION"/
//      "CCP-008 ADDITION"/"CCP-009 ADDITION" above). **THIS CLOSES OUT
//      AP_MotorsMatrix's OWN CONSTRUCTION-TIME CONFIGURATION SURFACE
//      ENTIRELY** - every real setup_*_matrix frame-class function is
//      ported, and the dispatcher that chooses among them by frame CLASS
//      is also ported. No AP_MotorsMatrix construction-time configuration
//      work remains unported.
//      copter-rust's own COP-005 finding that Y6's `default:` switch
//      branch is productive - answering for 24 of the real 64 upstream
//      frame configurations - was independently re-verified and resolved
//      by CCP-006 above; not deferred further. (COP-005's other two
//      findings - the X8 co-rotating scaling and its real float-vs-double
//      pitfall - were independently re-verified and resolved by CCP-005;
//      also not deferred further.)
//   2. Output stage - check_for_failed_motor is DONE as of CCP-011 (see
//      "CCP-011 ADDITION" above), set_actuator_with_slew /
//      actuator_spin_up_to_ground_idle are DONE as of CCP-012 (see
//      "CCP-012 ADDITION" above), and output_logic's own SpoolState enum
//      plus its SHUT_DOWN/GROUND_IDLE cases (real lines 591-768, "PART 1")
//      are DONE as of CCP-013 (see "CCP-013 ADDITION" above). The
//      remaining three SpoolState cases - SPOOLING_UP/THROTTLE_UNLIMITED/
//      SPOOLING_DOWN, real lines 769-884, "PART 2" - are DONE as of
//      CCP-014 (see "CCP-014 ADDITION" - search this file), and
//      output_to_pwm (AP_MotorsMulticopter.cpp line 457) is DONE as of
//      CCP-015 (see "CCP-015 ADDITION" above). output_to_motors (the real
//      per-motor dispatcher that calls output_to_pwm once per motor,
//      needing motor_enabled_/rc_write-equivalent output plumbing this
//      port has not built), output_armed_stabilizing (the real per-motor
//      mixing algorithm, ~190 real lines - the single largest remaining
//      function in this whole output-stage effort), thrust_compensation,
//      and disable_yaw_torque all remain separate, deliberately deferred
//      future phases, NOT started by this ticket - output_to_motors
//      specifically also needs real PWM output plumbing beyond just the
//      now-ported SpoolState enum and output_to_pwm itself. Actuator
//      slew-rate limiting itself is NOT part of that missing infrastructure, since
//      CCP-012 already ported it as the unconditional pure function it
//      really is (see "CCP-012 ADDITION" above for why it needs no
//      SpoolState awareness of its own). The full thrust-boost MECHANISM
//      itself (whatever sets thrust_boost_ true from a real motor-failure
//      trigger, and whatever reacts to get_lost_motor()) is also
//      deferred - CCP-011 only ported the thrust_boost_-reading/
//      off-switching logic INSIDE check_for_failed_motor, and CCP-013
//      only ported output_logic's own thrust_boost_/thrust_boost_ratio_
//      RESETS inside SHUT_DOWN/GROUND_IDLE, not the rest of the
//      mechanism. This is also where add_motor_num()'s real SRV_Channels
//      registration belongs (see NO-OP above).
//   3. set_throttle_factor/set_update_rate/set_frame_class_and_type/
//      output_test_num/_output_test_seq/get_factors - small real
//      accessors/setters not needed by this ticket's own core scope; add
//      only when a real future test/caller needs one.
//
// LITERAL-TYPING NOTE (updated for CCP-005): every literal in this file
// EXCEPT `kOctaquadCorotatingScaleFactor` is already explicitly
// float-suffixed (0.5f, 0.0f, 0.7981f, ...) in both upstream and here, so
// -fsingle-precision-constant has no observable effect on the REST of
// this file's own arithmetic. `kOctaquadCorotatingScaleFactor` is the one
// exception, and is handled by writing it as an explicit `0.9f` literal
// directly (see "THE REAL FLOAT-VS-DOUBLE PITFALL" above) rather than by
// linking `fwcpp_upstream_flags` - so this module still does not link
// that target, matching ap-compass/ap-gps's own precedent of not linking
// it for header-only INTERFACE modules with no literal that actually
// needs it.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/motors/thrust_linearization.hpp>

namespace fwcpp::motors {

// AP_MOTORS_MAX_NUM_MOTORS's real, fully-resolved value for this port's
// SITL target - see file banner above for the full investigation.
inline constexpr std::size_t kMaxNumMotors = 32;

// AP_MOTORS_MATRIX_YAW_FACTOR_CW/_CCW (AP_MotorsMatrix.h lines 10-11,
// re-verified directly: `#define AP_MOTORS_MATRIX_YAW_FACTOR_CW -1` /
// `_CCW 1`) - ported as float (matching MotorDef::yaw_factor's real
// type) rather than int, since every real caller uses these only in a
// float-typed yaw_factor position.
inline constexpr float kYawFactorCw = -1.0f;
inline constexpr float kYawFactorCcw = 1.0f;

// CCP-005: AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR
// (AP_Motors_config.h lines 64-66, re-verified directly: `#define
// AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR 0.9`, a bare
// UNSUFFIXED double literal in the real header) - ported here as an
// explicit `0.9f` `float` literal, NOT the bare `0.9` upstream's own
// header spells it as. See file banner's "THE REAL FLOAT-VS-DOUBLE
// PITFALL" section for the full design-decision writeup: upstream's real
// EFFECTIVE behavior is float-precision only because its entire build
// applies -fsingle-precision-constant, and this port's own ap-motors
// module does not link the fwcpp_upstream_flags target that carries that
// flag, so an explicit `f` suffix here is what reproduces upstream's real
// numeric result rather than silently computing in double precision.
inline constexpr float kOctaquadCorotatingScaleFactor = 0.9f;
static_assert(kOctaquadCorotatingScaleFactor < 1.0f,
              "kOctaquadCorotatingScaleFactor must be less than 1.0 - matches upstream's own real "
              "static_assert on AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR (AP_MotorsMatrix.cpp line 1102)");

// MotorsMatrix - port of AP_MotorsMatrix's core per-motor factor storage
// and arithmetic (CCP-001). See file banner for exactly what upstream
// behavior this reproduces and what is deferred.
class MotorsMatrix {
public:
    // Upstream AP_MotorsMatrix::MotorDef (AP_MotorsMatrix.h line 82) -
    // used by the symmetric-frame add_motors() loop.
    struct MotorDef {
        float angle_degrees;
        float yaw_factor;
        std::uint8_t testing_order;
    };

    // Upstream AP_MotorsMatrix::MotorDefRaw (AP_MotorsMatrix.h line 96) -
    // used by the raw-factor add_motors_raw() loop. Deliberately has no
    // throttle_factor field, matching upstream's own comment that the
    // omitted throttle factor "is only used in the scripting binding, not
    // in the static motors."
    struct MotorDefRaw {
        float roll_fac;
        float pitch_fac;
        float yaw_fac;
        std::uint8_t testing_order;
    };

    // CCP-002/CCP-003/CCP-004: port of upstream's real
    // `enum motor_frame_type` (AP_Motors_Class.h lines 78-99) - a SINGLE
    // enum genuinely shared across every real setup_*_matrix function
    // (setup_quad_matrix, setup_hexa_matrix, setup_octa_matrix, ...);
    // which function runs is chosen separately by the real
    // `motor_frame_class` enum (lines 54-72), not by this one - see file
    // banner's "CCP-003 ADDITION" for the full enum-sharing
    // investigation. This port's own FrameType therefore grows
    // incrementally as each frame-class ticket lands and is reused
    // verbatim by every setup_*_matrix method below, exactly mirroring
    // that real design - it does NOT get a second, function-specific
    // enum. Currently restricted to exactly the enumerators
    // setup_quad_matrix's, setup_hexa_matrix's, and setup_octa_matrix's
    // own real switch statements handle among them (NOT the full
    // upstream enum, which also has other frame-class-specific values no
    // setup function has been ported for yet - see file banner's
    // "CCP-002 ADDITION"/"CCP-003 ADDITION"/"CCP-004 ADDITION" for the
    // exhaustive real case lists this was checked against). NYT_PLUS/
    // NYT_X are deliberately absent - see file banner's "EXCLUDED" note.
    // CCP-003 added ZERO new enumerators: all five of setup_hexa_matrix's
    // real frame types (PLUS/X/H/DJI_X/CW_X) were already present from
    // CCP-002's setup_quad_matrix work, and setup_hexa_matrix(FrameType)
    // below reuses them directly. CCP-004 added EXACTLY ONE new
    // enumerator, `I` (appended below after `Y4`) - the only one of
    // setup_octa_matrix's seven real frame types not already present;
    // the other six (PLUS/X/V/H/DJI_X/CW_X) were re-confirmed already
    // present from CCP-002's work and are reused verbatim by
    // setup_octa_matrix(FrameType) below - see file banner's
    // "CCP-004 ADDITION" for the full investigation. Named/shaped per
    // this port's own house style for small state enums (e.g.
    // CalibrationState, fwcpp/airspeed/airspeed_sensor.hpp). CCP-005
    // added EXACTLY TWO new enumerators, `XCor`/`CwXCor` (appended below
    // after `I`) - the only two of setup_octaquad_matrix's nine real
    // frame types not already present; the other seven (PLUS/X/V/H/
    // CW_X/BF_X/BF_X_REV) were re-confirmed already present from
    // CCP-002's/CCP-004's own work and are reused verbatim by
    // setup_octaquad_matrix(FrameType) below - see file banner's
    // "CCP-005 ADDITION" for the full investigation. `XCor`/`CwXCor`
    // follow this enum's own established abbreviation convention exactly
    // (each upstream underscore-word boundary becomes a capitalized run
    // with no separator, e.g. `CwX` for CW_X) applied to X_COR/CW_X_COR.
    // CCP-006 added EXACTLY TWO new enumerators, `Y6B`/`Y6F` (appended
    // below after `CwXCor`) - MOTOR_FRAME_TYPE_Y6B/_Y6F
    // (AP_Motors_Class.h lines 86-87), both genuinely new to every
    // switch this port has ported so far - see file banner's
    // "CCP-006 ADDITION" for the full investigation, including the real
    // PRODUCTIVE `default:` fallback setup_y6_matrix(FrameType) below
    // reaches for every FrameType value OTHER than these two. CCP-007
    // added ZERO new enumerators - both of setup_dodecahexa_matrix's real
    // frame types (PLUS/X) were re-confirmed already present from
    // CCP-002's own work and are reused verbatim by
    // setup_dodecahexa_matrix(FrameType) below - see file banner's
    // "CCP-007 ADDITION" for the full investigation, including a
    // correction of the ticket's own (incorrect) claim that this would
    // be the first such zero-growth ticket in this arc - CCP-003 already
    // was. CCP-008 (setup_deca_matrix, the LAST of the seven real
    // setup_*_matrix functions) also added ZERO new enumerators - all
    // three of its real frame types (PLUS/X/CW_X, the latter two sharing
    // a real fall-through case, see file banner's "CCP-008 ADDITION")
    // were re-confirmed already present from CCP-002's own work - the
    // THIRD such zero-growth ticket in this arc (CCP-003, CCP-007,
    // CCP-008), independently re-counted against this file's own history
    // rather than trusted from any one ticket's own claim.
    enum class FrameType : std::uint8_t {
        Plus,
        X,
        BfX,
        BfXRev,
        DjiX,
        CwX,
        V,
        H,
        VTail,
        ATail,
        PlusRev,
        Y4,
        I,
        XCor,
        CwXCor,
        Y6B,
        Y6F,
    };

    // CCP-009: port of upstream's real `enum motor_frame_class`
    // (AP_Motors_Class.h lines 54-72, re-verified directly): MOTOR_FRAME_
    // UNDEFINED=0, MOTOR_FRAME_QUAD=1, MOTOR_FRAME_HEXA=2, MOTOR_FRAME_
    // OCTA=3, MOTOR_FRAME_OCTAQUAD=4, MOTOR_FRAME_Y6=5, MOTOR_FRAME_HELI=6,
    // MOTOR_FRAME_TRI=7, MOTOR_FRAME_SINGLE=8, MOTOR_FRAME_COAX=9,
    // MOTOR_FRAME_TAILSITTER=10, MOTOR_FRAME_HELI_DUAL=11, MOTOR_FRAME_
    // DODECAHEXA=12, MOTOR_FRAME_HELI_QUAD=13, MOTOR_FRAME_DECA=14,
    // MOTOR_FRAME_SCRIPTING_MATRIX=15, MOTOR_FRAME_6DOF_SCRIPTING=16,
    // MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX=17. Confirmed directly against
    // AP_MotorsMatrix::setup_motors's own real switch (AP_MotorsMatrix.cpp
    // lines 1290-1349, see setup_motors() below for the full dispatcher
    // investigation) that EXACTLY SEVEN of these eighteen real enumerators
    // are genuine cases in that switch - MOTOR_FRAME_QUAD, MOTOR_FRAME_
    // HEXA, MOTOR_FRAME_OCTA, MOTOR_FRAME_OCTAQUAD, MOTOR_FRAME_
    // DODECAHEXA, MOTOR_FRAME_Y6, MOTOR_FRAME_DECA - the exact seven this
    // port's own setup_*_matrix functions above already implement
    // (CCP-002 through CCP-008). Every other real enumerator (UNDEFINED,
    // HELI, TRI, SINGLE, COAX, TAILSITTER, HELI_DUAL, HELI_QUAD,
    // SCRIPTING_MATRIX, 6DOF_SCRIPTING, DYNAMIC_SCRIPTING_MATRIX) falls
    // into setup_motors's own real `default:` branch - NOT because
    // AP_MotorsMatrix has some other, unported case for them, but because
    // they are never real motor_frame_class values AP_MotorsMatrix's own
    // vehicle-level construction logic passes to THIS class at all: HELI/
    // HELI_DUAL/HELI_QUAD select AP_MotorsHeli(_Dual/_Quad), TRI selects
    // AP_MotorsTri, SINGLE/COAX select AP_MotorsSingle/AP_MotorsCoax,
    // TAILSITTER selects AP_MotorsTailsitter, SCRIPTING_MATRIX/6DOF_
    // SCRIPTING/DYNAMIC_SCRIPTING_MATRIX select the AP_MotorsMatrix_
    // Scripting family, and UNDEFINED is the real "never configured" zero
    // value - all real, entirely separate AP_Motors-family classes
    // instantiated at the vehicle level (AP_Motors_Class.h's own class
    // hierarchy), never routed through AP_MotorsMatrix::setup_motors at
    // all. This port's own FrameClass therefore contains ONLY the seven
    // real, already-ported enumerators - deliberately NOT a full mirror
    // of the real eighteen-value motor_frame_class, and NOT padded with
    // placeholder/commented-out entries for the eleven out-of-scope ones,
    // matching this ticket's own explicit scope. An out-of-range
    // FrameClass value (e.g. `static_cast<FrameClass>(255)`) exercises
    // setup_motors's own real `default:` branch exactly as any of those
    // eleven real-but-out-of-scope enumerators would.
    enum class FrameClass : std::uint8_t {
        Quad,
        Hexa,
        Octa,
        Octaquad,
        Dodecahexa,
        Y6,
        Deca,
    };

    // CCP-013: port of upstream's real `enum class SpoolState : uint8_t`
    // and `enum class DesiredSpoolState : uint8_t` (both declared on the
    // real `AP_Motors` base class, AP_Motors_Class.h lines 184-190 and
    // 171-175 respectively, re-verified directly against the pinned
    // worktree). Enumerator names/values/declared order are exact:
    // SpoolState::{ShutDown=0, GroundIdle=1, SpoolingUp=2,
    // ThrottleUnlimited=3, SpoolingDown=4} (five values);
    // DesiredSpoolState::{ShutDown=0, GroundIdle=1, ThrottleUnlimited=2}
    // (three values - real upstream has no "spooling" DESIRED state,
    // only the actual SpoolState has transitional values). Named in this
    // port's own PascalCase house style for small state enums (see
    // FrameType/FrameClass above and CalibrationState,
    // fwcpp/airspeed/airspeed_sensor.hpp), not upstream's SCREAMING_CASE.
    // See file banner's "CCP-013 ADDITION" for the full output_logic
    // (Part 1) investigation.
    enum class SpoolState : std::uint8_t {
        ShutDown,
        GroundIdle,
        SpoolingUp,
        ThrottleUnlimited,
        SpoolingDown,
    };

    enum class DesiredSpoolState : std::uint8_t {
        ShutDown,
        GroundIdle,
        ThrottleUnlimited,
    };

    // Upstream AP_Motors::initialised_ok()/set_initialised_ok() (see file
    // banner's GUARD design decision). Starts false, matching upstream's
    // real effective default before any successful init().
    [[nodiscard]] bool initialised_ok() const { return initialised_ok_; }
    void set_initialised_ok(bool val) { initialised_ok_ = val; }

    // add_motor_raw - upstream AP_MotorsMatrix::add_motor_raw (see file
    // banner for the exact ported formula and the add_motor_num()
    // NO-OP design decision).
    void add_motor_raw(std::int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, std::uint8_t testing_order,
                        float throttle_factor = 1.0f) {
        if (initialised_ok()) {
            // Do not allow motors to be set if the current frame type has
            // already initialised correctly - matches upstream exactly.
            return;
        }
        if (motor_num >= 0 && static_cast<std::size_t>(motor_num) < kMaxNumMotors) {
            const auto i = static_cast<std::size_t>(motor_num);
            motor_enabled_[i] = true;
            roll_factor_[i] = roll_fac;
            pitch_factor_[i] = pitch_fac;
            yaw_factor_[i] = yaw_fac;
            throttle_factor_[i] = throttle_factor;
            test_order_[i] = testing_order;
            // add_motor_num(motor_num) - disclosed no-op, see file banner.
        }
    }

    // add_motor (3-arg / symmetric position form) - upstream
    // AP_MotorsMatrix::add_motor(motor_num, angle_degrees, yaw_factor,
    // testing_order). Delegates to the 4-arg form with roll and pitch
    // angle inputs equal - see file banner: the two OUTPUT factors are
    // still asymmetric because only roll gets +90 before the cosine.
    void add_motor(std::int8_t motor_num, float angle_degrees, float yaw_factor, std::uint8_t testing_order) {
        add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
    }

    // add_motor (4-arg / asymmetric position form) - upstream
    // AP_MotorsMatrix::add_motor(motor_num, roll_factor_in_degrees,
    // pitch_factor_in_degrees, yaw_factor, testing_order). REAL, RE-
    // VERIFIED ASYMMETRY (see file banner): +90 applies to roll only.
    void add_motor(std::int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor,
                    std::uint8_t testing_order) {
        add_motor_raw(motor_num, std::cos(math::radians(roll_factor_in_degrees + 90.0f)),
                      std::cos(math::radians(pitch_factor_in_degrees)), yaw_factor, testing_order);
    }

    // remove_motor - upstream AP_MotorsMatrix::remove_motor: disables the
    // motor and zeros all four factors.
    void remove_motor(std::int8_t motor_num) {
        if (motor_num >= 0 && static_cast<std::size_t>(motor_num) < kMaxNumMotors) {
            const auto i = static_cast<std::size_t>(motor_num);
            motor_enabled_[i] = false;
            roll_factor_[i] = 0.0f;
            pitch_factor_[i] = 0.0f;
            yaw_factor_[i] = 0.0f;
            throttle_factor_[i] = 0.0f;
        }
    }

    // add_motors - upstream AP_MotorsMatrix::add_motors: trivial
    // index-order loop over MotorDef entries.
    void add_motors(const MotorDef* motor_defs, std::uint8_t num_motors) {
        for (std::uint8_t i = 0; i < num_motors; ++i) {
            add_motor(static_cast<std::int8_t>(i), motor_defs[i].angle_degrees, motor_defs[i].yaw_factor,
                      motor_defs[i].testing_order);
        }
    }

    // add_motors_raw - upstream AP_MotorsMatrix::add_motors_raw: trivial
    // index-order loop over MotorDefRaw entries.
    void add_motors_raw(const MotorDefRaw* motor_defs, std::uint8_t num_motors) {
        for (std::uint8_t i = 0; i < num_motors; ++i) {
            add_motor_raw(static_cast<std::int8_t>(i), motor_defs[i].roll_fac, motor_defs[i].pitch_fac, motor_defs[i].yaw_fac,
                          motor_defs[i].testing_order);
        }
    }

    // normalise_rpy_factors - upstream AP_MotorsMatrix::normalise_rpy_factors.
    // See file banner for the exact two-pass formula and the real
    // asymmetric throttle-vs-rpy clamping this reproduces faithfully.
    void normalise_rpy_factors() {
        float roll_fac = 0.0f;
        float pitch_fac = 0.0f;
        float yaw_fac = 0.0f;
        float throttle_fac = 0.0f;

        for (std::size_t i = 0; i < kMaxNumMotors; ++i) {
            if (motor_enabled_[i]) {
                roll_fac = std::max(roll_fac, std::fabs(roll_factor_[i]));
                pitch_fac = std::max(pitch_fac, std::fabs(pitch_factor_[i]));
                yaw_fac = std::max(yaw_fac, std::fabs(yaw_factor_[i]));
                throttle_fac = std::max(throttle_fac, std::max(0.0f, throttle_factor_[i]));
            }
        }

        for (std::size_t i = 0; i < kMaxNumMotors; ++i) {
            if (motor_enabled_[i]) {
                if (!math::is_zero(roll_fac)) {
                    roll_factor_[i] = 0.5f * roll_factor_[i] / roll_fac;
                }
                if (!math::is_zero(pitch_fac)) {
                    pitch_factor_[i] = 0.5f * pitch_factor_[i] / pitch_fac;
                }
                if (!math::is_zero(yaw_fac)) {
                    yaw_factor_[i] = 0.5f * yaw_factor_[i] / yaw_fac;
                }
                if (!math::is_zero(throttle_fac)) {
                    throttle_factor_[i] = std::max(0.0f, throttle_factor_[i] / throttle_fac);
                }
            }
        }
    }

    // setup_quad_matrix - CCP-002 port of upstream
    // AP_MotorsMatrix::setup_quad_matrix (AP_MotorsMatrix.cpp line 576).
    // Every case's angle/yaw-factor/testing-order values are transcribed
    // exactly from the real source - see file banner's "CCP-002 ADDITION"
    // for the full exhaustiveness/exclusion/default-branch investigation.
    // Returns false (upstream's own real return value) for any FrameType
    // not handled below - this port's FrameType enum only ever contains
    // handled values, so that path is unreachable through normal use, but
    // is kept to mirror upstream's real "not supported" semantics exactly
    // (e.g. against a future caller that widens the enum without updating
    // this switch).
    [[nodiscard]] bool setup_quad_matrix(FrameType frame_type) {
        frame_class_string_ = "QUAD";
        switch (frame_type) {
        case FrameType::Plus: {
            frame_type_string_ = "PLUS";
            static constexpr MotorDef motors[] = {
                {90.0f, kYawFactorCcw, 2},
                {-90.0f, kYawFactorCcw, 4},
                {0.0f, kYawFactorCw, 1},
                {180.0f, kYawFactorCw, 3},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::X: {
            frame_type_string_ = "X";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCcw, 1},
                {-135.0f, kYawFactorCcw, 3},
                {-45.0f, kYawFactorCw, 4},
                {135.0f, kYawFactorCw, 2},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::BfX: {
            // betaflight quad X order
            // see: https://fpvfrenzy.com/betaflight-motor-order/
            frame_type_string_ = "BF_X";
            static constexpr MotorDef motors[] = {
                {135.0f, kYawFactorCw, 2},
                {45.0f, kYawFactorCcw, 1},
                {-135.0f, kYawFactorCcw, 3},
                {-45.0f, kYawFactorCw, 4},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::BfXRev: {
            // betaflight quad X order, reversed motors
            frame_type_string_ = "X_REV";
            static constexpr MotorDef motors[] = {
                {135.0f, kYawFactorCcw, 2},
                {45.0f, kYawFactorCw, 1},
                {-135.0f, kYawFactorCw, 3},
                {-45.0f, kYawFactorCcw, 4},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::DjiX: {
            // DJI quad X order
            // see https://forum44.djicdn.com/data/attachment/forum/201711/26/172348bppvtt1ot1nrtp5j.jpg
            frame_type_string_ = "DJI_X";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCcw, 1},
                {-45.0f, kYawFactorCw, 4},
                {-135.0f, kYawFactorCcw, 3},
                {135.0f, kYawFactorCw, 2},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::CwX: {
            // "clockwise X" motor order. Motors are ordered clockwise from
            // front right matching test order
            frame_type_string_ = "CW_X";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCcw, 1},
                {135.0f, kYawFactorCw, 2},
                {-135.0f, kYawFactorCcw, 3},
                {-45.0f, kYawFactorCw, 4},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::V: {
            // Real, non-derived yaw factors - transcribed exactly from
            // upstream's own float literals (see file banner).
            frame_type_string_ = "V";
            static constexpr MotorDef motors[] = {
                {45.0f, 0.7981f, 1},
                {-135.0f, 1.0000f, 3},
                {-45.0f, -0.7981f, 4},
                {135.0f, -1.0000f, 2},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::H: {
            // H frame set-up - same as X but motors spin in opposite
            // directions
            frame_type_string_ = "H";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCw, 1},
                {-135.0f, kYawFactorCw, 3},
                {-45.0f, kYawFactorCcw, 4},
                {135.0f, kYawFactorCcw, 2},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::VTail: {
            /*
                Tested with: Lynxmotion Hunter Vtail 400
                - inverted rear outward blowing motors (at a 40 degree angle)
                - should also work with non-inverted rear outward blowing motors
                - no roll in rear motors
                - no yaw in front motors
                - should fly like some mix between a tricopter and X Quadcopter

                Roll control comes only from the front motors, Yaw control only from the rear motors.
                Roll & Pitch factor is measured by the angle away from the top of the forward axis to each arm.

                Note: if we want the front motors to help with yaw,
                    motors 1's yaw factor should be changed to sin(radians(40)).  Where "40" is the vtail angle
                    motors 3's yaw factor should be changed to -sin(radians(40))
            */
            frame_type_string_ = "VTAIL";
            add_motor(0, 60.0f, 60.0f, 0.0f, 1);
            add_motor(1, 0.0f, -160.0f, kYawFactorCw, 3);
            add_motor(2, -60.0f, -60.0f, 0.0f, 4);
            add_motor(3, 0.0f, 160.0f, kYawFactorCcw, 2);
            break;
        }
        case FrameType::ATail: {
            /*
                The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
                - The Yaw factors are reversed, because the rear motors are facing different directions

                With V-Shaped VTails, the props make a V-Shape when spinning, but with
                A-Shaped VTails, the props make an A-Shape when spinning.
                - Rear thrust on a V-Shaped V-Tail Quad is outward
                - Rear thrust on an A-Shaped V-Tail Quad is inward

                Still functions the same as the V-Shaped VTail mixing below:
                - Yaw control is entirely in the rear motors
                - Roll is is entirely in the front motors
            */
            frame_type_string_ = "ATAIL";
            add_motor(0, 60.0f, 60.0f, 0.0f, 1);
            add_motor(1, 0.0f, -160.0f, kYawFactorCcw, 3);
            add_motor(2, -60.0f, -60.0f, 0.0f, 4);
            add_motor(3, 0.0f, 160.0f, kYawFactorCw, 2);
            break;
        }
        case FrameType::PlusRev: {
            // plus with reversed motor directions - every yaw_factor is
            // PLUS's own yaw_factor negated, motor-for-motor (see file
            // banner's "PLUS vs PLUSREV" note).
            frame_type_string_ = "PLUSREV";
            static constexpr MotorDef motors[] = {
                {90.0f, kYawFactorCw, 2},
                {-90.0f, kYawFactorCw, 4},
                {0.0f, kYawFactorCcw, 1},
                {180.0f, kYawFactorCcw, 3},
            };
            add_motors(motors, 4);
            break;
        }
        case FrameType::Y4: {
            frame_type_string_ = "Y4";
            // Y4 motor definition with right front CCW, left front CW
            static constexpr MotorDefRaw motors[] = {
                {-1.0f, 1.000f, kYawFactorCcw, 1},
                {0.0f, -1.000f, kYawFactorCw, 2},
                {0.0f, -1.000f, kYawFactorCcw, 3},
                {1.0f, 1.000f, kYawFactorCw, 4},
            };
            add_motors_raw(motors, 4);
            break;
        }
        default:
            // quad frame class does not support this frame type - matches
            // upstream's own real default case exactly (see file banner:
            // this is the SIMPLE kind of default, not setup_y6_matrix's
            // own productive one).
            return false;
        }
        return true;
    }

    // setup_hexa_matrix - CCP-003 port of upstream
    // AP_MotorsMatrix::setup_hexa_matrix (AP_MotorsMatrix.cpp line 775).
    // Every case's angle/factor/testing-order values are transcribed
    // exactly from the real source - see file banner's "CCP-003 ADDITION"
    // for the full case-list/default-branch/enum-sharing investigation.
    // Takes the SAME FrameType parameter type as setup_quad_matrix above
    // and reuses its Plus/X/H/DjiX/CwX enumerators verbatim (no hexa-
    // specific enumerators exist - see file banner). Returns false
    // (upstream's own real return value) for any FrameType not handled
    // below, matching setup_quad_matrix's own real "not supported"
    // semantics exactly.
    [[nodiscard]] bool setup_hexa_matrix(FrameType frame_type) {
        frame_class_string_ = "HEXA";
        switch (frame_type) {
        case FrameType::Plus: {
            frame_type_string_ = "PLUS";
            static constexpr MotorDef motors[] = {
                {0.0f, kYawFactorCw, 1},
                {180.0f, kYawFactorCcw, 4},
                {-120.0f, kYawFactorCw, 5},
                {60.0f, kYawFactorCcw, 2},
                {-60.0f, kYawFactorCcw, 6},
                {120.0f, kYawFactorCw, 3},
            };
            add_motors(motors, 6);
            break;
        }
        case FrameType::X: {
            frame_type_string_ = "X";
            static constexpr MotorDef motors[] = {
                {90.0f, kYawFactorCw, 2},
                {-90.0f, kYawFactorCcw, 5},
                {-30.0f, kYawFactorCw, 6},
                {150.0f, kYawFactorCcw, 3},
                {30.0f, kYawFactorCcw, 1},
                {-150.0f, kYawFactorCw, 4},
            };
            add_motors(motors, 6);
            break;
        }
        case FrameType::H: {
            // H is same as X except middle motors are closer to center -
            // real explicit (roll_fac, pitch_fac) pairs, not angle
            // degrees (see file banner's "CCP-003 ADDITION").
            frame_type_string_ = "H";
            static constexpr MotorDefRaw motors[] = {
                {-1.0f, 0.0f, kYawFactorCw, 2},
                {1.0f, 0.0f, kYawFactorCcw, 5},
                {1.0f, 1.0f, kYawFactorCw, 6},
                {-1.0f, -1.0f, kYawFactorCcw, 3},
                {-1.0f, 1.0f, kYawFactorCcw, 1},
                {1.0f, -1.0f, kYawFactorCw, 4},
            };
            add_motors_raw(motors, 6);
            break;
        }
        case FrameType::DjiX: {
            frame_type_string_ = "DJI_X";
            static constexpr MotorDef motors[] = {
                {30.0f, kYawFactorCcw, 1},
                {-30.0f, kYawFactorCw, 6},
                {-90.0f, kYawFactorCcw, 5},
                {-150.0f, kYawFactorCw, 4},
                {150.0f, kYawFactorCcw, 3},
                {90.0f, kYawFactorCw, 2},
            };
            add_motors(motors, 6);
            break;
        }
        case FrameType::CwX: {
            frame_type_string_ = "CW_X";
            static constexpr MotorDef motors[] = {
                {30.0f, kYawFactorCcw, 1},
                {90.0f, kYawFactorCw, 2},
                {150.0f, kYawFactorCcw, 3},
                {-150.0f, kYawFactorCw, 4},
                {-90.0f, kYawFactorCcw, 5},
                {-30.0f, kYawFactorCw, 6},
            };
            add_motors(motors, 6);
            break;
        }
        default:
            // hexa frame class does not support this frame type - matches
            // upstream's own real default case exactly (see file banner:
            // this is the SIMPLE kind of default, not setup_y6_matrix's
            // own productive one).
            return false;
        }
        return true;
    }

    // setup_octa_matrix - CCP-004 port of upstream
    // AP_MotorsMatrix::setup_octa_matrix (AP_MotorsMatrix.cpp line 854).
    // Every case's angle/factor/testing-order values are transcribed
    // exactly from the real source - see file banner's "CCP-004 ADDITION"
    // for the full case-list/default-branch/enum-investigation. Takes the
    // SAME FrameType parameter type as setup_quad_matrix/setup_hexa_matrix
    // above; reuses Plus/X/V/H/DjiX/CwX verbatim (all six already existed
    // from CCP-002) and introduces exactly one new enumerator, `I` (see
    // file banner). Returns false (upstream's own real return value) for
    // any FrameType not handled below, matching setup_quad_matrix's/
    // setup_hexa_matrix's own real "not supported" semantics exactly.
    [[nodiscard]] bool setup_octa_matrix(FrameType frame_type) {
        frame_class_string_ = "OCTA";
        switch (frame_type) {
        case FrameType::Plus: {
            frame_type_string_ = "PLUS";
            static constexpr MotorDef motors[] = {
                {0.0f, kYawFactorCw, 1},
                {180.0f, kYawFactorCw, 5},
                {45.0f, kYawFactorCcw, 2},
                {135.0f, kYawFactorCcw, 4},
                {-45.0f, kYawFactorCcw, 8},
                {-135.0f, kYawFactorCcw, 6},
                {-90.0f, kYawFactorCw, 7},
                {90.0f, kYawFactorCw, 3},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::X: {
            frame_type_string_ = "X";
            static constexpr MotorDef motors[] = {
                {22.5f, kYawFactorCw, 1},
                {-157.5f, kYawFactorCw, 5},
                {67.5f, kYawFactorCcw, 2},
                {157.5f, kYawFactorCcw, 4},
                {-22.5f, kYawFactorCcw, 8},
                {-112.5f, kYawFactorCcw, 6},
                {-67.5f, kYawFactorCw, 7},
                {112.5f, kYawFactorCw, 3},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::V: {
            // Real, non-round explicit raw (roll_fac, pitch_fac) pairs -
            // NOT derived from any angle formula, transcribed exactly as
            // upstream's own float literals (see file banner's
            // "CCP-004 ADDITION").
            frame_type_string_ = "V";
            static constexpr MotorDefRaw motors[] = {
                {0.83f, 0.34f, kYawFactorCw, 7},
                {-0.67f, -0.32f, kYawFactorCw, 3},
                {0.67f, -0.32f, kYawFactorCcw, 6},
                {-0.50f, -1.00f, kYawFactorCcw, 4},
                {1.00f, 1.00f, kYawFactorCcw, 8},
                {-0.83f, 0.34f, kYawFactorCcw, 2},
                {-1.00f, 1.00f, kYawFactorCw, 1},
                {0.50f, -1.00f, kYawFactorCw, 5},
            };
            add_motors_raw(motors, 8);
            break;
        }
        case FrameType::H: {
            // Real explicit raw (roll_fac, pitch_fac) pairs - six of the
            // eight entries use plain +-1.0f pitch, but the two at
            // testing_order 2 and 6 use +-0.333f pitch instead,
            // transcribed exactly (NOT a typo, NOT rounded - see file
            // banner's "CCP-004 ADDITION").
            frame_type_string_ = "H";
            static constexpr MotorDefRaw motors[] = {
                {-1.0f, 1.0f, kYawFactorCw, 1},
                {1.0f, -1.0f, kYawFactorCw, 5},
                {-1.0f, 0.333f, kYawFactorCcw, 2},
                {-1.0f, -1.0f, kYawFactorCcw, 4},
                {1.0f, 1.0f, kYawFactorCcw, 8},
                {1.0f, -0.333f, kYawFactorCcw, 6},
                {1.0f, 0.333f, kYawFactorCw, 7},
                {-1.0f, -0.333f, kYawFactorCw, 3},
            };
            add_motors_raw(motors, 8);
            break;
        }
        case FrameType::I: {
            // (sideways H) octo only - upstream's own enumerator comment,
            // re-verified directly (AP_Motors_Class.h line 91). The
            // genuinely new frame type this ticket adds to FrameType -
            // see file banner's "CCP-004 ADDITION". Same +-1.0f/+-0.333f
            // value vocabulary as H's own above, but arranged
            // differently: here roll carries the four 0.333f-magnitude
            // values, not pitch.
            frame_type_string_ = "I";
            static constexpr MotorDefRaw motors[] = {
                {0.333f, -1.0f, kYawFactorCw, 5},
                {-0.333f, 1.0f, kYawFactorCw, 1},
                {1.0f, -1.0f, kYawFactorCcw, 6},
                {0.333f, 1.0f, kYawFactorCcw, 8},
                {-0.333f, -1.0f, kYawFactorCcw, 4},
                {-1.0f, 1.0f, kYawFactorCcw, 2},
                {-1.0f, -1.0f, kYawFactorCw, 3},
                {1.0f, 1.0f, kYawFactorCw, 7},
            };
            add_motors_raw(motors, 8);
            break;
        }
        case FrameType::DjiX: {
            frame_type_string_ = "DJI_X";
            static constexpr MotorDef motors[] = {
                {22.5f, kYawFactorCcw, 1},
                {-22.5f, kYawFactorCw, 8},
                {-67.5f, kYawFactorCcw, 7},
                {-112.5f, kYawFactorCw, 6},
                {-157.5f, kYawFactorCcw, 5},
                {157.5f, kYawFactorCw, 4},
                {112.5f, kYawFactorCcw, 3},
                {67.5f, kYawFactorCw, 2},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::CwX: {
            frame_type_string_ = "CW_X";
            static constexpr MotorDef motors[] = {
                {22.5f, kYawFactorCcw, 1},
                {67.5f, kYawFactorCw, 2},
                {112.5f, kYawFactorCcw, 3},
                {157.5f, kYawFactorCw, 4},
                {-157.5f, kYawFactorCcw, 5},
                {-112.5f, kYawFactorCw, 6},
                {-67.5f, kYawFactorCcw, 7},
                {-22.5f, kYawFactorCw, 8},
            };
            add_motors(motors, 8);
            break;
        }
        default:
            // octa frame class does not support this frame type - matches
            // upstream's own real default case exactly (see file banner:
            // this is the SIMPLE kind of default, not setup_y6_matrix's
            // own productive one).
            return false;
        }
        return true;
    }

    // setup_octaquad_matrix - CCP-005 port of upstream
    // AP_MotorsMatrix::setup_octaquad_matrix (AP_MotorsMatrix.cpp line
    // 973). Every case's angle/factor/testing-order values are
    // transcribed exactly from the real source - see file banner's
    // "CCP-005 ADDITION" for the full case-list/default-branch/enum
    // investigation. Takes the SAME FrameType parameter type as
    // setup_quad_matrix/setup_hexa_matrix/setup_octa_matrix above; reuses
    // Plus/X/V/H/CwX/BfX/BfXRev verbatim (all seven already existed) and
    // introduces exactly two new enumerators, XCor/CwXCor (see file
    // banner). XCor/CwXCor additionally apply a real, separate per-motor
    // rescaling step AFTER their own add_motors() call - see file
    // banner's "THE REAL X8 CO-ROTATING PITFALL" for the exact motor-
    // index subsets and the float-vs-double design decision behind
    // kOctaquadCorotatingScaleFactor. Returns false (upstream's own real
    // return value) for any FrameType not handled below, matching every
    // other setup_*_matrix method's own real "not supported" semantics
    // exactly.
    [[nodiscard]] bool setup_octaquad_matrix(FrameType frame_type) {
        frame_class_string_ = "OCTAQUAD";
        switch (frame_type) {
        case FrameType::Plus: {
            frame_type_string_ = "PLUS";
            static constexpr MotorDef motors[] = {
                {0.0f, kYawFactorCcw, 1},
                {-90.0f, kYawFactorCw, 7},
                {180.0f, kYawFactorCcw, 5},
                {90.0f, kYawFactorCw, 3},
                {-90.0f, kYawFactorCcw, 8},
                {0.0f, kYawFactorCw, 2},
                {90.0f, kYawFactorCcw, 4},
                {180.0f, kYawFactorCw, 6},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::X: {
            frame_type_string_ = "X";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCcw, 1},
                {-45.0f, kYawFactorCw, 7},
                {-135.0f, kYawFactorCcw, 5},
                {135.0f, kYawFactorCw, 3},
                {-45.0f, kYawFactorCcw, 8},
                {45.0f, kYawFactorCw, 2},
                {135.0f, kYawFactorCcw, 4},
                {-135.0f, kYawFactorCw, 6},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::V: {
            // Real MotorDef table (angle-derived roll/pitch, non-+-1
            // explicit yaw_factor) - same SHAPE as setup_quad_matrix's
            // own V case, NOT a MotorDefRaw table like setup_octa_matrix's
            // own V (see file banner's "CCP-005 ADDITION").
            frame_type_string_ = "V";
            static constexpr MotorDef motors[] = {
                {45.0f, 0.7981f, 1},
                {-45.0f, -0.7981f, 7},
                {-135.0f, 1.0000f, 5},
                {135.0f, -1.0000f, 3},
                {-45.0f, 0.7981f, 8},
                {45.0f, -0.7981f, 2},
                {135.0f, 1.0000f, 4},
                {-135.0f, -1.0000f, 6},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::H: {
            // H frame set-up - same as X but motors spin in opposite
            // directions.
            frame_type_string_ = "H";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCw, 1},
                {-45.0f, kYawFactorCcw, 7},
                {-135.0f, kYawFactorCw, 5},
                {135.0f, kYawFactorCcw, 3},
                {-45.0f, kYawFactorCw, 8},
                {45.0f, kYawFactorCcw, 2},
                {135.0f, kYawFactorCw, 4},
                {-135.0f, kYawFactorCcw, 6},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::CwX: {
            frame_type_string_ = "CW_X";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCcw, 1},
                {45.0f, kYawFactorCw, 2},
                {135.0f, kYawFactorCw, 3},
                {135.0f, kYawFactorCcw, 4},
                {-135.0f, kYawFactorCcw, 5},
                {-135.0f, kYawFactorCw, 6},
                {-45.0f, kYawFactorCw, 7},
                {-45.0f, kYawFactorCcw, 8},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::BfX: {
            // BF/X cinelifters using two 4-in-1 ESCs are quite common -
            // see: https://fpvfrenzy.com/betaflight-motor-order/
            frame_type_string_ = "BF_X";
            static constexpr MotorDef motors[] = {
                {135.0f, kYawFactorCw, 3},
                {45.0f, kYawFactorCcw, 1},
                {-135.0f, kYawFactorCcw, 5},
                {-45.0f, kYawFactorCw, 7},
                {135.0f, kYawFactorCcw, 4},
                {45.0f, kYawFactorCw, 2},
                {-135.0f, kYawFactorCw, 6},
                {-45.0f, kYawFactorCcw, 8},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::BfXRev: {
            // betaflight octa quad X order, reversed motors.
            frame_type_string_ = "X_REV";
            static constexpr MotorDef motors[] = {
                {135.0f, kYawFactorCcw, 3},
                {45.0f, kYawFactorCw, 1},
                {-135.0f, kYawFactorCw, 5},
                {-45.0f, kYawFactorCcw, 7},
                {135.0f, kYawFactorCw, 4},
                {45.0f, kYawFactorCcw, 2},
                {-135.0f, kYawFactorCcw, 6},
                {-45.0f, kYawFactorCw, 8},
            };
            add_motors(motors, 8);
            break;
        }
        case FrameType::XCor: {
            // Real X8 co-rotating pitfall (see file banner's "THE REAL
            // X8 CO-ROTATING PITFALL"): add_motors() over this frame's
            // own table, THEN a SEPARATE rescaling step over array
            // indices 0,1,2,3 (the FIRST FOUR motors) - a genuinely
            // different subset from CwXCor's own below.
            frame_type_string_ = "X_COR";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCcw, 1},
                {-45.0f, kYawFactorCw, 7},
                {-135.0f, kYawFactorCcw, 5},
                {135.0f, kYawFactorCw, 3},
                {-45.0f, kYawFactorCw, 8},
                {45.0f, kYawFactorCcw, 2},
                {135.0f, kYawFactorCw, 4},
                {-135.0f, kYawFactorCcw, 6},
            };
            add_motors(motors, 8);
            // Scale top layer to prevent a beat frequency between layers
            // in co-rotating setups - real upstream applies this to ALL
            // FOUR factor arrays, including throttle (whose value here is
            // add_motor_raw's own default throttle_factor=1.0f, since
            // this table never sets one explicitly).
            for (std::uint8_t i = 0; i < 4; ++i) {
                roll_factor_[i] *= kOctaquadCorotatingScaleFactor;
                pitch_factor_[i] *= kOctaquadCorotatingScaleFactor;
                yaw_factor_[i] *= kOctaquadCorotatingScaleFactor;
                throttle_factor_[i] *= kOctaquadCorotatingScaleFactor;
            }
            break;
        }
        case FrameType::CwXCor: {
            // Real X8 co-rotating pitfall (see file banner): add_motors()
            // over this frame's own table, THEN a SEPARATE rescaling step
            // over array indices 0,2,4,6 (EVERY OTHER motor, starting
            // from index 0) - a genuinely different subset from XCor's
            // own first-four above.
            frame_type_string_ = "CW_X_COR";
            static constexpr MotorDef motors[] = {
                {45.0f, kYawFactorCcw, 1},
                {45.0f, kYawFactorCcw, 2},
                {135.0f, kYawFactorCw, 3},
                {135.0f, kYawFactorCw, 4},
                {-135.0f, kYawFactorCcw, 5},
                {-135.0f, kYawFactorCcw, 6},
                {-45.0f, kYawFactorCw, 7},
                {-45.0f, kYawFactorCw, 8},
            };
            add_motors(motors, 8);
            // Scale top layer - same mechanism as XCor above, but the
            // EVEN-INDEXED subset (0,2,4,6), not the first four. All four
            // factor arrays scaled, including throttle (see XCor's own
            // comment above for why throttle's pre-scale value is 1.0f).
            for (std::uint8_t i = 0; i < 8; i += 2) {
                roll_factor_[i] *= kOctaquadCorotatingScaleFactor;
                pitch_factor_[i] *= kOctaquadCorotatingScaleFactor;
                yaw_factor_[i] *= kOctaquadCorotatingScaleFactor;
                throttle_factor_[i] *= kOctaquadCorotatingScaleFactor;
            }
            break;
        }
        default:
            // octaquad frame class does not support this frame type -
            // matches upstream's own real default case exactly (see file
            // banner: this is the SIMPLE kind of default, not
            // setup_y6_matrix's own productive one).
            return false;
        }
        return true;
    }

    // setup_y6_matrix - CCP-006 port of upstream
    // AP_MotorsMatrix::setup_y6_matrix (AP_MotorsMatrix.cpp line 1191).
    // Every case's raw factor/testing-order values are transcribed
    // exactly from the real source - see file banner's "CCP-006 ADDITION"
    // for the full case-list/default-branch investigation. Takes the
    // SAME FrameType parameter type as every setup_*_matrix above;
    // introduces exactly two new enumerators, Y6B/Y6F (see file banner).
    //
    // REAL, DELIBERATE DEPARTURE FROM EVERY OTHER setup_*_matrix ABOVE:
    // this function's own real default: case is NOT "unsupported, return
    // false" - it is a real, WORKING six-motor MotorDefRaw table that
    // upstream reaches for every FrameType value that is neither Y6B nor
    // Y6F (this port's own Plus/X/BfX/BfXRev/DjiX/CwX/V/H/VTail/ATail/
    // PlusRev/Y4/I/XCor/CwXCor all included), matching upstream's own
    // real "silently still build a working Y6 configuration" semantics
    // exactly, NOT a rejection. Confirmed directly: this function's own
    // body contains no `return false;` anywhere; every path - Y6B, Y6F,
    // and the productive default alike - falls through to the SAME
    // unconditional `return true;` below. See
    // motors_matrix_test.cpp's own dedicated fallback tests, this
    // ticket's single most important tests, which confirm several
    // different non-Y6B/Y6F FrameType inputs all produce the exact same
    // fallback table rather than being rejected or producing different
    // tables per input.
    [[nodiscard]] bool setup_y6_matrix(FrameType frame_type) {
        frame_class_string_ = "Y6";
        switch (frame_type) {
        case FrameType::Y6B: {
            // Y6 motor definition with all top motors spinning clockwise,
            // all bottom motors counter clockwise.
            frame_type_string_ = "Y6B";
            static constexpr MotorDefRaw motors[] = {
                {-1.0f, 0.500f, kYawFactorCw, 1},
                {-1.0f, 0.500f, kYawFactorCcw, 2},
                {0.0f, -1.000f, kYawFactorCw, 3},
                {0.0f, -1.000f, kYawFactorCcw, 4},
                {1.0f, 0.500f, kYawFactorCw, 5},
                {1.0f, 0.500f, kYawFactorCcw, 6},
            };
            add_motors_raw(motors, 6);
            break;
        }
        case FrameType::Y6F: {
            // Y6 motor layout for FireFlyY6.
            frame_type_string_ = "Y6F";
            static constexpr MotorDefRaw motors[] = {
                {0.0f, -1.000f, kYawFactorCcw, 3},
                {-1.0f, 0.500f, kYawFactorCcw, 1},
                {1.0f, 0.500f, kYawFactorCcw, 5},
                {0.0f, -1.000f, kYawFactorCw, 4},
                {-1.0f, 0.500f, kYawFactorCw, 2},
                {1.0f, 0.500f, kYawFactorCw, 6},
            };
            add_motors_raw(motors, 6);
            break;
        }
        default: {
            // THE REAL PRODUCTIVE DEFAULT (see file banner's
            // "CCP-006 ADDITION") - upstream's own real fallback table,
            // reached by every FrameType value that is NOT Y6B/Y6F. This
            // is NOT a rejection: it is a real, working six-motor
            // configuration, ported byte-for-byte from upstream's own
            // default: case body, which upstream itself gives no
            // explanatory comment beyond the table.
            frame_type_string_ = "default";
            static constexpr MotorDefRaw motors[] = {
                {-1.0f, 0.666f, kYawFactorCcw, 2},
                {1.0f, 0.666f, kYawFactorCw, 5},
                {1.0f, 0.666f, kYawFactorCcw, 6},
                {0.0f, -1.333f, kYawFactorCw, 4},
                {-1.0f, 0.666f, kYawFactorCw, 1},
                {0.0f, -1.333f, kYawFactorCcw, 3},
            };
            add_motors_raw(motors, 6);
            break;
        }
        }
        // Real, deliberate departure from every other setup_*_matrix
        // above (see file banner): upstream's own setup_y6_matrix NEVER
        // returns false - re-verified directly, there is no
        // `return false;` anywhere in its body. The default: case above
        // IS a working configuration, not a rejection, so this
        // unconditional `true` is correct, not an oversight.
        return true;
    }

    // setup_dodecahexa_matrix - CCP-007 port of upstream
    // AP_MotorsMatrix::setup_dodecahexa_matrix (AP_MotorsMatrix.cpp line
    // 1140). Every case's angle/yaw-factor/testing-order values are
    // transcribed exactly from the real source - see file banner's
    // "CCP-007 ADDITION" for the full case-list/default-branch/enum
    // investigation. Takes the SAME FrameType parameter type as every
    // setup_*_matrix above and reuses its Plus/X enumerators verbatim -
    // adds ZERO new enumerators (see file banner, including the
    // correction of the ticket's own claim about which ticket was first
    // to do so). A RETURN to the SIMPLE default: shape used by
    // setup_quad_matrix/setup_hexa_matrix/setup_octa_matrix/
    // setup_octaquad_matrix above, NOT a continuation of
    // setup_y6_matrix's own productive-default departure. Returns false
    // (upstream's own real return value) for any FrameType not handled
    // below.
    [[nodiscard]] bool setup_dodecahexa_matrix(FrameType frame_type) {
        frame_class_string_ = "DODECAHEXA";
        switch (frame_type) {
        case FrameType::Plus: {
            // Twelve motors - six physical positions (forward,
            // forward-right, back-right, back, back-left, forward-left),
            // each with a top/bottom motor pair sharing the same angle
            // but ALTERNATING yaw factors - see file banner's "THE ONE
            // REAL PITFALL" note for why the alternation (not just the
            // angle) is checked by this ticket's own tests.
            frame_type_string_ = "PLUS";
            static constexpr MotorDef motors[] = {
                {0.0f, kYawFactorCcw, 1},    // forward-top
                {0.0f, kYawFactorCw, 2},     // forward-bottom
                {60.0f, kYawFactorCw, 3},    // forward-right-top
                {60.0f, kYawFactorCcw, 4},   // forward-right-bottom
                {120.0f, kYawFactorCcw, 5},  // back-right-top
                {120.0f, kYawFactorCw, 6},   // back-right-bottom
                {180.0f, kYawFactorCw, 7},   // back-top
                {180.0f, kYawFactorCcw, 8},  // back-bottom
                {-120.0f, kYawFactorCcw, 9}, // back-left-top
                {-120.0f, kYawFactorCw, 10}, // back-left-bottom
                {-60.0f, kYawFactorCw, 11},  // forward-left-top
                {-60.0f, kYawFactorCcw, 12}, // forward-left-bottom
            };
            add_motors(motors, 12);
            break;
        }
        case FrameType::X: {
            // Same repeated-pair-of-six-positions structure as PLUS
            // above, rotated 30 degrees.
            frame_type_string_ = "X";
            static constexpr MotorDef motors[] = {
                {30.0f, kYawFactorCcw, 1},   // forward-right-top
                {30.0f, kYawFactorCw, 2},    // forward-right-bottom
                {90.0f, kYawFactorCw, 3},    // right-top
                {90.0f, kYawFactorCcw, 4},   // right-bottom
                {150.0f, kYawFactorCcw, 5},  // back-right-top
                {150.0f, kYawFactorCw, 6},   // back-right-bottom
                {-150.0f, kYawFactorCw, 7},  // back-left-top
                {-150.0f, kYawFactorCcw, 8}, // back-left-bottom
                {-90.0f, kYawFactorCcw, 9},  // left-top
                {-90.0f, kYawFactorCw, 10},  // left-bottom
                {-30.0f, kYawFactorCw, 11},  // forward-left-top
                {-30.0f, kYawFactorCcw, 12}, // forward-left-bottom
            };
            add_motors(motors, 12);
            break;
        }
        default:
            // dodeca-hexa frame class does not support this frame type -
            // matches upstream's own real default case exactly (see file
            // banner: this is the SIMPLE kind, a return to the shape used
            // before setup_y6_matrix's own productive one).
            return false;
        }
        return true;
    }

    // setup_deca_matrix - CCP-008 port of upstream
    // AP_MotorsMatrix::setup_deca_matrix (AP_MotorsMatrix.cpp line 1242).
    // **THE LAST OF THE SEVEN REAL setup_*_matrix FRAME-CLASS FUNCTIONS IN
    // THIS PORT'S SCOPE** - see file banner's "CCP-008 ADDITION" and
    // "DEFERRED FUTURE PHASES" for what remains (setup_motors' own
    // dispatcher only). Every case's angle/yaw-factor/testing-order values
    // are transcribed exactly from the real source - see file banner for
    // the full case-list/default-branch/enum investigation. Takes the SAME
    // FrameType parameter type as every setup_*_matrix above and reuses
    // its Plus/X/CwX enumerators verbatim - adds ZERO new enumerators (the
    // THIRD such zero-growth ticket in this arc; see file banner).
    //
    // REAL, GENUINELY DISTINCT STRUCTURAL PATTERN FROM EVERY PRIOR TICKET:
    // FrameType::X and FrameType::CwX below share the EXACT SAME case body
    // and the EXACT SAME 10-motor table, mirroring the real upstream
    // `case MOTOR_FRAME_TYPE_X: case MOTOR_FRAME_TYPE_CW_X:` fall-through
    // exactly - a single combined `frame_type_string_ = "X/CW_X"`, not two
    // separate strings. This is NOT the same shape as setup_octaquad_
    // matrix's own X_COR/CW_X_COR (CCP-005), which are two SEPARATE cases
    // with two genuinely DIFFERENT tables - see file banner for the full
    // comparison. motors_matrix_test.cpp's own dedicated test below
    // constructs two separate MotorsMatrix instances, one via
    // setup_deca_matrix(FrameType::X) and one via
    // setup_deca_matrix(FrameType::CwX), and confirms both produce
    // EXACTLY the same per-motor values, proving the shared fall-through
    // table is faithfully reproduced rather than accidentally split into
    // two different (wrong) tables.
    //
    // Returns false (upstream's own real return value) for any FrameType
    // not handled below - the SIMPLE kind of default, matching
    // setup_quad_matrix's/setup_hexa_matrix's/setup_octa_matrix's/
    // setup_octaquad_matrix's/setup_dodecahexa_matrix's own defaults
    // exactly, NOT setup_y6_matrix's own real productive fallback.
    [[nodiscard]] bool setup_deca_matrix(FrameType frame_type) {
        frame_class_string_ = "DECA";
        switch (frame_type) {
        case FrameType::Plus: {
            // Ten motors, angles evenly spaced 36 degrees apart around the
            // full circle starting at 0, alternating CCW/CW yaw factors.
            frame_type_string_ = "PLUS";
            static constexpr MotorDef motors[] = {
                {0.0f, kYawFactorCcw, 1},
                {36.0f, kYawFactorCw, 2},
                {72.0f, kYawFactorCcw, 3},
                {108.0f, kYawFactorCw, 4},
                {144.0f, kYawFactorCcw, 5},
                {180.0f, kYawFactorCw, 6},
                {-144.0f, kYawFactorCcw, 7},
                {-108.0f, kYawFactorCw, 8},
                {-72.0f, kYawFactorCcw, 9},
                {-36.0f, kYawFactorCw, 10},
            };
            add_motors(motors, 10);
            break;
        }
        case FrameType::X:
        case FrameType::CwX: {
            // REAL, RE-VERIFIED FALL-THROUGH (see class-level comment
            // above and file banner's "CCP-008 ADDITION"): upstream's own
            // switch has ONE case body shared by BOTH MOTOR_FRAME_TYPE_X
            // and MOTOR_FRAME_TYPE_CW_X, with a single combined
            // "X/CW_X" frame_type_string - not two separate cases with
            // two different tables. Same evenly-spaced-36-degrees pattern
            // as PLUS above, rotated 18 degrees.
            frame_type_string_ = "X/CW_X";
            static constexpr MotorDef motors[] = {
                {18.0f, kYawFactorCcw, 1},
                {54.0f, kYawFactorCw, 2},
                {90.0f, kYawFactorCcw, 3},
                {126.0f, kYawFactorCw, 4},
                {162.0f, kYawFactorCcw, 5},
                {-162.0f, kYawFactorCw, 6},
                {-126.0f, kYawFactorCcw, 7},
                {-90.0f, kYawFactorCw, 8},
                {-54.0f, kYawFactorCcw, 9},
                {-18.0f, kYawFactorCw, 10},
            };
            add_motors(motors, 10);
            break;
        }
        default:
            // deca frame class does not support this frame type - matches
            // upstream's own real default case exactly (see file banner:
            // this is the SIMPLE kind, not setup_y6_matrix's own
            // productive one).
            return false;
        }
        return true;
    }

    // setup_motors - CCP-009 port of upstream AP_MotorsMatrix::setup_motors
    // (AP_MotorsMatrix.cpp, real function lines 1290-1349, re-verified
    // directly against the pinned worktree; the ticket's own guessed span
    // matched exactly). This is the dispatcher that closes out
    // AP_MotorsMatrix's own construction-time configuration surface: it
    // routes a (FrameClass, FrameType) pair to the correct one of the
    // seven already-ported setup_*_matrix functions above. The real
    // seven-step structure, ported exactly:
    //   1. Unconditionally removes ALL kMaxNumMotors motor slots FIRST,
    //      via the real `for (int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS;
    //      i++) remove_motor(i);` loop - re-verified this runs
    //      unconditionally, before anything else, not gated on any prior
    //      state. This is what guarantees a re-configuration (calling
    //      setup_motors a second time with a different frame class/type)
    //      leaves no ENABLED motor and no non-zero roll/pitch/yaw/
    //      throttle factor from the first call - see motors_matrix_test.cpp's
    //      own dedicated re-configuration test. REAL, RE-VERIFIED QUIRK
    //      (confirmed directly against upstream's own real remove_motor,
    //      AP_MotorsMatrix.cpp lines 545-552, see CCP-001's own remove_motor
    //      formula in the file banner above): remove_motor's real body
    //      clears motor_enabled and the four RPYT factor arrays ONLY - it
    //      never touches _test_order. A disabled motor's test_order is
    //      therefore genuinely NOT reset by this loop and can retain a
    //      stale value from a PRIOR configuration even after a full
    //      re-configuration - upstream's own real behavior, not a port
    //      gap (test_order is only ever consulted for motor-test
    //      sequencing among ENABLED motors, so a stale value on a
    //      disabled slot is harmless in practice). motors_matrix_test.cpp's
    //      own re-configuration test asserts this exact stale-test_order
    //      behavior directly rather than incorrectly expecting 0.
    //   2. `set_initialised_ok(false);` immediately after - re-verified
    //      this runs BEFORE the switch, so the add_motor_raw guard (see
    //      file banner's GUARD design decision) does not block the
    //      setup_*_matrix call about to run below.
    //   3. `switch (frame_class)` with exactly the seven real cases
    //      (MOTOR_FRAME_QUAD/_HEXA/_OCTA/_OCTAQUAD/_DODECAHEXA/_Y6/_DECA)
    //      - re-verified directly against the real switch body that these
    //      are the ONLY seven real cases; see FrameClass's own class-body
    //      comment above for the full investigation of every other real
    //      motor_frame_class enumerator and why each is handled by an
    //      entirely separate AP_Motors-family class, never by this
    //      switch. Each case stores its setup_*_matrix's own bool return
    //      in `success` (real upstream default-initializes `success =
    //      true;` before the switch, matching this port's own
    //      initialization below).
    //   4. `default: success = false;` - real upstream also writes
    //      `_mav_type = MAV_TYPE_GENERIC;` here, pure MAVLink/GCS metadata
    //      this port has no GCS to report to - same disclosed-omission
    //      class as every setup_*_matrix's own _mav_type write (see file
    //      banner).
    //   5. `normalise_rpy_factors();` called UNCONDITIONALLY - re-verified
    //      directly this is NOT inside an `if (success)` guard. On
    //      failure, step 1 already cleared every motor slot, so this
    //      normalises an all-zero, all-disabled motor set - a real,
    //      harmless no-op (every early-return inside normalise_rpy_
    //      factors's own is_zero guards is taken), ported faithfully
    //      rather than skipped as "obviously unnecessary".
    //   6. `if (!success) { frame_class_string_ = "UNSUPPORTED"; }` -
    //      re-verified this exact string and condition. Note this
    //      OVERWRITES whatever frame_class_string_ an in-range-but-
    //      frame_type-rejecting setup_*_matrix call already wrote (e.g.
    //      setup_quad_matrix always sets frame_class_string_ = "QUAD" at
    //      its own top before checking frame_type) - matching upstream's
    //      real behavior exactly, since upstream's own _frame_class_
    //      string is a single shared field with the same overwrite
    //      semantics.
    //   7. `set_initialised_ok(success);` as the final statement.
    // Returns void, matching upstream's real signature exactly (unlike
    // every setup_*_matrix above, which returns bool) - callers observe
    // the outcome via initialised_ok()/frame_class_string(), exactly as
    // upstream's own real callers do.
    void setup_motors(FrameClass frame_class, FrameType frame_type) {
        for (std::size_t i = 0; i < kMaxNumMotors; ++i) {
            remove_motor(static_cast<std::int8_t>(i));
        }
        set_initialised_ok(false);
        bool success = true;

        switch (frame_class) {
        case FrameClass::Quad:
            success = setup_quad_matrix(frame_type);
            break;
        case FrameClass::Hexa:
            success = setup_hexa_matrix(frame_type);
            break;
        case FrameClass::Octa:
            success = setup_octa_matrix(frame_type);
            break;
        case FrameClass::Octaquad:
            success = setup_octaquad_matrix(frame_type);
            break;
        case FrameClass::Dodecahexa:
            success = setup_dodecahexa_matrix(frame_type);
            break;
        case FrameClass::Y6:
            success = setup_y6_matrix(frame_type);
            break;
        case FrameClass::Deca:
            success = setup_deca_matrix(frame_type);
            break;
        default:
            // matrix doesn't support the configured class - real upstream
            // also writes `_mav_type = MAV_TYPE_GENERIC;` here, a
            // disclosed GCS-metadata omission (see comment above).
            success = false;
            break;
        }

        // normalise factors to magnitude 0.5 - UNCONDITIONAL, real
        // upstream never guards this on `success` (see comment above).
        normalise_rpy_factors();

        if (!success) {
            frame_class_string_ = "UNSUPPORTED";
        }
        set_initialised_ok(success);
    }

    // frame_type_string()/frame_class_string() - port of upstream's real
    // _frame_type_string/_frame_class_string state (set by
    // setup_quad_matrix/setup_hexa_matrix/setup_octa_matrix above), even
    // though this port has no GCS to report them to - useful for
    // tests/debugging, per the ticket's own request. Empty until one of
    // those has been called at least once (matching upstream's own
    // uninitialized-until-setup
    // behavior).
    [[nodiscard]] const std::string& frame_type_string() const { return frame_type_string_; }
    [[nodiscard]] const std::string& frame_class_string() const { return frame_class_string_; }

    // check_for_failed_motor - CCP-011 port of upstream
    // AP_MotorsMatrix::check_for_failed_motor (real function lines
    // 414-461). See file banner's "CCP-011 ADDITION" for the full
    // six-step structure, the disclosed motor_lost_index_ bug-fix, the
    // active_frame_type_/thr_lin_ design decisions, and the
    // throttle_thrust_max parameter-shape rationale - all re-verified
    // directly against the real source, not trusted from the ticket's
    // own summary.
    void check_for_failed_motor(float throttle_thrust_best_plus_adj, float throttle_thrust_max, float dt_s,
                                 float air_density_ratio) {
        // Step 1/2: record filtered and scaled thrust output for motor
        // loss monitoring purposes - the real, non-standard alpha formula
        // (NOT math::calc_lowpass_alpha_dt - see file banner).
        const float alpha = dt_s / (dt_s + 0.5f);
        for (std::size_t i = 0; i < kMaxNumMotors; ++i) {
            if (motor_enabled_[i]) {
                thrust_rpyt_out_filt_[i] += alpha * (thrust_rpyt_out_[i] - thrust_rpyt_out_filt_[i]);
            }
        }

        // Step 3: second, separate pass - rpyt_high/rpyt_sum/number_motors,
        // plus the thrust_boost_-gated motor_lost_index_ update.
        float rpyt_high = 0.0f;
        float rpyt_sum = 0.0f;
        std::uint8_t number_motors = 0;
        for (std::size_t i = 0; i < kMaxNumMotors; ++i) {
            if (motor_enabled_[i]) {
                number_motors += 1;
                rpyt_sum += thrust_rpyt_out_filt_[i];
                // record highest filtered thrust command
                if (thrust_rpyt_out_filt_[i] > rpyt_high) {
                    rpyt_high = thrust_rpyt_out_filt_[i];
                    // hold motor lost index constant while thrust boost is active
                    if (!thrust_boost_) {
                        motor_lost_index_ = static_cast<std::uint8_t>(i);
                    }
                }
            }
        }

        // Step 4.
        float thrust_balance = 1.0f;
        if (rpyt_sum > 0.1f) {
            thrust_balance = rpyt_high * number_motors / rpyt_sum;
        }

        // Step 5: real hysteresis - TWO separate, sequential `if`
        // statements, not an if/else (see file banner).
        const bool is_corotating = active_frame_type_ == FrameType::XCor || active_frame_type_ == FrameType::CwXCor;
        if (number_motors >= 6 && thrust_balance >= 1.5f && thrust_balanced_ && !is_corotating) {
            thrust_balanced_ = false;
        }
        if (thrust_balance <= 1.25f && !thrust_balanced_) {
            thrust_balanced_ = true;
        }

        // Step 6: the ONLY place thrust_boost_ is assigned in this
        // function - never set true here.
        if ((throttle_thrust_max * thr_lin_.get_compensation_gain(air_density_ratio) > throttle_thrust_best_plus_adj) &&
            (rpyt_high < 0.9f) && thrust_balanced_) {
            thrust_boost_ = false;
        }
    }

    // Test-only mutators (CCP-011) - see file banner's "TEST-ONLY GAP,
    // DISCLOSED" and "active_frame_type_ TRACKING" sections: this port
    // has no output_armed_stabilizing yet to populate thrust_rpyt_out_
    // for real, and no init()/set_frame_class_and_type() yet to record
    // active_frame_type_ for real, so both are directly settable here.
    void set_thrust_rpyt_out(std::uint8_t i, float value) {
        if (i < kMaxNumMotors) {
            thrust_rpyt_out_[i] = value;
        }
    }
    void set_active_frame_type(FrameType frame_type) { active_frame_type_ = frame_type; }
    void set_thrust_boost(bool value) { thrust_boost_ = value; }
    void set_thrust_balanced(bool value) { thrust_balanced_ = value; }

    // Read accessors for check_for_failed_motor's own new state
    // (CCP-011) - bounds-checked per this class's own established
    // convention (see "Accessors" comment below).
    [[nodiscard]] float thrust_rpyt_out(std::uint8_t i) const { return i < kMaxNumMotors ? thrust_rpyt_out_[i] : 0.0f; }
    [[nodiscard]] float thrust_rpyt_out_filt(std::uint8_t i) const {
        return i < kMaxNumMotors ? thrust_rpyt_out_filt_[i] : 0.0f;
    }
    [[nodiscard]] FrameType active_frame_type() const { return active_frame_type_; }
    [[nodiscard]] bool thrust_boost() const { return thrust_boost_; }
    [[nodiscard]] bool thrust_balanced() const { return thrust_balanced_; }
    [[nodiscard]] std::uint8_t motor_lost_index() const { return motor_lost_index_; }
    // Exposes the CCP-010 ThrustLinearization dependency directly (see
    // file banner's "thr_lin_ MEMBER") - lets tests exercise a real,
    // non-default get_compensation_gain() (e.g. via
    // update_lift_max_from_batt_voltage) rather than only ever observing
    // the default lift_max_ == 1.0 path.
    [[nodiscard]] ThrustLinearization& thrust_linearization() { return thr_lin_; }
    [[nodiscard]] const ThrustLinearization& thrust_linearization() const { return thr_lin_; }

    // set_actuator_with_slew - CCP-012 port of upstream
    // AP_MotorsMulticopter::set_actuator_with_slew (real function body
    // lines 480-503). See file banner's "CCP-012 ADDITION" for the full
    // formula derivation, the real current-output-vs-destination bounding
    // pitfall (COP-004's own finding, independently re-verified here), and
    // the resolved "no SHUT_DOWN check" non-bug. `static` - see file
    // banner's "STATIC METHODS" - this function touches no MotorsMatrix
    // instance state.
    //
    // Mutates actuator_output BY REFERENCE, exactly matching upstream's own
    // `float&` reference parameter (see file banner's "PARAMETER SHAPE"). A
    // caller running this every control-loop tick keeps the same
    // actuator_output variable alive across calls - the CURRENT value this
    // function reads on call N is the value IT ITSELF wrote on call N-1,
    // which is exactly what makes the limits bound the STEP rather than the
    // destination.
    static void set_actuator_with_slew(float& actuator_output, float input, float slew_up_time, float slew_dn_time,
                                        float dt_s) {
        // "No slew limit" defaults - re-verified directly, NOT both 0 or
        // both 1 (see file banner).
        float output_slew_limit_up = 1.0f;
        float output_slew_limit_dn = 0.0f;

        if (math::is_positive(slew_up_time)) {
            const float output_delta_up_max = dt_s / math::constrain_value(slew_up_time, 0.0f, 0.5f);
            // Relative to the CURRENT actuator_output - the bounding
            // pitfall (file banner).
            output_slew_limit_up = math::constrain_value(actuator_output + output_delta_up_max, 0.0f, 1.0f);
        }

        if (math::is_positive(slew_dn_time)) {
            const float output_delta_dn_max = dt_s / math::constrain_value(slew_dn_time, 0.0f, 0.5f);
            output_slew_limit_dn = math::constrain_value(actuator_output - output_delta_dn_max, 0.0f, 1.0f);
        }

        actuator_output = math::constrain_value(input, output_slew_limit_dn, output_slew_limit_up);
    }

    // actuator_spin_up_to_ground_idle - CCP-012 port of upstream
    // AP_MotorsMulticopter::actuator_spin_up_to_ground_idle (real function
    // body lines 511-513). Takes spin_min directly rather than calling
    // thr_lin_.get_spin_min() - see file banner's "spin_min PARAMETER
    // CORRECTION": no such accessor exists, spin_min is a plain public
    // field on the caller-owned ThrustLinParams (CCP-010). `static` - see
    // file banner's "STATIC METHODS".
    [[nodiscard]] static float actuator_spin_up_to_ground_idle(float spin_up_ratio, float spin_min) {
        return math::constrain_value(spin_up_ratio, 0.0f, 1.0f) * spin_min;
    }

    // output_logic (PARTS 1 AND 2, COMPLETE) - CCP-013 ported upstream
    // AP_MotorsMulticopter::output_logic's real function body lines
    // 591-768 of the real 591-884 span (the safety preamble (597-622),
    // SpoolState::SHUT_DOWN (633-663), and SpoolState::GROUND_IDLE
    // (665-768, including its own nested switch on DesiredSpoolState)).
    // CCP-014 completes the same method with the remaining three real
    // cases, re-verified directly against the pinned worktree:
    // SpoolState::SPOOLING_UP (769-804), SpoolState::THROTTLE_UNLIMITED
    // (805-839), SpoolState::SPOOLING_DOWN (840-883) - replacing CCP-013's
    // own explicit `default:` no-op placeholder (see that ticket's own
    // comment, now removed) with the three real case bodies below. See
    // file banner's "CCP-013 ADDITION" for the full Part 1 structure, the
    // corrected spool_state_/spool_desired_ investigation (suspected
    // uninitialized-read bug, found NOT to be one - see the private
    // member declarations below), the SIX-member uninitialized-read bug
    // that WAS confirmed and fixed instead, the real same-call
    // spoolup_block read-after-write finding, and the parameter-shape
    // rationale.
    //
    // Every real external dependency (armed/interlock/spoolup_block/
    // limit flags/etc.) is an explicit parameter or explicit output, per
    // ADR-0012 - NO singleton/global is read. `spool_up_time` is taken
    // as `float&` and genuinely mutated when the floor clamp fires,
    // matching upstream's own real `_spool_up_time.set(...)` write-back
    // (COP-004's own finding #2, independently re-verified here - see
    // file banner). `limits_all_engaged`/`should_set_spoolup_block` are
    // explicit `bool&` outputs (this port has no `limit` flags struct
    // and no vehicle-level spoolup-block gate to call a setter on - see
    // file banner's "OUTPUT SHAPE" section): both are set to `false` at
    // the top of every call and only ever raised to `true`/kept `false`
    // within the branches that would have called `limit.set_all(true)`/
    // `limit.set_all(false)` / `set_spoolup_block(true)` in real
    // upstream - Part 1's own SHUT_DOWN/GROUND_IDLE cases only ever call
    // `set_all(true)`, and Part 2's own three cases below only ever call
    // `set_all(false)`, so this single bool still captures every real
    // behavior the complete function exercises.
    //
    // CCP-014 ADDITION (Part 2, two new explicit parameters) - real
    // upstream's own `get_throttle()` (SPOOLING_UP/THROTTLE_UNLIMITED,
    // real lines 791/831) and `get_current_limit_max_throttle()`
    // (SPOOLING_UP/THROTTLE_UNLIMITED/SPOOLING_DOWN, real lines
    // 791-792/831-832/875-877) are neither built in this port -
    // `get_throttle()` needs a real `update_throttle_filter()` this port
    // has not built (returns the FILTERED throttle command, NOT the raw
    // value most recently set - copter-rust's own COP-004 flagged this
    // exact trap), and `get_current_limit_max_throttle()` needs a real
    // `AP_BattMonitor` this port has no `ap-battery` module for at all
    // (COP-004 again: "a battery-less harness can only run with
    // MOT_BAT_CURR_MAX at 0 and the ceiling pinned to 1.0"). Per
    // ADR-0012, both become explicit `float` parameters -
    // `filtered_throttle` (the caller supplies an already-filtered value;
    // a future ticket building `update_throttle_filter` is responsible
    // for feeding a real one) and `current_limit_max_throttle` (tests
    // exercise both the real no-limiting default of `1.0f` COP-004
    // identified and a range of lower ceilings) - added to the existing
    // parameter list rather than as a new method, per this ticket's own
    // explicit instruction that this is an EDIT to CCP-013's real
    // single-switch method, not a second one.
    void output_logic(bool armed, bool interlock, bool disarm_disable_pwm, float safe_time, float& spool_up_time,
                       float spool_down_time, float spin_arm, float idle_time_delay_s, float spin_min, bool spoolup_block,
                       float filtered_throttle, float current_limit_max_throttle, float dt_s, bool& limits_all_engaged,
                       bool& should_set_spoolup_block) {
        limits_all_engaged = false;
        should_set_spoolup_block = false;

        // Real function-local constant (line ~594) - NOT a port-wide
        // constant, re-verified directly it is declared inside this
        // function in upstream, not at file/class scope.
        constexpr float minimum_spool_time = 0.05f;

        // 1. Disarm-PWM safety-window timer (real lines 597-611). Only
        // advances while armed; resets to 0 the instant it is not, so a
        // disarm always costs the full delay again rather than resuming
        // a part-elapsed one.
        if (armed) {
            if (disarm_disable_pwm && disarm_safe_timer_ < safe_time) {
                disarm_safe_timer_ += dt_s;
            } else {
                disarm_safe_timer_ = safe_time;
            }
        } else {
            disarm_safe_timer_ = 0.0f;
        }

        // 2. Global safety rule (real lines 613-617) - unconditional,
        // no ramp: forces BOTH the desired and actual spool state to
        // ShutDown immediately whenever either safety condition is
        // false.
        if (!armed || !interlock) {
            spool_desired_ = DesiredSpoolState::ShutDown;
            spool_state_ = SpoolState::ShutDown;
        }

        // 3. Spool-up-time floor clamp (real lines 619-622) - REAL
        // WRITE-BACK into the CALLER's own variable, matching upstream's
        // `_spool_up_time.set(...)` (see file banner and COP-004's own
        // finding #2).
        if (spool_up_time < minimum_spool_time) {
            spool_up_time = minimum_spool_time;
        }

        switch (spool_state_) {
        case SpoolState::ShutDown: {
            // "All limits engaged" output - real upstream
            // `limit.set_all(true)`, see file banner's "OUTPUT SHAPE"
            // simplification.
            limits_all_engaged = true;

            spin_up_ratio_ = 0.0f;
            throttle_thrust_max_ = 0.0f;
            idle_time_ = 0.0f;

            thrust_boost_ = false;
            thrust_boost_ratio_ = 0.0f;

            // Leave SHUT_DOWN only once the safe-time window has
            // elapsed AND a higher-energy state is requested - real,
            // re-verified two-part condition (real lines 660-662).
            if (spool_desired_ != DesiredSpoolState::ShutDown && disarm_safe_timer_ >= safe_time) {
                spool_state_ = SpoolState::GroundIdle;
            }
            break;
        }

        case SpoolState::GroundIdle: {
            limits_all_engaged = true;

            // Normalised against spin_min: with no spin_min there is
            // nothing to normalise against and the target is zero (real
            // lines 675-678).
            float spin_up_ground_idle_ratio = 0.0f;
            if (math::is_positive(spin_min)) {
                spin_up_ground_idle_ratio = spin_arm / spin_min;
            }

            // The idle-time delay measures time AT ground-idle spin, not
            // time since the request (real lines 680-683).
            if (spin_up_ratio_ >= spin_up_ground_idle_ratio) {
                idle_time_ = std::min(idle_time_delay_s, idle_time_ + dt_s);
            }

            switch (spool_desired_) {
            case DesiredSpoolState::ShutDown: {
                // Down path - real, easy-to-miss symmetry fallback: use
                // spool_down_time if actually configured above the
                // floor, else fall back to spool_up_time (real line
                // 690).
                const float spool_time = spool_down_time > minimum_spool_time ? spool_down_time : spool_up_time;
                const float spool_step = dt_s / spool_time;
                spin_up_ratio_ -= spool_step;

                if (spin_up_ratio_ <= 0.0f) {
                    spin_up_ratio_ = 0.0f;
                    spool_state_ = SpoolState::ShutDown;
                }
                break;
            }
            case DesiredSpoolState::ThrottleUnlimited: {
                const float spool_step = dt_s / spool_up_time;
                spin_up_ratio_ += spool_step;

                // Real, easy-to-miss idle-time gating (real lines
                // 726-729): a genuine EARLY BREAK, not merely a clamp -
                // holds spin at ground-idle and skips everything below
                // in this case for this call, while the delay is still
                // running.
                if (idle_time_ < idle_time_delay_s) {
                    spin_up_ratio_ = std::min(spin_up_ratio_, spin_up_ground_idle_ratio);
                    break;
                }

                // Real same-call read-after-write (re-verified directly
                // against AP_Motors_Class.h lines 127-128: get_spoolup_
                // block()/set_spoolup_block() are a plain bool getter/
                // setter pair with no other side effect). Upstream calls
                // `set_spoolup_block(true)` and then, a few lines below
                // IN THE SAME CALL, reads `get_spoolup_block()` - so the
                // transition below never fires on the very call that
                // first raises the block, it always sees the just-
                // written `true`. `spoolup_block_now` mirrors that
                // synchronous read-after-write; the `spoolup_block`
                // PARAMETER itself is an input and is never mutated -
                // the caller learns of the real transition via
                // `should_set_spoolup_block` and is expected to pass the
                // (now-updated) block state back in on a LATER call.
                bool spoolup_block_now = spoolup_block;
                if (spin_up_ratio_ < 1.0f) {
                    spin_up_complete_ = false;
                } else {
                    spin_up_ratio_ = 1.0f;
                    if (!spin_up_complete_) {
                        spin_up_complete_ = true;
                        should_set_spoolup_block = true;
                        spoolup_block_now = true;
                    }
                }
                if (spin_up_complete_ && !spoolup_block_now) {
                    spool_state_ = SpoolState::SpoolingUp;
                }
                break;
            }
            case DesiredSpoolState::GroundIdle: {
                // Real asymmetric slew toward spin_up_ground_idle_ratio
                // (real line 762) - structurally distinct from
                // set_actuator_with_slew's own formula (CCP-012, see
                // file banner): down bounded by spool_down_step, up by
                // spool_up_step, in one combined constrain_value call.
                const float spool_up_step = dt_s / spool_up_time;
                const float spool_down_time_effective = spool_down_time > minimum_spool_time ? spool_down_time : spool_up_time;
                const float spool_down_step = dt_s / spool_down_time_effective;

                spin_up_ratio_ +=
                    math::constrain_value(spin_up_ground_idle_ratio - spin_up_ratio_, -spool_down_step, spool_up_step);
                break;
            }
            }

            // Shared post-inner-switch reset (real lines 764-768) - runs
            // for ALL THREE inner cases above, OUTSIDE the inner switch,
            // NOT duplicated per-case (re-verified directly - see file
            // banner).
            throttle_thrust_max_ = 0.0f;
            thrust_boost_ = false;
            thrust_boost_ratio_ = 0.0f;
            break;
        }

        case SpoolState::SpoolingUp: {
            // CCP-014, real lines 769-804. Spin is already at 1.0 by the
            // time this state is reached; only the throttle ceiling ramps
            // here.
            const float spool_step = dt_s / spool_up_time;

            // "All limits RELEASED" output - the SAME bool CCP-013 added
            // for ShutDown/GroundIdle's own "all limits engaged", set to
            // the OPPOSITE value here (real upstream `limit.set_all
            // (false)`, re-verified: normal attitude/throttle authority
            // during the ramp).
            limits_all_engaged = false;

            // Direction-correction (real lines 777-780) - a genuine EARLY
            // BREAK, re-verified directly: if desired has dropped below
            // ThrottleUnlimited, reverse immediately and skip the rest of
            // this case for this call.
            if (spool_desired_ != DesiredSpoolState::ThrottleUnlimited) {
                spool_state_ = SpoolState::SpoolingDown;
                break;
            }

            spin_up_ratio_ = 1.0f;
            throttle_thrust_max_ += spool_step;

            // Transition once the moving ceiling no longer limits the
            // commanded (filtered) throttle, snapping the ceiling EXACTLY
            // to current_limit_max_throttle at that moment (real lines
            // 791-794, re-verified: the snap target is
            // current_limit_max_throttle itself, not the min() used only
            // for the comparison). The else-if lower-bound guard below is
            // re-verified to apply ONLY when this transition did NOT
            // fire, never in addition to it.
            if (throttle_thrust_max_ >= std::min(filtered_throttle, current_limit_max_throttle)) {
                throttle_thrust_max_ = current_limit_max_throttle;
                spool_state_ = SpoolState::ThrottleUnlimited;
            } else if (throttle_thrust_max_ < 0.0f) {
                throttle_thrust_max_ = 0.0f;
            }

            // Fade any thrust boost during spool-up (real lines 802-803).
            thrust_boost_ = false;
            thrust_boost_ratio_ = std::max(0.0f, thrust_boost_ratio_ - spool_step);
            break;
        }

        case SpoolState::ThrottleUnlimited: {
            // CCP-014, real lines 805-839. `spool_step` here IS genuinely
            // used, despite real upstream's own comment "not used for
            // throttle in this state" - re-verified directly: it drives
            // the thrust-boost-ratio ramp below, just not the throttle
            // ceiling. A port that dropped this variable because of the
            // comment alone would be wrong.
            const float spool_step = dt_s / spool_up_time;

            limits_all_engaged = false;

            // Same direction-correction as SpoolingUp above - a SEPARATE,
            // textually-duplicated check in real upstream, not shared
            // code (re-verified directly).
            if (spool_desired_ != DesiredSpoolState::ThrottleUnlimited) {
                spool_state_ = SpoolState::SpoolingDown;
                break;
            }

            spin_up_ratio_ = 1.0f;
            // Plain assignment, NOT incremental (re-verified directly):
            // the ceiling tracks the current limit instantaneously in
            // this state, no ramping.
            throttle_thrust_max_ = current_limit_max_throttle;

            // Real if/else (re-verified directly - NOT two independent
            // ifs, a genuinely different shape from check_for_failed_
            // motor's own CCP-011 hysteresis): ramp UP toward 1.0 while
            // boost is requested and not yet balanced, else ramp DOWN
            // toward 0.
            if (thrust_boost_ && !thrust_balanced_) {
                thrust_boost_ratio_ = std::min(1.0f, thrust_boost_ratio_ + spool_step);
            } else {
                thrust_boost_ratio_ = std::max(0.0f, thrust_boost_ratio_ - spool_step);
            }
            break;
        }

        case SpoolState::SpoolingDown: {
            // CCP-014, real lines 840-883.
            limits_all_engaged = false;

            // REVERSE direction-correction from SpoolingUp/
            // ThrottleUnlimited above (re-verified directly): if desired
            // has come BACK UP to ThrottleUnlimited, reverse immediately.
            if (spool_desired_ == DesiredSpoolState::ThrottleUnlimited) {
                spool_state_ = SpoolState::SpoolingUp;
                break;
            }

            // Spin stays at 1.0 through the whole down-ramp; spin
            // reduction happens only in GroundIdle, not here - real
            // upstream's own comment, re-verified directly.
            spin_up_ratio_ = 1.0f;

            // SAME symmetry-fallback formula as GroundIdle's own
            // ShutDown-desired case above, re-verified directly to be
            // identical - reusing the SAME minimum_spool_time local
            // constant already in scope at the top of this method, not
            // redeclared.
            const float spool_time = spool_down_time > minimum_spool_time ? spool_down_time : spool_up_time;
            const float spool_step = dt_s / spool_time;
            throttle_thrust_max_ -= spool_step;

            if (throttle_thrust_max_ <= 0.0f) {
                throttle_thrust_max_ = 0.0f;
            }

            // Real if/else-if (re-verified directly to be genuinely
            // mutually exclusive, not two independent ifs - transcribed
            // faithfully as the real structure rather than assumed
            // equivalent): snap DOWN to current_limit_max_throttle if the
            // ramp is still at/above it (can only matter if the limit
            // itself dropped below the already-ramping-down ceiling
            // mid-ramp); ELSE, transition to GroundIdle once the ceiling
            // has reached exactly zero.
            if (throttle_thrust_max_ >= current_limit_max_throttle) {
                throttle_thrust_max_ = current_limit_max_throttle;
            } else if (math::is_zero(throttle_thrust_max_)) {
                spool_state_ = SpoolState::GroundIdle;
            }

            // Fades thrust_boost_ratio_ only (real line 882) - re-verified
            // directly that thrust_boost_ itself (the bool) is NOT
            // touched anywhere in this case, unlike SpoolingUp's own
            // explicit `thrust_boost_ = false`.
            thrust_boost_ratio_ = std::max(0.0f, thrust_boost_ratio_ - spool_step);
            break;
        }
        }
    }

    // CCP-015 ADDITION - upstream AP_MotorsMulticopter::output_to_pwm, real
    // function body lines 457-472 (~16 lines), re-verified directly against
    // the pinned worktree (matches the ticket's own transcription exactly):
    //
    //   int16_t AP_MotorsMulticopter::output_to_pwm(float actuator)
    //   {
    //       float pwm_output;
    //       if (_spool_state == SpoolState::SHUT_DOWN) {
    //           if (_disarm_disable_pwm && !armed()) {
    //               pwm_output = 0;
    //           } else {
    //               pwm_output = get_pwm_output_min();
    //           }
    //       } else {
    //           pwm_output = get_pwm_output_min() + (get_pwm_output_max() - get_pwm_output_min()) * actuator;
    //       }
    //       return pwm_output;
    //   }
    //
    // TRUNCATION, NOT ROUNDING - the one real quirk here, independently
    // re-verified against the source above and matching copter-rust's own
    // COP-004 finding exactly ("output_to_pwm computes a float and returns
    // it through an int16_t, so the result TRUNCATES rather than rounds...
    // the sweep includes actuator values that land just under an integer
    // pulse, which is the only place a rounding port would disagree").
    // Real upstream's own return statement (`return pwm_output;`) is an
    // IMPLICIT float-to-int16_t conversion - C++'s standard conversion
    // semantics for that truncate toward zero, they do NOT round to
    // nearest. This port makes the conversion explicit
    // (`static_cast<std::int16_t>(pwm_output)`) but preserves the exact
    // same truncating behavior - NO std::round()/std::lround() is used, as
    // that would be a silent behavioral "improvement" that disagrees with
    // real upstream on any fractional PWM value >= 0.5. See this file's own
    // dedicated truncation-vs-rounding regression test in
    // motors_matrix_test.cpp.
    //
    // PARAMETER SHAPE - explicit parameters throughout, per ADR-0012 (this
    // port has no PWM-range-owning module yet): `pwm_output_min`/
    // `pwm_output_max` are typed `std::int16_t`, matching real upstream's
    // own `get_pwm_output_min()`/`get_pwm_output_max()` accessor return
    // type EXACTLY (AP_MotorsMulticopter.h: `int16_t get_pwm_output_min()
    // const { return _pwm_min; }`, likewise for max/_pwm_max, both
    // re-verified directly) rather than widening them to float - the
    // narrower int16_t choice documents that real upstream's own
    // configured PWM range can never itself carry a fractional value.
    // `armed`/`disarm_disable_pwm` reuse the IDENTICAL names/concepts as
    // CCP-013's own `output_logic(bool armed, bool interlock, bool
    // disarm_disable_pwm, ...)` signature above (re-checked directly) -
    // deliberately not reinvented under different names for the same two
    // real booleans. `spool_state` is this port's own real `SpoolState`
    // enum (CCP-013), now real/complete state rather than a placeholder.
    //
    // STATIC METHOD - like set_actuator_with_slew/
    // actuator_spin_up_to_ground_idle above (see that pair's own "STATIC
    // METHODS" design-decision comment), this function reads no
    // MotorsMatrix instance state - `spool_state` is taken as an explicit
    // parameter rather than reading the instance's own spool_state_,
    // matching the ticket's own explicit instruction that it be an
    // explicit parameter - so it is `static` rather than an ordinary
    // instance method, for the exact same reason.
    //
    // DEFERRED - NOT this ticket's scope (named explicitly, matching this
    // file's own "DEFERRED FUTURE PHASES" precedent): output_to_motors
    // (the real per-motor dispatcher that calls this function once per
    // motor, needing motor_enabled_/rc_write-equivalent output plumbing
    // this port has not built) and output_armed_stabilizing (the real
    // per-motor mixing algorithm, ~190 real lines - the single largest
    // remaining function in this whole output-stage effort) are both
    // separate, deliberately deferred future phases - see updated
    // "DEFERRED FUTURE PHASES" below.
    [[nodiscard]] static std::int16_t output_to_pwm(float actuator, SpoolState spool_state, bool armed,
                                                     bool disarm_disable_pwm, std::int16_t pwm_output_min,
                                                     std::int16_t pwm_output_max) {
        float pwm_output;
        if (spool_state == SpoolState::ShutDown) {
            if (disarm_disable_pwm && !armed) {
                pwm_output = 0.0f;
            } else {
                pwm_output = static_cast<float>(pwm_output_min);
            }
        } else {
            pwm_output = static_cast<float>(pwm_output_min) +
                         static_cast<float>(pwm_output_max - pwm_output_min) * actuator;
        }
        return static_cast<std::int16_t>(pwm_output);
    }

    // Test-only mutators (CCP-013) for output_logic's own new state -
    // matching CCP-011's own precedent (set_thrust_rpyt_out/
    // set_active_frame_type) of adding direct setters where no real
    // upstream setter exists yet for this port to call. `set_spool_
    // desired` is the RAW assignment upstream's own output_logic reads
    // (`_spool_desired`) - it is deliberately NOT the real, safety-
    // checked `AP_MotorsMulticopter::set_desired_spool_state` (a
    // separate real function, lines 565-577, outside this ticket's own
    // 591-768 scope) - tests that want the safety clamp exercise the
    // safety rule inside output_logic itself instead (see this ticket's
    // own dedicated tests). `set_spool_state` likewise lets tests seed
    // an arbitrary starting SpoolState directly, since this port has no
    // real init()/output_min()-equivalent constructor path yet.
    // `set_spin_up_ratio`/`set_idle_time` let tests seed the two GROUND_
    // IDLE ramps directly, so each of the three inner DesiredSpoolState
    // paths (and the idle-time early-break specifically) can be tested
    // from a precise, known starting point rather than compounding many
    // setup calls' own rounding into every assertion. `set_throttle_
    // thrust_max`/`set_thrust_boost_ratio` exist purely so the shared
    // post-inner-switch reset (throttle_thrust_max_/thrust_boost_/
    // thrust_boost_ratio_, all zeroed unconditionally regardless of
    // which inner path ran) can be proven to actually FIRE, by seeding a
    // nonzero value beforehand - both are otherwise always zero for any
    // MotorsMatrix this ticket's own scope can reach, so no test that
    // only read them could distinguish "the reset ran" from "it was
    // already zero".
    void set_spool_desired(DesiredSpoolState desired) { spool_desired_ = desired; }
    void set_spool_state(SpoolState state) { spool_state_ = state; }
    void set_spin_up_ratio(float value) { spin_up_ratio_ = value; }
    void set_idle_time(float value) { idle_time_ = value; }
    void set_throttle_thrust_max(float value) { throttle_thrust_max_ = value; }
    void set_thrust_boost_ratio(float value) { thrust_boost_ratio_ = value; }

    // Read accessors for output_logic's own new state (CCP-013).
    [[nodiscard]] SpoolState spool_state() const { return spool_state_; }
    [[nodiscard]] DesiredSpoolState spool_desired() const { return spool_desired_; }
    [[nodiscard]] float disarm_safe_timer() const { return disarm_safe_timer_; }
    [[nodiscard]] float spin_up_ratio() const { return spin_up_ratio_; }
    [[nodiscard]] float throttle_thrust_max() const { return throttle_thrust_max_; }
    [[nodiscard]] float idle_time() const { return idle_time_; }
    [[nodiscard]] bool spin_up_complete() const { return spin_up_complete_; }
    [[nodiscard]] float thrust_boost_ratio() const { return thrust_boost_ratio_; }

    // Accessors - not upstream methods (upstream reaches these fields as
    // protected member arrays from within the class hierarchy itself, or
    // via the small get_factors() accessor this ticket deliberately does
    // not port - see file banner), but the ticket's own required tests
    // need to observe per-motor state, so these are exposed directly.
    // All bounds-checked (an out-of-range i returns a harmless default
    // rather than invoking undefined behavior on the backing arrays) -
    // this is new port-only surface, not an upstream-mandated shape, so
    // safety takes precedence over mirroring a real accessor that does
    // not exist upstream.
    [[nodiscard]] bool motor_enabled(std::uint8_t i) const {
        return i < kMaxNumMotors && motor_enabled_[i];
    }
    [[nodiscard]] float roll_factor(std::uint8_t i) const { return i < kMaxNumMotors ? roll_factor_[i] : 0.0f; }
    [[nodiscard]] float pitch_factor(std::uint8_t i) const { return i < kMaxNumMotors ? pitch_factor_[i] : 0.0f; }
    [[nodiscard]] float yaw_factor(std::uint8_t i) const { return i < kMaxNumMotors ? yaw_factor_[i] : 0.0f; }
    [[nodiscard]] float throttle_factor(std::uint8_t i) const { return i < kMaxNumMotors ? throttle_factor_[i] : 0.0f; }
    [[nodiscard]] std::uint8_t test_order(std::uint8_t i) const { return i < kMaxNumMotors ? test_order_[i] : 0; }

private:
    bool initialised_ok_ = false;
    std::string frame_type_string_;
    std::string frame_class_string_;
    std::array<bool, kMaxNumMotors> motor_enabled_{};
    std::array<float, kMaxNumMotors> roll_factor_{};
    std::array<float, kMaxNumMotors> pitch_factor_{};
    std::array<float, kMaxNumMotors> yaw_factor_{};
    std::array<float, kMaxNumMotors> throttle_factor_{};
    std::array<std::uint8_t, kMaxNumMotors> test_order_{};

    // CCP-011 additions - see file banner's "CCP-011 ADDITION" for the
    // full rationale of each initial value below.
    std::array<float, kMaxNumMotors> thrust_rpyt_out_{};
    std::array<float, kMaxNumMotors> thrust_rpyt_out_filt_{};
    bool thrust_boost_ = false;   // AP_Motors_Class.cpp line 54, re-verified directly.
    bool thrust_balanced_ = true; // AP_Motors_Class.cpp line 55, re-verified directly.
    // DISCLOSED BUG FIX (NOT reproduced from upstream) - real upstream's
    // own _motor_lost_index has no in-class initializer and is never
    // assigned in the constructor, so it reads as indeterminate memory
    // until check_for_failed_motor's own first write. This port instead
    // gives it a real, defined initial value of 0 - confirmed harmless,
    // see file banner.
    std::uint8_t motor_lost_index_ = 0;
    // Real upstream's own _active_frame_type also has no in-class
    // initializer; given a real default here too, per this class's own
    // convention of never leaving a member indeterminate (see file
    // banner).
    FrameType active_frame_type_ = FrameType::Plus;
    // CCP-010's ThrustLinearization, held directly since this port has no
    // AP_MotorsMulticopter base class to inherit `thr_lin` from yet (see
    // file banner's "thr_lin_ MEMBER").
    ThrustLinearization thr_lin_;

    // CCP-013 additions - see file banner's "CCP-013 ADDITION" for the
    // full rationale of each initial value below.
    //
    // INVESTIGATED, NOT A BUG (corrects this ticket's own suspicion):
    // `_spool_state`/`_spool_desired` are declared on the real `AP_Motors`
    // BASE class (AP_Motors_Class.h lines 336-337, no in-class
    // initializer there), and this ticket's own text suspected the same
    // indeterminate-until-first-write bug class as CCP-011's
    // `_motor_lost_index`. Re-verified directly and found FALSE: real
    // `AP_Motors::AP_Motors(uint16_t)` (AP_Motors_Class.cpp lines 31-38)
    // explicitly initializes BOTH in its own member-initializer list -
    // `_spool_desired(DesiredSpoolState::SHUT_DOWN),
    // _spool_state(SpoolState::SHUT_DOWN)` - and
    // `AP_MotorsMulticopter::AP_MotorsMulticopter` (AP_MotorsMulticopter.cpp
    // lines 260-265) delegates to `AP_Motors(speed_hz)`, so that
    // initializer list genuinely runs for every real AP_MotorsMulticopter
    // instance. This port's own `spool_state_`/`spool_desired_` are given
    // the SAME real default (SpoolState::ShutDown / DesiredSpoolState::
    // ShutDown) below - a faithful port of upstream's actual behavior,
    // NOT a disclosed bug fix (unlike the six members below).
    SpoolState spool_state_ = SpoolState::ShutDown;
    DesiredSpoolState spool_desired_ = DesiredSpoolState::ShutDown;
    //
    // DISCLOSED BUG FIX (NOT reproduced from upstream), SAME CLASS as
    // CCP-011's own motor_lost_index_ finding - genuinely confirmed this
    // time: `_disarm_safe_timer`/`_spin_up_ratio`/`_throttle_thrust_max`/
    // `_idle_time`/`_spin_up_complete` (AP_MotorsMulticopter.h lines
    // 210-217, re-verified directly) and `_thrust_boost_ratio`
    // (AP_Motors_Class.h line 378) all have NO in-class default member
    // initializer. None of them is AP_Param (setup_object_defaults does
    // not touch them), and re-verified directly that NEITHER
    // AP_MotorsMulticopter's own constructor (AP_MotorsMulticopter.cpp
    // lines 260-265: only `_throttle_limit(1.0f)` in its own
    // member-initializer list, nothing else) NOR AP_Motors's own
    // constructor (AP_Motors_Class.cpp lines 31-53, re-verified in full:
    // only `_thrust_boost = false; _thrust_balanced = true;` are assigned
    // in the body - `_thrust_boost_ratio` itself is conspicuously absent)
    // assigns any of these six - real upstream therefore reads all six as
    // indeterminate memory until first written. Confirmed low-severity,
    // same reasoning class as CCP-011's own finding: `disarm_safe_timer_`
    // is unconditionally written by this method's own safety-preamble on
    // every call before any read; `spin_up_ratio_`/`throttle_thrust_max_`/
    // `idle_time_` are unconditionally zeroed by the SpoolState::ShutDown
    // case, which real upstream's own real startup path (disarmed forces
    // ShutDown) always reaches before anything reads them;
    // `spin_up_complete_` is written `false` on every GroundIdle+
    // ThrottleUnlimited-desired call where spin_up_ratio_ is still below
    // 1.0 (the common case while ramping up from 0), long before the
    // `else` branch would ever read it; `thrust_boost_ratio_` is
    // unconditionally zeroed by both the ShutDown and GroundIdle cases'
    // own shared resets. This port instead gives all six real, DEFINED
    // initial values (0.0f/false) - NOT reproduced as indeterminate,
    // matching CCP-011's own precedent and this class's own established
    // convention of never leaving a member indeterminate.
    float disarm_safe_timer_ = 0.0f;
    float spin_up_ratio_ = 0.0f;
    float throttle_thrust_max_ = 0.0f;
    float idle_time_ = 0.0f;
    bool spin_up_complete_ = false;
    float thrust_boost_ratio_ = 0.0f;
};

} // namespace fwcpp::motors
