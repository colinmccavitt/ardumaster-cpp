#pragma once

// Port of libraries/AP_Motors/AP_MotorsMatrix.cpp's (Copter-4.7.0) CORE
// factor storage/arithmetic - CCP-001, the first ticket of the copter-cpp
// effort (see /srv/ardumaster/tracker/efforts/copter-cpp.md) - PLUS
// (CCP-002) the first real consumer of that infrastructure,
// setup_quad_matrix (AP_MotorsMatrix.cpp ~line 576-711). Deliberately NOT
// the other six frame-class setup functions (setup_hexa_matrix through
// setup_deca_matrix), NOT setup_motors' own dispatcher, and NOT the output
// stage (output_to_motors/output_armed_stabilizing/check_for_failed_motor/
// thrust_compensation/disable_yaw_torque) - all real, substantial,
// separately-scoped future phases. See "DEFERRED FUTURE PHASES" below.
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
//   - AP_Motors_Class.h: initialised_ok()/set_initialised_ok() (lines
//     113-114), add_motor_num() declaration (line 310).
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
// DEFERRED FUTURE PHASES (named explicitly, not silently omitted):
//   1. Remaining frame tables - setup_quad_matrix (line 576) is DONE as of
//      CCP-002 (see "CCP-002 ADDITION" above); the other six frame-class
//      setup functions (setup_hexa_matrix, setup_octa_matrix,
//      setup_octaquad_matrix, setup_dodecahexa_matrix, setup_y6_matrix,
//      setup_deca_matrix) and setup_motors' own dispatcher (which chooses
//      among all seven by frame CLASS, ~1,400 upstream lines total across
//      all seven) remain future phases, named explicitly here so none is
//      silently folded into another ticket's scope. A future phase
//      MUST independently re-verify (not blindly trust) copter-rust's own
//      COP-005 findings:
//        a. Y6's `default:` switch branch is productive - it answers for
//           24 of the real 64 upstream frame configurations. A naive
//           switch-by-switch transcription silently drops all 24.
//        b. The co-rotating X8 frames (X_COR/CW_X_COR) scale their top
//           rotor layer by 0.9 AFTER the frame table is applied, and
//           disagree about which motors that applies to (X_COR: first
//           four; CW_X_COR: every other one of eight).
//        c. That 0.9 is a real `float` literal (upstream builds with
//           -fsingle-precision-constant). THIS PORT'S OWN root
//           CMakeLists.txt already defines an `fwcpp_upstream_flags`
//           INTERFACE target carrying -fsingle-precision-constant
//           (confirmed present, read directly) - so a future frame-table
//           module that links it as PRIVATE (matching ap-math/ap-common/
//           ap-filter's own established precedent, see those modules'
//           CMakeLists.txt) inherits the correct literal-typing behavior
//           for free. NOT independently re-confirmed end-to-end against a
//           compiled 0.9-scaling frame here (this ticket has no X8 frame
//           code to compile yet) - the future phase must still confirm it
//           empirically once that code exists, not just trust this note.
//   2. Output stage (output_to_motors, output_armed_stabilizing,
//      check_for_failed_motor, thrust_compensation, disable_yaw_torque) -
//      needs real thrust-linearization/battery-compensation infrastructure
//      this port has not built yet (the future C++ analogue of
//      copter-rust's already-done COP-006), plus real current-limiting.
//      This is also where add_motor_num()'s real SRV_Channels registration
//      belongs (see NO-OP above).
//   3. set_throttle_factor/set_update_rate/set_frame_class_and_type/
//      output_test_num/_output_test_seq/get_factors - small real
//      accessors/setters not needed by this ticket's own core scope; add
//      only when a real future test/caller needs one.
//
// THIS TICKET'S OWN LITERALS ARE ALL EXPLICITLY float-SUFFIXED (0.5f,
// 0.0f) IN BOTH UPSTREAM AND HERE - unlike the frame tables' bare `0.9`,
// -fsingle-precision-constant has no observable effect on this file's own
// arithmetic, so this module does not link fwcpp_upstream_flags (matching
// ap-compass/ap-gps's own precedent of not linking it for header-only
// INTERFACE modules with no ambiguously-typed literals).

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <fwcpp/math/scalar.hpp>

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

    // CCP-002: port of upstream's real `enum class motor_frame_type`
    // (AP_Motors_Class.h), restricted to exactly the enumerators
    // setup_quad_matrix's own real switch statement handles (NOT the full
    // upstream enum, which also has many other frame-class-specific
    // values this port has not built setup functions for yet - see file
    // banner's "CCP-002 ADDITION" for the exhaustive real case list this
    // was checked against). NYT_PLUS/NYT_X are deliberately absent - see
    // file banner's "EXCLUDED" note. Named/shaped per this port's own
    // house style for small state enums (e.g. CalibrationState,
    // fwcpp/airspeed/airspeed_sensor.hpp).
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

    // frame_type_string()/frame_class_string() - port of upstream's real
    // _frame_type_string/_frame_class_string state (set by
    // setup_quad_matrix above), even though this port has no GCS to
    // report them to - useful for tests/debugging, per the ticket's own
    // request. Empty until setup_quad_matrix has been called at least
    // once (matching upstream's own uninitialized-until-setup behavior).
    [[nodiscard]] const std::string& frame_type_string() const { return frame_type_string_; }
    [[nodiscard]] const std::string& frame_class_string() const { return frame_class_string_; }

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
};

} // namespace fwcpp::motors
