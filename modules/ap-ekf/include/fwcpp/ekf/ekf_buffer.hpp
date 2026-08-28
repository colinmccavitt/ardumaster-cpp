#pragma once

// Port of AP_NavEKF/EKF_Buffer.h + EKF_Buffer.cpp. CPP-066, PHASE 12 - the
// FIRST step of a new, larger epic (delay-buffer/observation-time-horizon
// fusion). Every fusion phase built in this port so far (CPP-056/059/
// 062/063: GPS velocity/position, magnetometer, baro height, airspeed)
// explicitly discloses the same standing gap: it fuses an
// already-time-aligned sample against the EKF's CURRENT state, not
// upstream's real historical-time-horizon fusion (a late-arriving sample
// gets fused against what the state was AT THE TIME it was actually
// valid, then the correction propagates forward - see ekf_core.hpp's own
// "SIMPLIFICATION 5" for the standing disclosure this eventually
// retires). This file builds ONLY the standalone, generic ring-buffer
// infrastructure upstream uses for that. It is NOT wired into EkfCore:
// no EkfCore field is added, no existing fusion function's calling
// convention or behavior changes at all. That wiring, the actual
// fusion-time-horizon concept, and the complementary-filter output-state
// blending it requires (AP_NavEKF3_Outputs.cpp) are separate, later
// phases - see ticket CPP-066's "Explicitly out of scope" list.
//
// Upstream read directly from the pinned Plane-4.7.0 worktree:
// AP_NavEKF/EKF_Buffer.h (205 lines, all read) and EKF_Buffer.cpp (202
// lines, all read - CORRECTION: the ticket estimated ~120 lines; the
// real file is 202).
//
// TWO DISTINCT TYPES, matching upstream's own real separation - verified
// directly, NOT assumed: ekf_ring_buffer and ekf_imu_buffer are two
// separate upstream base classes with different members and different
// algorithms, not variants of one shared implementation.
//
//   1. ObsBuffer<T, N>  (upstream: EKF_obs_buffer_t<T> / ekf_ring_buffer)
//      A timestamp-keyed ring buffer for observation samples (GPS, mag,
//      baro, TAS, ...). push() writes at the head and evicts the oldest
//      element once at capacity. recall(sample_time_ms) DESTRUCTIVELY
//      searches forward from the oldest element for the NEWEST element
//      that is no more than 100ms older than sample_time_ms - consuming
//      (removing) every element it walks past, whether matched or
//      discarded as too old, up to and including the match. An element
//      strictly newer than sample_time_ms stops the search immediately,
//      left untouched. See recall()'s own comment below for the full
//      verified algorithm and a real, slightly surprising consequence of
//      it: recalling the same sample_time_ms twice does NOT return the
//      same result the second time (the match and everything older than
//      it is already gone).
//
//   2. ImuBuffer<T, N>  (upstream: EKF_IMU_buffer_t<T> / ekf_imu_buffer)
//      A fixed-depth history ring with NO timestamp logic at all -
//      genuinely different access pattern, not a variant of ObsBuffer.
//      push_youngest_element() always overwrites the next slot (wrapping
//      unconditionally, no eviction decision); get_oldest_element()/
//      operator[]/reset_history() give direct positional access, used by
//      upstream to maintain a rolling window of output/IMU states and to
//      cold-start every slot to the same value.
//
// ADAPTATIONS FROM UPSTREAM (both real and disclosed; neither changes the
// observable push/recall/reset/push_youngest/get_oldest/reset_history
// algorithm itself):
//
//   a. Fixed-capacity std::array<T, N> (N a compile-time template
//      parameter) instead of upstream's runtime malloc'd buffer
//      (ekf_ring_buffer::init(uint8_t size) / ekf_imu_buffer::
//      init(uint32_t size), backed by calloc()). This matches this
//      port's own established no-dynamic-allocation convention (ADR-0012
//      decision 4 - same precedent as fwcpp::Bitmask<N> and matrix_alg's
//      fixed-size paths).
//
//      CORRECTION TO THIS TICKET'S OWN PREMISE: the ticket asserts that
//      "upstream's own real calling code always uses fixed,
//      compile-time-known buffer sizes... to confirm this is a safe,
//      disclosed adaptation rather than a behavior change." This is
//      FALSE, verified directly in AP_NavEKF3_core.cpp's setup_core()
//      (~lines 29-165 of the pinned Plane-4.7.0 worktree). Upstream
//      computes imu_buffer_length and obs_buffer_length at RUNTIME:
//        - maxTimeDelay_ms = MAX() of several delay parameters
//          (_hgtDelay_ms, _flowDelay_ms, _rngBcnDelay_ms, magDelay_ms,
//          tasDelay_ms) AND dal.gps().get_lag() - a value the GPS DRIVER
//          reports at runtime, not a compile-time constant, clamped to
//          [0, 250] ms - AND (if enabled) the visual-odometry driver's
//          own reported delay.
//        - imu_buffer_length = maxTimeDelay_ms / EKF_TARGET_DT_MS + 1.
//        - obs_buffer_length = (maxTimeDelay_ms * 1.5) /
//          frontend->sensorIntervalMin_ms + 1, where sensorIntervalMin_ms
//          is itself an auto-detected minimum sample interval across
//          whichever sensors are actually enabled/present, again runtime
//          data, then MIN()'d against imu_buffer_length.
//        - storedGPS/storedMag/storedBaro/storedTAS/storedBodyOdm/
//          storedYawAng/storedDrag.init(obs_buffer_length); storedOF.
//          init(flow_buffer_length); storedIMU/storedOutput.
//          init(imu_buffer_length); storedRange.init(MIN(2*obs_buffer_
//          length, imu_buffer_length)); rngBcn.storedRange.init(
//          imu_buffer_length+1) - every one of these sizes is a runtime
//          value, none a compile-time constant.
//      So a compile-time-fixed N here is a REAL adaptation with an
//      observable consequence: a future integration phase must pick a
//      constant conservatively large enough for the parameter/sensor
//      ranges this port intends to support, rather than upstream's
//      exact-to-configured-delay runtime sizing. It is still the right
//      call for this port (no dynamic allocation, ADR-0012 decision 4)
//      - this correction is to the ticket's REASONING, not its
//      conclusion; the buffer-size decision itself is explicitly
//      out-of-scope future-phase work (see ticket CPP-066).
//
//   b. No void*/memcpy/elsize type erasure. Upstream's ekf_ring_buffer/
//      ekf_imu_buffer base classes exist purely so EKF_obs_buffer_t<T>/
//      EKF_IMU_buffer_t<T> can share one non-template implementation
//      compiled once across many element types (private inheritance +
//      a `const uint8_t elsize` + `void *buffer` + memcpy through
//      get_offset()). A template-only design holding std::array<T, N>
//      directly is simpler and type-safe in modern C++ and reproduces
//      byte-for-byte the same observable behavior - verified by reading
//      ekf_ring_buffer::push()/recall()/reset() and ekf_imu_buffer::
//      push_youngest_element()/get_oldest_element()/reset_history()/
//      reset() line-by-line (EKF_Buffer.cpp lines 53-202) and reproducing
//      their exact index/count/oldest/youngest/filled bookkeeping below,
//      including two more real, slightly surprising details verified
//      directly against upstream and preserved on purpose (not bugs):
//        - ekf_imu_buffer::push_youngest_element() increments _youngest
//          BEFORE writing (EKF_Buffer.cpp lines 154-169): the first-ever
//          push on a freshly-reset buffer writes to index 1, not index 0
//          - index 0 stays whatever it was (zero-initialized here,
//            matching calloc()) until the ring wraps all the way around.
//        - ekf_imu_buffer::reset() (lines 191-196) zeroes the stored data
//          and resets _oldest/_youngest to 0, but does NOT touch _filled
//          - is_filled() can still report true immediately after reset()
//          if it was true before. reset_history() (lines 183-188) is
//          even narrower: it overwrites every slot's data and touches
//          NOTHING else (not _oldest/_youngest/_filled), matching its
//          upstream doc comment ("writes the same data to all elements")
//          literally - it is a bulk data write, not a state reset.
//
// OUT OF SCOPE for this ticket (see ticket CPP-066's "Explicitly out of
// scope" list for the full detail):
//   - Wiring either type into EkfCore at all.
//   - The delay/fusion-time-horizon concept and the complementary-filter
//     output-state blending it requires (AP_NavEKF3_Outputs.cpp).
//   - Any specific buffer-size choice for a real sensor stream.

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace fwcpp::ekf {

// Requirement matching upstream's EKF_obs_element_t: any element type used
// with ObsBuffer must carry a measurement timestamp (milliseconds). Just
// as upstream expresses this via inheritance and a
// `static_assert(std::is_base_of<EKF_obs_element_t, element_type>::value,
// ...)` (EKF_Buffer.h lines 65-68), ObsBuffer requires element types to
// derive from ObsElement and enforces it the same way - matching both
// upstream's own mechanism and this port's established idiom of a
// static_assert documenting a template's structural contract (see
// modules/ap-common/include/fwcpp/bitmask.hpp for another example of that
// idiom, albeit for a different contract).
struct ObsElement {
    std::uint32_t time_ms = 0;
};

// ObsBuffer<T, N> - see file banner. T must derive from ObsElement; N is
// the fixed compile-time capacity (see ADAPTATION (a) above).
template <typename T, std::size_t N>
class ObsBuffer {
    static_assert(std::is_base_of_v<ObsElement, T>,
                  "T must derive from fwcpp::ekf::ObsElement (must carry a time_ms field), "
                  "matching upstream's EKF_obs_element_t requirement");
    static_assert(N > 0, "must store something");

public:
    ObsBuffer() = default;

    // Matches ekf_ring_buffer::push() (EKF_Buffer.cpp lines 86-103)
    // exactly: write the new element at the ring's current head
    // (oldest_ + count_) % N, then either grow count_ (not yet at
    // capacity) or advance oldest_ (at capacity - the current oldest
    // element is silently overwritten/discarded).
    void push(const T& element) {
        const std::size_t head = (oldest_ + count_) % N;
        buffer_[head] = element;
        if (count_ < N) {
            ++count_;
        } else {
            oldest_ = (oldest_ + 1) % N;
        }
    }

    // Matches ekf_ring_buffer::recall() (EKF_Buffer.cpp lines 53-80)
    // exactly. Walks forward from oldest_ while count_ > 0. For each
    // element: dt = sample_time_ms - element.time_ms (matching upstream's
    // exact uint32_t-subtract-then-reinterpret-as-int32_t arithmetic,
    // well-defined in C++20). If 0 <= dt < 100 (element is no more than
    // 100ms older than sample_time_ms), remember it as the current best
    // match and KEEP GOING - a later matching element OVERWRITES the
    // earlier one, so the final match is the NEWEST element within the
    // window, not the first found. If dt < 0 (this element is actually
    // newer than sample_time_ms), STOP immediately WITHOUT consuming it.
    // Otherwise (matched, or more than 100ms too old), CONSUME the
    // element (it can never be recalled again) and continue. Returns
    // false, leaving `out` and the buffer's already-consumed elements
    // as-is (this last part IS a real observable effect, not a no-op:
    // every too-old element walked past before failure is still gone),
    // if the search never finds a match.
    //
    // REAL, VERIFIED, SLIGHTLY SURPRISING CONSEQUENCE (tested explicitly
    // in ekf_buffer_test.cpp): a successful recall() destructively
    // removes every element up to and including the match. Calling
    // recall() again with the exact same sample_time_ms will NOT return
    // the same element - it has already been consumed - and may return a
    // different (later) element, or fail, depending on what is left.
    bool recall(T& out, std::uint32_t sample_time_ms) {
        bool found = false;
        std::size_t best_index = 0;
        while (count_ > 0) {
            const std::uint32_t t_oldest = buffer_[oldest_].time_ms;
            const auto dt = static_cast<std::int32_t>(sample_time_ms - t_oldest);
            const bool matches = dt >= 0 && dt < 100;
            if (matches) {
                best_index = oldest_;
                found = true;
            }
            if (dt < 0) {
                // oldest_ is newer than requested - stop, don't consume it.
                break;
            }
            // Consume: matched, or more than 100ms too old either way.
            --count_;
            oldest_ = (oldest_ + 1) % N;
        }
        if (found) {
            out = buffer_[best_index];
        }
        return found;
    }

    // CPP-075 (NavEKF3-equivalent phase 21): read the CURRENT oldest
    // element WITHOUT consuming it - a minimal, GENERIC "peek" capability
    // with NO upstream equivalent (upstream's own ekf_ring_buffer /
    // EKF_obs_buffer_t<T> has no peek-without-consuming method at all;
    // this is new, additive infrastructure this port adds on top of the
    // faithfully-ported recall()/push()/reset() above - it does not
    // change recall()'s own existing, already-verified destructive-
    // consumption/newest-match-wins semantics in any way). It exists so
    // a caller can inspect whatever recall()'s own "an element strictly
    // newer than sample_time_ms stops the search immediately, left
    // untouched" behaviour (see recall()'s comment above) left behind,
    // without re-running the search itself, duplicating any storage, or
    // touching count_/oldest_ - the buffer is completely unaffected by a
    // call to this method, successful or not.
    //
    // Deliberately GENERIC (element-type-agnostic, like every other
    // method on this class) - this port's GPS-specific interpolation
    // logic that USES this peek belongs, and lives, in EkfCore itself
    // (see ekf_core.hpp's recall_gps_sample_interpolated()), not here:
    // ObsBuffer<T,N> is shared, sensor-agnostic infrastructure also used
    // by mag/baro/airspeed, and interpolating a GpsSample's
    // velocity_ned/position_ne fields is domain-specific knowledge that
    // does not belong in this generic buffer - matching this port's own
    // established sensor-agnostic-buffer-vs-sensor-specific-wrapper
    // convention (recall_gps_sample() itself is exactly this kind of
    // thin, GPS-specific wrapper over the generic recall() above).
    //
    // Returns false (leaving `out` untouched) if the buffer is empty.
    [[nodiscard]] bool peek_oldest(T& out) const {
        if (count_ == 0) {
            return false;
        }
        out = buffer_[oldest_];
        return true;
    }

    // Matches ekf_ring_buffer::reset(): drop all elements (does not
    // clear the stored data itself, matching upstream - only count_/
    // oldest_ are touched, exactly as upstream's reset() only zeroes
    // `count`/`oldest`, not the underlying buffer memory).
    void reset() {
        count_ = 0;
        oldest_ = 0;
    }

    [[nodiscard]] std::size_t size() const { return count_; }
    [[nodiscard]] static constexpr std::size_t capacity() { return N; }
    [[nodiscard]] bool empty() const { return count_ == 0; }

private:
    std::array<T, N> buffer_{};
    std::size_t oldest_ = 0;
    std::size_t count_ = 0;
};

// ImuBuffer<T, N> - see file banner. A genuinely different data structure
// from ObsBuffer, matching upstream's own separate ekf_imu_buffer base
// class (EKF_Buffer.h lines 95-143, EKF_Buffer.cpp lines 119-202). No
// timestamp requirement on T (upstream's ekf_imu_buffer has none either).
template <typename T, std::size_t N>
class ImuBuffer {
    static_assert(N > 0, "must store something");

public:
    ImuBuffer() = default;

    // Matches ekf_imu_buffer::push_youngest_element() (EKF_Buffer.cpp
    // lines 154-169) EXACTLY, including its pre-increment-then-write
    // order: youngest_ is incremented FIRST (wrapping to 0 and marking
    // the buffer "filled" the first time it wraps all the way around),
    // THEN the new element is written at the (new) youngest_ index, THEN
    // oldest_ is recomputed as (youngest_+1) % N. See file banner for the
    // real consequence: the very first push on a fresh buffer writes
    // index 1, not index 0.
    void push_youngest_element(const T& element) {
        ++youngest_;
        if (youngest_ == N) {
            youngest_ = 0;
            filled_ = true;
        }
        buffer_[youngest_] = element;
        oldest_ = (youngest_ + 1) % N;
    }

    // return true if the buffer has been filled at least once (matches
    // upstream's is_filled(); see file banner - reset() does NOT clear
    // this flag, only reset_history()+a full wrap of pushes would ever
    // need to "unfill" it, and upstream never does that either).
    [[nodiscard]] bool is_filled() const { return filled_; }

    // Matches ekf_imu_buffer::get_oldest_element() (returns a copy, like
    // the EKF_IMU_buffer_t<T> template wrapper's own by-value return).
    [[nodiscard]] T get_oldest_element() const { return buffer_[oldest_]; }

    // Matches ekf_imu_buffer::reset_history() (EKF_Buffer.cpp lines
    // 183-188) EXACTLY: writes `element` to every slot and touches
    // NOTHING else - not oldest_/youngest_/filled_. This is a bulk data
    // write, not a state reset (see file banner); used by upstream for
    // cold-start initialization right after init(), when oldest_/
    // youngest_/filled_ are already at their fresh-buffer defaults.
    void reset_history(const T& element) {
        for (auto& slot : buffer_) {
            slot = element;
        }
    }

    // Matches ekf_imu_buffer::reset() (EKF_Buffer.cpp lines 191-196)
    // EXACTLY: zeroes youngest_/oldest_ back to 0 and default-constructs
    // every slot's data, but deliberately does NOT touch filled_ (see
    // file banner - verified directly, not an oversight).
    void reset() {
        youngest_ = 0;
        oldest_ = 0;
        buffer_.fill(T{});
    }

    // Matches ekf_imu_buffer::get(index) / EKF_IMU_buffer_t<T>::
    // operator[](): raw positional access, no bounds check (matching
    // upstream, which does none either) and no relation to oldest_/
    // youngest_ - caller's responsibility to pass an index < N.
    [[nodiscard]] T& operator[](std::size_t index) { return buffer_[index]; }
    [[nodiscard]] const T& operator[](std::size_t index) const { return buffer_[index]; }

    [[nodiscard]] std::size_t get_oldest_index() const { return oldest_; }
    [[nodiscard]] std::size_t get_youngest_index() const { return youngest_; }

    [[nodiscard]] static constexpr std::size_t capacity() { return N; }

private:
    std::array<T, N> buffer_{};
    std::size_t oldest_ = 0;
    std::size_t youngest_ = 0;
    bool filled_ = false;
};

} // namespace fwcpp::ekf
