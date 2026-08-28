// Tests for fwcpp::ekf::ObsBuffer<T, N> and fwcpp::ekf::ImuBuffer<T, N>
// (CPP-066, PHASE 12 - standalone generic ring-buffer infrastructure, not
// wired into EkfCore). See fwcpp/ekf/ekf_buffer.hpp's file banner for the
// full upstream-verification notes and the two adaptations from upstream
// (fixed-size std::array capacity, no void*/memcpy type erasure).

#include <cstdint>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/ekf/ekf_buffer.hpp>

using fwcpp::ekf::ImuBuffer;
using fwcpp::ekf::ObsBuffer;
using fwcpp::ekf::ObsElement;

namespace {

struct TestObs : ObsElement {
    int value = 0;
};

struct TestImu {
    int value = 0;
};

} // namespace

// ---------------------------------------------------------------------
// ObsBuffer<T, N>
// ---------------------------------------------------------------------

TEST_CASE("ObsBuffer starts empty", "[ekf_buffer][obs]") {
    ObsBuffer<TestObs, 4> buf;
    REQUIRE(buf.empty());
    REQUIRE(buf.size() == 0);
    REQUIRE(buf.capacity() == 4);

    TestObs out;
    REQUIRE_FALSE(buf.recall(out, 1000));
}

TEST_CASE("ObsBuffer push-then-recall returns the matching element within the 100ms window",
          "[ekf_buffer][obs]") {
    ObsBuffer<TestObs, 4> buf;
    buf.push(TestObs{{1000}, 42});
    REQUIRE(buf.size() == 1);

    TestObs out;
    REQUIRE(buf.recall(out, 1050)); // dt = 50, within [0, 100)
    REQUIRE(out.value == 42);
    // Consumed: the matched element (and everything up to it) is gone.
    REQUIRE(buf.size() == 0);
    REQUIRE(buf.empty());
}

TEST_CASE("ObsBuffer recall window is half-open: dt of 0 to 99 matches, dt of 100 does not", "[ekf_buffer][obs]") {
    ObsBuffer<TestObs, 4> buf;
    buf.push(TestObs{{1000}, 1});

    SECTION("dt == 99 matches") {
        TestObs out;
        REQUIRE(buf.recall(out, 1099));
        REQUIRE(out.value == 1);
    }
    SECTION("dt == 100 does not match (too old) and is consumed as a miss") {
        TestObs out;
        REQUIRE_FALSE(buf.recall(out, 1100));
        REQUIRE(buf.empty()); // consumed even though it never matched
    }
    SECTION("dt == 0 matches (exact timestamp)") {
        TestObs out;
        REQUIRE(buf.recall(out, 1000));
        REQUIRE(out.value == 1);
    }
    SECTION("dt == -1 (element newer than query) leaves the buffer untouched") {
        TestObs out;
        REQUIRE_FALSE(buf.recall(out, 999));
        REQUIRE(buf.size() == 1); // not consumed
    }
}

TEST_CASE("ObsBuffer recall picks the NEWEST match within the window, not the first",
          "[ekf_buffer][obs]") {
    // All three are within 100ms of the query (dt = 99, 49, 19) - upstream's
    // recall() keeps overwriting best_index as it walks forward, so the
    // final match is the newest (largest time_ms) one, not element #1.
    ObsBuffer<TestObs, 4> buf;
    buf.push(TestObs{{1000}, 1});
    buf.push(TestObs{{1050}, 2});
    buf.push(TestObs{{1080}, 3});

    TestObs out;
    REQUIRE(buf.recall(out, 1099));
    REQUIRE(out.value == 3); // newest-within-window wins
    // All three were walked past (matched or consumed) - buffer is drained.
    REQUIRE(buf.empty());
}

TEST_CASE("ObsBuffer recall stops on an element newer than sample_time, leaving it unconsumed",
          "[ekf_buffer][obs]") {
    ObsBuffer<TestObs, 4> buf;
    buf.push(TestObs{{1000}, 1}); // dt = 50 vs query 1050: matches
    buf.push(TestObs{{2000}, 2}); // dt = -950 vs query 1050: newer, stop here

    TestObs out;
    REQUIRE(buf.recall(out, 1050));
    REQUIRE(out.value == 1);
    // The 2000ms element is strictly newer than the query and must survive.
    REQUIRE(buf.size() == 1);

    TestObs remaining;
    REQUIRE(buf.recall(remaining, 2050));
    REQUIRE(remaining.value == 2);
}

TEST_CASE("ObsBuffer recall with a query far in the future (beyond tolerance) drains the "
          "buffer and returns false",
          "[ekf_buffer][obs]") {
    ObsBuffer<TestObs, 4> buf;
    buf.push(TestObs{{1000}, 1});
    buf.push(TestObs{{1010}, 2});

    TestObs out;
    // Every stored element is more than 100ms older than 5000 - each one
    // is walked past and consumed as "too old", never matching, until the
    // buffer empties and the loop terminates with no match.
    REQUIRE_FALSE(buf.recall(out, 5000));
    REQUIRE(buf.empty());
}

TEST_CASE("CORRECTION: a query older than everything in the buffer does NOT drain it - "
          "it stops immediately, untouched",
          "[ekf_buffer][obs]") {
    // Ticket CPP-066's own premise describes "a query time older than
    // everything in the buffer" as draining the buffer and returning
    // false. Verified directly against upstream's real recall()
    // (EKF_Buffer.cpp lines 53-80): when sample_time_ms is OLDER than the
    // oldest stored element, dt = sample_time_ms - element.time_ms is
    // already negative on the very first element checked, which hits the
    // "dt < 0 -> stop, don't consume" branch immediately. The buffer is
    // therefore left completely UNTOUCHED, not drained. Draining-to-empty
    // only happens for the opposite case (query far in the FUTURE, beyond
    // every element's 100ms window - see the test above). This test
    // documents that correction explicitly.
    ObsBuffer<TestObs, 4> buf;
    buf.push(TestObs{{5000}, 1});

    TestObs out;
    REQUIRE_FALSE(buf.recall(out, 100)); // sample_time (100) << element time (5000)
    REQUIRE(buf.size() == 1);            // untouched, not drained
}

TEST_CASE("ObsBuffer at capacity evicts its oldest element on push", "[ekf_buffer][obs]") {
    ObsBuffer<TestObs, 3> buf;
    buf.push(TestObs{{1000}, 1});
    buf.push(TestObs{{1010}, 2});
    buf.push(TestObs{{1020}, 3});
    REQUIRE(buf.size() == 3);

    // Buffer is full; this push must evict time_ms=1000/value=1.
    buf.push(TestObs{{1030}, 4});
    REQUIRE(buf.size() == 3);

    TestObs out;
    // The oldest surviving element should now be value=2 (time_ms=1010).
    // A query timed just past 1010 must not be able to recall value=1
    // (it's gone) - it should land on value=2 instead.
    REQUIRE(buf.recall(out, 1010));
    REQUIRE(out.value == 2);
}

TEST_CASE("ObsBuffer recall is destructive: repeating the same query does not return the "
          "same result twice",
          "[ekf_buffer][obs]") {
    // Real, verified upstream design choice (not a port bug): recall()
    // removes everything up to and including its match. Recalling the
    // identical sample_time_ms again cannot see that data again.
    ObsBuffer<TestObs, 4> buf;
    buf.push(TestObs{{1000}, 1});
    buf.push(TestObs{{1010}, 2});

    TestObs first;
    REQUIRE(buf.recall(first, 1050)); // both within window; newest (1010) wins
    REQUIRE(first.value == 2);
    REQUIRE(buf.empty());

    TestObs second;
    // Same query time again - but the data is gone, so this now fails,
    // rather than returning value 2 again.
    REQUIRE_FALSE(buf.recall(second, 1050));
}

TEST_CASE("ObsBuffer::reset drops all elements", "[ekf_buffer][obs]") {
    ObsBuffer<TestObs, 4> buf;
    buf.push(TestObs{{1000}, 1});
    buf.push(TestObs{{1010}, 2});
    REQUIRE(buf.size() == 2);

    buf.reset();
    REQUIRE(buf.empty());
    REQUIRE(buf.capacity() == 4);

    TestObs out;
    REQUIRE_FALSE(buf.recall(out, 1050));
}

// ---------------------------------------------------------------------
// ImuBuffer<T, N> - a distinct type/access pattern, tested separately.
// ---------------------------------------------------------------------

TEST_CASE("ImuBuffer starts unfilled with oldest == youngest == 0", "[ekf_buffer][imu]") {
    ImuBuffer<TestImu, 3> buf;
    REQUIRE_FALSE(buf.is_filled());
    REQUIRE(buf.get_oldest_index() == 0);
    REQUIRE(buf.get_youngest_index() == 0);
    REQUIRE(buf.get_oldest_element().value == 0); // default-constructed slot
}

TEST_CASE("ImuBuffer::push_youngest_element writes index 1 first, not index 0",
          "[ekf_buffer][imu]") {
    // Verified directly against ekf_imu_buffer::push_youngest_element()
    // (EKF_Buffer.cpp lines 154-169): youngest is pre-incremented before
    // the write, so the very first push on a fresh buffer lands on index
    // 1 - index 0 stays untouched (default/zero) until the ring wraps.
    ImuBuffer<TestImu, 3> buf;
    buf.push_youngest_element(TestImu{7});

    REQUIRE(buf.get_youngest_index() == 1);
    REQUIRE(buf[1].value == 7);
    REQUIRE(buf[0].value == 0); // never written yet
    REQUIRE_FALSE(buf.is_filled());
}

TEST_CASE("ImuBuffer becomes filled exactly when the ring wraps back to index 0",
          "[ekf_buffer][imu]") {
    ImuBuffer<TestImu, 3> buf;
    buf.push_youngest_element(TestImu{1}); // writes idx 1
    REQUIRE_FALSE(buf.is_filled());
    buf.push_youngest_element(TestImu{2}); // writes idx 2
    REQUIRE_FALSE(buf.is_filled());
    buf.push_youngest_element(TestImu{3}); // wraps: writes idx 0, filled = true
    REQUIRE(buf.is_filled());
    REQUIRE(buf.get_youngest_index() == 0);
    REQUIRE(buf[0].value == 3);
}

TEST_CASE("ImuBuffer::get_oldest_element tracks (youngest + 1) % N positionally",
          "[ekf_buffer][imu]") {
    ImuBuffer<TestImu, 3> buf;
    buf.push_youngest_element(TestImu{1}); // youngest=1, oldest=(1+1)%3=2 (never written yet)
    REQUIRE(buf.get_oldest_index() == 2);
    REQUIRE(buf.get_oldest_element().value == 0); // real upstream quirk: unwritten slot

    buf.push_youngest_element(TestImu{2}); // youngest=2, oldest=(2+1)%3=0 (never written yet)
    REQUIRE(buf.get_oldest_index() == 0);
    REQUIRE(buf.get_oldest_element().value == 0);

    buf.push_youngest_element(TestImu{3}); // wraps: youngest=0, oldest=(0+1)%3=1
    REQUIRE(buf.get_oldest_index() == 1);
    REQUIRE(buf.get_oldest_element().value == 1); // idx 1 holds the first-ever push
}

TEST_CASE("ImuBuffer::push_youngest_element overwrites the oldest slot once full",
          "[ekf_buffer][imu]") {
    ImuBuffer<TestImu, 3> buf;
    buf.push_youngest_element(TestImu{1}); // idx1=1
    buf.push_youngest_element(TestImu{2}); // idx2=2
    buf.push_youngest_element(TestImu{3}); // idx0=3, wraps, filled=true
    buf.push_youngest_element(TestImu{4}); // idx1=4, overwriting the old value 1

    REQUIRE(buf.is_filled());
    REQUIRE(buf.get_youngest_index() == 1);
    REQUIRE(buf.get_oldest_index() == 2);
    REQUIRE(buf[0].value == 3);
    REQUIRE(buf[1].value == 4); // was 1, now overwritten
    REQUIRE(buf[2].value == 2);
    REQUIRE(buf.get_oldest_element().value == 2);
}

TEST_CASE("ImuBuffer::reset_history writes one element to every slot without touching "
          "indices or the filled flag",
          "[ekf_buffer][imu]") {
    ImuBuffer<TestImu, 3> buf;
    buf.push_youngest_element(TestImu{1});
    buf.push_youngest_element(TestImu{2});
    buf.push_youngest_element(TestImu{3}); // wraps -> filled = true, youngest=0, oldest=1

    buf.reset_history(TestImu{99});

    REQUIRE(buf[0].value == 99);
    REQUIRE(buf[1].value == 99);
    REQUIRE(buf[2].value == 99);
    // Indices and filled state are untouched by reset_history() - it is a
    // pure bulk data write, matching upstream's memcpy-every-slot loop
    // that never references _oldest/_youngest/_filled at all.
    REQUIRE(buf.get_youngest_index() == 0);
    REQUIRE(buf.get_oldest_index() == 1);
    REQUIRE(buf.is_filled());
}

TEST_CASE("ImuBuffer::reset zeroes data and indices but preserves is_filled()",
          "[ekf_buffer][imu]") {
    ImuBuffer<TestImu, 3> buf;
    buf.push_youngest_element(TestImu{1});
    buf.push_youngest_element(TestImu{2});
    buf.push_youngest_element(TestImu{3}); // wraps -> filled = true
    REQUIRE(buf.is_filled());

    buf.reset();

    REQUIRE(buf.get_oldest_index() == 0);
    REQUIRE(buf.get_youngest_index() == 0);
    REQUIRE(buf[0].value == 0);
    REQUIRE(buf[1].value == 0);
    REQUIRE(buf[2].value == 0);
    // Real, verified upstream quirk (EKF_Buffer.cpp lines 191-196):
    // reset() does NOT clear _filled/filled_, unlike reset_history()'s
    // sibling omission above - both leave the flag alone, reset() just
    // additionally clears the data and indices.
    REQUIRE(buf.is_filled());
}

TEST_CASE("ImuBuffer capacity() reports the compile-time size", "[ekf_buffer][imu]") {
    ImuBuffer<TestImu, 5> buf;
    REQUIRE(buf.capacity() == 5);
}
