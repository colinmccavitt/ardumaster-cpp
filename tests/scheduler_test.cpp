// Tests for fwcpp::scheduler::Scheduler (CPP-026 slice 1).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/scheduler/scheduler.hpp>

#include <array>
#include <vector>

using namespace fwcpp::scheduler;

namespace {
// A simple, deterministic, caller-controlled time source: each call
// advances by a fixed step - lets tests know exactly what "now" is at
// every point run() samples it.
struct FakeClock {
    std::uint32_t us = 0;
    std::uint32_t step_us = 100; // simulates each task taking 100us
};

std::uint32_t fake_now(void* ctx) {
    auto* clock = static_cast<FakeClock*>(ctx);
    clock->us += clock->step_us;
    return clock->us;
}

TimeSource make_time_source(FakeClock& clock) {
    return TimeSource{fake_now, &clock};
}

// Records which task ran, by an arbitrary caller-chosen index, into a
// shared log - lets tests assert both which tasks ran and in what order,
// without templates, globals, or heap allocation.
struct RecordingContext {
    std::vector<int>* log;
    int index;
};

void record_call(void* ctx_ptr) {
    auto* ctx = static_cast<RecordingContext*>(ctx_ptr);
    ctx->log->push_back(ctx->index);
}

TaskFn make_recording_task(RecordingContext& ctx) {
    return TaskFn{&record_call, &ctx};
}
} // namespace

TEST_CASE("tick() increments both the 16-bit and 32-bit counters", "[scheduler]") {
    Scheduler sched(50);
    REQUIRE(sched.ticks() == 0);
    sched.tick();
    sched.tick();
    REQUIRE(sched.ticks() == 2);
    REQUIRE(sched.ticks32() == 2);
}

TEST_CASE("loop_period_us matches 1000000/loop_rate_hz", "[scheduler]") {
    Scheduler sched(50);
    REQUIRE(sched.loop_period_us() == 20000); // 1e6/50
    Scheduler sched2(400);
    REQUIRE(sched2.loop_period_us() == 2500); // 1e6/400
}

TEST_CASE("a rate_hz==0 task ('every loop') runs on every tick once due", "[scheduler]") {
    Scheduler sched(50);
    std::vector<int> log;
    RecordingContext ctx{&log, 0};

    Task tasks[] = {
        {make_recording_task(ctx), "every_loop", 0.0f, 1000, kMaxFastTaskPriorities + 1},
    };
    std::array<std::uint16_t, 1> last_run{0};
    FakeClock clock;

    for (int i = 0; i < 3; ++i) {
        sched.tick();
        sched.run(tasks, {}, last_run, 10000, make_time_source(clock));
    }

    REQUIRE(log.size() == 3);
}

TEST_CASE("a rate-limited task only runs once its interval has elapsed", "[scheduler]") {
    Scheduler sched(100); // 100Hz loop
    std::vector<int> log;
    RecordingContext ctx{&log, 0};

    // rate_hz = 10 -> interval_ticks = 100/10 = 10 ticks between runs
    Task tasks[] = {
        {make_recording_task(ctx), "slow_task", 10.0f, 1000, kMaxFastTaskPriorities + 1},
    };
    std::array<std::uint16_t, 1> last_run{0};
    FakeClock clock;

    for (int i = 0; i < 25; ++i) {
        sched.tick();
        sched.run(tasks, {}, last_run, 10000, make_time_source(clock));
    }

    // Should have run at ticks 10 and 20 (dt >= interval_ticks=10) - 2 times in 25 ticks.
    REQUIRE(log.size() == 2);
}

TEST_CASE("a fast task (priority <= kMaxFastTaskPriorities) runs every tick regardless of rate_hz", "[scheduler]") {
    Scheduler sched(50);
    std::vector<int> log;
    RecordingContext ctx{&log, 0};

    Task tasks[] = {
        {make_recording_task(ctx), "fast_task", 1.0f /* would normally be slow */, 1000, 0 /* FAST_TASK_PRI0 */},
    };
    std::array<std::uint16_t, 1> last_run{0};
    FakeClock clock;

    for (int i = 0; i < 5; ++i) {
        sched.tick();
        sched.run(tasks, {}, last_run, 10000, make_time_source(clock));
    }

    REQUIRE(log.size() == 5); // every tick, not rate-limited
}

TEST_CASE("vehicle and common tasks merge by priority, vehicle wins ties", "[scheduler]") {
    Scheduler sched(50);
    std::vector<int> log;
    RecordingContext c1{&log, 1};
    RecordingContext c2{&log, 2};
    RecordingContext c3{&log, 3};
    RecordingContext c4{&log, 4};

    Task vehicle_tasks[] = {
        {make_recording_task(c1), "vehicle_pri5", 0.0f, 1000, 5},
        {make_recording_task(c3), "vehicle_pri10", 0.0f, 1000, 10},
    };
    Task common_tasks[] = {
        {make_recording_task(c2), "common_pri5", 0.0f, 1000, 5}, // tie with vehicle_pri5 - vehicle should run first
        {make_recording_task(c4), "common_pri20", 0.0f, 1000, 20},
    };
    std::array<std::uint16_t, 4> last_run{0, 0, 0, 0};
    FakeClock clock;

    sched.tick();
    sched.run(vehicle_tasks, common_tasks, last_run, 100000, make_time_source(clock));

    REQUIRE(log == std::vector<int>{1, 2, 3, 4}); // priority order, vehicle-first on the tie
}

TEST_CASE("a task is skipped (not run) when its max_time_micros exceeds time_available_us, but a later fast task still runs", "[scheduler]") {
    Scheduler sched(50);
    std::vector<int> log;
    RecordingContext c0{&log, 0};
    RecordingContext c1{&log, 1};

    Task tasks[] = {
        {make_recording_task(c0), "needs_a_lot", 0.0f, 5000, kMaxFastTaskPriorities + 1}, // ordinary task, needs 5000us
        {make_recording_task(c1), "fast", 0.0f, 1000, 0}, // fast task, always runs regardless of budget
    };
    std::array<std::uint16_t, 2> last_run{0, 0};
    FakeClock clock;

    sched.tick();
    // Only 100us available - not enough for the first (ordinary) task's
    // declared 5000us budget, so it's skipped; the fast task still runs.
    sched.run(tasks, {}, last_run, 100, make_time_source(clock));

    REQUIRE(log == std::vector<int>{1});
}

TEST_CASE("time_available_us running out mid-loop still allows subsequent fast tasks to run", "[scheduler]") {
    Scheduler sched(50);
    FakeClock clock;
    clock.step_us = 50000; // each "task" consumes more time than the whole budget

    std::vector<int> log;
    RecordingContext c0{&log, 0};
    RecordingContext c1{&log, 1};

    Task tasks[] = {
        {make_recording_task(c0), "ordinary", 0.0f, 1000, kMaxFastTaskPriorities + 1},
        {make_recording_task(c1), "fast", 0.0f, 1000, 0},
    };
    std::array<std::uint16_t, 2> last_run{0, 0};

    sched.tick();
    sched.run(tasks, {}, last_run, 10000, make_time_source(clock));

    // Both ran: the ordinary task fit the budget check at entry (max_time_micros=1000 <= 10000
    // available at that point), consumed way more than available afterward
    // (time_available_us -> 0), but the FAST task afterward still runs
    // unconditionally.
    REQUIRE(log == std::vector<int>{0, 1});
}
