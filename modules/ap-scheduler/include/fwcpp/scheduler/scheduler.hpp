#pragma once

// Port of AP_Scheduler/AP_Scheduler.h + AP_Scheduler.cpp's task table and
// run() dispatch loop. CPP-026, slice 1.
//
// SCOPE: covers tick()/run() - the actual "which tasks fire this tick,
// in what order, within what time budget" algorithm, which is real,
// deterministic, testable logic independent of any real hardware clock
// or OS thread. Deliberately NOT in this slice: init() (mostly AP_Param/
// AP_Vehicle wiring this port's vehicle skeleton doesn't have yet),
// perf_info/PerfInfo logging, load_average(), the var_info parameter
// table (_debug/_loop_rate_hz/_options as AP_Param-backed GCS-tunable
// values - this port takes loop_rate_hz as a plain constructor argument
// instead, matching the AP_Float-replaced-with-plain-float precedent
// used throughout this port until a caller actually wires AP_Param in),
// and the semaphore/thread-safety machinery (_rsem) - this port has no
// concurrent scheduler access to guard against yet.
//
// TaskFn/TimeSource REPLACE upstream's FUNCTOR_TYPEDEF and repeated
// AP_HAL::micros() calls: a context pointer + plain function pointer,
// not std::function (the standard doesn't guarantee std::function avoids
// heap allocation even for small captures) and not a hidden global clock
// (ADR-0012: no singletons, explicit context instead) - both zero-
// allocation, matching this port's flight-path allocation policy exactly
// like the rest of this codebase's now_ms/now_us parameters, just
// generalized to a callable since run() needs to re-sample time
// AFTER each task executes (a single passed-in timestamp can't do that -
// the whole point is measuring how long each task actually took).
//
// last_run REPLACES upstream's heap-allocated `_last_run = NEW_NOTHROW
// uint16_t[_num_tasks]`: a caller-owned array (typically a std::array
// sized to the task table, held by whoever owns the Scheduler and the
// task tables), persisted across run() calls by the caller instead of
// owned internally - same no-allocation-on-this-port's-actual-path
// principle already applied everywhere else.
//
// FAST TASK SEMANTICS preserved exactly: priority <= kMaxFastTaskPriorities
// (0/1/2, upstream's FAST_TASK_PRI0/1/2) runs UNCONDITONALLY every tick,
// no rate check, and its own budget check is skipped too (only counted
// against remaining time afterward) - matches upstream's `else {
// _task_time_allowed = get_loop_period_us(); }` branch, which never
// reaches the `if (_task_time_allowed > time_available) continue;` skip
// that ordinary tasks are subject to.
//
// PRIORITY MERGE, preserved exactly: vehicle_tasks and common_tasks are
// walked as two separately-sorted lists merged by priority (ties go to
// the vehicle-specific task) - not concatenated then re-sorted. Both
// input spans must already be sorted by priority (matching upstream's
// own init()-time sanity check that priorities never decrease within a
// table - not reproduced here as a runtime assertion, since this port's
// tables are static and reviewable at the point they're written, same
// treatment as other structurally-guaranteed invariants in this port).
//
// LITERAL SAFETY: no bare ambiguous double literals - every quantity
// here is an integer tick/microsecond count.

#include <cstdint>
#include <span>

namespace fwcpp::scheduler {

// Zero-allocation callable: a context pointer plus a plain function
// pointer taking it - see file banner for why not std::function.
struct TaskFn {
    void (*fn)(void*) = nullptr;
    void* ctx = nullptr;
    void operator()() const { fn(ctx); }
};

struct TimeSource {
    std::uint32_t (*now_us)(void*) = nullptr;
    void* ctx = nullptr;
    std::uint32_t operator()() const { return now_us(ctx); }
};

inline constexpr std::uint8_t kMaxFastTaskPriorities = 3; // matches upstream's MAX_FAST_TASK_PRIORITIES sentinel (after FAST_TASK_PRI0/1/2 = 0/1/2)

struct Task {
    TaskFn function;
    const char* name = nullptr;
    float rate_hz = 0.0f;          // 0 means "every loop" (matches upstream's own is_zero(rate_hz) -> interval_ticks=1)
    std::uint16_t max_time_micros = 0;
    std::uint8_t priority = 0;     // <= kMaxFastTaskPriorities means a FAST task (see file banner)
};

class Scheduler {
public:
    explicit Scheduler(std::uint16_t loop_rate_hz)
        : loop_rate_hz_(loop_rate_hz), loop_period_us_(1000000UL / loop_rate_hz) {}

    void tick() {
        ++tick_counter_;
        ++tick_counter32_;
    }

    [[nodiscard]] std::uint16_t ticks() const { return tick_counter_; }
    [[nodiscard]] std::uint32_t ticks32() const { return tick_counter32_; }
    [[nodiscard]] std::uint16_t loop_rate_hz() const { return loop_rate_hz_; }
    [[nodiscard]] std::uint32_t loop_period_us() const { return loop_period_us_; }

    // Runs as many due tasks as fit in time_available_us, merging
    // vehicle_tasks/common_tasks by priority (see file banner). Both
    // spans, and `last_run`, must have already been used together on
    // every prior call - last_run.size() must equal vehicle_tasks.size()
    // + common_tasks.size(), indices assigned in the same merge order
    // this function itself walks (vehicle-tasks-first on a priority tie).
    void run(std::span<const Task> vehicle_tasks, std::span<const Task> common_tasks,
             std::span<std::uint16_t> last_run, std::uint32_t time_available_us, const TimeSource& time_source) {
        std::uint32_t now = time_source();

        std::size_t vehicle_offset = 0;
        std::size_t common_offset = 0;
        const std::size_t num_tasks = vehicle_tasks.size() + common_tasks.size();

        for (std::size_t i = 0; i < num_tasks; ++i) {
            bool run_vehicle_task = false;
            if (vehicle_offset < vehicle_tasks.size() && common_offset < common_tasks.size()) {
                if (vehicle_tasks[vehicle_offset].priority <= common_tasks[common_offset].priority) {
                    run_vehicle_task = true;
                }
            } else if (vehicle_offset < vehicle_tasks.size()) {
                run_vehicle_task = true;
            } else if (common_offset < common_tasks.size()) {
                run_vehicle_task = false;
            } else {
                break; // matches upstream's INTERNAL_ERROR-then-break; both lists exhausted early is a caller bug
            }

            const Task& task = run_vehicle_task ? vehicle_tasks[vehicle_offset] : common_tasks[common_offset];
            if (run_vehicle_task) {
                ++vehicle_offset;
            } else {
                ++common_offset;
            }

            std::uint32_t task_time_allowed;
            if (task.priority > kMaxFastTaskPriorities) {
                const std::uint16_t dt = static_cast<std::uint16_t>(tick_counter_ - last_run[i]);
                std::uint32_t interval_ticks = (task.rate_hz == 0.0f) ? 1U : static_cast<std::uint32_t>(loop_rate_hz_ / task.rate_hz);
                if (interval_ticks < 1) {
                    interval_ticks = 1;
                }
                if (dt < interval_ticks) {
                    continue; // not yet due
                }

                task_time_allowed = task.max_time_micros;

                if (task_time_allowed > time_available_us) {
                    continue; // not enough time left this tick - maybe a later task fits
                }
            } else {
                task_time_allowed = loop_period_us_;
            }

            const std::uint32_t task_started = now;
            task.function();
            last_run[i] = tick_counter_;

            now = time_source();
            const std::uint32_t time_taken = now - task_started;

            if (time_taken >= time_available_us) {
                // Out of time, but keep walking the table - a FAST task
                // later in priority order still needs to run
                // unconditionally, matching upstream exactly.
                time_available_us = 0;
            } else {
                time_available_us -= time_taken;
            }
        }
    }

private:
    std::uint16_t loop_rate_hz_;
    std::uint32_t loop_period_us_;
    std::uint16_t tick_counter_ = 0;
    std::uint32_t tick_counter32_ = 0;
};

} // namespace fwcpp::scheduler
