#pragma once

#include <cstddef>

namespace fwcpp::qautotune {

// Upstream QAutoTune::log_pids() (qautotune.cpp:54-61, HAL_LOGGING_ENABLED)
// is three Write_PID calls. This ports the named identities and pid_info
// sources only — no logger subsystem. Off is a no-op.

inline constexpr const char* kLogPiqrIdentity = "PIQR";
inline constexpr const char* kLogPiqpIdentity = "PIQP";
inline constexpr const char* kLogPiqyIdentity = "PIQY";

inline constexpr const char* kRateRollPidInfo = "rate_roll";
inline constexpr const char* kRatePitchPidInfo = "rate_pitch";
inline constexpr const char* kRateYawPidInfo = "rate_yaw";

struct QAutotuneLogPidSpec {
    const char* identity;
    const char* pid_info_source;
};

inline constexpr QAutotuneLogPidSpec kQAutotuneLogPidHooks[] = {
    {kLogPiqrIdentity, kRateRollPidInfo},
    {kLogPiqpIdentity, kRatePitchPidInfo},
    {kLogPiqyIdentity, kRateYawPidInfo},
};

[[nodiscard]] inline constexpr std::size_t qautotune_log_pid_hook_count() {
    return sizeof(kQAutotuneLogPidHooks) / sizeof(kQAutotuneLogPidHooks[0]);
}

struct QAutotunePidInfo {
    float p{0.0f};
    float i{0.0f};
    float d{0.0f};
    float ff{0.0f};
};

struct QAutotuneRatePidSnapshot {
    QAutotunePidInfo rate_roll{};
    QAutotunePidInfo rate_pitch{};
    QAutotunePidInfo rate_yaw{};
};

struct QAutotuneLogPidChannel {
    bool write{false};
    const char* identity{""};
    const char* pid_info_source{""};
    QAutotunePidInfo pid{};
};

struct QAutotuneLogPidsResult {
    QAutotuneLogPidChannel piqr{};
    QAutotuneLogPidChannel piqp{};
    QAutotuneLogPidChannel piqy{};
};

[[nodiscard]] inline QAutotuneLogPidsResult resolve_qautotune_log_pids(bool hal_logging_enabled,
                                                                         const QAutotuneRatePidSnapshot& pids) {
    QAutotuneLogPidsResult out{};
    if (!hal_logging_enabled) {
        return out;
    }
    out.piqr.write = true;
    out.piqr.identity = kLogPiqrIdentity;
    out.piqr.pid_info_source = kRateRollPidInfo;
    out.piqr.pid = pids.rate_roll;
    out.piqp.write = true;
    out.piqp.identity = kLogPiqpIdentity;
    out.piqp.pid_info_source = kRatePitchPidInfo;
    out.piqp.pid = pids.rate_pitch;
    out.piqy.write = true;
    out.piqy.identity = kLogPiqyIdentity;
    out.piqy.pid_info_source = kRateYawPidInfo;
    out.piqy.pid = pids.rate_yaw;
    return out;
}

}  // namespace fwcpp::qautotune
