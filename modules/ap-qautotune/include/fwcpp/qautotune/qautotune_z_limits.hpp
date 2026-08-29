#pragma once

namespace fwcpp::qautotune {

struct QAutotuneZLimitInputs {
    float velocity_max_dn_m{0.0f};
    float velocity_max_up_ms{0.0f};
    float accel_z_mss{0.0f};
};

struct QAutotuneZLimitSetpoints {
    float max_speed_dn_m{0.0f};
    float max_speed_up_ms{0.0f};
    float accel_mss{0.0f};
};

[[nodiscard]] inline QAutotuneZLimitSetpoints resolve_z_limits(const QAutotuneZLimitInputs& in) {
    QAutotuneZLimitSetpoints out{};
    out.max_speed_dn_m = in.velocity_max_dn_m;
    out.max_speed_up_ms = in.velocity_max_up_ms;
    out.accel_mss = in.accel_z_mss;
    return out;
}

}  // namespace fwcpp::qautotune
