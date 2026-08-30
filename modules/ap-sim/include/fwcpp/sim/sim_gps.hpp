#pragma once

// Port of libraries/SITL/SIM_GPS.h/.cpp: GPS_Data synthesis (aircraft ->
// sample), delay interpolation, jamming, backend allocation, and the plant
// helpers the CCP-046 harness already consumes. Protocol backends live in
// sim_gps_*.hpp and are wired by sim_gps_factory.hpp (original SIM_GPS.cpp
// switch on Type).

#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sys/time.h>

#include <fwcpp/location.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

inline constexpr std::uint8_t kSimMaxGpsSensors = 4;
inline constexpr std::uint64_t kGpsLeapsecondsMillis = 18000ULL;
inline constexpr std::uint32_t kApSecPerWeek = 7U * 86400U;
inline constexpr std::uint32_t kApMsecPerSec = 1000U;

struct GPS_Data {
    std::uint32_t timestamp_ms{0};
    double latitude{0};
    double longitude{0};
    float altitude{0};
    double speedN{0};
    double speedE{0};
    double speedD{0};
    double yaw_deg{0};
    double roll_deg{0};
    double pitch_deg{0};
    bool have_lock{true};
    float horizontal_acc{0.3f};
    float vertical_acc{0.5f};
    float speed_acc{0.3f};
    std::uint8_t num_sats{10};
    std::uint8_t fix_type{3};

    [[nodiscard]] float ground_track_rad() const {
        return std::atan2(static_cast<float>(speedE), static_cast<float>(speedN));
    }
    [[nodiscard]] float speed_2d() const {
        return std::sqrt(static_cast<float>(speedN * speedN + speedE * speedE));
    }
};

inline GPS_Data gps_data_from_aircraft(const Aircraft& aircraft) {
    GPS_Data d;
    d.latitude = aircraft.location.lat * 1.0e-7;
    d.longitude = aircraft.location.lng * 1.0e-7;
    d.altitude = aircraft.location.alt * 0.01f;
    d.speedN = aircraft.velocity_ef.x;
    d.speedE = aircraft.velocity_ef.y;
    d.speedD = aircraft.velocity_ef.z;
    d.have_lock = true;
    d.num_sats = 10;
    d.fix_type = 3;
    return d;
}

struct SitlGpsSample {
    std::int32_t lat{0};
    std::int32_t lng{0};
    std::int32_t alt_cm{0};
    math::Vector3f velocity_ef{};
    bool have_lock{true};
    std::uint8_t num_sats{10};
};

inline SitlGpsSample sitl_gps_from_aircraft(const Aircraft& aircraft) {
    SitlGpsSample s;
    s.lat = aircraft.location.lat;
    s.lng = aircraft.location.lng;
    s.alt_cm = aircraft.location.alt;
    s.velocity_ef = aircraft.velocity_ef;
    s.have_lock = true;
    s.num_sats = 10;
    return s;
}

enum class GpsType : std::uint8_t {
    NONE = 0,
    UBLOX = 1,
    NMEA = 5,
    SBP = 6,
    FILE = 7,
    NOVA = 8,
    SBP2 = 9,
    SBF = 10,
    TRIMBLE = 11,
    MSP = 19,
};

enum class GpsHeading : std::uint8_t {
    NONE = 0,
    HDT = 1,
    THS = 2,
    KSXT = 3,
    BASE = 4,
};

enum class GpsOptions : std::int32_t {
    UBX_IS_F9P = 1,
};

struct GpsParms {
    bool enabled{true};
    std::uint16_t delay_ms{100};
    GpsType type{GpsType::UBLOX};
    float byteloss{0};
    std::uint8_t numsats{10};
    math::Vector3f glitch{};
    std::uint8_t hertz{5};
    float drift_alt{0};
    math::Vector3f pos_offset{};
    float noise{0};
    float lock_time{0};
    float alt_offset{0};
    GpsHeading hdg_enabled{GpsHeading::NONE};
    float accuracy{0.3f};
    math::Vector3f vel_err{};
    std::uint8_t jam{0};
    float heading_offset{0};
    std::int32_t options{0};
};

struct GpsWorldState {
    double latitude{0};
    double longitude{0};
    float altitude{0};
    double speedN{0};
    double speedE{0};
    double speedD{0};
    float yaw_deg{0};
    float roll_deg{0};
    float pitch_deg{0};
    float roll_rate{0};
    float pitch_rate{0};
    float yaw_rate{0};
    math::Quaternion quaternion{};
    double gps_init_lat_ofs{0};
    double gps_init_lon_ofs{0};
    float gps_init_alt_ofs{0};
    std::int64_t start_time_UTC{0};
    std::uint16_t gps_log_num{1};
};

inline GpsWorldState gps_world_from_aircraft(const Aircraft& aircraft) {
    GpsWorldState w;
    w.latitude = aircraft.location.lat * 1.0e-7;
    w.longitude = aircraft.location.lng * 1.0e-7;
    w.altitude = aircraft.location.alt * 0.01f;
    w.speedN = aircraft.velocity_ef.x;
    w.speedE = aircraft.velocity_ef.y;
    w.speedD = aircraft.velocity_ef.z;
    float r = 0, p = 0, y = 0;
    aircraft.dcm.to_euler(&r, &p, &y);
    w.roll_deg = math::degrees(r);
    w.pitch_deg = math::degrees(p);
    w.yaw_deg = math::degrees(y);
    w.roll_rate = math::degrees(aircraft.gyro.x);
    w.pitch_rate = math::degrees(aircraft.gyro.y);
    w.yaw_rate = math::degrees(aircraft.gyro.z);
    w.quaternion.from_rotation_matrix(aircraft.dcm);
    return w;
}

class GPS;

class GPS_Backend {
public:
    GPS_Backend(GPS& front, std::uint8_t instance);
    virtual ~GPS_Backend() = default;
    GPS_Backend(const GPS_Backend&) = delete;
    GPS_Backend& operator=(const GPS_Backend&) = delete;

    [[nodiscard]] virtual std::uint32_t device_baud() const { return 0; }
    ssize_t write_to_autopilot(const char* p, std::size_t size) const;
    ssize_t read_from_autopilot(char* buffer, std::size_t size) const;
    virtual void update_read();
    virtual void publish(const GPS_Data* d) = 0;

    struct GPS_TOW {
        std::uint16_t week{0};
        std::uint32_t ms{0};
    };
    static GPS_TOW gps_time();
    static void simulation_timeval(struct timeval* tv);
    static void set_sim_clock(std::uint64_t time_us, std::int64_t start_time_UTC);

protected:
    std::uint8_t instance;
    GPS& front;

    static std::uint64_t sim_time_us_;
    static std::int64_t sim_start_time_UTC_;
    static std::uint64_t first_usec_;
    static struct timeval first_tv_;
};

class GPS : public SerialDevice {
public:
    using Type = GpsType;

    explicit GPS(std::uint8_t instance);
    GPS(const GPS&) = delete;
    GPS& operator=(const GPS&) = delete;

    void update(const GpsWorldState& world, GpsParms parms[kSimMaxGpsSensors], std::uint32_t now_ms,
                std::uint64_t time_us);
    void publish_sample(const GPS_Data& d);

    ssize_t write_to_autopilot(const char* p, std::size_t size) override;
    [[nodiscard]] std::uint32_t device_baud() const override;

    [[nodiscard]] std::uint8_t instance_index() const { return instance; }
    [[nodiscard]] GpsParms& parms() { return parms_[instance]; }
    [[nodiscard]] const GpsParms& parms() const { return parms_[instance]; }
    [[nodiscard]] GpsParms* all_parms() { return parms_; }
    [[nodiscard]] const GpsWorldState& world() const { return world_; }
    [[nodiscard]] std::uint32_t now_ms() const { return now_ms_; }
    [[nodiscard]] GPS_Backend* backend() { return backend_.get(); }
    [[nodiscard]] Type allocated_type() const { return allocated_type_; }

    void set_backend(std::unique_ptr<GPS_Backend> b, Type t) {
        backend_ = std::move(b);
        allocated_type_ = t;
    }

    void check_backend_allocation();

    using BackendFactory = std::unique_ptr<GPS_Backend> (*)(GPS& gps, std::uint8_t instance, GpsType type);
    static void set_backend_factory(BackendFactory f) { factory_ = f; }

private:
    std::uint8_t instance;
    std::uint32_t last_write_update_ms{0};
    GPS_Data gps_history_[20]{};
    struct {
        std::uint32_t last_jam_ms{0};
        std::uint32_t jam_start_ms{0};
        std::uint32_t last_sats_change_ms{0};
        std::uint32_t last_vz_change_ms{0};
        std::uint32_t last_vel_change_ms{0};
        std::uint32_t last_pos_change_ms{0};
        std::uint32_t last_acc_change_ms{0};
        double latitude{0};
        double longitude{0};
    } jamming_[kSimMaxGpsSensors]{};

    bool gps_has_basestation_position_{false};
    GPS_Data gps_basestation_data_{};
    GpsType allocated_type_{GpsType::NONE};
    std::unique_ptr<GPS_Backend> backend_{};
    GpsParms parms_[kSimMaxGpsSensors]{};
    GpsWorldState world_{};
    std::uint32_t now_ms_{0};
    static BackendFactory factory_;

    void simulate_jamming(GPS_Data& d);
    GPS_Data interpolate_data(const GPS_Data& d, std::uint32_t delay_ms);
};

inline std::uint64_t GPS_Backend::sim_time_us_ = 0;
inline std::int64_t GPS_Backend::sim_start_time_UTC_ = 0;
inline std::uint64_t GPS_Backend::first_usec_ = 0;
inline struct timeval GPS_Backend::first_tv_ = {};
inline GPS::BackendFactory GPS::factory_ = nullptr;

inline GPS_Backend::GPS_Backend(GPS& front_ref, std::uint8_t inst) : instance{inst}, front{front_ref} {}

inline ssize_t GPS_Backend::write_to_autopilot(const char* p, std::size_t size) const {
    return front.write_to_autopilot(p, size);
}

inline ssize_t GPS_Backend::read_from_autopilot(char* buffer, std::size_t size) const {
    return front.read_from_autopilot(buffer, size);
}

inline void GPS_Backend::update_read() {
    char c;
    read_from_autopilot(&c, 1);
}

inline void GPS_Backend::set_sim_clock(std::uint64_t time_us, std::int64_t start_time_UTC) {
    sim_time_us_ = time_us;
    sim_start_time_UTC_ = start_time_UTC;
}

inline void GPS_Backend::simulation_timeval(struct timeval* tv) {
    const std::uint64_t now = sim_time_us_;
    if (first_usec_ == 0) {
        first_usec_ = now;
        first_tv_.tv_sec = static_cast<time_t>(sim_start_time_UTC_);
        first_tv_.tv_usec = 0;
    }
    *tv = first_tv_;
    tv->tv_sec += static_cast<time_t>(now / 1000000ULL);
    const std::uint64_t new_usec = static_cast<std::uint64_t>(tv->tv_usec) + (now % 1000000ULL);
    tv->tv_sec += static_cast<time_t>(new_usec / 1000000ULL);
    tv->tv_usec = static_cast<suseconds_t>(new_usec % 1000000ULL);
}

inline GPS_Backend::GPS_TOW GPS_Backend::gps_time() {
    GPS_TOW gps_tow;
    struct timeval tv {};
    simulation_timeval(&tv);
    const std::uint32_t epoch =
        86400U * (10U * 365U + (1980U - 1969U) / 4U + 1U + 6U - 2U) - static_cast<std::uint32_t>(kGpsLeapsecondsMillis / 1000ULL);
    const std::uint32_t epoch_seconds = static_cast<std::uint32_t>(tv.tv_sec) - epoch;
    gps_tow.week = static_cast<std::uint16_t>(epoch_seconds / kApSecPerWeek);
    const std::uint32_t t_ms = static_cast<std::uint32_t>(tv.tv_usec / 1000);
    gps_tow.ms = (epoch_seconds % kApSecPerWeek) * kApMsecPerSec + ((t_ms / 200U) * 200U);
    return gps_tow;
}

inline GPS::GPS(std::uint8_t inst) : SerialDevice(8192, 2048), instance{inst} {}

inline std::uint32_t GPS::device_baud() const {
    if (backend_ == nullptr) {
        return 0;
    }
    return backend_->device_baud();
}

inline ssize_t GPS::write_to_autopilot(const char* p, std::size_t size) {
    if (instance == 1 && !parms_[instance].enabled) {
        return -1;
    }
    const float byteloss = parms_[instance].byteloss;
    if (!(byteloss > 0.0f)) {
        return SerialDevice::write_to_autopilot(p, size);
    }
    std::size_t ret = 0;
    while (size--) {
        const float r = static_cast<float>((((unsigned)std::rand()) % 1000000)) / 1.0e4f;
        if (r < byteloss) {
            p++;
            continue;
        }
        const ssize_t pret = SerialDevice::write_to_autopilot(p, 1);
        if (pret == 0) {
            return static_cast<ssize_t>(ret);
        }
        if (pret != 1) {
            return pret;
        }
        ret++;
        p++;
    }
    return static_cast<ssize_t>(ret);
}

inline void GPS::simulate_jamming(GPS_Data& d) {
    auto& jam = jamming_[instance];
    const std::uint32_t now_ms = now_ms_;
    if (now_ms - jam.last_jam_ms > 1000) {
        jam.jam_start_ms = now_ms;
        jam.latitude = d.latitude;
        jam.longitude = d.longitude;
    }
    jam.last_jam_ms = now_ms;

    const float vz_change_hz = 0.5f;
    const float vel_change_hz = 0.8f;
    const float pos_change_hz = 1.1f;
    const float sats_change_hz = 3.0f;
    const float acc_change_hz = 3.0f;
    auto rand_float = []() -> float {
        return static_cast<float>((((unsigned)std::rand()) % 2000000) - 1.0e6) / 1.0e6f;
    };
    auto get_random16 = []() -> std::uint16_t { return static_cast<std::uint16_t>(std::rand() & 0xFFFF); };

    if (now_ms - jam.jam_start_ms < static_cast<unsigned>(1000U + (get_random16() % 5000))) {
        d.num_sats = 0;
        d.have_lock = false;
    } else {
        if ((now_ms - jam.last_sats_change_ms) * 0.001f > 2.0f * std::fabs(rand_float()) / sats_change_hz) {
            jam.last_sats_change_ms = now_ms;
            d.num_sats = static_cast<std::uint8_t>(2 + (get_random16() % 15));
            if (d.num_sats >= 4) {
                d.have_lock = (get_random16() % 2) != 0;
            } else {
                d.have_lock = false;
            }
        }
        if ((now_ms - jam.last_vz_change_ms) * 0.001f > 2.0f * std::fabs(rand_float()) / vz_change_hz) {
            jam.last_vz_change_ms = now_ms;
            d.speedD = rand_float() * 400.0f;
        }
        if ((now_ms - jam.last_vel_change_ms) * 0.001f > 2.0f * std::fabs(rand_float()) / vel_change_hz) {
            jam.last_vel_change_ms = now_ms;
            d.speedN = rand_float() * 400.0f;
            d.speedE = rand_float() * 400.0f;
        }
        if ((now_ms - jam.last_pos_change_ms) * 0.001f > 2.0f * std::fabs(rand_float()) / pos_change_hz) {
            jam.last_pos_change_ms = now_ms;
            jam.latitude += rand_float() * 200.0f * (1.0f / 0.011131884502145512f) * 1.0e-7f;
            jam.longitude += rand_float() * 200.0f * (1.0f / 0.011131884502145512f) * 1.0e-7f;
        }
        if ((now_ms - jam.last_acc_change_ms) * 0.001f > 2.0f * std::fabs(rand_float()) / acc_change_hz) {
            jam.last_acc_change_ms = now_ms;
            d.vertical_acc = std::fabs(rand_float()) * 300.0f;
            d.horizontal_acc = std::fabs(rand_float()) * 300.0f;
            d.speed_acc = std::fabs(rand_float()) * 50.0f;
        }
    }
    d.latitude = math::constrain_value(jam.latitude, -90.0, 90.0);
    d.longitude = math::constrain_value(jam.longitude, -180.0, 180.0);
}

inline GPS_Data GPS::interpolate_data(const GPS_Data& d, std::uint32_t delay_ms) {
    const std::uint8_t N = static_cast<std::uint8_t>(sizeof(gps_history_) / sizeof(gps_history_[0]));
    const std::uint32_t now_ms = d.timestamp_ms;
    std::memmove(&gps_history_[1], &gps_history_[0], sizeof(gps_history_[0]) * (N - 1));
    gps_history_[0] = d;
    for (std::uint8_t i = 0; i < N - 1; i++) {
        const std::uint32_t dt1 = now_ms - gps_history_[i].timestamp_ms;
        const std::uint32_t dt2 = now_ms - gps_history_[i + 1].timestamp_ms;
        if (delay_ms >= dt1 && delay_ms <= dt2) {
            const GPS_Data& s1 = gps_history_[i + 1];
            const GPS_Data& s2 = gps_history_[i];
            GPS_Data d2 = s1;
            const float p = (dt2 - delay_ms) / std::fmax(1.0f, static_cast<float>(dt2 - dt1));
            d2.latitude += p * (s2.latitude - s1.latitude);
            d2.longitude += p * (s2.longitude - s1.longitude);
            d2.altitude += p * (s2.altitude - s1.altitude);
            d2.speedN += p * (s2.speedN - s1.speedN);
            d2.speedE += p * (s2.speedE - s1.speedE);
            d2.speedD += p * (s2.speedD - s1.speedD);
            d2.yaw_deg += p * math::wrap_180(s2.yaw_deg - s1.yaw_deg);
            return d2;
        }
    }
    return gps_history_[N - 1];
}

inline void GPS::check_backend_allocation() {
    const GpsType configured = parms_[instance].type;
    if (allocated_type_ == configured) {
        return;
    }
    backend_.reset();
    allocated_type_ = GpsType::NONE;
    if (configured == GpsType::NONE) {
        return;
    }
    if (factory_ != nullptr) {
        backend_ = factory_(*this, instance, configured);
        if (backend_) {
            allocated_type_ = configured;
        }
    }
}

inline void GPS::publish_sample(const GPS_Data& d) {
    check_backend_allocation();
    if (backend_ == nullptr) {
        return;
    }
    backend_->publish(&d);
}

inline void GPS::update(const GpsWorldState& world, GpsParms parms[kSimMaxGpsSensors], std::uint32_t now_ms,
                        std::uint64_t time_us) {
    world_ = world;
    now_ms_ = now_ms;
    for (std::uint8_t i = 0; i < kSimMaxGpsSensors; i++) {
        parms_[i] = parms[i];
    }
    GPS_Backend::set_sim_clock(time_us, world.start_time_UTC);

    check_backend_allocation();
    if (backend_ == nullptr) {
        return;
    }

    double latitude = world.latitude;
    double longitude = world.longitude;
    float altitude = world.altitude;
    const double speedN = world.speedN;
    const double speedE = world.speedE;
    const double speedD = world.speedD;

    if (now_ms < 20000) {
        latitude += world.gps_init_lat_ofs;
        longitude += world.gps_init_lon_ofs;
        altitude += world.gps_init_alt_ofs;
    }

    if (!gps_has_basestation_position_ && now_ms >= static_cast<std::uint32_t>(parms_[0].lock_time * 1000.0f)) {
        gps_basestation_data_.latitude = latitude;
        gps_basestation_data_.longitude = longitude;
        gps_basestation_data_.altitude = altitude;
        gps_basestation_data_.speedN = speedN;
        gps_basestation_data_.speedE = speedE;
        gps_basestation_data_.speedD = speedD;
        gps_has_basestation_position_ = true;
    }

    const auto& params = parms_[instance];
    if ((now_ms - last_write_update_ms) < static_cast<std::uint32_t>(1000 / std::max<std::uint8_t>(params.hertz, 1))) {
        backend_->update_read();
        return;
    }
    last_write_update_ms = now_ms;

    GPS_Data d {};
    d.num_sats = params.numsats;
    d.latitude = latitude;
    d.longitude = longitude;
    d.yaw_deg = math::wrap_360(world.yaw_deg + params.heading_offset);
    d.roll_deg = world.roll_deg;
    d.pitch_deg = world.pitch_deg;
    d.altitude = altitude + params.noise * std::sin(now_ms * 0.0005f) + params.alt_offset;

    auto rand_float = []() -> float {
        return static_cast<float>((((unsigned)std::rand()) % 2000000) - 1.0e6) / 1.0e6f;
    };
    d.speedN = speedN + (params.vel_err.x * rand_float());
    d.speedE = speedE + (params.vel_err.y * rand_float());
    d.speedD = speedD + (params.vel_err.z * rand_float());
    d.have_lock = params.enabled && now_ms >= static_cast<std::uint32_t>(params.lock_time * 1000.0f);
    d.horizontal_acc = params.accuracy;
    d.vertical_acc = params.accuracy;
    d.speed_acc = params.vel_err.xy().length();
    if (params.drift_alt > 0) {
        d.altitude += params.drift_alt * std::sin(now_ms * 0.001f * 0.02f);
    }

    math::Vector3f posRelOffsetBF = params.pos_offset;
    if (!posRelOffsetBF.is_zero()) {
        math::Matrix3f rotmat;
        world.quaternion.rotation_matrix(rotmat);
        math::Vector3f posRelOffsetEF = rotmat * posRelOffsetBF;
        const double earth_rad_inv = 1.569612305760477e-7;
        const double lng_scale_factor = earth_rad_inv / std::cos(math::radians(d.latitude));
        d.latitude += math::degrees(posRelOffsetEF.x * earth_rad_inv);
        d.longitude += math::degrees(posRelOffsetEF.y * lng_scale_factor);
        d.altitude -= posRelOffsetEF.z;
        math::Vector3f gyro(math::radians(world.roll_rate), math::radians(world.pitch_rate),
                            math::radians(world.yaw_rate));
        math::Vector3f velRelOffsetBF = gyro % posRelOffsetBF;
        math::Vector3f velRelOffsetEF = rotmat * velRelOffsetBF;
        d.speedN += velRelOffsetEF.x;
        d.speedE += velRelOffsetEF.y;
        d.speedD += velRelOffsetEF.z;
    }

    d.timestamp_ms = now_ms;
    d = interpolate_data(d, params.delay_ms);
    d.latitude += params.glitch.x;
    d.longitude += params.glitch.y;
    d.altitude += params.glitch.z;
    if (params.jam == 1) {
        simulate_jamming(d);
    }
    backend_->publish(&d);
}

}  // namespace fwcpp::sim
