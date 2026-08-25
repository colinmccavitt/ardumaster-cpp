#pragma once

// Port of AP_Common/Location.h + Location.cpp. CPP-011, slice 1.
//
// SLICE BOUNDARY: struct (bitfield flags, lat/lng/alt, AltFrame), zero(),
// the (lat, lng, alt, frame) constructor, set_alt_cm/set_alt_m,
// get_alt_frame, longitude_scale/diff_longitude/wrap_longitude/
// limit_lattitude, offset_latlng/offset/offset_bearing/
// offset_bearing_and_pitch, get_distance, get_distance_NE (Vector2f only),
// get_distance_NED (Vector3f only).
//
// Deliberately NOT in this slice: get_alt_cm/get_alt_m/change_alt_frame
// (need home-position/EKF-origin/terrain-database context this port hasn't
// built yet - ADR-0012 decision 6's explicit-context-struct is the right
// answer once there IS a context to pass, not before), get_vector_from_
// origin_* (same reason - needs EKF origin), the Vector3p/Vector2p/double
// variants of get_distance_* (Vector3p/Vector2p don't exist in this port
// yet - a reduced-precision "postype" used for logging, out of scope until
// something needs it), sanitize, the Vector3f/Vector3d ekf_offset
// constructors (same origin-context dependency), and everything past
// line ~530 of Location.cpp (great-circle/line-intersection helpers,
// closest-point-on-line-between-two-locations, and more). Tracked in
// CPP-011's notes.
//
// LITERAL SAFETY: LOCATION_SCALING_FACTOR/_INV are upstream `#define
// LATLON_TO_M 0.011131884502145034` etc, narrowed to an explicitly-typed
// `constexpr float` by upstream's own declaration - the flag can't change
// the result since converting a double-parsed-as-float-anyway literal to
// float is a no-op either way (same reasoning already used for M_PI in
// scalar.cpp). longitude_scale's `1.0e-7 * DEG_TO_RAD` grouping IS
// reproduced exactly (not decomposed into calling this port's radians()
// helper) because floating point isn't associative and upstream computes
// this specific product before multiplying by lat - see longitude_scale's
// body below.
//
// IMPLEMENTATION NOTE, not a behavior change: zero() sets every member
// explicitly instead of upstream's memset(this, 0, sizeof(*this)) - same
// ADR-0012 no-unsafe-reinterpretation stance already applied to
// Matrix3::zero() and Vector3::zero().
//
// SKIPPED: get_alt_frame()'s SITL-only `AP_HAL::panic(...)` sanity check
// (terrain_alt set without relative_alt). ADR-0012 decision 2 forbids
// panics as control flow in production code; this was a debug-build
// assertion, not behavior, so it has no port-side equivalent to reproduce -
// noted rather than silently dropped.

#include <cstdint>
#include <cstring>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp {

class Location {
public:
    // Bitfields, matching upstream's storage layout intent (though this
    // port makes no byte-compatibility claim the way the Rust port's
    // AP_Param storage does - ADR-0010 was a deliberate, separate decision
    // for that one format, not a blanket policy).
    std::uint8_t relative_alt : 1 = 0;   // altitude is relative to home
    std::uint8_t loiter_ccw : 1 = 0;     // 0 clockwise, 1 counter-clockwise
    std::uint8_t terrain_alt : 1 = 0;    // altitude is above terrain
    std::uint8_t origin_alt : 1 = 0;     // altitude is above EKF origin
    std::uint8_t loiter_xtrack : 1 = 0;  // 0 crosstrack from center, 1 from tangent exit

    std::int32_t alt = 0; // centimeters
    std::int32_t lat = 0; // 1e7 degrees
    std::int32_t lng = 0; // 1e7 degrees

    enum class AltFrame {
        ABSOLUTE = 0,
        ABOVE_HOME = 1,
        ABOVE_ORIGIN = 2,
        ABOVE_TERRAIN = 3,
    };

    Location() = default;
    Location(std::int32_t latitude, std::int32_t longitude, std::int32_t alt_cm, AltFrame frame) {
        zero();
        lat = latitude;
        lng = longitude;
        set_alt_cm(alt_cm, frame);
    }

    void zero() {
        relative_alt = loiter_ccw = terrain_alt = origin_alt = loiter_xtrack = 0;
        alt = lat = lng = 0;
    }

    void set_alt_cm(std::int32_t alt_cm, AltFrame frame) {
        alt = alt_cm;
        relative_alt = false;
        terrain_alt = false;
        origin_alt = false;
        switch (frame) {
            case AltFrame::ABSOLUTE:
                break;
            case AltFrame::ABOVE_HOME:
                relative_alt = true;
                break;
            case AltFrame::ABOVE_ORIGIN:
                origin_alt = true;
                break;
            case AltFrame::ABOVE_TERRAIN:
                // Marked relative, matching upstream - it has no altitude
                // of its own, it rides on relative_alt's flag.
                relative_alt = true;
                terrain_alt = true;
                break;
        }
    }
    void set_alt_m(float alt_m, AltFrame frame) {
        set_alt_cm(static_cast<std::int32_t>(alt_m * 100), frame);
    }

    // See file banner: the SITL-only panic on an inconsistent
    // terrain_alt/relative_alt combination is not reproduced.
    [[nodiscard]] AltFrame get_alt_frame() const {
        if (terrain_alt) {
            return AltFrame::ABOVE_TERRAIN;
        }
        if (origin_alt) {
            return AltFrame::ABOVE_ORIGIN;
        }
        if (relative_alt) {
            return AltFrame::ABOVE_HOME;
        }
        return AltFrame::ABSOLUTE;
    }

    static constexpr float LOCATION_SCALING_FACTOR = 0.011131884502145034f;
    static constexpr float LOCATION_SCALING_FACTOR_INV = 89.83204953368922f;

    // Scale factor to convert a longitude delta (in the same 1e7-degree
    // units as `lng`) into a true east-west distance, compensating for
    // meridian convergence at this latitude. Clamped to 0.01 so a location
    // exactly at a pole doesn't collapse the scale to zero.
    //
    // The `lat * (1.0e-7f * deg_to_rad_constant())` grouping is reproduced
    // exactly as upstream computes it (the literal-product first, THEN
    // multiplied by lat) rather than expressed via this port's own
    // radians() helper - floating point multiplication isn't associative,
    // so `lat * (a * b)` and `(lat * a) * b` are not guaranteed to be the
    // same bit pattern, and upstream picked the former.
    [[nodiscard]] static float longitude_scale(std::int32_t lat) {
        const float scale = std::cos(static_cast<float>(lat) * (1.0e-7f * math::deg_to_rad_constant()));
        return scale > 0.01f ? scale : 0.01f;
    }

    // lon1 - lon2, wrapping at +-180e7.
    [[nodiscard]] static std::int32_t diff_longitude(std::int32_t lon1, std::int32_t lon2) {
        if ((static_cast<std::uint32_t>(lon1) & 0x80000000U) == (static_cast<std::uint32_t>(lon2) & 0x80000000U)) {
            return lon1 - lon2; // common case: same sign, no overflow risk
        }
        std::int64_t dlon = static_cast<std::int64_t>(lon1) - static_cast<std::int64_t>(lon2);
        if (dlon > 1800000000LL) {
            dlon -= 3600000000LL;
        } else if (dlon < -1800000000LL) {
            dlon += 3600000000LL;
        }
        return static_cast<std::int32_t>(dlon);
    }

    [[nodiscard]] static std::int32_t wrap_longitude(std::int64_t lon) {
        if (lon > 1800000000LL) {
            lon = static_cast<std::int32_t>(lon - 3600000000LL);
        } else if (lon < -1800000000LL) {
            lon = static_cast<std::int32_t>(lon + 3600000000LL);
        }
        return static_cast<std::int32_t>(lon);
    }

    [[nodiscard]] static std::int32_t limit_lattitude(std::int32_t lat) {
        if (lat > 900000000L) {
            return static_cast<std::int32_t>(1800000000LL - lat);
        }
        if (lat < -900000000L) {
            return static_cast<std::int32_t>(-(1800000000LL + lat));
        }
        return lat;
    }

    // Mutates lat/lng in place - matches upstream's free (static) helper,
    // which both offset() and the Vector3p-offset overload (not yet ported)
    // funnel through.
    static void offset_latlng(std::int32_t& lat_ref, std::int32_t& lng_ref, float ofs_north, float ofs_east) {
        const std::int32_t dlat = static_cast<std::int32_t>(ofs_north * LOCATION_SCALING_FACTOR_INV);
        const std::int64_t dlng = static_cast<std::int64_t>(
            (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat_ref + dlat / 2));
        lat_ref += dlat;
        lat_ref = limit_lattitude(lat_ref);
        lng_ref = wrap_longitude(dlng + lng_ref);
    }

    // Extrapolate this location's lat/lng by distances (meters) north and
    // east, in place.
    void offset(float ofs_north, float ofs_east) {
        offset_latlng(lat, lng, ofs_north, ofs_east);
    }

    // Accurate to about 1mm at 100m, per upstream's own comment - it works
    // in relative offsets rather than absolute trig, which keeps accuracy
    // for small distances.
    void offset_bearing(float bearing_deg, float distance) {
        const float ofs_north = std::cos(math::radians(bearing_deg)) * distance;
        const float ofs_east = std::sin(math::radians(bearing_deg)) * distance;
        offset(ofs_north, ofs_east);
    }

    void offset_bearing_and_pitch(float bearing_deg, float pitch_deg, float distance) {
        const float cp = std::cos(math::radians(pitch_deg));
        const float ofs_north = cp * std::cos(math::radians(bearing_deg)) * distance;
        const float ofs_east = cp * std::sin(math::radians(bearing_deg)) * distance;
        offset(ofs_north, ofs_east);
        const std::int32_t dalt = static_cast<std::int32_t>(std::sin(math::radians(pitch_deg)) * distance * 100.0f);
        alt += dalt;
    }

    // Horizontal distance in meters (flat-earth, latitude-compensated
    // approximation - not great-circle).
    [[nodiscard]] float get_distance(const Location& loc2) const {
        const float dlat = static_cast<float>(loc2.lat - lat);
        const float dlng = static_cast<float>(diff_longitude(loc2.lng, lng)) * longitude_scale((lat + loc2.lat) / 2);
        return std::sqrt(dlat * dlat + dlng * dlng) * LOCATION_SCALING_FACTOR;
    }

    // North/East distance to loc2, in meters, as a Vector2f. NOT altitude-
    // frame-aware - a pure horizontal-position difference.
    [[nodiscard]] math::Vector2f get_distance_NE(const Location& loc2) const {
        return math::Vector2f(
            static_cast<float>(loc2.lat - lat) * LOCATION_SCALING_FACTOR,
            static_cast<float>(diff_longitude(loc2.lng, lng)) * LOCATION_SCALING_FACTOR * longitude_scale((loc2.lat + lat) / 2));
    }

    // North/East/Down distance to loc2, in meters, as a Vector3f. NOT
    // altitude-frame-aware (the two Locations' alt fields are compared
    // directly regardless of frame) - matches upstream's own documented
    // caveat on this specific overload.
    [[nodiscard]] math::Vector3f get_distance_NED(const Location& loc2) const {
        return math::Vector3f(
            static_cast<float>(loc2.lat - lat) * LOCATION_SCALING_FACTOR,
            static_cast<float>(diff_longitude(loc2.lng, lng)) * LOCATION_SCALING_FACTOR * longitude_scale((lat + loc2.lat) / 2),
            static_cast<float>(alt - loc2.alt) * 0.01f); // cm -> m; verified against upstream: (alt - loc2.alt) * 0.01
    }

    // Bearing to loc2, radians, 0 to 2*pi. Defined in location.cpp - bare
    // M_PI literals (matches this port's established pattern: scalar.cpp's
    // wrap_* family, vector2.cpp's angle()).
    [[nodiscard]] float get_bearing(const Location& loc2) const;

    // Bearing to loc2, centidegrees, 0 to 35999.
    [[nodiscard]] std::int32_t get_bearing_to(const Location& loc2) const {
        return static_cast<std::int32_t>(math::rad_to_cd(get_bearing(loc2)) + 0.5f);
    }

    [[nodiscard]] bool same_latlon_as(const Location& loc2) const {
        return lat == loc2.lat && lng == loc2.lng;
    }

    // Upstream's slow path (different alt frames: converts both to a
    // common frame via get_height_above(), which needs home/EKF-origin
    // context this port hasn't built - see file banner) is NOT
    // reproduced. Only the fast path (same frame: direct alt comparison)
    // is - matching upstream exactly for same-frame callers, and
    // returning false for cross-frame callers rather than fabricating an
    // answer this port doesn't have the context to compute honestly. Every
    // known caller in the L1 controller (CPP-017) compares locations built
    // in the same frame within one control cycle, so this fast path is the
    // one that matters there; noted as a real gap for any future caller
    // that isn't.
    [[nodiscard]] bool same_alt_as(const Location& loc2) const {
        if (get_alt_frame() == loc2.get_alt_frame()) {
            return alt == loc2.alt;
        }
        return false;
    }

    [[nodiscard]] bool same_loc_as(const Location& loc2) const {
        return same_latlon_as(loc2) && same_alt_as(loc2);
    }
};

} // namespace fwcpp
