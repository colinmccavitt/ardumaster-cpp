#pragma once

// Port of AP_Common/Location.h + Location.cpp. CPP-011.
//
// SLICE BOUNDARY: struct (bitfield flags, lat/lng/alt, AltFrame), zero(),
// the (lat, lng, alt, frame) constructor, set_alt_cm/set_alt_m,
// get_alt_frame, longitude_scale/diff_longitude/wrap_longitude/
// limit_lattitude, offset_latlng/offset/offset_bearing/
// offset_bearing_and_pitch, get_distance, get_distance_NE (Vector2f only),
// get_distance_NED (Vector3f only), same_latlon_as/same_alt_as/
// same_loc_as, is_zero, check_latlng, line_path_proportion,
// past_interval_finish_line, get_alt_cm/get_alt_m/change_alt_frame/
// copy_alt_from/initialised/sanitize (via AltitudeContext).
//
// is_zero/check_latlng/line_path_proportion/past_interval_finish_line
// (slice 2) were picked out specifically because they're the remaining
// functions with NO home-position/EKF-origin/terrain dependency.
// check_lat/check_lng are upstream free functions (AP_Math/location.cpp);
// nothing else in this port calls them yet, so they're inlined into
// check_latlng() here rather than given their own public names.
//
// SLICE 3 adds get_alt_cm/get_alt_m/change_alt_frame/copy_alt_from/
// initialised/sanitize, via a new explicit AltitudeContext struct (see its
// own comment below) carrying home position and EKF origin - matches this
// port's standing pattern (L1Inputs, constrain_value's InternalError*) of
// taking external state as a parameter instead of reaching for a
// singleton (AP::ahrs()). Terrain-frame conversions (AltFrame::
// ABOVE_TERRAIN as EITHER the location's own frame or the desired frame)
// still fail unconditionally here, same as upstream's own #if
// AP_TERRAIN_AVAILABLE / #else return false / #endif path compiles to
// when terrain support is off - AP_Terrain is a real subsystem (a
// database, not a value) this port hasn't built, and AltitudeContext
// carrying a single caller-supplied terrain altitude wouldn't actually
// be equivalent to it (the real function can be asked about ANY
// location, not just the one Location instance a single cm value would
// cover) - so this isn't a smaller version of terrain support, it's
// honestly no terrain support, matching a real board built without it.
// sanitize()'s alt==0/relative_alt branch inherits this same limit: if
// get_alt_cm can't resolve default_loc's altitude (terrain frame, or
// context missing), that branch is simply skipped - matching upstream's
// own `if (...) { alt = ...; }` with no else, not a port-specific gap.
//
// SLICE 6 (final): postype.hpp landed (see that file), unblocking
// get_vector_xy_from_origin_NE_cm/get_vector_from_origin_NEU_cm/
// get_vector_from_origin_NEU/get_vector_xy_from_origin_NE_m/
// get_vector_from_origin_NED_m/get_vector_from_origin_NEU_m,
// get_distance_NE_postype/get_distance_NED_postype, and the ekf_offset
// constructors - reworked as static bool-returning factories
// (from_ekf_offset_NEU_cm/from_ekf_offset_NED_m) rather than upstream's
// silently-degrading constructor shape (see those functions' own
// comment). linearly_interpolate_alt was also added - it turned out to
// have no home/EKF-origin dependency at all (built entirely from already-
// ported line_path_proportion/constrain_value/set_alt_cm), just placed
// late in upstream's Location.cpp file order. That resolves an earlier
// version of this banner's claim about "great-circle/line-intersection
// helpers past line ~530" - re-checked directly against upstream and
// there aren't any; that claim was wrong and is corrected here rather
// than perpetuated. With this slice, CPP-011 covers every member of
// upstream Location except get_alt_cm/friends' terrain-frame path (no
// AP_Terrain in this port - see SLICE 3's own note, a real and permanent
// gap until AP_Terrain exists) and the Vector2p/Vector3p double-typed
// get_distance_*_postype's sibling `get_distance_NE_ftype`/
// `get_distance_NED_double` (ftype doesn't exist in this port yet -
// EKF-precision plumbing, CPP-011's own notes track it as a follow-on
// once an EKF module needs it).
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
#include <cstdlib>
#include <cstring>

#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp {

struct AltitudeContext; // full definition below Location - see get_alt_cm's own comment

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

    // Upstream compares the whole object's memory against a zero-inited
    // Location via memcmp - not reproduced (ADR-0012 no-unsafe-
    // reinterpretation stance, same reasoning as zero() above). Explicit
    // field-by-field comparison is exactly equivalent for this struct: it
    // has no padding-sensitive layout dependency anywhere else in this
    // port, so there's no observable difference from the memcmp version.
    [[nodiscard]] bool is_zero() const {
        return lat == 0 && lng == 0 && alt == 0
            && relative_alt == 0 && loiter_ccw == 0 && terrain_alt == 0
            && origin_alt == 0 && loiter_xtrack == 0;
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

    void copy_alt_from(const Location& other) {
        alt = other.alt;
        relative_alt = other.relative_alt;
        terrain_alt = other.terrain_alt;
        origin_alt = other.origin_alt;
    }

    // Upstream's own definition: only lat/lng/alt, NOT the bitfield flags
    // is_zero() checks - a Location with alt frame flags set but lat=lng=
    // alt=0 is "not initialised" the same as an all-zero one, but a
    // Location with a nonzero alt in ABOVE_HOME frame (lat=lng=0, alt!=0)
    // IS initialised despite looking exotic. Reproduced exactly as
    // upstream defines it, not derived from is_zero().
    [[nodiscard]] bool initialised() const { return lat != 0 || lng != 0 || alt != 0; }

    // Get this location's altitude in a different frame. Returns false
    // (leaving ret_alt_cm untouched) if the conversion isn't possible with
    // the given context - matching upstream's own failure contract exactly
    // (home not set, EKF origin not available, or a terrain-frame
    // conversion, which this port cannot honor - see file banner and
    // AltitudeContext's own comment). Declared here, defined below
    // AltitudeContext (needs the complete type).
    [[nodiscard]] bool get_alt_cm(AltFrame desired_frame, const AltitudeContext& ctx, std::int32_t& ret_alt_cm) const;

    [[nodiscard]] bool get_alt_m(AltFrame desired_frame, const AltitudeContext& ctx, float& ret_alt) const;

    // Converts this Location's own altitude (in place) to desired_frame.
    // Returns false (leaving this Location unchanged) if get_alt_cm fails.
    bool change_alt_frame(AltFrame desired_frame, const AltitudeContext& ctx);

    // Fills in lat/lng/alt from default_loc wherever this Location's own
    // values are missing or out of range, returning true if anything
    // changed. Declared here, defined below AltitudeContext (its alt
    // branch calls get_alt_cm).
    bool sanitize(const Location& default_loc, const AltitudeContext& ctx);

    // North/East vector (in cm) from ctx.ekf_origin to this Location.
    // Returns false (leaving vec_ne unmodified) if ctx has no EKF origin.
    // T is any type with .x/.y members - Vector2f, Vector2p, and (since
    // Vector3<T> also has plain .x/.y members, no aliasing needed) Vector3f/
    // Vector3p all work, which is exactly how get_vector_from_origin_NEU_cm
    // below reuses this same function on a Vector3 argument instead of
    // upstream's reinterpret_cast-based vec_neu.xy() trick (see this
    // header's own SLICE BOUNDARY note on why xy() itself isn't ported).
    // All defined below AltitudeContext (needs the complete type).
    template <typename T>
    [[nodiscard]] bool get_vector_xy_from_origin_NE_cm(T& vec_ne, const AltitudeContext& ctx) const;

    // North/East/Up vector (in cm) from ctx.ekf_origin to this Location's
    // full 3D position. Returns false if either the horizontal vector or
    // the altitude-above-origin conversion fails.
    template <typename T>
    [[nodiscard]] bool get_vector_from_origin_NEU_cm(T& vec_neu, const AltitudeContext& ctx) const;

    // Alias for get_vector_from_origin_NEU_cm - matches upstream's own
    // same-named alias (the "_cm" suffix used to be the only variant;
    // this name predates the _m-suffixed ones below).
    template <typename T>
    [[nodiscard]] bool get_vector_from_origin_NEU(T& vec_neu, const AltitudeContext& ctx) const;

    template <typename T>
    [[nodiscard]] bool get_vector_xy_from_origin_NE_m(T& vec_ne, const AltitudeContext& ctx) const;

    template <typename T>
    [[nodiscard]] bool get_vector_from_origin_NED_m(T& vec_ned, const AltitudeContext& ctx) const;

    template <typename T>
    [[nodiscard]] bool get_vector_from_origin_NEU_m(T& vec_neu, const AltitudeContext& ctx) const;

    // Builds a Location from an NEU (north/east/up) offset in CENTIMETERS
    // relative to ctx.ekf_origin, at the given altitude frame.
    //
    // DELIBERATE DIVERGENCE from upstream's shape, not its behavior:
    // upstream expresses this as a CONSTRUCTOR that silently leaves lat/lng
    // at 0 if ctx has no EKF origin (a constructor has no way to signal
    // failure). This port's own standing convention (get_alt_cm and
    // everything built on it) is bool-return + out-param for anything that
    // can fail - applied here too, as a static factory instead of a
    // constructor. `out` receives EXACTLY what upstream's constructor
    // would have produced in both the success and failure case (zero()'d,
    // then alt/frame always set via set_alt_cm regardless of origin
    // availability, lat/lng only set - via offset() from ctx.ekf_origin -
    // when this returns true) - the return value is new information this
    // port can expose that upstream's constructor shape couldn't, not a
    // change to what `out` ends up containing.
    template <typename T>
    static bool from_ekf_offset_NEU_cm(const math::Vector3<T>& ekf_offset_neu_cm, AltFrame frame, const AltitudeContext& ctx, Location& out);

    // Same as from_ekf_offset_NEU_cm, but takes an NED offset in METERS -
    // matches upstream's own from_ekf_offset_NED_m named constructor
    // exactly (x/y unchanged, z negated and both scaled by 100 to reach
    // NEU-centimeters, then delegates).
    template <typename T>
    static bool from_ekf_offset_NED_m(const math::Vector3<T>& ekf_offset_ned_m, AltFrame frame, const AltitudeContext& ctx, Location& out);

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

    // Upstream's LATLON_TO_CM macro (definitions.h) is LOCATION_SCALING_
    // FACTOR * 100 written as its own separate literal
    // (1.1131884502145034), not derived from LOCATION_SCALING_FACTOR at
    // compile time - reproduced as its own explicit-float constant here
    // too, rather than computing LOCATION_SCALING_FACTOR * 100.0f, since
    // upstream's own two independently-rounded-to-float literals aren't
    // guaranteed to be bit-identical to one float value scaled by another.
    static constexpr float LOCATION_SCALING_FACTOR_CM = 1.1131884502145034f;

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

    // North/East distance to loc2, in meters, at postype_t precision.
    // UPSTREAM'S OWN GENUINE PRECISION GAIN, reproduced deliberately: this
    // one casts LOCATION_SCALING_FACTOR to double BEFORE multiplying,
    // so when postype_t is double the multiplication itself happens at
    // double precision - unlike get_distance_NE (and unlike
    // get_distance_NED_postype right below), which multiplies as float
    // regardless of the result type. Not a port inconsistency - upstream's
    // own Location.cpp has get_distance_NE_postype do this and
    // get_distance_NED_postype NOT do it, side by side. Preserved exactly.
    [[nodiscard]] math::Vector2p get_distance_NE_postype(const Location& loc2) const {
        return math::Vector2p(
            static_cast<double>(loc2.lat - lat) * static_cast<double>(LOCATION_SCALING_FACTOR),
            static_cast<double>(diff_longitude(loc2.lng, lng)) * static_cast<double>(LOCATION_SCALING_FACTOR)
                * longitude_scale((lat + loc2.lat) / 2));
    }

    // North/East/Down distance to loc2, in meters, at postype_t precision.
    // See get_distance_NE_postype's comment: THIS overload does NOT cast
    // LOCATION_SCALING_FACTOR to double - float-precision arithmetic
    // widened into a double-typed Vector3p result, no more accurate than
    // get_distance_NED despite the name. Matches upstream exactly,
    // including the inconsistency with get_distance_NE_postype above.
    [[nodiscard]] math::Vector3p get_distance_NED_postype(const Location& loc2) const {
        return math::Vector3p(
            static_cast<float>(loc2.lat - lat) * LOCATION_SCALING_FACTOR,
            static_cast<float>(diff_longitude(loc2.lng, lng)) * LOCATION_SCALING_FACTOR * longitude_scale((lat + loc2.lat) / 2),
            static_cast<float>(alt - loc2.alt) * 0.01f);
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

    // Upstream's check_lat(int32_t)/check_lng(int32_t) (AP_Math/location.cpp)
    // inlined here - see file banner for why they aren't given their own
    // names.
    [[nodiscard]] bool check_latlng() const {
        return std::abs(lat) <= 900000000L && std::abs(lng) <= 1800000000L;
    }

    // Proportion of the way along the path from point1 to point2 that
    // *this* location's perpendicular projection falls at. 0 at point1, 1
    // at point2; can exceed 1 (past point2) or be negative (before
    // point1). If point1 and point2 are within ~3cm of each other,
    // returns 1.0 rather than dividing by a near-zero denominator -
    // matches upstream's own 0.001 (m^2) threshold exactly.
    [[nodiscard]] float line_path_proportion(const Location& point1, const Location& point2) const {
        const math::Vector2f vec1 = point1.get_distance_NE(point2);
        const math::Vector2f vec2 = point1.get_distance_NE(*this);
        const float dsquared = vec1.x * vec1.x + vec1.y * vec1.y;
        if (dsquared < 0.001f) {
            return 1.0f;
        }
        return (vec1 * vec2) / dsquared;
    }

    // True once this location has flown past the line through point2,
    // perpendicular to the point1->point2 track - the standard "have we
    // reached/passed the waypoint" test.
    [[nodiscard]] bool past_interval_finish_line(const Location& point1, const Location& point2) const {
        return line_path_proportion(point1, point2) >= 1.0f;
    }

    // Sets this Location's altitude (in point2's alt frame) by linearly
    // interpolating between point1's and point2's altitudes, using this
    // Location's own line_path_proportion along the point1->point2 track
    // (clamped to [0,1] - even if this Location is technically before
    // point1 or past point2, the interpolated altitude stays within
    // [point1.alt, point2.alt]).
    void linearly_interpolate_alt(const Location& point1, const Location& point2) {
        const float t = math::constrain_value(line_path_proportion(point1, point2), 0.0f, 1.0f);
        const float interpolated = static_cast<float>(point1.alt) + static_cast<float>(point2.alt - point1.alt) * t;
        set_alt_cm(static_cast<std::int32_t>(interpolated), point2.get_alt_frame());
    }
};

// Everything get_alt_cm needs from the AHRS for one altitude-frame
// conversion - see file banner. home/ekf_origin are themselves Locations
// (their own alt fields carry the ABSOLUTE altitude each is anchored at,
// matching upstream's AP::ahrs().get_home()/get_origin()); the *_is_set
// flags mirror upstream's own home_is_set()/get_origin() bool-return
// contract, since neither is guaranteed available at any given moment.
//
// Deliberately NOT modeled: terrain altitude as a general lookup. Upstream
// can ask AP_Terrain for the AMSL height under ANY location; this struct
// can only carry a value already looked up for one specific Location by
// the caller - not equivalent, so it isn't offered as a substitute at
// all. Every terrain-frame conversion fails here, honestly, matching a
// real board built without AP_TERRAIN_AVAILABLE.
struct AltitudeContext {
    bool home_is_set = false;
    Location home;
    bool origin_is_set = false;
    Location ekf_origin;
};

inline bool Location::get_alt_cm(AltFrame desired_frame, const AltitudeContext& ctx, std::int32_t& ret_alt_cm) const {
    const AltFrame frame = get_alt_frame();

    // Shortcut if desired and underlying frame are the same - matches
    // upstream, and is the only path that works when frame ==
    // desired_frame == ABOVE_TERRAIN (this port has no terrain database,
    // but doesn't need one if no actual conversion is being asked for).
    if (desired_frame == frame) {
        ret_alt_cm = alt;
        return true;
    }

    // Terrain frame involved on either side and no terrain database in
    // this port - matches upstream's own #else return false #endif path.
    if (frame == AltFrame::ABOVE_TERRAIN || desired_frame == AltFrame::ABOVE_TERRAIN) {
        return false;
    }

    std::int32_t alt_abs = 0;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            alt_abs = alt;
            break;
        case AltFrame::ABOVE_HOME:
            if (!ctx.home_is_set) {
                return false;
            }
            alt_abs = alt + ctx.home.alt;
            break;
        case AltFrame::ABOVE_ORIGIN:
            if (!ctx.origin_is_set) {
                return false;
            }
            alt_abs = alt + ctx.ekf_origin.alt;
            break;
        case AltFrame::ABOVE_TERRAIN:
            return false; // unreachable (handled above) - no fallthrough
    }

    switch (desired_frame) {
        case AltFrame::ABSOLUTE:
            ret_alt_cm = alt_abs;
            return true;
        case AltFrame::ABOVE_HOME:
            if (!ctx.home_is_set) {
                return false;
            }
            ret_alt_cm = alt_abs - ctx.home.alt;
            return true;
        case AltFrame::ABOVE_ORIGIN:
            if (!ctx.origin_is_set) {
                return false;
            }
            ret_alt_cm = alt_abs - ctx.ekf_origin.alt;
            return true;
        case AltFrame::ABOVE_TERRAIN:
            return false; // unreachable (handled above) - no fallthrough
    }
    return false;
}

inline bool Location::get_alt_m(AltFrame desired_frame, const AltitudeContext& ctx, float& ret_alt) const {
    std::int32_t ret_alt_cm;
    if (!get_alt_cm(desired_frame, ctx, ret_alt_cm)) {
        return false;
    }
    ret_alt = static_cast<float>(ret_alt_cm) * 0.01f;
    return true;
}

inline bool Location::change_alt_frame(AltFrame desired_frame, const AltitudeContext& ctx) {
    std::int32_t new_alt_cm;
    if (!get_alt_cm(desired_frame, ctx, new_alt_cm)) {
        return false;
    }
    set_alt_cm(new_alt_cm, desired_frame);
    return true;
}

inline bool Location::sanitize(const Location& default_loc, const AltitudeContext& ctx) {
    bool has_changed = false;

    // lat/lng == 0 conventionally means "use the current point".
    if (lat == 0 && lng == 0) {
        lat = default_loc.lat;
        lng = default_loc.lng;
        has_changed = true;
    }

    // A relative altitude of exactly 0 conventionally means "use the
    // current altitude" - only reachable if default_loc.get_alt_cm
    // actually succeeds (e.g. AltitudeContext has the needed home/origin
    // set); otherwise this Location's alt is left as-is, matching
    // upstream's own `if (...) { alt = ...; }` (no else branch).
    if (alt == 0 && relative_alt) {
        std::int32_t default_loc_alt = 0;
        if (default_loc.get_alt_cm(get_alt_frame(), ctx, default_loc_alt)) {
            alt = default_loc_alt;
            has_changed = true;
        }
    }

    if (!check_latlng()) {
        lat = default_loc.lat;
        lng = default_loc.lng;
        has_changed = true;
    }

    return has_changed;
}

template <typename T>
inline bool Location::get_vector_xy_from_origin_NE_cm(T& vec_ne, const AltitudeContext& ctx) const {
    if (!ctx.origin_is_set) {
        return false;
    }
    // Bare LOCATION_SCALING_FACTOR_CM, no double() upcast - matches
    // upstream's own get_vector_xy_from_origin_NE_cm exactly (unlike
    // get_distance_NE_postype above, this family never gains real double
    // precision even when T's members are double - see that function's
    // own comment for the upstream inconsistency this preserves).
    vec_ne.x = static_cast<float>(lat - ctx.ekf_origin.lat) * LOCATION_SCALING_FACTOR_CM;
    vec_ne.y = static_cast<float>(diff_longitude(lng, ctx.ekf_origin.lng)) * LOCATION_SCALING_FACTOR_CM
        * longitude_scale((lat + ctx.ekf_origin.lat) / 2);
    return true;
}

template <typename T>
inline bool Location::get_vector_from_origin_NEU_cm(T& vec_neu, const AltitudeContext& ctx) const {
    std::int32_t alt_above_origin_cm = 0;
    if (!get_alt_cm(AltFrame::ABOVE_ORIGIN, ctx, alt_above_origin_cm)) {
        return false;
    }
    if (!get_vector_xy_from_origin_NE_cm(vec_neu, ctx)) {
        return false;
    }
    // Direct int32_t -> T assignment, matching upstream's own
    // `vec_neu.z = alt_above_origin_cm;` exactly - NOT routed through
    // float first, which would needlessly truncate precision for
    // T=Vector3p (double) on large altitude values (float's 24-bit
    // mantissa can't exactly represent every int32_t; double's 53-bit one
    // can, for any centimeter altitude this port will ever see).
    vec_neu.z = alt_above_origin_cm;
    return true;
}

template <typename T>
inline bool Location::get_vector_from_origin_NEU(T& vec_neu, const AltitudeContext& ctx) const {
    return get_vector_from_origin_NEU_cm(vec_neu, ctx);
}

template <typename T>
inline bool Location::get_vector_xy_from_origin_NE_m(T& vec_ne, const AltitudeContext& ctx) const {
    if (!get_vector_xy_from_origin_NE_cm(vec_ne, ctx)) {
        return false;
    }
    vec_ne *= 0.01f;
    return true;
}

template <typename T>
inline bool Location::get_vector_from_origin_NED_m(T& vec_ned, const AltitudeContext& ctx) const {
    if (!get_vector_from_origin_NEU_cm(vec_ned, ctx)) {
        return false;
    }
    vec_ned *= 0.01f;
    vec_ned.z *= -1.0f;
    return true;
}

template <typename T>
inline bool Location::get_vector_from_origin_NEU_m(T& vec_neu, const AltitudeContext& ctx) const {
    if (!get_vector_from_origin_NEU_cm(vec_neu, ctx)) {
        return false;
    }
    vec_neu *= 0.01f;
    return true;
}

template <typename T>
inline bool Location::from_ekf_offset_NEU_cm(const math::Vector3<T>& ekf_offset_neu_cm, AltFrame frame, const AltitudeContext& ctx, Location& out) {
    out.zero();
    out.set_alt_cm(static_cast<std::int32_t>(ekf_offset_neu_cm.z), frame);
    if (!ctx.origin_is_set) {
        return false;
    }
    out.lat = ctx.ekf_origin.lat;
    out.lng = ctx.ekf_origin.lng;
    out.offset(static_cast<float>(ekf_offset_neu_cm.x) * 0.01f, static_cast<float>(ekf_offset_neu_cm.y) * 0.01f);
    return true;
}

template <typename T>
inline bool Location::from_ekf_offset_NED_m(const math::Vector3<T>& ekf_offset_ned_m, AltFrame frame, const AltitudeContext& ctx, Location& out) {
    const math::Vector3<T> ekf_offset_neu_cm(
        ekf_offset_ned_m.x * static_cast<T>(100), ekf_offset_ned_m.y * static_cast<T>(100), -ekf_offset_ned_m.z * static_cast<T>(100));
    return from_ekf_offset_NEU_cm(ekf_offset_neu_cm, frame, ctx, out);
}

} // namespace fwcpp
