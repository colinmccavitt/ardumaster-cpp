// Definitions for the wrap_* family declared in scalar.hpp. Compiled under
// fwcpp_upstream_flags (see CMakeLists.txt) - this is the ONE translation
// unit where -fsingle-precision-constant applies to these bodies, matching
// upstream's own AP_Math.cpp. See scalar.hpp's file banner for why these
// aren't header-inline.

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::math {

// Upstream AP_Math/definitions.h defines its own M_PI (not <cmath>'s),
// digit-for-digit reproduced here. Under -fsingle-precision-constant this
// bare literal is parsed as a float constant same as it is upstream - that
// is the point, not an oversight; it's why these bodies live in this
// specific translation unit.
static constexpr double kPi = 3.141592653589793238462643383279502884;
static constexpr double kTwoPi = kPi * 2;

// DEG_TO_RAD/RAD_TO_DEG: upstream (definitions.h) writes these as
// `M_PI / 180.0f` and `180.0f / M_PI` - under -fsingle-precision-constant
// M_PI's bare digits parse as float, so the *whole* constant is float
// precision even though it multiplies against a double `deg` in the double
// radians() overload. kDegToRad below is deliberately computed from a
// double kPi and then narrowed through float explicitly, to land on the
// exact bit pattern upstream's own float-typed constant has - not the
// higher-precision value a naive double computation would give.
static constexpr float kDegToRad = static_cast<float>(kPi) / 180.0f;
static constexpr float kRadToDeg = 180.0f / static_cast<float>(kPi);

double radians(double deg) {
    return deg * static_cast<double>(kDegToRad);
}

float radians(float deg) {
    return deg * kDegToRad;
}

float radians(int deg) {
    return static_cast<float>(deg) * kDegToRad;
}

float degrees(float rad) {
    return rad * kRadToDeg;
}

// RAD_TO_CDEG upstream is `18000.0f / M_PI` - float precision, same as
// kRadToDeg's reasoning, scaled by 100 (degrees -> centidegrees).
static constexpr float kRadToCDeg = 18000.0f / static_cast<float>(kPi);

float rad_to_cd(float rad) {
    return rad * kRadToCDeg;
}

// CDEG_TO_RAD upstream is `M_PI / 18000.0f` - float precision, same
// reasoning as kRadToCDeg/kDegToRad.
static constexpr float kCdegToRad = static_cast<float>(kPi) / 18000.0f;

float cd_to_rad(float cdeg) {
    return cdeg * kCdegToRad;
}

double pi_constant() {
    return kPi;
}

float deg_to_rad_constant() {
    return kDegToRad;
}

float wrap_360(float angle) {
    float res = std::fmod(angle, 360.0f);
    if (res < 0) {
        res += 360.0f;
    }
    return res;
}

double wrap_360(double angle) {
    double res = std::fmod(angle, 360.0);
    if (res < 0) {
        res += 360.0;
    }
    return res;
}

int wrap_360(int angle) {
    int res = angle % 360;
    if (res < 0) {
        res += 360;
    }
    return res;
}

float wrap_360_cd(float angle) {
    float res = std::fmod(angle, 36000.0f);
    if (res < 0) {
        res += 36000.0f;
    }
    return res;
}

double wrap_360_cd(double angle) {
    double res = std::fmod(angle, 36000.0);
    if (res < 0) {
        res += 36000.0;
    }
    return res;
}

long wrap_360_cd(long angle) {
    long res = angle % 36000;
    if (res < 0) {
        res += 36000;
    }
    return res;
}

int wrap_360_cd(int angle) {
    int res = angle % 36000;
    if (res < 0) {
        res += 36000;
    }
    return res;
}

float wrap_180(float angle) {
    float res = wrap_360(angle);
    if (res > 180.0f) {
        res -= 360.0f;
    }
    return res;
}

double wrap_180(double angle) {
    double res = wrap_360(angle);
    if (res > 180.0) {
        res -= 360.0;
    }
    return res;
}

int wrap_180(int angle) {
    int res = wrap_360(angle);
    if (res > 180) {
        res -= 360;
    }
    return res;
}

short wrap_180(short angle) {
    return static_cast<short>(wrap_180(static_cast<int>(angle)));
}

float wrap_180_cd(float angle) {
    float res = wrap_360_cd(angle);
    if (res > 18000.0f) {
        res -= 36000.0f;
    }
    return res;
}

double wrap_180_cd(double angle) {
    double res = wrap_360_cd(angle);
    if (res > 18000.0) {
        res -= 36000.0;
    }
    return res;
}

int wrap_180_cd(int angle) {
    int res = wrap_360_cd(angle);
    if (res > 18000) {
        res -= 36000;
    }
    return res;
}

long wrap_180_cd(long angle) {
    long res = wrap_360_cd(angle);
    if (res > 18000) {
        res -= 36000;
    }
    return res;
}

short wrap_180_cd(short angle) {
    return static_cast<short>(wrap_180_cd(static_cast<int>(angle)));
}

float wrap_2PI(float radian) {
    float res = std::fmod(radian, static_cast<float>(kTwoPi));
    if (res < 0) {
        res += static_cast<float>(kTwoPi);
    }
    return res;
}

double wrap_2PI(double radian) {
    double res = std::fmod(radian, kTwoPi);
    if (res < 0) {
        res += kTwoPi;
    }
    return res;
}

float wrap_PI(float radian) {
    float res = wrap_2PI(radian);
    if (res > static_cast<float>(kPi)) {
        res -= static_cast<float>(kTwoPi);
    }
    return res;
}

double wrap_PI(double radian) {
    double res = wrap_2PI(radian);
    if (res > kPi) {
        res -= kTwoPi;
    }
    return res;
}

float calc_lowpass_alpha_dt(float dt, float cutoff_freq, InternalError* err, std::uint16_t line) {
    if (is_negative(dt) || is_negative(cutoff_freq)) {
        if (err != nullptr) {
            err->record(InternalErrorCode::invalid_arg_or_result, line);
        }
        return 1.0f;
    }
    if (is_zero(cutoff_freq)) {
        return 1.0f;
    }
    if (is_zero(dt)) {
        return 0.0f;
    }
    const float rc = 1.0f / (static_cast<float>(kTwoPi) * cutoff_freq);
    return dt / (dt + rc);
}

} // namespace fwcpp::math
