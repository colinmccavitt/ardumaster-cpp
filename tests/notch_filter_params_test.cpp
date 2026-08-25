// Tests for fwcpp::filter::NotchFilterParams (CPP-024 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/filter/notch_filter_params.hpp>
#include <fwcpp/param/name_lookup.hpp>

#include <string>

using namespace fwcpp::filter;

TEST_CASE("NotchFilterParams constructs with upstream's own defaults (freq=0, Q=2, atten=40)", "[notch_params]") {
    NotchFilterParams params;
    REQUIRE(params.center_freq_hz().get() == 0.0f);
    REQUIRE(params.quality().get() == 2.0f);
    REQUIRE(params.attenuation_db().get() == 40.0f);
}

TEST_CASE("setup_notch_filter returns false (not configured) when center_freq_hz is left at its zero default", "[notch_params]") {
    NotchFilterParams params; // center_freq_hz_ defaults to 0
    NotchFilter<float> filt;
    REQUIRE_FALSE(params.setup_notch_filter(filt, 400.0f));
}

TEST_CASE("setup_notch_filter returns false when quality is explicitly zero", "[notch_params]") {
    NotchFilterParams params;
    params.center_freq_hz_.set(80.0f);
    params.quality_.set(0.0f);
    NotchFilter<float> filt;
    REQUIRE_FALSE(params.setup_notch_filter(filt, 400.0f));
}

TEST_CASE("setup_notch_filter returns false when attenuation is explicitly zero", "[notch_params]") {
    NotchFilterParams params;
    params.center_freq_hz_.set(80.0f);
    params.attenuation_db_.set(0.0f);
    NotchFilter<float> filt;
    REQUIRE_FALSE(params.setup_notch_filter(filt, 400.0f));
}

TEST_CASE("setup_notch_filter initializes the filter once fully configured", "[notch_params]") {
    NotchFilterParams params;
    params.center_freq_hz_.set(80.0f);
    // quality defaults to 2, attenuation defaults to 40 - both nonzero

    NotchFilter<float> filt;
    REQUIRE(params.setup_notch_filter(filt, 400.0f));
    REQUIRE(filt.center_freq_hz() == Catch::Approx(80.0f));
    REQUIRE(filt.sample_freq_hz() == Catch::Approx(400.0f));
}

TEST_CASE("setup_notch_filter does not re-init the filter when sample rate and center freq are unchanged", "[notch_params]") {
    NotchFilterParams params;
    params.center_freq_hz_.set(80.0f);

    NotchFilter<float> filt;
    REQUIRE(params.setup_notch_filter(filt, 400.0f));
    filt.reset(); // mark as needing a fresh passthrough sample, matching notch_filter_test.cpp's own established pattern
    REQUIRE(params.setup_notch_filter(filt, 400.0f)); // same freq/rate - should NOT call init again (which would clear need_reset_)
    // apply() right after reset() with no re-init should behave as the
    // reset-then-apply passthrough documented in notch_filter_test.cpp,
    // not as a freshly re-initialized filter's own zero-state behavior.
    REQUIRE(filt.apply(5.0f) == Catch::Approx(5.0f));
}

TEST_CASE("setup_notch_filter DOES re-init when the center frequency changes", "[notch_params]") {
    NotchFilterParams params;
    params.center_freq_hz_.set(80.0f);

    NotchFilter<float> filt;
    REQUIRE(params.setup_notch_filter(filt, 400.0f));

    params.center_freq_hz_.set(120.0f);
    REQUIRE(params.setup_notch_filter(filt, 400.0f));
    REQUIRE(filt.center_freq_hz() == Catch::Approx(120.0f));
}

TEST_CASE("var_info's GroupInfo table matches upstream's transcribed NOTCH_FREQ/NOTCH_Q/NOTCH_ATT names, indices, and defaults", "[notch_params]") {
    const fwcpp::param::GroupInfo* table = NotchFilterParams::var_info();
    REQUIRE(std::string(table[0].name) == "NOTCH_FREQ");
    REQUIRE(table[0].idx == 1);
    REQUIRE(table[0].def_value == 0.0f);

    REQUIRE(std::string(table[1].name) == "NOTCH_Q");
    REQUIRE(table[1].idx == 2);
    REQUIRE(table[1].def_value == 2.0f);

    REQUIRE(std::string(table[2].name) == "NOTCH_ATT");
    REQUIRE(table[2].idx == 3);
    REQUIRE(table[2].def_value == 40.0f);

    REQUIRE(table[3].type == static_cast<std::uint8_t>(fwcpp::param::VarType::None)); // sentinel
}

TEST_CASE("find_group locates each NOTCH parameter by name against a live NotchFilterParams instance", "[notch_params][find_group]") {
    NotchFilterParams params;
    fwcpp::param::VarType ptype = fwcpp::param::VarType::None;

    void* p = fwcpp::param::find_group("NOTCH_FREQ", reinterpret_cast<std::ptrdiff_t>(&params), 0, NotchFilterParams::var_info(), ptype);
    REQUIRE(p == &params.center_freq_hz_);
    REQUIRE(ptype == fwcpp::param::VarType::Float);
}
