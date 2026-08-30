#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include <fwcpp/sim/sim_gps_factory.hpp>

using namespace fwcpp::sim;

namespace {

GPS_Data sample() {
    GPS_Data d;
    d.timestamp_ms = 5000;
    d.latitude = -35.363261;
    d.longitude = 149.165230;
    d.altitude = 584.0f;
    d.speedN = 1.5;
    d.speedE = 0.25;
    d.speedD = -0.1;
    d.yaw_deg = 45.0;
    d.have_lock = true;
    d.num_sats = 10;
    d.horizontal_acc = 0.3f;
    d.vertical_acc = 0.5f;
    d.speed_acc = 0.3f;
    return d;
}

}  // namespace

TEST_CASE("UBLOX publish starts with UBX preamble and encodes lat/lon in POSLLH") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::UBLOX;
    gps.check_backend_allocation();
    REQUIRE(gps.backend() != nullptr);
    const auto d = sample();
    gps.publish_sample(d);
    const auto bytes = gps.drain_to_autopilot();
    REQUIRE(bytes.size() > 8);
    REQUIRE(bytes[0] == 0xb5);
    REQUIRE(bytes[1] == 0x62);
    REQUIRE(bytes[2] == 0x01);  // CLASS_NAV
    REQUIRE(bytes[3] == 0x02);  // POSLLH
    std::int32_t lon = 0;
    std::int32_t lat = 0;
    std::memcpy(&lon, &bytes[6 + 4], 4);
    std::memcpy(&lat, &bytes[6 + 8], 4);
    REQUIRE(lon == static_cast<std::int32_t>(d.longitude * 1.0e7));
    REQUIRE(lat == static_cast<std::int32_t>(d.latitude * 1.0e7));
}

TEST_CASE("NMEA publish emits GPGGA/GPVTG/GPRMC with checksum trailer") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::NMEA;
    gps.check_backend_allocation();
    GPS_Backend::set_sim_clock(0, 1600000000);
    gps.publish_sample(sample());
    const auto bytes = gps.drain_to_autopilot();
    const std::string s(bytes.begin(), bytes.end());
    REQUIRE(s.find("$GPGGA,") != std::string::npos);
    REQUIRE(s.find("$GPVTG,") != std::string::npos);
    REQUIRE(s.find("$GPRMC,") != std::string::npos);
    REQUIRE(s.find("*") != std::string::npos);
    REQUIRE(s.find("\r\n") != std::string::npos);
}

TEST_CASE("SBP publish frames with 0x55 preamble") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::SBP;
    gps.check_backend_allocation();
    gps.publish_sample(sample());
    const auto bytes = gps.drain_to_autopilot();
    REQUIRE_FALSE(bytes.empty());
    REQUIRE(bytes[0] == 0x55);
}

TEST_CASE("SBP2 publish frames with 0x55 preamble and time msg 0x0102") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::SBP2;
    gps.check_backend_allocation();
    gps.publish_sample(sample());
    const auto bytes = gps.drain_to_autopilot();
    REQUIRE(bytes.size() > 3);
    REQUIRE(bytes[0] == 0x55);
    std::uint16_t msg = 0;
    std::memcpy(&msg, &bytes[1], 2);
    REQUIRE(msg == 0x0102);
}

TEST_CASE("SBF publish starts with $@ and PVTGeodetic id 0x0FA7") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::SBF;
    gps.check_backend_allocation();
    gps.publish_sample(sample());
    const auto bytes = gps.drain_to_autopilot();
    REQUIRE(bytes.size() > 8);
    REQUIRE(bytes[0] == 0x24);
    REQUIRE(bytes[1] == 0x40);
    std::uint16_t msgid = static_cast<std::uint16_t>(bytes[4] | (bytes[5] << 8));
    REQUIRE(msgid == 0x0FA7);
}

TEST_CASE("NOVA publish starts with AA 44 12") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::NOVA;
    gps.check_backend_allocation();
    gps.publish_sample(sample());
    const auto bytes = gps.drain_to_autopilot();
    REQUIRE(bytes.size() > 3);
    REQUIRE(bytes[0] == 0xaa);
    REQUIRE(bytes[1] == 0x44);
    REQUIRE(bytes[2] == 0x12);
    REQUIRE(gps.device_baud() == 19200);
}

TEST_CASE("MSP publish starts with $X< GPS command") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::MSP;
    gps.check_backend_allocation();
    GPS_Backend::set_sim_clock(0, 1600000000);
    gps.publish_sample(sample());
    const auto bytes = gps.drain_to_autopilot();
    REQUIRE(bytes.size() > 8);
    REQUIRE(bytes[0] == 0x24);
    REQUIRE(bytes[1] == 0x58);
    REQUIRE(bytes[2] == 0x3C);
    std::uint16_t cmd = 0;
    std::memcpy(&cmd, &bytes[4], 2);
    REQUIRE(cmd == 0x1F03);
}

TEST_CASE("FILE backend does not crash when log is missing") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::FILE;
    gps.check_backend_allocation();
    gps.publish_sample(sample());
    const auto bytes = gps.drain_to_autopilot();
    REQUIRE(bytes.empty());
}

TEST_CASE("Trimble GSOF emits STX GENOUT after 10Hz channels enabled") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::TRIMBLE;
    gps.check_backend_allocation();
    auto* tb = static_cast<GPS_Trimble*>(gps.backend());
    REQUIRE(tb != nullptr);
    tb->enable_gsof_channel(DCOL_Parser::Gsof_Msg_Record_Type::POSITION_TIME, DCOL_Parser::Output_Rate::FREQ_10_HZ);
    tb->enable_gsof_channel(DCOL_Parser::Gsof_Msg_Record_Type::LLH, DCOL_Parser::Output_Rate::FREQ_10_HZ);
    const auto d = sample();
    GpsParms parms[kSimMaxGpsSensors]{};
    parms[0].type = GpsType::TRIMBLE;
    parms[0].hertz = 5;
    parms[0].enabled = true;
    GpsWorldState world{};
    world.latitude = d.latitude;
    world.longitude = d.longitude;
    world.altitude = d.altitude;
    world.start_time_UTC = 1600000000;
    gps.update(world, parms, 1000, 1000000ULL);
    const auto bytes = gps.drain_to_autopilot();
    REQUIRE(bytes.size() > 6);
    REQUIRE(bytes[0] == 0x02);  // STX
    REQUIRE(bytes[2] == 0x40);  // GENOUT
    REQUIRE(bytes.back() == 0x03);  // ETX
}

TEST_CASE("GPS factory NONE leaves backend null") {
    gps_backend_registration();
    GPS gps(0);
    gps.parms().type = GpsType::NONE;
    gps.check_backend_allocation();
    REQUIRE(gps.backend() == nullptr);
}

TEST_CASE("sitl_gps_from_aircraft still matches Aircraft location") {
    Aircraft a;
    a.set_start_location(fwcpp::Location(-353632610, 1491652300, 58400, fwcpp::Location::AltFrame::ABSOLUTE), 0);
    const auto s = sitl_gps_from_aircraft(a);
    REQUIRE(s.lat == a.location.lat);
    REQUIRE(s.lng == a.location.lng);
}
