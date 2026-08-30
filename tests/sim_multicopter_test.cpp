// CCP-045: SIM_Multicopter Frame/Motor mixing + rigid-body plant tests.
#include <cmath>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/sim/sim_frame.hpp>
#include <fwcpp/sim/sim_motor.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

using fwcpp::sim::Frame;
using fwcpp::sim::SimMulticopter;
using fwcpp::sim::SitlInput;
using fwcpp::sim::kGravityMss;
using fwcpp::sim::kMotorsYawFactorCcw;
using fwcpp::sim::kMotorsYawFactorCw;

TEST_CASE("create_frame matches original SITL name table", "[copter][sitl][ccp-045][frame]") {
    auto x = Frame::create_frame("x");
    REQUIRE(x.valid());
    REQUIRE(x.num_motors == 4);
    REQUIRE(std::string(x.name) == "x");

    auto plus = Frame::create_frame("quad");
    REQUIRE(plus.valid());
    REQUIRE(std::string(plus.name) == "quad");
    REQUIRE(plus.num_motors == 4);

    auto hexa = Frame::create_frame("hexa");
    REQUIRE(hexa.valid());
    REQUIRE(hexa.num_motors == 6);

    auto octa = Frame::create_frame("octa-quad");
    REQUIRE(octa.valid());
    REQUIRE(octa.num_motors == 8);

    auto missing = Frame::create_frame("not-a-frame");
    REQUIRE_FALSE(missing.valid());
}

TEST_CASE("quad X motor angles and yaw factors match SIM_Frame", "[copter][sitl][ccp-045][frame]") {
    auto x = Frame::create_frame("x");
    REQUIRE(x.motors[0].angle == Catch::Approx(45.0f));
    REQUIRE(x.motors[0].yaw_factor == Catch::Approx(kMotorsYawFactorCcw));
    REQUIRE(x.motors[1].angle == Catch::Approx(-135.0f));
    REQUIRE(x.motors[1].yaw_factor == Catch::Approx(kMotorsYawFactorCcw));
    REQUIRE(x.motors[2].angle == Catch::Approx(-45.0f));
    REQUIRE(x.motors[2].yaw_factor == Catch::Approx(kMotorsYawFactorCw));
    REQUIRE(x.motors[3].angle == Catch::Approx(135.0f));
    REQUIRE(x.motors[3].yaw_factor == Catch::Approx(kMotorsYawFactorCw));
}

TEST_CASE("equal hover command produces near-1g body-z thrust", "[copter][sitl][ccp-045][aero]") {
    SimMulticopter copter{"x"};
    copter.position.z = -10.0f;  // airborne so ground clamp does not apply
    copter.velocity_ef = {};
    SitlInput in;
    copter.home_alt_amsl_m = copter.frame().get_model().refAlt;
    copter.set_equal_command(in, copter.hover_command());
    fwcpp::math::Vector3f rot;
    fwcpp::math::Vector3f body;
    copter.calculate_forces(in, rot, body);
    // Hover command solves expo so velocity_out == hover_velocity_out.
    REQUIRE(body.z == Catch::Approx(-kGravityMss).margin(1.5f));
    REQUIRE(std::fabs(body.x) < 0.5f);
    REQUIRE(std::fabs(body.y) < 0.5f);
    REQUIRE(std::fabs(rot.x) < 0.5f);
    REQUIRE(std::fabs(rot.y) < 0.5f);
}

TEST_CASE("zero PWM stays on the ground", "[copter][sitl][ccp-045][aero]") {
    SimMulticopter copter{"x"};
    REQUIRE(copter.on_ground());
    SitlInput in;  // all zeros
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 400; ++i) {
        copter.update(in, kDt);
    }
    REQUIRE(copter.on_ground());
    REQUIRE((-copter.position.z) == Catch::Approx(0.0f).margin(0.01f));
}

TEST_CASE("climb command leaves the ground", "[copter][sitl][ccp-045][aero]") {
    SimMulticopter copter{"x"};
    SitlInput in;
    copter.set_equal_command(in, 0.70f);
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 1200; ++i) {  // 3 s
        copter.update(in, kDt);
    }
    REQUIRE((-copter.position.z) > 2.0f);
    REQUIRE_FALSE(copter.on_ground());
}

TEST_CASE("differential thrust on left motors produces positive roll",
          "[copter][sitl][ccp-045][mixing]") {
    SimMulticopter copter{"x"};
    copter.position.z = -10.0f;
    SitlInput in;
    const std::uint16_t high = copter.command_to_pwm(0.70f);
    const std::uint16_t low = copter.command_to_pwm(0.20f);
    // Quad X: MOT_1 servo0 +45 (right-front), MOT_2 servo1 -135 (left-rear),
    // MOT_3 servo2 -45 (left-front), MOT_4 servo3 +135 (right-rear).
    in.servos[0] = low;   // right-front
    in.servos[1] = high;  // left-rear
    in.servos[2] = high;  // left-front
    in.servos[3] = low;   // right-rear
    fwcpp::math::Vector3f rot;
    fwcpp::math::Vector3f body;
    copter.calculate_forces(in, rot, body);
    REQUIRE(rot.x > 1.0f);  // +roll, right wing down
    REQUIRE(std::fabs(rot.y) < std::fabs(rot.x));
}

TEST_CASE("differential thrust on rear motors produces positive pitch",
          "[copter][sitl][ccp-045][mixing]") {
    SimMulticopter copter{"x"};
    copter.position.z = -10.0f;
    SitlInput in;
    const std::uint16_t high = copter.command_to_pwm(0.70f);
    const std::uint16_t low = copter.command_to_pwm(0.20f);
    // More front (MOT_1 +45 x>0, MOT_3 -45 x>0) than rear -> +pitch (nose up).
    // torque.y = T * position.x from position % (0,0,-T).
    in.servos[0] = high;
    in.servos[1] = low;
    in.servos[2] = high;
    in.servos[3] = low;
    fwcpp::math::Vector3f rot;
    fwcpp::math::Vector3f body;
    copter.calculate_forces(in, rot, body);
    REQUIRE(rot.y > 1.0f);
    REQUIRE(std::fabs(rot.x) < std::fabs(rot.y));
}

TEST_CASE("CCW vs CW command imbalance produces yaw", "[copter][sitl][ccp-045][mixing]") {
    SimMulticopter copter{"x"};
    copter.position.z = -10.0f;
    SitlInput in;
    const std::uint16_t high = copter.command_to_pwm(0.70f);
    const std::uint16_t low = copter.command_to_pwm(0.20f);
    // MOT_1 and MOT_2 are CCW (yaw_factor +1); MOT_3 and MOT_4 are CW (-1).
    // More CCW command -> +yaw (NED down).
    in.servos[0] = high;
    in.servos[1] = high;
    in.servos[2] = low;
    in.servos[3] = low;
    fwcpp::math::Vector3f rot;
    fwcpp::math::Vector3f body;
    copter.calculate_forces(in, rot, body);
    REQUIRE(std::fabs(rot.z) > 0.5f);
}

TEST_CASE("integrated differential roll actually banks the rigid body",
          "[copter][sitl][ccp-045][mixing]") {
    SimMulticopter copter{"x"};
    copter.position.z = -20.0f;
    SitlInput in;
    const std::uint16_t high = copter.command_to_pwm(0.65f);
    const std::uint16_t low = copter.command_to_pwm(0.25f);
    in.servos[0] = low;
    in.servos[1] = high;
    in.servos[2] = high;
    in.servos[3] = low;
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 80; ++i) {  // 0.2 s
        copter.update(in, kDt);
    }
    float r = 0, p = 0, y = 0;
    copter.dcm.to_euler(&r, &p, &y);
    REQUIRE(r > 0.05f);
}

TEST_CASE("plus vs X frames have different motor angles", "[copter][sitl][ccp-045][frame]") {
    auto plus = Frame::create_frame("+");
    auto x = Frame::create_frame("x");
    REQUIRE(plus.motors[0].angle == Catch::Approx(90.0f));
    REQUIRE(x.motors[0].angle == Catch::Approx(45.0f));
}
