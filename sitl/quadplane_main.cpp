#include <cmath>
// VCP-010: QuadPlane sitl smoke on the real SIM_QuadPlane plant.
#include <cstdio>
#include <fwcpp/sim/sim_quadplane.hpp>

int main() {
    fwcpp::sim::SimQuadPlane qp("quadplane");
    fwcpp::sim::SitlInput in{};
    qp.frame().set_equal_command(in, qp.frame().hover_command());
    in.servos[2] = 1500;
    qp.position.z = -1.0f;
    for (int i = 0; i < 400; ++i) {
        qp.update(in, 0.0025f);
    }
    std::printf("quadplane smoke alt=%.2f V=%.2f gyro.z=%.4f\n", -qp.position.z, qp.battery_voltage, qp.gyro.z);
    return std::isfinite(-qp.position.z) ? 0 : 1;
}
