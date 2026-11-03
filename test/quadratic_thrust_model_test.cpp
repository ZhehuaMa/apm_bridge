#include "quadratic_thrust_model.h"

#include <algorithm>
#include <initializer_list>
#include <limits>

namespace
{
    void require(bool condition, const char *message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    quadratic_thrust_model::MotorParams parameters()
    {
        quadratic_thrust_model::MotorParams p{};
        p.A = 2.26453885e-06;
        p.B = -0.00026519864;
        p.C = 0.0179075134;
        p.D = 0.0119440187;
        p.n_motors = 1;
        p.thrust_limit = 0.9;
        p.volt_max = 25.2;
        p.vbat_a = -0.0776732;
        p.vbat_b = 2.7346374;
        p.volt_ref = 22.2;
        return p;
    }

    double curve(const quadratic_thrust_model::MotorParams &p, double force)
    {
        return ((p.A * force + p.B) * force + p.C) * force + p.D;
    }

    void density_and_voltage()
    {
        using namespace quadratic_thrust_model;
        for (bool fitted : {false, true}) {
            for (bool linear_voltage : {false, true}) {
                for (int motors : {1, 4}) {
                    auto p = parameters();
                    p.n_motors = motors;
                    if (linear_voltage)
                        p.vbat_a = 0.0;
                    if (fitted)
                        require(initInverseFit(p) >= 0.0, "inverse fit failed");
                    for (double spin : {0.0, 0.3}) {
                        p.spin_k = spin;
                        for (double voltage : {-1.0, 22.2, 25.2}) {
                            for (double kp : {0.8, 1.0, 1.2}) {
                                for (double per_motor_force : {2.0, 10.0, 20.0}) {
                                    const double force = per_motor_force * motors;
                                    double reference = per_motor_force / kp;
                                    if (voltage > 0.0 && !linear_voltage)
                                        reference *= vbatScale(p, voltage);
                                    double expected = curve(p, reference);
                                    if (voltage > 0.0 && linear_voltage)
                                        expected *= p.volt_max / voltage;
                                    expected = (1.0 - spin) * expected + spin * expected * expected;
                                    const double throttle = forceToThrust(p, force, kp, voltage);
                                    require(std::abs(throttle - expected) < 1e-12,
                                            "density must scale force before polynomial and spin correction");
                                    require(std::abs(thrustToForce(p, throttle, kp, voltage) - force) < 0.02,
                                            "density/voltage/spin inverse mismatch");
                                    const double reference_force = thrustToForce(p, throttle, 1.0, voltage);
                                    require(std::abs(reference_force * kp - force) < 0.02,
                                            "output does not realize requested physical force at this density");
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

int main()
{
    using namespace quadratic_thrust_model;
    density_and_voltage();
    auto p = parameters();
    const double force = 1.067 * 9.81;
    const double kp = 273.15 / 293.15;
    require(std::abs(forceToThrust(p, force, kp, 25.2) - 0.148366) < 1e-6,
            "hardware hover density regression");
    require(forceToThrust(p, force, 0.8) > forceToThrust(p, force, 1.0),
            "thin air must increase throttle");
    require(forceToThrust(p, force, 1.2) < forceToThrust(p, force, 1.0),
            "dense air must decrease throttle");
    require(forceToThrust(p, 200.0, 0.8) == p.thrust_limit, "thrust limit not preserved");

    p.A = p.B = 0.0;
    p.C = 0.011891350984191815;
    p.D = -0.059949564628813956;
    require(std::abs(forceToThrust(p, 2.0 * 9.80665, 1.0) - 0.17327906962943534) < 1e-12,
            "uncompensated AirSim hover changed");
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        for (bool inverse : {false, true}) {
            bool rejected = false;
            try {
                if (inverse)
                    thrustToForce(p, 0.2, invalid);
                else
                    forceToThrust(p, force, invalid);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "invalid air density ratio accepted");
        }
    }
}
