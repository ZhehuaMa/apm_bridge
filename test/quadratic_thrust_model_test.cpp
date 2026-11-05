#include "quadratic_thrust_model.h"

#include <algorithm>
#include <initializer_list>
#include <limits>

double quadratic_thrust_model_second_tu(const quadratic_thrust_model::MotorParams &params, double force);

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
    void safety_guards()
    {
        using namespace quadratic_thrust_model;
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();

        // Shipped hardware model: valid, monotonic everywhere (no peak clamp).
        auto p = parameters();
        require(validateMotorParams(p, true).empty(), "shipped hardware model rejected");
        require(p.x_peak == 0.0, "monotonic model must not be truncated");

        // Negative / non-finite force requests saturate at zero force, never extrapolate.
        require(forceToThrust(p, -50.0, 1.0) == forceToThrust(p, 0.0, 1.0),
                "negative force must map to the zero-force throttle");
        require(forceToThrust(p, nan, 1.0) == forceToThrust(p, 0.0, 1.0),
                "non-finite force must map to the zero-force throttle");
        require(std::isfinite(forceToThrust(p, nan, 1.0, 22.2)), "NaN force produced NaN throttle");

        // Voltage floor (fpv_test.md section 4): a sagging/bogus reading below volt_min is
        // compensated as volt_min, so throttle is not amplified without bound.
        for (bool linear_voltage : {false, true}) {
            auto q = parameters();
            if (linear_voltage)
                q.vbat_a = 0.0;
            q.volt_min = 18.0;
            const double force = 8.0;
            const double at_floor = forceToThrust(q, force, 1.0, 18.0);
            require(forceToThrust(q, force, 1.0, 2.0) == at_floor, "voltage below floor not clamped");
            auto unclamped = q;
            unclamped.volt_min = 0.0;
            require(forceToThrust(q, force, 1.0, 22.2) == forceToThrust(unclamped, force, 1.0, 22.2),
                    "voltage floor changed nominal-voltage output");
            require(forceToThrust(unclamped, force, 1.0, 2.0) > at_floor,
                    "test premise: an unclamped low reading amplifies throttle");
            require(forceToThrust(q, force, 1.0, nan) == forceToThrust(q, force, 1.0),
                    "non-finite voltage must disable compensation");
            const double throttle = forceToThrust(q, force, 1.0, 2.0);
            require(std::abs(thrustToForce(q, throttle, 1.0, 2.0) - force) < 0.02,
                    "voltage floor broke forward/inverse symmetry");
        }

        // A turning cubic (A < 0) peaks at x = sqrt(C / 3|A|): requests past the peak must
        // saturate instead of collapsing to zero throttle.
        quadratic_thrust_model::MotorParams turning{};
        turning.A = -2.0e-5;
        turning.C = 0.06;
        turning.n_motors = 1;
        turning.thrust_limit = 0.9;
        turning.volt_max = 25.2;
        turning.volt_ref = 22.2;
        require(validateMotorParams(turning, false).empty(), "valid turning model rejected");
        const double peak = std::sqrt(turning.C / (3.0 * 2.0e-5));
        require(std::abs(turning.x_peak - peak) < 1e-9, "x_peak is not the polynomial maximum");
        double previous = 0.0;
        for (double force = 0.0; force <= 200.0; force += 0.5) {
            const double throttle = forceToThrust(turning, force, 1.0);
            require(throttle + 1e-12 >= previous, "throttle must be non-decreasing in requested force");
            previous = throttle;
        }
        require(forceToThrust(turning, 200.0, 1.0) == turning.thrust_limit,
                "force beyond the curve peak must saturate at thrust_limit");

        // Rejections.
        auto bad = turning;
        bad.A = -2.0e-4;  // peaks at throttle 0.4 < thrust_limit
        require(!validateMotorParams(bad, false).empty(), "non-monotonic throttle domain accepted");
        bad = parameters();
        bad.C = 0.0;
        require(!validateMotorParams(bad, false).empty(), "C <= 0 accepted");
        bad = parameters();
        bad.D = nan;
        require(!validateMotorParams(bad, false).empty(), "non-finite coefficient accepted");
        bad = parameters();
        bad.thrust_limit = 1.5;
        require(!validateMotorParams(bad, false).empty(), "thrust_limit > 1 accepted");
        bad = parameters();
        bad.volt_max = 0.0;
        require(!validateMotorParams(bad, true).empty(), "zero volt_max accepted with voltage compensation");
        require(validateMotorParams(bad, false).empty(), "volt_max must only matter with voltage compensation");
        bad = parameters();
        bad.vbat_b = 1.0;  // a*V + b turns negative inside [0, volt_max]
        require(!validateMotorParams(bad, true).empty(), "vbat curve crossing zero accepted");
    }
}

int main()
{
    using namespace quadratic_thrust_model;
    density_and_voltage();
    safety_guards();
    require(quadratic_thrust_model_second_tu(parameters(), 5.0) == forceToThrust(parameters(), 5.0, 1.0),
            "second translation unit disagrees");
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
