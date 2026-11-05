// fpv_test.md §4 runtime inference tests for quadratic_thrust_model.h.
#include "quadratic_thrust_model.h"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>

double quadratic_thrust_model_second_tu(const quadratic_thrust_model::PropulsionModel &model, double force);

namespace
{
    using namespace quadratic_thrust_model;
    constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    void require(bool condition, const char *message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    // ---- fpv_test.md §5 C reference implementation, transcribed verbatim as the oracle ----
    namespace doc
    {
        typedef struct {
            float a3;
            float a2;
            float a1;
            float v_ref;
            float u_idle;
            float v_lpf_alpha;
            float v_filtered;
        } PropulsionModel_t;

        void Propulsion_Init(PropulsionModel_t *model, float a3, float a2, float a1, float u_idle)
        {
            model->a3 = a3;
            model->a2 = a2;
            model->a1 = a1;
            model->v_ref = 22.20f;
            model->u_idle = u_idle;
            model->v_lpf_alpha = 0.02f;
            model->v_filtered = 22.20f;
        }

        float Propulsion_CalculateThrottle(PropulsionModel_t *model, float f_req, float t_celsius,
                                           float p_pa, float v_raw)
        {
            if (f_req <= 0.001f)
                return model->u_idle;
            model->v_filtered += model->v_lpf_alpha * (v_raw - model->v_filtered);
            if (model->v_filtered < 12.0f)
                model->v_filtered = 12.0f;
            const float P0 = 101325.0f;
            const float T0 = 273.15f;
            float t_kelvin = t_celsius + 273.15f;
            float f_std_target = f_req * (P0 / p_pa) * (t_kelvin / T0);
            float u_est = sqrtf(f_std_target / (model->a3 + model->a2 + model->a1 + 1e-4f));
            if (u_est > 1.0f) u_est = 1.0f;
            if (u_est < 0.01f) u_est = 0.01f;
            for (int i = 0; i < 3; i++) {
                float u2 = u_est * u_est;
                float u3 = u2 * u_est;
                float f_val = model->a3 * u3 + model->a2 * u2 + model->a1 * u_est - f_std_target;
                float f_prime = 3.0f * model->a3 * u2 + 2.0f * model->a2 * u_est + model->a1;
                if (fabsf(f_prime) < 1e-6f) break;
                float delta = f_val / f_prime;
                u_est -= delta;
                if (u_est < 0.0f) u_est = 0.0f;
                if (u_est > 1.2f) u_est = 1.2f;
                if (fabsf(delta) < 1e-4f) break;
            }
            float u_std = u_est;
            float u_cmd = u_std * (model->v_ref / model->v_filtered);
            if (u_cmd > 1.0f) {
                u_cmd = 1.0f;
            } else if (u_cmd < model->u_idle) {
                u_cmd = model->u_idle;
            }
            return u_cmd;
        }
    } // namespace doc

    PropulsionModel make(double a3, double a2, double a1, double r_eq = 0.0, double k_i = 0.0)
    {
        PropulsionModel m;
        m.a3 = a3;
        m.a2 = a2;
        m.a1 = a1;
        m.r_eq = r_eq;
        m.k_i = k_i;
        require(validateModel(m).empty(), "test model must be valid");
        require(initInverse(m) >= 0.0, "inverse init failed");
        return m;
    }

    double exactRoot(const PropulsionModel &m, double f)
    {
        double lo = 0.0, hi = kProjectMax;
        for (int i = 0; i < 200; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            (standardThrust(m, mid) < f ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    }

    void matches_document_reference_code()
    {
        const auto m = make(-15.0, 70.0, 6.0);
        for (double t_c : {-10.0, 0.0, 30.0})
            for (double p_pa : {85000.0, 101325.0})
                for (double v : {14.0, 20.0, 22.2, 25.2, 9.0})
                    for (double f : {0.0005, 2.0, 10.0, 25.0, 45.0})
                    {
                        doc::PropulsionModel_t ref;
                        doc::Propulsion_Init(&ref, -15.0f, 70.0f, 6.0f, 0.05f);
                        ref.v_lpf_alpha = 1.0f; // feed the filtered voltage directly
                        const double expected = doc::Propulsion_CalculateThrottle(
                            &ref, static_cast<float>(f), static_cast<float>(t_c), static_cast<float>(p_pa),
                            static_cast<float>(v));
                        const double actual = calculateThrottle(m, f, environmentFactor(p_pa, t_c), v);
                        require(std::abs(actual - expected) < 5e-4, "differs from fpv_test.md section 5 code");
                    }
    }

    void step1_environment()
    {
        require(environmentFactor(101325.0, 0.0) == 1.0, "standard state factor must be 1");
        const double f = environmentFactor(85000.0, 30.0);
        require(std::abs(f - (101325.0 / 85000.0) * (303.15 / 273.15)) < 1e-12, "environment formula");
        for (double p : {0.0, -1.0, kNan})
            require(std::isnan(environmentFactor(p, 20.0)), "invalid pressure must give NaN");
        require(std::isnan(environmentFactor(101325.0, -300.0)), "below absolute zero must give NaN");
        const auto m = make(-15.0, 70.0, 6.0);
        require(calculateThrottle(m, 20.0, f) == calculateThrottle(m, 20.0 * f, 1.0),
                "F_std_target must be F_req * factor");
        require(calculateThrottle(m, 20.0, kNan) == calculateThrottle(m, 20.0, 1.0),
                "invalid factor must fall back to the standard state");
    }

    void step2_newton()
    {
        for (const auto &c : {std::initializer_list<double>{-15.0, 70.0, 6.0}, {0.0, 100.0, 1e-3},
                              {0.0, 0.0, 90.0}, {40.0, 60.0, 2.0}, {-15.85, 79.77, 0.99}})
        {
            const double *a = c.begin();
            const auto m = make(a[0], a[1], a[2]);
            for (int i = 0; i <= 20; ++i)
            {
                const double u = 0.05 + 0.0475 * i;
                const double f = standardThrust(m, u);
                require(std::abs(solveStandardThrottle(m, f) - exactRoot(m, f)) < kNewtonTolerance,
                        "Newton-Raphson must reach 1e-5");
            }
        }
        PropulsionModel quadratic;
        quadratic.a2 = 100.0;
        quadratic.a1 = 1e-9;
        require(initInverse(quadratic) < 1e-6, "inverse cubic in sqrt(F) must be exact for F = k u^2");
        PropulsionModel linear;
        linear.a1 = 90.0;
        require(initInverse(linear) < 1e-9, "inverse cubic in sqrt(F) must be exact for F = k u");
        auto general = make(-15.0, 70.0, 6.0);
        require(initInverse(general) < 2e-3, "inverse cubic initial guess quality");
        PropulsionModel invalid;
        invalid.a1 = -1.0;
        require(initInverse(invalid) < 0.0 && !invalid.inverse_valid, "invalid model must not init");
    }

    void step3_compensation()
    {
        const auto m = make(-15.0, 70.0, 6.0);
        const double f = 20.0;
        const double u_std = solveStandardThrottle(m, f);
        require(std::abs(calculateThrottle(m, f, 1.0, 20.0) - u_std * 22.2 / 20.0) < 1e-12, "first-order");
        require(calculateThrottle(m, f, 1.0, kNan) == calculateThrottle(m, f, 1.0), "no voltage = no comp");
        require(std::abs(compensate(m, u_std, f, 5.0) - u_std * 22.2 / 12.0) < 1e-12, "12 V floor");
        const auto full = make(-15.0, 70.0, 6.0, 0.03, 1.5);
        const double current = 40.0, voltage = 21.0;
        const double expected = (u_std * 22.2 + (current - 1.5 * f) * 0.03) / voltage;
        require(std::abs(calculateThrottle(full, f, 1.0, voltage, current) - expected) < 1e-12,
                "full form with I_meas, I_std and R_eq");
        require(std::abs(calculateThrottle(full, f, 1.0, voltage, kNan) - u_std * 22.2 / voltage) < 1e-12,
                "missing current must fall back to first order");
    }

    void step4_clip_and_guards()
    {
        const auto m = make(-15.0, 70.0, 6.0);
        for (double f : {0.0, 0.001, -3.0, kNan})
            require(calculateThrottle(m, f) == m.u_idle, "zero/negative/NaN force -> u_idle");
        require(calculateThrottle(m, 1e6) == 1.0, "clip at 1.0");
        require(calculateThrottle(m, 0.01) == m.u_idle, "clip at u_idle");
        require(calculateThrottle(PropulsionModel{}, 10.0) == PropulsionModel{}.u_idle, "uncalibrated -> u_idle");
    }

    void validation()
    {
        PropulsionModel m;
        m.a3 = -100.0;
        m.a2 = 50.0;
        m.a1 = 10.0;
        require(!validateModel(m).empty(), "dF/du < 0 at u=1 accepted");
        m = PropulsionModel{};
        m.a2 = 50.0;
        require(!validateModel(m).empty(), "a1 = 0 accepted");
        m.a1 = 1.0;
        require(validateModel(m).empty(), "valid model rejected");
        auto bad = m;
        bad.u_idle = 1.0;
        require(!validateModel(bad).empty(), "u_idle >= 1 accepted");
        bad = m;
        bad.r_eq = -0.1;
        require(!validateModel(bad).empty(), "negative r_eq accepted");
        bad = m;
        bad.v_min = bad.v_ref;
        require(!validateModel(bad).empty(), "v_min >= v_ref accepted");
        bad = m;
        bad.a3 = kNan;
        require(!validateModel(bad).empty(), "NaN accepted");
    }

    void low_pass_and_duty_mapping()
    {
        LowPassFilter lpf{3.2, 22.2};
        lpf.update(21.2, 0.001);
        require(std::abs((22.2 - lpf.value) - 0.02) < 2e-4, "doc alpha=0.02@1kHz is ~3.2 Hz");
        LowPassFilter step{3.0, 22.2};
        const double tau = 1.0 / (2.0 * M_PI * 3.0);
        step.update(20.0, tau);
        require(std::abs((22.2 - step.value) / 2.2 - (1.0 - std::exp(-1.0))) < 1e-12, "63% at one tau");
        LowPassFilter fresh{3.0};
        require(fresh.update(21.0, 0.1) == 21.0 && fresh.update(kNan, 0.1) == 21.0, "init/NaN handling");
        require(collectiveToDuty(0.0, 0.055) == 0.055 && collectiveToDuty(1.0, 0.055) == 1.0, "duty map");
        require(dutyToCollective(0.05, 0.055) == 0.0, "duty below idle -> collective 0");
        require(std::abs(dutyToCollective(collectiveToDuty(0.37, 0.055), 0.055) - 0.37) < 1e-12, "round trip");
        // u_idle (§4 step 4 floor) is the FC motor_idle duty: it maps exactly to collective 0
        const auto m = make(-15.0, 70.0, 6.0);
        require(dutyToCollective(calculateThrottle(m, 0.0), m.u_idle) == 0.0, "u_idle must be collective 0");
    }

    void motor_idle_from_cli()
    {
        require(idleDutyFromCli("motor_idle = 550") == 0.055, "default BF motor_idle");
        require(idleDutyFromCli("motor_idle = 0\r\n") == 0.0, "zero idle with line ending");
        require(idleDutyFromCli("  motor_idle=2000 ") == 0.2, "upper bound");
        for (const char *bad : {"motor_idle = 2001", "motor_idle = -5", "motor_idle = 5.5", "motor_idle =",
                                "dshot_idle_value = 550", "NOT FOUND", "", "motor_idle = 99999999"})
            require(std::isnan(idleDutyFromCli(bad)), "malformed motor_idle reply accepted");
    }

    // Synthetic plant (test/propulsion_plant.py): F = K·ρ·RPM², K_e·RPM = u·V − I·R_eq,
    // I = k·ρ·RPM², V = V_oc − I·R_bat.
    struct Plant
    {
        double rpm, current, voltage, thrust;
    };

    Plant plant(double u, double rho, double v_oc)
    {
        constexpr double ke = 7.4e-4, k_aero = 9.6e-8, k_load = 1.92e-7, r_eq = 0.03, r_bat = 0.02;
        const double a = (u * r_bat + r_eq) * k_load * rho;
        const double rpm = (-ke + std::sqrt(ke * ke + 4.0 * a * u * v_oc)) / (2.0 * a);
        const double current = k_load * rho * rpm * rpm;
        return {rpm, current, v_oc - current * r_bat, k_aero * rho * rpm * rpm};
    }

    void closed_loop_on_plant()
    {
        // Coefficients from the fpv_test.md section 3 pipeline (propulsion_model.calibrate) on
        // 20 bench points of the same plant at 85 kPa / 30 degC (see thrust_test_fit_test.py).
        const auto m = make(-15.852827148889013, 79.76953283285637, 0.9908208740299711,
                            0.029999999999998896, 1.507596558355762);
        for (const auto &s : {std::initializer_list<double>{101325.0, 0.0, 22.2}, {85000.0, 30.0, 24.0},
                              {95000.0, -10.0, 21.0}})
        {
            const double *c = s.begin();
            const double env = environmentFactor(c[0], c[1]);
            for (double f_req : {10.0, 25.0, 45.0})
            {
                double voltage = c[2], current = kNan;
                Plant state{};
                for (int i = 0; i < 40; ++i)
                {
                    state = plant(calculateThrottle(m, f_req, env, voltage, current), 1.0 / env, c[2]);
                    voltage = state.voltage;
                    current = state.current;
                }
                require(std::abs(state.thrust / f_req - 1.0) < 0.01, "closed-loop thrust error > 1%");
            }
        }
    }
} // namespace

int main()
{
    matches_document_reference_code();
    step1_environment();
    step2_newton();
    step3_compensation();
    step4_clip_and_guards();
    validation();
    low_pass_and_duty_mapping();
    motor_idle_from_cli();
    closed_loop_on_plant();
    const auto m = make(-15.0, 70.0, 6.0);
    require(quadratic_thrust_model_second_tu(m, 20.0) == calculateThrottle(m, 20.0),
            "second translation unit disagrees");
    return 0;
}
