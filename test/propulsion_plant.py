"""Synthetic quadrotor propulsion plant consistent with fpv_test.md section 1 (test helper).

Total thrust F = K_aero·(ρ/ρ0)·RPM_rms²; back-EMF K_e·RPM = u·V − I·R_eq; aerodynamic
current I = k_load·(ρ/ρ0)·RPM²; the measured bus voltage sags across the pack
resistance, V = V_oc − I·R_bat. Solving the three equations for RPM is a quadratic.

AirSim mode (linear=True) mimics the simulator: thrust linear in the rotor duty, no sag,
no RPM feedback, no current sensor.
"""

import math

KE = 7.4e-4        # V per RPM (whole-craft back-EMF constant, K_v* ≈ 1351 rpm/V)
K_AERO = 9.6e-8    # N per RPM² at standard density (four rotors)
K_LOAD = 1.92e-7   # A per RPM² at standard density
R_EQ = 0.03        # Ω, bus + ESC + windings
V_OC = 24.0        # V, pack open-circuit voltage
R_BAT = 0.02       # Ω, pack internal resistance (sags the measured V)
V_REF = 22.2
AIRSIM_A1 = 90.0   # N per unit duty (AirSim rotor model is thrust-linear)
MOTOR_SPREAD = (0.98, 1.0, 1.01, 1.01)


def density_ratio(pressure_pa, temperature_c):
    return (pressure_pa / 101325.0) * (273.15 / (temperature_c + 273.15))


def state(u, rho_ratio=1.0, v_oc=V_OC, r_bat=R_BAT):
    """Steady state at DShot duty u: dict(rpm, I, V, F)."""
    if u <= 0.0:
        return {'rpm': 0.0, 'I': 0.0, 'V': v_oc, 'F': 0.0}
    a = (u * r_bat + R_EQ) * K_LOAD * rho_ratio
    rpm = (-KE + math.sqrt(KE * KE + 4.0 * a * u * v_oc)) / (2.0 * a)
    current = K_LOAD * rho_ratio * rpm * rpm
    return {'rpm': rpm, 'I': current, 'V': v_oc - current * r_bat, 'F': K_AERO * rho_ratio * rpm * rpm}


def standard_thrust(u):
    """Ground-truth thrust in the fpv_test.md standard state (ρ0, constant V_ref)."""
    return state(u, 1.0, V_REF, 0.0)['F']


def motor_rpms(rpm_rms):
    """Four motor speeds whose RMS equals rpm_rms."""
    scale = math.sqrt(sum(s * s for s in MOTOR_SPREAD) / len(MOTOR_SPREAD))
    return [rpm_rms * s / scale for s in MOTOR_SPREAD]


def bench_point(u, pressure_pa, temperature_c, v_oc=V_OC):
    """One fpv_test.md section 2 data point measured on the synthetic bench."""
    s = state(u, density_ratio(pressure_pa, temperature_c), v_oc)
    return {'u': u, 'V': s['V'], 'I': s['I'], 'rpm': motor_rpms(s['rpm']), 'F': s['F'],
            'P': pressure_pa, 't': temperature_c}


def closed_loop_thrust(model, f_req, pressure_pa, temperature_c, v_oc, current_feedback=True,
                       iterations=40):
    """Apply model.throttle() to the plant until V/I feedback settles; return realized thrust."""
    import propulsion_model as pm
    rho = density_ratio(pressure_pa, temperature_c)
    env = pm.environment_factor(pressure_pa, temperature_c)
    voltage, current = v_oc, float('nan')
    for _ in range(iterations):
        u = model.throttle(f_req, env, voltage, current if current_feedback else float('nan'))
        s = state(u, rho, v_oc)
        voltage, current = s['V'], s['I']
    return s['F'], u
