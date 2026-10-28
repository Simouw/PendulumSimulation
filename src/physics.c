#include "physics.h"
#include <math.h>

void init_system_params(SystemParams *params) {
    params->M = DEFAULT_M;
    params->m = DEFAULT_m;
    params->L = DEFAULT_L;
    params->g = DEFAULT_G;
    params->b = DEFAULT_B;
    params->c = DEFAULT_C;
    params->F_max = DEFAULT_F_MAX;
    params->x_max = DEFAULT_X_MAX;
}

void init_system_state(SystemState *state, double x, double theta) {
    state->x = x;
    state->x_dot = 0.0;
    state->theta = wrap_angle_pi(theta);
    state->theta_dot = 0.0;
}

double wrap_angle_pi(double angle) {
    double a = fmod(angle + M_PI, 2.0 * M_PI);
    if (a <= 0.0) {
        a += 2.0 * M_PI;
    }
    return a - M_PI;
}

void compute_derivatives(const SystemState *state,
                         const SystemParams *params,
                         double force,
                         StateDerivative *deriv) {
    // 1. First-order kinematic relations (dx/dt = v, dtheta/dt = omega)
    deriv->x_dot = state->x_dot;
    deriv->theta_dot = state->theta_dot;

    // 2. Trigonometric terms and useful products
    double s = sin(state->theta);
    double c = cos(state->theta);
    double theta_dot_sq = state->theta_dot * state->theta_dot;

    // 3. Coupling denominator D(theta) = M + m * sin^2(theta)
    double D = params->M + params->m * s * s;

    // 4. Cart linear acceleration (x_ddot)
    double num_x = force 
                   - params->b * state->x_dot 
                   + params->m * params->L * theta_dot_sq * s 
                   - params->m * params->g * s * c 
                   + (params->c * state->theta_dot * c) / params->L;

    deriv->x_ddot = num_x / D;

    // 5. Pendulum angular acceleration (theta_ddot)
    double num_theta = params->g * s 
                       - deriv->x_ddot * c 
                       - (params->c * state->theta_dot) / (params->m * params->L);

    deriv->theta_ddot = num_theta / params->L;
}

double compute_total_energy(const SystemState *state, const SystemParams *params) {
    // Cart kinetic energy: 0.5 * M * x_dot^2
    double Ec_cart = 0.5 * params->M * state->x_dot * state->x_dot;

    // Bob velocity squared: v_bob^2 = x_dot^2 + L^2 * theta_dot^2 + 2 * L * x_dot * theta_dot * cos(theta)
    double v_bob_sq = state->x_dot * state->x_dot 
                    + params->L * params->L * state->theta_dot * state->theta_dot 
                    + 2.0 * params->L * state->x_dot * state->theta_dot * cos(state->theta);

    // Bob kinetic energy: 0.5 * m * v_bob^2
    double Ec_pendulum = 0.5 * params->m * v_bob_sq;

    // Bob potential energy: m * g * y_bob where y_bob = L * cos(theta) (upright = +L)
    double Ep = params->m * params->g * params->L * cos(state->theta);

    return Ec_cart + Ec_pendulum + Ep;
}

double compute_pendulum_energy(const SystemState *state, const SystemParams *params) {
    // Pendulum mechanical energy relative to upright equilibrium (E = 0 at theta = 0, theta_dot = 0)
    // E_rel = 0.5 * m * L^2 * theta_dot^2 + m * g * L * (cos(theta) - 1.0)
    double Ec_rot = 0.5 * params->m * params->L * params->L * state->theta_dot * state->theta_dot;
    double Ep_rel = params->m * params->g * params->L * (cos(state->theta) - 1.0);

    return Ec_rot + Ep_rel;
}
