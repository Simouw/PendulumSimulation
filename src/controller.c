#include "controller.h"
#include <math.h>

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

static double sign_with_zero(double v) {
    if (v > 0.0) return 1.0;
    if (v < 0.0) return -1.0;
    return 0.0;
}

void init_controller(Controller *ctrl, ControlMode initial_mode) {
    /* K_x and K_vx are positive on purpose: a restoring term on the cart fights
       the pendulum regulation and leaves it parked at a rail end. */
    (*ctrl).active_mode = initial_mode;
    (*ctrl).K_x = 5.0;
    (*ctrl).K_vx = 11.25;
    (*ctrl).K_theta = 78.0;
    (*ctrl).K_vtheta = 45.75;

    (*ctrl).K_i = 0.0;

    (*ctrl).k_E = 45.0;
    (*ctrl).theta_threshold = 0.35;

    (*ctrl).x_ref = 0.0;
    (*ctrl).error_integral = 0.0;
}

double saturate_value(double val, double min, double max) {
    return CLAMP(val, min, max);
}

void update_control_mode(Controller *ctrl, const SystemState *state) {
    /* CTRL_OFF is a latched state: the user must leave it explicitly. */
    if ((*ctrl).active_mode == CTRL_OFF) {
        return;
    }

    /* Switch on |theta| only: each control law is valid in its own regime. */
    if (fabs((*state).theta) <= (*ctrl).theta_threshold) {
        (*ctrl).active_mode = CTRL_STABILIZATION;
    } else {
        (*ctrl).active_mode = CTRL_SWING_UP;
    }
}

double compute_swing_up_force(const Controller *ctrl,
                              const SystemState *state,
                              const SystemParams *params) {
    /* E_relu is ALREADY relative to upright, so E_target = 0 and the guide's
     * (E_rel - E_target) term collapses to E_rel.
     * compute_total_energy() would be WRONG here (cart kinetic + non-relative potential). */
    double E_relu = compute_pendulum_energy(state, params);
    double direction = sign_with_zero(((*state).theta_dot) * cos((*state).theta));

    return (*ctrl).k_E * E_relu * direction;
}

double compute_stabilization_force(Controller *ctrl,
                                   const SystemState *state,
                                   double dt) {
    double position_error = (*state).x - (*ctrl).x_ref;

    (*ctrl).error_integral += position_error * dt;

    return (*ctrl).K_x * position_error
         + (*ctrl).K_vx * (*state).x_dot
         + (*ctrl).K_theta * (*state).theta
         + (*ctrl).K_vtheta * (*state).theta_dot
         + (*ctrl).K_i * (*ctrl).error_integral;
}

double compute_motor_command(Controller *ctrl,
                              const SystemState *state,
                              const SystemParams *params,
                              double dt) {
    update_control_mode(ctrl, state);

    switch ((*ctrl).active_mode) {
        case CTRL_OFF:
            return 0.0;

        case CTRL_SWING_UP:
            return saturate_value(compute_swing_up_force(ctrl, state, params),
                                  -(*params).F_max, (*params).F_max);

        case CTRL_STABILIZATION:
            return saturate_value(compute_stabilization_force(ctrl, state, dt),
                                  -(*params).F_max, (*params).F_max);

        default:
            return 0.0;
    }
}