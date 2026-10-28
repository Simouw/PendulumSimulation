#ifndef PHYSICS_H
#define PHYSICS_H

#include "config.h"
#include <stdbool.h>

/**
 * @brief Continuous state vector Y(t) of the cart-pole system.
 */
typedef struct {
    double x;         /**< Cart horizontal position on the rail (m) */
    double x_dot;     /**< Cart linear velocity dx/dt (m/s) */
    double theta;     /**< Pendulum angle from upright vertical (rad, in ]-pi, pi]) */
    double theta_dot; /**< Pendulum angular velocity dtheta/dt (rad/s) */
} SystemState;

/**
 * @brief State derivative vector dY/dt = (dx/dt, dv/dt, dtheta/dt, domega/dt).
 */
typedef struct {
    double x_dot;      /**< Derivative of position = velocity (m/s) */
    double x_ddot;     /**< Linear acceleration of the cart (m/s^2) */
    double theta_dot;  /**< Derivative of angle = angular velocity (rad/s) */
    double theta_ddot; /**< Angular acceleration of the pendulum (rad/s^2) */
} StateDerivative;

/**
 * @brief Invariants and physical parameters of the cart-pole system.
 */
typedef struct {
    double M;     /**< Cart mass (kg, > 0) */
    double m;     /**< Pendulum bob mass (kg, > 0) */
    double L;     /**< Rigid rod length (m, > 0) */
    double g;     /**< Gravitational acceleration (m/s^2) */
    double b;     /**< Cart viscous friction coefficient (N.s/m, >= 0) */
    double c;     /**< Pivot damping coefficient (N.m.s/rad, >= 0) */
    double F_max; /**< Maximum motor force saturation (N, > 0) */
    double x_max; /**< Rail half-length travel limit (m, > 0) */
} SystemParams;

/* ========================================================================= */
/* Physics Module Function Signatures                                        */
/* ========================================================================= */

/**
 * @brief Initialize physical parameters with default values.
 * @param params Pointer to SystemParams structure to initialize.
 */
void init_system_params(SystemParams *params);

/**
 * @brief Initialize kinematic state at t = 0.
 * @param state Pointer to SystemState structure.
 * @param x Initial position of the cart (m).
 * @param theta Initial angle of the pendulum (rad).
 */
void init_system_state(SystemState *state, double x, double theta);

/**
 * @brief Wrap an angle into the principal interval ]-pi, +pi].
 * @param angle Raw angle in radians.
 * @return Wrapped angle in ]-pi, +pi].
 */
double wrap_angle_pi(double angle);

/**
 * @brief Evaluate coupled Euler-Lagrange equations of motion to compute state derivatives dY/dt.
 * @param state Constant pointer to the current kinematic state Y(t).
 * @param params Constant pointer to the physical parameters.
 * @param force External horizontal force F applied to the cart (N).
 * @param deriv Pointer to StateDerivative structure receiving dY/dt.
 */
void compute_derivatives(const SystemState *state,
                         const SystemParams *params,
                         double force,
                         StateDerivative *deriv);

/**
 * @brief Compute total mechanical energy (kinetic + potential) of the full system.
 * @param state Constant pointer to the current kinematic state.
 * @param params Constant pointer to the physical parameters.
 * @return Total energy in Joules (J).
 */
double compute_total_energy(const SystemState *state, const SystemParams *params);

/**
 * @brief Compute pendulum mechanical energy relative to the upright position (E = 0 at theta = 0).
 * @param state Constant pointer to the current kinematic state.
 * @param params Constant pointer to the physical parameters.
 * @return Pendulum energy in Joules (J).
 */
double compute_pendulum_energy(const SystemState *state, const SystemParams *params);

#endif /* PHYSICS_H */
