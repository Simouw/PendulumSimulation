#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "physics.h"

/* ========================================================================= */
/* Numerical Integrator Function Signatures                                  */
/* ========================================================================= */

/**
 * @brief Compute linear combination of state and derivative: result = base + factor * deriv.
 * @param base Constant pointer to base state.
 * @param deriv Constant pointer to state derivative.
 * @param factor Multiplicative scale factor (e.g., dt/2 or dt).
 * @param result Pointer to destination state structure.
 */
void combine_intermediate_state(const SystemState *base,
                                const StateDerivative *deriv,
                                double factor,
                                SystemState *result);

/**
 * @brief Advance system state from t to t + dt using Runge-Kutta 4th order (RK4).
 * Post-conditions:
 * - State is updated to t + dt.
 * - Angle theta is normalized within ]-pi, +pi].
 * - Mechanical rail limits [-x_max, +x_max] are enforced.
 * @param state Pointer to state to integrate (in/out).
 * @param params Constant pointer to physical parameters.
 * @param force Constant force applied during the time step (N).
 * @param dt Time step duration (s).
 */
void integrate_rk4(SystemState *state,
                   const SystemParams *params,
                   double force,
                   double dt);

#endif /* INTEGRATOR_H */
