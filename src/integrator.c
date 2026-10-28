#include "integrator.h"
#include "physics.h"
#include <math.h>

void combine_intermediate_state(const SystemState *base,
                                const StateDerivative *deriv,
                                double factor,
                                SystemState *result) {
    // TODO: Compute intermediate linear combination: result = base + factor * deriv
  (*result).x = (*base).x + factor * (*deriv).x_dot;
  (*result).x_dot = (*base).x_dot + factor * (*deriv).x_ddot;

  (*result).theta = (*base).theta + factor*(*deriv).theta_dot;
  (*result).theta_dot = (*base).theta_dot + factor * (*deriv).theta_ddot;


}

void integrate_rk4(SystemState *state,
                   const SystemParams *params,
                   double force,
                   double dt) {
    // TODO: Execute one 4th-order Runge-Kutta step (k1, k2, k3, k4), normalize theta, clamp x

  StateDerivative k1;
  compute_derivatives(state, params, force, &k1);
  StateDerivative k2;
  SystemState yintermediate1;
  combine_intermediate_state(state, &k1, dt/2, &yintermediate1);
  compute_derivatives(&yintermediate1, params, force, &k2 );
  StateDerivative k3;
  SystemState yintermediate2;
  combine_intermediate_state(state, &k2, dt/2, &yintermediate2);
  compute_derivatives(&yintermediate2, params, force, &k3 );
  StateDerivative k4;
  SystemState yintermediate3;
  combine_intermediate_state(state, &k3, dt, &yintermediate3);
  compute_derivatives(&yintermediate3, params, force, &k4 );

  (*state).x = (*state).x + dt/6 * ( k1.x_dot + 2*k2.x_dot + 2*k3.x_dot + k4.x_dot);
  (*state).x_dot = (*state).x_dot + dt/6 * ( k1.x_ddot + 2*k2.x_ddot + 2*k3.x_ddot + k4.x_ddot);
  (*state).theta = (*state).theta + dt/6 * ( k1.theta_dot + 2*k2.theta_dot + 2*k3.theta_dot + k4.theta_dot);
  (*state).theta_dot = (*state).theta_dot + dt/6 * ( k1.theta_ddot + 2*k2.theta_ddot + 2*k3.theta_ddot + k4.theta_ddot);

  (*state).theta = wrap_angle_pi((*state).theta);
  
  if (fabs((*state).x) > (*params).x_max) {
    if ((*state).x < 0){
      (*state).x = -(*params).x_max;
    }
    else{
      (*state).x = (*params).x_max;
    }
    (*state).x_dot = 0;
  }
}
