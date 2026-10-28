#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "physics.h"

/**
 * @brief Finite state machine modes for the cart-pole controller.
 */
typedef enum {
    CTRL_OFF = 0,           /**< Motor disengaged (F = 0), free passive dynamics */
    CTRL_SWING_UP = 1,      /**< Non-linear energy pumping (|theta| > threshold) */
    CTRL_STABILIZATION = 2  /**< Linear state-feedback regulation (|theta| <= threshold) */
} ControlMode;

/**
 * @brief Controller structure holding gains, thresholds, and state variables.
 */
typedef struct {
    ControlMode active_mode;    /**< Current active mode in the state machine */
    double K_x;                 /**< State gain for cart position */
    double K_vx;                /**< State gain for cart velocity */
    double K_theta;             /**< State gain for pendulum angle */
    double K_vtheta;            /**< State gain for pendulum angular velocity */
    double K_i;                 /**< Integral gain on cart position error (same sign family as K_x) */
    double k_E;                 /**< Energy pumping gain for swing-up mode */
    double theta_threshold;     /**< Angle threshold for switching to stabilization (rad) */
    double x_ref;               /**< Target position for cart on the rail (m) */
    double error_integral;      /**< Integral accumulator to eliminate steady-state error */
} Controller;

/* ========================================================================= */
/* Controller Module Function Signatures                                     */
/* ========================================================================= */

/**
 * @brief Initialize the controller with default gains and an initial mode.
 * @param ctrl Pointer to Controller structure.
 * @param initial_mode Initial control mode (e.g., CTRL_OFF or CTRL_SWING_UP).
 */
void init_controller(Controller *ctrl, ControlMode initial_mode);

/**
 * @brief Clamp a real value within [min, max].
 * @param val Input value.
 * @param min Lower bound.
 * @param max Upper bound.
 * @return Clamped value.
 */
double saturate_value(double val, double min, double max);

/**
 * @brief Update controller finite state machine mode based on current state.
 * @param ctrl Pointer to Controller structure.
 * @param state Constant pointer to current kinematic state.
 */
void update_control_mode(Controller *ctrl, const SystemState *state);

/**
 * @brief Compute non-linear energy-pumping swing-up force.
 * @param ctrl Constant pointer to Controller.
 * @param state Constant pointer to current kinematic state.
 * @param params Constant pointer to physical parameters.
 * @return Computed raw horizontal force (N).
 */
double compute_swing_up_force(const Controller *ctrl,
                              const SystemState *state,
                              const SystemParams *params);

/**
 * @brief Compute linear full-state feedback stabilization force (LQR/PID).
 * @param ctrl Pointer to Controller.
 * @param state Constant pointer to current kinematic state.
 * @param dt Time step (s).
 * @return Computed raw stabilization force (N).
 */
double compute_stabilization_force(Controller *ctrl,
                                   const SystemState *state,
                                   double dt);

/**
 * @brief Master controller function: selects mode, calculates force, and clamps to [-F_max, +F_max].
 * @param ctrl Pointer to Controller.
 * @param state Constant pointer to current kinematic state.
 * @param params Constant pointer to physical parameters.
 * @param dt Time step (s).
 * @return Saturated motor force command (N).
 */
double compute_motor_command(Controller *ctrl,
                             const SystemState *state,
                             const SystemParams *params,
                             double dt);

#endif /* CONTROLLER_H */
