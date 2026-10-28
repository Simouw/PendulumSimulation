#ifndef RENDERER_H
#define RENDERER_H

#include "physics.h"
#include "controller.h"
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief User actions captured from keyboard or input events.
 */
typedef enum {
    ACTION_NONE = 0,
    ACTION_QUIT,
    ACTION_RESET,
    ACTION_RESET_DOWN,
    ACTION_PUSH_LEFT,
    ACTION_PUSH_RIGHT,
    ACTION_TOGGLE_CONTROL
} UserAction;

/**
 * @brief Simulation context holding timing and main loop execution state.
 */
typedef struct {
    double current_time; /**< Current simulated physical time (s) */
    double dt_phys;      /**< Physical RK4 integration step (s) */
    double dt_render;    /**< Visual display refresh period (s) */
    bool running;        /**< Flag indicating if the main loop is active */
} SimulationContext;

/* Strip chart sampling: the main loop pushes at PLOT_RATE_HZ from the physics
   loop, not from the render loop, so the time axis stays uniform. */
#define PLOT_CAPACITY 1400
#define PLOT_RATE_HZ  100
#define PLOT_WINDOW_S ((double)PLOT_CAPACITY / (double)PLOT_RATE_HZ)

/**
 * @brief Record one sample into the plot ring buffer. Call at PLOT_RATE_HZ.
 */
void renderer_plot_push(double t, double theta, double theta_dot, double energy);

/**
 * @brief Clear the plot ring buffer.
 */
void renderer_plot_reset(void);

/* ========================================================================= */
/* Renderer & Telemetry Function Signatures                                  */
/* ========================================================================= */

/**
 * @brief Initialize the graphical window and open the CSV telemetry file.
 * @param width Window width in pixels.
 * @param height Window height in pixels.
 * @param csv_path Telemetry CSV output file path.
 * @return true on success, false otherwise.
 */
bool renderer_init(int width, int height, const char *csv_path);

/**
 * @brief Poll input events non-blockingly.
 * @return Detected user action.
 */
UserAction renderer_poll_events(void);

/**
 * @brief Process user action and update state, controller, context, or manual force.
 * @param action User action to process.
 * @param state Pointer to SystemState to reset or modify.
 * @param ctrl Pointer to Controller.
 * @param ctx Pointer to SimulationContext.
 * @param manual_force Pointer to manual force variable (N).
 * @param params Constant pointer to physical parameters.
 */
void renderer_handle_action(UserAction action,
                            SystemState *state,
                            Controller *ctrl,
                            SimulationContext *ctx,
                            double *manual_force,
                            const SystemParams *params);

/**
 * @brief Render a single visual frame: rail, bumpers, cart, rod, bob, force vector, and HUD.
 * @param state Constant pointer to current state.
 * @param params Constant pointer to physical parameters.
 * @param ctrl Constant pointer to controller.
 * @param force Current applied motor force (N).
 * @param time Current simulation time (s).
 */
void renderer_draw(const SystemState *state,
                   const SystemParams *params,
                   const Controller *ctrl,
                   double force,
                   double time);

/**
 * @brief Write a timestamped telemetry entry (t, x, x_dot, theta, theta_dot, force, energy) into CSV.
 * @param time Current simulation time (s).
 * @param state Constant pointer to state.
 * @param force Applied force (N).
 * @param energy Total mechanical energy (J).
 */
void renderer_log_csv(double time,
                      const SystemState *state,
                      double force,
                      double energy);

/**
 * @brief Cleanly close file handles and release graphical resources.
 */
void renderer_close(void);

#endif /* RENDERER_H */
