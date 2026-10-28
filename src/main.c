#include "config.h"
#include "physics.h"
#include "integrator.h"
#include "controller.h"
#include "renderer.h"

#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    printf("============================================================\n");
    printf("     INVERTED PENDULUM ON A CART SIMULATOR (C11 C-POLE)    \n");
    printf("============================================================\n");
    printf("  Keyboard Controls:\n");
    printf("    [SPACE]          : Reset state (theta = pi/4, zero velocity)\n");
    printf("    [B]              : Reset state (theta = pi, hanging down)\n");
    printf("    [LEFT/RIGHT]     : Apply manual horizontal force (+/- %.1f N)\n", PERTURBATION_FORCE);
    printf("    [C] or [TAB]     : Toggle controller mode (OFF / SWING_UP / STAB)\n");
    printf("    [ESC] or [Q]     : Quit cleanly\n");
    printf("============================================================\n\n");

    // 1. Initialize physical parameters
    SystemParams params;
    init_system_params(&params);

    // Safety fallback if skeleton values are zeroed
    if (params.M <= 0.0)     params.M = DEFAULT_M;
    if (params.m <= 0.0)     params.m = DEFAULT_m;
    if (params.L <= 0.0)     params.L = DEFAULT_L;
    if (params.g <= 0.0)     params.g = DEFAULT_G;
    if (params.b < 0.0)      params.b = DEFAULT_B;
    if (params.c < 0.0)      params.c = DEFAULT_C;
    if (params.F_max <= 0.0) params.F_max = DEFAULT_F_MAX;
    if (params.x_max <= 0.0) params.x_max = DEFAULT_X_MAX;

    // 2. Initialize kinematic state (theta_0 = 0.5 rad per specification)
    SystemState state;
    init_system_state(&state, 0.0, 0.5);
    state.x = 0.0;
    state.x_dot = 0.0;
    state.theta = 0.5;
    state.theta_dot = 0.0;

    // 3. Initialize controller in open loop (CTRL_OFF)
    Controller ctrl;
    init_controller(&ctrl, CTRL_OFF);
    ctrl.active_mode = CTRL_OFF;

    // 4. Initialize timing context
    SimulationContext ctx;
    ctx.current_time = 0.0;
    ctx.dt_phys = DT_PHYS;     // 1 ms (1000 Hz)
    ctx.dt_render = DT_RENDER; // 16.6 ms (~60 Hz)
    ctx.running = true;

    // 5. Initialize graphical renderer and CSV telemetry
    const char *csv_filename = "telemetrie.csv";
    if (!renderer_init(WINDOW_WIDTH, WINDOW_HEIGHT, csv_filename)) {
        fprintf(stderr, "[MAIN ERROR] Failed to initialize display interface.\n");
        return 1;
    }

    printf("[MAIN] Starting fixed-step simulation loop...\n");
    printf("[MAIN] Physics step: %.3f ms | Render step: %.1f ms | CSV output: %s\n\n",
           ctx.dt_phys * 1000.0, ctx.dt_render * 1000.0, csv_filename);

    // 6. Sub-stepping temporal orchestration loop
    double accumulator = 0.0;
    double manual_force = 0.0;
    double force_cmd = 0.0;
    double plot_accum = 0.0;
    const double plot_period = 1.0 / (double)PLOT_RATE_HZ;

    uint64_t perf_freq = SDL_GetPerformanceFrequency();
    uint64_t last_counter = SDL_GetPerformanceCounter();

    while (ctx.running) {
        // A. Poll user input events at 60 Hz
        UserAction act = renderer_poll_events();
        renderer_handle_action(act, &state, &ctrl, &ctx, &manual_force, &params);
        if (!ctx.running) break;

        // Wall-clock elapsed measurement
        uint64_t now_counter = SDL_GetPerformanceCounter();
        double real_elapsed = (double)(now_counter - last_counter) / (double)perf_freq;
        last_counter = now_counter;

        // Anti-spiral lag clamp
        if (real_elapsed > 0.1) real_elapsed = 0.1;
        accumulator += real_elapsed;

        // B. High-frequency physics sub-stepping (dt = 1 ms)
        while (accumulator >= ctx.dt_phys) {
            if (ctrl.active_mode == CTRL_OFF) {
                force_cmd = manual_force;
            } else {
                force_cmd = compute_motor_command(&ctrl, &state, &params, ctx.dt_phys) + manual_force;
            }
            force_cmd = saturate_value(force_cmd, -params.F_max, params.F_max);

            integrate_rk4(&state, &params, force_cmd, ctx.dt_phys);

            ctx.current_time += ctx.dt_phys;
            accumulator -= ctx.dt_phys;

            plot_accum += ctx.dt_phys;
            while (plot_accum >= plot_period) {
                plot_accum -= plot_period;
                renderer_plot_push(ctx.current_time, state.theta, state.theta_dot,
                                  compute_total_energy(&state, &params));
            }
        }

        // C. Render frame and log telemetry at 60 Hz
        double current_energy = compute_total_energy(&state, &params);
        renderer_draw(&state, &params, &ctrl, force_cmd, ctx.current_time);
        renderer_log_csv(ctx.current_time, &state, force_cmd, current_energy);

        // Frame rate pacing
        SDL_Delay(1);
    }

    // 7. Clean shutdown
    renderer_close();
    printf("\n[MAIN] Simulation terminated cleanly at t = %.2f s.\n", ctx.current_time);
    printf("[MAIN] Telemetry logged to '%s'.\n", csv_filename);

    return 0;
}
