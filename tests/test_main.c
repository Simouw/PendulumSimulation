#include "config.h"
#include "physics.h"
#include "integrator.h"
#include "controller.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define ANSI_GREEN  "\033[1;32m"
#define ANSI_RED    "\033[1;31m"
#define ANSI_YELLOW "\033[1;33m"
#define ANSI_CYAN   "\033[1;36m"
#define ANSI_RESET  "\033[0m"

static int s_tests_run = 0;
static int s_tests_passed = 0;
static int s_tests_failed = 0;

#define TEST_CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        printf("    " ANSI_RED "[ASSERT FAILED]" ANSI_RESET " " msg "\n", ##__VA_ARGS__); \
        test_ok = false; \
    } \
} while (0)

/**
 * @brief Test 1: Angular normalization in ]-pi, +pi].
 * Verifies that wrap_angle_pi() maps 3*PI, -3*PI, PI + 0.1, and 0.0 into ]-PI, PI].
 */
static bool test_wrap_angle(void) {
    s_tests_run++;
    bool test_ok = true;
    printf(ANSI_CYAN "[TEST 1/4]" ANSI_RESET " test_wrap_angle()...\n");

    // 1. Zero angle
    double a0 = wrap_angle_pi(0.0);
    TEST_CHECK(fabs(a0 - 0.0) < EPSILON, "wrap_angle_pi(0.0) expected 0.0, got %f", a0);

    // 2. 3*PI wraps to +PI (upper bound included in ]-PI, PI])
    double a_3pi = wrap_angle_pi(3.0 * M_PI);
    TEST_CHECK(fabs(a_3pi - M_PI) < EPSILON, "wrap_angle_pi(3*PI) expected +PI, got %f", a_3pi);

    // 3. -3*PI wraps to +PI (-3*PI + 4*PI = +PI since -PI is excluded)
    double a_m3pi = wrap_angle_pi(-3.0 * M_PI);
    TEST_CHECK(fabs(a_m3pi - M_PI) < EPSILON, "wrap_angle_pi(-3*PI) expected +PI, got %f", a_m3pi);

    // 4. PI + 0.1 wraps to -PI + 0.1
    double a_pi_plus = wrap_angle_pi(M_PI + 0.1);
    double expected_pi_plus = -M_PI + 0.1;
    TEST_CHECK(fabs(a_pi_plus - expected_pi_plus) < EPSILON,
               "wrap_angle_pi(PI + 0.1) expected %f, got %f", expected_pi_plus, a_pi_plus);

    if (test_ok) {
        printf("  " ANSI_GREEN "[PASS]" ANSI_RESET " test_wrap_angle(): Angles correctly normalized in ]-PI, PI].\n");
        s_tests_passed++;
    } else {
        printf("  " ANSI_RED "[FAIL]" ANSI_RESET " test_wrap_angle(): Angular normalization failure.\n");
        s_tests_failed++;
    }
    return test_ok;
}

/**
 * @brief Test 2: Equilibrium derivatives.
 * Verifies that at theta = 0 and theta = PI (zero velocities and force),
 * compute_derivatives() yields x_ddot = 0 and theta_ddot = 0.
 */
static bool test_equilibrium_derivatives(void) {
    s_tests_run++;
    bool test_ok = true;
    printf(ANSI_CYAN "[TEST 2/4]" ANSI_RESET " test_equilibrium_derivatives()...\n");

    SystemParams params;
    params.M = 1.0;
    params.m = 0.2;
    params.L = 1.0;
    params.g = 9.81;
    params.b = 0.1;
    params.c = 0.01;
    params.F_max = 20.0;
    params.x_max = 2.4;

    // Upright equilibrium: theta = 0
    SystemState state_up = { .x = 0.0, .x_dot = 0.0, .theta = 0.0, .theta_dot = 0.0 };
    StateDerivative deriv_up = {0};
    compute_derivatives(&state_up, &params, 0.0, &deriv_up);

    TEST_CHECK(fabs(deriv_up.x_ddot) < EPSILON,
               "theta = 0: x_ddot expected 0.0, got %f", deriv_up.x_ddot);
    TEST_CHECK(fabs(deriv_up.theta_ddot) < EPSILON,
               "theta = 0: theta_ddot expected 0.0, got %f", deriv_up.theta_ddot);
    TEST_CHECK(fabs(deriv_up.x_dot) < EPSILON,
               "theta = 0: x_dot expected 0.0, got %f", deriv_up.x_dot);
    TEST_CHECK(fabs(deriv_up.theta_dot) < EPSILON,
               "theta = 0: theta_dot expected 0.0, got %f", deriv_up.theta_dot);

    // Downward equilibrium: theta = PI
    SystemState state_down = { .x = 0.0, .x_dot = 0.0, .theta = M_PI, .theta_dot = 0.0 };
    StateDerivative deriv_down = {0};
    compute_derivatives(&state_down, &params, 0.0, &deriv_down);

    TEST_CHECK(fabs(deriv_down.x_ddot) < EPSILON,
               "theta = PI: x_ddot expected 0.0, got %f", deriv_down.x_ddot);
    TEST_CHECK(fabs(deriv_down.theta_ddot) < EPSILON,
               "theta = PI: theta_ddot expected 0.0, got %f", deriv_down.theta_ddot);

    if (test_ok) {
        printf("  " ANSI_GREEN "[PASS]" ANSI_RESET " test_equilibrium_derivatives(): Accelerations are zero at equilibrium points.\n");
        s_tests_passed++;
    } else {
        printf("  " ANSI_RED "[FAIL]" ANSI_RESET " test_equilibrium_derivatives(): Non-zero accelerations at equilibria.\n");
        s_tests_failed++;
    }
    return test_ok;
}

/**
 * @brief Test 3: Conservation of mechanical energy using RK4.
 * Simulates 5000 steps (dt = 0.001 s) without friction (b = 0, c = 0, F = 0, theta_0 = PI/3)
 * with integrate_rk4() and verifies that compute_total_energy() stays constant within 1e-6.
 */
static bool test_energy_conservation_rk4(void) {
    s_tests_run++;
    bool test_ok = true;
    printf(ANSI_CYAN "[TEST 3/4]" ANSI_RESET " test_energy_conservation_rk4()...\n");

    SystemParams params;
    params.M = 1.0;
    params.m = 0.2;
    params.L = 1.0;
    params.g = 9.81;
    params.b = 0.0;
    params.c = 0.0;
    params.F_max = 50.0;
    params.x_max = 50.0;

    SystemState state = { .x = 0.0, .x_dot = 0.0, .theta = M_PI / 3.0, .theta_dot = 0.0 };

    double dt = 0.001;
    int nb_steps = 5000;

    double E0 = compute_total_energy(&state, &params);
    double max_delta_E = 0.0;

    for (int step = 0; step < nb_steps; step++) {
        integrate_rk4(&state, &params, 0.0, dt);
        double E = compute_total_energy(&state, &params);
        double delta_E = fabs(E - E0);
        if (delta_E > max_delta_E) {
            max_delta_E = delta_E;
        }
    }

    TEST_CHECK(max_delta_E < 1e-6,
               "Energy conservation: max |delta_E| = %e > 1e-6 (E0 = %f)", max_delta_E, E0);

    if (test_ok) {
        printf("  " ANSI_GREEN "[PASS]" ANSI_RESET " test_energy_conservation_rk4(): Energy conserved within %e J over 5000 steps.\n", max_delta_E);
        s_tests_passed++;
    } else {
        printf("  " ANSI_RED "[FAIL]" ANSI_RESET " test_energy_conservation_rk4(): Energy drift detected (%e > 1e-6).\n", max_delta_E);
        s_tests_failed++;
    }
    return test_ok;
}

/**
 * @brief Test 4: Small-angle oscillation period.
 * Blocked cart (M = 1e9), frictionless, released at theta_0 = PI - 0.05.
 * Verifies that the first passage through theta = PI occurs at t = (PI/2)*sqrt(L/g) within 1%%.
 */
static bool test_small_angle_period(void) {
    s_tests_run++;
    bool test_ok = true;
    printf(ANSI_CYAN "[TEST 4/4]" ANSI_RESET " test_small_angle_period()...\n");

    SystemParams params;
    params.M = 1e9; // Immobile cart
    params.m = 0.2;
    params.L = 1.0;
    params.g = 9.81;
    params.b = 0.0;
    params.c = 0.0;
    params.F_max = 50.0;
    params.x_max = 50.0;

    double initial_angle = M_PI - 0.05;
    SystemState state = { .x = 0.0, .x_dot = 0.0, .theta = initial_angle, .theta_dot = 0.0 };

    double dt = 0.0005;
    double t_sim = 0.0;
    double prev_offset = wrap_angle_pi(initial_angle - M_PI);
    double prev_time = 0.0;
    double t_first_passage = -1.0;

    double t_theoretical = (M_PI / 2.0) * sqrt(params.L / params.g);

    while (t_sim < 3.0) {
        prev_offset = wrap_angle_pi(state.theta - M_PI);
        prev_time = t_sim;

        integrate_rk4(&state, &params, 0.0, dt);
        t_sim += dt;

        /* Offset to the bottom, so the bottom sits at 0 instead of at the +PI wrap edge. */
        double offset = wrap_angle_pi(state.theta - M_PI);

        if (prev_offset < 0.0 && offset >= 0.0) {
            double fraction = (0.0 - prev_offset) / (offset - prev_offset);
            t_first_passage = prev_time + fraction * dt;
            break;
        }
    }

    if (t_first_passage < 0.0) {
        TEST_CHECK(false, "Pendulum never reached theta = PI during the simulation window.");
    } else {
        double relative_error = fabs(t_first_passage - t_theoretical) / t_theoretical;
        TEST_CHECK(relative_error <= 0.01,
                   "First passage: measured = %.5fs, theoretical = %.5fs, rel error = %.3f%% (> 1%%)",
                   t_first_passage, t_theoretical, relative_error * 100.0);
    }

    if (test_ok) {
        printf("  " ANSI_GREEN "[PASS]" ANSI_RESET " test_small_angle_period(): First passage detected at %.4fs (theory %.4fs, error < 1%%).\n",
               t_first_passage, t_theoretical);
        s_tests_passed++;
    } else {
        printf("  " ANSI_RED "[FAIL]" ANSI_RESET " test_small_angle_period(): Period deviates from theoretical model.\n");
        s_tests_failed++;
    }
    return test_ok;
}

int main(void) {
    printf("============================================================\n");
    printf("       INVERTED PENDULUM UNIT TEST SUITE (C11 CLI)         \n");
    printf("============================================================\n\n");

    test_wrap_angle();
    printf("\n");
    test_equilibrium_derivatives();
    printf("\n");
    test_energy_conservation_rk4();
    printf("\n");
    test_small_angle_period();
    printf("\n");

    printf("============================================================\n");
    printf("  TEST SUMMARY: %d / %d passed (%d failed)\n",
           s_tests_passed, s_tests_run, s_tests_failed);
    printf("============================================================\n");

    if (s_tests_failed > 0) {
        printf(ANSI_YELLOW "[INFO] Notice: Failures are expected while functions in src/physics.c and src/integrator.c remain TODO skeletons.\n" ANSI_RESET);
        return 1;
    }

    printf(ANSI_GREEN "[SUCCESS] All unit tests passed successfully!\n" ANSI_RESET);
    return 0;
}
