#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================= */
/* Default Physical Parameters                                               */
/* ========================================================================= */

#define DEFAULT_M       1.0     /**< Cart mass M (kg, > 0) */
#define DEFAULT_m       0.2     /**< Pendulum bob mass m (kg, > 0) */
#define DEFAULT_L       1.0     /**< Rod length L (m, > 0) */
#define DEFAULT_G       9.81    /**< Gravitational acceleration g (m/s^2) */
#define DEFAULT_B       0.1     /**< Cart viscous friction coefficient b (N.s/m) */
#define DEFAULT_C       0.01    /**< Pivot damping coefficient c (N.m.s/rad) */
#define DEFAULT_F_MAX   9.0     /**< Motor force saturation F_max (N, > 0) */
#define DEFAULT_X_MAX   5.0     /**< Rail half-length limit X_max (m, > 0) */

/* ========================================================================= */
/* Numerical and Time-Stepping Settings                                      */
/* ========================================================================= */

#define DT_PHYS         0.001   /**< Physics integration step dt = 1 ms (1000 Hz) */
#define DT_RENDER       0.0166  /**< Screen refresh period (~60 Hz) */

/* ========================================================================= */
/* Graphical and GUI Settings                                                */
/* ========================================================================= */

#define WINDOW_WIDTH        1280    /**< Window width (pixels) */
#define WINDOW_HEIGHT       720     /**< Window height (pixels) */
#define PIXELS_PER_METER    120.0   /**< Meter-to-pixel scale (bottom strip reserved for charts) */
#define PERTURBATION_FORCE  12.0    /**< Manual perturbation force (N) */

/* ========================================================================= */
/* Controller & Numerical Precision Constants                                */
/* ========================================================================= */

#define EPSILON             1e-6    /**< Floating-point tolerance */
#define SWINGUP_THETA_SEUIL 0.35    /**< Angle threshold: Swing-up -> Stabilization (rad) */

#endif /* CONFIG_H */
