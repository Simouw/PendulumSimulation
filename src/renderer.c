#include "renderer.h"
#include <SDL2/SDL.h>
#include <stdio.h>
#include <math.h>

static SDL_Window   *s_window = NULL;
static SDL_Renderer *s_renderer = NULL;
static FILE         *s_csv_file = NULL;
static bool          s_headless = false;
static int           s_win_width = WINDOW_WIDTH;
static int           s_win_height = WINDOW_HEIGHT;

/**
 * @brief Draw a filled circle using horizontal scanline rasterization.
 */
static void draw_filled_circle(SDL_Renderer *renderer, int cx, int cy, int radius) {
    for (int dy = -radius; dy <= radius; dy++) {
        int dx = (int)sqrt((double)(radius * radius - dy * dy));
        SDL_RenderDrawLine(renderer, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

/* ========================================================================= */
/* 5x7 bitmap font, column-major, bit 0 = top row. ASCII 0x20..0x5A.        */
/* ========================================================================= */
static const unsigned char FONT_5X7[0x5B - 0x20][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}  /* 'Z' */
};

#define FONT_W 5
#define FONT_H 7

static const unsigned char *font_glyph(char c) {
    unsigned char u = (unsigned char)c;
    if (u < 0x20 || u > 0x5A) return NULL;
    return FONT_5X7[u - 0x20];
}

/* Runs of lit pixels in a column are merged into one rect, ~3x fewer draw calls. */
static int draw_text(SDL_Renderer *r, int x, int y, int scale, const char *text) {
    int pen_x = x;
    for (const char *p = text; *p != '\0'; p++) {
        const unsigned char *g = font_glyph(*p);
        if (g) {
            for (int col = 0; col < FONT_W; col++) {
                int row = 0;
                while (row < FONT_H) {
                    if (g[col] & (1 << row)) {
                        int start = row;
                        while (row < FONT_H && (g[col] & (1 << row))) row++;
                        SDL_Rect px = {
                            pen_x + col * scale,
                            y + start * scale,
                            scale,
                            (row - start) * scale
                        };
                        SDL_RenderFillRect(r, &px);
                    } else {
                        row++;
                    }
                }
            }
        }
        pen_x += (FONT_W + 1) * scale;
    }
    return pen_x - x;
}

/**
 * @brief Horizontal force vector arrow originating from the cart center.
 */
static void draw_force_arrow(SDL_Renderer *renderer, int x_start, int y, double force) {
    if (fabs(force) < 0.1) return;

    int length = (int)(force * 3.5); // 3.5 pixels per Newton
    int x_end = x_start + length;

    // Main line
    SDL_SetRenderDrawColor(renderer, 240, 80, 80, 255);
    SDL_RenderDrawLine(renderer, x_start, y, x_end, y);
    SDL_RenderDrawLine(renderer, x_start, y - 1, x_end, y - 1);
    SDL_RenderDrawLine(renderer, x_start, y + 1, x_end, y + 1);

    // Arrowhead
    int arrow_dir = (force > 0.0) ? 1 : -1;
    for (int i = 0; i < 8; i++) {
        SDL_RenderDrawLine(renderer, x_end, y, x_end - arrow_dir * i, y - i);
        SDL_RenderDrawLine(renderer, x_end, y, x_end - arrow_dir * i, y + i);
    }
}

bool renderer_init(int width, int height, const char *csv_path) {
    s_win_width = width;
    s_win_height = height;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        fprintf(stderr, "[RENDERER WARNING] SDL_Init failed: %s\n", SDL_GetError());
        fprintf(stderr, "[RENDERER WARNING] Running in headless mode (no window).\n");
        s_headless = true;
    } else {
        s_window = SDL_CreateWindow("Inverted Pendulum Simulation (C11)",
                                    SDL_WINDOWPOS_CENTERED,
                                    SDL_WINDOWPOS_CENTERED,
                                    width,
                                    height,
                                    SDL_WINDOW_SHOWN);
        if (!s_window) {
            fprintf(stderr, "[RENDERER WARNING] SDL_CreateWindow failed: %s\n", SDL_GetError());
            s_headless = true;
        } else {
            s_renderer = SDL_CreateRenderer(s_window, -1,
                                            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if (!s_renderer) {
                s_renderer = SDL_CreateRenderer(s_window, -1, SDL_RENDERER_SOFTWARE);
            }
        }
    }

    if (csv_path != NULL) {
        s_csv_file = fopen(csv_path, "w");
        if (s_csv_file) {
            // Header matching specification: t,x,x_dot,theta,theta_dot,force,energy
            fprintf(s_csv_file, "t,x,x_dot,theta,theta_dot,force,energy\n");
            fflush(s_csv_file);
        } else {
            fprintf(stderr, "[RENDERER WARNING] Unable to open CSV file: %s\n", csv_path);
        }
    }

    return true;
}

UserAction renderer_poll_events(void) {
    if (s_headless) return ACTION_NONE;

    SDL_Event event;
    UserAction action = ACTION_NONE;

    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            return ACTION_QUIT;
        }
        if (event.type == SDL_KEYDOWN) {
            /* Scancode, not keycode: SDL only names lowercase, there is no SDLK_B. */
            if (event.key.keysym.scancode == SDL_SCANCODE_B) {
                return ACTION_RESET_DOWN;
            }
            switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                case SDLK_q:
                    return ACTION_QUIT;
                case SDLK_SPACE:
                    return ACTION_RESET;
                case SDLK_c:
                case SDLK_TAB:
                    return ACTION_TOGGLE_CONTROL;
                default:
                    break;
            }
        }
    }

    return action;
}

void renderer_handle_action(UserAction action,
                            SystemState *state,
                            Controller *ctrl,
                            SimulationContext *ctx,
                            double *manual_force,
                            const SystemParams *params) {
    (void)params;

    if (action == ACTION_QUIT) {
        if (ctx) ctx->running = false;
        return;
    }

    if (action == ACTION_RESET) {
        renderer_plot_reset();
        if (state) {
            state->x = 0.0;
            state->x_dot = 0.0;
            state->theta = M_PI / 4.0; // Reset angle = pi/4, zero velocities
            state->theta_dot = 0.0;
        }
        if (ctrl) {
            ctrl->error_integral = 0.0;
        }
    }

    if (action == ACTION_RESET_DOWN) {
        renderer_plot_reset();
        if (state) {
            state->x = 0.0;
            state->x_dot = 0.0;
            state->theta = M_PI;
            state->theta_dot = 0.0;
        }
        if (ctrl) {
            ctrl->error_integral = 0.0;
        }
    }

    if (action == ACTION_TOGGLE_CONTROL && ctrl) {
        ctrl->active_mode = (ControlMode)((ctrl->active_mode + 1) % 3);
    }

    // Continuous arrow keys handling
    if (manual_force) {
        *manual_force = 0.0;
        if (!s_headless) {
            const Uint8 *keys = SDL_GetKeyboardState(NULL);
            if (keys) {
                if (keys[SDL_SCANCODE_LEFT]) {
                    *manual_force -= PERTURBATION_FORCE;
                }
                if (keys[SDL_SCANCODE_RIGHT]) {
                    *manual_force += PERTURBATION_FORCE;
                }
            }
        }
    }
}

static const char *control_mode_label(const Controller *ctrl) {
    if (!ctrl) return "UNKNOWN";
    switch (ctrl->active_mode) {
        case CTRL_OFF:           return "FREE MODE";
        case CTRL_SWING_UP:      return "SWING-UP";
        case CTRL_STABILIZATION: return "VERTICAL BALANCE";
        default:                 return "UNKNOWN";
    }
}

static SDL_Color control_mode_color(const Controller *ctrl) {
    SDL_Color c = { 150, 150, 160, 255 };
    if (!ctrl) return c;
    switch (ctrl->active_mode) {
        case CTRL_OFF:           c = (SDL_Color){ 220,  80,  80, 255 }; break; // red
        case CTRL_SWING_UP:      c = (SDL_Color){ 240, 170,  50, 255 }; break; // amber
        case CTRL_STABILIZATION: c = (SDL_Color){  70, 205, 120, 255 }; break; // green
        default:                 c = (SDL_Color){ 150, 150, 160, 255 }; break;
    }
    return c;
}

/**
 * @brief Measure the pixel width of a string rendered at the given scale.
 */
static int text_width(const char *s, int scale) {
    int w = 0;
    for (const char *p = s; *p != '\0'; p++) w += (FONT_W + 1) * scale;
    return (w > 0) ? w - scale : 0;
}

/* ========================================================================= */
/* Live strip charts: theta(t), phase portrait, energy(t)                     */
/* ========================================================================= */
static double s_plot_t[PLOT_CAPACITY];
static double s_plot_theta[PLOT_CAPACITY];       /* unwrapped: continuous phase   */
static double s_plot_omega[PLOT_CAPACITY];       /* d(theta)/dt                  */
static double s_plot_energy[PLOT_CAPACITY];      /* mechanical energy (J)        */
static int    s_plot_head = 0;                   /* next write slot               */
static int    s_plot_count = 0;                  /* valid samples, saturates at CAP */

static double s_plot_theta_prev = 0.0;           /* last unwrapped angle         */
static bool   s_plot_unwrap_init = false;

#define PLOT_STRIP_Y    500
#define PLOT_PANEL_W    400
#define PLOT_PANEL_H    210
#define PLOT_PANEL_GAP  20
#define PLOT_STRIP_X    20

void renderer_plot_reset(void) {
    s_plot_head = 0;
    s_plot_count = 0;
    s_plot_unwrap_init = false;
}

void renderer_plot_push(double t, double theta, double theta_dot, double energy) {
    /* Unwrap: theta is stored modulo 2*PI, so crossing the hanging position shows
       up as a -2*PI step. Adding the folds back gives the continuous phase. */
    double unwrapped = theta;
    if (s_plot_unwrap_init) {
        double delta = theta - s_plot_theta_prev;
        while (delta >  M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        unwrapped = s_plot_theta_prev + delta;
    }
    s_plot_theta_prev = unwrapped;
    s_plot_unwrap_init = true;

    s_plot_t[s_plot_head] = t;
    s_plot_theta[s_plot_head] = unwrapped;
    s_plot_omega[s_plot_head] = theta_dot;
    s_plot_energy[s_plot_head] = energy;
    s_plot_head = (s_plot_head + 1) % PLOT_CAPACITY;
    if (s_plot_count < PLOT_CAPACITY) s_plot_count++;
}

/** Round a span up to a visually "nice" step (1, 2, 5 x 10^k). */
static double nice_step(double span) {
    if (span <= 0.0) return 0.5;
    double p = pow(10.0, floor(log10(span)));
    double f = span / p;
    if (f <= 1.0) return p;
    if (f <= 2.0) return 2.0 * p;
    if (f <= 5.0) return 5.0 * p;
    return 10.0 * p;
}

/** Axis rectangle of one panel. */
typedef struct { int x, y, w, h; } PlotRect;

/**
 * @brief Draw a panel background, title and border; return its axis rect.
 */
static PlotRect plot_frame(SDL_Renderer *r, int px, int py, int pw, int ph,
                           const char *title, SDL_Color accent) {
    SDL_SetRenderDrawColor(r, 16, 18, 24, 210);
    SDL_Rect panel = { px, py, pw, ph };
    SDL_RenderFillRect(r, &panel);
    SDL_SetRenderDrawColor(r, accent.r, accent.g, accent.b, 120);
    SDL_RenderDrawRect(r, &panel);

    SDL_SetRenderDrawColor(r, 200, 205, 215, 255);
    draw_text(r, px + 7, py + 5, 1, title);

    PlotRect g;
    g.x = px + 36;
    g.y = py + 22;
    g.w = pw - 36 - 10;
    g.h = ph - 22 - 15;
    return g;
}

static void format_axis_value(char *buf, size_t n, double v) {
    double a = fabs(v);
    if (a >= 1000.0 || (a > 0.0 && a < 0.01))      snprintf(buf, n, "%.0e", v);
    else if (a >= 100.0)                         snprintf(buf, n, "%.0f", v);
    else if (a >= 10.0)                          snprintf(buf, n, "%.1f", v);
    else                                          snprintf(buf, n, "%.2f", v);
}

/** Draw a small numeric label centred on a given axis height. */
static void plot_ylab(SDL_Renderer *r, int px, double value, int axis_y) {
    char buf[16];
    format_axis_value(buf, sizeof buf, value);
    SDL_SetRenderDrawColor(r, 110, 118, 135, 255);
    draw_text(r, px + 6, axis_y - FONT_H / 2, 1, buf);
}

/** Index in the ring buffer of the i-th sample, oldest first. */
static int plot_index(int i) {
    return (s_plot_head - s_plot_count + i + PLOT_CAPACITY) % PLOT_CAPACITY;
}

static void draw_strip_charts(SDL_Renderer *r, const Controller *ctrl) {
    if (s_plot_count < 2) return;

    SDL_Color accent = control_mode_color(ctrl);
    const double t_now = s_plot_t[(s_plot_head - 1 + PLOT_CAPACITY) % PLOT_CAPACITY];
    const double t_start = t_now - PLOT_WINDOW_S;

    /* ---------------------------- theta(t) ---------------------------- */
    {
        int px = PLOT_STRIP_X;
        PlotRect g = plot_frame(r, px, PLOT_STRIP_Y, PLOT_PANEL_W, PLOT_PANEL_H,
                                "THETA (RAD) VS TIME", accent);

        double sum = 0.0;
        for (int i = 0; i < s_plot_count; i++) sum += s_plot_theta[plot_index(i)];
        double mean = sum / (double)s_plot_count;

        double dev = 0.0;
        for (int i = 0; i < s_plot_count; i++) {
            double d = fabs(s_plot_theta[plot_index(i)] - mean);
            if (d > dev) dev = d;
        }
        double half = nice_step(dev * 1.15);
        if (half < 0.2) half = 0.2;
        if (half > 4.0 * M_PI) half = 4.0 * M_PI;  /* keep a spinning bob legible */
        double y_top = mean + half;
        double y_bot = mean - half;
        double y_range = y_top - y_bot;

        SDL_SetRenderDrawColor(r, 70, 78, 95, 255);
        SDL_RenderDrawLine(r, g.x, g.y + g.h / 2, g.x + g.w, g.y + g.h / 2);
        plot_ylab(r, px, mean, g.y + g.h / 2);
        plot_ylab(r, px, y_top, g.y);
        plot_ylab(r, px, y_bot, g.y + g.h);

        for (int sec = 0; sec <= (int)PLOT_WINDOW_S; sec += 4) {
            double tt = t_start + sec;
            if (tt > t_now) break;
            int x = g.x + (int)((tt - t_start) / PLOT_WINDOW_S * (double)g.w);
            SDL_SetRenderDrawColor(r, 70, 78, 95, 255);
            SDL_RenderDrawLine(r, x, g.y, x, g.y + g.h);
        }

        for (int k = -1; k <= 1; k++) {
            int b = (k == 0) ? 255 : 190;
            SDL_SetRenderDrawColor(r, accent.r, accent.g, accent.b, b);
            int prev_x = 0, prev_y = 0;
            for (int i = 0; i < s_plot_count; i++) {
                int idx = plot_index(i);
                int sx = g.x + (int)((s_plot_t[idx] - t_start) / PLOT_WINDOW_S * (double)g.w);
                double n = (s_plot_theta[idx] - y_bot) / y_range;
                if (n < 0.0) n = 0.0;
                if (n > 1.0) n = 1.0;
                int sy = g.y + (int)((1.0 - n) * (double)g.h);
                if (i > 0) SDL_RenderDrawLine(r, prev_x, prev_y, sx, sy);
                prev_x = sx; prev_y = sy;
            }
        }

        int last = (s_plot_head - 1 + PLOT_CAPACITY) % PLOT_CAPACITY;
        int sx = g.x + (int)((s_plot_t[last] - t_start) / PLOT_WINDOW_S * (double)g.w);
        double n = (s_plot_theta[last] - y_bot) / y_range;
        if (n < 0.0) n = 0.0;
        if (n > 1.0) n = 1.0;
        SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
        draw_filled_circle(r, sx, g.y + (int)((1.0 - n) * (double)g.h), 3);
    }

    /* ------------------- phase portrait (theta, theta_dot) ------------------ */
    {
        int px = PLOT_STRIP_X + PLOT_PANEL_W + PLOT_PANEL_GAP;
        PlotRect g = plot_frame(r, px, PLOT_STRIP_Y, PLOT_PANEL_W, PLOT_PANEL_H,
                                "PHASE THETA VS THETA DOT", accent);

        double th_lo = 1e30, th_hi = -1e30, om_lo = 1e30, om_hi = -1e30;
        for (int i = 0; i < s_plot_count; i++) {
            int idx = plot_index(i);
            double th = s_plot_theta[idx];
            double om = s_plot_omega[idx];
            if (th < th_lo) th_lo = th;
            if (th > th_hi) th_hi = th;
            if (om < om_lo) om_lo = om;
            if (om > om_hi) om_hi = om;
        }
        double th_h = (th_hi - th_lo) / 2.0;
        double om_h = (om_hi - om_lo) / 2.0;
        if (th_h < 0.15) th_h = 0.15;
        if (om_h < 0.15) om_h = 0.15;
        th_h *= 1.12;
        om_h *= 1.12;
        double th_c = (th_hi + th_lo) / 2.0;
        double om_c = (om_hi + om_lo) / 2.0;

        double scale = (double)g.w / (2.0 * th_h);
        double sy_scale = (double)g.h / (2.0 * om_h);
        if (sy_scale < scale) scale = sy_scale;
        int off_x = g.x + g.w / 2 - (int)(th_c * scale);
        int off_y = g.y + g.h / 2 + (int)(om_c * scale);

        SDL_SetRenderDrawColor(r, 70, 78, 95, 255);
        SDL_RenderDrawLine(r, off_x, g.y, off_x, g.y + g.h);
        SDL_RenderDrawLine(r, g.x, off_y, g.x + g.w, off_y);

        for (int i = 0; i < s_plot_count; i++) {
            int idx = plot_index(i);
            int x = off_x + (int)(s_plot_theta[idx] * scale);
            int y = off_y - (int)(s_plot_omega[idx] * scale);
            int age = (int)((double)i * 215.0 / (double)s_plot_count);
            SDL_SetRenderDrawColor(r, accent.r, accent.g, accent.b, 40 + age);
            draw_filled_circle(r, x, y, 2);
        }
    }

    /* ---------------------------- energy(t) ---------------------------- */
    {
        int px = PLOT_STRIP_X + 2 * (PLOT_PANEL_W + PLOT_PANEL_GAP);
        PlotRect g = plot_frame(r, px, PLOT_STRIP_Y, PLOT_PANEL_W, PLOT_PANEL_H,
                                "ENERGY (J) VS TIME", accent);

        double e_lo = 1e30, e_hi = -1e30;
        for (int i = 0; i < s_plot_count; i++) {
            double e = s_plot_energy[plot_index(i)];
            if (e < e_lo) e_lo = e;
            if (e > e_hi) e_hi = e;
        }
        double step = nice_step((e_hi - e_lo) * 1.15);
        if (step < 0.02) step = 0.02;
        double base = floor(e_lo / step) * step - step;
        double top  = ceil(e_hi / step) * step + step;
        double e_range = top - base;

        for (double v = base; v <= top + 1e-9; v += step) {
            int y = g.y + g.h - (int)((v - base) / e_range * (double)g.h);
            if (y < g.y || y > g.y + g.h) continue;
            SDL_SetRenderDrawColor(r, 60, 68, 84, 255);
            SDL_RenderDrawLine(r, g.x, y, g.x + g.w, y);
            plot_ylab(r, px, v, y);
        }

        for (int k = -1; k <= 1; k++) {
            int b = (k == 0) ? 255 : 190;
            SDL_SetRenderDrawColor(r, 150, 200, 255, b);
            int prev_x = 0, prev_y = 0;
            for (int i = 0; i < s_plot_count; i++) {
                int idx = plot_index(i);
                int sx = g.x + (int)((s_plot_t[idx] - t_start) / PLOT_WINDOW_S * (double)g.w);
                double n = (s_plot_energy[idx] - base) / e_range;
                if (n < 0.0) n = 0.0;
                if (n > 1.0) n = 1.0;
                int sy = g.y + (int)((1.0 - n) * (double)g.h);
                if (i > 0) SDL_RenderDrawLine(r, prev_x, prev_y, sx, sy);
                prev_x = sx; prev_y = sy;
            }
        }
    }
}


/**
 * @brief Draw the control mode banner in the top-left corner of the canvas.
 * The colour encodes the mode, so the state is readable at a glance without
 * reading the text. A telemetry line below reports energy, swing-up
 * threshold and motor saturation.
 */
static void draw_mode_banner(SDL_Renderer *r,
                             const Controller *ctrl,
                             const SystemState *state,
                             const SystemParams *params,
                             double force) {
    const char *label = control_mode_label(ctrl);
    SDL_Color accent = control_mode_color(ctrl);

    const int scale = 3;
    const int pad = 14;
    const int small = 1;
    const int col_gap = 18;

    /* Telemetry only makes sense while the motor is actually driven.
       In FREE MODE no control law runs, so the row is dropped. */
    const int show_telemetry = (ctrl != NULL && ctrl->active_mode != CTRL_OFF);

    char line_e[48];
    char line_th[64];
    char line_sat[40];

    if (state && params) {
        snprintf(line_e,  sizeof line_e,  "E %.2f J", compute_total_energy(state, params));
        snprintf(line_th, sizeof line_th, "TH %.2f", fabs(state->theta));
        snprintf(line_sat, sizeof line_sat, "SAT %3d%%",
                 (int)(100.0 * fabs(force) / (params->F_max > 0.0 ? params->F_max : 1.0)));
    } else {
        snprintf(line_e,  sizeof line_e,  "E --");
        snprintf(line_th, sizeof line_th, "TH --");
        snprintf(line_sat, sizeof line_sat, "SAT --");
    }

    int content_w = text_width(label, scale);
    int w1 = text_width(line_e, small);
    int w2 = text_width(line_th, small);
    int w3 = text_width(line_sat, small);

    if (show_telemetry) {
        /* The telemetry fields sit side by side, so the row is as wide as the
           SUM of the columns plus their gaps, not as wide as the widest one. */
        int row_w = w1 + col_gap + w2 + col_gap + w3;
        if (row_w > content_w) content_w = row_w;
    }

    const int box_w = content_w + pad + 5 + pad;
    const int box_h = show_telemetry
                    ? (pad + FONT_H * scale + 8 + FONT_H * small + pad)
                    : (pad + FONT_H * scale + pad);

    /* Panel background */
    SDL_SetRenderDrawColor(r, 16, 18, 24, 210);
    SDL_Rect panel = { 20, 20, box_w, box_h };
    SDL_RenderFillRect(r, &panel);

    /* Accent bar on the left, colour-coded by mode */
    SDL_SetRenderDrawColor(r, accent.r, accent.g, accent.b, 255);
    SDL_Rect bar = { 20, 20, 5, box_h };
    SDL_RenderFillRect(r, &bar);

    /* Panel outline */
    SDL_SetRenderDrawColor(r, accent.r, accent.g, accent.b, 120);
    SDL_RenderDrawRect(r, &panel);

    /* Mode name */
    const int text_x = 20 + pad + 5;
    SDL_SetRenderDrawColor(r, 235, 238, 245, 255);
    draw_text(r, text_x, 20 + pad, scale, label);

    if (!show_telemetry) return;

    /* Telemetry line */
    int ty = 20 + pad + FONT_H * scale + 8;
    int tx = text_x;
    SDL_SetRenderDrawColor(r, 150, 200, 255, 255);
    draw_text(r, tx, ty, small, line_e); tx += w1 + col_gap;
    draw_text(r, tx, ty, small, line_th); tx += w2 + col_gap;
    draw_text(r, tx, ty, small, line_sat);
}

void renderer_draw(const SystemState *state,
                   const SystemParams *params,
                   const Controller *ctrl,
                   double force,
                   double time) {
    // 1. Update window title with real-time telemetry
    if (s_window) {
        const char *mode_str = control_mode_label(ctrl);
        double deg = (state ? state->theta : 0.0) * (180.0 / M_PI);
        double energy = (params && state) ? compute_total_energy(state, params) : 0.0;

        char title[256];
        snprintf(title, sizeof(title),
                 "Inverted Pendulum | t=%.2fs | theta=%.3frad (%+05.1f deg) | E=%.2fJ | Mode=%s | F=%+.1fN",
                 time,
                 state ? state->theta : 0.0,
                 deg,
                 energy,
                 mode_str,
                 force);
        SDL_SetWindowTitle(s_window, title);
    }

    if (s_headless || !s_renderer) return;

    // 2. Screen projection geometry
    double scale = PIXELS_PER_METER;
    int origin_x = s_win_width / 2;
    int origin_y = s_win_height / 2 - 30;

    double x_val = state ? state->x : 0.0;
    double theta_val = state ? state->theta : 0.0;
    double L_val = params ? params->L : DEFAULT_L;
    double x_max_val = params ? params->x_max : DEFAULT_X_MAX;

    // Cart screen coordinates
    int cart_x = origin_x + (int)(x_val * scale);
    int cart_y = origin_y;

    // Pendulum bob screen coordinates: (x + L*sin(theta), L*cos(theta))
    // Note: theta = 0 points upwards (screen Y is inverted)
    int bob_x = origin_x + (int)((x_val + L_val * sin(theta_val)) * scale);
    int bob_y = origin_y - (int)((L_val * cos(theta_val)) * scale);

    // 3. Clear background (dark slate)
    SDL_SetRenderDrawColor(s_renderer, 24, 28, 38, 255);
    SDL_RenderClear(s_renderer);

    // 3b. Control mode banner + telemetry, top-left of the canvas
    draw_mode_banner(s_renderer, ctrl, state, params, force);

    // 3c. Scrolling theta(t) plot, top-right of the canvas
    draw_strip_charts(s_renderer, ctrl);

    // 4. Horizontal rail [-x_max, +x_max]
    int rail_x_min = origin_x - (int)(x_max_val * scale);
    int rail_x_max = origin_x + (int)(x_max_val * scale);
    int rail_y = origin_y + 18;

    SDL_SetRenderDrawColor(s_renderer, 70, 80, 100, 255);
    SDL_RenderDrawLine(s_renderer, rail_x_min - 20, rail_y, rail_x_max + 20, rail_y);
    SDL_RenderDrawLine(s_renderer, rail_x_min - 20, rail_y + 1, rail_x_max + 20, rail_y + 1);

    // Rail tick marks every 0.5 meters
    for (double gx = -x_max_val; gx <= x_max_val + 1e-4; gx += 0.5) {
        int tick_x = origin_x + (int)(gx * scale);
        int tick_h = (fabs(gx) < 1e-3) ? 12 : 6;
        SDL_SetRenderDrawColor(s_renderer, 100, 110, 130, 255);
        SDL_RenderDrawLine(s_renderer, tick_x, rail_y, tick_x, rail_y + tick_h);
    }

    // Mechanical end stops at [-x_max, +x_max]
    SDL_Rect stop_left = { rail_x_min - 8, rail_y - 25, 8, 30 };
    SDL_Rect stop_right = { rail_x_max, rail_y - 25, 8, 30 };
    SDL_SetRenderDrawColor(s_renderer, 220, 60, 60, 255);
    SDL_RenderFillRect(s_renderer, &stop_left);
    SDL_RenderFillRect(s_renderer, &stop_right);

    // 5. Pendulum rod (metallic line)
    SDL_SetRenderDrawColor(s_renderer, 210, 215, 230, 255);
    SDL_RenderDrawLine(s_renderer, cart_x, cart_y, bob_x, bob_y);
    SDL_RenderDrawLine(s_renderer, cart_x - 1, cart_y, bob_x - 1, bob_y);
    SDL_RenderDrawLine(s_renderer, cart_x + 1, cart_y, bob_x + 1, bob_y);

    // 6. Cart rectangle at (x, 0)
    int cart_w = 70;
    int cart_h = 34;
    SDL_Rect cart_rect = { cart_x - cart_w / 2, cart_y - cart_h / 2, cart_w, cart_h };

    SDL_SetRenderDrawColor(s_renderer, 45, 125, 210, 255);
    SDL_RenderFillRect(s_renderer, &cart_rect);
    SDL_SetRenderDrawColor(s_renderer, 100, 180, 255, 255);
    SDL_RenderDrawRect(s_renderer, &cart_rect);

    // Wheels
    SDL_SetRenderDrawColor(s_renderer, 30, 30, 40, 255);
    draw_filled_circle(s_renderer, cart_x - cart_w / 3, cart_y + cart_h / 2, 6);
    draw_filled_circle(s_renderer, cart_x + cart_w / 3, cart_y + cart_h / 2, 6);

    // Center pivot
    SDL_SetRenderDrawColor(s_renderer, 255, 255, 255, 255);
    draw_filled_circle(s_renderer, cart_x, cart_y, 4);

    // 7. Pendulum mass bob
    SDL_SetRenderDrawColor(s_renderer, 240, 160, 40, 255); // Golden amber
    draw_filled_circle(s_renderer, bob_x, bob_y, 14);
    SDL_SetRenderDrawColor(s_renderer, 255, 210, 100, 255);
    draw_filled_circle(s_renderer, bob_x - 3, bob_y - 3, 4); // Highlight

    // 8. Force vector arrow
    draw_force_arrow(s_renderer, cart_x, cart_y, force);

    // 9. Vertical guide line for theta = 0
    SDL_SetRenderDrawColor(s_renderer, 60, 90, 70, 180);
    for (int y = cart_y - (int)(L_val * scale) - 20; y < cart_y; y += 8) {
        SDL_RenderDrawPoint(s_renderer, cart_x, y);
    }

    SDL_RenderPresent(s_renderer);
}

void renderer_log_csv(double time,
                      const SystemState *state,
                      double force,
                      double energy) {
    if (!s_csv_file || !state) return;

    fprintf(s_csv_file,
            "%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.6f\n",
            time,
            state->x,
            state->x_dot,
            state->theta,
            state->theta_dot,
            force,
            energy);
}

void renderer_close(void) {
    if (s_csv_file) {
        fflush(s_csv_file);
        fclose(s_csv_file);
        s_csv_file = NULL;
    }

    if (s_renderer) {
        SDL_DestroyRenderer(s_renderer);
        s_renderer = NULL;
    }

    if (s_window) {
        SDL_DestroyWindow(s_window);
        s_window = NULL;
    }

    if (!s_headless) {
        SDL_Quit();
    }
}
