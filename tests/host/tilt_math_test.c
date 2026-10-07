/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Host-side unit test for src/pointing/rip_tilt_math.h (no Zephyr needed).
 *
 * Run from the repository root (test.py runs the same in CI):
 *   cc -std=c11 -Wall -Wextra -Werror -o /tmp/tilt_math_test tests/host/tilt_math_test.c -lm \
 *     && /tmp/tilt_math_test
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../src/pointing/rip_tilt_math.h"

static int failures;
static int checks;

#define CHECK(cond, ...)                                                                           \
    do {                                                                                           \
        checks++;                                                                                  \
        if (!(cond)) {                                                                             \
            failures++;                                                                            \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                                            \
            printf(__VA_ARGS__);                                                                   \
            printf("\n");                                                                          \
        }                                                                                          \
    } while (0)

static void check_matrix(const char *label, struct rip_tilt_matrix m, int32_t xx, int32_t xy,
                         int32_t yx, int32_t yy) {
    CHECK(m.xx == xx && m.xy == xy && m.yx == yx && m.yy == yy,
          "%s: got [[%d, %d], [%d, %d]] expected [[%d, %d], [%d, %d]]", label, m.xx, m.xy, m.yx,
          m.yy, xx, xy, yx, yy);
}

enum report_kind {
    BOTH_AXES, /* X event then Y event per report */
    X_ONLY,
    Y_ONLY,
};

struct totals {
    long x;
    long y;
};

/* Feed `count` reports 1 ms apart (inside the idle window) and sum outputs. */
static struct totals feed(struct rip_tilt_matrix m, enum report_kind kind, int16_t x, int16_t y,
                          int count, struct rip_tilt_state *state_out) {
    struct rip_tilt_state state = {0};
    struct totals t = {0, 0};
    int64_t now = 1000;
    for (int i = 0; i < count; i++, now++) {
        if (kind != Y_ONLY) {
            t.x += rip_tilt_apply_event(&m, true, x, now, &state);
        }
        if (kind != X_ONLY) {
            t.y += rip_tilt_apply_event(&m, false, y, now, &state);
        }
        if (labs(state.remainder_x) > RIP_TRANSFORM_REMAINDER_LIMIT ||
            labs(state.remainder_y) > RIP_TRANSFORM_REMAINDER_LIMIT) {
            CHECK(0, "remainder beyond limit: %d %d", state.remainder_x, state.remainder_y);
            break;
        }
    }
    if (state_out) {
        *state_out = state;
    }
    return t;
}

/*
 * Both-axes reports converge to n * M * v. Y includes the same report's X
 * cross term (only its fraction is pending); X's cross term from Y lags one
 * report. `slack` covers matrix rounding when comparing with exact math.
 */
static void check_converges(const char *label, struct rip_tilt_matrix m, int16_t x, int16_t y,
                            int n, double exp_x, double exp_y, double slack) {
    struct totals t = feed(m, BOTH_AXES, x, y, n, NULL);
    double lag_x = labs((long)m.xy * y) / 1000.0 + 1.0;
    CHECK(fabs(t.x - exp_x) <= lag_x + slack && fabs(t.y - exp_y) <= 1.0 + slack,
          "%s (%d, %d) x%d: got (%ld, %ld) expected (%.2f, %.2f)", label, x, y, n, t.x, t.y, exp_x,
          exp_y);
}

static void check_converges_fixed(const char *label, struct rip_tilt_matrix m, int16_t x, int16_t y,
                                  int n) {
    check_converges(label, m, x, y, n, n * (m.xx * x + m.xy * y) / 1000.0,
                    n * (m.yx * x + m.yy * y) / 1000.0, 0.0);
}

static void test_identity(void) {
    struct rip_tilt_matrix m = rip_tilt_compute_matrix(0, 0, 0);
    check_matrix("rot0 tilt0", m, 1000, 0, 0, 1000);
    CHECK(rip_tilt_matrix_is_identity(&m), "rot0 tilt0 must be identity");

    /* Any axis with zero tilt is still the identity. */
    for (int axis = -180; axis <= 180; axis += 15) {
        m = rip_tilt_compute_matrix(0, 0, axis);
        CHECK(rip_tilt_matrix_is_identity(&m), "tilt 0 axis %d must be identity", axis);
    }
    /* A full turn of rotation is the identity too. */
    m = rip_tilt_compute_matrix(360, 0, 0);
    CHECK(rip_tilt_matrix_is_identity(&m), "rot360 must be identity");
    m = rip_tilt_compute_matrix(-720, 0, 0);
    CHECK(rip_tilt_matrix_is_identity(&m), "rot-720 must be identity");

    struct rip_tilt_matrix boosted = rip_tilt_compute_matrix(0, 45, 0);
    CHECK(!rip_tilt_matrix_is_identity(&boosted), "tilt 45 must not be identity");
    CHECK(rip_tilt_matrix_equal(&m, &m) && !rip_tilt_matrix_equal(&m, &boosted), "matrix equality");

    /* The identity matrix passes values straight through, any pattern. */
    struct totals t = feed(m, BOTH_AXES, 7, -3, 10, NULL);
    CHECK(t.x == 70 && t.y == -30, "identity both: (%ld, %ld)", t.x, t.y);
}

static void test_tilt_axes(void) {
    /* 12-6 o'clock axis: only y is boosted by 1/cos(45) = 1.41421. */
    check_matrix("tilt45 axis0", rip_tilt_compute_matrix(0, 45, 0), 1000, 0, 0, 1414);
    /* 3-9 o'clock axis: only x is boosted. */
    check_matrix("tilt45 axis90", rip_tilt_compute_matrix(0, 45, 90), 1414, 0, 0, 1000);
    check_matrix("tilt60 axis0", rip_tilt_compute_matrix(0, 60, 0), 1000, 0, 0, 2000);
    check_matrix("tilt60 axis90", rip_tilt_compute_matrix(0, 60, 90), 2000, 0, 0, 1000);
    /* 1:30-7:30 axis, d = (0.707, -0.707): off-diagonal terms appear. */
    check_matrix("tilt60 axis45", rip_tilt_compute_matrix(0, 60, 45), 1500, -500, -500, 1500);
    check_matrix("tilt45 axis45", rip_tilt_compute_matrix(0, 45, 45), 1207, -207, -207, 1207);
    /* 10:30-4:30 axis flips the sign of the coupling. */
    check_matrix("tilt60 axis-45", rip_tilt_compute_matrix(0, 60, -45), 1500, 500, 500, 1500);
    /* 1-7 o'clock: d = (0.5, -0.866). */
    check_matrix("tilt60 axis30", rip_tilt_compute_matrix(0, 60, 30), 1250, -433, -433, 1750);
    check_matrix("tilt60 axis-30", rip_tilt_compute_matrix(0, 60, -30), 1250, 433, 433, 1750);

    /* Along the boosted 1:30 direction motion doubles; across it is unchanged. */
    struct rip_tilt_matrix m = rip_tilt_compute_matrix(0, 60, 45);
    CHECK((m.xx * 10 + m.xy * -10) / 1000 == 20 && (m.yx * 10 + m.yy * -10) / 1000 == -20,
          "M * (10, -10) must be (20, -20)");
    CHECK((m.xx * 10 + m.xy * 10) / 1000 == 10 && (m.yx * 10 + m.yy * 10) / 1000 == 10,
          "M * (10, 10) must be (10, 10)");
    check_converges_fixed("along axis45", m, 10, -10, 100);
    check_converges_fixed("across axis45", m, 10, 10, 100);
    check_converges_fixed("axis30", rip_tilt_compute_matrix(0, 60, 30), 4, -9, 200);

    /* The axis is 180-degree periodic. */
    struct rip_tilt_matrix a = rip_tilt_compute_matrix(0, 50, 30);
    struct rip_tilt_matrix b = rip_tilt_compute_matrix(0, 50, -150);
    CHECK(rip_tilt_matrix_equal(&a, &b), "axis 30 and -150 must match");
}

static void test_rotation_then_tilt(void) {
    /* Rotation +90 maps right to down; the 12-6 boost then doubles it. */
    struct rip_tilt_matrix m = rip_tilt_compute_matrix(90, 60, 0);
    check_matrix("rot90 tilt60 axis0", m, 0, -1000, 2000, 0);
    /* One both-axes report (7, 0): X's cross term reaches Y in the same report. */
    struct rip_tilt_state s = {0};
    int16_t ox = rip_tilt_apply_event(&m, true, 7, 1000, &s);
    int16_t oy = rip_tilt_apply_event(&m, false, 0, 1000, &s);
    CHECK(ox == 0 && oy == 14, "rot90 tilt60 right: got (%d, %d) expected (0, 14)", ox, oy);

    /* Rotation alone keeps the historical x cos - y sin / x sin + y cos form. */
    check_matrix("rot90", rip_tilt_compute_matrix(90, 0, 0), 0, -1000, 1000, 0);
    check_matrix("rot-90", rip_tilt_compute_matrix(-90, 0, 0), 0, 1000, -1000, 0);
    check_matrix("rot180", rip_tilt_compute_matrix(180, 0, 0), -1000, 0, 0, -1000);
    check_matrix("rot30", rip_tilt_compute_matrix(30, 0, 0), 866, -500, 500, 866);

    /* Rotation 90 with both-axes reports: rotated totals within one report lag. */
    m = rip_tilt_compute_matrix(90, 0, 0);
    check_converges("rot90 both", m, 5, 3, 100, -300.0, 500.0, 0.0);
    struct totals t = feed(m, BOTH_AXES, 5, 3, 100, NULL);
    CHECK(t.x == -297 && t.y == 500, "rot90 lag: got (%ld, %ld) expected (-297, 500)", t.x, t.y);
}

static void test_clamp_and_normalize(void) {
    CHECK(rip_tilt_clamp_degrees(70) == 60, "clamp 70");
    CHECK(rip_tilt_clamp_degrees(60) == 60, "clamp 60");
    CHECK(rip_tilt_clamp_degrees(59) == 59, "clamp 59");
    CHECK(rip_tilt_clamp_degrees(0) == 0, "clamp 0");
    CHECK(rip_tilt_clamp_degrees(-5) == 0, "clamp -5");
    CHECK(rip_tilt_clamp_degrees(4294967295LL) == 60, "clamp UINT32_MAX");
    CHECK(rip_tilt_clamp_degrees(256) == 60, "clamp 256 must not wrap");
    struct rip_tilt_matrix over = rip_tilt_compute_matrix(0, 70, 0);
    struct rip_tilt_matrix max = rip_tilt_compute_matrix(0, 60, 0);
    CHECK(rip_tilt_matrix_equal(&over, &max), "tilt 70 must equal tilt 60");

    static const struct {
        int32_t in;
        int16_t out;
    } cases[] = {
        {0, 0},   {90, 90},   {-90, 90}, {120, -60}, {270, 90}, {-270, 90}, {180, 0},   {-180, 0},
        {360, 0}, {91, -89},  {-91, 89}, {-89, -89}, {45, 45},  {-45, -45}, {135, -45}, {-135, 45},
        {30, 30}, {-30, -30}, {450, 90}, {-450, 90}, {179, -1}, {-179, 1},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        int16_t got = rip_tilt_normalize_axis_degrees(cases[i].in);
        CHECK(got == cases[i].out, "normalize %d: got %d expected %d", cases[i].in, got,
              cases[i].out);
    }
    for (int64_t deg = INT32_MIN; deg <= INT32_MAX; deg += 9973) {
        int16_t got = rip_tilt_normalize_axis_degrees((int32_t)deg);
        if (got <= -90 || got > 90) {
            CHECK(0, "normalize %lld out of range: %d", (long long)deg, got);
            break;
        }
    }
    CHECK(rip_tilt_normalize_axis_degrees(INT32_MIN) > -90 &&
              rip_tilt_normalize_axis_degrees(INT32_MIN) <= 90,
          "normalize INT32_MIN");
    CHECK(rip_tilt_normalize_axis_degrees(INT32_MAX) > -90 &&
              rip_tilt_normalize_axis_degrees(INT32_MAX) <= 90,
          "normalize INT32_MAX");
}

/* Drivers such as the PMW3610 one report only the axis that moved. */
static void test_single_axis_reports(void) {
    struct rip_tilt_matrix m45 = rip_tilt_compute_matrix(0, 45, 0);
    struct totals t = feed(m45, X_ONLY, 3, 0, 100, NULL);
    CHECK(t.x == 300 && t.y == 0, "tilt45 X-only: got (%ld, %ld) expected (300, 0)", t.x, t.y);
    t = feed(m45, Y_ONLY, 0, 3, 100, NULL);
    CHECK(t.x == 0 && labs(t.y - 424) <= 1, "tilt45 Y-only: got (%ld, %ld) expected (0, 424)", t.x,
          t.y);
    t = feed(m45, Y_ONLY, 0, -3, 100, NULL);
    CHECK(t.x == 0 && labs(t.y + 424) <= 1, "tilt45 -Y-only: got (%ld, %ld)", t.x, t.y);

    /* tilt 60 doubles: 1000 single counts along the boosted axis give 2000. */
    struct rip_tilt_matrix m60 = rip_tilt_compute_matrix(0, 60, 0);
    t = feed(m60, Y_ONLY, 0, 1, 1000, NULL);
    CHECK(t.x == 0 && labs(t.y - 2000) <= 1, "tilt60 Y-only carry: got (%ld, %ld)", t.x, t.y);
    t = feed(m60, X_ONLY, 1, 0, 1000, NULL);
    CHECK(t.x == 1000 && t.y == 0, "tilt60 X-only untouched: got (%ld, %ld)", t.x, t.y);

    /* Fractional gain must not be lost to truncation: 1.414 per count. */
    t = feed(m45, Y_ONLY, 0, 1, 1000, NULL);
    CHECK(t.x == 0 && labs(t.y - 1414) <= 1, "tilt45 1-count carry: got (%ld, %ld)", t.x, t.y);
    t = feed(m45, BOTH_AXES, 0, -1, 1000, NULL);
    CHECK(t.x == 0 && labs(t.y + 1414) <= 1, "tilt45 both -1 carry: got (%ld, %ld)", t.x, t.y);

    /* Plain rotation: the old truncating formula lost every 1-count step at
     * 30 degrees on x (866/1000 -> 0); the carry keeps 866 of 1000. */
    t = feed(rip_tilt_compute_matrix(30, 0, 0), BOTH_AXES, 1, 0, 1000, NULL);
    CHECK(labs(t.x - 866) <= 1 && labs(t.y - 500) <= 1, "rot30 carry: got (%ld, %ld)", t.x, t.y);

    /* Single-axis reports on a diagonal axis: the cross term waits for the
     * other axis instead of being dropped, then arrives with it. */
    struct rip_tilt_matrix m30 = rip_tilt_compute_matrix(0, 60, 30);
    struct rip_tilt_state s;
    t = feed(m30, X_ONLY, 10, 0, 5, &s);
    CHECK(t.x == 62 && t.y == 0 && s.remainder_y == -21650,
          "axis30 X-only: got (%ld, %ld) pending %d", t.x, t.y, s.remainder_y);
    int16_t oy = rip_tilt_apply_event(&m30, false, 0, s.last_event_ms + 1, &s);
    CHECK(oy == -21, "axis30 pending cross term: got %d expected -21", oy);
}

static void test_remainder_clamp_and_idle_reset(void) {
    CHECK(rip_tilt_clamp_remainder(200000) == RIP_TRANSFORM_REMAINDER_LIMIT &&
              rip_tilt_clamp_remainder(-200000) == -RIP_TRANSFORM_REMAINDER_LIMIT &&
              rip_tilt_clamp_remainder(5) == 5,
          "clamp_remainder");

    /* A long X-only run at rotation 30 piles up 500 per count for Y; the
     * first later Y event releases at most 127 counts. */
    struct rip_tilt_matrix m = rip_tilt_compute_matrix(30, 0, 0);
    struct rip_tilt_state s;
    struct totals t = feed(m, X_ONLY, 1, 0, 1000, &s);
    CHECK(labs(t.x - 866) <= 1 && s.remainder_y == RIP_TRANSFORM_REMAINDER_LIMIT,
          "clamped pending: x=%ld pending=%d", t.x, s.remainder_y);
    int16_t oy = rip_tilt_apply_event(&m, false, 0, s.last_event_ms + 1, &s);
    CHECK(oy == 127, "clamped release: got %d expected 127", oy);

    /* Idle reset: at exactly 100 ms the carry survives, past it both clear. */
    m = rip_tilt_compute_matrix(0, 60, 30);
    s = (struct rip_tilt_state){0};
    int16_t ox = rip_tilt_apply_event(&m, true, 10, 1000, &s);
    CHECK(ox == 12 && s.remainder_x == 500 && s.remainder_y == -4330, "idle setup: %d %d %d", ox,
          s.remainder_x, s.remainder_y);
    oy = rip_tilt_apply_event(&m, false, 0, 1000 + RIP_TRANSFORM_IDLE_RESET_MS, &s);
    CHECK(oy == -4 && s.remainder_y == -330, "within idle window: got %d rem %d", oy,
          s.remainder_y);
    ox = rip_tilt_apply_event(&m, true, 0, 1000 + 2 * RIP_TRANSFORM_IDLE_RESET_MS + 1, &s);
    CHECK(ox == 0 && s.remainder_x == 0 && s.remainder_y == 0, "idle reset: got %d rem %d %d", ox,
          s.remainder_x, s.remainder_y);

    s = (struct rip_tilt_state){0};
    (void)rip_tilt_apply_event(&m, true, 10, 5000, &s);
    oy = rip_tilt_apply_event(&m, false, 0, 5000 + RIP_TRANSFORM_IDLE_RESET_MS + 1, &s);
    CHECK(oy == 0, "stale cross term dropped after pause: got %d", oy);
}

/* Upstream: (int32_t)(cos(angle) * 1000.0) with pi = 3.14159265359. */
static void test_rotation_matches_old_formula(void) {
    static const int16_t vectors[][2] = {{3, 0}, {0, 3}, {5, -7}, {-11, 4}, {20, 20}};
    for (int32_t deg = -360; deg <= 360; deg++) {
        double angle = (double)deg * 3.14159265359 / 180.0;
        int32_t old_cos = deg == 0 ? 1000 : (int32_t)(cos(angle) * 1000.0);
        int32_t old_sin = deg == 0 ? 0 : (int32_t)(sin(angle) * 1000.0);
        struct rip_tilt_matrix m = rip_tilt_compute_matrix(deg, 0, 0);
        CHECK(labs(m.xx - old_cos) <= 1 && labs(m.yy - old_cos) <= 1 && labs(m.yx - old_sin) <= 1 &&
                  labs(m.xy + old_sin) <= 1,
              "rot %d matrix differs: new [[%d,%d],[%d,%d]] old cos=%d sin=%d", deg, m.xx, m.xy,
              m.yx, m.yy, old_cos, old_sin);
        if (deg % 5 != 0) {
            continue;
        }
        /* Both-axes reports converge to the exact rotation. */
        double c = cos((double)deg * RIP_TILT_PI / 180.0);
        double s = sin((double)deg * RIP_TILT_PI / 180.0);
        for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++) {
            int16_t x = vectors[i][0];
            int16_t y = vectors[i][1];
            int n = 40;
            double slack = n * (abs(x) + abs(y)) * 0.0005 + 1e-9;
            check_converges("rot", m, x, y, n, n * (x * c - y * s), n * (x * s + y * c), slack);
        }
    }
}

static void test_no_overflow(void) {
    struct rip_tilt_matrix m = rip_tilt_compute_matrix(0, 60, 0);
    struct rip_tilt_state s = {0};
    CHECK(rip_tilt_apply_event(&m, false, INT16_MAX, 1, &s) == INT16_MAX && s.remainder_y == 0,
          "clamp positive");
    CHECK(rip_tilt_apply_event(&m, false, INT16_MIN, 2, &s) == INT16_MIN && s.remainder_y == 0,
          "clamp negative");
    s.remainder_x = RIP_TRANSFORM_REMAINDER_LIMIT;
    CHECK(rip_tilt_apply_event(&m, true, INT16_MAX, 3, &s) == INT16_MAX, "pending at max");

    /* Every legal setting at the extreme inputs stays within int16 and keeps
     * the remainders bounded. */
    static const int16_t extremes[] = {INT16_MIN, -1, 0, 1, INT16_MAX};
    for (int rot = -360; rot <= 360; rot += 45) {
        for (int tilt = 0; tilt <= 60; tilt += 15) {
            for (int axis = -75; axis <= 90; axis += 15) {
                struct rip_tilt_matrix t = rip_tilt_compute_matrix(rot, tilt, axis);
                CHECK(labs(t.xx) <= 2000 && labs(t.xy) <= 2000 && labs(t.yx) <= 2000 &&
                          labs(t.yy) <= 2000,
                      "matrix entry above 2.0 at rot %d tilt %d axis %d", rot, tilt, axis);
                struct rip_tilt_state st = {0};
                int64_t now = 1;
                for (size_t i = 0; i < 5; i++) {
                    for (size_t j = 0; j < 5; j++) {
                        (void)rip_tilt_apply_event(&t, true, extremes[i], now, &st);
                        bool own_ok = labs(st.remainder_x) < RIP_TILT_FIXED_ONE;
                        (void)rip_tilt_apply_event(&t, false, extremes[j], now++, &st);
                        own_ok = own_ok && labs(st.remainder_y) < RIP_TILT_FIXED_ONE;
                        if (!own_ok || labs(st.remainder_x) > RIP_TRANSFORM_REMAINDER_LIMIT) {
                            CHECK(0, "remainder escaped range at rot %d tilt %d axis %d", rot, tilt,
                                  axis);
                        }
                    }
                }
            }
        }
    }
}

int main(void) {
    test_identity();
    test_tilt_axes();
    test_rotation_then_tilt();
    test_clamp_and_normalize();
    test_single_axis_reports();
    test_remainder_clamp_and_idle_reset();
    test_rotation_matches_old_formula();
    test_no_overflow();

    if (failures) {
        printf("tilt_math_test: %d of %d checks FAILED\n", failures, checks);
        return 1;
    }
    printf("tilt_math_test: all %d checks passed\n", checks);
    return 0;
}
