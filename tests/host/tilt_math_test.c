/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Host-side unit test for src/pointing/rip_tilt_math.h (no Zephyr needed).
 *
 * Run from the repository root:
 *   cc -std=c11 -Wall -Wextra -Werror -o /tmp/tilt_math_test tests/host/tilt_math_test.c -lm \
 *     && /tmp/tilt_math_test
 */

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

/* Feed `count` paired events through one matrix and sum each axis. */
static void feed(struct rip_tilt_matrix m, int16_t x, int16_t y, int count, long *sum_x,
                 long *sum_y) {
    int32_t rem_x = 0;
    int32_t rem_y = 0;
    *sum_x = 0;
    *sum_y = 0;
    for (int i = 0; i < count; i++) {
        *sum_x += rip_tilt_apply_axis(m.xx, m.xy, x, y, &rem_x);
        *sum_y += rip_tilt_apply_axis(m.yx, m.yy, x, y, &rem_y);
        CHECK(labs(rem_x) < RIP_TILT_FIXED_ONE && labs(rem_y) < RIP_TILT_FIXED_ONE,
              "remainder out of range: %d %d", rem_x, rem_y);
    }
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

    /* Moving along the boosted 1:30 direction doubles; perpendicular is unchanged. */
    struct rip_tilt_matrix m = rip_tilt_compute_matrix(0, 60, 45);
    int32_t rx = 0;
    int32_t ry = 0;
    int16_t ox = rip_tilt_apply_axis(m.xx, m.xy, 10, -10, &rx);
    int16_t oy = rip_tilt_apply_axis(m.yx, m.yy, 10, -10, &ry);
    CHECK(ox == 20 && oy == -20, "along axis45: got (%d, %d) expected (20, -20)", ox, oy);
    rx = ry = 0;
    ox = rip_tilt_apply_axis(m.xx, m.xy, 10, 10, &rx);
    oy = rip_tilt_apply_axis(m.yx, m.yy, 10, 10, &ry);
    CHECK(ox == 10 && oy == 10, "across axis45: got (%d, %d) expected (10, 10)", ox, oy);

    /* The axis is 180-degree periodic. */
    struct rip_tilt_matrix a = rip_tilt_compute_matrix(0, 50, 30);
    struct rip_tilt_matrix b = rip_tilt_compute_matrix(0, 50, -150);
    CHECK(rip_tilt_matrix_equal(&a, &b), "axis 30 and -150 must match");
}

static void test_rotation_then_tilt(void) {
    /* Rotation +90 maps right to down; the 12-6 boost then doubles it. */
    struct rip_tilt_matrix m = rip_tilt_compute_matrix(90, 60, 0);
    check_matrix("rot90 tilt60 axis0", m, 0, -1000, 2000, 0);
    int32_t rx = 0;
    int32_t ry = 0;
    int16_t ox = rip_tilt_apply_axis(m.xx, m.xy, 7, 0, &rx);
    int16_t oy = rip_tilt_apply_axis(m.yx, m.yy, 7, 0, &ry);
    CHECK(ox == 0 && oy == 14, "rot90 tilt60 right: got (%d, %d) expected (0, 14)", ox, oy);

    /* Rotation alone keeps the historical x cos - y sin / x sin + y cos form. */
    check_matrix("rot90", rip_tilt_compute_matrix(90, 0, 0), 0, -1000, 1000, 0);
    check_matrix("rot-90", rip_tilt_compute_matrix(-90, 0, 0), 0, 1000, -1000, 0);
    check_matrix("rot180", rip_tilt_compute_matrix(180, 0, 0), -1000, 0, 0, -1000);
    check_matrix("rot30", rip_tilt_compute_matrix(30, 0, 0), 866, -500, 500, 866);
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

static void test_remainder_carry(void) {
    long sx;
    long sy;

    /* tilt 60 doubles: 1000 single counts along the boosted axis give 2000. */
    feed(rip_tilt_compute_matrix(0, 60, 0), 0, 1, 1000, &sx, &sy);
    CHECK(sx == 0 && labs(sy - 2000) <= 1, "tilt60 y carry: got (%ld, %ld)", sx, sy);
    feed(rip_tilt_compute_matrix(0, 60, 0), 1, 0, 1000, &sx, &sy);
    CHECK(sx == 1000 && sy == 0, "tilt60 x untouched: got (%ld, %ld)", sx, sy);

    /* Fractional gain must not be lost to truncation: 1.414 per count. */
    feed(rip_tilt_compute_matrix(0, 45, 0), 0, 1, 1000, &sx, &sy);
    CHECK(sx == 0 && labs(sy - 1414) <= 1, "tilt45 y carry: got (%ld, %ld)", sx, sy);
    feed(rip_tilt_compute_matrix(0, 45, 0), 0, -1, 1000, &sx, &sy);
    CHECK(sx == 0 && labs(sy + 1414) <= 1, "tilt45 -y carry: got (%ld, %ld)", sx, sy);

    /* Off-diagonal coupling smaller than one count still accumulates. */
    feed(rip_tilt_compute_matrix(0, 60, 30), 0, -1, 1000, &sx, &sy);
    CHECK(labs(sx - 433) <= 1 && labs(sy + 1750) <= 1, "tilt60 axis30 carry: got (%ld, %ld)", sx,
          sy);

    /* Plain rotation: the old truncating formula lost every 1-count step at
     * 30 degrees on x (866/1000 -> 0); the carry keeps 866 of 1000. */
    feed(rip_tilt_compute_matrix(30, 0, 0), 1, 0, 1000, &sx, &sy);
    CHECK(labs(sx - 866) <= 1 && labs(sy - 500) <= 1, "rot30 carry: got (%ld, %ld)", sx, sy);
}

/* Upstream: (int32_t)(cos(angle) * 1000.0) with pi = 3.14159265359, then
 * (x * cos - y * sin) / 1000 with C truncation. */
static void test_rotation_matches_old_formula(void) {
    for (int32_t deg = -360; deg <= 360; deg++) {
        double angle = (double)deg * 3.14159265359 / 180.0;
        int32_t old_cos = deg == 0 ? 1000 : (int32_t)(cos(angle) * 1000.0);
        int32_t old_sin = deg == 0 ? 0 : (int32_t)(sin(angle) * 1000.0);
        struct rip_tilt_matrix m = rip_tilt_compute_matrix(deg, 0, 0);
        checks++;
        if (labs(m.xx - old_cos) > 1 || labs(m.yy - old_cos) > 1 || labs(m.yx - old_sin) > 1 ||
            labs(m.xy + old_sin) > 1) {
            CHECK(0, "rot %d matrix differs: new [[%d,%d],[%d,%d]] old cos=%d sin=%d", deg, m.xx,
                  m.xy, m.yx, m.yy, old_cos, old_sin);
            continue;
        }
        for (int16_t x = -100; x <= 100; x += 25) {
            for (int16_t y = -100; y <= 100; y += 25) {
                int32_t old_x = (x * old_cos - y * old_sin) / 1000;
                int32_t old_y = (x * old_sin + y * old_cos) / 1000;
                int32_t rx = 0;
                int32_t ry = 0;
                int16_t new_x = rip_tilt_apply_axis(m.xx, m.xy, x, y, &rx);
                int16_t new_y = rip_tilt_apply_axis(m.yx, m.yy, x, y, &ry);
                if (labs(new_x - old_x) > 1 || labs(new_y - old_y) > 1) {
                    CHECK(0, "rot %d (%d, %d): new (%d, %d) old (%d, %d)", deg, x, y, new_x, new_y,
                          old_x, old_y);
                }
            }
        }
    }
}

static void test_no_overflow(void) {
    struct rip_tilt_matrix m = rip_tilt_compute_matrix(0, 60, 0);
    int32_t rem = 0;
    CHECK(rip_tilt_apply_axis(m.yx, m.yy, 0, INT16_MAX, &rem) == INT16_MAX && rem == 0,
          "clamp positive");
    rem = 0;
    CHECK(rip_tilt_apply_axis(m.yx, m.yy, 0, INT16_MIN, &rem) == INT16_MIN && rem == 0,
          "clamp negative");
    rem = 999;
    CHECK(rip_tilt_apply_axis(m.xx, m.xy, INT16_MAX, 0, &rem) == INT16_MAX, "carry at max");

    /* Every legal setting at the extreme inputs stays within int16 and keeps
     * the remainder in range. */
    static const int16_t extremes[] = {INT16_MIN, -1, 0, 1, INT16_MAX};
    for (int rot = -360; rot <= 360; rot += 45) {
        for (int tilt = 0; tilt <= 60; tilt += 15) {
            for (int axis = -75; axis <= 90; axis += 15) {
                struct rip_tilt_matrix t = rip_tilt_compute_matrix(rot, tilt, axis);
                CHECK(labs(t.xx) <= 2000 && labs(t.xy) <= 2000 && labs(t.yx) <= 2000 &&
                          labs(t.yy) <= 2000,
                      "matrix entry above 2.0 at rot %d tilt %d axis %d", rot, tilt, axis);
                for (size_t i = 0; i < 5; i++) {
                    for (size_t j = 0; j < 5; j++) {
                        int32_t r1 = 999;
                        int32_t r2 = -999;
                        (void)rip_tilt_apply_axis(t.xx, t.xy, extremes[i], extremes[j], &r1);
                        (void)rip_tilt_apply_axis(t.yx, t.yy, extremes[i], extremes[j], &r2);
                        if (labs(r1) >= RIP_TILT_FIXED_ONE || labs(r2) >= RIP_TILT_FIXED_ONE) {
                            CHECK(0, "remainder escaped range");
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
    test_remainder_carry();
    test_rotation_matches_old_formula();
    test_no_overflow();

    if (failures) {
        printf("tilt_math_test: %d of %d checks FAILED\n", failures, checks);
        return 1;
    }
    printf("tilt_math_test: all %d checks passed\n", checks);
    return 0;
}
