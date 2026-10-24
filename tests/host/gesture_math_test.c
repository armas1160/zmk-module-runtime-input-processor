/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Host-side unit test for src/pointing/rip_gesture_math.h (no Zephyr needed).
 *
 * Run from the repository root (test.py runs the same in CI):
 *   cc -std=c11 -Wall -Wextra -Werror -o /tmp/gesture_math_test tests/host/gesture_math_test.c \
 *     && /tmp/gesture_math_test
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../src/pointing/rip_gesture_math.h"

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

/* Firmware defaults: 7 mm at 1200 CPI within the Kconfig window, 150 ms still. */
static const struct rip_gesture_params P = {.distance = 330, .still_ms = 150, .window_ms = 150};

/* The sensor reports both axes every REPORT ms (6 on USB, about 15 on BLE). */
#define REPORT 8

struct sim {
    struct rip_gesture g;
    uint32_t now;
    int fired[RIP_GESTURE_DIRS];
    int fires;
    int last;
};

static void sim_init(struct sim *s, uint32_t start) {
    *s = (struct sim){.now = start, .last = RIP_GESTURE_NONE};
}

/* One report: X then Y, as the driver sends them. */
static void report(struct sim *s, bool held, int dx, int dy) {
    int d = rip_gesture_event(&s->g, &P, held, dx * RIP_GESTURE_MILLI, 0, s->now);
    if (d == RIP_GESTURE_NONE) {
        d = rip_gesture_event(&s->g, &P, held, 0, dy * RIP_GESTURE_MILLI, s->now);
    } else {
        // The Y half of the same report can not fire again (locked)
        int again = rip_gesture_event(&s->g, &P, held, 0, dy * RIP_GESTURE_MILLI, s->now);
        CHECK(again == RIP_GESTURE_NONE, "fired twice within one report");
    }
    if (d != RIP_GESTURE_NONE) {
        s->fired[d]++;
        s->fires++;
        s->last = d;
    }
}

/* Move total (dx, dy) counts over ms, one report every REPORT ms, spread evenly. */
static void move(struct sim *s, bool held, int dx, int dy, uint32_t ms) {
    int steps = (int)(ms / REPORT);
    if (steps < 1) {
        steps = 1;
    }
    int sent_x = 0;
    int sent_y = 0;
    for (int i = 1; i <= steps; i++) {
        s->now += REPORT;
        int x = dx * i / steps - sent_x;
        int y = dy * i / steps - sent_y;
        sent_x += x;
        sent_y += y;
        report(s, held, x, y);
    }
}

/* A flick: most of the motion early, then the ball slows down (decays). */
static void flick(struct sim *s, int dx, int dy) {
    move(s, true, dx * 7 / 10, dy * 7 / 10, 64);
    move(s, true, dx * 2 / 10, dy * 2 / 10, 64);
    move(s, true, dx / 10, dy / 10, 96);
}

static void wait_ms(struct sim *s, uint32_t ms) { s->now += ms; }

static void press(struct sim *s) { rip_gesture_begin(&s->g); }

static void test_clamp(void) {
    CHECK(rip_gesture_clamp_distance(0) == RIP_GESTURE_MIN_DISTANCE, "distance low");
    CHECK(rip_gesture_clamp_distance(330) == 330, "distance kept");
    CHECK(rip_gesture_clamp_distance(100000) == RIP_GESTURE_MAX_DISTANCE, "distance high");
    CHECK(rip_gesture_clamp_still_ms(-5) == RIP_GESTURE_MIN_STILL_MS, "still low");
    CHECK(rip_gesture_clamp_still_ms(150) == 150, "still kept");
    CHECK(rip_gesture_clamp_still_ms(5000) == RIP_GESTURE_MAX_STILL_MS, "still high");
    CHECK(rip_gesture_opposite(RIP_GESTURE_LEFT) == RIP_GESTURE_RIGHT &&
              rip_gesture_opposite(RIP_GESTURE_RIGHT) == RIP_GESTURE_LEFT &&
              rip_gesture_opposite(RIP_GESTURE_UP) == RIP_GESTURE_DOWN &&
              rip_gesture_opposite(RIP_GESTURE_DOWN) == RIP_GESTURE_UP,
          "opposites");
    CHECK(P.window_ms <= RIP_GESTURE_MAX_WINDOW_MS, "window fits the ring");
}

/* Each direction fires once, with y down (HID) meaning DOWN. */
static void test_directions(void) {
    const int vec[RIP_GESTURE_DIRS][2] = {{-600, 0}, {600, 0}, {0, -600}, {0, 600}};
    for (int d = 0; d < RIP_GESTURE_DIRS; d++) {
        struct sim s;
        sim_init(&s, 100000);
        press(&s);
        flick(&s, vec[d][0], vec[d][1]);
        CHECK(s.fires == 1 && s.fired[d] == 1, "direction %d: %d fires, last %d", d, s.fires,
              s.last);
    }
}

/* A flick fires as soon as it crosses the distance, then the spinning ball
 * stays locked; no second fire however long it spins. */
static void test_once_per_flick(void) {
    struct sim s;
    sim_init(&s, 5000);
    press(&s);
    s.now += REPORT;
    report(&s, true, 60, 0);
    CHECK(s.fires == 0, "one report of 60 is not a flick");
    for (int i = 0; i < 5; i++) {
        s.now += REPORT;
        report(&s, true, 60, 0);
    }
    CHECK(s.fires == 1 && s.fired[RIP_GESTURE_RIGHT] == 1, "fired at 360 counts");
    // A long free spin, slowing down
    for (int v = 60; v > 2; v -= 1) {
        s.now += REPORT;
        report(&s, true, v, 0);
    }
    CHECK(s.fires == 1, "spin kept locked: %d fires", s.fires);
}

/* Slow rolling never fires, however far. */
static void test_slow_roll(void) {
    struct sim s;
    sim_init(&s, 7000);
    press(&s);
    move(&s, true, 3000, 0, 3000); // 1000 counts per second, about 21 mm/s
    CHECK(s.fires == 0, "slow roll fired %d", s.fires);
    move(&s, true, 0, -1500, 1500);
    CHECK(s.fires == 0, "slow roll up fired %d", s.fires);
}

/* Just under the distance within the window does not fire; just over does. */
static void test_distance_edge(void) {
    struct sim s;
    sim_init(&s, 9000);
    press(&s);
    move(&s, true, 320, 0, 96);
    CHECK(s.fires == 0, "320 counts fired");
    wait_ms(&s, 400);
    move(&s, true, 340, 0, 96);
    CHECK(s.fires == 1, "340 counts did not fire");
}

/* Flick, stop, flick again in the same direction: two gestures. */
static void test_repeat_after_stop(void) {
    struct sim s;
    sim_init(&s, 20000);
    press(&s);
    flick(&s, 700, 0);
    wait_ms(&s, 160);
    flick(&s, 700, 0);
    wait_ms(&s, 160);
    flick(&s, 700, 0);
    CHECK(s.fired[RIP_GESTURE_RIGHT] == 3 && s.fires == 3, "three flicks: %d", s.fires);
}

/* Stopping for less than still_ms is not enough. */
static void test_short_stop(void) {
    struct sim s;
    sim_init(&s, 30000);
    press(&s);
    flick(&s, 700, 0);
    wait_ms(&s, 100);
    flick(&s, 700, 0);
    CHECK(s.fires == 1, "fired after a 100 ms stop: %d", s.fires);
}

/* Flick right, stop briefly, drag back left quickly: the drag back does not
 * undo the gesture. A deliberate left flick later still works. */
static void test_drag_back(void) {
    struct sim s;
    sim_init(&s, 40000);
    press(&s);
    flick(&s, 700, 0);
    wait_ms(&s, 200); // over still_ms, under still_ms + RIP_GESTURE_REVERSE_EXTRA_MS
    move(&s, true, -500, 0, 120);
    CHECK(s.fires == 1 && s.fired[RIP_GESTURE_LEFT] == 0, "drag back fired left");
    // Drag back without a pause: the ball never stopped
    flick(&s, 700, 0);
    move(&s, true, -800, 0, 120);
    CHECK(s.fired[RIP_GESTURE_LEFT] == 0, "drag back without a pause fired left");
    wait_ms(&s, P.still_ms + RIP_GESTURE_REVERSE_EXTRA_MS + 10);
    flick(&s, -700, 0);
    CHECK(s.fired[RIP_GESTURE_LEFT] == 1, "deliberate left flick did not fire");
}

/* Ratcheting: flick, drag the ball back, flick again, with no pause at all.
 * Each flick fires; no drag back ever does. */
static void test_ratchet(void) {
    struct sim s;
    sim_init(&s, 42000);
    press(&s);
    for (int i = 0; i < 3; i++) {
        flick(&s, 700, 0);
        move(&s, true, -450, 0, 200); // drag back, about 45 mm/s
    }
    CHECK(s.fired[RIP_GESTURE_RIGHT] == 3 && s.fired[RIP_GESTURE_LEFT] == 0,
          "ratchet: right %d left %d", s.fired[RIP_GESTURE_RIGHT], s.fired[RIP_GESTURE_LEFT]);

    // A quick drag back right after the flick (faster than a flick) still
    // does not fire the opposite way
    sim_init(&s, 43000);
    press(&s);
    flick(&s, 0, 700);
    move(&s, true, 0, -900, 120);
    CHECK(s.fired[RIP_GESTURE_DOWN] == 1 && s.fired[RIP_GESTURE_UP] == 0, "fast drag back fired");

    // A slight wobble against the spin (under the turn threshold) does not
    // re-arm the flick's direction
    sim_init(&s, 44000);
    press(&s);
    move(&s, true, 420, 0, 48);
    CHECK(s.fires == 1, "first flick");
    for (int i = 0; i < 10; i++) {
        s.now += REPORT;
        report(&s, true, (i == 5) ? -2 : 30, 0);
    }
    CHECK(s.fires == 1, "wobble re-armed: %d", s.fires);
}

/* A flick is measured from where the ball turned: a small wind-up the other
 * way just before it does not eat into it. */
static void test_windup(void) {
    struct sim s;
    sim_init(&s, 46000);
    press(&s);
    move(&s, true, -200, 0, 96); // wind-up, under the distance
    CHECK(s.fires == 0, "wind-up fired");
    move(&s, true, 360, 0, 56); // just over the distance from the turn
    CHECK(s.fires == 1 && s.fired[RIP_GESTURE_RIGHT] == 1, "flick after a wind-up: %d", s.fires);

    // Same on the other axis, after a ratchet drag back
    sim_init(&s, 47000);
    press(&s);
    flick(&s, 0, -700);
    move(&s, true, 0, 300, 140);
    move(&s, true, 0, -360, 56);
    CHECK(s.fired[RIP_GESTURE_UP] == 2 && s.fired[RIP_GESTURE_DOWN] == 0,
          "up after a drag back: up %d down %d", s.fired[RIP_GESTURE_UP],
          s.fired[RIP_GESTURE_DOWN]);
}

/* A perpendicular flick right after the stop is fine (only the opposite waits). */
static void test_perpendicular_after_stop(void) {
    struct sim s;
    sim_init(&s, 45000);
    press(&s);
    flick(&s, 700, 0);
    wait_ms(&s, 160);
    flick(&s, 0, -700);
    CHECK(s.fired[RIP_GESTURE_RIGHT] == 1 && s.fired[RIP_GESTURE_UP] == 1, "right then up");
}

/* Diagonal flicks are ambiguous and do not fire; a slight angle does. */
static void test_diagonal(void) {
    struct sim s;
    sim_init(&s, 50000);
    press(&s);
    flick(&s, 600, -600);
    CHECK(s.fires == 0, "45 degree flick fired %d", s.fires);
    wait_ms(&s, 600);
    flick(&s, 600, -250); // about 23 degrees
    CHECK(s.fires == 1 && s.fired[RIP_GESTURE_RIGHT] == 1, "23 degree flick: %d", s.fires);
}

/* Pressing while the ball is still moving: nothing fires until it stops. */
static void test_press_while_moving(void) {
    struct sim s;
    sim_init(&s, 60000);
    move(&s, false, 900, 0, 200); // moving the cursor, key up
    press(&s);
    move(&s, true, 900, 0, 120); // same motion carries on
    CHECK(s.fires == 0, "fired while already moving: %d", s.fires);
    wait_ms(&s, 160);
    flick(&s, 700, 0);
    CHECK(s.fires == 1, "flick after the stop: %d", s.fires);
}

/* Pressing after the ball has been still: the first flick fires at once. */
static void test_press_when_still(void) {
    struct sim s;
    sim_init(&s, 70000);
    move(&s, false, 900, 0, 200);
    wait_ms(&s, 500);
    press(&s);
    flick(&s, 0, 700);
    CHECK(s.fires == 1 && s.fired[RIP_GESTURE_DOWN] == 1, "first flick: %d", s.fires);
}

/* Sensor noise from a resting thumb neither fires nor keeps the ball "moving". */
static void test_jitter(void) {
    struct sim s;
    sim_init(&s, 80000);
    press(&s);
    flick(&s, 700, 0);
    for (int i = 0; i < 40; i++) {
        s.now += 20;
        report(&s, true, (i & 1) ? 1 : -1, (i & 2) ? 1 : 0);
    }
    CHECK(s.fires == 1, "jitter fired: %d", s.fires);
    // The jitter did not keep it locked: the next flick fires
    flick(&s, 700, 0);
    CHECK(s.fires == 2, "flick after jitter: %d", s.fires);
}

/* Not held: nothing ever fires, but the stillness is tracked. */
static void test_not_held(void) {
    struct sim s;
    sim_init(&s, 90000);
    move(&s, false, 2000, 0, 100);
    CHECK(s.fires == 0, "fired without the key");
    CHECK(rip_gesture_quiet_ms(&s.g, s.now) == 0, "moving right now");
    wait_ms(&s, 300);
    CHECK(rip_gesture_quiet_ms(&s.g, s.now) == 300, "still for 300 ms");
}

/* Whether the hold used the ball (for the tap of a tap & hold key). */
static void test_used(void) {
    struct sim s;
    sim_init(&s, 100000);
    press(&s);
    s.now += 10;
    report(&s, true, 3, 2);
    CHECK(!rip_gesture_end(&s.g), "a touch is not use");

    press(&s);
    move(&s, true, 30, 0, 80);
    CHECK(rip_gesture_end(&s.g), "30 counts is use");

    press(&s);
    wait_ms(&s, 300);
    flick(&s, 700, 0);
    CHECK(rip_gesture_end(&s.g), "a gesture is use");

    // end() unlocks nothing: the next hold starts locked again
    CHECK(s.g.armed == 0 && s.g.last_dir == RIP_GESTURE_NONE, "end resets");
}

/* A new hold starts fresh: the previous hold's last flick does not block the
 * opposite direction. */
static void test_new_hold(void) {
    struct sim s;
    sim_init(&s, 110000);
    press(&s);
    flick(&s, 700, 0);
    rip_gesture_end(&s.g);
    wait_ms(&s, 160);
    press(&s);
    flick(&s, -700, 0);
    CHECK(s.fired[RIP_GESTURE_LEFT] == 1, "left after a new press: %d", s.fired[RIP_GESTURE_LEFT]);
}

/* Huge reports neither overflow nor fire twice. */
static void test_extremes(void) {
    struct sim s;
    sim_init(&s, 120000);
    press(&s);
    for (int i = 0; i < 100; i++) {
        s.now += 1;
        report(&s, true, 32767, 0);
    }
    CHECK(s.fires == 1, "extreme motion: %d fires", s.fires);
    // Timer wraparound: the stillness spans the 32-bit wrap
    sim_init(&s, UINT32_MAX - 1000);
    move(&s, false, 900, 0, 200);
    press(&s);
    wait_ms(&s, 1500); // wraps past zero
    flick(&s, 700, 0);
    CHECK(s.fires == 1, "across the wrap: %d", s.fires);
}

/* Motion another processor used (scroll) only marks the ball as moving: a
 * press mid-scroll waits for the ball to stop, and nothing is summed. */
static void test_note_motion(void) {
    struct sim s;
    sim_init(&s, 140000);
    for (int i = 0; i < 20; i++) {
        s.now += REPORT;
        rip_gesture_note_motion(&s.g, 40 * RIP_GESTURE_MILLI, s.now);
    }
    CHECK(rip_gesture_quiet_ms(&s.g, s.now) == 0, "scroll counts as moving");
    press(&s);
    move(&s, true, 400, 0, 48); // the scroll motion goes on as a fast stroke
    CHECK(s.fires == 0, "fired while the ball was scrolling: %d", s.fires);
    wait_ms(&s, 200);
    flick(&s, 700, 0);
    CHECK(s.fires == 1, "after the stop: %d", s.fires);

    // Noted motion never adds to a flick
    sim_init(&s, 150000);
    wait_ms(&s, 500);
    press(&s);
    s.now += REPORT;
    report(&s, true, 0, 0);
    for (int i = 0; i < 20; i++) {
        s.now += REPORT;
        rip_gesture_note_motion(&s.g, 400 * RIP_GESTURE_MILLI, s.now);
    }
    CHECK(s.fires == 0, "noted motion fired");
}

/* After the key goes up the cursor stays put until the ball stops, at most
 * RIP_GESTURE_DRAIN_MAX_MS. */
static void test_draining(void) {
    struct sim s;
    sim_init(&s, 160000);
    press(&s);
    flick(&s, 700, 0);
    rip_gesture_end(&s.g);
    uint32_t up = s.now;
    CHECK(rip_gesture_draining(&s.g, up, s.now + 8), "spinning at release");
    s.now += 8;
    report(&s, false, 20, 0);
    CHECK(rip_gesture_draining(&s.g, up, s.now + 8), "still spinning");
    CHECK(!rip_gesture_draining(&s.g, up, s.now + RIP_GESTURE_DRAIN_STILL_MS + 8),
          "stopped for the still time");
    // A ball that never stops: released at most RIP_GESTURE_DRAIN_MAX_MS later
    for (int i = 0; i < 60; i++) {
        s.now += REPORT;
        report(&s, false, 20, 0);
    }
    CHECK(!rip_gesture_draining(&s.g, up, s.now), "drain capped at the max");
    // Released after the ball had already stopped: nothing to drain
    sim_init(&s, 170000);
    press(&s);
    flick(&s, 700, 0);
    wait_ms(&s, 200);
    rip_gesture_end(&s.g);
    CHECK(!rip_gesture_draining(&s.g, s.now, s.now + 8), "nothing to drain");
}

/* Reports on BLE come about every 15 ms, often batched: the window still sees
 * a flick. */
static void test_ble_pacing(void) {
    struct sim s;
    sim_init(&s, 130000);
    press(&s);
    for (int i = 0; i < 4; i++) {
        s.now += 15;
        report(&s, true, 0, -120);
    }
    CHECK(s.fires == 1 && s.fired[RIP_GESTURE_UP] == 1, "BLE paced flick up: %d", s.fires);
}

int main(void) {
    test_clamp();
    test_directions();
    test_once_per_flick();
    test_slow_roll();
    test_distance_edge();
    test_repeat_after_stop();
    test_short_stop();
    test_drag_back();
    test_ratchet();
    test_windup();
    test_perpendicular_after_stop();
    test_diagonal();
    test_press_while_moving();
    test_press_when_still();
    test_jitter();
    test_not_held();
    test_used();
    test_new_hold();
    test_extremes();
    test_note_motion();
    test_draining();
    test_ble_pacing();

    if (failures) {
        printf("gesture_math_test: %d of %d checks FAILED\n", failures, checks);
        return 1;
    }
    printf("gesture_math_test: all %d checks passed\n", checks);
    return 0;
}
