/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Pure logic for trackball gestures: while a gesture key is held the ball
 * does not move the cursor, and a quick flick fires the gesture assigned to
 * its direction, once.
 *
 * A flick is motion that covers `distance` sensor counts within `window_ms`
 * along one direction (the dominant axis, at least
 * RIP_GESTURE_DOMINANCE_PERCENT of the other), so rolling the ball slowly
 * (bringing the thumb back after a flick) never fires. It is measured from
 * where the ball last turned back on that axis, so a drag back just before a
 * flick does not eat into it.
 *
 * After a flick nothing fires while the ball keeps going (a free spin stays
 * locked). The flick's own direction and the perpendicular ones re-arm once
 * the ball turns back (the thumb dragging it back for the next flick) or has
 * been still for `still_ms`; the opposite direction only after it has been
 * still for RIP_GESTURE_REVERSE_EXTRA_MS longer, so a drag back never undoes
 * the gesture. A key press starts locked too: a ball already moving when the
 * key goes down has to stop first.
 *
 * Motion comes in as milli-counts in the cursor frame (x right, y down, after
 * the processor's rotation, inversion and swap; see the caller) and is kept
 * in RIP_GESTURE_SLOT_MS slots of a ring, so window sums cost a few adds and
 * no event is ever stored. The ring and the time the ball last moved are fed
 * whether the key is held or not, so a press knows whether the ball is still.
 *
 * Kept free of Zephyr dependencies so it can be unit-tested on the host
 * (see tests/host/gesture_math_test.c). Integer only.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Directions, also the order of the keycodes and of the RPC fields. */
#define RIP_GESTURE_LEFT 0
#define RIP_GESTURE_RIGHT 1
#define RIP_GESTURE_UP 2
#define RIP_GESTURE_DOWN 3
#define RIP_GESTURE_DIRS 4
#define RIP_GESTURE_NONE (-1)
#define RIP_GESTURE_ALL ((uint8_t)0x0F)

/* gesture-distance range in sensor counts (about 1 to 50 mm at 1200 CPI). */
#define RIP_GESTURE_MIN_DISTANCE 40
#define RIP_GESTURE_MAX_DISTANCE 2400

/* gesture-still-ms range. */
#define RIP_GESTURE_MIN_STILL_MS 50
#define RIP_GESTURE_MAX_STILL_MS 1000

/* Ring of motion slots; it must cover the longest window. */
#define RIP_GESTURE_SLOT_MS 10
#define RIP_GESTURE_SLOTS 32
#define RIP_GESTURE_MAX_WINDOW_MS ((RIP_GESTURE_SLOTS - 1) * RIP_GESTURE_SLOT_MS)

/* The ball counts as moving while the motion of the last
 * RIP_GESTURE_MOVING_MS adds up to RIP_GESTURE_MOVING_MILLI (6 counts in
 * 30 ms, about 4 mm/s at 1200 CPI). A resting thumb's sensor noise stays
 * below it. */
#define RIP_GESTURE_MOVING_MS 30
#define RIP_GESTURE_MOVING_MILLI 6000

/* The ball turned back on an axis when the last RIP_GESTURE_MOVING_MS moved
 * this far against the older motion there; sensor noise never does. */
#define RIP_GESTURE_TURN_MILLI 6000

/* A flick's dominant axis must be at least this percent of the other one
 * (130 = within about 37 degrees of up, down, left or right). */
#define RIP_GESTURE_DOMINANCE_PERCENT 130

/* The direction opposite to the last flick re-arms this much later. */
#define RIP_GESTURE_REVERSE_EXTRA_MS 250

/* A hold "used the ball" (so the tap of a tap & hold key is dropped) after
 * this much motion, about 0.5 mm at 1200 CPI, or once a gesture fired. */
#define RIP_GESTURE_USED_MILLI 24000

/* After the key goes up the cursor stays put until the ball has been still
 * this long (a flick released mid-spin does not throw the cursor), but no
 * longer than RIP_GESTURE_DRAIN_MAX_MS. */
#define RIP_GESTURE_DRAIN_STILL_MS 60
#define RIP_GESTURE_DRAIN_MAX_MS 300

#define RIP_GESTURE_MILLI 1000

struct rip_gesture_params {
    uint32_t distance;  /* sensor counts within the window */
    uint32_t still_ms;  /* the ball must be still this long before the next flick */
    uint32_t window_ms; /* a flick covers the distance within this long */
};

struct rip_gesture_slot {
    uint32_t index; /* slot number (ms / RIP_GESTURE_SLOT_MS); stale slots are skipped */
    int32_t dx;     /* milli-counts, cursor frame */
    int32_t dy;
    uint32_t moved; /* sum of |dx| + |dy| of the events, milli-counts */
};

/* Zero-initialize. */
struct rip_gesture {
    struct rip_gesture_slot slot[RIP_GESTURE_SLOTS];
    uint32_t last_moving_ms; /* when the ball last counted as moving */
    bool has_moved;          /* last_moving_ms is valid */
    uint8_t armed;           /* directions that may fire (bit per direction) */
    int8_t last_dir;         /* direction of the last flick in this hold, or NONE */
    bool fired;              /* a flick fired in this hold */
    uint32_t used;           /* motion in this hold, milli-counts (saturating) */
};

static inline uint16_t rip_gesture_clamp_distance(int64_t distance) {
    if (distance < RIP_GESTURE_MIN_DISTANCE) {
        return RIP_GESTURE_MIN_DISTANCE;
    }
    if (distance > RIP_GESTURE_MAX_DISTANCE) {
        return RIP_GESTURE_MAX_DISTANCE;
    }
    return (uint16_t)distance;
}

static inline uint16_t rip_gesture_clamp_still_ms(int64_t ms) {
    if (ms < RIP_GESTURE_MIN_STILL_MS) {
        return RIP_GESTURE_MIN_STILL_MS;
    }
    if (ms > RIP_GESTURE_MAX_STILL_MS) {
        return RIP_GESTURE_MAX_STILL_MS;
    }
    return (uint16_t)ms;
}

static inline int rip_gesture_opposite(int dir) { return dir ^ 1; }

static inline uint32_t rip_gesture_abs(int32_t v) {
    return v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

static inline int32_t rip_gesture_sat_add(int32_t a, int32_t b) {
    int64_t sum = (int64_t)a + b;
    if (sum > INT32_MAX) {
        return INT32_MAX;
    }
    if (sum < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)sum;
}

static inline uint32_t rip_gesture_sat_addu(uint32_t a, uint32_t b) {
    return UINT32_MAX - a < b ? UINT32_MAX : a + b;
}

/* Forget the motion in the ring (the time the ball last moved stays). */
static inline void rip_gesture_clear_window(struct rip_gesture *g) {
    memset(g->slot, 0, sizeof(g->slot));
}

/* Sum the slots of the last span_ms up to now_ms (the slot of now_ms
 * included). */
static inline void rip_gesture_sum(const struct rip_gesture *g, uint32_t now_ms, uint32_t span_ms,
                                   int64_t *sx, int64_t *sy, uint64_t *moved) {
    uint32_t newest = now_ms / RIP_GESTURE_SLOT_MS;
    uint32_t count = (span_ms + RIP_GESTURE_SLOT_MS - 1) / RIP_GESTURE_SLOT_MS;
    if (count < 1) {
        count = 1;
    }
    if (count > RIP_GESTURE_SLOTS) {
        count = RIP_GESTURE_SLOTS;
    }
    int64_t x = 0;
    int64_t y = 0;
    uint64_t m = 0;
    for (uint32_t k = 0; k < count; k++) {
        uint32_t index = newest - k;
        const struct rip_gesture_slot *s = &g->slot[index % RIP_GESTURE_SLOTS];
        if (s->index == index) {
            x += s->dx;
            y += s->dy;
            m += s->moved;
        }
    }
    *sx = x;
    *sy = y;
    *moved = m;
}

/* Sum the slots of the last RIP_GESTURE_MOVING_MS (recent) and of the rest of
 * the ring (older) separately. */
#define RIP_GESTURE_RECENT_SLOTS                                                                   \
    ((RIP_GESTURE_MOVING_MS + RIP_GESTURE_SLOT_MS - 1) / RIP_GESTURE_SLOT_MS)

/* Where the last RIP_GESTURE_MOVING_MS moved the ball against the older
 * motion of an axis, forget the older motion of that axis: a new stroke
 * starts there. Returns the recent motion along dir (negative = backwards). */
static inline int64_t rip_gesture_turn(struct rip_gesture *g, uint32_t now_ms, int dir) {
    uint32_t newest = now_ms / RIP_GESTURE_SLOT_MS;
    int64_t rx = 0;
    int64_t ry = 0;
    int64_t ox = 0;
    int64_t oy = 0;
    for (uint32_t k = 0; k < RIP_GESTURE_SLOTS; k++) {
        uint32_t index = newest - k;
        const struct rip_gesture_slot *s = &g->slot[index % RIP_GESTURE_SLOTS];
        if (s->index != index) {
            continue;
        }
        if (k < RIP_GESTURE_RECENT_SLOTS) {
            rx += s->dx;
            ry += s->dy;
        } else {
            ox += s->dx;
            oy += s->dy;
        }
    }
    bool turn_x = (rx >= RIP_GESTURE_TURN_MILLI && ox < 0) ||
                  (rx <= -RIP_GESTURE_TURN_MILLI && ox > 0);
    bool turn_y = (ry >= RIP_GESTURE_TURN_MILLI && oy < 0) ||
                  (ry <= -RIP_GESTURE_TURN_MILLI && oy > 0);
    if (turn_x || turn_y) {
        for (uint32_t k = RIP_GESTURE_RECENT_SLOTS; k < RIP_GESTURE_SLOTS; k++) {
            uint32_t index = newest - k;
            struct rip_gesture_slot *s = &g->slot[index % RIP_GESTURE_SLOTS];
            if (s->index != index) {
                continue;
            }
            if (turn_x) {
                s->dx = 0;
            }
            if (turn_y) {
                s->dy = 0;
            }
        }
    }

    switch (dir) {
    case RIP_GESTURE_LEFT:
        return -rx;
    case RIP_GESTURE_RIGHT:
        return rx;
    case RIP_GESTURE_UP:
        return -ry;
    case RIP_GESTURE_DOWN:
        return ry;
    default:
        return 0;
    }
}

/* How long the ball has been still at now_ms (UINT32_MAX if it never moved). */
static inline uint32_t rip_gesture_quiet_ms(const struct rip_gesture *g, uint32_t now_ms) {
    return g->has_moved ? now_ms - g->last_moving_ms : UINT32_MAX;
}

/* Add motion to the ring and note whether the ball is moving. */
static inline void rip_gesture_add_slot(struct rip_gesture *g, int32_t dx_milli, int32_t dy_milli,
                                        uint32_t moved_milli, uint32_t now_ms) {
    uint32_t index = now_ms / RIP_GESTURE_SLOT_MS;
    struct rip_gesture_slot *s = &g->slot[index % RIP_GESTURE_SLOTS];
    if (s->index != index) {
        *s = (struct rip_gesture_slot){.index = index};
    }
    s->dx = rip_gesture_sat_add(s->dx, dx_milli);
    s->dy = rip_gesture_sat_add(s->dy, dy_milli);
    s->moved = rip_gesture_sat_addu(s->moved, moved_milli);

    int64_t sx;
    int64_t sy;
    uint64_t moved;
    rip_gesture_sum(g, now_ms, RIP_GESTURE_MOVING_MS, &sx, &sy, &moved);
    if (moved >= RIP_GESTURE_MOVING_MILLI) {
        g->last_moving_ms = now_ms;
        g->has_moved = true;
    }
}

static inline void rip_gesture_add(struct rip_gesture *g, int32_t dx_milli, int32_t dy_milli,
                                   uint32_t now_ms) {
    rip_gesture_add_slot(
        g, dx_milli, dy_milli,
        rip_gesture_sat_addu(rip_gesture_abs(dx_milli), rip_gesture_abs(dy_milli)), now_ms);
}

/* Motion that another processor used (turned into scroll on its layer): it
 * only tells whether the ball is moving, so a gesture key pressed mid-scroll
 * still waits for the ball to stop. */
static inline void rip_gesture_note_motion(struct rip_gesture *g, uint32_t moved_milli,
                                           uint32_t now_ms) {
    rip_gesture_add_slot(g, 0, 0, moved_milli, now_ms);
}

/* Whether the cursor still stays put at now_ms after the key went up at
 * up_ms: until the ball has been still RIP_GESTURE_DRAIN_STILL_MS (before the
 * event being handled), at most RIP_GESTURE_DRAIN_MAX_MS. */
static inline bool rip_gesture_draining(const struct rip_gesture *g, uint32_t up_ms,
                                        uint32_t now_ms) {
    return now_ms - up_ms < RIP_GESTURE_DRAIN_MAX_MS &&
           rip_gesture_quiet_ms(g, now_ms) < RIP_GESTURE_DRAIN_STILL_MS;
}

/* A gesture key went down (the first of them). Starts locked: the first
 * event re-arms once the ball has been still long enough. */
static inline void rip_gesture_begin(struct rip_gesture *g) {
    g->armed = 0;
    g->last_dir = RIP_GESTURE_NONE;
    g->fired = false;
    g->used = 0;
}

/* The last gesture key went up. Returns whether the hold used the ball (a
 * flick fired, or it moved more than RIP_GESTURE_USED_MILLI). */
static inline bool rip_gesture_end(struct rip_gesture *g) {
    bool used = g->fired || g->used >= RIP_GESTURE_USED_MILLI;
    g->armed = 0;
    g->last_dir = RIP_GESTURE_NONE;
    g->fired = false;
    g->used = 0;
    return used;
}

/* Re-arm once the ball has been still: every direction after still_ms,
 * except the opposite of the last flick, which waits
 * RIP_GESTURE_REVERSE_EXTRA_MS longer. Uses the stillness before the event
 * being handled, and forgets the motion of the previous flick. */
static inline void rip_gesture_rearm(struct rip_gesture *g, const struct rip_gesture_params *p,
                                     uint32_t now_ms) {
    if (g->armed == RIP_GESTURE_ALL) {
        return;
    }
    uint32_t quiet = rip_gesture_quiet_ms(g, now_ms);
    if (quiet < p->still_ms) {
        return;
    }
    uint8_t armed = RIP_GESTURE_ALL;
    if (g->last_dir != RIP_GESTURE_NONE &&
        quiet - p->still_ms < RIP_GESTURE_REVERSE_EXTRA_MS) {
        armed &= (uint8_t)~(1u << rip_gesture_opposite(g->last_dir));
    }
    if (armed != g->armed) {
        g->armed = armed;
        rip_gesture_clear_window(g);
    }
}

/* The direction of the motion in the window if it is a flick, else NONE. */
static inline int rip_gesture_flick(const struct rip_gesture *g,
                                    const struct rip_gesture_params *p, uint32_t now_ms) {
    int64_t sx;
    int64_t sy;
    uint64_t moved;
    rip_gesture_sum(g, now_ms, p->window_ms, &sx, &sy, &moved);
    int64_t ax = sx < 0 ? -sx : sx;
    int64_t ay = sy < 0 ? -sy : sy;
    int dir;
    int64_t major;
    int64_t minor;
    if (ax >= ay) {
        dir = sx > 0 ? RIP_GESTURE_RIGHT : RIP_GESTURE_LEFT;
        major = ax;
        minor = ay;
    } else {
        dir = sy > 0 ? RIP_GESTURE_DOWN : RIP_GESTURE_UP;
        major = ay;
        minor = ax;
    }
    if (major < (int64_t)p->distance * RIP_GESTURE_MILLI ||
        major * 100 < minor * RIP_GESTURE_DOMINANCE_PERCENT) {
        return RIP_GESTURE_NONE;
    }
    return dir;
}

/*
 * Feed one motion event (milli-counts, cursor frame) at now_ms. Returns the
 * direction that fired, or RIP_GESTURE_NONE. Call it for every event of the
 * processor, held or not, so the ring and the stillness stay current.
 */
static inline int rip_gesture_event(struct rip_gesture *g, const struct rip_gesture_params *p,
                                    bool held, int32_t dx_milli, int32_t dy_milli,
                                    uint32_t now_ms) {
    if (held) {
        rip_gesture_rearm(g, p, now_ms);
        g->used = rip_gesture_sat_addu(
            g->used, rip_gesture_sat_addu(rip_gesture_abs(dx_milli), rip_gesture_abs(dy_milli)));
    }
    rip_gesture_add(g, dx_milli, dy_milli, now_ms);
    int64_t along_last = rip_gesture_turn(g, now_ms, g->last_dir);
    if (!held) {
        return RIP_GESTURE_NONE;
    }
    // The thumb drags the ball back after a flick: the next flick may come
    // (in the same or a perpendicular direction, not the drag's own)
    if (g->armed == 0 && g->last_dir != RIP_GESTURE_NONE &&
        along_last <= -RIP_GESTURE_TURN_MILLI) {
        g->armed = RIP_GESTURE_ALL & (uint8_t)~(1u << rip_gesture_opposite(g->last_dir));
    }
    if (g->armed == 0) {
        return RIP_GESTURE_NONE;
    }

    int dir = rip_gesture_flick(g, p, now_ms);
    if (dir == RIP_GESTURE_NONE || !(g->armed & (1u << dir))) {
        return RIP_GESTURE_NONE;
    }
    // Once per flick: locked until the ball is still again, and the motion
    // that fired it is not counted twice
    g->armed = 0;
    g->last_dir = (int8_t)dir;
    g->fired = true;
    rip_gesture_clear_window(g);
    return dir;
}
