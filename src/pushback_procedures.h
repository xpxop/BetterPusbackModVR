/*
 * Default pushback procedures per airport stand.
 *
 * Procedures are aircraft independent: each one is a short list of steps
 * ("push back 40 m", "turn so the tail points east", ...) that is resolved
 * against the aircraft's actual start pose and then fed to the legacy
 * segment planner (compute_segs) for the current aircraft. That way one
 * entry, transcribed from an airport's published pushback procedure (AIP
 * AD 2.20, ground handling manual, ...), works for any aircraft type.
 *
 * File format (one file per airport, <ICAO>.txt, '#' starts a comment):
 *
 *   stand <pattern>        start of a procedure; pattern matches the 1300
 *                          ramp-start name, '*' and '?' wildcards allowed,
 *                          case-insensitive. The pattern is also tried
 *                          against the last word of the stand name, so
 *                          "A12" matches "Gate A12".
 *   name <text>            short label shown in the planner
 *   source <text>          where the procedure comes from (optional)
 *   back <meters>          straight pushback
 *   forward <meters>       straight pull forward
 *   turn face <dir> [radius <m>] [cw|ccw] [forward]
 *   turn tail <dir> [radius <m>] [cw|ccw] [forward]
 *                          turn while pushing (or pulling with 'forward')
 *                          until the nose ('face') or tail ('tail') points
 *                          towards <dir>. <dir> is a compass point (N, NE,
 *                          ENE, ...), a true heading in degrees (090), a
 *                          relative change (+90 / -45), or 'left'/'right'
 *                          optionally followed by an angle (default 90):
 *                          "turn tail left" swings the tail to the pilot's
 *                          left by 90 degrees. cw/ccw force the direction
 *                          in which the nose heading changes; otherwise the
 *                          shortest turn is used.
 *   end                    end of the procedure
 *
 * This module has no XPLM/libacfutils dependencies so it can be unit tested.
 */

#ifndef _PUSHBACK_PROCEDURES_H_
#define _PUSHBACK_PROCEDURES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PB_PROC_PATTERN_LEN 40
#define PB_PROC_NAME_LEN 96
#define PB_PROC_SOURCE_LEN 128
#define PB_PROC_MAX_STEPS 24
#define PB_PROC_MAX_POSES (PB_PROC_MAX_STEPS * 4)
#define PB_PROC_DEFAULT_RADIUS 30.0
#define PB_PROC_ERR_LEN 160

typedef enum {
    PB_PROC_STEP_STRAIGHT,
    PB_PROC_STEP_TURN
} pb_proc_step_type_t;

typedef enum {
    PB_PROC_DIR_ABSOLUTE,   /* value is a true heading */
    PB_PROC_DIR_RELATIVE    /* value is a signed change of the nose hdg */
} pb_proc_dir_kind_t;

typedef struct {
    pb_proc_step_type_t type;
    bool forward;           /* pull forward instead of pushing back */
    double dist;            /* straight: meters */
    bool tail;              /* turn: value refers to the tail direction */
    pb_proc_dir_kind_t dir_kind;
    double value;
    int rotation;           /* 0 = shortest, +1 = cw, -1 = ccw */
    double radius;          /* 0 = default */
} pb_proc_step_t;

typedef struct {
    char pattern[PB_PROC_PATTERN_LEN];
    char name[PB_PROC_NAME_LEN];
    char source[PB_PROC_SOURCE_LEN];
    pb_proc_step_t steps[PB_PROC_MAX_STEPS];
    unsigned n_steps;
    bool user;              /* loaded from the user override directory */
} pb_proc_t;

typedef struct {
    pb_proc_t *procs;
    size_t n, cap;
} pb_proc_set_t;

typedef struct {
    double x;               /* meters east */
    double y;               /* meters north */
    double hdg;             /* true nose heading */
} pb_proc_pose_t;

/* Generic description of a planned segment, used to save procedures. */
typedef struct {
    bool turn;
    bool backward;
    double len;             /* straight length */
    double radius;          /* turn radius */
    double start_hdg;
    double end_hdg;
} pb_proc_seg_t;

void pb_proc_set_init(pb_proc_set_t *set);
void pb_proc_set_free(pb_proc_set_t *set);

/*
 * Parses procedures from a stream and appends them to `set`. On a syntax
 * error returns false and describes it in `err` (with line number); the
 * procedures parsed before the error are kept.
 */
bool pb_proc_parse_stream(FILE *fp, bool user, pb_proc_set_t *set,
    char *err, size_t err_cap);
bool pb_proc_load_file(const char *path, bool user, pb_proc_set_t *set,
    char *err, size_t err_cap);

bool pb_proc_pattern_matches(const char *pattern, const char *stand);

/*
 * Collects the procedures applicable to `stand`: user procedures first,
 * then exact pattern matches before wildcard matches. Returns the number of
 * matches (may exceed max_out; only max_out are stored).
 */
size_t pb_proc_match(const pb_proc_set_t *set, const char *stand,
    const pb_proc_t **out, size_t max_out);

/*
 * Resolves a procedure into a chain of target poses starting at `start`.
 * Turns are split into pieces of at most 90 degrees so each piece can be
 * planned by a single compute_segs() call. Returns the number of poses or
 * -1 if the procedure is empty or too long.
 */
int pb_proc_resolve(const pb_proc_t *proc, pb_proc_pose_t start,
    pb_proc_pose_t *poses, bool *forward, size_t max_poses);

/* Builds a procedure from planned segments (merging straight runs). */
void pb_proc_from_segs(const pb_proc_seg_t *segs, size_t n,
    const char *pattern, const char *name, const char *source,
    pb_proc_t *out);

bool pb_proc_write(FILE *fp, const pb_proc_t *proc);

/*
 * Replaces every user procedure in `set` whose pattern equals
 * `proc->pattern` with `proc` (placed first) and writes all user procedures
 * to `path`. Built-in (non-user) procedures are never written.
 */
bool pb_proc_save_user(pb_proc_set_t *set, const pb_proc_t *proc,
    const char *path);

double pb_proc_normalize_hdg(double hdg);

#ifdef __cplusplus
}
#endif

#endif
