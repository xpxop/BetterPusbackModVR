/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2026 xpxop. All rights reserved.
 */
/*
 * Camera-independent planner interface.
 *
 * The legacy top-down planner (bp_cam.c) drives the route planner from the
 * X-Plane camera, a fullscreen invisible window and a key sniffer, none of
 * which work in VR. These functions expose the same planner operations so
 * the 2D map planner window (planner_map_ui.cpp) can drive them with
 * pointer clicks and buttons. They only act while the planner is open and
 * must be called from the X-Plane main thread.
 */

#ifndef _PLANNER_API_H_
#define _PLANNER_API_H_

#include <stddef.h>

#include <acfutils/geom.h>
#include <acfutils/list.h>
#include <acfutils/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BP_PLANNER_PROMPT_NONE,
    BP_PLANNER_PROMPT_SELECT,   /* choose a saved route or plan new */
    BP_PLANNER_PROMPT_REPLACE   /* both saved slots occupied */
} bp_planner_prompt_t;

bool_t bp_planner_is_open(void);
/* True when the planner runs in the map window instead of the camera. */
bool_t bp_planner_map_mode(void);

/* Live aircraft position (local OpenGL x, -z) and true heading. */
void bp_planner_aircraft_pose(vect2_t *pos, double *hdg);
/* Start of the next segment: end of the committed route or the aircraft. */
void bp_planner_next_start(vect2_t *pos, double *hdg);

/* Committed (accepted) and predicted segment lists (seg_t elements). */
const list_t *bp_planner_segs(void);
const list_t *bp_planner_pred_segs(void);

/*
 * Predicts a route from the next start to `end_pos`/`end_hdg`. Returns
 * the number of predicted segments (0 when unreachable).
 */
int bp_planner_predict(vect2_t end_pos, double end_hdg);
void bp_planner_clear_prediction(void);
/* Moves the prediction into the committed route. */
bool_t bp_planner_commit_prediction(void);
/* Deletes back to the previous user-placed segment. */
void bp_planner_undo(void);
void bp_planner_clear(void);
/* Accept the route and close the planner (may raise the REPLACE prompt). */
void bp_planner_accept(void);
/* Discard the route and close the planner. */
void bp_planner_cancel(void);
/* Discard the route and connect the tug first. */
void bp_planner_connect_first(void);

/*
 * Replaces the committed route with a chain of target poses (each pose is
 * planned with the legacy segment solver from the previous one). Returns
 * B_FALSE and leaves the route empty if any piece cannot be planned.
 */
bool_t bp_planner_apply_poses(const vect2_t *pos, const double *hdg,
    size_t n);

/* Saved-route prompt. */
bp_planner_prompt_t bp_planner_prompt(void);
unsigned bp_planner_prompt_option_count(void);
bool_t bp_planner_prompt_option_enabled(unsigned choice);
void bp_planner_prompt_option_text(unsigned choice, char *text, size_t cap);
void bp_planner_prompt_title(char *title, size_t title_cap,
    char *instruction, size_t instruction_cap);
void bp_planner_prompt_choose(unsigned choice);

/* Last informational planner message (may be empty). */
const char *bp_planner_message(void);

/*
 * Published stand the aircraft started from (apt.dat 1300 row). Returns
 * B_FALSE if the start position was not recognized.
 */
bool_t bp_planner_gate(char *airport, size_t airport_cap, char *ramp,
    size_t ramp_cap);
/* Nearest airport ident (may be empty). */
void bp_planner_airport(char *icao, size_t cap);

/*
 * Aircraft outline transformed to the given pose. Writes pairs of points
 * (line segments) to `out`; returns the number of points written.
 */
size_t bp_planner_acf_outline(vect2_t pos, double hdg, vect2_t *out,
    size_t max_pts);
double bp_planner_acf_semispan(void);

/* Marks the committed route as edited (for saved-route bookkeeping). */
void bp_planner_mark_dirty(void);

#ifdef __cplusplus
}
#endif

#endif
