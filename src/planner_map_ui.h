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
 * 2D map pushback planner window.
 *
 * Shows the airport (pavement, painted lines, stands, runways) from apt.dat
 * with the planned pushback route on top, and lets the pilot plan with
 * pointer clicks and large buttons. The window is an XPImgWindow, so X-Plane
 * places it in the cockpit when VR is enabled - which the legacy top-down
 * camera planner cannot do.
 */

#ifndef _PLANNER_MAP_UI_H_
#define _PLANNER_MAP_UI_H_

#include <acfutils/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PLANNER_MAP_MODE_AUTO = 0,      /* map window in VR, camera otherwise */
    PLANNER_MAP_MODE_ALWAYS = 1,    /* always use the map window */
    PLANNER_MAP_MODE_NEVER = 2      /* always use the top-down camera */
} planner_map_mode_t;

#define PLANNER_MAP_MODE_CONF_KEY "planner_map_mode"
#define PLANNER_MAP_AUTO_APPLY_CONF_KEY "planner_auto_apply_procedure"

/* Whether the next planner session should use the map window. */
bool_t planner_map_ui_wanted(void);
bool_t planner_map_ui_vr_enabled(void);

planner_map_mode_t planner_map_ui_get_mode(void);
void planner_map_ui_set_mode(planner_map_mode_t mode);
bool_t planner_map_ui_get_auto_apply(void);
void planner_map_ui_set_auto_apply(bool_t flag);

/* Called by bp_cam_start()/bp_cam_stop() in map mode. */
void planner_map_ui_open(void);
void planner_map_ui_close(void);

/* Releases the window, the loader thread and cached airport data. */
void planner_map_ui_fini(void);

#ifdef __cplusplus
}
#endif

#endif
