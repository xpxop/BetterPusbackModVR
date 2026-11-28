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
 * Airport map data for the 2D (VR-capable) pushback planner.
 *
 * This module is intentionally free of XPLM and libacfutils dependencies so
 * it can run on a worker thread and be unit tested on its own. It extracts
 * one airport from X-Plane apt.dat files and converts it into flat geometry
 * expressed in meters east (x) / north (y) of a reference point.
 *
 * Parsed apt.dat rows:
 *   1/16/17  airport header (ident is matched)
 *   100      land runways (drawn as filled quads)
 *   110      pavement (taxiway/apron) polygons incl. holes, with Beziers
 *   120      linear features (painted lines), with Beziers
 *   111-116  nodes for 110/120/130 chains
 *   1201/1202 taxi routing network (drawn when no painted lines exist;
 *            its taxiway names are always used for map labels)
 *   1300     ramp starts (stands/gates)
 */

#ifndef _AIRPORT_MAP_H_
#define _AIRPORT_MAP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AIRPORT_MAP_IDENT_LEN 16
#define AIRPORT_MAP_NAME_LEN 40
#define AIRPORT_MAP_PATH_LEN 1024

typedef enum {
    AIRPORT_MAP_LINE_YELLOW = 0,  /* taxi centerlines, hold lines, ... */
    AIRPORT_MAP_LINE_WHITE,       /* roadway / service markings */
    AIRPORT_MAP_LINE_OTHER,       /* any other painted line type */
    AIRPORT_MAP_LINE_TAXI_ROUTE,  /* 1202 taxi routing edge */
    AIRPORT_MAP_LINE_PAVEMENT,    /* pavement outline */
    AIRPORT_MAP_LINE_ORANGE,      /* type 90: orange offset taxi lane */
    AIRPORT_MAP_LINE_BLUE,        /* type 91: blue offset taxi lane */
    AIRPORT_MAP_LINE_KIND_COUNT
} airport_map_line_kind_t;

typedef struct {
    float x;    /* meters east of the reference point */
    float y;    /* meters north of the reference point */
} airport_map_pt_t;

typedef struct {
    uint32_t first;     /* index into airport_map_t.pts */
    uint32_t count;
    uint8_t kind;       /* airport_map_line_kind_t */
    uint8_t closed;
} airport_map_line_t;

typedef struct {
    char name[AIRPORT_MAP_NAME_LEN];
    float x;
    float y;
    float hdg;          /* true heading, degrees */
} airport_map_stand_t;

typedef struct {
    char name[8];
    airport_map_pt_t pos;   /* runway end threshold */
} airport_map_rwy_end_t;

/* Taxiway name placed along the taxi routing network. */
typedef struct {
    char name[16];
    airport_map_pt_t pos;
} airport_map_label_t;

typedef struct {
    bool found;
    char ident[AIRPORT_MAP_IDENT_LEN];
    char source[AIRPORT_MAP_PATH_LEN];

    bool have_ref;
    double ref_lat;
    double ref_lon;

    bool have_bounds;
    float min_x, min_y, max_x, max_y;

    airport_map_pt_t *pts;          /* polyline vertex pool */
    size_t n_pts, cap_pts;
    airport_map_line_t *lines;
    size_t n_lines, cap_lines;

    airport_map_pt_t *pave_tris;    /* triangle list, 3 vertices each */
    size_t n_pave_tris, cap_pave_tris;
    airport_map_pt_t *hole_tris;    /* grass holes cut into pavement */
    size_t n_hole_tris, cap_hole_tris;
    airport_map_pt_t *rwy_tris;     /* runway surfaces */
    size_t n_rwy_tris, cap_rwy_tris;

    airport_map_rwy_end_t *rwy_ends;
    size_t n_rwy_ends, cap_rwy_ends;

    airport_map_stand_t *stands;
    size_t n_stands, cap_stands;

    airport_map_label_t *labels;
    size_t n_labels, cap_labels;
} airport_map_t;

void airport_map_init(airport_map_t *map);
void airport_map_free(airport_map_t *map);

/*
 * Builds the list of apt.dat files in X-Plane priority order: every enabled
 * SCENERY_PACK of Custom Scenery/scenery_packs.ini (the XP12 placeholder
 * *GLOBAL_AIRPORTS* is honored at its position), followed by the global
 * airports file if it was not already listed. Only existing files are
 * returned. The caller frees the result with airport_map_free_paths().
 */
size_t airport_map_find_apt_dats(const char *xpdir, char ***paths_out);
void airport_map_free_paths(char **paths, size_t count);

/*
 * Scans the files in order and parses the first airport whose ident matches
 * `ident` (case-insensitive). Returns true when the airport was found.
 */
bool airport_map_load(char *const *paths, size_t n_paths, const char *ident,
    airport_map_t *map);

/* Parses a single open apt.dat stream. Exposed for tests. */
bool airport_map_parse_stream(FILE *fp, const char *ident,
    airport_map_t *map);

/* Projects a geographic position into the map's flat frame. */
void airport_map_geo_to_xy(airport_map_t *map, double lat, double lon,
    double *x, double *y);

/* Finds the stand nearest to (x, y) within max_dist meters, or -1. */
int airport_map_nearest_stand(const airport_map_t *map, double x, double y,
    double max_dist);

#ifdef __cplusplus
}
#endif

#endif
