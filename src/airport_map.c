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
 * See airport_map.h for an overview.
 */

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "airport_map.h"

#define AMAP_EARTH_RADIUS 6371008.8
#define AMAP_PI 3.14159265358979323846
#define AMAP_DEG2RAD(d) ((d) * AMAP_PI / 180.0)
#define AMAP_BEZIER_STEPS 8
#define AMAP_LINE_BUF 2048
#define AMAP_MAX_TOKENS 24
#define LABEL_MIN_EDGE 30.0     /* ignore shorter taxi edges for labels (m) */
#define LABEL_SPACING 400.0     /* min distance between equal labels (m) */

typedef struct {
    double lat, lon;
    bool has_ctrl;
    double clat, clon;
} amap_node_t;

typedef enum {
    CHAIN_NONE,
    CHAIN_PAVEMENT,
    CHAIN_LINE,
    CHAIN_IGNORE
} chain_kind_t;

typedef struct {
    long id;
    airport_map_pt_t pt;
} taxi_node_t;

typedef struct {
    long a, b;
    char name[16];
} taxi_edge_t;

typedef struct {
    airport_map_t *map;
    chain_kind_t chain;
    int ring_index;             /* 0 = outer pavement ring, >0 = hole */
    uint8_t line_kind;          /* kind of the painted run in `ring` */
    bool prev_painted;          /* segment starting at `prev` is painted */
    uint8_t prev_kind;
    bool have_first;
    amap_node_t first;
    bool have_prev;
    amap_node_t prev;
    airport_map_pt_t *ring;     /* points of the ring/line being built */
    size_t n_ring, cap_ring;

    taxi_node_t *tnodes;
    size_t n_tnodes, cap_tnodes;
    taxi_edge_t *tedges;
    size_t n_tedges, cap_tedges;
    bool have_painted_lines;
} parse_ctx_t;

/* ------------------------------------------------------------------ */
/* small helpers                                                       */
/* ------------------------------------------------------------------ */

static bool
grow(void **buf, size_t *cap, size_t need, size_t elem)
{
    size_t ncap;
    void *n;

    if (need <= *cap)
        return true;
    ncap = *cap != 0 ? *cap : 64;
    while (ncap < need)
        ncap *= 2;
    n = realloc(*buf, ncap * elem);
    if (n == NULL)
        return false;
    *buf = n;
    *cap = ncap;
    return true;
}

#define GROW(arr, n, cap, extra) \
    grow((void **)&(arr), &(cap), (n) + (extra), sizeof(*(arr)))

static char *
dup_str(const char *s)
{
    size_t l = strlen(s) + 1;
    char *d = malloc(l);

    if (d != NULL)
        memcpy(d, s, l);
    return d;
}

static void
copy_str(char *dst, size_t cap, const char *src)
{
    size_t l = strlen(src);

    if (cap == 0)
        return;
    if (l >= cap)
        l = cap - 1;
    memcpy(dst, src, l);
    dst[l] = '\0';
}

static bool
streq_ci(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0') {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
            return false;
        a++;
        b++;
    }
    return *a == *b;
}

static void
strip_eol(char *s)
{
    size_t l = strlen(s);

    while (l > 0 && (s[l - 1] == '\n' || s[l - 1] == '\r' ||
        s[l - 1] == ' ' || s[l - 1] == '\t'))
        s[--l] = '\0';
}

/*
 * Splits `line` in place into whitespace separated tokens. `rest[i]` points
 * at the remainder of the original line starting at token i (used for names
 * that contain spaces). Returns the token count.
 */
static int
tokenize(char *line, char *tok[AMAP_MAX_TOKENS],
    const char *rest[AMAP_MAX_TOKENS], char *orig)
{
    int n = 0;
    char *p = line;

    while (*p != '\0' && n < AMAP_MAX_TOKENS) {
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '\0')
            break;
        tok[n] = p;
        rest[n] = orig + (p - line);
        n++;
        while (*p != '\0' && *p != ' ' && *p != '\t')
            p++;
        if (*p != '\0')
            *p++ = '\0';
    }
    return n;
}

void
airport_map_init(airport_map_t *map)
{
    memset(map, 0, sizeof(*map));
}

void
airport_map_free(airport_map_t *map)
{
    free(map->pts);
    free(map->lines);
    free(map->pave_tris);
    free(map->hole_tris);
    free(map->rwy_tris);
    free(map->rwy_ends);
    free(map->stands);
    free(map->labels);
    airport_map_init(map);
}

void
airport_map_geo_to_xy(airport_map_t *map, double lat, double lon,
    double *x, double *y)
{
    if (!map->have_ref) {
        map->have_ref = true;
        map->ref_lat = lat;
        map->ref_lon = lon;
    }
    *x = AMAP_DEG2RAD(lon - map->ref_lon) * AMAP_EARTH_RADIUS *
        cos(AMAP_DEG2RAD(map->ref_lat));
    *y = AMAP_DEG2RAD(lat - map->ref_lat) * AMAP_EARTH_RADIUS;
}

static airport_map_pt_t
project(airport_map_t *map, double lat, double lon)
{
    double x, y;
    airport_map_pt_t p;

    airport_map_geo_to_xy(map, lat, lon, &x, &y);
    p.x = (float)x;
    p.y = (float)y;
    return p;
}

static void
extend_bounds(airport_map_t *map, airport_map_pt_t p)
{
    if (!map->have_bounds) {
        map->have_bounds = true;
        map->min_x = map->max_x = p.x;
        map->min_y = map->max_y = p.y;
        return;
    }
    if (p.x < map->min_x) map->min_x = p.x;
    if (p.x > map->max_x) map->max_x = p.x;
    if (p.y < map->min_y) map->min_y = p.y;
    if (p.y > map->max_y) map->max_y = p.y;
}

static bool
add_line(airport_map_t *map, const airport_map_pt_t *pts, size_t n,
    uint8_t kind, bool closed)
{
    if (n < 2)
        return true;
    if (!GROW(map->pts, map->n_pts, map->cap_pts, n) ||
        !GROW(map->lines, map->n_lines, map->cap_lines, 1))
        return false;
    map->lines[map->n_lines].first = (uint32_t)map->n_pts;
    map->lines[map->n_lines].count = (uint32_t)n;
    map->lines[map->n_lines].kind = kind;
    map->lines[map->n_lines].closed = closed ? 1 : 0;
    map->n_lines++;
    for (size_t i = 0; i < n; i++)
        extend_bounds(map, pts[i]);
    memcpy(&map->pts[map->n_pts], pts, n * sizeof(*pts));
    map->n_pts += n;
    return true;
}

/* ------------------------------------------------------------------ */
/* triangulation (ear clipping)                                        */
/* ------------------------------------------------------------------ */

static double
cross3(airport_map_pt_t a, airport_map_pt_t b, airport_map_pt_t c)
{
    return ((double)b.x - a.x) * ((double)c.y - a.y) -
        ((double)b.y - a.y) * ((double)c.x - a.x);
}

static bool
point_in_tri(airport_map_pt_t p, airport_map_pt_t a, airport_map_pt_t b,
    airport_map_pt_t c)
{
    double d1 = cross3(a, b, p), d2 = cross3(b, c, p), d3 = cross3(c, a, p);
    bool neg = d1 < 0 || d2 < 0 || d3 < 0;
    bool pos = d1 > 0 || d2 > 0 || d3 > 0;

    return !(neg && pos);
}

static bool
emit_tri(airport_map_pt_t **tris, size_t *n, size_t *cap,
    airport_map_pt_t a, airport_map_pt_t b, airport_map_pt_t c)
{
    if (!grow((void **)tris, cap, *n + 3, sizeof(**tris)))
        return false;
    (*tris)[(*n)++] = a;
    (*tris)[(*n)++] = b;
    (*tris)[(*n)++] = c;
    return true;
}

/*
 * Triangulates a simple polygon into the given triangle list. Polygons that
 * are not simple still terminate: when no ear can be found the remainder is
 * emitted as a fan.
 */
static bool
triangulate(const airport_map_pt_t *poly, size_t n, airport_map_pt_t **tris,
    size_t *n_tris, size_t *cap_tris)
{
    size_t *idx, m;
    double area = 0;
    bool ok = true;

    if (n < 3)
        return true;
    idx = malloc(n * sizeof(*idx));
    if (idx == NULL)
        return false;
    for (size_t i = 0; i < n; i++) {
        const airport_map_pt_t *a = &poly[i], *b = &poly[(i + 1) % n];
        area += (double)a->x * b->y - (double)b->x * a->y;
    }
    /* work in counter-clockwise order */
    for (size_t i = 0; i < n; i++)
        idx[i] = area >= 0 ? i : n - 1 - i;
    m = n;

    while (m > 3 && ok) {
        bool clipped = false;

        for (size_t i = 0; i < m; i++) {
            size_t ip = idx[(i + m - 1) % m], ic = idx[i],
                in = idx[(i + 1) % m];
            airport_map_pt_t a = poly[ip], b = poly[ic], c = poly[in];
            bool ear = true;
            double cr = cross3(a, b, c);

            if (cr < 0)
                continue;       /* reflex vertex */
            if (cr == 0) {
                /* collinear: drop the middle vertex */
                memmove(&idx[i], &idx[i + 1], (m - i - 1) * sizeof(*idx));
                m--;
                clipped = true;
                break;
            }
            for (size_t j = 0; j < m; j++) {
                size_t k = idx[j];

                if (k == ip || k == ic || k == in)
                    continue;
                if (point_in_tri(poly[k], a, b, c)) {
                    ear = false;
                    break;
                }
            }
            if (!ear)
                continue;
            ok = emit_tri(tris, n_tris, cap_tris, a, b, c);
            memmove(&idx[i], &idx[i + 1], (m - i - 1) * sizeof(*idx));
            m--;
            clipped = true;
            break;
        }
        if (!clipped) {
            /* degenerate/self-intersecting: fan out the rest */
            for (size_t i = 1; i + 1 < m && ok; i++)
                ok = emit_tri(tris, n_tris, cap_tris, poly[idx[0]],
                    poly[idx[i]], poly[idx[i + 1]]);
            m = 0;
        }
    }
    if (m == 3 && ok)
        ok = emit_tri(tris, n_tris, cap_tris, poly[idx[0]], poly[idx[1]],
            poly[idx[2]]);
    free(idx);
    return ok;
}

/* ------------------------------------------------------------------ */
/* chains (110/120/130 + 111..116)                                      */
/* ------------------------------------------------------------------ */

static bool
ring_push(parse_ctx_t *ctx, airport_map_pt_t p)
{
    if (ctx->n_ring > 0) {
        airport_map_pt_t q = ctx->ring[ctx->n_ring - 1];
        if (fabsf(q.x - p.x) < 0.01f && fabsf(q.y - p.y) < 0.01f)
            return true;
    }
    if (!GROW(ctx->ring, ctx->n_ring, ctx->cap_ring, 1))
        return false;
    ctx->ring[ctx->n_ring++] = p;
    return true;
}

static double
bez(double p0, double p1, double p2, double p3, double t)
{
    double u = 1 - t;
    return u * u * u * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 +
        t * t * t * p3;
}

/* Appends the curve from ctx->prev to `b` (excluding prev, including b). */
static bool
chain_edge(parse_ctx_t *ctx, const amap_node_t *a, const amap_node_t *b)
{
    if (!a->has_ctrl && !b->has_ctrl)
        return ring_push(ctx, project(ctx->map, b->lat, b->lon));
    {
        double p1lat = a->has_ctrl ? a->clat : a->lat;
        double p1lon = a->has_ctrl ? a->clon : a->lon;
        double p2lat = b->has_ctrl ? 2 * b->lat - b->clat : b->lat;
        double p2lon = b->has_ctrl ? 2 * b->lon - b->clon : b->lon;

        for (int i = 1; i <= AMAP_BEZIER_STEPS; i++) {
            double t = (double)i / AMAP_BEZIER_STEPS;
            double lat = bez(a->lat, p1lat, p2lat, b->lat, t);
            double lon = bez(a->lon, p1lon, p2lon, b->lon, t);

            if (!ring_push(ctx, project(ctx->map, lat, lon)))
                return false;
        }
    }
    return true;
}

static uint8_t
line_kind_for_type(int type)
{
    if ((type >= 1 && type <= 14) || (type >= 51 && type <= 64))
        return AIRPORT_MAP_LINE_YELLOW;
    if ((type >= 20 && type <= 22) || (type >= 70 && type <= 72))
        return AIRPORT_MAP_LINE_WHITE;
    /*
     * XP12 offset taxi lanes. Not in the published spec; the colours are
     * taken from the default ENGM scenery, where type 90 lines are named
     * "Taxi_L_Orange_*" and type 91 lines "Taxi_L_Blue_*".
     */
    if (type == 90)
        return AIRPORT_MAP_LINE_ORANGE;
    if (type == 91)
        return AIRPORT_MAP_LINE_BLUE;
    return AIRPORT_MAP_LINE_OTHER;
}

/*
 * Emits the painted run collected in ctx->ring (if any) as one line. In a
 * 120 chain the line type of a node applies to the segment that starts at
 * that node; a node without a type starts an unpainted gap. Every run of
 * equally painted segments therefore becomes its own polyline.
 */
static bool
flush_line_run(parse_ctx_t *ctx)
{
    bool ok = true;

    if (ctx->n_ring >= 2) {
        ok = add_line(ctx->map, ctx->ring, ctx->n_ring, ctx->line_kind,
            false);
        ctx->have_painted_lines = true;
    }
    ctx->n_ring = 0;
    return ok;
}

/* Adds the edge prev -> node of a 120 chain, styled by `prev`. */
static bool
line_edge(parse_ctx_t *ctx, const amap_node_t *a, const amap_node_t *b,
    bool b_painted, uint8_t b_kind)
{
    if (ctx->prev_painted) {
        if (ctx->n_ring == 0) {
            ctx->line_kind = ctx->prev_kind;
            if (!ring_push(ctx, project(ctx->map, a->lat, a->lon)))
                return false;
        }
        if (!chain_edge(ctx, a, b))
            return false;
    }
    /* a gap or a different line type ends the current run */
    if (!b_painted || b_kind != ctx->line_kind)
        return flush_line_run(ctx);
    return true;
}

static bool
finish_ring(parse_ctx_t *ctx, bool closed)
{
    airport_map_t *map = ctx->map;
    bool ok = true;

    if (closed && ctx->n_ring > 1) {
        airport_map_pt_t a = ctx->ring[0], b = ctx->ring[ctx->n_ring - 1];
        if (fabsf(a.x - b.x) < 0.01f && fabsf(a.y - b.y) < 0.01f)
            ctx->n_ring--;
    }
    if (ctx->chain == CHAIN_PAVEMENT && ctx->n_ring >= 3) {
        if (ctx->ring_index == 0)
            ok = triangulate(ctx->ring, ctx->n_ring, &map->pave_tris,
                &map->n_pave_tris, &map->cap_pave_tris);
        else
            ok = triangulate(ctx->ring, ctx->n_ring, &map->hole_tris,
                &map->n_hole_tris, &map->cap_hole_tris);
        if (ok)
            ok = add_line(map, ctx->ring, ctx->n_ring,
                AIRPORT_MAP_LINE_PAVEMENT, true);
    } else if (ctx->chain == CHAIN_LINE) {
        ok = flush_line_run(ctx);
    }
    ctx->n_ring = 0;
    ctx->have_first = false;
    ctx->have_prev = false;
    ctx->ring_index++;
    return ok;
}

static void
begin_chain(parse_ctx_t *ctx, chain_kind_t kind)
{
    ctx->chain = kind;
    ctx->ring_index = 0;
    ctx->n_ring = 0;
    ctx->have_first = false;
    ctx->have_prev = false;
    ctx->line_kind = AIRPORT_MAP_LINE_OTHER;
    ctx->prev_painted = false;
    ctx->prev_kind = AIRPORT_MAP_LINE_OTHER;
}

static bool
chain_node(parse_ctx_t *ctx, int code, char *tok[], int ntok)
{
    amap_node_t node;
    int type_idx;

    if (ctx->chain == CHAIN_NONE || ctx->chain == CHAIN_IGNORE)
        return true;
    if (ntok < 3)
        return true;
    memset(&node, 0, sizeof(node));
    node.lat = atof(tok[1]);
    node.lon = atof(tok[2]);
    node.has_ctrl = (code == 112 || code == 114 || code == 116);
    if (node.has_ctrl) {
        if (ntok < 5)
            return true;
        node.clat = atof(tok[3]);
        node.clon = atof(tok[4]);
        type_idx = 5;
    } else {
        type_idx = 3;
    }
    if (ctx->chain == CHAIN_LINE) {
        bool painted = false;
        uint8_t kind = AIRPORT_MAP_LINE_OTHER;

        /* attributes: painted line type (< 100) and/or lighting (>= 100) */
        for (int i = type_idx; i < ntok; i++) {
            int type = atoi(tok[i]);
            if (type > 0 && type < 100) {
                painted = true;
                kind = line_kind_for_type(type);
                break;
            }
        }
        if (ctx->have_prev && !line_edge(ctx, &ctx->prev, &node, painted,
            kind))
            return false;
        if (!ctx->have_prev) {
            ctx->first = node;
            ctx->have_first = true;
        }
        ctx->prev = node;
        ctx->have_prev = true;
        ctx->prev_painted = painted;
        ctx->prev_kind = kind;
        if (code == 113 || code == 114) {
            /* closing segment back to the first node, styled by this node */
            if (!line_edge(ctx, &ctx->prev, &ctx->first, false, kind))
                return false;
            return finish_ring(ctx, true);
        }
        if (code == 115 || code == 116)
            return finish_ring(ctx, false);
        return true;
    }

    if (!ctx->have_prev) {
        if (!ring_push(ctx, project(ctx->map, node.lat, node.lon)))
            return false;
        ctx->first = node;
        ctx->have_first = true;
    } else if (!chain_edge(ctx, &ctx->prev, &node)) {
        return false;
    }
    ctx->prev = node;
    ctx->have_prev = true;

    if (code == 113 || code == 114) {
        /* close the loop back to the first node */
        if (ctx->have_first && !chain_edge(ctx, &ctx->prev, &ctx->first))
            return false;
        return finish_ring(ctx, true);
    }
    if (code == 115 || code == 116)
        return finish_ring(ctx, false);
    return true;
}

/* ------------------------------------------------------------------ */
/* other rows                                                          */
/* ------------------------------------------------------------------ */

static bool
parse_runway(parse_ctx_t *ctx, char *tok[], int ntok)
{
    airport_map_t *map = ctx->map;
    double width, lat1, lon1, lat2, lon2, x1, y1, x2, y2, dx, dy, len, nx,
        ny;
    airport_map_pt_t c[4];

    if (ntok < 20)
        return true;
    width = atof(tok[1]);
    lat1 = atof(tok[9]);
    lon1 = atof(tok[10]);
    lat2 = atof(tok[18]);
    lon2 = atof(tok[19]);
    airport_map_geo_to_xy(map, lat1, lon1, &x1, &y1);
    airport_map_geo_to_xy(map, lat2, lon2, &x2, &y2);
    dx = x2 - x1;
    dy = y2 - y1;
    len = sqrt(dx * dx + dy * dy);
    if (len < 1)
        return true;
    nx = -dy / len * width / 2;
    ny = dx / len * width / 2;
    c[0].x = (float)(x1 + nx); c[0].y = (float)(y1 + ny);
    c[1].x = (float)(x2 + nx); c[1].y = (float)(y2 + ny);
    c[2].x = (float)(x2 - nx); c[2].y = (float)(y2 - ny);
    c[3].x = (float)(x1 - nx); c[3].y = (float)(y1 - ny);
    for (int i = 0; i < 4; i++)
        extend_bounds(map, c[i]);
    if (!emit_tri(&map->rwy_tris, &map->n_rwy_tris, &map->cap_rwy_tris,
        c[0], c[1], c[2]) ||
        !emit_tri(&map->rwy_tris, &map->n_rwy_tris, &map->cap_rwy_tris,
        c[0], c[2], c[3]))
        return false;
    if (!GROW(map->rwy_ends, map->n_rwy_ends, map->cap_rwy_ends, 2))
        return false;
    copy_str(map->rwy_ends[map->n_rwy_ends].name,
        sizeof(map->rwy_ends[0].name), tok[8]);
    map->rwy_ends[map->n_rwy_ends].pos.x = (float)x1;
    map->rwy_ends[map->n_rwy_ends].pos.y = (float)y1;
    map->n_rwy_ends++;
    copy_str(map->rwy_ends[map->n_rwy_ends].name,
        sizeof(map->rwy_ends[0].name), tok[17]);
    map->rwy_ends[map->n_rwy_ends].pos.x = (float)x2;
    map->rwy_ends[map->n_rwy_ends].pos.y = (float)y2;
    map->n_rwy_ends++;
    return true;
}

static bool
parse_ramp_start(parse_ctx_t *ctx, char *tok[], const char *rest[],
    int ntok)
{
    airport_map_t *map = ctx->map;
    airport_map_stand_t *s;
    airport_map_pt_t p;

    if (ntok < 4)
        return true;
    if (!GROW(map->stands, map->n_stands, map->cap_stands, 1))
        return false;
    p = project(map, atof(tok[1]), atof(tok[2]));
    s = &map->stands[map->n_stands];
    memset(s, 0, sizeof(*s));
    s->x = p.x;
    s->y = p.y;
    /* apt.dat allows negative ramp headings (e.g. -163.9 at ENGM) */
    s->hdg = (float)fmod(fmod(atof(tok[3]), 360.0) + 360.0, 360.0);
    if (ntok >= 7)
        copy_str(s->name, sizeof(s->name), rest[6]);
    extend_bounds(map, p);
    map->n_stands++;
    return true;
}

static bool
parse_taxi_node(parse_ctx_t *ctx, char *tok[], int ntok)
{
    taxi_node_t *n;

    if (ntok < 5)
        return true;
    if (!GROW(ctx->tnodes, ctx->n_tnodes, ctx->cap_tnodes, 1))
        return false;
    n = &ctx->tnodes[ctx->n_tnodes++];
    n->pt = project(ctx->map, atof(tok[1]), atof(tok[2]));
    n->id = atol(tok[4]);
    return true;
}

static bool
parse_taxi_edge(parse_ctx_t *ctx, char *tok[], int ntok)
{
    if (ntok < 3)
        return true;
    if (ntok >= 5 && strcmp(tok[4], "runway") == 0)
        return true;    /* runway edges are already drawn as runways */
    if (!GROW(ctx->tedges, ctx->n_tedges, ctx->cap_tedges, 1))
        return false;
    ctx->tedges[ctx->n_tedges].a = atol(tok[1]);
    ctx->tedges[ctx->n_tedges].b = atol(tok[2]);
    /* 1202 <node> <node> <oneway|twoway> <taxiway|runway> <name> */
    copy_str(ctx->tedges[ctx->n_tedges].name,
        sizeof(ctx->tedges[ctx->n_tedges].name), ntok >= 6 ? tok[5] : "");
    ctx->n_tedges++;
    return true;
}

static int
tnode_cmp(const void *a, const void *b)
{
    long x = ((const taxi_node_t *)a)->id, y = ((const taxi_node_t *)b)->id;
    return (x > y) - (x < y);
}

typedef struct {
    size_t edge;
    double len;
    airport_map_pt_t mid;
} label_cand_t;

static int
label_cand_cmp(const void *a, const void *b)
{
    double la = ((const label_cand_t *)a)->len;
    double lb = ((const label_cand_t *)b)->len;
    return (la < lb) - (la > lb);   /* longest first */
}

/*
 * Places taxiway name labels: one at the middle of the longest named edges,
 * skipping a name when the same name is already labelled nearby.
 */
static bool
add_taxi_labels(parse_ctx_t *ctx)
{
    airport_map_t *map = ctx->map;
    label_cand_t *cand;
    size_t n_cand = 0;
    bool ok = true;

    if (ctx->n_tedges == 0)
        return true;
    cand = calloc(ctx->n_tedges, sizeof(*cand));
    if (cand == NULL)
        return false;
    for (size_t i = 0; i < ctx->n_tedges; i++) {
        taxi_node_t ka, kb, *a, *b;

        if (ctx->tedges[i].name[0] == '\0')
            continue;
        ka.id = ctx->tedges[i].a;
        kb.id = ctx->tedges[i].b;
        a = bsearch(&ka, ctx->tnodes, ctx->n_tnodes, sizeof(ka), tnode_cmp);
        b = bsearch(&kb, ctx->tnodes, ctx->n_tnodes, sizeof(kb), tnode_cmp);
        if (a == NULL || b == NULL)
            continue;
        cand[n_cand].edge = i;
        cand[n_cand].len = hypot(b->pt.x - a->pt.x, b->pt.y - a->pt.y);
        cand[n_cand].mid.x = (a->pt.x + b->pt.x) / 2;
        cand[n_cand].mid.y = (a->pt.y + b->pt.y) / 2;
        if (cand[n_cand].len >= LABEL_MIN_EDGE)
            n_cand++;
    }
    qsort(cand, n_cand, sizeof(*cand), label_cand_cmp);
    for (size_t i = 0; i < n_cand && ok; i++) {
        const char *name = ctx->tedges[cand[i].edge].name;
        bool near = false;

        for (size_t j = 0; j < map->n_labels && !near; j++) {
            if (strcmp(map->labels[j].name, name) == 0 &&
                hypot(map->labels[j].pos.x - cand[i].mid.x,
                map->labels[j].pos.y - cand[i].mid.y) < LABEL_SPACING)
                near = true;
        }
        if (near)
            continue;
        if (!GROW(map->labels, map->n_labels, map->cap_labels, 1)) {
            ok = false;
            break;
        }
        copy_str(map->labels[map->n_labels].name,
            sizeof(map->labels[map->n_labels].name), name);
        map->labels[map->n_labels].pos = cand[i].mid;
        map->n_labels++;
    }
    free(cand);
    return ok;
}

static bool
finish_taxi_network(parse_ctx_t *ctx)
{
    if (ctx->n_tedges == 0)
        return true;
    qsort(ctx->tnodes, ctx->n_tnodes, sizeof(*ctx->tnodes), tnode_cmp);
    if (!add_taxi_labels(ctx))
        return false;
    if (ctx->have_painted_lines)
        return true;
    for (size_t i = 0; i < ctx->n_tedges; i++) {
        taxi_node_t ka, kb, *a, *b;
        airport_map_pt_t seg[2];

        ka.id = ctx->tedges[i].a;
        kb.id = ctx->tedges[i].b;
        a = bsearch(&ka, ctx->tnodes, ctx->n_tnodes, sizeof(ka), tnode_cmp);
        b = bsearch(&kb, ctx->tnodes, ctx->n_tnodes, sizeof(kb), tnode_cmp);
        if (a == NULL || b == NULL)
            continue;
        seg[0] = a->pt;
        seg[1] = b->pt;
        if (!add_line(ctx->map, seg, 2, AIRPORT_MAP_LINE_TAXI_ROUTE, false))
            return false;
    }
    return true;
}

static bool
is_airport_header(int code)
{
    return code == 1 || code == 16 || code == 17;
}

bool
airport_map_parse_stream(FILE *fp, const char *ident, airport_map_t *map)
{
    char buf[AMAP_LINE_BUF], work[AMAP_LINE_BUF];
    bool in_airport = false, ok = true;
    parse_ctx_t ctx;

    memset(&ctx, 0, sizeof(ctx));
    ctx.map = map;

    while (ok && fgets(buf, sizeof(buf), fp) != NULL) {
        char *tok[AMAP_MAX_TOKENS];
        const char *rest[AMAP_MAX_TOKENS];
        size_t len = strlen(buf);
        int ntok, code;

        if (len == sizeof(buf) - 1 && buf[len - 1] != '\n') {
            /* overlong line: discard the remainder */
            int ch;
            while ((ch = fgetc(fp)) != EOF && ch != '\n')
                ;
        }
        /* fast reject of non-header lines while searching */
        if (!in_airport && !(buf[0] == '1' && (buf[1] == ' ' ||
            ((buf[1] == '6' || buf[1] == '7') && buf[2] == ' '))))
            continue;
        strip_eol(buf);
        memcpy(work, buf, strlen(buf) + 1);
        ntok = tokenize(work, tok, rest, buf);
        if (ntok == 0)
            continue;
        code = atoi(tok[0]);

        if (is_airport_header(code) || code == 99) {
            if (in_airport)
                break;  /* next airport begins: we're done */
            if (code != 99 && ntok >= 5 && streq_ci(tok[4], ident)) {
                in_airport = true;
                map->found = true;
                copy_str(map->ident, sizeof(map->ident), tok[4]);
            }
            continue;
        }
        if (!in_airport)
            continue;

        if (code >= 111 && code <= 116) {
            ok = chain_node(&ctx, code, tok, ntok);
            continue;
        }
        /* any other row terminates an unterminated chain */
        if (ctx.chain != CHAIN_NONE && ctx.n_ring > 0)
            ok = finish_ring(&ctx, ctx.chain == CHAIN_PAVEMENT);
        ctx.chain = CHAIN_NONE;
        if (!ok)
            break;

        switch (code) {
        case 100:
            ok = parse_runway(&ctx, tok, ntok);
            break;
        case 110:
            begin_chain(&ctx, CHAIN_PAVEMENT);
            break;
        case 120:
            begin_chain(&ctx, CHAIN_LINE);
            break;
        case 130:
            begin_chain(&ctx, CHAIN_IGNORE);
            break;
        case 1201:
            ok = parse_taxi_node(&ctx, tok, ntok);
            break;
        case 1202:
            ok = parse_taxi_edge(&ctx, tok, ntok);
            break;
        case 1300:
            ok = parse_ramp_start(&ctx, tok, rest, ntok);
            break;
        default:
            break;
        }
    }
    if (ok && ctx.chain != CHAIN_NONE && ctx.n_ring > 0)
        ok = finish_ring(&ctx, ctx.chain == CHAIN_PAVEMENT);
    if (ok && map->found)
        ok = finish_taxi_network(&ctx);

    free(ctx.ring);
    free(ctx.tnodes);
    free(ctx.tedges);
    return ok && map->found;
}

bool
airport_map_load(char *const *paths, size_t n_paths, const char *ident,
    airport_map_t *map)
{
    for (size_t i = 0; i < n_paths; i++) {
        FILE *fp = fopen(paths[i], "r");
        bool found;

        if (fp == NULL)
            continue;
        airport_map_free(map);
        found = airport_map_parse_stream(fp, ident, map);
        fclose(fp);
        if (found) {
            copy_str(map->source, sizeof(map->source), paths[i]);
            return true;
        }
    }
    airport_map_free(map);
    return false;
}

int
airport_map_nearest_stand(const airport_map_t *map, double x, double y,
    double max_dist)
{
    int best = -1;
    double best_d2 = max_dist * max_dist;

    for (size_t i = 0; i < map->n_stands; i++) {
        double dx = map->stands[i].x - x, dy = map->stands[i].y - y;
        double d2 = dx * dx + dy * dy;

        if (d2 <= best_d2) {
            best_d2 = d2;
            best = (int)i;
        }
    }
    return best;
}

/* ------------------------------------------------------------------ */
/* apt.dat discovery                                                   */
/* ------------------------------------------------------------------ */

static bool
file_readable(const char *path)
{
    FILE *fp = fopen(path, "r");

    if (fp == NULL)
        return false;
    fclose(fp);
    return true;
}

static bool
path_is_absolute(const char *p)
{
    return p[0] == '/' || p[0] == '\\' ||
        (isalpha((unsigned char)p[0]) && p[1] == ':');
}

static bool
add_path(char ***paths, size_t *n, size_t *cap, const char *path)
{
    char *d;

    for (size_t i = 0; i < *n; i++) {
        if (strcmp((*paths)[i], path) == 0)
            return true;
    }
    if (!file_readable(path))
        return true;
    if (!grow((void **)paths, cap, *n + 1, sizeof(**paths)))
        return false;
    d = dup_str(path);
    if (d == NULL)
        return false;
    (*paths)[(*n)++] = d;
    return true;
}

static void
global_airports_path(const char *xpdir, char *out, size_t cap)
{
    (void)snprintf(out, cap, "%s/Global Scenery/Global Airports/"
        "Earth nav data/apt.dat", xpdir);
    if (!file_readable(out)) {
        (void)snprintf(out, cap, "%s/Resources/default scenery/"
            "default apt dat/Earth nav data/apt.dat", xpdir);
    }
}

size_t
airport_map_find_apt_dats(const char *xpdir, char ***paths_out)
{
    char **paths = NULL;
    size_t n = 0, cap = 0;
    char path[AIRPORT_MAP_PATH_LEN * 2];
    char line[AIRPORT_MAP_PATH_LEN];
    FILE *ini;

    (void)snprintf(path, sizeof(path), "%s/Custom Scenery/scenery_packs.ini",
        xpdir);
    ini = fopen(path, "r");
    if (ini != NULL) {
        while (fgets(line, sizeof(line), ini) != NULL) {
            char *pack;
            size_t l;

            strip_eol(line);
            if (strncmp(line, "SCENERY_PACK ", 13) != 0)
                continue;
            pack = line + 13;
            while (*pack == ' ')
                pack++;
            if (strcmp(pack, "*GLOBAL_AIRPORTS*") == 0) {
                global_airports_path(xpdir, path, sizeof(path));
            } else {
                l = strlen(pack);
                while (l > 0 && (pack[l - 1] == '/' || pack[l - 1] == '\\'))
                    pack[--l] = '\0';
                if (path_is_absolute(pack)) {
                    (void)snprintf(path, sizeof(path),
                        "%s/Earth nav data/apt.dat", pack);
                } else {
                    (void)snprintf(path, sizeof(path),
                        "%s/%s/Earth nav data/apt.dat", xpdir, pack);
                }
            }
            if (!add_path(&paths, &n, &cap, path))
                break;
        }
        fclose(ini);
    }
    global_airports_path(xpdir, path, sizeof(path));
    (void)add_path(&paths, &n, &cap, path);

    *paths_out = paths;
    return n;
}

void
airport_map_free_paths(char **paths, size_t count)
{
    for (size_t i = 0; i < count; i++)
        free(paths[i]);
    free(paths);
}
