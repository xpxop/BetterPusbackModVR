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
 * Default pushback procedures per airport stand.
 * See pushback_procedures.h for the file format.
 */

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pushback_procedures.h"

#define PB_PI 3.14159265358979323846
#define PB_DEG2RAD(d) ((d) * PB_PI / 180.0)
#define PB_RAD2DEG(r) ((r) * 180.0 / PB_PI)
#define PB_LINE_LEN 512
#define PB_MAX_TOK 12
#define PB_MAX_TURN_PIECE 90.0

void
pb_proc_set_init(pb_proc_set_t *set)
{
    memset(set, 0, sizeof(*set));
}

void
pb_proc_set_free(pb_proc_set_t *set)
{
    free(set->procs);
    pb_proc_set_init(set);
}

double
pb_proc_normalize_hdg(double hdg)
{
    hdg = fmod(hdg, 360.0);
    if (hdg < 0)
        hdg += 360.0;
    if (hdg >= 360.0)
        hdg -= 360.0;
    return hdg;
}

static double
rel_hdg(double from, double to)
{
    double d = pb_proc_normalize_hdg(to - from);
    return d > 180.0 ? d - 360.0 : d;
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
ieq(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0') {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b))
            return false;
        a++;
        b++;
    }
    return *a == *b;
}

static void
set_err(char *err, size_t cap, unsigned line, const char *msg,
    const char *tok)
{
    if (err == NULL || cap == 0)
        return;
    if (tok != NULL)
        (void)snprintf(err, cap, "line %u: %s '%s'", line, msg, tok);
    else
        (void)snprintf(err, cap, "line %u: %s", line, msg);
}

static bool
parse_number(const char *s, double *out)
{
    char *end;
    double v;

    if (s == NULL || *s == '\0')
        return false;
    v = strtod(s, &end);
    if (*end != '\0' || !isfinite(v))
        return false;
    *out = v;
    return true;
}

static bool
parse_compass(const char *s, double *hdg)
{
    static const char *names[16] = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    };
    static const char *long_names[8] = {
        "north", "northeast", "east", "southeast", "south", "southwest",
        "west", "northwest"
    };

    for (int i = 0; i < 16; i++) {
        if (ieq(s, names[i])) {
            *hdg = i * 22.5;
            return true;
        }
    }
    for (int i = 0; i < 8; i++) {
        if (ieq(s, long_names[i])) {
            *hdg = i * 45.0;
            return true;
        }
    }
    return false;
}

static int
tokenize(char *line, char *tok[PB_MAX_TOK])
{
    int n = 0;
    char *p = line;

    while (*p != '\0' && n < PB_MAX_TOK) {
        while (isspace((unsigned char)*p))
            p++;
        if (*p == '\0')
            break;
        tok[n++] = p;
        while (*p != '\0' && !isspace((unsigned char)*p))
            p++;
        if (*p != '\0')
            *p++ = '\0';
    }
    return n;
}

static const char *
rest_after_keyword(const char *line)
{
    while (isspace((unsigned char)*line))
        line++;
    while (*line != '\0' && !isspace((unsigned char)*line))
        line++;
    while (isspace((unsigned char)*line))
        line++;
    return line;
}

static void
strip(char *s)
{
    size_t l;
    char *hash = strchr(s, '#');

    if (hash != NULL)
        *hash = '\0';
    l = strlen(s);
    while (l > 0 && isspace((unsigned char)s[l - 1]))
        s[--l] = '\0';
}

static bool
parse_turn(char *tok[], int ntok, pb_proc_step_t *step, unsigned lineno,
    char *err, size_t err_cap)
{
    int i = 3;
    double v;

    step->type = PB_PROC_STEP_TURN;
    if (ntok < 3) {
        set_err(err, err_cap, lineno, "turn needs 'face' or 'tail' and a "
            "direction", NULL);
        return false;
    }
    if (ieq(tok[1], "face")) {
        step->tail = false;
    } else if (ieq(tok[1], "tail")) {
        step->tail = true;
    } else {
        set_err(err, err_cap, lineno, "expected 'face' or 'tail', got",
            tok[1]);
        return false;
    }

    if (ieq(tok[2], "left") || ieq(tok[2], "right")) {
        double angle = 90.0;
        bool left = ieq(tok[2], "left");

        if (ntok > 3 && parse_number(tok[3], &v)) {
            angle = fabs(v);
            i = 4;
        }
        step->dir_kind = PB_PROC_DIR_RELATIVE;
        /*
         * Nose turning left decreases the heading. The tail swinging to
         * the pilot's left means the nose rotates right.
         */
        if (step->tail)
            step->value = left ? angle : -angle;
        else
            step->value = left ? -angle : angle;
    } else if (tok[2][0] == '+' || tok[2][0] == '-') {
        if (!parse_number(tok[2], &v)) {
            set_err(err, err_cap, lineno, "bad relative angle", tok[2]);
            return false;
        }
        step->dir_kind = PB_PROC_DIR_RELATIVE;
        step->value = v;
    } else if (parse_compass(tok[2], &v) || parse_number(tok[2], &v)) {
        step->dir_kind = PB_PROC_DIR_ABSOLUTE;
        step->value = pb_proc_normalize_hdg(v);
    } else {
        set_err(err, err_cap, lineno, "bad direction", tok[2]);
        return false;
    }

    for (; i < ntok; i++) {
        if (ieq(tok[i], "radius")) {
            if (i + 1 >= ntok || !parse_number(tok[i + 1], &v) || v <= 0) {
                set_err(err, err_cap, lineno, "radius needs a positive "
                    "number of meters", NULL);
                return false;
            }
            step->radius = v;
            i++;
        } else if (ieq(tok[i], "cw")) {
            step->rotation = 1;
        } else if (ieq(tok[i], "ccw")) {
            step->rotation = -1;
        } else if (ieq(tok[i], "forward")) {
            step->forward = true;
        } else {
            set_err(err, err_cap, lineno, "unknown turn option", tok[i]);
            return false;
        }
    }
    return true;
}

bool
pb_proc_parse_stream(FILE *fp, bool user, pb_proc_set_t *set, char *err,
    size_t err_cap)
{
    char line[PB_LINE_LEN], work[PB_LINE_LEN];
    unsigned lineno = 0;
    pb_proc_t cur;
    bool in_proc = false;

    if (err != NULL && err_cap > 0)
        err[0] = '\0';
    while (fgets(line, sizeof(line), fp) != NULL) {
        char *tok[PB_MAX_TOK];
        int ntok;

        lineno++;
        strip(line);
        memcpy(work, line, strlen(line) + 1);
        ntok = tokenize(work, tok);
        if (ntok == 0)
            continue;

        if (ieq(tok[0], "stand")) {
            if (in_proc) {
                set_err(err, err_cap, lineno, "missing 'end' before", "stand");
                return false;
            }
            if (ntok < 2) {
                set_err(err, err_cap, lineno, "stand needs a name or pattern",
                    NULL);
                return false;
            }
            memset(&cur, 0, sizeof(cur));
            copy_str(cur.pattern, sizeof(cur.pattern),
                rest_after_keyword(line));
            cur.user = user;
            in_proc = true;
            continue;
        }
        if (!in_proc) {
            set_err(err, err_cap, lineno, "expected 'stand', got", tok[0]);
            return false;
        }
        if (ieq(tok[0], "name")) {
            copy_str(cur.name, sizeof(cur.name), rest_after_keyword(line));
        } else if (ieq(tok[0], "source")) {
            copy_str(cur.source, sizeof(cur.source),
                rest_after_keyword(line));
        } else if (ieq(tok[0], "end")) {
            pb_proc_t *n;

            if (cur.n_steps == 0) {
                set_err(err, err_cap, lineno, "procedure has no steps for",
                    cur.pattern);
                return false;
            }
            if (cur.name[0] == '\0')
                (void)snprintf(cur.name, sizeof(cur.name), "Stand %s",
                    cur.pattern);
            if (set->n == set->cap) {
                size_t ncap = set->cap != 0 ? set->cap * 2 : 8;
                n = realloc(set->procs, ncap * sizeof(*n));
                if (n == NULL) {
                    set_err(err, err_cap, lineno, "out of memory", NULL);
                    return false;
                }
                set->procs = n;
                set->cap = ncap;
            }
            set->procs[set->n++] = cur;
            in_proc = false;
        } else if (ieq(tok[0], "back") || ieq(tok[0], "forward")) {
            pb_proc_step_t *step;
            double v;

            if (cur.n_steps >= PB_PROC_MAX_STEPS) {
                set_err(err, err_cap, lineno, "too many steps", NULL);
                return false;
            }
            if (ntok != 2 || !parse_number(tok[1], &v) || v <= 0) {
                set_err(err, err_cap, lineno, "expected a positive distance "
                    "in meters after", tok[0]);
                return false;
            }
            step = &cur.steps[cur.n_steps++];
            memset(step, 0, sizeof(*step));
            step->type = PB_PROC_STEP_STRAIGHT;
            step->forward = ieq(tok[0], "forward");
            step->dist = v;
        } else if (ieq(tok[0], "turn")) {
            pb_proc_step_t step;

            if (cur.n_steps >= PB_PROC_MAX_STEPS) {
                set_err(err, err_cap, lineno, "too many steps", NULL);
                return false;
            }
            memset(&step, 0, sizeof(step));
            if (!parse_turn(tok, ntok, &step, lineno, err, err_cap))
                return false;
            cur.steps[cur.n_steps++] = step;
        } else {
            set_err(err, err_cap, lineno, "unknown keyword", tok[0]);
            return false;
        }
    }
    if (in_proc) {
        set_err(err, err_cap, lineno, "missing 'end' for stand", cur.pattern);
        return false;
    }
    return true;
}

bool
pb_proc_load_file(const char *path, bool user, pb_proc_set_t *set,
    char *err, size_t err_cap)
{
    FILE *fp = fopen(path, "r");
    bool ok;

    if (fp == NULL) {
        if (err != NULL && err_cap > 0)
            err[0] = '\0';
        return false;
    }
    ok = pb_proc_parse_stream(fp, user, set, err, err_cap);
    fclose(fp);
    return ok;
}

static bool
glob_ci(const char *p, const char *s)
{
    while (*p != '\0') {
        if (*p == '*') {
            while (*p == '*')
                p++;
            if (*p == '\0')
                return true;
            for (; *s != '\0'; s++) {
                if (glob_ci(p, s))
                    return true;
            }
            return false;
        }
        if (*s == '\0')
            return false;
        if (*p != '?' &&
            tolower((unsigned char)*p) != tolower((unsigned char)*s))
            return false;
        p++;
        s++;
    }
    return *s == '\0';
}

bool
pb_proc_pattern_matches(const char *pattern, const char *stand)
{
    const char *last;

    if (glob_ci(pattern, stand))
        return true;
    last = strrchr(stand, ' ');
    return last != NULL && last[1] != '\0' && glob_ci(pattern, last + 1);
}

static bool
has_wildcard(const char *p)
{
    return strchr(p, '*') != NULL || strchr(p, '?') != NULL;
}

size_t
pb_proc_match(const pb_proc_set_t *set, const char *stand,
    const pb_proc_t **out, size_t max_out)
{
    size_t count = 0;

    /* pass 0: user exact, 1: user wildcard, 2: built-in exact, 3: wild */
    for (int pass = 0; pass < 4; pass++) {
        bool want_user = pass < 2, want_wild = (pass % 2) == 1;

        for (size_t i = 0; i < set->n; i++) {
            const pb_proc_t *p = &set->procs[i];

            if (p->user != want_user || has_wildcard(p->pattern) != want_wild)
                continue;
            if (!pb_proc_pattern_matches(p->pattern, stand))
                continue;
            if (count < max_out)
                out[count] = p;
            count++;
        }
    }
    return count;
}

static void
dir_vec(double hdg, double *x, double *y)
{
    *x = sin(PB_DEG2RAD(hdg));
    *y = cos(PB_DEG2RAD(hdg));
}

/*
 * Moves `pose` along a circular arc that changes the nose heading by
 * `delta` degrees (positive = clockwise) with the given radius.
 */
static pb_proc_pose_t
arc(pb_proc_pose_t pose, double delta, double radius, bool forward)
{
    double motion = forward ? pose.hdg : pose.hdg + 180.0;
    double s = delta >= 0 ? 1.0 : -1.0;
    double cx, cy, ux, uy;
    pb_proc_pose_t out;

    dir_vec(motion + 90.0 * s, &ux, &uy);
    cx = pose.x + radius * ux;
    cy = pose.y + radius * uy;
    dir_vec(motion - 90.0 * s + delta, &ux, &uy);
    out.x = cx + radius * ux;
    out.y = cy + radius * uy;
    out.hdg = pb_proc_normalize_hdg(pose.hdg + delta);
    return out;
}

static double
turn_delta(const pb_proc_step_t *step, double hdg)
{
    double delta;

    if (step->dir_kind == PB_PROC_DIR_RELATIVE)
        return step->value;
    delta = rel_hdg(hdg, step->value + (step->tail ? 180.0 : 0.0));
    if (step->rotation > 0 && delta < 0)
        delta += 360.0;
    else if (step->rotation < 0 && delta > 0)
        delta -= 360.0;
    return delta;
}

int
pb_proc_resolve(const pb_proc_t *proc, pb_proc_pose_t start,
    pb_proc_pose_t *poses, bool *forward, size_t max_poses)
{
    pb_proc_pose_t cur = start;
    size_t n = 0;

    if (proc->n_steps == 0)
        return -1;
    cur.hdg = pb_proc_normalize_hdg(cur.hdg);
    for (unsigned i = 0; i < proc->n_steps; i++) {
        const pb_proc_step_t *step = &proc->steps[i];

        if (step->type == PB_PROC_STEP_STRAIGHT) {
            double dx, dy, sign = step->forward ? 1.0 : -1.0;

            if (n >= max_poses)
                return -1;
            dir_vec(cur.hdg, &dx, &dy);
            cur.x += sign * step->dist * dx;
            cur.y += sign * step->dist * dy;
            forward[n] = step->forward;
            poses[n++] = cur;
        } else {
            double delta = turn_delta(step, cur.hdg);
            double radius = step->radius > 0 ? step->radius :
                PB_PROC_DEFAULT_RADIUS;
            int pieces = (int)ceil(fabs(delta) / PB_MAX_TURN_PIECE - 1e-9);

            if (fabs(delta) < 0.5)
                continue;
            if (pieces < 1)
                pieces = 1;
            for (int k = 0; k < pieces; k++) {
                if (n >= max_poses)
                    return -1;
                cur = arc(cur, delta / pieces, radius, step->forward);
                forward[n] = step->forward;
                poses[n++] = cur;
            }
        }
    }
    return (int)n;
}

void
pb_proc_from_segs(const pb_proc_seg_t *segs, size_t n, const char *pattern,
    const char *name, const char *source, pb_proc_t *out)
{
    memset(out, 0, sizeof(*out));
    copy_str(out->pattern, sizeof(out->pattern), pattern);
    copy_str(out->name, sizeof(out->name), name);
    copy_str(out->source, sizeof(out->source), source);
    out->user = true;

    for (size_t i = 0; i < n && out->n_steps < PB_PROC_MAX_STEPS; i++) {
        const pb_proc_seg_t *s = &segs[i];
        pb_proc_step_t *prev = out->n_steps > 0 ?
            &out->steps[out->n_steps - 1] : NULL;
        pb_proc_step_t *step;

        if (!s->turn) {
            if (s->len < 0.05)
                continue;
            if (prev != NULL && prev->type == PB_PROC_STEP_STRAIGHT &&
                prev->forward == !s->backward) {
                prev->dist += s->len;
                continue;
            }
            step = &out->steps[out->n_steps++];
            memset(step, 0, sizeof(*step));
            step->type = PB_PROC_STEP_STRAIGHT;
            step->forward = !s->backward;
            step->dist = s->len;
        } else {
            double delta = rel_hdg(s->start_hdg, s->end_hdg);

            if (fabs(delta) < 0.5)
                continue;
            step = &out->steps[out->n_steps++];
            memset(step, 0, sizeof(*step));
            step->type = PB_PROC_STEP_TURN;
            step->forward = !s->backward;
            step->dir_kind = PB_PROC_DIR_ABSOLUTE;
            step->value = pb_proc_normalize_hdg(s->end_hdg);
            step->rotation = delta > 0 ? 1 : -1;
            step->radius = s->radius;
        }
    }
}

bool
pb_proc_write(FILE *fp, const pb_proc_t *proc)
{
    if (fprintf(fp, "stand %s\n", proc->pattern) < 0)
        return false;
    if (proc->name[0] != '\0' && fprintf(fp, "name %s\n", proc->name) < 0)
        return false;
    if (proc->source[0] != '\0' &&
        fprintf(fp, "source %s\n", proc->source) < 0)
        return false;
    for (unsigned i = 0; i < proc->n_steps; i++) {
        const pb_proc_step_t *s = &proc->steps[i];
        int r;

        if (s->type == PB_PROC_STEP_STRAIGHT) {
            r = fprintf(fp, "%s %.1f\n", s->forward ? "forward" : "back",
                s->dist);
        } else {
            char dir[32], radius[32] = "";

            if (s->dir_kind == PB_PROC_DIR_RELATIVE)
                (void)snprintf(dir, sizeof(dir), "%+.0f", s->value);
            else
                (void)snprintf(dir, sizeof(dir), "%03.0f", s->value);
            if (s->radius > 0)
                (void)snprintf(radius, sizeof(radius), " radius %.1f",
                    s->radius);
            r = fprintf(fp, "turn %s %s%s%s%s\n", s->tail ? "tail" : "face",
                dir, radius, s->rotation > 0 ? " cw" :
                s->rotation < 0 ? " ccw" : "",
                s->forward ? " forward" : "");
        }
        if (r < 0)
            return false;
    }
    return fprintf(fp, "end\n\n") >= 0;
}

bool
pb_proc_save_user(pb_proc_set_t *set, const pb_proc_t *proc,
    const char *path)
{
    char tmp[1100];
    FILE *fp;
    bool ok = true;
    size_t w = 0;
    pb_proc_t copy = *proc;    /* `proc` may point into `set` */

    proc = &copy;
    /* drop user procedures for the same stand */
    for (size_t i = 0; i < set->n; i++) {
        const pb_proc_t *p = &set->procs[i];

        if (p->user && ieq(p->pattern, proc->pattern))
            continue;
        set->procs[w++] = *p;
    }
    set->n = w;
    if (set->n == set->cap) {
        size_t ncap = set->cap != 0 ? set->cap * 2 : 8;
        pb_proc_t *n = realloc(set->procs, ncap * sizeof(*n));

        if (n == NULL)
            return false;
        set->procs = n;
        set->cap = ncap;
    }
    memmove(&set->procs[1], &set->procs[0], set->n * sizeof(*set->procs));
    set->procs[0] = *proc;
    set->procs[0].user = true;
    set->n++;

    (void)snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    fp = fopen(tmp, "w");
    if (fp == NULL)
        return false;
    ok = fprintf(fp, "# BetterPushback user pushback procedures.\n"
        "# Written by the planner's 'Save as stand default' button; you may "
        "edit this file.\n\n") >= 0;
    for (size_t i = 0; ok && i < set->n; i++) {
        if (set->procs[i].user)
            ok = pb_proc_write(fp, &set->procs[i]);
    }
    if (fclose(fp) != 0)
        ok = false;
    if (!ok) {
        (void)remove(tmp);
        return false;
    }
    (void)remove(path);
    return rename(tmp, path) == 0;
}
