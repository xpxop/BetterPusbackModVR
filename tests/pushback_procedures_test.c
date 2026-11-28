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
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pushback_procedures.h"

static int
near(double a, double b, double eps)
{
    return fabs(a - b) < eps;
}

static FILE *
text_stream(const char *text)
{
    FILE *fp = tmpfile();

    assert(fp != NULL);
    fputs(text, fp);
    rewind(fp);
    return fp;
}

static const char *sample =
    "# sample\n"
    "stand A1*\n"
    "name Tail left\n"
    "turn tail left\n"
    "end\n"
    "\n"
    "stand Gate A12\n"
    "name Push tail east, face west   # trailing comment\n"
    "source AIP XTST AD 2.20\n"
    "back 20\n"
    "turn tail E radius 25\n"
    "back 10.5\n"
    "end\n"
    "stand B1\n"
    "turn face 090 cw forward\n"
    "end\n";

static void
test_parse(void)
{
    pb_proc_set_t set;
    char err[PB_PROC_ERR_LEN];
    FILE *fp = text_stream(sample);
    const pb_proc_t *p;

    pb_proc_set_init(&set);
    assert(pb_proc_parse_stream(fp, false, &set, err, sizeof(err)));
    fclose(fp);
    assert(set.n == 3);

    p = &set.procs[0];
    assert(strcmp(p->pattern, "A1*") == 0);
    assert(p->n_steps == 1);
    assert(p->steps[0].type == PB_PROC_STEP_TURN);
    assert(p->steps[0].dir_kind == PB_PROC_DIR_RELATIVE);
    assert(near(p->steps[0].value, 90, 1e-9));

    p = &set.procs[1];
    assert(strcmp(p->pattern, "Gate A12") == 0);
    assert(strcmp(p->name, "Push tail east, face west") == 0);
    assert(strcmp(p->source, "AIP XTST AD 2.20") == 0);
    assert(p->n_steps == 3);
    assert(p->steps[1].tail && near(p->steps[1].value, 90, 1e-9));
    assert(near(p->steps[1].radius, 25, 1e-9));
    assert(near(p->steps[2].dist, 10.5, 1e-9));

    p = &set.procs[2];
    assert(strcmp(p->name, "Stand B1") == 0);
    assert(p->steps[0].rotation == 1 && p->steps[0].forward);
    pb_proc_set_free(&set);
}

static void
test_errors(void)
{
    static const char *bad[] = {
        "back 10\n",
        "stand X\nback -3\nend\n",
        "stand X\nturn face nowhere\nend\n",
        "stand X\nturn tail N radius\nend\n",
        "stand X\nback 5\n",
        "stand X\nend\n",
        "stand X\njump 5\nend\n",
    };

    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        pb_proc_set_t set;
        char err[PB_PROC_ERR_LEN];
        FILE *fp = text_stream(bad[i]);

        pb_proc_set_init(&set);
        assert(!pb_proc_parse_stream(fp, false, &set, err, sizeof(err)));
        assert(strncmp(err, "line ", 5) == 0);
        fclose(fp);
        pb_proc_set_free(&set);
    }
}

static void
test_match(void)
{
    pb_proc_set_t set;
    const pb_proc_t *out[8];
    FILE *fp = text_stream(sample);

    pb_proc_set_init(&set);
    assert(pb_proc_parse_stream(fp, false, &set, NULL, 0));
    fclose(fp);

    assert(pb_proc_pattern_matches("a1*", "A15"));
    assert(pb_proc_pattern_matches("A1?", "Gate A12"));
    assert(!pb_proc_pattern_matches("A1?", "A1"));
    assert(pb_proc_pattern_matches("B1", "Stand b1"));

    /* exact before wildcard */
    assert(pb_proc_match(&set, "Gate A12", out, 8) == 2);
    assert(strcmp(out[0]->pattern, "Gate A12") == 0);
    assert(strcmp(out[1]->pattern, "A1*") == 0);
    assert(pb_proc_match(&set, "C3", out, 8) == 0);

    /* user procedures come first */
    set.procs[0].user = true;
    assert(pb_proc_match(&set, "Gate A12", out, 8) == 2);
    assert(strcmp(out[0]->pattern, "A1*") == 0);
    pb_proc_set_free(&set);
}

static void
test_resolve(void)
{
    pb_proc_t proc;
    pb_proc_pose_t start = {100, 200, 0}, poses[PB_PROC_MAX_POSES];
    bool fwd[PB_PROC_MAX_POSES];
    pb_proc_set_t set;
    FILE *fp = text_stream(sample);
    int n;

    pb_proc_set_init(&set);
    assert(pb_proc_parse_stream(fp, false, &set, NULL, 0));
    fclose(fp);

    /* Gate A12 facing north: back 20 -> (100,180) */
    proc = set.procs[1];
    n = pb_proc_resolve(&proc, start, poses, fwd, PB_PROC_MAX_POSES);
    assert(n == 3);
    assert(near(poses[0].x, 100, 1e-9) && near(poses[0].y, 180, 1e-9));
    assert(!fwd[0]);
    /*
     * Tail east means nose west (270): shortest change from 0 is -90
     * (ccw). Backing up while the nose swings left, the aircraft moves
     * south and curves east: end = (125, 155) with radius 25.
     */
    assert(near(poses[1].hdg, 270, 1e-9));
    assert(near(poses[1].x, 125, 1e-6) && near(poses[1].y, 155, 1e-6));
    /* back 10.5 facing west -> moves east */
    assert(near(poses[2].x, 135.5, 1e-6) && near(poses[2].y, 155, 1e-6));

    /* "tail left" from north: nose +90, tail moves to the pilot's left */
    proc = set.procs[0];
    n = pb_proc_resolve(&proc, start, poses, fwd, PB_PROC_MAX_POSES);
    assert(n == 1);
    assert(near(poses[0].hdg, 90, 1e-9));
    assert(poses[0].x < 100 && poses[0].y < 200);
    assert(near(poses[0].x, 100 - 30, 1e-6) && near(poses[0].y, 200 - 30, 1e-6));

    /* forced cw from 180 to 090 = 270 deg, split into three 90 pieces */
    proc = set.procs[2];
    start.hdg = 180;
    n = pb_proc_resolve(&proc, start, poses, fwd, PB_PROC_MAX_POSES);
    assert(n == 3);
    assert(near(poses[0].hdg, 270, 1e-9));
    assert(near(poses[1].hdg, 0, 1e-9));
    assert(near(poses[2].hdg, 90, 1e-9));
    assert(fwd[0] && fwd[2]);
    pb_proc_set_free(&set);
}

static void
test_from_segs_round_trip(void)
{
    pb_proc_seg_t segs[] = {
        {false, true, 12, 0, 0, 0},
        {false, true, 8, 0, 0, 0},
        {true, true, 0, 32, 0, 270},
        {false, true, 5, 0, 270, 270},
    };
    pb_proc_t proc, again;
    pb_proc_set_t set;
    pb_proc_pose_t start = {0, 0, 0}, a[PB_PROC_MAX_POSES],
        b[PB_PROC_MAX_POSES];
    bool fa[PB_PROC_MAX_POSES], fb[PB_PROC_MAX_POSES];
    FILE *fp;
    int na, nb;

    pb_proc_from_segs(segs, 4, "A12", "Saved", "user", &proc);
    assert(proc.n_steps == 3);
    assert(near(proc.steps[0].dist, 20, 1e-9));
    assert(proc.steps[1].rotation == -1);
    assert(near(proc.steps[1].value, 270, 1e-9));

    fp = tmpfile();
    assert(fp != NULL);
    assert(pb_proc_write(fp, &proc));
    rewind(fp);
    pb_proc_set_init(&set);
    assert(pb_proc_parse_stream(fp, true, &set, NULL, 0));
    fclose(fp);
    assert(set.n == 1);
    again = set.procs[0];
    assert(again.user);
    na = pb_proc_resolve(&proc, start, a, fa, PB_PROC_MAX_POSES);
    nb = pb_proc_resolve(&again, start, b, fb, PB_PROC_MAX_POSES);
    assert(na == nb);
    for (int i = 0; i < na; i++) {
        assert(near(a[i].x, b[i].x, 0.06) && near(a[i].y, b[i].y, 0.06));
        assert(near(a[i].hdg, b[i].hdg, 0.5));
    }
    pb_proc_set_free(&set);
}

static void
test_save_user(const char *dir)
{
    char path[1024];
    pb_proc_set_t set, loaded;
    pb_proc_t proc;
    pb_proc_seg_t seg = {false, true, 30, 0, 0, 0};
    FILE *fp = text_stream(sample);

    (void)snprintf(path, sizeof(path), "%s/XTST.txt", dir);
    pb_proc_set_init(&set);
    assert(pb_proc_parse_stream(fp, false, &set, NULL, 0));
    fclose(fp);

    pb_proc_from_segs(&seg, 1, "Gate A12", "Mine", "", &proc);
    assert(pb_proc_save_user(&set, &proc, path));
    /* saving again replaces, not duplicates */
    proc.steps[0].dist = 31;
    assert(pb_proc_save_user(&set, &proc, path));
    assert(set.n == 4);

    pb_proc_set_init(&loaded);
    assert(pb_proc_load_file(path, true, &loaded, NULL, 0));
    /* only user procedures are written */
    assert(loaded.n == 1);
    assert(near(loaded.procs[0].steps[0].dist, 31, 1e-9));
    pb_proc_set_free(&loaded);
    pb_proc_set_free(&set);
    remove(path);
}

int
main(int argc, char **argv)
{
    assert(argc == 2);
    test_parse();
    test_errors();
    test_match();
    test_resolve();
    test_from_segs_round_trip();
    test_save_user(argv[1]);
    printf("pushback_procedures tests passed\n");
    return 0;
}
