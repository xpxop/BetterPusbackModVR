/*
 * Integration test: default pushback procedures resolved by
 * pushback_procedures.c must be solvable by the legacy segment planner
 * (driving.c compute_segs) and end exactly on the resolved poses.
 */
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#include <acfutils/list.h>

#include "driving.h"
#include "pushback_procedures.h"

/* minimal stand-ins for plugin/XPLM symbols referenced by driving.c */
int bp_xp_ver = 12000;
const char *const bp_xpdir = ".";
void log_impl(const char *f, int l, const char *fmt, ...)
{
    va_list a;
    (void)f;
    (void)l;
    va_start(a, fmt);
    vfprintf(stderr, fmt, a);
    va_end(a);
    fputc('\n', stderr);
}
void XPLMDebugString(const char *s) { fputs(s, stderr); }
double rel_hdg_impl(double h1, double h2, const char *f, int l)
{
    double d = fmod(h2 - h1, 360.0);
    (void)f;
    (void)l;
    if (d < 0)
        d += 360;
    if (d > 180)
        d -= 360;
    return d;
}

static void
check(const char *text, double start_hdg, double wheelbase)
{
    vehicle_t veh = {.wheelbase = wheelbase, .fixed_z_off = -2,
        .max_steer = 65};
    pb_proc_pose_t poses[PB_PROC_MAX_POSES], start = {500, -300, 0};
    bool fwd[PB_PROC_MAX_POSES];
    pb_proc_set_t set;
    list_t segs;
    FILE *fp = tmpfile();
    vect2_t sp;
    double sh;
    int n;

    assert(fp != NULL);
    fputs(text, fp);
    rewind(fp);
    pb_proc_set_init(&set);
    assert(pb_proc_parse_stream(fp, false, &set, NULL, 0));
    fclose(fp);
    start.hdg = start_hdg;
    n = pb_proc_resolve(&set.procs[0], start, poses, fwd, PB_PROC_MAX_POSES);
    assert(n > 0);

    list_create(&segs, sizeof(seg_t), offsetof(seg_t, node));
    sp = VECT2(start.x, start.y);
    sh = start_hdg;
    for (int i = 0; i < n; i++) {
        seg_t *t;
        int c = compute_segs(&veh, sp, sh, VECT2(poses[i].x, poses[i].y),
            poses[i].hdg, &segs);

        assert(c >= 0);
        t = list_tail(&segs);
        assert(t != NULL);
        assert(fabs(t->end_pos.x - poses[i].x) < 0.5);
        assert(fabs(t->end_pos.y - poses[i].y) < 0.5);
        assert(fabs(rel_hdg_impl(t->end_hdg, poses[i].hdg, NULL, 0)) < 1.0);
        /* a pushback procedure without 'forward' never pulls forward */
        assert(t->backward || fwd[i]);
        sp = t->end_pos;
        sh = t->end_hdg;
    }
    for (seg_t *s = list_head(&segs); s != NULL; s = list_head(&segs)) {
        list_remove_head(&segs);
        free(s);
    }
    list_destroy(&segs);
    pb_proc_set_free(&set);
}

int
main(void)
{
    static const char *cases[] = {
        "stand A\nback 30\nturn tail E radius 30\nback 15\nend\n",
        "stand A\nturn tail left\nend\n",
        "stand A\nback 20\nturn face 090 cw\nend\n",
        "stand A\nback 40\nturn tail right 45\nback 10\nend\n",
        "stand A\nturn tail N radius 40\nend\n",
        "stand A\nback 25\nturn tail SW\nback 5\nend\n",
    };
    static const double hdgs[] = {0, 37.5, 181, 270};

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        for (size_t h = 0; h < sizeof(hdgs) / sizeof(hdgs[0]); h++)
            check(cases[i], hdgs[h], 18);
    }
    printf("procedure_segments tests passed\n");
    return 0;
}
