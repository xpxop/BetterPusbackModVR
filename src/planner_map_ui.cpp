/*
 * 2D map pushback planner window (VR compatible). See planner_map_ui.h.
 *
 * Threading: apt.dat scanning runs on a worker thread (pure file I/O in
 * airport_map.c, no XPLM calls). Everything else, including the conversion
 * to X-Plane local coordinates, runs on the main thread.
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "ImgWindow/xp_img_window.h"

#include <XPLMDataAccess.h>
#include <XPLMGraphics.h>

#include <acfutils/conf.h>
#include <acfutils/geom.h>
#include <acfutils/helpers.h>
#include <acfutils/list.h>
#include <acfutils/log.h>
#include <acfutils/thread.h>

#include "airport_map.h"
#include "bp.h"
#include "cfg.h"
#include "driving.h"
#include "planner_api.h"
#include "planner_map_ui.h"
#include "pushback_procedures.h"
#include "ui_runtime.h"
#include "xplane.h"

namespace {

constexpr int MAP_WINDOW_W = 980;
constexpr int MAP_WINDOW_H = 700;
constexpr float SIDE_PANEL_W = 270.0f;
constexpr double MIN_SCALE = 0.05;     /* pixels per meter */
constexpr double MAX_SCALE = 20.0;
constexpr double ZOOM_STEP = 1.35;
constexpr float CLICK_DRAG_THRESHOLD = 6.0f;
constexpr int VERTEX_BUDGET = 60000;   /* 16-bit ImDrawIdx limit guard */
constexpr double STAND_MATCH_DIST = 40.0;
constexpr double SNAP_DIST = 25.0;
constexpr size_t MAX_PROC_MATCHES = 8;
constexpr double TURN_DRAW_STEP = 5.0;

const ImU32 COL_BG = IM_COL32(38, 52, 40, 255);
const ImU32 COL_PAVEMENT = IM_COL32(92, 94, 98, 255);
const ImU32 COL_PAVE_EDGE = IM_COL32(140, 142, 148, 255);
const ImU32 COL_RUNWAY = IM_COL32(52, 52, 56, 255);
const ImU32 COL_RWY_TEXT = IM_COL32(235, 235, 235, 255);
const ImU32 COL_YELLOW = IM_COL32(236, 196, 40, 255);
const ImU32 COL_WHITE = IM_COL32(225, 225, 225, 200);
const ImU32 COL_OTHER = IM_COL32(170, 170, 170, 160);
const ImU32 COL_ORANGE = IM_COL32(255, 140, 20, 255);
const ImU32 COL_BLUE = IM_COL32(70, 150, 255, 255);
const ImU32 COL_TWY_TEXT = IM_COL32(20, 20, 20, 255);
const ImU32 COL_TWY_LABEL = IM_COL32(236, 196, 40, 230);
const ImU32 COL_TAXI_ROUTE = IM_COL32(236, 196, 40, 150);
const ImU32 COL_STAND = IM_COL32(120, 200, 255, 255);
const ImU32 COL_STAND_TEXT = IM_COL32(190, 225, 255, 255);
const ImU32 COL_ROUTE = IM_COL32(60, 120, 255, 255);
const ImU32 COL_WING = IM_COL32(255, 64, 255, 190);
const ImU32 COL_PRED = IM_COL32(240, 220, 40, 255);
const ImU32 COL_ACF_NOW = IM_COL32(255, 255, 255, 255);
const ImU32 COL_ACF_END = IM_COL32(40, 230, 70, 255);
const ImU32 COL_ACF_TARGET = IM_COL32(240, 220, 40, 255);
const ImU32 COL_ACF_BAD = IM_COL32(255, 60, 50, 255);
const ImU32 COL_NOSE = IM_COL32(40, 230, 70, 255);
const ImU32 COL_TAIL = IM_COL32(255, 60, 50, 255);

/* ------------------------------------------------------------------ */
/* configuration                                                       */
/* ------------------------------------------------------------------ */

int
conf_int(const char *key, int def)
{
    int v = def;

    if (bp_conf != NULL)
        (void)conf_get_i(bp_conf, key, &v);
    return v;
}

/* ------------------------------------------------------------------ */
/* airport loader                                                      */
/* ------------------------------------------------------------------ */

struct MapLoader {
    mutex_t lock;
    bool lock_inited = false;
    thread_t thread;
    bool thread_running = false;    /* thread created and not joined */
    bool done = false;              /* worker finished (under lock) */
    bool ok = false;
    std::string xpdir;
    char ident[AIRPORT_MAP_IDENT_LEN] = {0};
    airport_map_t result;

    /* main-thread owned, converted to local coordinates */
    bool have_local = false;
    char local_ident[AIRPORT_MAP_IDENT_LEN] = {0};
    airport_map_t local;
} loader;

void
loader_worker(void *arg)
{
    MapLoader *l = static_cast<MapLoader *>(arg);
    char **paths = nullptr;
    size_t n_paths;
    airport_map_t map;
    bool ok;

    airport_map_init(&map);
    n_paths = airport_map_find_apt_dats(l->xpdir.c_str(), &paths);
    ok = airport_map_load(paths, n_paths, l->ident, &map);
    airport_map_free_paths(paths, n_paths);

    mutex_enter(&l->lock);
    l->result = map;
    l->ok = ok;
    l->done = true;
    mutex_exit(&l->lock);
}

void
loader_join()
{
    if (loader.thread_running) {
        thread_join(&loader.thread);
        loader.thread_running = false;
    }
}

void
loader_start(const char *ident)
{
    char xpdir[2048];

    if (ident == nullptr || ident[0] == '\0')
        return;
    if (loader.have_local && strcmp(loader.local_ident, ident) == 0)
        return;
    if (loader.thread_running && strcmp(loader.ident, ident) == 0)
        return;
    loader_join();
    if (!loader.lock_inited) {
        mutex_init(&loader.lock);
        loader.lock_inited = true;
    }
    if (loader.done)
        airport_map_free(&loader.result);
    XPLMGetSystemPath(xpdir);
    loader.xpdir = xpdir;
    snprintf(loader.ident, sizeof(loader.ident), "%s", ident);
    loader.done = false;
    loader.ok = false;
    airport_map_init(&loader.result);
    if (!thread_create(&loader.thread, loader_worker, &loader)) {
        logMsg(BP_WARN_LOG "Map planner: unable to start the airport loader");
        return;
    }
    loader.thread_running = true;
    logMsg(BP_INFO_LOG "Map planner: loading airport layout for %s", ident);
}

/* Affine ENU-meters -> local (x, -z) transform built with XPLMWorldToLocal */
struct LocalXform {
    vect2_t o, e, n;
};

vect2_t
local_of(double lat, double lon)
{
    double x, y, z;

    XPLMWorldToLocal(lat, lon, 0, &x, &y, &z);
    return VECT2(x, -z);
}

LocalXform
make_xform(const airport_map_t *map)
{
    const double d = 1000.0;
    const double dlat = RAD2DEG(d / 6371008.8);
    const double dlon = dlat / cos(DEG2RAD(map->ref_lat));
    LocalXform t;

    t.o = local_of(map->ref_lat, map->ref_lon);
    t.e = vect2_scmul(vect2_sub(local_of(map->ref_lat, map->ref_lon + dlon),
        t.o), 1.0 / d);
    t.n = vect2_scmul(vect2_sub(local_of(map->ref_lat + dlat, map->ref_lon),
        t.o), 1.0 / d);
    return t;
}

void
xform_pt(const LocalXform &t, float *x, float *y)
{
    vect2_t p = vect2_add(t.o, vect2_add(vect2_scmul(t.e, *x),
        vect2_scmul(t.n, *y)));
    *x = (float)p.x;
    *y = (float)p.y;
}

void
xform_pts(const LocalXform &t, airport_map_pt_t *pts, size_t n)
{
    for (size_t i = 0; i < n; i++)
        xform_pt(t, &pts[i].x, &pts[i].y);
}

/* Called every frame: picks up a finished worker result. */
void
loader_poll()
{
    bool done, ok;

    if (!loader.thread_running)
        return;
    mutex_enter(&loader.lock);
    done = loader.done;
    ok = loader.ok;
    mutex_exit(&loader.lock);
    if (!done)
        return;
    loader_join();

    if (loader.have_local)
        airport_map_free(&loader.local);
    loader.have_local = false;
    if (!ok) {
        logMsg(BP_WARN_LOG "Map planner: airport %s not found in any "
            "apt.dat; the map shows the route only", loader.ident);
        airport_map_free(&loader.result);
        snprintf(loader.local_ident, sizeof(loader.local_ident), "%s",
            loader.ident);
        loader.done = false;
        return;
    }
    loader.local = loader.result;
    airport_map_init(&loader.result);
    loader.done = false;

    {
        airport_map_t *m = &loader.local;
        LocalXform t = make_xform(m);
        float minx = 1e30f, miny = 1e30f, maxx = -1e30f, maxy = -1e30f;

        xform_pts(t, m->pts, m->n_pts);
        xform_pts(t, m->pave_tris, m->n_pave_tris);
        xform_pts(t, m->hole_tris, m->n_hole_tris);
        xform_pts(t, m->rwy_tris, m->n_rwy_tris);
        for (size_t i = 0; i < m->n_rwy_ends; i++)
            xform_pt(t, &m->rwy_ends[i].pos.x, &m->rwy_ends[i].pos.y);
        for (size_t i = 0; i < m->n_stands; i++)
            xform_pt(t, &m->stands[i].x, &m->stands[i].y);
        for (size_t i = 0; i < m->n_labels; i++)
            xform_pt(t, &m->labels[i].pos.x, &m->labels[i].pos.y);
        for (size_t i = 0; i < m->n_pts; i++) {
            minx = fminf(minx, m->pts[i].x); maxx = fmaxf(maxx, m->pts[i].x);
            miny = fminf(miny, m->pts[i].y); maxy = fmaxf(maxy, m->pts[i].y);
        }
        for (size_t i = 0; i < m->n_stands; i++) {
            minx = fminf(minx, m->stands[i].x);
            maxx = fmaxf(maxx, m->stands[i].x);
            miny = fminf(miny, m->stands[i].y);
            maxy = fmaxf(maxy, m->stands[i].y);
        }
        m->min_x = minx; m->min_y = miny; m->max_x = maxx; m->max_y = maxy;
    }
    loader.have_local = true;
    snprintf(loader.local_ident, sizeof(loader.local_ident), "%s",
        loader.ident);
    logMsg(BP_INFO_LOG "Map planner: %s loaded from %s (%u stands, %u "
        "lines, %u pavement triangles)", loader.local.ident,
        loader.local.source, (unsigned)loader.local.n_stands,
        (unsigned)loader.local.n_lines,
        (unsigned)(loader.local.n_pave_tris / 3));
}

const airport_map_t *
current_map()
{
    return loader.have_local ? &loader.local : nullptr;
}

bool
loader_busy()
{
    return loader.thread_running;
}

/* ------------------------------------------------------------------ */
/* procedures                                                          */
/* ------------------------------------------------------------------ */

struct ProcState {
    char airport[16] = {0};
    char stand[48] = {0};
    bool stand_from_gate = false;
    pb_proc_set_t set;
    bool set_inited = false;
    const pb_proc_t *matches[MAX_PROC_MATCHES];
    size_t n_matches = 0;
    std::string status;
    bool auto_applied = false;
} procs;

std::string
builtin_proc_path(const char *icao)
{
    std::string file = std::string(icao) + ".txt";
    char *p = mkpathname(bp_xpdir, bp_plugindir, "data",
        "pushback_procedures", file.c_str(), NULL);
    std::string r = p;
    free(p);
    return r;
}

std::string
user_proc_dir()
{
    char *p = mkpathname(bp_xpdir, "Output", "preferences",
        "BetterPushback_procedures", NULL);
    std::string r = p;
    free(p);
    return r;
}

std::string
user_proc_path(const char *icao)
{
    std::string file = std::string(icao) + ".txt";
    char *p = mkpathname(user_proc_dir().c_str(), file.c_str(), NULL);
    std::string r = p;
    free(p);
    return r;
}

void
procs_rematch()
{
    procs.n_matches = 0;
    if (procs.stand[0] == '\0' || !procs.set_inited)
        return;
    procs.n_matches = pb_proc_match(&procs.set, procs.stand, procs.matches,
        MAX_PROC_MATCHES);
    if (procs.n_matches > MAX_PROC_MATCHES)
        procs.n_matches = MAX_PROC_MATCHES;
}

void
procs_load(const char *icao)
{
    char err[PB_PROC_ERR_LEN];

    if (procs.set_inited)
        pb_proc_set_free(&procs.set);
    pb_proc_set_init(&procs.set);
    procs.set_inited = true;
    procs.status.clear();
    snprintf(procs.airport, sizeof(procs.airport), "%s", icao);
    if (icao[0] == '\0')
        return;

    std::string user = user_proc_path(icao);
    std::string builtin = builtin_proc_path(icao);

    if (!pb_proc_load_file(user.c_str(), true, &procs.set, err,
        sizeof(err)) && err[0] != '\0') {
        logMsg(BP_WARN_LOG "Pushback procedures %s: %s", user.c_str(), err);
        procs.status = std::string("Error in your procedure file: ") + err;
    }
    if (!pb_proc_load_file(builtin.c_str(), false, &procs.set, err,
        sizeof(err)) && err[0] != '\0') {
        logMsg(BP_WARN_LOG "Pushback procedures %s: %s", builtin.c_str(),
            err);
        procs.status = std::string("Error in procedure file: ") + err;
    }
    if (procs.set.n != 0) {
        logMsg(BP_INFO_LOG "Loaded %u pushback procedure(s) for %s",
            (unsigned)procs.set.n, icao);
    }
}

/* Identify the stand: published gate start, else the nearest map stand. */
void
procs_update_stand()
{
    char airport[16], ramp[48];

    if (bp_planner_gate(airport, sizeof(airport), ramp, sizeof(ramp))) {
        if (!procs.stand_from_gate || strcmp(ramp, procs.stand) != 0) {
            snprintf(procs.stand, sizeof(procs.stand), "%s", ramp);
            procs.stand_from_gate = true;
            procs_rematch();
        }
        return;
    }
    if (procs.stand[0] != '\0')
        return;
    const airport_map_t *map = current_map();
    if (map == nullptr)
        return;
    vect2_t pos;
    double hdg;
    bp_planner_aircraft_pose(&pos, &hdg);
    int idx = airport_map_nearest_stand(map, pos.x, pos.y,
        STAND_MATCH_DIST);
    if (idx >= 0) {
        snprintf(procs.stand, sizeof(procs.stand), "%s",
            map->stands[idx].name);
        procs.stand_from_gate = false;
        procs_rematch();
    }
}

bool
procs_apply(const pb_proc_t *proc)
{
    static const double radius_scales[] = {1.0, 1.5, 2.0, 3.0};
    pb_proc_pose_t poses[PB_PROC_MAX_POSES], start;
    bool fwd[PB_PROC_MAX_POSES];
    vect2_t pos[PB_PROC_MAX_POSES], apos;
    double hdg[PB_PROC_MAX_POSES], ahdg;

    bp_planner_aircraft_pose(&apos, &ahdg);
    start.x = apos.x;
    start.y = apos.y;
    start.hdg = ahdg;

    /*
     * Published procedures rarely state a turn radius. If the default is
     * too tight for this aircraft, retry turns without an explicit radius
     * with progressively wider ones.
     */
    for (double scale : radius_scales) {
        pb_proc_t p = *proc;
        int n;

        for (unsigned i = 0; i < p.n_steps; i++) {
            if (p.steps[i].type == PB_PROC_STEP_TURN && p.steps[i].radius <= 0)
                p.steps[i].radius = PB_PROC_DEFAULT_RADIUS * scale;
        }
        n = pb_proc_resolve(&p, start, poses, fwd, PB_PROC_MAX_POSES);
        if (n <= 0) {
            procs.status = "Procedure has no usable steps";
            return false;
        }
        for (int i = 0; i < n; i++) {
            pos[i] = VECT2(poses[i].x, poses[i].y);
            hdg[i] = poses[i].hdg;
        }
        if (bp_planner_apply_poses(pos, hdg, (size_t)n)) {
            procs.status = std::string("Applied: ") + proc->name;
            if (scale > 1.0)
                procs.status += " (wider turns for this aircraft)";
            logMsg(BP_INFO_LOG "Pushback procedure \"%s\" applied at %s %s "
                "(turn radius scale %.1f)", proc->name, procs.airport,
                procs.stand, scale);
            return true;
        }
    }
    procs.status = std::string("\"") + proc->name +
        "\" cannot be flown by this aircraft (turn too tight?)";
    return false;
}

bool
procs_save_current()
{
    std::vector<pb_proc_seg_t> segs;
    const list_t *list = bp_planner_segs();
    pb_proc_t proc;
    char name[PB_PROC_NAME_LEN], date[32];
    time_t now = time(nullptr);

    if (procs.airport[0] == '\0' || procs.stand[0] == '\0') {
        procs.status = "No stand identified - cannot save a stand default";
        return false;
    }
    for (const seg_t *s = (const seg_t *)list_head(list); s != nullptr;
        s = (const seg_t *)list_next(list, s)) {
        pb_proc_seg_t p;

        memset(&p, 0, sizeof(p));
        p.turn = s->type == SEG_TYPE_TURN;
        p.backward = s->backward;
        p.len = p.turn ? 0 : fabs(s->len);
        p.radius = p.turn ? s->turn.r : 0;
        p.start_hdg = s->start_hdg;
        p.end_hdg = s->end_hdg;
        segs.push_back(p);
    }
    if (segs.empty()) {
        procs.status = "Plan a route first";
        return false;
    }
    strftime(date, sizeof(date), "%Y-%m-%d", localtime(&now));
    snprintf(name, sizeof(name), "Saved %s (tail %s)", date,
        [&]() {
            static const char *pts[8] = {"N", "NE", "E", "SE", "S", "SW",
                "W", "NW"};
            double tail = pb_proc_normalize_hdg(segs.back().end_hdg + 180);
            return pts[(int)floor((tail + 22.5) / 45.0) % 8];
        }());
    pb_proc_from_segs(segs.data(), segs.size(), procs.stand, name,
        "Saved from the BetterPushback map planner", &proc);

    std::string dir = user_proc_dir();
    if (!create_directory_recursive(dir.c_str())) {
        procs.status = "Cannot create " + dir;
        return false;
    }
    std::string path = user_proc_path(procs.airport);
    if (!pb_proc_save_user(&procs.set, &proc, path.c_str())) {
        procs.status = "Cannot write " + path;
        return false;
    }
    procs_rematch();
    procs.status = std::string("Saved as default for ") + procs.stand;
    logMsg(BP_INFO_LOG "Pushback procedure for %s %s saved to %s",
        procs.airport, procs.stand, path.c_str());
    return true;
}

/* ------------------------------------------------------------------ */
/* geometry helpers                                                    */
/* ------------------------------------------------------------------ */

void
seg_polyline(const seg_t *seg, std::vector<vect2_t> &out)
{
    if (seg->type == SEG_TYPE_STRAIGHT) {
        out.push_back(seg->start_pos);
        out.push_back(seg->end_pos);
        return;
    }
    vect2_t c = vect2_add(seg->start_pos, vect2_scmul(
        vect2_norm(hdg2dir(seg->start_hdg), seg->turn.right), seg->turn.r));
    vect2_t c2s = vect2_sub(seg->start_pos, c);
    double rhdg = rel_hdg(seg->start_hdg, seg->end_hdg);
    int steps = (int)ceil(fabs(rhdg) / TURN_DRAW_STEP);

    if (steps < 1)
        steps = 1;
    for (int i = 0; i <= steps; i++) {
        double a = rhdg * i / steps;
        out.push_back(vect2_add(c, vect2_rot(c2s, a)));
    }
}

double
seg_hdg_at(const seg_t *seg, size_t i, size_t n)
{
    if (seg->type == SEG_TYPE_STRAIGHT || n < 2)
        return seg->start_hdg;
    double rhdg = rel_hdg(seg->start_hdg, seg->end_hdg);
    return seg->start_hdg + rhdg * (double)i / (double)(n - 1);
}

/* ------------------------------------------------------------------ */
/* window                                                              */
/* ------------------------------------------------------------------ */

struct ViewState {
    bool inited = false;
    vect2_t center = VECT2(0, 0);
    double scale = 2.0;     /* pixels per meter */
    bool heading_up = false;
    bool snap = true;
    bool quick = false;
    bool show_labels = true;
} view;

class PlannerMapWindow : public XPImgWindow {
public:
    PlannerMapWindow() :
        XPImgWindow(WND_MODE_FLOAT_CNT_VR, WND_STYLE_SOLID,
            WndRect(0, MAP_WINDOW_H, MAP_WINDOW_W, 0))
    {
        SetWindowTitle("BetterPushback - Pushback Planner");
        SetWindowResizingLimits(640, 480, 2400, 1800);
        vect2_t pos;
        bp_planner_aircraft_pose(&pos, &cursor_hdg);
        if (!view.inited) {
            view.inited = true;
            view.scale = 2.0;
        }
        view.center = pos;
    }

    void request_close() { close_requested = true; }

protected:
    ImGuiWindowFlags_ beforeBegin() override
    {
        return static_cast<ImGuiWindowFlags_>(ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse |
            ImGuiWindowFlags_NoSavedSettings);
    }

    bool onShow() override { return true; }

    void buildInterface() override;

private:
    bool close_requested = false;
    double cursor_hdg = 0;
    bool have_target = false;
    vect2_t target = VECT2(0, 0);
    bool target_reachable = true;
    bool dragging = false;
    ImVec2 press_pos;
    vect2_t press_center;
    std::vector<vect2_t> scratch;
    std::vector<vect2_t> outline;

    /* view transform */
    ImVec2 c_min, c_max, c_mid;
    double rot_sin = 0, rot_cos = 1;

    void setup_xform(ImVec2 mn, ImVec2 mx);
    ImVec2 w2s(double x, double y) const;
    vect2_t s2w(ImVec2 s) const;
    bool vtx_budget_ok(ImDrawList *dl) const
    {
        return dl->VtxBuffer.Size < VERTEX_BUDGET;
    }

    void draw_side_panel();
    void draw_prompt();
    void draw_canvas();
    void draw_airport(ImDrawList *dl);
    void draw_route(ImDrawList *dl, const list_t *segs, ImU32 col);
    void draw_acf(ImDrawList *dl, vect2_t pos, double hdg, ImU32 col,
        bool arrows);
    void handle_canvas_input(bool hovered, bool active);
    double snap_heading(vect2_t p, double hdg, bool *snapped) const;
    bool big_button(const char *label, float w = -1.0f, float h = 0.0f);
};

PlannerMapWindow *map_window = nullptr;

void
PlannerMapWindow::setup_xform(ImVec2 mn, ImVec2 mx)
{
    vect2_t apos;
    double ahdg;

    c_min = mn;
    c_max = mx;
    c_mid = ImVec2((mn.x + mx.x) / 2, (mn.y + mx.y) / 2);
    bp_planner_aircraft_pose(&apos, &ahdg);
    double rot = view.heading_up ? ahdg : 0;
    rot_sin = sin(DEG2RAD(rot));
    rot_cos = cos(DEG2RAD(rot));
}

/* world (x east, y north) -> screen; heading-up rotates by -heading */
ImVec2
PlannerMapWindow::w2s(double x, double y) const
{
    double dx = x - view.center.x, dy = y - view.center.y;
    double rx = dx * rot_cos - dy * rot_sin;
    double ry = dx * rot_sin + dy * rot_cos;
    return ImVec2((float)(c_mid.x + rx * view.scale),
        (float)(c_mid.y - ry * view.scale));
}

vect2_t
PlannerMapWindow::s2w(ImVec2 s) const
{
    double rx = (s.x - c_mid.x) / view.scale;
    double ry = -(s.y - c_mid.y) / view.scale;
    double dx = rx * rot_cos + ry * rot_sin;
    double dy = -rx * rot_sin + ry * rot_cos;
    return VECT2(view.center.x + dx, view.center.y + dy);
}

bool
PlannerMapWindow::big_button(const char *label, float w, float h)
{
    float bh = h > 0 ? h : ImGui::GetFrameHeight() * 1.6f;
    float bw = w < 0 ? ImGui::GetContentRegionAvail().x : w;
    return ImGui::Button(label, ImVec2(bw, bh));
}

void
PlannerMapWindow::buildInterface()
{
    if (close_requested)
        return;
    loader_poll();
    procs_update_stand();

    /*
     * Automatic application of the stand default: only while the planner
     * is untouched. A pending saved-route choice, an existing route or any
     * pilot input ends the opportunity for this planner session.
     */
    if (!procs.auto_applied) {
        if (bp_planner_prompt() != BP_PLANNER_PROMPT_NONE ||
            list_head(bp_planner_segs()) != nullptr || have_target ||
            !planner_map_ui_get_auto_apply()) {
            procs.auto_applied = true;
        } else if (procs.n_matches > 0) {
            procs.auto_applied = true;
            (void)procs_apply(procs.matches[0]);
        }
    }

    ImGui::BeginChild("##side", ImVec2(SIDE_PANEL_W, 0), true);
    draw_side_panel();
    ImGui::EndChild();
    if (!bp_planner_is_open())
        return;     /* the route was accepted/cancelled this frame */
    ImGui::SameLine();
    ImGui::BeginChild("##map", ImVec2(0, 0), false,
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    draw_canvas();
    ImGui::EndChild();
}

void
PlannerMapWindow::draw_side_panel()
{
    char buf[256];
    vect2_t apos;
    double ahdg;
    const list_t *segs = bp_planner_segs();
    size_t nsegs = list_count(segs);
    bool prompt = bp_planner_prompt() != BP_PLANNER_PROMPT_NONE;
    float half = (ImGui::GetContentRegionAvail().x -
        ImGui::GetStyle().ItemSpacing.x) / 2;
    float quarter = (ImGui::GetContentRegionAvail().x -
        3 * ImGui::GetStyle().ItemSpacing.x) / 4;

    bp_planner_aircraft_pose(&apos, &ahdg);

    /* header */
    if (procs.airport[0] != '\0') {
        snprintf(buf, sizeof(buf), "%s%s%s", procs.airport,
            procs.stand[0] != '\0' ? "  Stand " : "", procs.stand);
        ImGui::TextUnformatted(buf);
    } else {
        ImGui::TextUnformatted("No airport nearby");
    }
    if (loader_busy())
        ImGui::TextDisabled("Loading airport layout...");
    else if (current_map() == nullptr)
        ImGui::TextDisabled("No airport layout (route only)");
    snprintf(buf, sizeof(buf), "Route: %u segment%s", (unsigned)nsegs,
        nsegs == 1 ? "" : "s");
    ImGui::TextUnformatted(buf);
    ImGui::Separator();

    if (prompt)
        ImGui::BeginDisabled();

    /* target heading */
    double tail = pb_proc_normalize_hdg(cursor_hdg + 180);
    snprintf(buf, sizeof(buf), "Target: nose %03.0f  tail %03.0f",
        pb_proc_normalize_hdg(cursor_hdg), tail);
    ImGui::TextUnformatted(buf);
    {
        struct { const char *l; double d; } rot[4] = {
            {"-15", -15}, {"-5", -5}, {"+5", 5}, {"+15", 15}};
        for (int i = 0; i < 4; i++) {
            if (i > 0)
                ImGui::SameLine();
            if (big_button(rot[i].l, quarter))
                cursor_hdg = pb_proc_normalize_hdg(cursor_hdg + rot[i].d);
        }
    }
    if (big_button("Tail left 90", half))
        cursor_hdg = pb_proc_normalize_hdg(cursor_hdg + 90);
    ImGui::SameLine();
    if (big_button("Tail right 90", half))
        cursor_hdg = pb_proc_normalize_hdg(cursor_hdg - 90);

    /* compass dial: click/drag to set the nose heading directly */
    {
        float r = 46;
        ImVec2 p = ImGui::GetCursorScreenPos();
        float avail = ImGui::GetContentRegionAvail().x;
        ImVec2 c(p.x + avail / 2, p.y + r + 4);
        ImDrawList *dl = ImGui::GetWindowDrawList();

        ImGui::InvisibleButton("##dial", ImVec2(avail, 2 * r + 8));
        if (ImGui::IsItemActive()) {
            ImVec2 m = ImGui::GetIO().MousePos;
            double h = RAD2DEG(atan2(m.x - c.x, -(m.y - c.y)));
            if (view.heading_up)
                h += ahdg;
            cursor_hdg = pb_proc_normalize_hdg(round(h));
        }
        dl->AddCircleFilled(c, r, IM_COL32(30, 30, 34, 255), 48);
        dl->AddCircle(c, r, IM_COL32(160, 160, 170, 255), 48, 1.5f);
        double base = view.heading_up ? ahdg : 0;
        for (int i = 0; i < 4; i++) {
            static const char *n[4] = {"N", "E", "S", "W"};
            double a = DEG2RAD(i * 90 - base);
            ImVec2 tp(c.x + (float)sin(a) * (r - 10) - 4,
                c.y - (float)cos(a) * (r - 10) - 7);
            dl->AddText(tp, IM_COL32(200, 200, 210, 255), n[i]);
        }
        double a = DEG2RAD(cursor_hdg - base);
        ImVec2 tip(c.x + (float)sin(a) * (r - 4), c.y - (float)cos(a) * (r - 4));
        ImVec2 tl(c.x - (float)sin(a) * (r - 4), c.y + (float)cos(a) * (r - 4));
        dl->AddLine(c, tip, COL_NOSE, 3);
        dl->AddLine(c, tl, COL_TAIL, 3);
    }

    ImGui::Checkbox("Snap to taxi lines", &view.snap);
    ImGui::Checkbox("Click adds segment", &view.quick);

    if (!have_target || !target_reachable)
        ImGui::BeginDisabled();
    if (big_button("Add segment")) {
        if (bp_planner_commit_prediction())
            have_target = false;
    }
    if (!have_target || !target_reachable)
        ImGui::EndDisabled();
    if (big_button("Undo", half))
        bp_planner_undo();
    ImGui::SameLine();
    if (big_button("Clear", half)) {
        bp_planner_clear();
        have_target = false;
    }

    /* procedures */
    ImGui::Separator();
    ImGui::TextUnformatted("Stand procedures");
    if (procs.n_matches == 0) {
        ImGui::TextDisabled(procs.stand[0] != '\0' ? "None for this stand" :
            "Stand not identified");
    }
    for (size_t i = 0; i < procs.n_matches; i++) {
        const pb_proc_t *p = procs.matches[i];
        ImGui::PushID((int)i);
        snprintf(buf, sizeof(buf), "%s%s", p->user ? "* " : "", p->name);
        if (big_button(buf)) {
            if (procs_apply(p))
                have_target = false;
        }
        if (ImGui::IsItemHovered() && p->source[0] != '\0')
            ImGui::SetTooltip("%s", p->source);
        ImGui::PopID();
    }
    if (nsegs == 0 || procs.stand[0] == '\0')
        ImGui::BeginDisabled();
    if (big_button("Save as stand default"))
        (void)procs_save_current();
    if (nsegs == 0 || procs.stand[0] == '\0')
        ImGui::EndDisabled();
    if (!procs.status.empty())
        ImGui::TextWrapped("%s", procs.status.c_str());

    if (prompt)
        ImGui::EndDisabled();

    /* view */
    ImGui::Separator();
    if (big_button("Zoom +", quarter))
        view.scale = fmin(view.scale * ZOOM_STEP, MAX_SCALE);
    ImGui::SameLine();
    if (big_button("Zoom -", quarter))
        view.scale = fmax(view.scale / ZOOM_STEP, MIN_SCALE);
    ImGui::SameLine();
    if (big_button("Center", 2 * quarter + ImGui::GetStyle().ItemSpacing.x))
        view.center = apos;
    ImGui::Checkbox("Heading up", &view.heading_up);
    ImGui::SameLine();
    ImGui::Checkbox("Labels", &view.show_labels);

    /* finish */
    ImGui::Separator();
    if (prompt)
        ImGui::BeginDisabled();
    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(30, 120, 50, 255));
    if (nsegs == 0)
        ImGui::BeginDisabled();
    if (big_button("Accept route"))
        bp_planner_accept();
    if (nsegs == 0)
        ImGui::EndDisabled();
    ImGui::PopStyleColor();
    if (big_button("Connect tug first", half))
        bp_planner_connect_first();
    ImGui::SameLine();
    if (big_button("Cancel", half))
        bp_planner_cancel();
    if (prompt)
        ImGui::EndDisabled();

    const char *msg = bp_planner_message();
    if (msg != nullptr && msg[0] != '\0')
        ImGui::TextWrapped("%s", msg);
}

void
PlannerMapWindow::draw_prompt()
{
    char title[160], instr[200], text[192];
    unsigned n = bp_planner_prompt_option_count();
    ImVec2 size(fminf(560, c_max.x - c_min.x - 40),
        110 + n * (ImGui::GetFrameHeight() * 1.8f + 8));
    ImVec2 pos((c_min.x + c_max.x - size.x) / 2,
        (c_min.y + c_max.y - size.y) / 2);

    bp_planner_prompt_title(title, sizeof(title), instr, sizeof(instr));
    ImGui::SetCursorScreenPos(pos);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(20, 24, 30, 240));
    ImGui::BeginChild("##prompt", size, true);
    ImGui::TextUnformatted(title);
    ImGui::TextWrapped("%s", instr);
    ImGui::Separator();
    for (unsigned i = 0; i < n; i++) {
        bool enabled = bp_planner_prompt_option_enabled(i);
        bp_planner_prompt_option_text(i, text, sizeof(text));
        ImGui::PushID((int)i);
        if (!enabled)
            ImGui::BeginDisabled();
        if (big_button(text, -1, ImGui::GetFrameHeight() * 1.8f))
            bp_planner_prompt_choose(i);
        if (!enabled)
            ImGui::EndDisabled();
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();
}

double
PlannerMapWindow::snap_heading(vect2_t p, double hdg, bool *snapped) const
{
    const airport_map_t *map = current_map();
    double best = SNAP_DIST, best_hdg = hdg;

    *snapped = false;
    if (map == nullptr)
        return hdg;
    for (size_t li = 0; li < map->n_lines; li++) {
        const airport_map_line_t *l = &map->lines[li];

        if (l->kind != AIRPORT_MAP_LINE_YELLOW &&
            l->kind != AIRPORT_MAP_LINE_ORANGE &&
            l->kind != AIRPORT_MAP_LINE_BLUE &&
            l->kind != AIRPORT_MAP_LINE_TAXI_ROUTE)
            continue;
        for (uint32_t i = 0; i + 1 < l->count; i++) {
            const airport_map_pt_t &a = map->pts[l->first + i];
            const airport_map_pt_t &b = map->pts[l->first + i + 1];
            double vx = b.x - a.x, vy = b.y - a.y;
            double len2 = vx * vx + vy * vy;
            if (len2 < 1e-6)
                continue;
            double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2;
            t = fmax(0, fmin(1, t));
            double qx = a.x + t * vx - p.x, qy = a.y + t * vy - p.y;
            double d = sqrt(qx * qx + qy * qy);
            if (d < best) {
                /*
                 * atan2() yields -180..180, but libacfutils' rel_hdg()
                 * asserts (and kills the sim) on anything outside 0..360.
                 */
                double lh = pb_proc_normalize_hdg(RAD2DEG(atan2(vx, vy)));
                /* pick the line direction closest to the current heading */
                if (fabs(rel_hdg(pb_proc_normalize_hdg(hdg), lh)) > 90)
                    lh += 180;
                best = d;
                best_hdg = pb_proc_normalize_hdg(lh);
                *snapped = true;
            }
        }
    }
    return best_hdg;
}

void
PlannerMapWindow::handle_canvas_input(bool hovered, bool active)
{
    ImGuiIO &io = ImGui::GetIO();
    bool prompt = bp_planner_prompt() != BP_PLANNER_PROMPT_NONE;

    if (hovered && io.MouseWheel != 0) {
        /* zoom about the pointer */
        vect2_t before = s2w(io.MousePos);
        view.scale = io.MouseWheel > 0 ?
            fmin(view.scale * ZOOM_STEP, MAX_SCALE) :
            fmax(view.scale / ZOOM_STEP, MIN_SCALE);
        vect2_t after = s2w(io.MousePos);
        view.center = vect2_add(view.center, vect2_sub(before, after));
    }
    if (ImGui::IsItemActivated()) {
        press_pos = io.MousePos;
        press_center = view.center;
        dragging = false;
    }
    if (active) {
        ImVec2 d(io.MousePos.x - press_pos.x, io.MousePos.y - press_pos.y);
        if (!dragging && (fabsf(d.x) > CLICK_DRAG_THRESHOLD ||
            fabsf(d.y) > CLICK_DRAG_THRESHOLD))
            dragging = true;
        if (dragging) {
            /* pan: keep the pressed world point under the pointer */
            view.center = press_center;
            vect2_t a = s2w(press_pos), b = s2w(io.MousePos);
            view.center = vect2_add(press_center, vect2_sub(a, b));
        }
    }
    if (ImGui::IsItemDeactivated() && !dragging && !prompt) {
        vect2_t p = s2w(io.MousePos);
        bool snapped = false;

        if (view.snap)
            cursor_hdg = snap_heading(p, cursor_hdg, &snapped);
        target = p;
        have_target = true;
        if (view.quick) {
            if (bp_planner_predict(target, cursor_hdg) > 0 &&
                bp_planner_commit_prediction())
                have_target = false;
        }
    }
}

void
PlannerMapWindow::draw_airport(ImDrawList *dl)
{
    const airport_map_t *m = current_map();

    if (m == nullptr)
        return;
    auto tri_visible = [&](const ImVec2 &a, const ImVec2 &b,
        const ImVec2 &c) {
        float minx = fminf(a.x, fminf(b.x, c.x));
        float maxx = fmaxf(a.x, fmaxf(b.x, c.x));
        float miny = fminf(a.y, fminf(b.y, c.y));
        float maxy = fmaxf(a.y, fmaxf(b.y, c.y));
        return !(maxx < c_min.x || minx > c_max.x || maxy < c_min.y ||
            miny > c_max.y);
    };
    auto fill_tris = [&](const airport_map_pt_t *t, size_t n, ImU32 col) {
        for (size_t i = 0; i + 2 < n; i += 3) {
            if (!vtx_budget_ok(dl))
                return;
            ImVec2 a = w2s(t[i].x, t[i].y), b = w2s(t[i + 1].x, t[i + 1].y),
                c = w2s(t[i + 2].x, t[i + 2].y);
            if (!tri_visible(a, b, c))
                continue;
            dl->PrimReserve(3, 3);
            ImVec2 uv = ImGui::GetFontTexUvWhitePixel();
            ImDrawIdx base = (ImDrawIdx)dl->_VtxCurrentIdx;
            dl->PrimWriteVtx(a, uv, col);
            dl->PrimWriteVtx(b, uv, col);
            dl->PrimWriteVtx(c, uv, col);
            dl->PrimWriteIdx(base);
            dl->PrimWriteIdx((ImDrawIdx)(base + 1));
            dl->PrimWriteIdx((ImDrawIdx)(base + 2));
        }
    };

    fill_tris(m->pave_tris, m->n_pave_tris, COL_PAVEMENT);
    fill_tris(m->hole_tris, m->n_hole_tris, COL_BG);
    fill_tris(m->rwy_tris, m->n_rwy_tris, COL_RUNWAY);

    std::vector<ImVec2> pts;
    for (size_t li = 0; li < m->n_lines; li++) {
        const airport_map_line_t *l = &m->lines[li];
        ImU32 col;
        float w = 1.0f;

        switch (l->kind) {
        case AIRPORT_MAP_LINE_YELLOW: col = COL_YELLOW; w = 1.6f; break;
        case AIRPORT_MAP_LINE_WHITE: col = COL_WHITE; break;
        case AIRPORT_MAP_LINE_ORANGE: col = COL_ORANGE; w = 1.6f; break;
        case AIRPORT_MAP_LINE_BLUE: col = COL_BLUE; w = 1.6f; break;
        case AIRPORT_MAP_LINE_TAXI_ROUTE: col = COL_TAXI_ROUTE; w = 1.4f;
            break;
        case AIRPORT_MAP_LINE_PAVEMENT:
            if (view.scale < 0.4)
                continue;
            col = COL_PAVE_EDGE;
            break;
        default:
            if (view.scale < 1.0)
                continue;
            col = COL_OTHER;
            break;
        }
        pts.clear();
        float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
        for (uint32_t i = 0; i < l->count; i++) {
            const airport_map_pt_t &p = m->pts[l->first + i];
            ImVec2 s = w2s(p.x, p.y);
            minx = fminf(minx, s.x); maxx = fmaxf(maxx, s.x);
            miny = fminf(miny, s.y); maxy = fmaxf(maxy, s.y);
            pts.push_back(s);
        }
        if (maxx < c_min.x || minx > c_max.x || maxy < c_min.y ||
            miny > c_max.y)
            continue;
        if (!vtx_budget_ok(dl))
            break;
        dl->AddPolyline(pts.data(), (int)pts.size(), col,
            l->closed ? ImDrawFlags_Closed : ImDrawFlags_None, w);
    }

    /* taxiway names (from the taxi routing network) */
    if (view.show_labels && view.scale >= 0.35) {
        for (size_t i = 0; i < m->n_labels && vtx_budget_ok(dl); i++) {
            const airport_map_label_t *lb = &m->labels[i];
            ImVec2 s = w2s(lb->pos.x, lb->pos.y);
            if (s.x < c_min.x || s.x > c_max.x || s.y < c_min.y ||
                s.y > c_max.y)
                continue;
            ImVec2 ts = ImGui::CalcTextSize(lb->name);
            ImVec2 p0(s.x - ts.x / 2 - 3, s.y - ts.y / 2 - 1);
            ImVec2 p1(s.x + ts.x / 2 + 3, s.y + ts.y / 2 + 1);
            dl->AddRectFilled(p0, p1, COL_TWY_LABEL, 3.0f);
            dl->AddText(ImVec2(p0.x + 3, p0.y + 1), COL_TWY_TEXT, lb->name);
        }
    }

    /* runway designators */
    for (size_t i = 0; i < m->n_rwy_ends; i++) {
        ImVec2 s = w2s(m->rwy_ends[i].pos.x, m->rwy_ends[i].pos.y);
        if (s.x < c_min.x || s.x > c_max.x || s.y < c_min.y || s.y > c_max.y)
            continue;
        dl->AddText(ImVec2(s.x - 8, s.y - 7), COL_RWY_TEXT,
            m->rwy_ends[i].name);
    }

    /* stands */
    for (size_t i = 0; i < m->n_stands && vtx_budget_ok(dl); i++) {
        const airport_map_stand_t *st = &m->stands[i];
        ImVec2 s = w2s(st->x, st->y);
        if (s.x < c_min.x - 50 || s.x > c_max.x + 50 || s.y < c_min.y - 50 ||
            s.y > c_max.y + 50)
            continue;
        vect2_t dir = hdg2dir(st->hdg);
        double len = fmax(6.0, 8.0 / view.scale);
        ImVec2 tip = w2s(st->x + dir.x * len, st->y + dir.y * len);
        dl->AddCircleFilled(s, 3.0f, COL_STAND, 8);
        dl->AddLine(s, tip, COL_STAND, 1.5f);
        if (view.show_labels && view.scale >= 1.2 && st->name[0] != '\0')
            dl->AddText(ImVec2(s.x + 5, s.y + 2), COL_STAND_TEXT, st->name);
    }
}

void
PlannerMapWindow::draw_route(ImDrawList *dl, const list_t *segs, ImU32 col)
{
    double semispan = bp_planner_acf_semispan();
    std::vector<ImVec2> path, left, right;

    for (const seg_t *seg = (const seg_t *)list_head(segs); seg != nullptr;
        seg = (const seg_t *)list_next(segs, seg)) {
        scratch.clear();
        seg_polyline(seg, scratch);
        path.clear();
        left.clear();
        right.clear();
        for (size_t i = 0; i < scratch.size(); i++) {
            vect2_t p = scratch[i];
            path.push_back(w2s(p.x, p.y));
            if (semispan > 0) {
                double h = seg_hdg_at(seg, i, scratch.size());
                vect2_t r = vect2_scmul(vect2_norm(hdg2dir(h), B_TRUE),
                    semispan);
                vect2_t pl = vect2_sub(p, r), pr = vect2_add(p, r);
                left.push_back(w2s(pl.x, pl.y));
                right.push_back(w2s(pr.x, pr.y));
            }
        }
        if (!vtx_budget_ok(dl))
            return;
        dl->AddPolyline(path.data(), (int)path.size(), col, ImDrawFlags_None,
            3.0f);
        if (!left.empty()) {
            dl->AddPolyline(left.data(), (int)left.size(), COL_WING,
                ImDrawFlags_None, 1.5f);
            dl->AddPolyline(right.data(), (int)right.size(), COL_WING,
                ImDrawFlags_None, 1.5f);
        }
    }
}

void
PlannerMapWindow::draw_acf(ImDrawList *dl, vect2_t pos, double hdg,
    ImU32 col, bool arrows)
{
    outline.resize(4096);
    size_t n = bp_planner_acf_outline(pos, hdg, outline.data(),
        outline.size());

    if (n == 0) {
        /* no outline available: simple triangle symbol */
        vect2_t f = hdg2dir(hdg), r = vect2_norm(f, B_TRUE);
        vect2_t a = vect2_add(pos, vect2_scmul(f, 20));
        vect2_t b = vect2_add(pos, vect2_scmul(r, 12));
        vect2_t c = vect2_sub(pos, vect2_scmul(r, 12));
        dl->AddTriangle(w2s(a.x, a.y), w2s(b.x, b.y), w2s(c.x, c.y), col,
            2.0f);
    }
    for (size_t i = 0; i + 1 < n && vtx_budget_ok(dl); i += 2) {
        dl->AddLine(w2s(outline[i].x, outline[i].y),
            w2s(outline[i + 1].x, outline[i + 1].y), col, 2.0f);
    }
    if (arrows) {
        double len = fmax(30.0, 60.0 / view.scale);
        vect2_t f = vect2_add(pos, vect2_scmul(hdg2dir(hdg), len));
        vect2_t t = vect2_sub(pos, vect2_scmul(hdg2dir(hdg), len));
        ImVec2 s = w2s(pos.x, pos.y);
        dl->AddLine(s, w2s(f.x, f.y), COL_NOSE, 2.0f);
        dl->AddLine(s, w2s(t.x, t.y), COL_TAIL, 2.0f);
    }
}

void
PlannerMapWindow::draw_canvas()
{
    ImVec2 mn = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImVec2 mx(mn.x + fmaxf(avail.x, 50), mn.y + fmaxf(avail.y, 50));
    ImDrawList *dl = ImGui::GetWindowDrawList();
    vect2_t apos;
    double ahdg;
    bool prompt = bp_planner_prompt() != BP_PLANNER_PROMPT_NONE;

    setup_xform(mn, mx);
    ImGui::InvisibleButton("##canvas", ImVec2(mx.x - mn.x, mx.y - mn.y),
        ImGuiButtonFlags_MouseButtonLeft);
    bool hovered = ImGui::IsItemHovered(), active = ImGui::IsItemActive();
    handle_canvas_input(hovered, active);
    setup_xform(mn, mx);

    dl->PushClipRect(mn, mx, true);
    ImDrawListFlags saved_flags = dl->Flags;
    dl->Flags &= ~ImDrawListFlags_AntiAliasedFill;
    dl->AddRectFilled(mn, mx, COL_BG);
    draw_airport(dl);
    dl->Flags = saved_flags;

    /* scale bar */
    {
        double meters = 10;
        while (meters * view.scale < 60)
            meters *= meters >= 100 && fmod(meters, 500) != 0 ? 2.5 : 2;
        float px = (float)(meters * view.scale);
        ImVec2 a(mn.x + 12, mx.y - 16), b(mn.x + 12 + px, mx.y - 16);
        char lbl[32];
        snprintf(lbl, sizeof(lbl), "%.0f m", meters);
        dl->AddLine(a, b, IM_COL32(255, 255, 255, 255), 2);
        dl->AddText(ImVec2(a.x, a.y - 16), IM_COL32(255, 255, 255, 255), lbl);
    }
    bp_planner_aircraft_pose(&apos, &ahdg);

    /* prediction for the placed target */
    target_reachable = true;
    if (have_target && !prompt) {
        int n = bp_planner_predict(target, cursor_hdg);
        target_reachable = n > 0;
    } else if (!have_target) {
        bp_planner_clear_prediction();
    }

    draw_route(dl, bp_planner_segs(), COL_ROUTE);
    if (have_target && target_reachable)
        draw_route(dl, bp_planner_pred_segs(), COL_PRED);

    draw_acf(dl, apos, ahdg, COL_ACF_NOW, false);
    const seg_t *last = (const seg_t *)list_tail(bp_planner_segs());
    if (last != nullptr)
        draw_acf(dl, last->end_pos, last->end_hdg, COL_ACF_END, false);
    if (have_target)
        draw_acf(dl, target, cursor_hdg, target_reachable ? COL_ACF_TARGET :
            COL_ACF_BAD, true);

    /* hint line */
    const char *hint = prompt ? "Choose an option to continue" :
        !have_target ? "Click the map to place the end of the next segment. "
        "Drag to pan, wheel to zoom." :
        !target_reachable ? "Not reachable from here - move the target or "
        "change the heading." :
        view.quick ? "" : "Adjust the heading, then press Add segment.";
    dl->AddRectFilled(mn, ImVec2(mx.x, mn.y + 22), IM_COL32(0, 0, 0, 140));
    dl->AddText(ImVec2(mn.x + 8, mn.y + 4), IM_COL32(255, 255, 255, 255),
        hint);
    /* north indicator */
    {
        ImVec2 c(mx.x - 26, mn.y + 50);
        double a = DEG2RAD(view.heading_up ? -ahdg : 0);
        ImVec2 tip(c.x + (float)sin(a) * 16, c.y - (float)cos(a) * 16);
        dl->AddCircleFilled(c, 18, IM_COL32(0, 0, 0, 140), 24);
        dl->AddLine(c, tip, IM_COL32(255, 80, 80, 255), 3);
        dl->AddText(ImVec2(tip.x - 4, tip.y - 14), IM_COL32(255, 255, 255,
            255), "N");
    }
    dl->PopClipRect();

    if (prompt)
        draw_prompt();
}

void
destroy_window()
{
    if (map_window != nullptr) {
        delete map_window;
        map_window = nullptr;
    }
}

/* Window destruction is deferred out of the draw callback. */
float
deferred_close_cb(float, float, int, void *)
{
    destroy_window();
    return 0;
}

XPLMFlightLoopID close_floop = nullptr;

} /* namespace */

/* ------------------------------------------------------------------ */
/* C interface                                                         */
/* ------------------------------------------------------------------ */

extern "C" bool_t
planner_map_ui_vr_enabled(void)
{
    static XPLMDataRef vr = nullptr;

    if (vr == nullptr)
        vr = XPLMFindDataRef("sim/graphics/VR/enabled");
    return vr != nullptr && XPLMGetDatai(vr) != 0;
}

extern "C" planner_map_mode_t
planner_map_ui_get_mode(void)
{
    int v = conf_int(PLANNER_MAP_MODE_CONF_KEY, PLANNER_MAP_MODE_AUTO);

    if (v < PLANNER_MAP_MODE_AUTO || v > PLANNER_MAP_MODE_NEVER)
        v = PLANNER_MAP_MODE_AUTO;
    return static_cast<planner_map_mode_t>(v);
}

extern "C" void
planner_map_ui_set_mode(planner_map_mode_t mode)
{
    if (bp_conf != NULL)
        conf_set_i(bp_conf, PLANNER_MAP_MODE_CONF_KEY, (int)mode);
}

extern "C" bool_t
planner_map_ui_get_auto_apply(void)
{
    bool_t v = B_FALSE;

    if (bp_conf != NULL)
        (void)conf_get_b(bp_conf, PLANNER_MAP_AUTO_APPLY_CONF_KEY, &v);
    return v;
}

extern "C" void
planner_map_ui_set_auto_apply(bool_t flag)
{
    if (bp_conf != NULL)
        conf_set_b(bp_conf, PLANNER_MAP_AUTO_APPLY_CONF_KEY, flag);
}

extern "C" bool_t
planner_map_ui_wanted(void)
{
    switch (planner_map_ui_get_mode()) {
    case PLANNER_MAP_MODE_ALWAYS:
        return B_TRUE;
    case PLANNER_MAP_MODE_NEVER:
        return B_FALSE;
    default:
        return planner_map_ui_vr_enabled();
    }
}

extern "C" void
planner_map_ui_open(void)
{
    char icao[16] = {0};

    if (!bp_ui_runtime_init())
        return;
    if (close_floop != nullptr)
        XPLMScheduleFlightLoop(close_floop, 0, 0);
    destroy_window();

    bp_planner_airport(icao, sizeof(icao));
    procs.stand[0] = '\0';
    procs.stand_from_gate = false;
    procs.auto_applied = false;
    procs_load(icao);
    procs_update_stand();
    loader_start(icao);

    map_window = new PlannerMapWindow();
    map_window->SetVisible(true);
}

extern "C" void
planner_map_ui_close(void)
{
    if (map_window == nullptr)
        return;
    /*
     * bp_cam_stop() can be reached from a button inside our own draw
     * callback, so never delete the window here: hide it and destroy it
     * from a flight loop.
     */
    map_window->request_close();
    map_window->SetVisible(false);
    if (close_floop == nullptr) {
        XPLMCreateFlightLoop_t fl = {};
        fl.structSize = sizeof(fl);
        fl.phase = xplm_FlightLoop_Phase_BeforeFlightModel;
        fl.callbackFunc = deferred_close_cb;
        fl.refcon = nullptr;
        close_floop = XPLMCreateFlightLoop(&fl);
    }
    XPLMScheduleFlightLoop(close_floop, -1, 1);
}

extern "C" void
planner_map_ui_fini(void)
{
    if (close_floop != nullptr) {
        XPLMDestroyFlightLoop(close_floop);
        close_floop = nullptr;
    }
    destroy_window();
    loader_join();
    if (loader.done)
        airport_map_free(&loader.result);
    loader.done = false;
    if (loader.have_local)
        airport_map_free(&loader.local);
    loader.have_local = false;
    loader.local_ident[0] = '\0';
    if (loader.lock_inited) {
        mutex_destroy(&loader.lock);
        loader.lock_inited = false;
    }
    if (procs.set_inited) {
        pb_proc_set_free(&procs.set);
        procs.set_inited = false;
    }
}
