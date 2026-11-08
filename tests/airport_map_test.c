#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "airport_map.h"

static double
tri_area(const airport_map_pt_t *t, size_t n)
{
    double a = 0;

    for (size_t i = 0; i + 2 < n; i += 3) {
        a += fabs(((double)t[i + 1].x - t[i].x) * ((double)t[i + 2].y - t[i].y) -
            ((double)t[i + 1].y - t[i].y) * ((double)t[i + 2].x - t[i].x)) / 2;
    }
    return a;
}

static void
test_parse_sample(const char *path)
{
    FILE *fp = fopen(path, "r");
    airport_map_t map;
    size_t yellow = 0, pavement = 0, route = 0;

    assert(fp != NULL);
    airport_map_init(&map);
    assert(airport_map_parse_stream(fp, "xtst", &map));
    fclose(fp);

    assert(map.found);
    assert(strcmp(map.ident, "XTST") == 0);
    /* only this airport's stands, names keep spaces */
    assert(map.n_stands == 2);
    assert(strcmp(map.stands[0].name, "Gate A12") == 0);
    assert(strcmp(map.stands[1].name, "B1") == 0);
    assert(fabs(map.stands[0].hdg - 180.0) < 1e-6);

    /* runway: 2 triangles, ~1430 m x 45 m */
    assert(map.n_rwy_tris == 6);
    assert(fabs(tri_area(map.rwy_tris, map.n_rwy_tris) - 1430.0 * 45.0) <
        1430.0 * 45.0 * 0.01);
    assert(map.n_rwy_ends == 2);
    assert(strcmp(map.rwy_ends[0].name, "09") == 0);

    /* outer ring triangulated, hole separately */
    assert(map.n_pave_tris >= 3 && map.n_pave_tris % 3 == 0);
    assert(map.n_hole_tris == 3);    /* triangular hole */
    assert(tri_area(map.pave_tris, map.n_pave_tris) >
        tri_area(map.hole_tris, map.n_hole_tris));

    for (size_t i = 0; i < map.n_lines; i++) {
        if (map.lines[i].kind == AIRPORT_MAP_LINE_YELLOW) {
            yellow++;
            /* bezier end tessellated: 1 start + 8 steps */
            assert(map.lines[i].count == 9);
            assert(!map.lines[i].closed);
        } else if (map.lines[i].kind == AIRPORT_MAP_LINE_PAVEMENT) {
            pavement++;
            assert(map.lines[i].closed);
        } else if (map.lines[i].kind == AIRPORT_MAP_LINE_TAXI_ROUTE) {
            route++;
        }
    }
    assert(yellow == 1);
    assert(pavement == 2);
    /* painted lines exist, so the taxi network is not drawn */
    assert(route == 0);

    assert(airport_map_nearest_stand(&map, map.stands[1].x,
        map.stands[1].y + 1, 10) == 1);
    assert(airport_map_nearest_stand(&map, 5000, 5000, 10) == -1);
    assert(map.min_x < map.max_x && map.min_y < map.max_y);
    airport_map_free(&map);
}

static void
test_missing_airport(const char *path)
{
    FILE *fp = fopen(path, "r");
    airport_map_t map;

    assert(fp != NULL);
    airport_map_init(&map);
    assert(!airport_map_parse_stream(fp, "NONE", &map));
    assert(!map.found);
    fclose(fp);
    airport_map_free(&map);
}

static void
test_projection(void)
{
    airport_map_t map;
    double x, y;

    airport_map_init(&map);
    airport_map_geo_to_xy(&map, 50.0, 8.0, &x, &y);
    assert(x == 0 && y == 0);
    airport_map_geo_to_xy(&map, 50.0 + 1.0 / 60.0, 8.0, &x, &y);
    assert(fabs(y - 1853.2) < 2.0);
    airport_map_geo_to_xy(&map, 50.0, 8.0 + 1.0 / 60.0, &x, &y);
    assert(fabs(x - 1853.2 * cos(50.0 * 3.14159265358979 / 180)) < 2.0);
    airport_map_free(&map);
}

static void
test_concave_pavement(void)
{
    /* L-shaped apron: area = 3 squares of ~111 m x ~71 m at 50N */
    FILE *fp = tmpfile();
    airport_map_t map;
    double sq;

    assert(fp != NULL);
    fputs("1 0 0 0 XL Concave\n110 1 0.25 0 L\n"
        "111 50.000 8.000\n111 50.000 8.002\n111 50.001 8.002\n"
        "111 50.001 8.001\n111 50.002 8.001\n113 50.002 8.000\n", fp);
    rewind(fp);
    airport_map_init(&map);
    assert(airport_map_parse_stream(fp, "XL", &map));
    fclose(fp);
    sq = (0.001 * 3.14159265358979 / 180 * 6371008.8) *
        (0.001 * 3.14159265358979 / 180 * 6371008.8 *
        cos(50.0 * 3.14159265358979 / 180));
    assert(map.n_pave_tris == 12);
    assert(fabs(tri_area(map.pave_tris, map.n_pave_tris) - 3 * sq) < sq * 0.01);
    airport_map_free(&map);
}

int
main(int argc, char **argv)
{
    assert(argc == 2);
    test_parse_sample(argv[1]);
    test_missing_airport(argv[1]);
    test_projection();
    test_concave_pavement();
    printf("airport_map tests passed\n");
    return 0;
}
