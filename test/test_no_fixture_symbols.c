// Acceptance guard: the viewer must contain no code that behaves differently
// for synthetic rays.
//
// The fixtures are test harnesses that emit over the wire like any vehicle. If
// a fixture symbol ever appears in the map or ingest sources -- a name, an
// #ifdef, a special case -- then the viewer can tell a synthetic ray from a
// lidar, and the fixtures stop proving anything. This test greps for that.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef SRC_DIR
#define SRC_DIR "src"
#endif

#ifdef _WIN32
int main(void) {
    printf("no_fixture_symbols: skipped (POSIX directory walk)\n");
    return 0;
}
#else

#include <dirent.h>

// Symbols and headers that exist only under test/, plus the conditional
// compilation smells that would let the viewer branch on who sent a ray. None
// of them may appear in the *code* under src/.
static const char *const k_forbidden[] = {
    "ray_injector",
    "map_checker",
    "truth_ray",
    "truth_header",
    "truth_load",
    "geom_scene",
    "geom_plane",
    "ob_rotation",
    "ob_sensor_axis",
    "fixture",
    "FIXTURE",
    "FIXTURES_DIR",
    "synthetic",
    "SYNTHETIC",
    "TEST_ONLY",
    "HAWKEYE_TEST",
    "#ifdef TEST",
    "#if defined(TEST)",
};

// Blank out comments and string literals in place, preserving newlines so line
// numbers still line up. The rule being enforced is about code -- a comment
// explaining why the fixtures exist is documentation, not a code path that can
// tell a synthetic ray from a lidar.
static void strip_noncode(char *s) {
    enum { CODE, LINE_COMMENT, BLOCK_COMMENT, STRING, CHARLIT } st = CODE;
    for (char *p = s; *p; p++) {
        switch (st) {
            case CODE:
                if (p[0] == '/' && p[1] == '/') { st = LINE_COMMENT; p[0] = p[1] = ' '; p++; }
                else if (p[0] == '/' && p[1] == '*') { st = BLOCK_COMMENT; p[0] = p[1] = ' '; p++; }
                else if (p[0] == '"') { st = STRING; p[0] = ' '; }
                else if (p[0] == '\'') { st = CHARLIT; p[0] = ' '; }
                break;
            case LINE_COMMENT:
                if (*p == '\n') st = CODE;
                else *p = ' ';
                break;
            case BLOCK_COMMENT:
                if (p[0] == '*' && p[1] == '/') { st = CODE; p[0] = p[1] = ' '; p++; }
                else if (*p != '\n') *p = ' ';
                break;
            case STRING:
            case CHARLIT: {
                const char quote = (st == STRING) ? '"' : '\'';
                if (*p == '\\' && p[1]) { *p = ' '; p++; *p = ' '; }
                else if (*p == quote) { *p = ' '; st = CODE; }
                else if (*p != '\n') *p = ' ';
                break;
            }
        }
    }
}

static int scan_file(const char *dir, const char *name) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, name);

    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 1; }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    rewind(f);
    if (size <= 0) { fclose(f); return 0; }

    char *buf = (char *)malloc((size_t)size + 1);
    if (!buf) { fclose(f); return 1; }
    if (fread(buf, 1, (size_t)size, f) != (size_t)size) { free(buf); fclose(f); return 1; }
    buf[size] = '\0';
    fclose(f);

    // Includes are checked against the raw text, because stripping string
    // literals would also erase the header name we are looking for.
    int bad = 0;
    static const char *const k_test_headers[] = {
        "truth.h", "geom.h", "orientation_basis.h", "canvas.h", "ortho_render.h",
    };
    for (size_t i = 0; i < sizeof(k_test_headers) / sizeof(k_test_headers[0]); i++) {
        char needle[64];
        snprintf(needle, sizeof(needle), "#include \"%s\"", k_test_headers[i]);
        if (strstr(buf, needle)) {
            fprintf(stderr, "FAIL %s includes the test-only header \"%s\"\n",
                    path, k_test_headers[i]);
            bad++;
        }
    }

    strip_noncode(buf);

    for (size_t i = 0; i < sizeof(k_forbidden) / sizeof(k_forbidden[0]); i++) {
        const char *hit = strstr(buf, k_forbidden[i]);
        if (!hit) continue;
        // Report the line so the failure points at something actionable.
        int line = 1;
        for (const char *p = buf; p < hit; p++) if (*p == '\n') line++;
        fprintf(stderr, "FAIL %s:%d contains fixture symbol \"%s\"\n",
                path, line, k_forbidden[i]);
        bad++;
    }
    free(buf);
    return bad;
}

int main(void) {
    DIR *d = opendir(SRC_DIR);
    if (!d) { fprintf(stderr, "cannot open %s\n", SRC_DIR); return 1; }

    int bad = 0, files = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        const char *dot = strrchr(e->d_name, '.');
        if (!dot) continue;
        if (strcmp(dot, ".c") != 0 && strcmp(dot, ".h") != 0) continue;
        bad += scan_file(SRC_DIR, e->d_name);
        files++;
    }
    closedir(d);

    if (files == 0) { fprintf(stderr, "no sources scanned under %s\n", SRC_DIR); return 1; }
    printf("no_fixture_symbols: scanned %d files under %s, %d violations\n",
           files, SRC_DIR, bad);
    return bad ? 1 : 0;
}
#endif
