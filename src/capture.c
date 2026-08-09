#include "capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"

#include "gif.h"

#if defined(_WIN32)
#include <direct.h>
#define hk_mkdir(p) _mkdir(p)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define hk_mkdir(p) mkdir((p), 0755)
#endif

void capture_defaults(capture_t *c) {
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->fps = 10.0;
    c->downscale = 1;
}

static bool next_arg(int argc, char **argv, int i, const char **out) {
    if (i + 1 >= argc) return false;
    *out = argv[i + 1];
    return true;
}

int capture_parse_arg(capture_t *c, int argc, char **argv, int i) {
    if (!c || i >= argc) return 0;
    const char *a = argv[i];
    const char *v = NULL;

    if (strcmp(a, "--capture-dir") == 0 && next_arg(argc, argv, i, &v)) {
        snprintf(c->png_dir, sizeof(c->png_dir), "%s", v);
        c->active = true;
        return 2;
    }
    if (strcmp(a, "--capture-gif") == 0 && next_arg(argc, argv, i, &v)) {
        snprintf(c->gif_path, sizeof(c->gif_path), "%s", v);
        c->active = true;
        return 2;
    }
    if (strcmp(a, "--capture-fps") == 0 && next_arg(argc, argv, i, &v)) {
        c->fps = atof(v);
        if (c->fps <= 0.0) c->fps = 10.0;
        return 2;
    }
    if (strcmp(a, "--capture-scale") == 0 && next_arg(argc, argv, i, &v)) {
        c->downscale = atoi(v);
        if (c->downscale < 1) c->downscale = 1;
        return 2;
    }
    if (strcmp(a, "--capture-after") == 0 && next_arg(argc, argv, i, &v)) {
        c->start_s = atof(v);
        return 2;
    }
    if (strcmp(a, "--capture-frames") == 0 && next_arg(argc, argv, i, &v)) {
        const long n = atol(v);
        c->max_frames = (n > 0) ? (uint32_t)n : 0;
        return 2;
    }
    if (strcmp(a, "--exit-after") == 0 && next_arg(argc, argv, i, &v)) {
        c->stop_s = atof(v);
        // A wall-clock timeout is useful on its own -- a CI smoke test wants to
        // run the viewer for twenty seconds and see it exit, with no frames
        // written. The timer lives in capture_tick, which does nothing at all
        // unless the module is active, so this has to switch it on.
        c->active = true;
        return 2;
    }
    return 0;
}

void capture_usage(void) {
    printf("Headless capture:\n");
    printf("  --capture-dir <dir>   write frame-NNNNN.png into <dir>\n");
    printf("  --capture-gif <file>  write an animated GIF\n");
    printf("  --capture-fps <n>     capture rate, default 10\n");
    printf("  --capture-scale <n>   downscale by n (2 = half size)\n");
    printf("  --capture-after <s>   skip the first <s> seconds\n");
    printf("  --capture-frames <n>  stop after n frames\n");
    printf("  --exit-after <s>      quit after <s> seconds of wall time\n");
    printf("  Run under Xvfb for a fully headless capture, e.g.\n");
    printf("    xvfb-run -s \"-screen 0 1600x900x24\" hawkeye --tlog a.tlog \\\n");
    printf("      --capture-gif out.gif --exit-after 30\n");
}

bool capture_begin(capture_t *c, const char *err_prefix) {
    if (!c || !c->active) return true;
    if (!err_prefix) err_prefix = "capture";

    // Timer only: nothing to size, allocate or announce.
    if (!c->png_dir[0] && !c->gif_path[0]) {
        c->began_s = GetTime();
        c->next_due_s = c->began_s + c->start_s;
        return true;
    }

    const int sw = GetScreenWidth(), sh = GetScreenHeight();
    c->out_w = sw / c->downscale;
    c->out_h = sh / c->downscale;
    if (c->out_w < 1 || c->out_h < 1) {
        fprintf(stderr, "%s: screen is %dx%d, nothing to capture\n", err_prefix, sw, sh);
        return false;
    }

    if (c->png_dir[0]) {
        hk_mkdir(c->png_dir);   // an existing directory is fine
        char probe[600];
        snprintf(probe, sizeof(probe), "%s/.hawkeye-capture", c->png_dir);
        FILE *f = fopen(probe, "wb");
        if (!f) {
            fprintf(stderr, "%s: cannot write into %s\n", err_prefix, c->png_dir);
            return false;
        }
        fclose(f);
        remove(probe);
    }

    if (c->gif_path[0]) {
        // GIF delays are centiseconds, so the playback rate is quantised. Report
        // the rate actually written rather than the one asked for.
        int delay_cs = (int)(100.0 / c->fps + 0.5);
        if (delay_cs < 2) delay_cs = 2;
        c->gif = gif_open(c->gif_path, c->out_w, c->out_h, delay_cs, true);
        if (!c->gif) {
            fprintf(stderr, "%s: cannot open %s\n", err_prefix, c->gif_path);
            return false;
        }
        printf("capture: %s at %dx%d, %.1f fps (%d cs/frame)\n",
               c->gif_path, c->out_w, c->out_h, 100.0 / delay_cs, delay_cs);
    }
    if (c->png_dir[0])
        printf("capture: %s/frame-NNNNN.png at %dx%d, %.1f fps\n",
               c->png_dir, c->out_w, c->out_h, c->fps);

    c->scratch = (uint8_t *)malloc((size_t)c->out_w * (size_t)c->out_h * 3);
    if (!c->scratch) {
        fprintf(stderr, "%s: out of memory\n", err_prefix);
        return false;
    }

    c->began_s = GetTime();
    c->next_due_s = c->began_s + c->start_s;
    return true;
}

// Box-filter down by an integer factor. Point sampling would alias the map's
// one-pixel cell edges into a shimmer that reads as noise in the recording.
static void downscale_rgb(const uint8_t *src, int sw, int sh,
                          uint8_t *dst, int dw, int dh, int n) {
    for (int y = 0; y < dh; y++) {
        for (int x = 0; x < dw; x++) {
            uint32_t r = 0, g = 0, b = 0, count = 0;
            for (int j = 0; j < n; j++) {
                const int sy = y * n + j;
                if (sy >= sh) break;
                for (int i = 0; i < n; i++) {
                    const int sx = x * n + i;
                    if (sx >= sw) break;
                    const uint8_t *p = src + ((size_t)sy * sw + sx) * 4;
                    r += p[0]; g += p[1]; b += p[2];
                    count++;
                }
            }
            uint8_t *o = dst + ((size_t)y * dw + x) * 3;
            if (count) { o[0] = (uint8_t)(r / count); o[1] = (uint8_t)(g / count); o[2] = (uint8_t)(b / count); }
            else       { o[0] = o[1] = o[2] = 0; }
        }
    }
}

bool capture_tick(capture_t *c) {
    if (!c || !c->active || c->finished) return false;

    const double now = GetTime();
    if (c->stop_s > 0.0 && now - c->began_s >= c->stop_s) {
        c->finished = true;
        return true;
    }
    if (!c->png_dir[0] && !c->gif) return false;   // timer only
    if (now < c->next_due_s) return false;
    c->next_due_s = now + 1.0 / c->fps;

    Image shot = LoadImageFromScreen();
    if (!shot.data) return false;
    ImageFormat(&shot, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);

    if (c->png_dir[0]) {
        char path[600];
        snprintf(path, sizeof(path), "%s/frame-%05u.png", c->png_dir, c->frames);
        if (c->downscale > 1) {
            Image small = ImageCopy(shot);
            ImageResize(&small, c->out_w, c->out_h);
            ExportImage(small, path);
            UnloadImage(small);
        } else {
            ExportImage(shot, path);
        }
    }

    if (c->gif && c->scratch) {
        downscale_rgb((const uint8_t *)shot.data, shot.width, shot.height,
                      c->scratch, c->out_w, c->out_h, c->downscale);
        gif_add_frame(c->gif, c->scratch);
    }

    UnloadImage(shot);
    c->frames++;

    if (c->max_frames && c->frames >= c->max_frames) {
        c->finished = true;
        return true;
    }
    return false;
}

void capture_finish(capture_t *c) {
    if (!c || !c->active) return;
    if (c->gif) {
        const bool ok = gif_close(c->gif);
        printf("capture: wrote %u frame%s to %s%s\n", c->frames,
               c->frames == 1 ? "" : "s", c->gif_path, ok ? "" : "  (WRITE FAILED)");
        c->gif = NULL;
    } else if (c->png_dir[0]) {
        printf("capture: wrote %u frame%s to %s\n", c->frames,
               c->frames == 1 ? "" : "s", c->png_dir);
    }
    free(c->scratch);
    c->scratch = NULL;
    c->active = false;
}
