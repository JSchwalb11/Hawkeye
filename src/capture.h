#ifndef HAWKEYE_CAPTURE_H
#define HAWKEYE_CAPTURE_H

// Headless frame capture.
//
// A screenshot key is fine when someone is sitting at the machine. The
// interesting cases are not: a CI job proving the map renders after a scrub, a
// cooperative-mapping demo whose whole point is that it runs unattended, a
// regression that only shows up 900 seconds into a replay. All of those need
// the viewer to record itself and then leave.
//
// So this drives off the same frames the operator would see -- it reads the
// real framebuffer after the real draw, with no separate offscreen path that
// could quietly diverge from what ships. Under Xvfb or any software GL that is
// a fully headless capture; on a desktop it is the same thing with a window in
// front of it.
//
// PNG frames go to a directory (ffmpeg or any tool can make a video of them);
// a GIF is written inline. Both can run at once.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gif_writer gif_writer_t;

typedef struct {
    bool   active;

    char   png_dir[512];     // empty = no PNG sequence
    char   gif_path[512];    // empty = no GIF
    double fps;              // capture rate, independent of render rate
    int    downscale;        // 1 = full res; 2 = half, and so on
    double start_s;          // wall seconds to wait before the first frame
    double stop_s;           // wall seconds after which to stop and quit (0 = never)
    uint32_t max_frames;     // 0 = unlimited

    // --- runtime ---
    gif_writer_t *gif;
    uint32_t      frames;
    double        next_due_s;
    double        began_s;
    bool          finished;
    int           out_w, out_h;
    uint8_t      *scratch;   // downscaled RGB, out_w*out_h*3
} capture_t;

void capture_defaults(capture_t *c);

// Parse one argument. Returns the number of argv entries consumed (0 if the
// argument is not a capture flag).
int  capture_parse_arg(capture_t *c, int argc, char **argv, int i);

// Usage text for --help, so the flags document themselves in one place.
void capture_usage(void);

// Called once the window exists. Returns false if capture was requested and
// could not be set up, which the caller should treat as fatal -- silently
// producing no evidence is worse than refusing to start.
bool capture_begin(capture_t *c, const char *err_prefix);

// Call once per frame, after EndDrawing. Returns true when the run should end
// because --capture-seconds has elapsed or the frame cap was reached.
bool capture_tick(capture_t *c);

// Close the GIF and report. Safe to call when nothing was captured.
void capture_finish(capture_t *c);

#ifdef __cplusplus
}
#endif

#endif
