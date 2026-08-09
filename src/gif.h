#ifndef HAWKEYE_GIF_H
#define HAWKEYE_GIF_H

// A streaming animated-GIF writer.
//
// Hawkeye's evidence is mostly motion: a map filling in, two vehicles merging
// their views, a playhead scrubbing back. A still frame cannot show any of it,
// and a video file would mean carrying a codec. GIF is the one animated format
// that is small enough to implement honestly -- a per-frame palette by median
// cut, then LZW -- and that renders inline everywhere the evidence gets read.
//
// Frames are written as they arrive; nothing is buffered beyond the one frame
// being encoded, so a long capture costs no more memory than a short one.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gif_writer gif_writer_t;

// delay_cs is the inter-frame delay in centiseconds, which is GIF's own unit.
// Values below 2 are widely clamped up to 10 by browsers, so 3 (33 fps) is
// about the fastest that plays back as written.
gif_writer_t *gif_open(const char *path, int width, int height,
                       int delay_cs, bool loop);

// `rgb` is width*height*3 bytes, top row first.
bool gif_add_frame(gif_writer_t *g, const uint8_t *rgb);

// Writes the trailer and closes. Returns false if any write failed along the
// way; the file is closed either way.
bool gif_close(gif_writer_t *g);

// Frames written so far -- handy for a one-line "captured N frames" report.
uint32_t gif_frame_count(const gif_writer_t *g);

#ifdef __cplusplus
}
#endif

#endif
