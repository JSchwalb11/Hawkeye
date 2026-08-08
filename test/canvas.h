#ifndef TEST_CANVAS_H
#define TEST_CANVAS_H

// A tiny RGB canvas and PNG writer, so the fixtures can publish orthographic
// views of the map they built. No dependencies: the PNG is emitted with stored
// (uncompressed) deflate blocks, which is a handful of lines and produces files
// every viewer reads.
//
// The point of the pictures is not decoration. Free, unknown and occupied have
// to be distinguishable, divergence has to be visible, and a vehicle's own
// contribution has to be separable -- those are acceptance criteria, and a
// rendered fixture is the cheapest honest evidence of them.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      w, h;
    uint8_t *px;   // RGB, row-major
} canvas_t;

int  canvas_init(canvas_t *c, int w, int h, uint32_t rgb_background);
void canvas_free(canvas_t *c);

void canvas_fill_rect(canvas_t *c, int x, int y, int w, int h, uint32_t rgb, float alpha);
void canvas_rect_outline(canvas_t *c, int x, int y, int w, int h, uint32_t rgb);
void canvas_hline(canvas_t *c, int x0, int x1, int y, uint32_t rgb, float alpha);
void canvas_vline(canvas_t *c, int x, int y0, int y1, uint32_t rgb, float alpha);
void canvas_plot(canvas_t *c, int x, int y, uint32_t rgb, float alpha);

// 5x7 bitmap text, uppercased. `scale` of 1 gives 5x7 pixels per glyph.
void canvas_text(canvas_t *c, int x, int y, const char *s, uint32_t rgb, int scale);
int  canvas_text_width(const char *s, int scale);

int  canvas_write_png(const canvas_t *c, const char *path);

#ifdef __cplusplus
}
#endif

#endif
