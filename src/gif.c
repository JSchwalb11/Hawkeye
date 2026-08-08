#include "gif.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Colours are histogrammed at 5 bits per channel. That is 32768 buckets, which
// is small enough to walk repeatedly during the median cut and fine enough that
// the quantisation it introduces is below what the palette will do anyway.
#define GIF_HIST_BITS 5
#define GIF_HIST_SIZE (1 << (GIF_HIST_BITS * 3))
#define GIF_MAX_COLOURS 256

struct gif_writer {
    FILE    *f;
    int      w, h;
    uint32_t frames;
    int      delay_cs;
    bool     ok;

    uint32_t *hist;        // GIF_HIST_SIZE population counts
    uint32_t *sum_r, *sum_g, *sum_b;
    uint16_t *populated;   // indices into hist that are non-zero
    uint8_t  *lut;         // 5-bit colour -> palette index
    uint8_t  *lut_valid;
    uint8_t  *indices;     // w*h palette indices for the frame being written
    uint8_t   palette[GIF_MAX_COLOURS * 3];
    int       palette_size;

    // LZW output accumulator.
    uint8_t  block[255];
    int      block_len;
    uint32_t bit_buf;
    int      bit_count;
};

// ------------------------------------------------------------ raw output

static void put_bytes(gif_writer_t *g, const void *p, size_t n) {
    if (!g->ok) return;
    if (fwrite(p, 1, n, g->f) != n) g->ok = false;
}

static void put_u8(gif_writer_t *g, uint8_t v) { put_bytes(g, &v, 1); }

static void put_u16(gif_writer_t *g, uint16_t v) {
    const uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    put_bytes(g, b, 2);
}

// ---------------------------------------------------------- median cut

typedef struct { int lo, hi; } gif_box_t;   // range within the populated list

static int g_sort_shift;   // channel being sorted on, as a bit shift

static int cmp_channel(const void *a, const void *b) {
    const uint16_t ia = *(const uint16_t *)a, ib = *(const uint16_t *)b;
    const int va = (ia >> g_sort_shift) & ((1 << GIF_HIST_BITS) - 1);
    const int vb = (ib >> g_sort_shift) & ((1 << GIF_HIST_BITS) - 1);
    return va - vb;
}

// Longest axis of a box, as a bit shift into the packed 5:5:5 index.
static int box_longest_axis(const gif_writer_t *g, const gif_box_t *b) {
    int lo[3] = { 255, 255, 255 }, hi[3] = { -1, -1, -1 };
    for (int i = b->lo; i <= b->hi; i++) {
        const uint16_t key = g->populated[i];
        const int c[3] = { (key >> 10) & 31, (key >> 5) & 31, key & 31 };
        for (int k = 0; k < 3; k++) {
            if (c[k] < lo[k]) lo[k] = c[k];
            if (c[k] > hi[k]) hi[k] = c[k];
        }
    }
    // Weighted for perceived luminance: a given spread in green matters more
    // than the same spread in blue, and a palette that ignores that spends its
    // entries on differences nobody can see.
    const int span[3] = { (hi[0] - lo[0]) * 30, (hi[1] - lo[1]) * 59, (hi[2] - lo[2]) * 11 };
    int best = 0;
    if (span[1] > span[best]) best = 1;
    if (span[2] > span[best]) best = 2;
    if (span[best] == 0) return -1;
    return (best == 0) ? 10 : (best == 1) ? 5 : 0;
}

static void build_palette(gif_writer_t *g, int want) {
    int pop_count = 0;
    for (int i = 0; i < GIF_HIST_SIZE; i++)
        if (g->hist[i]) g->populated[pop_count++] = (uint16_t)i;

    if (pop_count == 0) {
        g->palette_size = 1;
        g->palette[0] = g->palette[1] = g->palette[2] = 0;
        return;
    }

    gif_box_t boxes[GIF_MAX_COLOURS];
    int box_count = 1;
    boxes[0].lo = 0;
    boxes[0].hi = pop_count - 1;

    while (box_count < want) {
        // Split the box holding the most pixels along a real axis. Picking by
        // population rather than by extent is what stops one stray highlight
        // consuming half the palette.
        int pick = -1;
        uint64_t best_pop = 0;
        int pick_shift = -1;
        for (int i = 0; i < box_count; i++) {
            if (boxes[i].hi <= boxes[i].lo) continue;
            const int shift = box_longest_axis(g, &boxes[i]);
            if (shift < 0) continue;
            uint64_t pop = 0;
            for (int k = boxes[i].lo; k <= boxes[i].hi; k++) pop += g->hist[g->populated[k]];
            if (pop > best_pop) { best_pop = pop; pick = i; pick_shift = shift; }
        }
        if (pick < 0) break;

        gif_box_t *b = &boxes[pick];
        g_sort_shift = pick_shift;
        qsort(&g->populated[b->lo], (size_t)(b->hi - b->lo + 1),
              sizeof(uint16_t), cmp_channel);

        uint64_t total = 0;
        for (int k = b->lo; k <= b->hi; k++) total += g->hist[g->populated[k]];
        uint64_t half = 0;
        int split = b->lo;
        for (int k = b->lo; k < b->hi; k++) {
            half += g->hist[g->populated[k]];
            if (half * 2 >= total) { split = k; break; }
            split = k;
        }

        boxes[box_count].lo = split + 1;
        boxes[box_count].hi = b->hi;
        b->hi = split;
        box_count++;
    }

    for (int i = 0; i < box_count; i++) {
        uint64_t n = 0, r = 0, gg = 0, bb = 0;
        for (int k = boxes[i].lo; k <= boxes[i].hi; k++) {
            const uint16_t key = g->populated[k];
            n += g->hist[key];
            r += g->sum_r[key];
            gg += g->sum_g[key];
            bb += g->sum_b[key];
        }
        if (n == 0) { n = 1; }
        g->palette[i * 3 + 0] = (uint8_t)(r / n);
        g->palette[i * 3 + 1] = (uint8_t)(gg / n);
        g->palette[i * 3 + 2] = (uint8_t)(bb / n);
    }
    g->palette_size = box_count;
}

static uint8_t nearest_index(gif_writer_t *g, uint16_t key) {
    if (g->lut_valid[key]) return g->lut[key];
    const int r = (int)(((key >> 10) & 31) * 255 / 31);
    const int gg = (int)(((key >> 5) & 31) * 255 / 31);
    const int b = (int)((key & 31) * 255 / 31);
    int best = 0;
    long best_d = 1L << 30;
    for (int i = 0; i < g->palette_size; i++) {
        const long dr = r - g->palette[i * 3 + 0];
        const long dg = gg - g->palette[i * 3 + 1];
        const long db = b - g->palette[i * 3 + 2];
        const long d = dr * dr * 30 + dg * dg * 59 + db * db * 11;
        if (d < best_d) { best_d = d; best = i; }
    }
    g->lut[key] = (uint8_t)best;
    g->lut_valid[key] = 1;
    return (uint8_t)best;
}

// ------------------------------------------------------------------ LZW

static void flush_block(gif_writer_t *g) {
    if (g->block_len == 0) return;
    put_u8(g, (uint8_t)g->block_len);
    put_bytes(g, g->block, (size_t)g->block_len);
    g->block_len = 0;
}

static void emit_byte(gif_writer_t *g, uint8_t v) {
    g->block[g->block_len++] = v;
    if (g->block_len == 255) flush_block(g);
}

static void emit_code(gif_writer_t *g, int code, int code_bits) {
    g->bit_buf |= (uint32_t)code << g->bit_count;
    g->bit_count += code_bits;
    while (g->bit_count >= 8) {
        emit_byte(g, (uint8_t)(g->bit_buf & 0xFF));
        g->bit_buf >>= 8;
        g->bit_count -= 8;
    }
}

// The dictionary is a flat open-addressed table keyed by (prefix, suffix).
#define LZW_TABLE_SIZE 5003

static void lzw_encode(gif_writer_t *g, const uint8_t *idx, size_t count, int min_code_size) {
    const int clear_code = 1 << min_code_size;
    const int eoi_code = clear_code + 1;

    int32_t *keys = (int32_t *)malloc(LZW_TABLE_SIZE * sizeof(int32_t));
    int32_t *vals = (int32_t *)malloc(LZW_TABLE_SIZE * sizeof(int32_t));
    if (!keys || !vals) { free(keys); free(vals); g->ok = false; return; }

    int next_code = eoi_code + 1;
    int code_bits = min_code_size + 1;

    memset(keys, 0xFF, LZW_TABLE_SIZE * sizeof(int32_t));
    g->block_len = 0;
    g->bit_buf = 0;
    g->bit_count = 0;

    emit_code(g, clear_code, code_bits);

    if (count == 0) {
        emit_code(g, eoi_code, code_bits);
        if (g->bit_count > 0) emit_byte(g, (uint8_t)(g->bit_buf & 0xFF));
        flush_block(g);
        free(keys); free(vals);
        return;
    }

    int prefix = idx[0];
    for (size_t i = 1; i < count; i++) {
        const int suffix = idx[i];
        const int32_t key = (prefix << 8) | suffix;

        int slot = (int)(((uint32_t)key * 2654435761u) % LZW_TABLE_SIZE);
        int found = -1;
        for (;;) {
            if (keys[slot] == -1) break;
            if (keys[slot] == key) { found = vals[slot]; break; }
            slot++;
            if (slot == LZW_TABLE_SIZE) slot = 0;
        }

        if (found >= 0) { prefix = found; continue; }

        emit_code(g, prefix, code_bits);

        if (next_code < 4096) {
            keys[slot] = key;
            vals[slot] = next_code;
            if (next_code == (1 << code_bits) && code_bits < 12) code_bits++;
            next_code++;
        } else {
            // The dictionary is full. Reset it rather than stop growing: the
            // alternative degrades steadily on anything that changes over time,
            // and a map filling in changes constantly.
            emit_code(g, clear_code, code_bits);
            memset(keys, 0xFF, LZW_TABLE_SIZE * sizeof(int32_t));
            next_code = eoi_code + 1;
            code_bits = min_code_size + 1;
        }
        prefix = suffix;
    }

    emit_code(g, prefix, code_bits);
    emit_code(g, eoi_code, code_bits);
    if (g->bit_count > 0) emit_byte(g, (uint8_t)(g->bit_buf & 0xFF));
    flush_block(g);

    free(keys);
    free(vals);
}

// ------------------------------------------------------------------ API

gif_writer_t *gif_open(const char *path, int width, int height,
                       int delay_cs, bool loop) {
    if (!path || width <= 0 || height <= 0) return NULL;
    if (width > 65535 || height > 65535) return NULL;

    gif_writer_t *g = (gif_writer_t *)calloc(1, sizeof(*g));
    if (!g) return NULL;

    g->w = width;
    g->h = height;
    g->ok = true;
    g->hist      = (uint32_t *)calloc(GIF_HIST_SIZE, sizeof(uint32_t));
    g->sum_r     = (uint32_t *)calloc(GIF_HIST_SIZE, sizeof(uint32_t));
    g->sum_g     = (uint32_t *)calloc(GIF_HIST_SIZE, sizeof(uint32_t));
    g->sum_b     = (uint32_t *)calloc(GIF_HIST_SIZE, sizeof(uint32_t));
    g->populated = (uint16_t *)calloc(GIF_HIST_SIZE, sizeof(uint16_t));
    g->lut       = (uint8_t *)calloc(GIF_HIST_SIZE, 1);
    g->lut_valid = (uint8_t *)calloc(GIF_HIST_SIZE, 1);
    g->indices   = (uint8_t *)calloc((size_t)width * (size_t)height, 1);

    g->f = fopen(path, "wb");
    if (!g->f || !g->hist || !g->sum_r || !g->sum_g || !g->sum_b ||
        !g->populated || !g->lut || !g->lut_valid || !g->indices) {
        gif_close(g);
        return NULL;
    }

    put_bytes(g, "GIF89a", 6);
    put_u16(g, (uint16_t)width);
    put_u16(g, (uint16_t)height);
    put_u8(g, 0x70);   // no global colour table, 8-bit colour resolution
    put_u8(g, 0);      // background index
    put_u8(g, 0);      // pixel aspect ratio

    if (loop) {
        put_u8(g, 0x21); put_u8(g, 0xFF); put_u8(g, 11);
        put_bytes(g, "NETSCAPE2.0", 11);
        put_u8(g, 3); put_u8(g, 1);
        put_u16(g, 0);   // loop forever
        put_u8(g, 0);
    }

    if (delay_cs < 1) delay_cs = 1;
    if (delay_cs > 65535) delay_cs = 65535;
    g->delay_cs = delay_cs;
    return g;
}

bool gif_add_frame(gif_writer_t *g, const uint8_t *rgb) {
    if (!g || !g->ok || !rgb) return false;

    const size_t n = (size_t)g->w * (size_t)g->h;

    memset(g->hist, 0, GIF_HIST_SIZE * sizeof(uint32_t));
    memset(g->sum_r, 0, GIF_HIST_SIZE * sizeof(uint32_t));
    memset(g->sum_g, 0, GIF_HIST_SIZE * sizeof(uint32_t));
    memset(g->sum_b, 0, GIF_HIST_SIZE * sizeof(uint32_t));
    memset(g->lut_valid, 0, GIF_HIST_SIZE);

    for (size_t i = 0; i < n; i++) {
        const uint8_t r = rgb[i * 3 + 0], gg = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        const uint16_t key = (uint16_t)(((r >> 3) << 10) | ((gg >> 3) << 5) | (b >> 3));
        g->hist[key]++;
        g->sum_r[key] += r;
        g->sum_g[key] += gg;
        g->sum_b[key] += b;
    }

    build_palette(g, GIF_MAX_COLOURS);

    for (size_t i = 0; i < n; i++) {
        const uint8_t r = rgb[i * 3 + 0], gg = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        const uint16_t key = (uint16_t)(((r >> 3) << 10) | ((gg >> 3) << 5) | (b >> 3));
        g->indices[i] = nearest_index(g, key);
    }

    // GIF colour tables are a power of two, so round up and pad.
    int bits = 1;
    while ((1 << bits) < g->palette_size) bits++;
    if (bits > 8) bits = 8;
    const int table_size = 1 << bits;

    put_u8(g, 0x21); put_u8(g, 0xF9); put_u8(g, 4);
    put_u8(g, 0x04);                       // disposal: leave in place
    put_u16(g, (uint16_t)g->delay_cs);
    put_u8(g, 0);                          // transparent index (unused)
    put_u8(g, 0);

    put_u8(g, 0x2C);
    put_u16(g, 0); put_u16(g, 0);
    put_u16(g, (uint16_t)g->w);
    put_u16(g, (uint16_t)g->h);
    put_u8(g, (uint8_t)(0x80 | (bits - 1)));   // local colour table of 2^bits

    for (int i = 0; i < table_size; i++) {
        if (i < g->palette_size) put_bytes(g, &g->palette[i * 3], 3);
        else { const uint8_t z[3] = { 0, 0, 0 }; put_bytes(g, z, 3); }
    }

    const int min_code_size = (bits < 2) ? 2 : bits;
    put_u8(g, (uint8_t)min_code_size);

    lzw_encode(g, g->indices, n, min_code_size);
    put_u8(g, 0);   // block terminator

    g->frames++;
    return g->ok;
}

bool gif_close(gif_writer_t *g) {
    if (!g) return false;
    bool ok = g->ok;
    if (g->f) {
        if (g->frames > 0) put_u8(g, 0x3B);
        ok = g->ok;
        fclose(g->f);
    } else {
        ok = false;
    }
    free(g->hist); free(g->sum_r); free(g->sum_g); free(g->sum_b);
    free(g->populated); free(g->lut); free(g->lut_valid); free(g->indices);
    free(g);
    return ok;
}

uint32_t gif_frame_count(const gif_writer_t *g) { return g ? g->frames : 0; }
