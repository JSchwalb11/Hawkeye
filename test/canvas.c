#include "canvas.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int canvas_init(canvas_t *c, int w, int h, uint32_t bg) {
    if (!c || w <= 0 || h <= 0) return -1;
    c->w = w; c->h = h;
    c->px = (uint8_t *)malloc((size_t)w * (size_t)h * 3);
    if (!c->px) return -1;
    const uint8_t r = (uint8_t)(bg >> 16), g = (uint8_t)(bg >> 8), b = (uint8_t)bg;
    for (size_t i = 0; i < (size_t)w * (size_t)h; i++) {
        c->px[i * 3 + 0] = r;
        c->px[i * 3 + 1] = g;
        c->px[i * 3 + 2] = b;
    }
    return 0;
}

void canvas_free(canvas_t *c) {
    if (!c) return;
    free(c->px);
    c->px = NULL;
}

void canvas_plot(canvas_t *c, int x, int y, uint32_t rgb, float a) {
    if (x < 0 || y < 0 || x >= c->w || y >= c->h) return;
    if (a <= 0.0f) return;
    if (a > 1.0f) a = 1.0f;
    uint8_t *p = &c->px[((size_t)y * (size_t)c->w + (size_t)x) * 3];
    const uint8_t sr = (uint8_t)(rgb >> 16), sg = (uint8_t)(rgb >> 8), sb = (uint8_t)rgb;
    p[0] = (uint8_t)(p[0] + (sr - p[0]) * a);
    p[1] = (uint8_t)(p[1] + (sg - p[1]) * a);
    p[2] = (uint8_t)(p[2] + (sb - p[2]) * a);
}

void canvas_fill_rect(canvas_t *c, int x, int y, int w, int h, uint32_t rgb, float a) {
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
            canvas_plot(c, i, j, rgb, a);
}

void canvas_hline(canvas_t *c, int x0, int x1, int y, uint32_t rgb, float a) {
    if (x1 < x0) { const int t = x0; x0 = x1; x1 = t; }
    for (int x = x0; x <= x1; x++) canvas_plot(c, x, y, rgb, a);
}

void canvas_vline(canvas_t *c, int x, int y0, int y1, uint32_t rgb, float a) {
    if (y1 < y0) { const int t = y0; y0 = y1; y1 = t; }
    for (int y = y0; y <= y1; y++) canvas_plot(c, x, y, rgb, a);
}

void canvas_rect_outline(canvas_t *c, int x, int y, int w, int h, uint32_t rgb) {
    canvas_hline(c, x, x + w - 1, y, rgb, 1.0f);
    canvas_hline(c, x, x + w - 1, y + h - 1, rgb, 1.0f);
    canvas_vline(c, x, y, y + h - 1, rgb, 1.0f);
    canvas_vline(c, x + w - 1, y, y + h - 1, rgb, 1.0f);
}

// ---------------------------------------------------------------- text

// 5x7, one byte per column, bit 0 = top row. Uppercase, digits and the few
// symbols the labels need.
typedef struct { char ch; uint8_t col[5]; } glyph_t;

static const glyph_t k_font[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'J', {0x20,0x40,0x41,0x3F,0x01}},
    {'K', {0x7F,0x08,0x14,0x22,0x41}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'Q', {0x3E,0x41,0x51,0x21,0x5E}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}},
    {'V', {0x1F,0x20,0x40,0x20,0x1F}},
    {'W', {0x7F,0x20,0x18,0x20,0x7F}},
    {'X', {0x63,0x14,0x08,0x14,0x63}},
    {'Y', {0x03,0x04,0x78,0x04,0x03}},
    {'Z', {0x61,0x51,0x49,0x45,0x43}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x42,0x61,0x51,0x49,0x46}},
    {'3', {0x21,0x41,0x45,0x4B,0x31}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {'.', {0x00,0x00,0x40,0x00,0x00}},
    {',', {0x00,0x40,0x30,0x00,0x00}},
    {':', {0x00,0x00,0x24,0x00,0x00}},
    {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'+', {0x08,0x08,0x3E,0x08,0x08}},
    {'/', {0x20,0x10,0x08,0x04,0x02}},
    {'(', {0x00,0x1C,0x22,0x41,0x00}},
    {')', {0x00,0x41,0x22,0x1C,0x00}},
    {'%', {0x23,0x13,0x08,0x64,0x62}},
    {'=', {0x14,0x14,0x14,0x14,0x14}},
    {'>', {0x41,0x22,0x14,0x08,0x00}},
    {'<', {0x08,0x14,0x22,0x41,0x00}},
    {'?', {0x02,0x01,0x51,0x09,0x06}},
    {'!', {0x00,0x00,0x5F,0x00,0x00}},
    {'*', {0x14,0x08,0x3E,0x08,0x14}},
    {'#', {0x14,0x7F,0x14,0x7F,0x14}},
};

static const glyph_t *find_glyph(char ch) {
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    for (size_t i = 0; i < sizeof(k_font) / sizeof(k_font[0]); i++)
        if (k_font[i].ch == ch) return &k_font[i];
    return &k_font[0];
}

int canvas_text_width(const char *s, int scale) {
    if (scale < 1) scale = 1;
    return (int)strlen(s) * 6 * scale;
}

void canvas_text(canvas_t *c, int x, int y, const char *s, uint32_t rgb, int scale) {
    if (scale < 1) scale = 1;
    for (const char *p = s; *p; p++) {
        const glyph_t *g = find_glyph(*p);
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if (!(g->col[col] & (1u << row))) continue;
                canvas_fill_rect(c, x + col * scale, y + row * scale, scale, scale, rgb, 1.0f);
            }
        }
        x += 6 * scale;
    }
}

// ---------------------------------------------------------------- PNG

static uint32_t crc32_of(const uint8_t *data, size_t len, uint32_t crc) {
    static uint32_t table[256];
    static bool built = false;
    if (!built) {
        for (uint32_t n = 0; n < 256; n++) {
            uint32_t c = n;
            for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[n] = c;
        }
        built = true;
    }
    crc ^= 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

static void be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

static int write_chunk(FILE *f, const char *type, const uint8_t *data, size_t len) {
    uint8_t hdr[8];
    be32(hdr, (uint32_t)len);
    memcpy(hdr + 4, type, 4);
    if (fwrite(hdr, 1, 8, f) != 8) return -1;
    if (len && fwrite(data, 1, len, f) != len) return -1;

    uint32_t crc = crc32_of((const uint8_t *)type, 4, 0);
    if (len) crc = crc32_of(data, len, crc);
    uint8_t tail[4];
    be32(tail, crc);
    return (fwrite(tail, 1, 4, f) == 4) ? 0 : -1;
}

// ------------------------------------------------- fixed-Huffman deflate
//
// Enough of a compressor to keep the rendered fixtures out of the "too big to
// commit" category. Fixed Huffman codes and a greedy hash-chain match search:
// no dynamic tables, no lazy matching, and no zlib dependency.

typedef struct {
    uint8_t *out;
    size_t   len, cap;
    uint32_t bitbuf;
    int      bitcount;
    bool     failed;
} bitw_t;

static void bw_byte(bitw_t *w, uint8_t b) {
    if (w->len == w->cap) {
        const size_t cap = w->cap ? w->cap * 2 : 65536;
        uint8_t *p = (uint8_t *)realloc(w->out, cap);
        if (!p) { w->failed = true; return; }
        w->out = p;
        w->cap = cap;
    }
    w->out[w->len++] = b;
}

// Deflate is a little-endian bit stream: bits fill from the LSB up.
static void bw_bits(bitw_t *w, uint32_t value, int n) {
    w->bitbuf |= (value & ((1u << n) - 1u)) << w->bitcount;
    w->bitcount += n;
    while (w->bitcount >= 8) {
        bw_byte(w, (uint8_t)(w->bitbuf & 0xFF));
        w->bitbuf >>= 8;
        w->bitcount -= 8;
    }
}

// Huffman codes travel MSB-first, so they are reversed into the LSB-first stream.
static void bw_huff(bitw_t *w, uint32_t code, int n) {
    uint32_t rev = 0;
    for (int i = 0; i < n; i++) rev |= ((code >> i) & 1u) << (n - 1 - i);
    bw_bits(w, rev, n);
}

static void bw_flush(bitw_t *w) {
    if (w->bitcount > 0) bw_byte(w, (uint8_t)(w->bitbuf & 0xFF));
    w->bitbuf = 0;
    w->bitcount = 0;
}

static void emit_literal(bitw_t *w, uint8_t lit) {
    if (lit < 144) bw_huff(w, 0x30u + lit, 8);
    else           bw_huff(w, 0x190u + lit - 144, 9);
}

static const uint16_t k_len_base[29] = {
    3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258
};
static const uint8_t k_len_extra[29] = {
    0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0
};
static const uint16_t k_dist_base[30] = {
    1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,
    1025,1537,2049,3073,4097,6145,8193,12289,16385,24577
};
static const uint8_t k_dist_extra[30] = {
    0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13
};

static void emit_match(bitw_t *w, int length, int distance) {
    int li = 28;
    while (li > 0 && k_len_base[li] > length) li--;
    const int lcode = 257 + li;
    if (lcode < 280) bw_huff(w, (uint32_t)(lcode - 256), 7);
    else             bw_huff(w, 0xC0u + (uint32_t)(lcode - 280), 8);
    bw_bits(w, (uint32_t)(length - k_len_base[li]), k_len_extra[li]);

    int di = 29;
    while (di > 0 && k_dist_base[di] > distance) di--;
    bw_huff(w, (uint32_t)di, 5);
    bw_bits(w, (uint32_t)(distance - k_dist_base[di]), k_dist_extra[di]);
}

#define DEF_WINDOW  32768
#define DEF_HASHBIT 15
#define DEF_HASHSZ  (1 << DEF_HASHBIT)
#define DEF_MAXCHAIN 24

static uint32_t def_hash(const uint8_t *p) {
    return (uint32_t)(((p[0] << 10) ^ (p[1] << 5) ^ p[2]) & (DEF_HASHSZ - 1));
}

// Returns a malloc'd deflate stream, or NULL. `*out_len` receives its length.
static uint8_t *deflate_fixed(const uint8_t *src, size_t n, size_t *out_len) {
    bitw_t w;
    memset(&w, 0, sizeof(w));

    int32_t *head = (int32_t *)malloc((size_t)DEF_HASHSZ * sizeof(int32_t));
    int32_t *prev = (int32_t *)malloc(n * sizeof(int32_t) + sizeof(int32_t));
    if (!head || !prev) { free(head); free(prev); return NULL; }
    for (int i = 0; i < DEF_HASHSZ; i++) head[i] = -1;

    bw_bits(&w, 1, 1);   // final block
    bw_bits(&w, 1, 2);   // fixed Huffman

    size_t i = 0;
    while (i < n) {
        int best_len = 0, best_dist = 0;
        if (i + 3 <= n) {
            const uint32_t h = def_hash(src + i);
            int32_t cand = head[h];
            int chain = 0;
            while (cand >= 0 && chain++ < DEF_MAXCHAIN) {
                const size_t dist = i - (size_t)cand;
                if (dist == 0 || dist > DEF_WINDOW) break;
                size_t len = 0;
                const size_t maxlen = (n - i < 258) ? (n - i) : 258;
                while (len < maxlen && src[cand + len] == src[i + len]) len++;
                if ((int)len > best_len) { best_len = (int)len; best_dist = (int)dist; }
                if (best_len >= 258) break;
                cand = prev[cand];
            }
            prev[i] = head[h];
            head[h] = (int32_t)i;
        }

        if (best_len >= 3) {
            emit_match(&w, best_len, best_dist);
            // Keep the hash chain populated across the whole match, or the next
            // match search starts blind.
            for (int k = 1; k < best_len; k++) {
                const size_t j = i + (size_t)k;
                if (j + 3 > n) break;
                const uint32_t h = def_hash(src + j);
                prev[j] = head[h];
                head[h] = (int32_t)j;
            }
            i += (size_t)best_len;
        } else {
            emit_literal(&w, src[i]);
            i++;
        }
        if (w.failed) break;
    }

    bw_huff(&w, 0, 7);   // end of block
    bw_flush(&w);

    free(head);
    free(prev);
    if (w.failed) { free(w.out); return NULL; }
    *out_len = w.len;
    return w.out;
}

// Per-row filter choice by the usual minimum-absolute-sum heuristic. Flat
// regions and vertical edges both get a filter that turns them into runs, which
// is most of why the compressor above is enough.
static uint8_t choose_filter(const uint8_t *row, const uint8_t *up, size_t bpl,
                             int bpp, uint8_t *out) {
    uint8_t best = 0;
    uint64_t best_score = UINT64_MAX;
    for (int f = 0; f < 3; f++) {
        uint64_t score = 0;
        for (size_t x = 0; x < bpl; x++) {
            const uint8_t a = (x >= (size_t)bpp) ? row[x - bpp] : 0;
            const uint8_t b = up ? up[x] : 0;
            uint8_t v;
            if (f == 0) v = row[x];
            else if (f == 1) v = (uint8_t)(row[x] - a);
            else v = (uint8_t)(row[x] - b);
            score += (v < 128) ? v : (uint8_t)(256 - v);
        }
        if (score < best_score) { best_score = score; best = (uint8_t)f; }
    }
    for (size_t x = 0; x < bpl; x++) {
        const uint8_t a = (x >= (size_t)bpp) ? row[x - bpp] : 0;
        const uint8_t b = up ? up[x] : 0;
        if (best == 0) out[x] = row[x];
        else if (best == 1) out[x] = (uint8_t)(row[x] - a);
        else out[x] = (uint8_t)(row[x] - b);
    }
    return best;
}

int canvas_write_png(const canvas_t *c, const char *path) {
    if (!c || !c->px || !path) return -1;
    FILE *f = fopen(path, "wb");
    if (!f) return -1;

    static const uint8_t sig[8] = { 137, 'P', 'N', 'G', 13, 10, 26, 10 };
    if (fwrite(sig, 1, 8, f) != 8) { fclose(f); return -1; }

    uint8_t ihdr[13];
    be32(ihdr, (uint32_t)c->w);
    be32(ihdr + 4, (uint32_t)c->h);
    ihdr[8] = 8;    // bit depth
    ihdr[9] = 2;    // truecolour
    ihdr[10] = ihdr[11] = ihdr[12] = 0;
    if (write_chunk(f, "IHDR", ihdr, sizeof(ihdr)) != 0) { fclose(f); return -1; }

    // Filtered scanlines: one filter byte plus RGB.
    const size_t bpl = (size_t)c->w * 3;
    const size_t stride = bpl + 1;
    const size_t raw_len = stride * (size_t)c->h;
    uint8_t *raw = (uint8_t *)malloc(raw_len);
    if (!raw) { fclose(f); return -1; }
    for (int y = 0; y < c->h; y++) {
        const uint8_t *row = c->px + (size_t)y * bpl;
        const uint8_t *up = (y > 0) ? (c->px + (size_t)(y - 1) * bpl) : NULL;
        uint8_t *dst = raw + (size_t)y * stride;
        dst[0] = choose_filter(row, up, bpl, 3, dst + 1);
    }

    size_t def_len = 0;
    uint8_t *def = deflate_fixed(raw, raw_len, &def_len);
    if (!def) { free(raw); fclose(f); return -1; }

    uint8_t *z = (uint8_t *)malloc(def_len + 6);
    if (!z) { free(def); free(raw); fclose(f); return -1; }
    z[0] = 0x78;   // CM = deflate, CINFO = 7
    z[1] = 0x01;   // FCHECK so the header is a multiple of 31
    memcpy(z + 2, def, def_len);

    // Adler-32 of the uncompressed data.
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < raw_len; i++) {
        a = (a + raw[i]) % 65521;
        b = (b + a) % 65521;
    }
    be32(z + 2 + def_len, (b << 16) | a);

    const int rc = write_chunk(f, "IDAT", z, def_len + 6);
    free(z);
    free(def);
    free(raw);
    if (rc != 0) { fclose(f); return -1; }

    if (write_chunk(f, "IEND", NULL, 0) != 0) { fclose(f); return -1; }
    fclose(f);
    return 0;
}
