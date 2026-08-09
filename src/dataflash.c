#include "dataflash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Field widths and semantics of the DataFlash format characters.
static int df_type_size(char t) {
    switch (t) {
        case 'b': case 'B': case 'M': return 1;
        case 'h': case 'H': case 'c': case 'C': return 2;
        case 'i': case 'I': case 'e': case 'E': case 'L': case 'f': return 4;
        case 'd': case 'q': case 'Q': return 8;
        case 'n': return 4;
        case 'N': return 16;
        case 'Z': return 64;
        case 'a': return 64;   // int16_t[32]
        default:  return 0;
    }
}

// Types 'c', 'C', 'e' and 'E' are stored as integers scaled by 100.
static double df_type_scale(char t) {
    switch (t) {
        case 'c': case 'C': case 'e': case 'E': return 0.01;
        case 'L': return 1e-7;   // degE7 -> degrees
        default:  return 1.0;
    }
}

static int16_t rd_i16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static int32_t rd_i32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int64_t rd_i64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | p[i];
    return (int64_t)v;
}
static float rd_f32(const uint8_t *p) { float f; uint32_t v = rd_u32(p); memcpy(&f, &v, 4); return f; }
static double rd_f64(const uint8_t *p) { double d; int64_t v = rd_i64(p); memcpy(&d, &v, 8); return d; }

bool df_record_is(const df_record_t *rec, const char *name) {
    return rec && rec->fmt && strcmp(rec->fmt->name, name) == 0;
}

int df_reader_open(df_reader_t *r, const char *path) {
    if (!r || !path) return -1;
    memset(r, 0, sizeof(*r));

    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return -1; }
    const long size = ftell(f);
    if (size <= 0) { fclose(f); return -1; }
    rewind(f);

    r->data = (uint8_t *)malloc((size_t)size);
    if (!r->data) { fclose(f); return -1; }
    if (fread(r->data, 1, (size_t)size, f) != (size_t)size) {
        free(r->data); r->data = NULL; fclose(f); return -1;
    }
    fclose(f);
    r->len = (size_t)size;

    // The FMT record describes itself; seeding it means the first record in the
    // file parses like any other.
    df_format_t *self = &r->formats[DF_FMT_TYPE];
    self->known = true;
    self->type = DF_FMT_TYPE;
    self->length = 89;
    return 0;
}

void df_reader_close(df_reader_t *r) {
    if (!r) return;
    free(r->data);
    r->data = NULL;
    r->len = r->pos = 0;
}

void df_reader_rewind(df_reader_t *r) {
    if (r) { r->pos = 0; r->records = 0; }
}

static void register_format(df_reader_t *r, const uint8_t *p) {
    // FMT payload: Type(B) Length(B) Name(n) Format(N) Labels(Z)
    const uint8_t type = p[0];
    df_format_t *f = &r->formats[type];
    memset(f, 0, sizeof(*f));
    f->known = true;
    f->type = type;
    f->length = p[1];
    memcpy(f->name, p + 2, 4);   f->name[4] = '\0';
    memcpy(f->format, p + 6, 16); f->format[16] = '\0';
    memcpy(f->labels, p + 22, 64); f->labels[64] = '\0';

    // Trim the space-padded fixed-width strings.
    for (int i = 3; i >= 0; i--) { if (f->name[i] == ' ' || f->name[i] == '\0') f->name[i] = '\0'; else break; }

    char labels[65];
    memcpy(labels, f->labels, sizeof(labels));
    uint16_t offset = 0;
    int n = 0;
    char *save = labels;
    for (const char *fc = f->format; *fc && n < DF_MAX_FIELDS; fc++) {
        const int sz = df_type_size(*fc);
        if (sz == 0) break;   // unknown format character: stop describing fields
        f->field_type[n] = *fc;
        f->field_offset[n] = offset;

        char *comma = strchr(save, ',');
        if (comma) *comma = '\0';
        size_t label_len = strlen(save);
        if (label_len > sizeof(f->field_name[n]) - 1) label_len = sizeof(f->field_name[n]) - 1;
        memcpy(f->field_name[n], save, label_len);
        f->field_name[n][label_len] = '\0';
        save = comma ? comma + 1 : save + strlen(save);

        offset = (uint16_t)(offset + sz);
        n++;
    }
    f->field_count = (uint8_t)n;
}

int df_reader_next(df_reader_t *r, df_record_t *out) {
    if (!r || !r->data || !out) return -1;

    while (r->pos + 3 <= r->len) {
        if (r->data[r->pos] != DF_HEAD1 || r->data[r->pos + 1] != DF_HEAD2) {
            r->pos++;                  // hunt for the next header
            continue;
        }
        const uint8_t type = r->data[r->pos + 2];
        const df_format_t *fmt = &r->formats[type];
        if (!fmt->known || fmt->length < 3) {
            r->unknown_records++;
            r->pos++;
            continue;
        }
        if (r->pos + fmt->length > r->len) return 0;

        // Take the length now. A FMT record can describe FMT itself, and
        // register_format writes straight into r->formats[type] -- the same
        // struct `fmt` points at. Advancing by fmt->length afterwards would
        // step by the length the log just declared rather than the one this
        // record was actually validated against, and desynchronise the stream.
        const uint8_t rec_len = fmt->length;
        const uint8_t *payload = r->data + r->pos + 3;
        const size_t payload_len = (size_t)rec_len - 3;

        if (type == DF_FMT_TYPE && payload_len >= 86) register_format(r, payload);

        out->fmt = &r->formats[type];
        out->payload = payload;
        out->payload_len = payload_len;
        r->pos += rec_len;
        r->records++;
        return 1;
    }
    return 0;
}

static int find_field(const df_record_t *rec, const char *label) {
    if (!rec || !rec->fmt) return -1;
    for (int i = 0; i < rec->fmt->field_count; i++)
        if (strcmp(rec->fmt->field_name[i], label) == 0) return i;
    return -1;
}

static bool read_field(const df_record_t *rec, int idx, double *dv, int64_t *iv) {
    const df_format_t *f = rec->fmt;
    const char t = f->field_type[idx];
    const uint16_t off = f->field_offset[idx];
    if ((size_t)off + (size_t)df_type_size(t) > rec->payload_len) return false;
    const uint8_t *p = rec->payload + off;

    int64_t raw = 0;
    double real = 0.0;
    bool is_real = false;

    switch (t) {
        case 'b': raw = (int8_t)p[0]; break;
        case 'B': case 'M': raw = p[0]; break;
        case 'h': case 'c': raw = rd_i16(p); break;
        case 'H': case 'C': raw = rd_u16(p); break;
        case 'i': case 'e': case 'L': raw = rd_i32(p); break;
        case 'I': case 'E': raw = rd_u32(p); break;
        case 'q': case 'Q': raw = rd_i64(p); break;
        case 'f': real = (double)rd_f32(p); is_real = true; break;
        case 'd': real = rd_f64(p); is_real = true; break;
        default: return false;
    }

    if (dv) *dv = is_real ? real : (double)raw * df_type_scale(t);
    if (iv) *iv = is_real ? (int64_t)real : raw;
    return true;
}

bool df_field_double(const df_record_t *rec, const char *label, double *out) {
    const int i = find_field(rec, label);
    if (i < 0) return false;
    return read_field(rec, i, out, NULL);
}

bool df_field_int(const df_record_t *rec, const char *label, int64_t *out) {
    const int i = find_field(rec, label);
    if (i < 0) return false;
    return read_field(rec, i, NULL, out);
}

bool df_field_string(const df_record_t *rec, const char *label, char *out, size_t out_len) {
    const int i = find_field(rec, label);
    if (i < 0 || !out || out_len == 0) return false;
    const char t = rec->fmt->field_type[i];
    const int sz = df_type_size(t);
    if (t != 'n' && t != 'N' && t != 'Z') return false;
    const uint16_t off = rec->fmt->field_offset[i];
    if ((size_t)off + (size_t)sz > rec->payload_len) return false;

    size_t n = (size_t)sz;
    if (n > out_len - 1) n = out_len - 1;
    memcpy(out, rec->payload + off, n);
    out[n] = '\0';
    // Fields are space- or NUL-padded to their fixed width.
    for (size_t k = n; k-- > 0; ) {
        if (out[k] == ' ' || out[k] == '\0') out[k] = '\0';
        else break;
    }
    return true;
}

bool df_record_time_ns(const df_record_t *rec, int64_t *out_ns) {
    int64_t us = 0;
    if (!df_field_int(rec, "TimeUS", &us)) return false;
    // TimeUS is boot-relative microseconds read straight out of the log. A
    // corrupt or misaligned field can hand back anything a 64-bit integer
    // holds, and multiplying that by 1000 overflows into a negative timestamp
    // that then sets the timeline's span. Around 292 years of uptime is where
    // the honest range ends.
    if (us < 0 || us > INT64_MAX / 1000LL) return false;
    if (out_ns) *out_ns = us * 1000LL;
    return true;
}
