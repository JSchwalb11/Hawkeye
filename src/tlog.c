#include "tlog.h"

#include <stdlib.h>
#include <string.h>

#include <mavlink.h>

// A plausible UNIX microsecond timestamp: after 2001, before 2100. Used to
// resynchronise after a truncated or corrupt record rather than giving up on
// the rest of the file.
#define TLOG_TS_MIN_US 1000000000000000ULL
#define TLOG_TS_MAX_US 4102444800000000ULL

static uint64_t read_be64(const uint8_t *p) {
    return ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) | ((uint64_t)p[2] << 40)
         | ((uint64_t)p[3] << 32) | ((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16)
         | ((uint64_t)p[6] << 8)  | (uint64_t)p[7];
}

static void write_be64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (56 - 8 * i));
}

int tlog_reader_open(tlog_reader_t *r, const char *path, uint8_t channel) {
    if (!r || !path) return -1;
    memset(r, 0, sizeof(*r));
    r->channel = channel;

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
    // A fresh channel must not inherit a half-parsed frame from a previous run.
    mavlink_reset_channel_status(channel);
    return 0;
}

void tlog_reader_rewind(tlog_reader_t *r) {
    if (!r) return;
    r->pos = 0;
    r->frames_read = 0;
    mavlink_reset_channel_status(r->channel);
}

void tlog_reader_close(tlog_reader_t *r) {
    if (!r) return;
    free(r->data);
    r->data = NULL;
    r->len = r->pos = 0;
}

// Advance to the next byte offset that looks like the start of a record.
static bool resync(tlog_reader_t *r) {
    for (size_t p = r->pos + 1; p + 9 <= r->len; p++) {
        const uint64_t ts = read_be64(r->data + p);
        if (ts < TLOG_TS_MIN_US || ts > TLOG_TS_MAX_US) continue;
        const uint8_t magic = r->data[p + 8];
        if (magic != 0xFD && magic != 0xFE) continue;
        r->pos = p;
        r->resyncs++;
        return true;
    }
    r->pos = r->len;
    return false;
}

int tlog_reader_next(tlog_reader_t *r, struct __mavlink_message *msg_out, int64_t *arrival_ns) {
    if (!r || !r->data || !msg_out) return -1;
    mavlink_message_t *msg = (mavlink_message_t *)msg_out;
    mavlink_status_t status;

    for (;;) {
        if (r->pos + 9 > r->len) return 0;

        const uint64_t ts_us = read_be64(r->data + r->pos);
        if (ts_us < TLOG_TS_MIN_US || ts_us > TLOG_TS_MAX_US) {
            if (!resync(r)) return 0;
            continue;
        }

        size_t p = r->pos + 8;
        bool complete = false;
        // A v2 frame maxes out at 280 bytes; anything longer means we are not
        // looking at a frame boundary.
        const size_t limit = (p + 300 < r->len) ? p + 300 : r->len;
        while (p < limit) {
            if (mavlink_parse_char(r->channel, r->data[p++], msg, &status)) {
                complete = true;
                break;
            }
        }
        if (!complete) {
            mavlink_reset_channel_status(r->channel);
            if (!resync(r)) return 0;
            continue;
        }

        r->pos = p;
        r->frames_read++;
        const int64_t ns = (int64_t)ts_us * 1000LL;
        if (arrival_ns) *arrival_ns = ns;
        if (!r->span_known) { r->first_arrival_ns = ns; r->span_known = true; }
        r->last_arrival_ns = ns;
        return 1;
    }
}

bool tlog_reader_span(tlog_reader_t *r, int64_t *first_ns, int64_t *last_ns) {
    if (!r || !r->data) return false;
    const size_t saved = r->pos;

    tlog_reader_rewind(r);
    mavlink_message_t msg;
    int64_t t = 0, first = 0, last = 0;
    bool any = false;
    while (tlog_reader_next(r, (struct __mavlink_message *)&msg, &t) == 1) {
        if (!any) { first = t; any = true; }
        last = t;
    }
    tlog_reader_rewind(r);
    r->pos = saved;

    if (!any) return false;
    if (first_ns) *first_ns = first;
    if (last_ns) *last_ns = last;
    r->first_arrival_ns = first;
    r->last_arrival_ns = last;
    r->span_known = true;
    return true;
}

// ------------------------------------------------------------ writer

int tlog_writer_open(tlog_writer_t *w, const char *path) {
    if (!w || !path) return -1;
    memset(w, 0, sizeof(*w));
    w->f = fopen(path, "wb");
    return w->f ? 0 : -1;
}

int tlog_writer_write(tlog_writer_t *w, int64_t arrival_unix_ns,
                      const uint8_t *frame, size_t len) {
    if (!w || !w->f || !frame || len == 0) return -1;
    uint8_t hdr[8];
    write_be64(hdr, (uint64_t)(arrival_unix_ns / 1000LL));
    if (fwrite(hdr, 1, sizeof(hdr), w->f) != sizeof(hdr)) return -1;
    if (fwrite(frame, 1, len, w->f) != len) return -1;
    w->frames++;
    w->bytes += sizeof(hdr) + len;
    return 0;
}

int tlog_writer_write_msg(tlog_writer_t *w, int64_t arrival_unix_ns,
                          const struct __mavlink_message *msg_in) {
    if (!msg_in) return -1;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, (const mavlink_message_t *)msg_in);
    return tlog_writer_write(w, arrival_unix_ns, buf, len);
}

void tlog_writer_close(tlog_writer_t *w) {
    if (!w || !w->f) return;
    fclose(w->f);
    w->f = NULL;
}
