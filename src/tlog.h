#ifndef TLOG_H
#define TLOG_H

// tlog: raw MAVLink frames, each preceded by a big-endian 64-bit microsecond
// UNIX arrival timestamp. The format QGroundControl and MAVProxy write.
//
// This is the only source that preserves what the viewer actually saw --
// latency, loss and ordering included -- which makes it both the recorder and
// the regression format. A ULog tells you what the vehicle believed; a tlog
// tells you what arrived.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct __mavlink_message;

typedef struct {
    uint8_t *data;
    size_t   len;
    size_t   pos;
    uint8_t  channel;         // MAVLink parse channel, one per open reader
    uint64_t frames_read;
    uint64_t resyncs;         // times the reader had to hunt for the next record
    int64_t  first_arrival_ns;
    int64_t  last_arrival_ns;
    bool     span_known;
} tlog_reader_t;

// `channel` must be unique among concurrently open parsers.
int  tlog_reader_open(tlog_reader_t *r, const char *path, uint8_t channel);

// 1 = frame produced, 0 = end of log, -1 = unrecoverable.
int  tlog_reader_next(tlog_reader_t *r, struct __mavlink_message *msg, int64_t *arrival_ns);

void tlog_reader_rewind(tlog_reader_t *r);
void tlog_reader_close(tlog_reader_t *r);

// Scan the whole file for its time span without keeping the frames.
bool tlog_reader_span(tlog_reader_t *r, int64_t *first_ns, int64_t *last_ns);

typedef struct {
    FILE    *f;
    uint64_t frames;
    uint64_t bytes;
} tlog_writer_t;

int  tlog_writer_open(tlog_writer_t *w, const char *path);
int  tlog_writer_write(tlog_writer_t *w, int64_t arrival_unix_ns,
                       const uint8_t *frame, size_t len);
int  tlog_writer_write_msg(tlog_writer_t *w, int64_t arrival_unix_ns,
                           const struct __mavlink_message *msg);
void tlog_writer_close(tlog_writer_t *w);

#ifdef __cplusplus
}
#endif

#endif
