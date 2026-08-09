#ifndef DATAFLASH_H
#define DATAFLASH_H

// ArduPilot DataFlash (.bin) reader.
//
// The format is self-describing: FMT records declare every other record's
// layout, so rather than hard-coding offsets per firmware version this parser
// builds a field table and looks values up by label. ArduPilot has reshuffled
// PRX and RFND more than once; label lookup survives that, fixed offsets do not.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DF_MAX_FIELDS 32
#define DF_HEAD1 0xA3
#define DF_HEAD2 0x95
#define DF_FMT_TYPE 0x80

typedef struct {
    bool     known;
    uint8_t  type;
    uint8_t  length;         // total record length including the 3-byte header
    char     name[5];
    char     format[17];
    char     labels[65];
    uint8_t  field_count;
    char     field_name[DF_MAX_FIELDS][17];
    char     field_type[DF_MAX_FIELDS];
    uint16_t field_offset[DF_MAX_FIELDS];  // from the start of the payload
} df_format_t;

typedef struct {
    uint8_t       *data;
    size_t         len;
    size_t         pos;
    df_format_t    formats[256];
    uint64_t       records;
    uint64_t       unknown_records;
} df_reader_t;

typedef struct {
    const df_format_t *fmt;
    const uint8_t     *payload;
    size_t             payload_len;
} df_record_t;

int  df_reader_open(df_reader_t *r, const char *path);
void df_reader_close(df_reader_t *r);
void df_reader_rewind(df_reader_t *r);

// 1 = record produced, 0 = end of file, -1 = unrecoverable.
int  df_reader_next(df_reader_t *r, df_record_t *out);

// Field access by label. Returns false when the record has no such field.
bool df_field_double(const df_record_t *rec, const char *label, double *out);
bool df_field_int(const df_record_t *rec, const char *label, int64_t *out);
bool df_field_string(const df_record_t *rec, const char *label, char *out, size_t out_len);

// Convenience: TimeUS in nanoseconds. Returns false when absent.
bool df_record_time_ns(const df_record_t *rec, int64_t *out_ns);

bool df_record_is(const df_record_t *rec, const char *name);

#ifdef __cplusplus
}
#endif

#endif
