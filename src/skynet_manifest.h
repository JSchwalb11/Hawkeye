#ifndef SKYNET_MANIFEST_H
#define SKYNET_MANIFEST_H

// A skynet run record is not a log format -- it is a manifest. It names a run
// and points at the logs that run produced, plus whatever metadata the run
// carried. Opening one means opening the sources it lists, which are tlogs,
// ULogs or DataFlash logs handled by the parsers that already exist.
//
// The reader is a small JSON subset: objects, arrays, strings, numbers,
// booleans and null. No dependency, and a malformed manifest fails loudly with
// a line number rather than half-loading.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SKYNET_MAX_ENTRIES 256
#define SKYNET_PATH_MAX    1024

typedef enum {
    SKYNET_LOG_UNKNOWN = 0,
    SKYNET_LOG_TLOG,
    SKYNET_LOG_ULOG,
    SKYNET_LOG_DATAFLASH,
} skynet_log_kind_t;

const char *skynet_log_kind_name(skynet_log_kind_t k);

typedef struct {
    char              path[SKYNET_PATH_MAX];   // resolved against the manifest's directory
    skynet_log_kind_t kind;
    char              vehicle[64];
    int               sysid;                   // -1 when the manifest does not say
    double            time_offset_s;           // operator-supplied alignment, 0 if absent
    bool              has_offset;
} skynet_entry_t;

typedef struct {
    char            run_id[128];
    char            name[128];
    char            notes[256];
    char            origin_source[64];
    double          origin_lat, origin_lon, origin_alt;
    bool            has_origin;
    skynet_entry_t  entries[SKYNET_MAX_ENTRIES];
    int             entry_count;
} skynet_manifest_t;

// Returns 0 on success. On failure `err` (when given) receives a short
// human-readable reason.
int skynet_manifest_load(skynet_manifest_t *m, const char *path,
                         char *err, size_t err_len);

// Guess a log kind from a file extension.
skynet_log_kind_t skynet_kind_from_path(const char *path);

#ifdef __cplusplus
}
#endif

#endif
