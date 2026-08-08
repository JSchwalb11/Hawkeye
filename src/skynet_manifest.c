#include "skynet_manifest.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *skynet_log_kind_name(skynet_log_kind_t k) {
    switch (k) {
        case SKYNET_LOG_TLOG:      return "tlog";
        case SKYNET_LOG_ULOG:      return "ULog";
        case SKYNET_LOG_DATAFLASH: return "DataFlash";
        default:                   return "unknown";
    }
}

skynet_log_kind_t skynet_kind_from_path(const char *path) {
    if (!path) return SKYNET_LOG_UNKNOWN;
    const char *dot = strrchr(path, '.');
    if (!dot) return SKYNET_LOG_UNKNOWN;
    if (strcmp(dot, ".tlog") == 0) return SKYNET_LOG_TLOG;
    if (strcmp(dot, ".ulg") == 0)  return SKYNET_LOG_ULOG;
    if (strcmp(dot, ".bin") == 0 || strcmp(dot, ".BIN") == 0) return SKYNET_LOG_DATAFLASH;
    return SKYNET_LOG_UNKNOWN;
}

// ---------------------------------------------------------------- JSON

typedef struct {
    const char *p;
    const char *end;
    int         line;
    char        err[160];
    bool        failed;
} sj_t;

static void sj_fail(sj_t *j, const char *what) {
    if (j->failed) return;
    j->failed = true;
    snprintf(j->err, sizeof(j->err), "line %d: %s", j->line, what);
}

static void sj_skip_ws(sj_t *j) {
    while (j->p < j->end) {
        if (*j->p == '\n') { j->line++; j->p++; }
        else if (isspace((unsigned char)*j->p)) j->p++;
        else break;
    }
}

static bool sj_accept(sj_t *j, char c) {
    sj_skip_ws(j);
    if (j->p < j->end && *j->p == c) { j->p++; return true; }
    return false;
}

static void sj_expect(sj_t *j, char c) {
    if (!sj_accept(j, c)) {
        char msg[32];
        snprintf(msg, sizeof(msg), "expected '%c'", c);
        sj_fail(j, msg);
    }
}

static bool sj_string(sj_t *j, char *out, size_t out_len) {
    sj_skip_ws(j);
    if (j->p >= j->end || *j->p != '"') { sj_fail(j, "expected string"); return false; }
    j->p++;
    size_t n = 0;
    while (j->p < j->end && *j->p != '"') {
        char c = *j->p++;
        if (c == '\\' && j->p < j->end) {
            const char e = *j->p++;
            switch (e) {
                case 'n': c = '\n'; break;
                case 't': c = '\t'; break;
                case 'r': c = '\r'; break;
                case 'u':
                    // Not a unicode-aware viewer field; skip the code point.
                    for (int k = 0; k < 4 && j->p < j->end; k++) j->p++;
                    continue;
                default: c = e; break;
            }
        }
        if (c == '\n') j->line++;
        if (out && n + 1 < out_len) out[n++] = c;
    }
    if (j->p >= j->end) { sj_fail(j, "unterminated string"); return false; }
    j->p++;
    if (out && out_len) out[n] = '\0';
    return true;
}

static bool sj_number(sj_t *j, double *out) {
    sj_skip_ws(j);
    char *endp = NULL;
    const double v = strtod(j->p, &endp);
    if (endp == j->p) { sj_fail(j, "expected number"); return false; }
    j->p = endp;
    if (out) *out = v;
    return true;
}

static void sj_skip_value(sj_t *j);

static void sj_skip_container(sj_t *j, char open, char close) {
    sj_expect(j, open);
    if (sj_accept(j, close)) return;
    do {
        if (j->failed) return;
        if (open == '{') { sj_string(j, NULL, 0); sj_expect(j, ':'); }
        sj_skip_value(j);
    } while (sj_accept(j, ','));
    sj_expect(j, close);
}

static void sj_skip_value(sj_t *j) {
    sj_skip_ws(j);
    if (j->p >= j->end) { sj_fail(j, "unexpected end"); return; }
    switch (*j->p) {
        case '"': sj_string(j, NULL, 0); break;
        case '{': sj_skip_container(j, '{', '}'); break;
        case '[': sj_skip_container(j, '[', ']'); break;
        case 't': case 'f': case 'n':
            while (j->p < j->end && isalpha((unsigned char)*j->p)) j->p++;
            break;
        default: sj_number(j, NULL); break;
    }
}

// ---------------------------------------------------------------- manifest

static void resolve_path(const char *manifest_path, const char *rel, char *out, size_t out_len) {
    if (rel[0] == '/' || (rel[0] && rel[1] == ':')) {
        snprintf(out, out_len, "%s", rel);
        return;
    }
    const char *slash = strrchr(manifest_path, '/');
#ifdef _WIN32
    const char *back = strrchr(manifest_path, '\\');
    if (back && (!slash || back > slash)) slash = back;
#endif
    if (!slash) { snprintf(out, out_len, "%s", rel); return; }
    const size_t dir_len = (size_t)(slash - manifest_path) + 1;
    snprintf(out, out_len, "%.*s%s", (int)dir_len, manifest_path, rel);
}

static void parse_entry(sj_t *j, skynet_manifest_t *m, const char *manifest_path) {
    if (m->entry_count >= SKYNET_MAX_ENTRIES) { sj_skip_value(j); return; }
    skynet_entry_t *e = &m->entries[m->entry_count];
    memset(e, 0, sizeof(*e));
    e->sysid = -1;

    char rel[SKYNET_PATH_MAX] = {0};
    char kind[32] = {0};

    sj_expect(j, '{');
    if (!sj_accept(j, '}')) {
        do {
            if (j->failed) return;
            char key[64];
            if (!sj_string(j, key, sizeof(key))) return;
            sj_expect(j, ':');

            if (strcmp(key, "path") == 0 || strcmp(key, "file") == 0) {
                sj_string(j, rel, sizeof(rel));
            } else if (strcmp(key, "kind") == 0 || strcmp(key, "type") == 0) {
                sj_string(j, kind, sizeof(kind));
            } else if (strcmp(key, "vehicle") == 0 || strcmp(key, "name") == 0) {
                sj_string(j, e->vehicle, sizeof(e->vehicle));
            } else if (strcmp(key, "sysid") == 0) {
                double v = 0.0; if (sj_number(j, &v)) e->sysid = (int)v;
            } else if (strcmp(key, "time_offset_s") == 0) {
                double v = 0.0;
                if (sj_number(j, &v)) { e->time_offset_s = v; e->has_offset = true; }
            } else {
                sj_skip_value(j);
            }
        } while (sj_accept(j, ','));
        sj_expect(j, '}');
    }
    if (j->failed) return;
    if (rel[0] == '\0') return;   // an entry without a path names nothing

    resolve_path(manifest_path, rel, e->path, sizeof(e->path));
    if (kind[0]) {
        if (strcmp(kind, "tlog") == 0) e->kind = SKYNET_LOG_TLOG;
        else if (strcmp(kind, "ulog") == 0 || strcmp(kind, "ulg") == 0) e->kind = SKYNET_LOG_ULOG;
        else if (strcmp(kind, "dataflash") == 0 || strcmp(kind, "bin") == 0)
            e->kind = SKYNET_LOG_DATAFLASH;
    }
    if (e->kind == SKYNET_LOG_UNKNOWN) e->kind = skynet_kind_from_path(e->path);
    m->entry_count++;
}

static void parse_origin(sj_t *j, skynet_manifest_t *m) {
    sj_expect(j, '{');
    if (sj_accept(j, '}')) return;
    do {
        if (j->failed) return;
        char key[64];
        if (!sj_string(j, key, sizeof(key))) return;
        sj_expect(j, ':');
        if (strcmp(key, "lat") == 0)      { sj_number(j, &m->origin_lat); m->has_origin = true; }
        else if (strcmp(key, "lon") == 0) { sj_number(j, &m->origin_lon); m->has_origin = true; }
        else if (strcmp(key, "alt") == 0) { sj_number(j, &m->origin_alt); }
        else if (strcmp(key, "source") == 0) sj_string(j, m->origin_source, sizeof(m->origin_source));
        else sj_skip_value(j);
    } while (sj_accept(j, ','));
    sj_expect(j, '}');
}

int skynet_manifest_load(skynet_manifest_t *m, const char *path,
                         char *err, size_t err_len) {
    if (!m || !path) return -1;
    memset(m, 0, sizeof(*m));

    FILE *f = fopen(path, "rb");
    if (!f) {
        if (err) snprintf(err, err_len, "cannot open %s", path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    rewind(f);
    if (size <= 0) { fclose(f); if (err) snprintf(err, err_len, "empty manifest"); return -1; }

    char *buf = (char *)malloc((size_t)size + 1);
    if (!buf) { fclose(f); if (err) snprintf(err, err_len, "out of memory"); return -1; }
    if (fread(buf, 1, (size_t)size, f) != (size_t)size) {
        free(buf); fclose(f);
        if (err) snprintf(err, err_len, "short read");
        return -1;
    }
    fclose(f);
    buf[size] = '\0';

    sj_t j = { buf, buf + size, 1, {0}, false };
    sj_expect(&j, '{');
    if (!sj_accept(&j, '}')) {
        do {
            if (j.failed) break;
            char key[64];
            if (!sj_string(&j, key, sizeof(key))) break;
            sj_expect(&j, ':');

            if (strcmp(key, "run_id") == 0 || strcmp(key, "id") == 0) {
                sj_string(&j, m->run_id, sizeof(m->run_id));
            } else if (strcmp(key, "name") == 0) {
                sj_string(&j, m->name, sizeof(m->name));
            } else if (strcmp(key, "notes") == 0) {
                sj_string(&j, m->notes, sizeof(m->notes));
            } else if (strcmp(key, "origin") == 0) {
                parse_origin(&j, m);
            } else if (strcmp(key, "logs") == 0 || strcmp(key, "sources") == 0) {
                sj_expect(&j, '[');
                if (!sj_accept(&j, ']')) {
                    do { parse_entry(&j, m, path); } while (sj_accept(&j, ','));
                    sj_expect(&j, ']');
                }
            } else {
                sj_skip_value(&j);
            }
        } while (sj_accept(&j, ','));
        sj_expect(&j, '}');
    }

    const bool failed = j.failed;
    if (failed && err) snprintf(err, err_len, "%s", j.err);
    free(buf);

    if (failed) return -1;
    if (m->entry_count == 0) {
        if (err) snprintf(err, err_len, "manifest lists no logs");
        return -1;
    }
    return 0;
}
