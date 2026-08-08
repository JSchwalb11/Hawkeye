#include "data_source.h"
#include "map_session.h"
#include "mavlink_map_decode.h"
#include "mavlink_receiver.h"
#include "tlog.h"

#include <stdlib.h>
#include <string.h>

#include <mavlink.h>

typedef struct {
    mavlink_receiver_t recv;
    data_source_t     *ds;
    tlog_writer_t      rec;
    bool               recording;
} mavlink_impl_t;

// Length the frame should have on the wire, so the recorder can tell a clean
// slice of the datagram from one that started in the previous packet. When they
// disagree we re-encode rather than write bytes we are not sure about.
static uint16_t expected_frame_len(const mavlink_message_t *msg) {
    if (msg->magic == MAVLINK_STX) {
        uint16_t n = (uint16_t)(12 + msg->len);
        if (msg->incompat_flags & MAVLINK_IFLAG_SIGNED) n = (uint16_t)(n + 13);
        return n;
    }
    return (uint16_t)(8 + msg->len);
}

static void on_frame(void *user, const void *mavlink_msg, const uint8_t *raw,
                     uint16_t raw_len, int64_t arrival_unix_ns) {
    mavlink_impl_t *impl = (mavlink_impl_t *)user;
    const mavlink_message_t *msg = (const mavlink_message_t *)mavlink_msg;
    data_source_t *ds = impl->ds;

    if (impl->recording) {
        if (raw && raw_len == expected_frame_len(msg))
            tlog_writer_write(&impl->rec, arrival_unix_ns, raw, raw_len);
        else
            tlog_writer_write_msg(&impl->rec, arrival_unix_ns, mavlink_msg);
    }

    if (ds->map) {
        const int slot = mavlink_map_decode((map_session_t *)ds->map, ds->map_slot,
                                            mavlink_msg, arrival_unix_ns);
        if (slot >= 0) {
            const map_session_t *ms = (const map_session_t *)ds->map;
            ds->time_provenance = (int)ms->veh[slot].tb.provenance;
            ds->time_offset_ns = ms->veh[slot].tb.offset_ns;
        }
    }
}

static void mavlink_poll(data_source_t *ds, float dt) {
    (void)dt;
    mavlink_impl_t *impl = (mavlink_impl_t *)ds->impl;
    mavlink_receiver_poll(&impl->recv);

    ds->state = impl->recv.state;
    ds->home = impl->recv.home;
    ds->connected = impl->recv.connected;
    ds->sysid = impl->recv.sysid;
    ds->mav_type = impl->recv.mav_type;
}

static void mavlink_close(data_source_t *ds) {
    mavlink_impl_t *impl = (mavlink_impl_t *)ds->impl;
    if (!impl) return;
    if (impl->recording) tlog_writer_close(&impl->rec);
    mavlink_receiver_close(&impl->recv);
    free(impl);
    ds->impl = NULL;
}

static const data_source_ops_t mavlink_ops = {
    .poll = mavlink_poll,
    .close = mavlink_close,
};

int data_source_mavlink_create(data_source_t *ds, uint16_t port, uint8_t channel, bool debug_flag) {
    memset(ds, 0, sizeof(*ds));
    ds->ops = &mavlink_ops;
    ds->map_slot = -1;

    mavlink_impl_t *impl = (mavlink_impl_t *)calloc(1, sizeof(mavlink_impl_t));
    if (!impl) return -1;
    impl->ds = ds;
    impl->recv.debug = debug_flag;
    impl->recv.frame_user = impl;
    impl->recv.on_frame = on_frame;

    int ret = mavlink_receiver_init(&impl->recv, port, channel);
    if (ret != 0) {
        free(impl);
        return ret;
    }

    ds->impl = impl;
    ds->debug = debug_flag;
    ds->playback.speed = 1.0f;
    return 0;
}

int data_source_mavlink_record(data_source_t *ds, const char *tlog_path) {
    if (!ds || ds->ops != &mavlink_ops) return -1;
    mavlink_impl_t *impl = (mavlink_impl_t *)ds->impl;
    if (!impl) return -1;

    if (impl->recording) {
        tlog_writer_close(&impl->rec);
        impl->recording = false;
    }
    if (!tlog_path) return 0;
    if (tlog_writer_open(&impl->rec, tlog_path) != 0) return -1;
    impl->recording = true;
    return 0;
}
