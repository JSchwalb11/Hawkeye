#ifndef MAVLINK_MAP_DECODE_H
#define MAVLINK_MAP_DECODE_H

// MAVLink -> map session. The only file that needs the dialect headers on the
// ingest side, so the map, the timeline and the transforms stay testable
// without them.
//
// The message is the contract: if DISTANCE_SENSOR or OBSTACLE_DISTANCE arrives
// it is truth, whether it came from a lidar, ArduPilot's proximity simulation,
// Gazebo, or a test harness. There is deliberately no branch here that asks who
// sent it.

#include <stdbool.h>
#include <stdint.h>

#include "map_session.h"

#ifdef __cplusplus
extern "C" {
#endif

struct __mavlink_message;

// Feed one decoded frame.
//
//   slot              vehicle slot, or -1 to resolve from the frame's sysid
//   arrival_unix_ns   wall-clock arrival, or 0 when unknown. tlog replay passes
//                     the recorded stamp; the live path passes the receive time.
//
// Returns the vehicle slot the frame was attributed to, or -1.
int mavlink_map_decode(map_session_t *ms, int slot,
                       const struct __mavlink_message *msg,
                       int64_t arrival_unix_ns);

// Session time the last decoded frame resolved to. Useful for sources that
// want to drive a playhead directly from decode order.
int64_t mavlink_map_decode_last_session_ns(void);

#ifdef __cplusplus
}
#endif

#endif
