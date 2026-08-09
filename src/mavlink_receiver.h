#ifndef MAVLINK_RECEIVER_H
#define MAVLINK_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef _WIN32
typedef uintptr_t sock_t;
#define SOCK_INVALID (~(sock_t)0)
#else
typedef int sock_t;
#define SOCK_INVALID (-1)
#endif

typedef struct {
    float quaternion[4]; // w, x, y, z
    int32_t lat;         // degE7
    int32_t lon;         // degE7
    int32_t alt;         // mm
    int16_t vx, vy, vz;          // cm/s, NED
    uint16_t ind_airspeed;       // cm/s
    uint16_t true_airspeed;      // cm/s
    uint64_t time_usec;          // timestamp (time since boot), microseconds
    bool valid;
} hil_state_t;

typedef struct {
    int32_t lat;         // degE7
    int32_t lon;         // degE7
    int32_t alt;         // mm (AMSL)
    bool valid;
} home_position_t;

typedef struct {
    sock_t sockfd;
    uint16_t port;
    uint8_t channel;
    bool connected;
    bool debug;
    uint8_t sysid;
    uint8_t mav_type;            // MAV_TYPE from heartbeat
    double last_msg_time;        // wall-clock time of last received message
    hil_state_t state;
    home_position_t home;
    bool hil_valid;             // HIL_STATE_QUATERNION seen; takes priority over native
    bool attitude_valid;        // ATTITUDE received (ArduPilot/native telemetry path)
    bool global_position_valid; // GLOBAL_POSITION_INT received
    bool sender_known;           // true once we've seen a packet
    uint8_t sender_addr[16];     // sockaddr_in stored as opaque bytes

    // Every parsed frame is handed on verbatim, with the wall-clock arrival
    // time. The shared map and the tlog recorder both hang off this; neither
    // needs the receiver to know what they do with it.
    void  *frame_user;
    void (*on_frame)(void *user, const void *mavlink_msg,
                     const uint8_t *raw, uint16_t raw_len, int64_t arrival_unix_ns);
    double last_timesync_s;      // when we last asked for a round trip
} mavlink_receiver_t;

// UNIX nanoseconds, for stamping arrivals and aligning against vehicle clocks.
int64_t mavlink_receiver_unix_ns(void);

// Initialize UDP socket on given port with MAVLink parse channel. Returns 0 on success.
int mavlink_receiver_init(mavlink_receiver_t *recv, uint16_t port, uint8_t channel);

// Poll for new messages (non-blocking). Call once per frame.
void mavlink_receiver_poll(mavlink_receiver_t *recv);

// Cleanup socket.
void mavlink_receiver_close(mavlink_receiver_t *recv);

#endif
