#ifdef __linux__
// recvmmsg() and struct mmsghdr are GNU extensions; the build uses -std=gnu11,
// which does not imply _GNU_SOURCE. Must precede every include.
#define _GNU_SOURCE
#endif

#include "mavlink_receiver.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#ifndef _WIN32
#include <time.h>
#endif

#ifdef _WIN32
#include <ws2tcpip.h>
#define SOCK_CLOSE(s) closesocket(s)
#else
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define SOCK_CLOSE(s) close(s)
#endif

// MAVLink config: use common message set
// MAVLINK_COMM_NUM_BUFFERS=256 is set via CMake for runtime-sized fleets.
#include <mavlink.h>

#define DISCONNECT_TIMEOUT_S 2.0

static void euler_to_quaternion(float roll, float pitch, float yaw, float q[4]) {
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static void request_home_position(mavlink_receiver_t *recv) {
    if (!recv->sender_known) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(255, 0, &msg,
        recv->sysid, 1,               // target system, component
        MAV_CMD_REQUEST_MESSAGE,       // command 512
        0,                             // confirmation
        MAVLINK_MSG_ID_HOME_POSITION,  // param1: message id to request
        0, 0, 0, 0, 0, 0);            // params 2-7 unused

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(recv->sockfd, (char *)buf, len, 0,
           (struct sockaddr *)recv->sender_addr, sizeof(struct sockaddr_in));
    printf("Requested HOME_POSITION from system %u\n", recv->sysid);
}

static void request_message_interval(mavlink_receiver_t *recv, uint32_t message_id,
                                     float interval_us) {
    if (!recv->sender_known) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(255, 0, &msg,
        recv->sysid, 1,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        (float)message_id, interval_us,
        0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(recv->sockfd, (char *)buf, len, 0,
           (struct sockaddr *)recv->sender_addr, sizeof(struct sockaddr_in));
}

static void request_native_position_stream(mavlink_receiver_t *recv) {
    // ArduPilot does not necessarily stream these until a GCS asks. Twenty Hz
    // is close to PX4 SIH's visualizer rate without wasting bandwidth.
    request_message_interval(recv, MAVLINK_MSG_ID_ATTITUDE, 50000.0f);
    request_message_interval(recv, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 50000.0f);
}

static double get_wall_time(void) {
#ifdef _WIN32
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return (double)count.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

int64_t mavlink_receiver_unix_ns(void) {
#ifdef _WIN32
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    uint64_t t = ((uint64_t)ft.dwHighDateTime << 32) | ft.dwLowDateTime;
    // FILETIME counts 100 ns ticks from 1601-01-01.
    return (int64_t)((t - 116444736000000000ULL) * 100ULL);
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
#endif
}

// Ask for a TIMESYNC round trip. The reply gives both a link latency number and
// a time reference better than "assume boot-relative zero".
static void request_timesync(mavlink_receiver_t *recv) {
    if (!recv->sender_known) return;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_timesync_pack(255, 0, &msg, 0, mavlink_receiver_unix_ns(),
                              recv->sysid, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(recv->sockfd, (char *)buf, len, 0,
           (struct sockaddr *)recv->sender_addr, sizeof(struct sockaddr_in));
}

static int set_nonblocking(sock_t s) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(s, FIONBIO, &mode);
#else
    int flags = fcntl(s, F_GETFL, 0);
    return fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
}

int mavlink_receiver_init(mavlink_receiver_t *recv, uint16_t port, uint8_t channel) {
    bool debug = recv->debug;
    void *frame_user = recv->frame_user;
    void (*on_frame)(void *, const void *, const uint8_t *, uint16_t, int64_t) = recv->on_frame;
    memset(recv, 0, sizeof(*recv));
    recv->debug = debug;
    recv->frame_user = frame_user;
    recv->on_frame = on_frame;
    recv->port = port;
    recv->channel = channel;
    recv->sockfd = SOCK_INVALID;

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed: %d\n", WSAGetLastError());
        return -1;
    }
#endif

    recv->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv->sockfd == SOCK_INVALID) {
        perror("socket");
        return -1;
    }

    set_nonblocking(recv->sockfd);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(recv->sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        SOCK_CLOSE(recv->sockfd);
        recv->sockfd = SOCK_INVALID;
        return -1;
    }

    printf("Listening for MAVLink on UDP port %u\n", port);
    return 0;
}

// Feed one datagram through the MAVLink byte parser. Split out of
// mavlink_receiver_poll() so the batched and the portable receive paths below
// dispatch messages through exactly the same code.
static void parse_datagram(mavlink_receiver_t *recv, const uint8_t *buf, int n) {
    mavlink_message_t msg;
    mavlink_status_t status;

    // Frame offsets are datagram-local, exactly as before: one call per
    // datagram, so a frame never spans two of them.
    size_t frame_start = 0;

    for (int i = 0; i < n; i++) {
        if (mavlink_parse_char(recv->channel, buf[i], &msg, &status)) {
            if (recv->on_frame) {
                // Hand on the exact bytes as they arrived: a recorder that
                // re-encodes would quietly normalise away the framing the
                // viewer actually saw.
                const size_t frame_len = (size_t)i + 1 - frame_start;
                recv->on_frame(recv->frame_user, &msg, buf + frame_start,
                               (uint16_t)frame_len, mavlink_receiver_unix_ns());
            }
            frame_start = (size_t)i + 1;

            if (recv->debug) {
                printf("[MAVLink] msgid=%u sysid=%u compid=%u seq=%u len=%u\n",
                       msg.msgid, msg.sysid, msg.compid, msg.seq, msg.len);
            }

            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    recv->mav_type = hb.type;
                    if (!recv->connected) {
                        recv->connected = true;
                        recv->sysid = msg.sysid;
                        printf("Connected to system %u (type %u)\n", msg.sysid, hb.type);
                        request_home_position(recv);
                        request_native_position_stream(recv);
                    }
                    break;
                }

                case MAVLINK_MSG_ID_HOME_POSITION: {
                    mavlink_home_position_t hp;
                    mavlink_msg_home_position_decode(&msg, &hp);
                    recv->home.lat = hp.latitude;
                    recv->home.lon = hp.longitude;
                    recv->home.alt = hp.altitude;
                    recv->home.valid = true;
                    printf("Home position: lat=%.7f lon=%.7f alt=%.1fm\n",
                           hp.latitude * 1e-7, hp.longitude * 1e-7, hp.altitude * 1e-3);
                    break;
                }

                case MAVLINK_MSG_ID_HIL_STATE_QUATERNION: {
                    mavlink_hil_state_quaternion_t hil;
                    mavlink_msg_hil_state_quaternion_decode(&msg, &hil);

                    recv->state.quaternion[0] = hil.attitude_quaternion[0];
                    recv->state.quaternion[1] = hil.attitude_quaternion[1];
                    recv->state.quaternion[2] = hil.attitude_quaternion[2];
                    recv->state.quaternion[3] = hil.attitude_quaternion[3];
                    recv->state.lat = hil.lat;
                    recv->state.lon = hil.lon;
                    recv->state.alt = hil.alt;
                    recv->state.vx = hil.vx;
                    recv->state.vy = hil.vy;
                    recv->state.vz = hil.vz;
                    recv->state.ind_airspeed = hil.ind_airspeed;
                    recv->state.true_airspeed = hil.true_airspeed;
                    recv->state.time_usec = hil.time_usec;
                    recv->state.valid = true;
                    recv->hil_valid = true;

                    if (recv->debug) {
                        printf("  HIL_STATE_Q: lat=%d lon=%d alt=%d q=[%.3f,%.3f,%.3f,%.3f]\n",
                               hil.lat, hil.lon, hil.alt,
                               hil.attitude_quaternion[0], hil.attitude_quaternion[1],
                               hil.attitude_quaternion[2], hil.attitude_quaternion[3]);
                    }
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE: {
                    // Fallback only. PX4 streams ATTITUDE alongside
                    // HIL_STATE_QUATERNION; taking it would downgrade the
                    // attitude and mix boot-relative into absolute time.
                    if (recv->hil_valid) break;
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    euler_to_quaternion(attitude.roll, attitude.pitch,
                                        attitude.yaw, recv->state.quaternion);
                    recv->attitude_valid = true;
                    recv->state.time_usec = (uint64_t)attitude.time_boot_ms * 1000ULL;
                    recv->state.valid = recv->global_position_valid;

                    if (recv->debug) {
                        printf("  ATTITUDE: rpy=[%.3f,%.3f,%.3f] q=[%.3f,%.3f,%.3f,%.3f]\n",
                               attitude.roll, attitude.pitch, attitude.yaw,
                               recv->state.quaternion[0], recv->state.quaternion[1],
                               recv->state.quaternion[2], recv->state.quaternion[3]);
                    }
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    if (recv->hil_valid) break;  // see ATTITUDE above
                    mavlink_global_position_int_t position;
                    mavlink_msg_global_position_int_decode(&msg, &position);
                    recv->state.lat = position.lat;
                    recv->state.lon = position.lon;
                    recv->state.alt = position.alt;
                    recv->state.vx = position.vx;
                    recv->state.vy = position.vy;
                    recv->state.vz = position.vz;
                    recv->state.time_usec = (uint64_t)position.time_boot_ms * 1000ULL;
                    recv->global_position_valid = true;
                    recv->state.valid = recv->attitude_valid;

                    if (recv->debug) {
                        printf("  GLOBAL_POSITION_INT: lat=%d lon=%d alt=%d vel=[%d,%d,%d]\n",
                               position.lat, position.lon, position.alt,
                               position.vx, position.vy, position.vz);
                    }
                    break;
                }
            }
        }
    }
}

#ifdef __linux__
// One recvfrom() per datagram is what the profile caught: 25 vehicles at 60 fps
// issued ~7,125 recvfrom syscalls/s, 1,500 of which were pure EAGAIN "nothing
// here" returns. recvmmsg() lifts up to RECV_BATCH datagrams per syscall.
#define RECV_BATCH 32
#define RECV_BUFSZ 2048

// Receive scratch shared by every receiver. Polling happens on the render
// thread only, one socket at a time, and nothing here outlives the call, so a
// single copy is enough — per-receiver buffers would cost 64 KB x 255 vehicles.
static uint8_t            batch_buf[RECV_BATCH][RECV_BUFSZ];
static struct mmsghdr     batch_msgs[RECV_BATCH];
static struct iovec       batch_iov[RECV_BATCH];
static struct sockaddr_in batch_addr[RECV_BATCH];
#endif

void mavlink_receiver_poll(mavlink_receiver_t *recv) {
    bool got_data = false;

#ifdef __linux__
    for (;;) {
        for (int i = 0; i < RECV_BATCH; i++) {
            batch_iov[i].iov_base = batch_buf[i];
            batch_iov[i].iov_len = RECV_BUFSZ;
            batch_msgs[i].msg_len = 0;
            batch_msgs[i].msg_hdr = (struct msghdr){
                .msg_name = &batch_addr[i],
                .msg_namelen = sizeof(batch_addr[i]),
                .msg_iov = &batch_iov[i],
                .msg_iovlen = 1,
            };
        }

        // The socket stays non-blocking, so this hands back whatever is queued
        // right now and -1/EAGAIN once the queue is empty.
        int count = recvmmsg(recv->sockfd, batch_msgs, RECV_BATCH, 0, NULL);
        if (count <= 0) break;

        got_data = true;

        if (!recv->sender_known) {
            memcpy(recv->sender_addr, &batch_addr[0], sizeof(struct sockaddr_in));
            recv->sender_known = true;
        }

        for (int i = 0; i < count; i++)
            parse_datagram(recv, batch_buf[i], (int)batch_msgs[i].msg_len);

        // A short batch means the queue is drained; skip the follow-up syscall
        // that could only return EAGAIN.
        if (count < RECV_BATCH) break;
    }
#else
    // Portable fallback (Windows, macOS): recvmmsg() is Linux-only.
    uint8_t buf[2048];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);

    for (;;) {
        int n = recvfrom(recv->sockfd, (char *)buf, sizeof(buf), 0,
                         (struct sockaddr *)&sender, &sender_len);
        if (n <= 0) break;

        got_data = true;

        if (!recv->sender_known) {
            memcpy(recv->sender_addr, &sender, sizeof(struct sockaddr_in));
            recv->sender_known = true;
        }

        parse_datagram(recv, buf, n);
    }
#endif

    if (got_data) {
        recv->last_msg_time = get_wall_time();
        // One round trip a second is enough to hold a time reference and a
        // latency number without adding meaningful uplink traffic.
        const double now = recv->last_msg_time;
        if (recv->connected && now - recv->last_timesync_s >= 1.0) {
            recv->last_timesync_s = now;
            request_timesync(recv);
        }
    } else if (recv->connected && recv->last_msg_time > 0) {
        double now = get_wall_time();
        if (now - recv->last_msg_time > DISCONNECT_TIMEOUT_S) {
            recv->connected = false;
            recv->state.valid = false;
            recv->home.valid = false;
            recv->hil_valid = false;
            recv->attitude_valid = false;
            recv->global_position_valid = false;
            recv->sender_known = false;
            printf("Disconnected from system %u\n", recv->sysid);
        }
    }
}

void mavlink_receiver_close(mavlink_receiver_t *recv) {
    if (recv->sockfd != SOCK_INVALID) {
        SOCK_CLOSE(recv->sockfd);
        recv->sockfd = SOCK_INVALID;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}
