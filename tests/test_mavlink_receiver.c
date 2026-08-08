#include "mavlink_receiver.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <mavlink.h>

static void send_message(int sock, const struct sockaddr_in *destination,
                         const mavlink_message_t *message) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(bytes, message);
    assert(sendto(sock, bytes, length, 0, (const struct sockaddr *)destination,
                  sizeof(*destination)) == length);
}

int main(void) {
    const uint16_t port = 29410;
    mavlink_receiver_t receiver = {.debug = false};
    assert(mavlink_receiver_init(&receiver, port, 0) == 0);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    assert(sock >= 0);
    struct sockaddr_in destination = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
    };
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    mavlink_message_t message;
    mavlink_msg_heartbeat_pack(1, 1, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0,
                               MAV_STATE_ACTIVE);
    send_message(sock, &destination, &message);

    const float half_pi = 1.57079632679f;
    mavlink_msg_attitude_pack(1, 1, &message, 1234, 0.0f, 0.0f, half_pi,
                              0.0f, 0.0f, 0.0f);
    send_message(sock, &destination, &message);
    mavlink_msg_global_position_int_pack(1, 1, &message, 1240,
                                         473977420, 85455940, 500000, 10600,
                                         120, -230, 40, 9000);
    send_message(sock, &destination, &message);

    usleep(10000);
    mavlink_receiver_poll(&receiver);

    assert(receiver.connected);
    assert(receiver.state.valid);
    assert(receiver.state.lat == 473977420);
    assert(receiver.state.lon == 85455940);
    assert(receiver.state.alt == 500000);
    assert(receiver.state.vx == 120);
    assert(receiver.state.vy == -230);
    assert(receiver.state.vz == 40);
    assert(receiver.state.time_usec == 1240000ULL);
    assert(fabsf(receiver.state.quaternion[0] - 0.70710678f) < 1e-5f);
    assert(fabsf(receiver.state.quaternion[3] - 0.70710678f) < 1e-5f);

    close(sock);
    mavlink_receiver_close(&receiver);
    puts("PASS native ATTITUDE + GLOBAL_POSITION_INT receiver");
    return 0;
}
