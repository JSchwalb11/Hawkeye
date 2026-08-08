#include "mavlink_map_decode.h"

#include <math.h>
#include <string.h>

#include <mavlink.h>

// EKF_STATUS_REPORT is an ardupilotmega message; the id is stable and the
// layout is unpacked by hand below.
#define HAWKEYE_MSG_ID_EKF_STATUS_REPORT 193

static int64_t g_last_session_ns = 0;

int64_t mavlink_map_decode_last_session_ns(void) { return g_last_session_ns; }

// Timestamp carried by the frame, in the vehicle's own clock. Returns false
// when the message has no time field at all, in which case the caller keeps the
// last value seen for that vehicle.
static bool frame_source_ns(const mavlink_message_t *msg, int64_t *out) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_SYSTEM_TIME:
            *out = (int64_t)mavlink_msg_system_time_get_time_boot_ms(msg) * 1000000LL;
            return true;
        case MAVLINK_MSG_ID_ATTITUDE:
            *out = (int64_t)mavlink_msg_attitude_get_time_boot_ms(msg) * 1000000LL;
            return true;
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            *out = (int64_t)mavlink_msg_attitude_quaternion_get_time_boot_ms(msg) * 1000000LL;
            return true;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            *out = (int64_t)mavlink_msg_global_position_int_get_time_boot_ms(msg) * 1000000LL;
            return true;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            *out = (int64_t)mavlink_msg_local_position_ned_get_time_boot_ms(msg) * 1000000LL;
            return true;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
            *out = (int64_t)mavlink_msg_distance_sensor_get_time_boot_ms(msg) * 1000000LL;
            return true;
        case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
            *out = (int64_t)mavlink_msg_obstacle_distance_get_time_usec(msg) * 1000LL;
            return true;
        case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
            *out = (int64_t)mavlink_msg_hil_state_quaternion_get_time_usec(msg) * 1000LL;
            return true;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            *out = (int64_t)mavlink_msg_gps_raw_int_get_time_usec(msg) * 1000LL;
            return true;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:
            *out = (int64_t)mavlink_msg_local_position_ned_cov_get_time_usec(msg) * 1000LL;
            return true;
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
            *out = (int64_t)mavlink_msg_estimator_status_get_time_usec(msg) * 1000LL;
            return true;
        case MAVLINK_MSG_ID_ALTITUDE:
            *out = (int64_t)mavlink_msg_altitude_get_time_usec(msg) * 1000LL;
            return true;
        case MAVLINK_MSG_ID_WIND_COV:
            *out = (int64_t)mavlink_msg_wind_cov_get_time_usec(msg) * 1000LL;
            return true;
        case MAVLINK_MSG_ID_VIBRATION:
            *out = (int64_t)mavlink_msg_vibration_get_time_usec(msg) * 1000LL;
            return true;
        default:
            return false;
    }
}

// Sequence-gap loss. The counter is per link, so a gap is frames the sender
// emitted and we never saw.
static void track_sequence(map_vehicle_t *v, uint8_t seq) {
    if (v->have_seq) {
        const uint8_t expected = (uint8_t)(v->last_seq + 1);
        if (seq != expected) {
            const uint8_t gap = (uint8_t)(seq - expected);
            v->quality.seq_gaps += gap;
        }
    }
    v->last_seq = seq;
    v->have_seq = true;
    v->quality.seq_received++;
    const uint32_t total = v->quality.seq_received + v->quality.seq_gaps;
    v->quality.loss_pct = total ? (100.0f * (float)v->quality.seq_gaps / (float)total) : 0.0f;
}

static void handle_distance_sensor(map_session_t *ms, int slot, int64_t t_ns,
                                   const mavlink_message_t *msg) {
    mavlink_distance_sensor_t d;
    mavlink_msg_distance_sensor_decode(msg, &d);

    ray_obs_t o;
    memset(&o, 0, sizeof(o));
    o.distance_m     = (float)d.current_distance * 0.01f;
    o.min_distance_m = (float)d.min_distance * 0.01f;
    o.max_distance_m = (float)d.max_distance * 0.01f;
    o.orientation    = d.orientation;
    o.horizontal_fov_rad = d.horizontal_fov;
    o.vertical_fov_rad   = d.vertical_fov;
    o.covariance_cm2 = d.covariance;
    o.signal_quality = d.signal_quality;

    // MAV_SENSOR_ROTATION_CUSTOM is 100; more usefully, a populated quaternion
    // wins outright whatever `orientation` claims.
    const bool q_populated = !(d.quaternion[0] == 0.0f && d.quaternion[1] == 0.0f &&
                               d.quaternion[2] == 0.0f && d.quaternion[3] == 0.0f);
    if (q_populated) {
        o.have_quaternion = true;
        memcpy(o.sensor_q, d.quaternion, sizeof(o.sensor_q));
    }

    map_session_feed_distance(ms, slot, t_ns, &o);
}

static void handle_obstacle_distance(map_session_t *ms, int slot, int64_t t_ns,
                                     const mavlink_message_t *msg) {
    mavlink_obstacle_distance_t d;
    mavlink_msg_obstacle_distance_decode(msg, &d);

    obstacle_obs_t o;
    memset(&o, 0, sizeof(o));
    memcpy(o.distances_cm, d.distances, sizeof(o.distances_cm));
    o.sector_count = OBSTACLE_DISTANCE_SECTORS;
    o.frame = d.frame;
    // increment_f is the float form and takes precedence when populated; the
    // integer `increment` is degrees. An off-by-one here rotates the whole map.
    o.increment_deg = (d.increment_f > 0.0f) ? d.increment_f : (float)d.increment;
    o.angle_offset_deg = d.angle_offset;
    o.min_distance_cm = d.min_distance;
    o.max_distance_cm = d.max_distance;

    map_session_feed_obstacle(ms, slot, t_ns, &o);
}

static void handle_quality(map_session_t *ms, int slot, int64_t t_ns,
                           const mavlink_message_t *msg) {
    vehicle_quality_t *q = &ms->veh[slot].quality;

    switch (msg->msgid) {
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t g;
            mavlink_msg_gps_raw_int_decode(msg, &g);
            q->gps_valid = true;
            q->fix_type = g.fix_type;
            q->satellites_visible = g.satellites_visible;
            // eph/epv are HDOP/VDOP x 100 and dimensionless. h_acc/v_acc are
            // millimetres. Only the latter can draw a ring on the ground.
            q->hdop = (g.eph == UINT16_MAX) ? NAN : (float)g.eph * 0.01f;
            q->vdop = (g.epv == UINT16_MAX) ? NAN : (float)g.epv * 0.01f;
            if (g.h_acc > 0) {
                q->h_acc_m = (float)g.h_acc * 0.001f;
                q->v_acc_m = (float)g.v_acc * 0.001f;
                q->vel_acc_ms = (float)g.vel_acc * 0.001f;
                q->h_acc_valid = true;
            }
            break;
        }
        case MAVLINK_MSG_ID_GPS_STATUS: {
            mavlink_gps_status_t g;
            mavlink_msg_gps_status_decode(msg, &g);
            q->sats_valid = true;
            q->sat_count = g.satellites_visible > QUALITY_MAX_SATS
                         ? QUALITY_MAX_SATS : g.satellites_visible;
            for (int i = 0; i < q->sat_count; i++) {
                q->sats[i].prn = g.satellite_prn[i];
                q->sats[i].used = g.satellite_used[i];
                q->sats[i].elevation_deg = g.satellite_elevation[i];
                q->sats[i].azimuth_deg = g.satellite_azimuth[i];
                q->sats[i].snr = g.satellite_snr[i];
            }
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV: {
            mavlink_local_position_ned_cov_t c;
            mavlink_msg_local_position_ned_cov_decode(msg, &c);
            // Upper-triangular row-major over the 9-state vector: row 0 holds
            // indices 0..8, row 1 holds 9..16, row 2 holds 17..23.
            q->pos_cov[0] = c.covariance[0];   // xx
            q->pos_cov[1] = c.covariance[1];   // xy
            q->pos_cov[2] = c.covariance[2];   // xz
            q->pos_cov[3] = c.covariance[9];   // yy
            q->pos_cov[4] = c.covariance[10];  // yz
            q->pos_cov[5] = c.covariance[17];  // zz
            q->cov_valid = isfinite(q->pos_cov[0]) && isfinite(q->pos_cov[3]);
            break;
        }
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
            mavlink_estimator_status_t e;
            mavlink_msg_estimator_status_decode(msg, &e);
            q->est_px4_valid = true;
            q->vel_ratio = e.vel_ratio;
            q->pos_horiz_ratio = e.pos_horiz_ratio;
            q->pos_vert_ratio = e.pos_vert_ratio;
            q->mag_ratio = e.mag_ratio;
            q->hagl_ratio = e.hagl_ratio;
            q->tas_ratio = e.tas_ratio;
            q->est_flags = e.flags;
            break;
        }
        case HAWKEYE_MSG_ID_EKF_STATUS_REPORT: {
            // ArduPilot's estimator report lives in the ardupilotmega dialect,
            // and Hawkeye builds against common only. Rather than pull in a
            // whole dialect for one message, unpack it from the wire layout:
            // five floats in declaration order, then the uint16 flags.
            const char *p = _MAV_PAYLOAD(msg);
            if (msg->len < 22) break;
            q->est_ardu_valid = true;
            q->ekf_velocity_variance    = _MAV_RETURN_float(msg, 0);
            q->ekf_pos_horiz_variance   = _MAV_RETURN_float(msg, 4);
            q->ekf_pos_vert_variance    = _MAV_RETURN_float(msg, 8);
            q->ekf_compass_variance     = _MAV_RETURN_float(msg, 12);
            q->ekf_terrain_alt_variance = _MAV_RETURN_float(msg, 16);
            q->ekf_flags                = _MAV_RETURN_uint16_t(msg, 20);
            (void)p;
            break;
        }
        case MAVLINK_MSG_ID_WIND_COV: {
            mavlink_wind_cov_t w;
            mavlink_msg_wind_cov_decode(msg, &w);
            q->wind_valid = true;
            q->wind_ned[0] = w.wind_x;
            q->wind_ned[1] = w.wind_y;
            q->wind_ned[2] = w.wind_z;
            q->wind_var_horiz = w.var_horiz;
            q->wind_var_vert = w.var_vert;
            break;
        }
        case MAVLINK_MSG_ID_VIBRATION: {
            mavlink_vibration_t v;
            mavlink_msg_vibration_decode(msg, &v);
            q->vibe_valid = true;
            q->vibration[0] = v.vibration_x;
            q->vibration[1] = v.vibration_y;
            q->vibration[2] = v.vibration_z;
            q->clipping[0] = v.clipping_0;
            q->clipping[1] = v.clipping_1;
            q->clipping[2] = v.clipping_2;
            break;
        }
        case MAVLINK_MSG_ID_TERRAIN_REPORT: {
            mavlink_terrain_report_t t;
            mavlink_msg_terrain_report_decode(msg, &t);
            q->terrain_valid = true;
            q->terrain_height = t.terrain_height;
            q->current_height = t.current_height;
            q->terrain_pending = t.pending;
            q->terrain_loaded = t.loaded;
            break;
        }
        case MAVLINK_MSG_ID_ALTITUDE: {
            mavlink_altitude_t a;
            mavlink_msg_altitude_decode(msg, &a);
            if (isfinite(a.bottom_clearance)) {
                q->bottom_clearance = a.bottom_clearance;
                q->clearance_valid = true;
            }
            break;
        }
        case MAVLINK_MSG_ID_COLLISION: {
            mavlink_collision_t c;
            mavlink_msg_collision_decode(msg, &c);
            q->collision_reported = true;
            q->collision_id = c.id;
            q->collision_time_to_min_delta = c.time_to_minimum_delta;
            q->collision_altitude_delta = c.altitude_minimum_delta;
            q->collision_horizontal_delta = c.horizontal_minimum_delta;
            q->collision_threat_level = c.threat_level;
            map_session_feed_event(ms, slot, t_ns, TL_EVENT_SYSTEM_EVENT, 2,
                                   c.id, "COLLISION");
            break;
        }
        default: break;
    }
}

int mavlink_map_decode(map_session_t *ms, int slot,
                       const struct __mavlink_message *msg_in,
                       int64_t arrival_unix_ns) {
    if (!ms || !msg_in) return -1;
    const mavlink_message_t *msg = (const mavlink_message_t *)msg_in;

    if (slot < 0) slot = map_session_slot_for_sysid(ms, msg->sysid);
    if (slot < 0 || slot >= MS_MAX_VEHICLES) return -1;
    if (!ms->veh[slot].present) map_session_bind_slot(ms, slot, msg->sysid);

    map_vehicle_t *v = &ms->veh[slot];
    v->compid = msg->compid;
    track_sequence(v, msg->seq);

    int64_t src_ns;
    if (frame_source_ns(msg, &src_ns)) {
        v->src_now_ns = src_ns;
        v->have_src_time = true;
    } else {
        src_ns = v->have_src_time ? v->src_now_ns : arrival_unix_ns;
    }

    // Arrival alignment is the fallback tier: it updates freely until something
    // better (TIMESYNC, GPS) takes the source over and locks it out.
    if (arrival_unix_ns != 0)
        timebase_observe_arrival(&ms->clock, &v->tb, src_ns, arrival_unix_ns);

    const time_provenance_t prov_before = v->tb.provenance;
    const int64_t t_ns = timebase_to_session(&v->tb, src_ns);
    g_last_session_ns = t_ns;
    timebase_session_note(&ms->clock, t_ns);

    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(msg, &hb);
            v->mav_type = hb.type;
            v->autopilot = hb.autopilot;
            if (!v->have_mode || v->custom_mode != hb.custom_mode ||
                v->base_mode != hb.base_mode) {
                const bool first = !v->have_mode;
                v->custom_mode = hb.custom_mode;
                v->base_mode = hb.base_mode;
                v->have_mode = true;
                if (!first)
                    map_session_feed_event(ms, slot, t_ns, TL_EVENT_MODE_CHANGE, 6,
                                           hb.custom_mode, "mode change");
            }
            break;
        }

        case MAVLINK_MSG_ID_SYSTEM_TIME: {
            mavlink_system_time_t st;
            mavlink_msg_system_time_decode(msg, &st);
            timebase_observe_system_time(&ms->clock, &v->tb,
                                         st.time_boot_ms, st.time_unix_usec);
            break;
        }

        case MAVLINK_MSG_ID_TIMESYNC: {
            mavlink_timesync_t ts;
            mavlink_msg_timesync_decode(msg, &ts);
            if (ts.tc1 != 0 && arrival_unix_ns != 0) {
                timebase_observe_timesync(&ms->clock, &v->tb, ts.ts1, ts.tc1, arrival_unix_ns);
                v->quality.rtt_ms = (float)v->tb.rtt_ns * 1e-6f;
                v->quality.rtt_ms_min = (v->tb.rtt_ns_min == INT64_MAX)
                    ? v->quality.rtt_ms : (float)v->tb.rtt_ns_min * 1e-6f;
            }
            break;
        }

        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: {
            mavlink_gps_global_origin_t o;
            mavlink_msg_gps_global_origin_decode(msg, &o);
            map_session_feed_origin(ms, slot, FLEET_ORIGIN_SRC_GPS_ORIGIN,
                                    o.latitude * 1e-7, o.longitude * 1e-7,
                                    o.altitude * 1e-3);
            break;
        }

        case MAVLINK_MSG_ID_HOME_POSITION: {
            mavlink_home_position_t h;
            mavlink_msg_home_position_decode(msg, &h);
            map_session_feed_origin(ms, slot, FLEET_ORIGIN_SRC_HOME,
                                    h.latitude * 1e-7, h.longitude * 1e-7,
                                    h.altitude * 1e-3);
            break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t p;
            mavlink_msg_global_position_int_decode(msg, &p);
            const float vel[3] = { p.vx * 0.01f, p.vy * 0.01f, p.vz * 0.01f };
            map_session_feed_global(ms, slot, t_ns, p.lat * 1e-7, p.lon * 1e-7,
                                    p.alt * 1e-3, vel);
            break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t p;
            mavlink_msg_local_position_ned_decode(msg, &p);
            const double ned[3] = { p.x, p.y, p.z };
            const float vel[3] = { p.vx, p.vy, p.vz };
            map_session_feed_local_ned(ms, slot, t_ns, ned, vel);
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
            mavlink_attitude_quaternion_t a;
            mavlink_msg_attitude_quaternion_decode(msg, &a);
            const float q[4] = { a.q1, a.q2, a.q3, a.q4 };
            map_session_feed_attitude(ms, slot, t_ns, q);
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t a;
            mavlink_msg_attitude_decode(msg, &a);
            float q[4];
            rt_quat_from_euler(a.roll, a.pitch, a.yaw, q);
            map_session_feed_attitude(ms, slot, t_ns, q);
            break;
        }

        case MAVLINK_MSG_ID_HIL_STATE_QUATERNION: {
            mavlink_hil_state_quaternion_t h;
            mavlink_msg_hil_state_quaternion_decode(msg, &h);
            map_session_feed_attitude(ms, slot, t_ns, h.attitude_quaternion);
            const float vel[3] = { h.vx * 0.01f, h.vy * 0.01f, h.vz * 0.01f };
            map_session_feed_global(ms, slot, t_ns, h.lat * 1e-7, h.lon * 1e-7,
                                    h.alt * 1e-3, vel);
            break;
        }

        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
            handle_distance_sensor(ms, slot, t_ns, msg);
            break;

        case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
            handle_obstacle_distance(ms, slot, t_ns, msg);
            break;

        case MAVLINK_MSG_ID_STATUSTEXT: {
            mavlink_statustext_t s;
            mavlink_msg_statustext_decode(msg, &s);
            char text[51];
            memcpy(text, s.text, 50);
            text[50] = '\0';
            map_session_feed_event(ms, slot, t_ns, TL_EVENT_STATUSTEXT,
                                   s.severity, s.id, text);
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t a;
            mavlink_msg_command_ack_decode(msg, &a);
            map_session_feed_event(ms, slot, t_ns, TL_EVENT_COMMAND_ACK,
                                   a.result == MAV_RESULT_ACCEPTED ? 6 : 4,
                                   a.command, NULL);
            break;
        }

        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
            mavlink_mission_item_reached_t m;
            mavlink_msg_mission_item_reached_decode(msg, &m);
            map_session_feed_event(ms, slot, t_ns, TL_EVENT_MISSION_ITEM, 6, m.seq, NULL);
            break;
        }

        case MAVLINK_MSG_ID_EVENT: {
            mavlink_event_t e;
            mavlink_msg_event_decode(msg, &e);
            map_session_feed_event(ms, slot, t_ns, TL_EVENT_SYSTEM_EVENT,
                                   (uint8_t)(e.log_levels & 0x0F), e.id, NULL);
            break;
        }

        case MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE: {
            mavlink_current_event_sequence_t e;
            mavlink_msg_current_event_sequence_decode(msg, &e);
            map_session_feed_event(ms, slot, t_ns, TL_EVENT_SYSTEM_EVENT, 6,
                                   e.sequence, NULL);
            break;
        }

        default:
            handle_quality(ms, slot, t_ns, msg);
            break;
    }

    // GPS_RAW_INT does double duty: quality numbers and a time reference.
    if (msg->msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        mavlink_gps_raw_int_t g;
        mavlink_msg_gps_raw_int_decode(msg, &g);
        handle_quality(ms, slot, t_ns, msg);
        if (g.fix_type >= GPS_FIX_TYPE_3D_FIX)
            timebase_observe_gps_raw(&ms->clock, &v->tb,
                                     (uint64_t)(src_ns / 1000LL), g.time_usec);
    }

    if (v->tb.provenance != prov_before) {
        char note[TL_EVENT_TEXT];
        snprintf(note, sizeof(note), "time base: %s",
                 timebase_provenance_name(v->tb.provenance));
        map_session_feed_event(ms, slot, t_ns, TL_EVENT_TIME_REALIGN, 6,
                               (uint32_t)v->tb.provenance, note);
    }
    return slot;
}
