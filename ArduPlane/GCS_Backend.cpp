#include "Plane.h" // for global plane object

#if HIL_SUPPORT
extern mavlink_hil_state_t last_hil_state; // see GCS_Mavlink.cpp
#endif

bool GCS_Backend_Plane::should_try_send_message(enum ap_message id)
{
    if (telemetry_delayed()) {
        return false;
    }

    // if we don't have a minimum mount of time remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!plane.in_mavlink_delay && plane.scheduler.time_available_usec() < 1200) {
        out_of_time = true;
        return false;
    }

    return true;
}

bool GCS_Backend_Plane::send_AHRS()
{
    send_ahrs(plane.ahrs);
    return true;
}
bool GCS_Backend_Plane::send_AHRS2()
{
    send_ahrs2(plane.ahrs);
    return true;
}
bool GCS_Backend_Plane::send_ATTITUDE()
{
    plane.send_attitude(chan);
    return true;
}
bool GCS_Backend_Plane::send_BATTERY2()
{
    send_battery2(plane.battery);
    return true;
}
bool GCS_Backend_Plane::send_CAMERA_FEEDBACK()
{
#if CAMERA == ENABLED
    plane.camera.send_feedback(chan, plane.gps, plane.ahrs, plane.current_loc);
#endif
    return true;
}
bool GCS_Backend_Plane::send_EKF_STATUS_REPORT()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    plane.ahrs.send_ekf_status_report(chan);
#endif
    return true;
}
bool GCS_Backend_Plane::send_FENCE_STATUS()
{
#if GEOFENCE_ENABLED == ENABLED
    plane.send_fence_status(chan);
#endif
    return true;
}
bool GCS_Backend_Plane::send_GIMBAL_REPORT()
{
#if MOUNT == ENABLED
    plane.camera_mount.send_gimbal_report(chan);
#endif
    return true;
}
bool GCS_Backend_Plane::send_GLOBAL_POSITION_INT()
{
    plane.send_location(chan);
    return true;
}
bool GCS_Backend_Plane::send_GPS_RAW()
{
    send_gps_raw(plane.gps);
    return true;
}
bool GCS_Backend_Plane::send_HEARTBEAT()
{
    plane.send_heartbeat(chan);
    return true;
}
bool GCS_Backend_Plane::send_HWSTATUS()
{
    plane.send_hwstatus(chan);
    return true;
}
bool GCS_Backend_Plane::send_LOCAL_POSITION_NED()
{
    send_local_position(plane.ahrs);
    return true;
}
bool GCS_Backend_Plane::send_MAG_CAL_PROGRESS()
{
    plane.compass.send_mag_cal_progress(chan);
    return true;
}
bool GCS_Backend_Plane::send_MAG_CAL_REPORT()
{
    plane.compass.send_mag_cal_report(chan);
    return true;
}
bool GCS_Backend_Plane::send_MOUNT_STATUS()
{
#if MOUNT == ENABLED
    plane.camera_mount.status_msg(chan);
#endif
    return true;
}
bool GCS_Backend_Plane::send_NAV_CONTROLLER_OUTPUT()
{
    if (plane.control_mode != MANUAL) {
        plane.send_nav_controller_output(chan);
    }
    return true;
}
bool GCS_Backend_Plane::send_OPTICAL_FLOW()
{
#if OPTFLOW == ENABLED
    send_opticalflow(plane.ahrs, plane.optflow);
#endif
    return true;
}
bool GCS_Backend_Plane::send_PID_TUNING()
{
    plane.send_pid_tuning(chan);
    return true;
}
bool GCS_Backend_Plane::send_RANGEFINDER()
{
    plane.send_rangefinder(chan);
    return true;
}
bool GCS_Backend_Plane::send_RAW_IMU()
{
    send_raw_imu(plane.ins, plane.compass);
    return true;
}
bool GCS_Backend_Plane::send_RC_CHANNELS_RAW()
{
    send_radio_in(plane.receiver_rssi);
    return true;
}
bool GCS_Backend_Plane::send_RC_CHANNELS_SCALED()
{
#if HIL_SUPPORT
    if (plane.g.hil_mode == 1) {
        plane.send_servo_out(chan);
    }
#endif
    return true;
}
bool GCS_Backend_Plane::send_RPM()
{
    plane.send_rpm(chan);
    return true;
}
bool GCS_Backend_Plane::send_SCALED_PRESSURE()
{
    send_scaled_pressure(plane.barometer);
    return true;
}
bool GCS_Backend_Plane::send_SENSOR_OFFSETS()
{
    send_sensor_offsets(plane.ins, plane.compass, plane.barometer);
    return true;
}
bool GCS_Backend_Plane::send_SERVO_OUTPUT_RAW()
{
    plane.send_radio_out(chan);
    return true;
}
bool GCS_Backend_Plane::send_SIMSTATE()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    plane.send_simstate(chan);
#endif
    return true;
}
bool GCS_Backend_Plane::send_STATUSTEXT()
{
    // this looks wrong; if a Backend is told to send its statustext
    // then it should not be talking to the frontend:
    plane.gcs_frontend.send_statustext(chan);
    return true;
}
bool GCS_Backend_Plane::send_SYS_STATUS()
{
    plane.send_extended_status1(chan);
    return true;
}
bool GCS_Backend_Plane::send_SYSTEM_TIME()
{
    send_system_time(plane.gps);
    return true;
}
bool GCS_Backend_Plane::send_TERRAIN_REQUEST()
{
#if AP_TERRAIN_AVAILABLE
    plane.terrain.send_request(chan);
#endif
    return true;
}
bool GCS_Backend_Plane::send_VFR_HUD()
{
    plane.send_vfr_hud(chan);
    return true;
}
bool GCS_Backend_Plane::send_MISSION_CURRENT()
{
    plane.send_current_waypoint(chan);
    return true;
}
bool GCS_Backend_Plane::send_MISSION_ITEM_REACHED()
{
    mavlink_msg_mission_item_reached_send(chan, mission_item_reached_index);
    return true;
}
bool GCS_Backend_Plane::send_VIBRATION()
{
    send_vibration(plane.ins);
    return true;
}
bool GCS_Backend_Plane::send_WIND()
{
    plane.send_wind(chan);
    return true;
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_Backend_Plane::try_send_message(enum ap_message id)
{
    if (!should_try_send_message(id)) {
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        send_HEARTBEAT();
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_SYS_STATUS();
        CHECK_PAYLOAD_SIZE2(POWER_STATUS);
        send_power_status();
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_ATTITUDE();
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_GLOBAL_POSITION_INT();
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_LOCAL_POSITION_NED();
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_NAV_CONTROLLER_OUTPUT();
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_GPS_RAW();
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_SYSTEM_TIME();
        break;

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_RC_CHANNELS_SCALED();
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_RC_CHANNELS_RAW();
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_SERVO_OUTPUT_RAW();
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_VFR_HUD();
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_RAW_IMU();
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_SCALED_PRESSURE();
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_SENSOR_OFFSETS();
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        send_MISSION_CURRENT();
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        queued_waypoint_send();
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_STATUSTEXT();
        break;

    case MSG_FENCE_STATUS:
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        send_FENCE_STATUS();
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_AHRS();
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_SIMSTATE();
        CHECK_PAYLOAD_SIZE2(AHRS2);
        send_AHRS2();
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_HWSTATUS();
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_RANGEFINDER();
        break;

    case MSG_TERRAIN:
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        send_TERRAIN_REQUEST();
        break;

    case MSG_CAMERA_FEEDBACK:
        CHECK_PAYLOAD_SIZE(CAMERA_FEEDBACK);
        send_CAMERA_FEEDBACK();
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        send_BATTERY2();
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_WIND();
        break;

    case MSG_MOUNT_STATUS:
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        send_MOUNT_STATUS();
        break;

    case MSG_OPTICAL_FLOW:
        CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
        send_OPTICAL_FLOW();
        break;

    case MSG_EKF_STATUS_REPORT:
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        send_EKF_STATUS_REPORT();
        break;

    case MSG_GIMBAL_REPORT:
        CHECK_PAYLOAD_SIZE(GIMBAL_REPORT);
        send_GIMBAL_REPORT();
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning

    case MSG_LIMITS_STATUS:
        // unused
        break;

    case MSG_PID_TUNING:
        CHECK_PAYLOAD_SIZE(PID_TUNING);
        send_PID_TUNING();
        break;

    case MSG_VIBRATION:
        CHECK_PAYLOAD_SIZE(VIBRATION);
        send_VIBRATION();
        break;

    case MSG_RPM:
        CHECK_PAYLOAD_SIZE(RPM);
        send_RPM();
        break;

    case MSG_MISSION_ITEM_REACHED:
        CHECK_PAYLOAD_SIZE(MISSION_ITEM_REACHED);
        send_MISSION_ITEM_REACHED();
        break;

    case MSG_MAG_CAL_PROGRESS:
        CHECK_PAYLOAD_SIZE(MAG_CAL_PROGRESS);
        send_MAG_CAL_PROGRESS();
        break;

    case MSG_MAG_CAL_REPORT:
        CHECK_PAYLOAD_SIZE(MAG_CAL_REPORT);
        send_MAG_CAL_REPORT();
        break;
    }
    return true;
}


/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  1),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  1),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  1),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  1),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  1),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  1),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  1),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  1),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  10),
    AP_GROUPEND
};


// see if we should send a stream now. Called at 50Hz
bool GCS_Backend_Plane::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRate(stream_num).get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) && 
        (waypoint_receiving() || queued_parameter() != NULL)) {
        rate *= 0.25f;
    }

    if (rate <= 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) - 1 + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_Backend_Plane::data_stream_send(void)
{
    out_of_time = false;

    if (!plane.in_mavlink_delay) {
        handle_log_send(plane.DataFlash);
    }

    if (queued_parameter() != NULL) {
        if (streamRate(STREAM_PARAMS).get() <= 0) {
            streamRate(STREAM_PARAMS).set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (out_of_time) return;

    if (plane.in_mavlink_delay) {
#if HIL_SUPPORT
        if (plane.g.hil_mode == 1) {
            // in HIL we need to keep sending servo values to ensure
            // the simulator doesn't pause, otherwise our sensor
            // calibration could stall
            if (stream_trigger(STREAM_RAW_CONTROLLER)) {
                send_message(MSG_SERVO_OUT);
            }
            if (stream_trigger(STREAM_RC_CHANNELS)) {
                send_message(MSG_RADIO_OUT);
            }
        }
#endif
        // don't send any other stream types while in the delay callback
        return;
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_FENCE_STATUS);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
        send_message(MSG_RPM);
        if (plane.control_mode != MANUAL) {
            send_message(MSG_PID_TUNING);
        }
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_WIND);
        send_message(MSG_RANGEFINDER);
        send_message(MSG_SYSTEM_TIME);
#if AP_TERRAIN_AVAILABLE
        send_message(MSG_TERRAIN);
#endif
        send_message(MSG_MAG_CAL_REPORT);
        send_message(MSG_MAG_CAL_PROGRESS);
        send_message(MSG_BATTERY2);
        send_message(MSG_MOUNT_STATUS);
        send_message(MSG_OPTICAL_FLOW);
        send_message(MSG_EKF_STATUS_REPORT);
        send_message(MSG_GIMBAL_REPORT);
        send_message(MSG_VIBRATION);
    }
}


/*
  handle a request to switch to guided mode. This happens via a
  callback from handle_mission_item()
 */
void GCS_Backend_Plane::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (plane.control_mode != GUIDED) {
        // only accept position updates when in GUIDED mode
        return;
    }
    plane.guided_WP_loc = cmd.content.location;
    
    // add home alt if needed
    if (plane.guided_WP_loc.flags.relative_alt) {
        plane.guided_WP_loc.alt += plane.home.alt;
        plane.guided_WP_loc.flags.relative_alt = 0;
    }

    plane.set_guided_WP();
}

/*
  handle a request to change current WP altitude. This happens via a
  callback from handle_mission_item()
 */
void GCS_Backend_Plane::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    plane.next_WP_loc.alt = cmd.content.location.alt;
    if (cmd.content.location.flags.relative_alt) {
        plane.next_WP_loc.alt += plane.home.alt;
    }
    plane.next_WP_loc.flags.relative_alt = false;
    plane.next_WP_loc.flags.terrain_alt = cmd.content.location.flags.terrain_alt;
    plane.reset_offset_altitude();
}

void GCS_Backend_Plane::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        handle_request_data_stream(msg, true);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        // do command
        send_text(MAV_SEVERITY_INFO,"Command received: ");

        switch(packet.command) {

        case MAV_CMD_START_RX_PAIR:
            // initiate bind procedure
            if (!hal.rcin->rc_bind(packet.param1)) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            plane.set_mode(LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            plane.set_mode(RTL);
            result = MAV_RESULT_ACCEPTED;
            break;

#if MOUNT == ENABLED
        // Sets the region of interest (ROI) for the camera
        case MAV_CMD_DO_SET_ROI:
            // sanity check location
            if (fabsf(packet.param5) > 90.0f || fabsf(packet.param6) > 180.0f) {
                break;
            }
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
                // switch off the camera tracking if enabled
                if (plane.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                    plane.camera_mount.set_mode_to_default();
                }
            } else {
                // send the command to the camera mount
                plane.camera_mount.set_roi_target(roi_loc);
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif

#if CAMERA == ENABLED
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            plane.camera.configure(packet.param1,
                                   packet.param2,
                                   packet.param3,
                                   packet.param4,
                                   packet.param5,
                                   packet.param6,
                                   packet.param7);

            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            if (plane.camera.control(packet.param1,
                                     packet.param2,
                                     packet.param3,
                                     packet.param4,
                                     packet.param5,
                                     packet.param6)) {
                plane.log_picture();
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif // CAMERA == ENABLED

        case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
            plane.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
#endif
            break;

        case MAV_CMD_MISSION_START:
            plane.set_mode(AUTO);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if(hal.util->get_soft_armed()) {
                result = MAV_RESULT_FAILED;
                break;
            }
            plane.in_calibration = true;
            if (is_equal(packet.param1,1.0f)) {
                plane.ins.init_gyro();
                if (plane.ins.gyro_calibrated_ok_all()) {
                    plane.ahrs.reset_gyro_drift();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param3,1.0f)) {
                plane.init_barometer();
                if (plane.airspeed.enabled()) {
                    plane.zero_airspeed(false);
                }
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param4,1.0f)) {
                plane.trim_radio();
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param5,1.0f)) {
                result = MAV_RESULT_ACCEPTED;
                // start with gyro calibration
                plane.ins.init_gyro();
                // reset ahrs gyro bias
                if (plane.ins.gyro_calibrated_ok_all()) {
                    plane.ahrs.reset_gyro_drift();
                } else {
                    result = MAV_RESULT_FAILED;
                }
                plane.ins.acal_init();
                plane.ins.get_acal()->start(this);

            } else if (is_equal(packet.param5,2.0f)) {
                // start with gyro calibration
                plane.ins.init_gyro();
                // accel trim
                float trim_roll, trim_pitch;
                if(plane.ins.calibrate_trim(trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    plane.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            }
            else {
                    send_text(MAV_SEVERITY_WARNING, "Unsupported preflight calibration");
            }
            plane.in_calibration = false;
            break;

        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            if (is_equal(packet.param1,2.0f)) {
                // save first compass's offsets
                plane.compass.set_and_save_offsets(0, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            if (is_equal(packet.param1,5.0f)) {
                // save secondary compass's offsets
                plane.compass.set_and_save_offsets(1, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1,1.0f)) {
                // run pre_arm_checks and arm_checks and display failures
                if (plane.arm_motors(AP_Arming::MAVLINK)) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_zero(packet.param1))  {
                if (plane.disarm_motors()) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_GET_HOME_POSITION:
            if (plane.home_is_set != HOME_UNSET) {
                send_home(plane.ahrs.get_home());
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_MODE:
            switch ((uint16_t)packet.param1) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                plane.set_mode(MANUAL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                plane.set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                plane.set_mode(FLY_BY_WIRE_A);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (plane.ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (plane.ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (plane.ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (plane.ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(is_equal(packet.param1,3.0f));
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_LAND_START:
            result = MAV_RESULT_FAILED;
            
            // attempt to switch to next DO_LAND_START command in the mission
            if (plane.jump_to_landing_sequence()) {
                result = MAV_RESULT_ACCEPTED;
            } 
            break;

        case MAV_CMD_DO_GO_AROUND:
            result = MAV_RESULT_FAILED;

            //Not allowing go around at FLIGHT_LAND_FINAL stage on purpose --
            //if plane is close to the ground a go around coudld be dangerous.
            if (plane.flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
                plane.flight_stage == AP_SpdHgtControl::FLIGHT_LAND_PREFLARE ||
                plane.flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
                // Initiate an aborted landing. This will trigger a pitch-up and
                // climb-out to a safe altitude holding heading then one of the
                // following actions will occur, check for in this order:
                // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
                //      increment mission index to execute it
                // - else if DO_LAND_START is available, jump to it
                // - else decrement the mission index to repeat the landing approach

                if (!is_zero(packet.param1)) {
                    plane.auto_state.takeoff_altitude_rel_cm = packet.param1 * 100;
                }
                plane.auto_state.commanded_go_around = true;
               
                result = MAV_RESULT_ACCEPTED;
                _frontend->send_text(MAV_SEVERITY_INFO,"Go around command accepted");
            } else {
                _frontend->send_text(MAV_SEVERITY_NOTICE,"Rejected go around command");
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
            result = MAV_RESULT_ACCEPTED;
            
            if (!plane.geofence_present()) {
                result = MAV_RESULT_FAILED;
            } switch((uint16_t)packet.param1) {
                case 0:
                    if (! plane.geofence_set_enabled(false, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED;
                    }
                    break;
                case 1:
                    if (! plane.geofence_set_enabled(true, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED; 
                    }
                    break;
                case 2: //disable fence floor only 
                    if (! plane.geofence_set_floor_enabled(false)) {
                        result = MAV_RESULT_FAILED;
                    } else {
                        _frontend->send_text(MAV_SEVERITY_NOTICE,"Fence floor disabled");
                    }
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
            break;

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
            if (is_equal(packet.param1,1.0f)) {
                send_autopilot_version(FIRMWARE_VERSION);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_HOME:
            // param1 : use current (1=use current location, 0=use specified location)
            // param5 : latitude
            // param6 : longitude
            // param7 : altitude (absolute)
            result = MAV_RESULT_FAILED; // assume failure
            if (is_equal(packet.param1,1.0f)) {
                plane.init_home();
            } else {
                if (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7)) {
                    // don't allow the 0,0 position
                    break;
                }
                // sanity check location
                if (fabsf(packet.param5) > 90.0f || fabsf(packet.param6) > 180.0f) {
                    break;
                }
                Location new_home_loc {};
                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
                plane.ahrs.set_home(new_home_loc);
                plane.home_is_set = HOME_SET_NOT_LOCKED;
                plane.Log_Write_Home_And_Origin();
                _frontend->send_home(new_home_loc); // all GCS get new home
                result = MAV_RESULT_ACCEPTED;
                _frontend->send_text_fmt(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %um",
                                        (double)(new_home_loc.lat*1.0e-7f),
                                        (double)(new_home_loc.lng*1.0e-7f),
                                        (uint32_t)(new_home_loc.alt*0.01f));
            }
            break;
        }

        case MAV_CMD_DO_AUTOTUNE_ENABLE:
            // param1 : enable/disable
            plane.autotune_enable(!is_zero(packet.param1));
            break;

        case MAV_CMD_DO_START_MAG_CAL:
        case MAV_CMD_DO_ACCEPT_MAG_CAL:
        case MAV_CMD_DO_CANCEL_MAG_CAL:
            result = plane.compass.handle_mag_cal_command(packet);
            break;

#if PARACHUTE == ENABLED
        case MAV_CMD_DO_PARACHUTE:
            // configure or release parachute
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
                case PARACHUTE_DISABLE:
                    plane.parachute.enabled(false);
                    break;
                case PARACHUTE_ENABLE:
                    plane.parachute.enabled(true);
                    break;
                case PARACHUTE_RELEASE:
                    // treat as a manual release which performs some additional check of altitude
                    if (plane.parachute.released()) {
                        _frontend->send_text_fmt(MAV_SEVERITY_NOTICE, "Parachute already released");
                        result = MAV_RESULT_FAILED;
                    } else if (!plane.parachute.enabled()) {
                        _frontend->send_text_fmt(MAV_SEVERITY_NOTICE, "Parachute not enabled");
                        result = MAV_RESULT_FAILED;
                    } else {
                        if (!plane.parachute_manual_release()) {
                            result = MAV_RESULT_FAILED;
                        }
                    }
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
            break;
#endif

        case MAV_CMD_DO_VTOL_TRANSITION:
            result = plane.quadplane.handle_do_vtol_transition(packet);
            break;
            
        default:
            break;
        }

        mavlink_msg_command_ack_send_buf(
            msg,
            chan,
            packet.command,
            result);

        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:
    {
        handle_set_mode(msg, FUNCTOR_BIND(&plane, &Plane::mavlink_set_mode, bool, uint8_t));
        break;
    }

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        handle_mission_request_list(plane.mission, msg);
        break;
    }


    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        handle_mission_request(plane.mission, msg);
        break;
    }


    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        // nothing to do
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        // mark the firmware version in the tlog
        send_text(MAV_SEVERITY_INFO, FIRMWARE_STRING);

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text(MAV_SEVERITY_INFO, "PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION);
#endif
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        handle_param_request_read(msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        handle_mission_clear_all(plane.mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        // disable cross-track when user asks for WP change, to
        // prevent unexpected flight paths
        plane.auto_state.next_wp_no_crosstrack = true;
        handle_mission_set_current(plane.mission, msg);
        if (plane.control_mode == AUTO && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
            plane.mission.resume();
        }
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        handle_mission_count(plane.mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        handle_mission_write_partial_list(plane.mission, msg);
        break;
    }

    // GCS has sent us a mission item, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        if (handle_mission_item(msg, plane.mission)) {
            plane.DataFlash.Log_Write_EntireMission(plane.mission);
        }
        break;
    }

#if GEOFENCE_ENABLED == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (plane.g.fence_action != FENCE_ACTION_NONE) {
            send_text(MAV_SEVERITY_WARNING,"Fencing must be disabled");
        } else if (packet.count != plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else if (fabsf(packet.lat) > 90.0f || fabsf(packet.lng) > 180.0f) {
            send_text(MAV_SEVERITY_WARNING,"Invalid fence point, lat or lng too large");
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7f;
            point.y = packet.lng*1.0e7f;
            plane.set_fence_point_with_index(point, packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (packet.idx >= plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else {
            Vector2l point = plane.get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send_buf(msg, chan, msg->sysid, msg->compid, packet.idx, plane.g.fence_total,
                                             point.x*1.0e-7f, point.y*1.0e-7f);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    // receive a rally point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_RALLY_POINT: {
        mavlink_rally_point_t packet;
        mavlink_msg_rally_point_decode(msg, &packet);
        
        if (packet.idx >= plane.rally.get_rally_total() || 
            packet.idx >= plane.rally.get_rally_max()) {
            send_text(MAV_SEVERITY_WARNING,"Bad rally point message ID");
            break;
        }

        if (packet.count != plane.rally.get_rally_total()) {
            send_text(MAV_SEVERITY_WARNING,"Bad rally point message count");
            break;
        }

        RallyLocation rally_point;
        rally_point.lat = packet.lat;
        rally_point.lng = packet.lng;
        rally_point.alt = packet.alt;
        rally_point.break_alt = packet.break_alt;
        rally_point.land_dir = packet.land_dir;
        rally_point.flags = packet.flags;
        plane.rally.set_rally_point_with_index(packet.idx, rally_point);
        break;
    }

    //send a rally point to the GCS
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
        mavlink_rally_fetch_point_t packet;
        mavlink_msg_rally_fetch_point_decode(msg, &packet);
        if (packet.idx > plane.rally.get_rally_total()) {
            send_text(MAV_SEVERITY_WARNING, "Bad rally point index");
            break;
        }
        RallyLocation rally_point;
        if (!plane.rally.get_rally_point_with_index(packet.idx, rally_point)) {
            send_text(MAV_SEVERITY_WARNING, "Failed to set rally point");
            break;
        }

        mavlink_msg_rally_point_send_buf(msg,
                                         chan, msg->sysid, msg->compid, packet.idx, 
                                         plane.rally.get_rally_total(), rally_point.lat, rally_point.lng, 
                                         rally_point.alt, rally_point.break_alt, rally_point.land_dir, 
                                         rally_point.flags);
        break;
    }    

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        handle_param_set(msg, &plane.DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_GIMBAL_REPORT:
    {
#if MOUNT == ENABLED
        handle_gimbal_report(plane.camera_mount, msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != plane.g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        if (hal.rcin->set_overrides(v, 8)) {
            plane.failsafe.last_valid_rc_ms = AP_HAL::millis();
        }

        // a RC override message is consiered to be a 'heartbeat' from
        // the ground station for failsafe purposes
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg->sysid != plane.g.sysid_my_gcs) break;
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_HIL_STATE:
    {
#if HIL_SUPPORT
        if (plane.g.hil_mode != 1) {
            break;
        }
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        last_hil_state = packet;

        // set gps hil sensor
        Location loc;
        memset(&loc, 0, sizeof(loc));
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        // setup airspeed pressure based on 3D speed, no wind
        plane.airspeed.setHIL(sq(vel.length()) / 2.0f + 2013);

        plane.gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                         packet.time_usec/1000,
                         loc, vel, 10, 0, true);

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * GRAVITY_MSS*0.001f;
        accels.y = packet.yacc * GRAVITY_MSS*0.001f;
        accels.z = packet.zacc * GRAVITY_MSS*0.001f;

        plane.ins.set_gyro(0, gyros);
        plane.ins.set_accel(0, accels);

        plane.barometer.setHIL(packet.alt*0.001f);
        plane.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
        plane.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);

        // cope with DCM getting badly off due to HIL lag
        if (plane.g.hil_err_limit > 0 &&
            (fabsf(packet.roll - plane.ahrs.roll) > ToRad(plane.g.hil_err_limit) ||
             fabsf(packet.pitch - plane.ahrs.pitch) > ToRad(plane.g.hil_err_limit) ||
             wrap_PI(fabsf(packet.yaw - plane.ahrs.yaw)) > ToRad(plane.g.hil_err_limit))) {
            plane.ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
        }
#endif
        break;
    }

#if CAMERA == ENABLED
    //deprecated. Use MAV_CMD_DO_DIGICAM_CONFIGURE
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        break;
    }

    //deprecated. Use MAV_CMD_DO_DIGICAM_CONTROL
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        plane.camera.control_msg(msg);
        plane.log_picture();
        break;
    }
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    //deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
    {
        plane.camera_mount.configure_msg(msg);
        break;
    }

    //deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        plane.camera_mount.control_msg(msg);
        break;
    }
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        handle_radio_status(msg, plane.DataFlash, plane.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        plane.in_log_download = true;
        /* no break */
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!plane.in_mavlink_delay) {
            handle_log_message(msg, plane.DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        plane.in_log_download = false;
        if (!plane.in_mavlink_delay) {
            handle_log_message(msg, plane.DataFlash);
        }
        break;

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, plane.gps);
        break;

    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg, plane.gps);
        break;

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        send_autopilot_version(FIRMWARE_VERSION);
        break;

    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        plane.DataFlash.remote_log_block_status_msg(chan, msg);
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
        if((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            // don't allow the 0,0 position
            break;
        }
        // sanity check location
        if (labs(packet.latitude) > 90*10e7 || labs(packet.longitude) > 180 * 10e7) {
            break;
        }
        Location new_home_loc {};
        new_home_loc.lat = packet.latitude;
        new_home_loc.lng = packet.longitude;
        new_home_loc.alt = packet.altitude * 100;
        plane.ahrs.set_home(new_home_loc);
        plane.home_is_set = HOME_SET_NOT_LOCKED;
        plane.Log_Write_Home_And_Origin();
        _frontend->send_home(new_home_loc); // all GCS get new home
        _frontend->send_text_fmt(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %um",
                                (double)(new_home_loc.lat*1.0e-7f),
                                (double)(new_home_loc.lng*1.0e-7f),
                                (uint32_t)(new_home_loc.alt*0.01f));
        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
        plane.adsb.update_vehicle(msg);
        break;
    } // end switch
} // end handle mavlink
