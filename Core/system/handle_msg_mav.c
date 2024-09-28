/*
 * handle_msg_mav.c
 *
 *  Created on: Sep 5, 2024
 *      Author: SAOLATEK
 */

#include "handle_msg_mav.h"

static uint16_t last_heartbeat_time = 0;
static bool alive = true;

void Handle_Heartbeat(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_heartbeat_decode(msg, &mav_data->heartbeat);

    uint32_t cur_time = HAL_GetTick();
    // avoid overflow
    uint32_t elapsed_time = (cur_time - last_heartbeat_time + 0xFFFFFFFF) % 0xFFFFFFFF;
    if (elapsed_time > HEARTBEAT_INTERVAL_MS) {
        alive = false;
    }
    else last_heartbeat_time = cur_time;
    // update heartbeat
    printf("Heartbeat received: Type: %d, Autopilot: %d, Base mode: %d, System status: %d\n",
            &mav_data->heartbeat.type,  &mav_data->heartbeat.autopilot, 
            &mav_data->heartbeat.base_mode,  &mav_data->heartbeat.system_status);
}

void Handle_Command_Long(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_command_long_decode(msg, &mav_data->command_long);

    switch (mav_data->command_long.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (mav_data->command_long.param1 == 1) {
                printf("Arming the drone.\n");
            } else {
                printf("Disarming the drone.\n");
            }
            break;
        case MAV_CMD_NAV_LAND:
            printf("Landing command received.\n");
            break;
        default:
            printf("Unknown command: %d\n", mav_data->command_long.command);
            break;
    }
}

void Handle_Command_Ack(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_command_ack_decode(msg,  &mav_data->command_ack);

    printf("Command Acknowledge received for command %d with result %d\n",
           mav_data->command_ack.command, mav_data->command_ack.result);
}

void Handle_Status_Text(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_statustext_decode(msg, &mav_data->statustext);

    printf("Status Text: %s\n", mav_data->statustext.text);
}

void Handle_GPS_Raw(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_gps_raw_int_decode(msg, &mav_data->gps_raw);

    // update infor GPS
    printf("GPS Raw: Lat: %ld, Lon: %ld, Alt: %ld, Satellites: %d\n",
           mav_data->gps_raw.lat, mav_data->gps_raw.lon, mav_data->gps_raw.alt, mav_data->gps_raw.satellites_visible);
}

void Handle_Timesync(mavlink_message_t *msg, mavlink_data_t *mav_data){
    mavlink_msg_timesync_decode(msg, &mav_data->timesync);

    printf("Timesync received: TC1: %ld, TS1: %ld\n",
           mav_data->timesync.tc1, mav_data->timesync.ts1);
}

void Handle_Attitude(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_attitude_decode(msg, &mav_data->attitude);

    // update attitude of drone
    printf("Attitude: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
           mav_data->attitude.roll, mav_data->attitude.pitch, mav_data->attitude.yaw);
}

void Handle_Battery_Status(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_battery_status_decode(msg, &mav_data->battery_status);

    // update battery status
    printf("Battery: Voltage: %dmV, Current: %dmA, Remaining: %d%%\n",
           mav_data->battery_status.voltages[0], mav_data->battery_status.current_battery, 
           mav_data->battery_status.battery_remaining);
}

void Handle_Sys_Status(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_sys_status_decode(msg, &mav_data->sys_status);

    // update system status of drone
    printf("Sys Status: Battery remaining: %d%%, Load: %d%%\n",
           mav_data->sys_status.battery_remaining, mav_data->sys_status.load);
}

void Handle_Set_Mode(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    mavlink_msg_set_mode_decode(msg, &mav_data->set_mode);

    printf("Set Mode received: Base mode: %d, Custom mode: %d\n",
           mav_data->set_mode.base_mode, mav_data->set_mode.custom_mode);
}

// define function Handle_Mission_Items
void Handle_Mission_Items(mavlink_message_t *msg, mavlink_data_t *mav_data) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_MISSION_ITEM:
            // Giả sử bạn muốn lưu trữ waypoint trong mav_data
            // mavlink_msg_mission_item_decode(msg, &mav_data->mission_item);
            printf("Mission Item received.\n");
            // Xử lý nhận một waypoint
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
            // Giả sử bạn muốn lưu trữ yêu cầu waypoint trong mav_data
            // mavlink_msg_mission_request_decode(msg, &mav_data->mission_request);
            printf("Mission Request received.\n");
            // Xử lý yêu cầu gửi waypoint
            break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
            // Giả sử bạn muốn lưu trữ số lượng waypoint trong mav_data
            // mavlink_msg_mission_count_decode(msg, &mav_data->mission_count);
            printf("Mission Count received.\n");
            // Xử lý thông báo số lượng waypoint
            break;
        case MAVLINK_MSG_ID_MISSION_ACK:
            // Giả sử bạn muốn lưu trữ xác nhận nhiệm vụ trong mav_data
            // mavlink_msg_mission_ack_decode(msg, &mav_data->mission_ack);
            printf("Mission Acknowledge received.\n");
            // Xử lý xác nhận hoàn tất nhiệm vụ
            break;
        default:
            break;
    }
}

