/*
 * communicate.c
 *
 *  Created on: Sep 5, 2024
 *      Author: SAOLATEK
 */

#include "communicate.h"


static uint8_t rx_buffer[MAVLINK_MAX_PACKET_LEN]; 
static mavlink_data_t mav_data;

void MAVLink_UART_Init(mavlink_system_t * mav_system, uint8_t system_id) {
    mav_system->sysid = system_id;  
    mav_system->compid = MAV_COMP_ID_AUTOPILOT1; 
    HAL_UART_Receive_IT(MAVLINK_UART, rx_buffer, sizeof(rx_buffer));
}

void MAVLink_UART_Receive_IT(void) {
    HAL_UART_Receive_IT(MAVLINK_UART, rx_buffer, sizeof(rx_buffer));
}


void MAVLink_UART_Transmit(uint8_t *data, uint16_t length) {
    HAL_UART_Transmit(MAVLINK_UART, data, length, HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == MAVLINK_UART.Instance) {
        MAVLink_Process_Data(rx_buffer, sizeof(rx_buffer));
        HAL_UART_Receive_IT(huart, rx_buffer, sizeof(rx_buffer)); 
    }
}

void MAVLink_Send_Message(mavlink_message_t *msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(buffer, msg);
    MAVLink_UART_Transmit(buffer, length);
}

void MAVLink_Process_Data(uint8_t *data, uint16_t length) {
    mavlink_message_t msg;
    mavlink_status_t status;

    for (uint16_t i = 0; i < length; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            MAVLink_Handle_Message(&msg);
        }
    }
}


void MAVLink_Handle_Message(mavlink_message_t *msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: // heartbeat
            Handle_Heartbeat(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG: // command
            Handle_Command_Long(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_COMMAND_ACK: // command ack
            Handle_Command_Ack(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_STATUSTEXT: // status text
            Handle_Status_Text(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_GPS_RAW_INT: // GPS raw
            Handle_GPS_Raw(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_ATTITUDE: // attitude
            Handle_Attitude(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_BATTERY_STATUS: // battery status
            Handle_Battery_Status(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_SYS_STATUS: // system status
            Handle_Sys_Status(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_SET_MODE: // set mode
            Handle_Set_Mode(msg, &mav_data);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM:
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        case MAVLINK_MSG_ID_MISSION_COUNT:
        case MAVLINK_MSG_ID_MISSION_ACK:
            Handle_Mission_Items(msg, &mav_data);
            break;
        default:
            break;
    }
}

void send_heartbeat(mavlink_system_t * mav_system) {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;
    
    mavlink_msg_heartbeat_pack(mav_system->sysid, mav_system->compid, &msg,
    MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_ARMED, 
    0, MAV_STATE_ACTIVE);

    len = mavlink_msg_to_send_buffer(buffer, &msg);

    MAVLink_UART_Transmit(buffer, len);
}