/*
 * communicate.h
 *
 *  Created on: Sep 5, 2024
 *      Author: SAOLATEK
 */

#ifndef COMMUNICATE_H_
#define COMMUNICATE_H_

#include "c_library_v2/common/mavlink.h"
#include "handle_msg_mav.h"

// USER define for MAVLink communication
#define MAVLINK_UART (&huart1)
// End USER define for MAVLink communication

// System define for MAVLink communication
#define HEARTBEAT_INTERVAL_MS 1000 // send heartbeat every 1s

typedef struct {
    uint8_t sysid;   
    uint8_t compid;  
} mavlink_system_t;

typedef struct {
    mavlink_heartbeat_t heartbeat;
    mavlink_command_long_t command_long;
    mavlink_attitude_t attitude;
    mavlink_command_ack_t command_ack;
    mavlink_timesync_t timesync;
    mavlink_statustext_t statustext;
    mavlink_gps_raw_int_t gps_raw;
    mavlink_battery_status_t battery_status;
    mavlink_sys_status_t sys_status;
    mavlink_set_mode_t set_mode;
} mavlink_data_t;
// End system define for MAVLink communication

void MAVLink_UART_Init(mavlink_system_t * mav_system, uint8_t system_id);
void MAVLink_UART_Transmit(uint8_t *data, uint16_t length);
void MAVLink_UART_Receive_IT(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void MAVLink_Send_Message(mavlink_message_t *msg);
void MAVLink_Process_Data(uint8_t *data, uint16_t length);

void MAVLink_Handle_Message(mavlink_message_t *msg);

void send_heartbeat();
#endif /* COMMUNICATE_H_ */

