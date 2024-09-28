/*
 * handle_msg_mav.h
 *
 *  Created on: Sep 5, 2024
 *      Author: SAOLATEK
 */

#ifndef INC_HANDLE_MSG_MAV_H_
#define INC_HANDLE_MSG_MAV_H_

#include "c_library_v2/common/mavlink.h"
#include "communicate.h"

void Handle_Heartbeat(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Command_Long(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Attitude(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Command_Ack(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Timesync(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Status_Text(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_GPS_Raw(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Attitude(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Battery_Status(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Sys_Status(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Set_Mode(mavlink_message_t *msg, mavlink_data_t *mav_data);
void Handle_Mission_Items(mavlink_message_t *msg, mavlink_data_t *mav_data);
#endif /* INC_HANDLE_MSG_MAV_H_ */
