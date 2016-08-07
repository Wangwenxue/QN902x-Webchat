/**
 ****************************************************************************************
 *
 * @file app_wxps.c
 *
 * @brief Application WXPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_WXPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_WX_SENSOR
#include "app_wxps.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the heart rate service database - at initiation        *//**
 *
 * @param[in] features Heart rate features used to create database, possible bit-mask values are:
 * - WXPS_BODY_SENSOR_LOC_CHAR_SUP
 * - WXPS_ENGY_EXP_FEAT_SUP
 *
 * @response WXPS_CREATE_DB_CFM
 * @description
 * This function shall be send after system power-on (or after GAP Reset) in order to 
 * create heart rate profile database. This database will be visible from a peer device but
 * not usable until app_wxps_enable_req() is called within a BLE connection.
 * @note The Heart Rate profile requires the presence of one DIS characteristic
 ****************************************************************************************
 */
void app_wxps_create_db(uint8_t features)
{
    struct wxps_create_db_req * msg = KE_MSG_ALLOC(WXPS_CREATE_DB_REQ, TASK_WXPS, TASK_APP, wxps_create_db_req);

    msg->features = features;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the heart rate profile - at connection      *//**
 * 
 * @param[in] conhdl Connection handle for which the profile Heart Rate sensor role is enabled.
 * @param[in] sec_lvl Security level required for protection of HRS attributes:
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(0) or discovery(1)
 * @param[in] hr_meas_ntf_en Heart Rate Notification configuration
 * @param[in] body_sensor_loc Body sensor leocation, Possible values are:
 * - HRS_LOC_OTHER
 * - HRS_LOC_CHEST
 * - HRS_LOC_WRIST
 * - HRS_LOC_FINGER
 * - HRS_LOC_HAND
 * - HRS_LOC_EAR_LOBE
 * - HRS_LOC_FOOT
 *
 * @response None
 * @description
 * This function is used for enabling the Heart Rate Sensor role of the Heart Rate profile.
 * Before calling this function, a BLE connection shall exist with peer device. 
 * Application shall provide connection handle in order to activate the profile.
 ****************************************************************************************
 */
void app_wxps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type,
                         uint16_t hr_meas_ntf_en, uint8_t body_sensor_loc)
{
    struct wxps_enable_req * msg = KE_MSG_ALLOC(WXPS_ENABLE_REQ, TASK_WXPS, TASK_APP,
                                                wxps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->hr_meas_ntf_en = hr_meas_ntf_en;
    msg->body_sensor_loc = body_sensor_loc;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send Heart Rate measurement value - at connection.        *//**
 *
 * @param[in] conhdl Connection handle for which the profile Heart Rate sensor role is
 * enabled.
 * @param[in] meas_val Pointer to the struct hrs_hr_meas containing Heart Rate measurement value
 *
 * @response WXPS_MEAS_SEND_CFM or None
 * @description
 * This function is used by the application (which handles the Heart Rate device driver 
 * and measurements) to send a Heart Rate measurement through the Heart Rate sensor role
 ****************************************************************************************
 */
void app_wxps_measurement_send(uint16_t conhdl, uint8_t length,uint8_t *data)
{
    struct wxps_meas_send_req * msg = KE_MSG_ALLOC_DYN(WXPS_MEAS_SEND_REQ, TASK_WXPS, TASK_APP,
                                                   wxps_meas_send_req,length);

    msg->conhdl = conhdl;
   // msg->meas_val = *meas_val;
		msg->length = length;
		memcpy(msg->data, data, length);
	#ifdef DEBUG_MSG_ENABLE
		QPRINTF("measurement_send\r\n");
	#endif
    ke_msg_send(msg);
}

#endif // BLE_WX_SENSOR

/// @} APP_WXPS_API
