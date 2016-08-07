/**
 ****************************************************************************************
 *
 * @file wxps.c
 *
 * @brief Heart Rate Profile Sensor implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup WXPS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_WX_SENSOR)
#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"
#include "smpc_task.h"
#include "wxps.h"
#include "wxps_task.h"

/*
 * HTPT PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// Full HRS Database Description - Used to add attributes into the database
const struct atts_desc wxps_att_db[WXS_IDX_NB] =
{
    // Heart Rate Service Declaration
    [WXS_IDX_SVC]                      =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), sizeof(wxps_wxs_svc),
                                         sizeof(wxps_wxs_svc), (uint8_t *)&wxps_wxs_svc},

    // Heart Rate Measurement Characteristic Declaration
    [WXS_IDX_WX_TX_CHAR]            =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(wxps_wx_tx_char),
                                         sizeof(wxps_wx_tx_char), (uint8_t *)&wxps_wx_tx_char},
    // Heart Rate Measurement Characteristic Value
    [WXS_IDX_WX_TX_VAL]             =   {ATT_CHAR_HEART_RATE_MEAS, PERM(IND, ENABLE), WXPS_HT_MEAS_MAX_LEN,
                                         0, NULL},
    // Heart Rate Measurement Characteristic - Client Characteristic Configuration Descriptor
    [WXS_IDX_WX_TX_IND_CFG]         =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE)|PERM(WR, ENABLE), sizeof(uint16_t),
                                         0, NULL},

    // Body Sensor Location Characteristic Declaration
    [WXS_IDX_WX_GET_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(wxps_wx_get_char),
                                         sizeof(wxps_wx_get_char), (uint8_t *)&wxps_wx_get_char},
    // Body Sensor Location Characteristic Value
    [WXS_IDX_WX_GET_VAL]   =   {ATT_CHAR_BODY_SENSOR_LOCATION, PERM(RD, ENABLE), WXPS_WX_GET_MAX_LEN*sizeof(uint8_t),
                                         0, NULL},

    // Heart Rate Control Point Characteristic Declaration
    [WXS_IDX_WX_RX_CHAR]        =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(wxps_wx_rx_char),
                                         sizeof(wxps_wx_rx_char), (uint8_t *)&wxps_wx_rx_char},
    // Heart Rate Control Point Characteristic Value
    [WXS_IDX_WX_RX_VAL]         =   {ATT_CHAR_HEART_RATE_CNTL_POINT, PERM(WR, ENABLE), WXPS_HT_MEAS_MAX_LEN,
                                         0, NULL},
};

/*
 *  Heart Rate PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// Heart Rate Sensor Service
const atts_svc_desc_t wxps_wxs_svc = ATT_SVC_WX_IOT;

/// Heart Rate Sensor Service - Heart Rate Measurement Characteristic
const struct atts_char_desc wxps_wx_tx_char = ATTS_CHAR(ATT_CHAR_PROP_IND,
                                                          0,
                                                          WXS_IND_CHAR_UUID);

/// Heart Rate Sensor Service -Body Sensor Location characteristic
const struct atts_char_desc wxps_wx_get_char = ATTS_CHAR(ATT_CHAR_PROP_RD,
                                                                  0,
                                                                  WXS_RD_CHAR_UUID);

/// Heart Rate Sensor Service - Heart Rate Control Point characteristic
const struct atts_char_desc wxps_wx_rx_char = ATTS_CHAR(ATT_CHAR_PROP_WR,
                                                                0,
                                                                WXS_WR_CHAR_UUID);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct wxps_env_tag wxps_env;

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void wxps_init(void)
{
    // Reset environment
    memset(&wxps_env, 0, sizeof(wxps_env));

	// Register WXPS task into kernel
    task_wxps_desc_register();

    // Go to IDLE state
    ke_state_set(TASK_WXPS, WXPS_DISABLED);
}

void wxps_meas_send_cfm_send(uint8_t status)
{
    // Send CFM to APP that value has been sent or not
    struct wxps_meas_send_cfm * cfm = KE_MSG_ALLOC(WXPS_MEAS_SEND_CFM, wxps_env.con_info.appid,
                                                   TASK_WXPS, wxps_meas_send_cfm);

    cfm->conhdl = wxps_env.con_info.conhdl;
    cfm->status = status;

    ke_msg_send(cfm);
}

uint8_t wxps_pack_meas_value(uint8_t *packed_hr, const struct hrs_hr_meas* pmeas_val)
{
    uint8_t cursor = 0;
    uint8_t i = 0;

    // Heart Rate measurement flags
    *(packed_hr) = pmeas_val->flags;

    if ((pmeas_val->flags & HRS_FLAG_HR_16BITS_VALUE) == HRS_FLAG_HR_16BITS_VALUE)
    {
        // Heart Rate Measurement Value 16 bits
        co_write16p(packed_hr + 1, pmeas_val->heart_rate);
        cursor += 3;
    }
    else
    {
        // Heart Rate Measurement Value 8 bits
        *(packed_hr + 1) = pmeas_val->heart_rate;
        cursor += 2;
    }

    if ((pmeas_val->flags & HRS_FLAG_ENERGY_EXPENDED_PRESENT) == HRS_FLAG_ENERGY_EXPENDED_PRESENT)
    {
        // Energy Expended present
        co_write16p(packed_hr + cursor, pmeas_val->energy_expended);
        cursor += 2;
    }

    if ((pmeas_val->flags & HRS_FLAG_RR_INTERVAL_PRESENT) == HRS_FLAG_RR_INTERVAL_PRESENT)
    {
        for(i = 0 ; (i < (pmeas_val->nb_rr_interval)) && (i < (HRS_MAX_RR_INTERVAL)) ; i++)
        {
            // RR-Intervals
            co_write16p(packed_hr + cursor, pmeas_val->rr_intervals[i]);
            cursor += 2;
        }
    }

    // Clear unused packet data
    if(cursor < WXPS_HT_MEAS_MAX_LEN)
    {
        memset(packed_hr + cursor, 0, WXPS_HT_MEAS_MAX_LEN - cursor);
    }

    return cursor;
}

void wxps_disable(void)
{
    //Disable HRS in database
    attsdb_svc_set_permission(wxps_env.shdl, PERM_RIGHT_DISABLE);

    //Send current configuration to APP
    struct wxps_disable_ind *ind = KE_MSG_ALLOC(WXPS_DISABLE_IND,
                                                wxps_env.con_info.appid, TASK_WXPS,
                                                wxps_disable_ind);

    memcpy(&ind->conhdl, &wxps_env.con_info.conhdl, sizeof(uint16_t));

    //Heart Rate Measurement Char. - Notifications Configuration
    if (WXPS_IS_SUPPORTED(WXPS_WX_MEAS_NTF_CFG))
    {
        ind->hr_meas_ntf_en = PRF_CLI_START_IND;

        //Reset notifications bit field
        wxps_env.features &= ~WXPS_WX_MEAS_NTF_CFG;
    }

    ke_msg_send(ind);

    //Go to idle state
    ke_state_set(TASK_WXPS, WXPS_IDLE);
}

#endif /* BLE_WX_SENSOR */

/// @} WXPS
