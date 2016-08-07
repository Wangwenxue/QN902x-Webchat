/**
 ****************************************************************************************
 *
 * @file wxps.h
 *
 * @brief Header file - Heart Rate Profile Sensor.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _WXPS_H_
#define _WXPS_H_

/**
 ****************************************************************************************
 * @addtogroup WXPS Heart Rate Profile Sensor
 * @ingroup HRP
 * @brief Heart Rate Profile Sensor
 *
 * Heart Rate sensor (HRS) profile provides functionalities to upper layer module
 * application. The device using this profile takes the role of Heart Rate sensor.
 *
 * The interface of this role to the Application is:
 *  - Enable the profile role (from APP)
 *  - Disable the profile role (from APP)
 *  - Notify peer device during Heart Rate measurement (from APP)
 *  - Indicate measurements performed to peer device (from APP)
 *
 * Profile didn't manages multiple users configuration and storage of offline measurements.
 * This must be handled by application.
 *
 * Heart Rate Profile Sensor. (WXPS): A WXPS (e.g. PC, phone, etc)
 * is the term used by this profile to describe a device that can perform Heart Rate
 * measurement and notify about on-going measurement and indicate final result to a peer
 * BLE device.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_WX_SENSOR)
#include "wxp_common.h"
#include "prf_types.h"
#include "attm.h"
#include "atts.h"
#include "atts_db.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define ATT_SVC_WX_IOT           0xFEE7 
#define WXS_IND_CHAR_UUID        0xFEC8
#define WXS_RD_CHAR_UUID         0xFEC9
#define WXS_WR_CHAR_UUID         0xFEC7

#define WXPS_WX_GET_MAX_LEN             (10)
#define WXPS_HT_MEAS_MAX_LEN            (20)

#define WXPS_MANDATORY_MASK             (0x0F)
#define WXPS_BODY_SENSOR_LOC_MASK       (0x30)
#define WXPS_WX_CTNL_PT_MASK            (0xC0)

/*
 * MACROS
 ****************************************************************************************
 */

#define WXPS_IS_SUPPORTED(mask) \
    (((wxps_env.features & mask) == mask))

///Attributes State Machine
enum
{
    WXS_IDX_SVC,

    WXS_IDX_WX_TX_CHAR,
    WXS_IDX_WX_TX_VAL,
    WXS_IDX_WX_TX_IND_CFG,

    WXS_IDX_WX_GET_CHAR,
    WXS_IDX_WX_GET_VAL,

    WXS_IDX_WX_RX_CHAR,
    WXS_IDX_WX_RX_VAL,

    WXS_IDX_NB,
};

enum
{
    WXPS_WX_MEAS_CHAR,
    WXPS_BODY_SENSOR_LOC_CHAR,
    WXPS_WX_CTNL_PT_CHAR,

    WXPS_CHAR_MAX,
};

enum
{
    /// Body Sensor Location Feature Supported
    WXPS_BODY_SENSOR_LOC_CHAR_SUP       = 0x01,
    /// Energy Expanded Feature Supported
    WXPS_ENGY_EXP_FEAT_SUP              = 0x02,

    WXPS_WX_MEAS_NTF_CFG                = 0x04,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Heart Rate Profile Sensor environment variable
struct wxps_env_tag
{
    /// Connection Info
    struct prf_con_info con_info;

    ///Service Start Handle
    uint16_t shdl;
    ///Database configuration
    uint8_t features;
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern const struct atts_desc wxps_att_db[WXS_IDX_NB];

/// Heart Rate Sensor Service - only one instance for now
extern const atts_svc_desc_t wxps_wxs_svc;

extern const struct atts_char_desc wxps_wx_tx_char;
extern const struct atts_char_desc wxps_wx_get_char;
extern const struct atts_char_desc wxps_wx_rx_char;

extern struct wxps_env_tag wxps_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the WXPS module.
 * This function performs all the initializations of the WXPS module.
 ****************************************************************************************
 */
void wxps_init(void);

/**
 ****************************************************************************************
 * @brief Send a WXPS_MEAS_SEND_CFM message to the application.
 *
 * @param[in] status Confirmation Status
 ****************************************************************************************
 */
void wxps_meas_send_cfm_send(uint8_t status);

/**
 ****************************************************************************************
 * @brief Pack Heart Rate measurement value
 *
 * @param[in] p_meas_val Heart Rate measurement value
 ****************************************************************************************
 */
uint8_t wxps_pack_meas_value(uint8_t *packed_hr, const struct hrs_hr_meas* pmeas_val);

/**
 ****************************************************************************************
 * @brief Disable actions grouped in getting back to IDLE and sending configuration to requester task
 ****************************************************************************************
 */
void wxps_disable(void);

#endif /* #if (BLE_WX_SENSOR) */

/// @} WXPS

#endif /* _WXPS_H_ */
