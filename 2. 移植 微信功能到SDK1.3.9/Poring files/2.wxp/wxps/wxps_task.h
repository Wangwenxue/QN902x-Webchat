/**
 ****************************************************************************************
 *
 * @file wxps_task.h
 *
 * @brief Header file - Heart Rate Profile Sensor Task.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */


#ifndef _WXPS_TASK_H_
#define _WXPS_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup WXPSTASK Task
 * @ingroup WXPS
 * @brief Heart Rate Profile Task.
 *
 * The WXPSTASK is responsible for handling the messages coming in and out of the
 * @ref WXPS collector block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if (BLE_WX_SENSOR)
#include <stdint.h>
#include "ke_task.h"
#include "wxps.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of Heart Rate task instances
#define WXPS_IDX_MAX     0x01

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the WXPS task
enum
{
    /// Disabled state
    WXPS_DISABLED,
    /// Idle state
    WXPS_IDLE,
    /// Connected state
    WXPS_CONNECTED,

    /// Number of defined states.
    WXPS_STATE_MAX,
};

/// Messages for Heart Rate Profile Sensor
enum
{
    ///Start the Heart Rate Profile Sensor - at connection
    WXPS_ENABLE_REQ = KE_FIRST_MSG(TASK_WXPS),

    /// Disable confirmation with configuration to save after profile disabled
    WXPS_DISABLE_IND,

    /// Error indication to Host
    WXPS_ERROR_IND,

    ///Send Heart Rate measurement value from APP
    WXPS_MEAS_SEND_REQ,
    ///Send Heart Rate measurement value confirm to APP so stable values can be erased
    ///if correctly sent.
    WXPS_MEAS_SEND_CFM,
    ///Inform APP of new configuration value
    WXPS_CFG_INDNTF_IND,
    ///Inform APP that Energy Expanded must be reset value
    WXPS_ENERGY_EXP_RESET_IND,

    ///Add HRS into the database
    WXPS_CREATE_DB_REQ,
    ///Inform APP about DB creation status
    WXPS_CREATE_DB_CFM,
};

///Parameters of the @ref WXPS_CREATE_DB_REQ message
struct wxps_create_db_req
{
    ///Database configuration
    uint8_t features;
};

/// Parameters of the @ref WXPS_ENABLE_REQ message
struct wxps_enable_req
{
    ///Connection handle
    uint16_t conhdl;
    /// security level: b0= nothing, b1=unauthenticated, b2=authenticated, b3=authorized;
    /// b1 or b2 and b3 can go together
    uint8_t sec_lvl;
    ///Type of connection - will someday depend on button press length; can be CFG or DISCOVERY
    uint8_t con_type;

    /// Heart Rate Notification configuration
    uint16_t hr_meas_ntf_en;

    ///Body Sensor Location
    uint8_t body_sensor_loc;
};


/////Parameters of the @ref WXPS_MEAS_SEND_REQ message
struct wxps_meas_send_req
{
    ///Connection handle
    uint16_t conhdl;
    ///Heart Rate measurement
   // struct hrs_hr_meas meas_val;
	  uint8_t length;
	  uint8_t data[1];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_WXPS_TASK
 * @{
 ****************************************************************************************
 */

///Parameters of the @ref WXPS_CREATE_DB_CFM message
struct wxps_create_db_cfm
{
    ///Status
    uint8_t status;
};

///Parameters of the @ref WXPS_DISABLE_IND message
struct wxps_disable_ind
{
    uint16_t conhdl;
    /// Heart Rate Notification configuration
    uint16_t hr_meas_ntf_en;
};


///Parameters of the @ref WXPS_CFG_INDNTF_IND message
struct wxps_cfg_indntf_ind
{
    ///Connection handle
    uint16_t conhdl;
    ///Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

///Parameters of the @ref WXPS_MEAS_SEND_CFM message
struct wxps_meas_send_cfm
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t status;
};

///Parameters of the @ref WXPS_ENERGY_EXP_RESET_IND message
struct wxps_energy_exp_reset_ind
{
    ///Connection handle
    uint16_t conhdl;
	  uint8_t length;
    uint8_t data[20];
};
/// @} APP_WXPS_TASK

/// @cond



/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler wxps_state_handler[WXPS_STATE_MAX];
extern const struct ke_state_handler wxps_default_handler;
extern ke_state_t wxps_state[WXPS_IDX_MAX];

extern void task_wxps_desc_register(void);

#endif /* #if BLE_WX_SENSOR */

/// @} WXPSTASK
/// @endcond
#endif /* _WXPS_TASK_H_ */
