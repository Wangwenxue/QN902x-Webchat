/**
 ****************************************************************************************
 *
 * @file wxps_task.c
 *
 * @brief Heart Rate Profile Sensor Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup WXPSTASK
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
#include "wxps.h"
#include "wxps_task.h"

#include "prf_utils.h"
#include "app_env.h"


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WXPS_CREATE_DB_REQ message.
 * The handler adds HRS into the database using the database
 * configuration value given in param.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int wxps_create_db_req_handler(ke_msg_id_t const msgid,
                                      struct wxps_create_db_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    //Service Configuration Flag
    uint8_t cfg_flag = WXPS_MANDATORY_MASK;
    //Database Creation Status
    uint8_t status;

    //Save Profile ID
    wxps_env.con_info.prf_id = TASK_WXPS;
    //Save Database Configuration
    wxps_env.features = param->features;

    /*---------------------------------------------------*
     * Heart Rate Service Creation
     *---------------------------------------------------*/
    //Set Configuration Flag Value
    if (WXPS_IS_SUPPORTED(WXPS_BODY_SENSOR_LOC_CHAR_SUP))
    {
        cfg_flag |= WXPS_BODY_SENSOR_LOC_MASK;
    }
    if (WXPS_IS_SUPPORTED(WXPS_ENGY_EXP_FEAT_SUP))
    {
        cfg_flag |= WXPS_WX_CTNL_PT_MASK;
    }

    //Add Service Into Database
    status = atts_svc_create_db(&wxps_env.shdl, (uint8_t *)&cfg_flag, WXS_IDX_NB, NULL,
                               dest_id, &wxps_att_db[0]);
    //Disable HRS
    attsdb_svc_set_permission(wxps_env.shdl, PERM(SVC, DISABLE));
    attsdb_svc_set_permission(0x0001, PERM(SVC_HIDE, ENABLE));
	  attsdb_svc_set_permission(0x000C, PERM(SVC_HIDE, ENABLE));
    //Go to Idle State
    if (status == ATT_ERR_NO_ERROR)
    {
        //If we are here, database has been fulfilled with success, go to idle test
        ke_state_set(TASK_WXPS, WXPS_IDLE);
    }

    //Send response to application
    struct wxps_create_db_cfm * cfm = KE_MSG_ALLOC(WXPS_CREATE_DB_CFM, src_id,
                                                   TASK_WXPS, wxps_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WXPS_ENABLE_REQ message.
 * The handler enables the Heart Rate Sensor Profile.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int wxps_enable_req_handler(ke_msg_id_t const msgid,
                                   struct wxps_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint16_t value = 0;

    // Save the application task id
    wxps_env.con_info.appid = src_id;
    // Save the connection handle associated to the profile
    wxps_env.con_info.conhdl = param->conhdl;

    // Check if the provided connection exist
    if (gap_get_rec_idx(param->conhdl) == GAP_INVALID_CONIDX)
    {
        // The connection doesn't exist, request disallowed
        prf_server_error_ind_send((prf_env_struct *)&wxps_env, PRF_ERR_REQ_DISALLOWED,
                                  WXPS_ERROR_IND, WXPS_ENABLE_REQ);
    }
    else
    {
        // If this connection is a not configuration one, apply config saved by app
        if(param->con_type == PRF_CON_NORMAL)
        {
            value = param->hr_meas_ntf_en;

            if (param->hr_meas_ntf_en == PRF_CLI_START_IND)
            {
                wxps_env.features |= WXPS_WX_MEAS_NTF_CFG;
            }
        }

        //Set HR Meas. Char. NTF Configuration in DB
        attsdb_att_set_value(wxps_env.shdl + WXS_IDX_WX_TX_IND_CFG, sizeof(uint16_t),
                             (uint8_t *)&value);

        if (WXPS_IS_SUPPORTED(WXPS_BODY_SENSOR_LOC_CHAR_SUP))
        {
            //Set Body Sensor Location Char Value in DB - Not supposed to change during connection
            attsdb_att_set_value(wxps_env.shdl + WXS_IDX_WX_GET_VAL, sizeof(uint8_t),
                                 (uint8_t *)&param->body_sensor_loc);
        }

        // Enable Service + Set Security Level
        attsdb_svc_set_permission(wxps_env.shdl, param->sec_lvl);

        // Go to connected state
        ke_state_set(TASK_WXPS, WXPS_CONNECTED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WXPS_MEAS_SEND_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int wxps_meas_send_req_handler(ke_msg_id_t const msgid,
                                      struct wxps_meas_send_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Status
    uint8_t status = PRF_ERR_OK;
    // Packed Heart Measurement Value
    //uint8_t packed_hr[WXPS_HT_MEAS_MAX_LEN];
    // Packet size
   // uint8_t size;

    if((param->conhdl == wxps_env.con_info.conhdl))
           // && (param->meas_val.nb_rr_interval <= HRS_MAX_RR_INTERVAL))
    {
        //Pack the HR Measurement value
       // size = wxps_pack_meas_value(&packed_hr[0], &param->meas_val);

        // Check if notifications are enabled
        if(WXPS_IS_SUPPORTED(WXPS_WX_MEAS_NTF_CFG))
        {
            //Update value in DB
            attsdb_att_set_value(wxps_env.shdl + WXS_IDX_WX_TX_VAL,
                                 param->length, (void *)param->data);

            //send notification through GATT
            struct gatt_indicate_req * ind = KE_MSG_ALLOC(GATT_INDICATE_REQ, TASK_GATT,
                                                        TASK_WXPS, gatt_indicate_req);
            ind->conhdl  = wxps_env.con_info.conhdl;
            ind->charhdl = wxps_env.shdl + WXS_IDX_WX_TX_VAL;
	#ifdef DEBUG_MSG_ENABLE		
			QPRINTF("meas_send_req\r\n"); //nicole
	#endif
            ke_msg_send(ind);
        }
        //notification not enabled, simply don't send anything
        else
        {
            status = PRF_ERR_IND_DISABLED;
        }
    }
    else
    {
        status = PRF_ERR_INVALID_PARAM;
    }

    if (status != PRF_ERR_OK)
    {
        // Value has not been sent
        wxps_meas_send_cfm_send(status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * The handler compares the new values with current ones and notifies them if they changed.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_write_cmd_ind_handler(ke_msg_id_t const msgid,
                                      struct gatt_write_cmd_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint16_t value = 0x0000;
    uint8_t status = PRF_ERR_OK;

    if (param->conhdl == wxps_env.con_info.conhdl)
    {
			  //QPRINTF("write value\r\n");
        //BP Measurement Char. - Client Char. Configuration
        if (param->handle == (wxps_env.shdl + WXS_IDX_WX_TX_IND_CFG))
        {
            //Extract value before check
            memcpy(&value, &(param->value), sizeof(uint16_t));
					  QPRINTF("write ind value:%d\r\n",value);

            if ((value == PRF_CLI_STOP_NTFIND) || (value == PRF_CLI_START_IND))
            {
                if (value == PRF_CLI_STOP_NTFIND)
                {
                    wxps_env.features &= ~WXPS_WX_MEAS_NTF_CFG;
                }
                else //PRF_CLI_START_IND
                {
                    wxps_env.features |= WXPS_WX_MEAS_NTF_CFG;
                }
            }
            else
            {
                status = PRF_APP_ERROR;
            }

            if (status == PRF_ERR_OK)
            {
                //Update the attribute value
                attsdb_att_set_value(param->handle, sizeof(uint16_t), (uint8_t *)&value);
                if(param->last)
                {
                    //Inform APP of configuration change
                    struct wxps_cfg_indntf_ind * ind = KE_MSG_ALLOC(WXPS_CFG_INDNTF_IND,
                                                                    wxps_env.con_info.appid, TASK_WXPS,
                                                                    wxps_cfg_indntf_ind);

                    memcpy(&ind->conhdl, &wxps_env.con_info.conhdl, sizeof(uint16_t));
                    memcpy(&ind->cfg_val, &value, sizeof(uint16_t));
                    QPRINTF("ind com value:%d\r\n",value);
                    ke_msg_send(ind);
                }
            }
        }
        //HR Control Point Char. Value
        else if(param->handle == (wxps_env.shdl + WXS_IDX_WX_RX_VAL))
        {
            //if (WXPS_IS_SUPPORTED(WXPS_ENGY_EXP_FEAT_SUP))
						if (param->length <= WXPS_HT_MEAS_MAX_LEN)
            {               
							//inform APP of configuration change
							struct wxps_energy_exp_reset_ind * ind = KE_MSG_ALLOC_DYN(WXPS_ENERGY_EXP_RESET_IND,
																																		wxps_env.con_info.appid,
																																		TASK_WXPS,
																																		wxps_energy_exp_reset_ind,param->length);

							memcpy(&ind->conhdl, &(wxps_env.con_info.conhdl), sizeof(uint16_t));
							//Send received data to app value
							ind->length = param->length;
							memcpy(ind->data, param->value, param->length);

							ke_msg_send(ind);
						}
						else
						{
								status = HRS_ERR_HR_CNTL_POINT_NOT_SUPPORTED;
						}
				}
				else
				{
						//Allowed to send Application Error if value inconvenient
						status = HRS_ERR_HR_CNTL_POINT_NOT_SUPPORTED;
				}
        
    }

    if (param->response)
    {
			//QPRINTF("write response:0x%x\r\n",status);
			//Send write response
			atts_write_rsp_send(wxps_env.con_info.conhdl, param->handle, status);
		}

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_HANDLE_VALUE_CFM message meaning that Measurement indication
 * has been correctly sent to peer device.
 *
 * Convey this information to appli task using @ref BLPS_MEAS_SEND_CFM
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_handle_value_cfm_handler(ke_msg_id_t const msgid,
                                        struct gatt_handle_value_cfm const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
	 // QPRINTF("WXPS indication cfm.\r\n");
    wxps_meas_send_cfm_send(param->status);

    return (KE_MSG_CONSUMED);
}


#if 0
/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that Measurement notification
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * Convey this information to appli task using @ref WXPS_MEAS_SEND_CFM
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_notify_cmp_evt_handler(ke_msg_id_t const msgid,
                                       struct gatt_notify_cmp_evt const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    wxps_meas_send_cfm_send(param->status);

    return (KE_MSG_CONSUMED);
}

#endif

/**
 ****************************************************************************************
 * @brief Disconnection indication to WXPS.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gap_discon_cmp_evt_handler(ke_msg_id_t const msgid,
                                        struct gap_discon_cmp_evt const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    //Check Connection Handle
    if (param->conhdl == wxps_env.con_info.conhdl)
    {
        wxps_disable();
    }

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Disabled State handler definition.
const struct ke_msg_handler wxps_disabled[] =
{
    {WXPS_CREATE_DB_REQ,       (ke_msg_func_t) wxps_create_db_req_handler}
};

/// Idle State handler definition.
const struct ke_msg_handler wxps_idle[] =
{
    {WXPS_ENABLE_REQ,       (ke_msg_func_t) wxps_enable_req_handler}
};

/// Connected State handler definition.
const struct ke_msg_handler wxps_connected[] =
{
    {WXPS_MEAS_SEND_REQ,    (ke_msg_func_t) wxps_meas_send_req_handler},
    {GATT_WRITE_CMD_IND,    (ke_msg_func_t) gatt_write_cmd_ind_handler},
    //{GATT_NOTIFY_CMP_EVT,   (ke_msg_func_t) gatt_notify_cmp_evt_handler},
		{GATT_HANDLE_VALUE_CFM, (ke_msg_func_t) gatt_handle_value_cfm_handler},
};

/* Default State handlers definition. */
const struct ke_msg_handler wxps_default_state[] =
{
    {GAP_DISCON_CMP_EVT,         (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

/// Specifies the message handler structure for every input state.
const struct ke_state_handler wxps_state_handler[WXPS_STATE_MAX] =
{
    [WXPS_DISABLED]       = KE_STATE_HANDLER(wxps_disabled),
    [WXPS_IDLE]           = KE_STATE_HANDLER(wxps_idle),
    [WXPS_CONNECTED]      = KE_STATE_HANDLER(wxps_connected),
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler wxps_default_handler = KE_STATE_HANDLER(wxps_default_state);

/// Defines the place holder for the states of all the task instances.
ke_state_t wxps_state[WXPS_IDX_MAX];

// Register WXPS task into kernel
void task_wxps_desc_register(void)
{
    struct ke_task_desc  task_wxps_desc;
    
    task_wxps_desc.state_handler = wxps_state_handler;
    task_wxps_desc.default_handler=&wxps_default_handler;
    task_wxps_desc.state = wxps_state;
    task_wxps_desc.state_max = WXPS_STATE_MAX;
    task_wxps_desc.idx_max = WXPS_IDX_MAX;

    task_desc_register(TASK_WXPS, task_wxps_desc);
}

#endif /* #if (BLE_WX_SENSOR) */

/// @} WXPSTASK
