/**
 ****************************************************************************************
 *
 * @file app_wxps_task.c
 *
 * @brief Application WXPS implementation
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
 * @addtogroup APP_WXPS_TASK
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

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_wxps_env_tag *app_wxps_env = &app_env.wxps_ev;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the WXPS.       *//**
 *
 * @param[in] msgid     WXPS_CREATE_DB_CFM
 * @param[in] param     struct wxps_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_WXPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered after a database creation. It contains status of database creation.
 ****************************************************************************************
 */
int app_wxps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct wxps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_WX_SENSOR_BIT);
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the WXPS.       *//**
 *
 * @param[in] msgid     WXPS_DISABLE_IND
 * @param[in] param     Pointer to the struct wxps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_WXPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. The configuration
 * that the collector has set in HRS attributes must be conserved and the 4 values that are 
 * important are sent back to the application for safe keeping until the next time this 
 * profile role is enabled.
 ****************************************************************************************
 */
int app_wxps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct wxps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_wxps_env->conhdl = 0xFFFF;
    app_wxps_env->enabled = false;
    app_wxps_env->ntf_sending = false;
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the eror indication nessage from the WXPS.        *//**
 *
 * @param[in] msgid     WXPS_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_WXPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of an occurred error.
 ****************************************************************************************
 */
int app_wxps_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
	  QPRINTF(("WXPS error indication.\r\n"));
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the send means confirm message from the WXPS.     *//**
 *
 * @param[in] msgid     WXPS_MEAS_SEND_CFM
 * @param[in] param     Pointer to the struct wxps_meas_send_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_WXPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to report to the application a confirmation, or error status of a notification
 * request being sent by application.
 ****************************************************************************************
 */
int app_wxps_means_send_cfm_handler(ke_msg_id_t const msgid,
                                    struct wxps_meas_send_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    if(param->conhdl == app_wxps_env->conhdl && param->status == PRF_ERR_OK)
		{		
			app_wxps_env->ntf_sending = false;
		//	QPRINTF("WXPS send status: %d.\r\n", param->status);
			app_task_msg_hdl(msgid, param);
		}
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the WXPS.     *//**
 *
 * @param[in] msgid     WXPS_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct wxps_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_WXPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is usedto inform application that peer device has changed notification
 * configuration.
 ****************************************************************************************
 */
int app_wxps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct wxps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
     QPRINTF("indntf\r\n");
    if (param->cfg_val == PRF_CLI_START_IND)
    {
        app_wxps_env->features |= WXPS_WX_MEAS_NTF_CFG;
    }
    else
    {
        app_wxps_env->features &= ~WXPS_WX_MEAS_NTF_CFG;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the energy exp reset ind message from the WXPS.       *//**
 *
 * @param[in] msgid     WXPS_ENERGY_EXP_RESET_IND
 * @param[in] param     Pointer to the struct wxps_energy_exp_reset_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_WXPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform application that Energy Expanded value shall be reset.
 ****************************************************************************************
 */
int app_wxps_energy_exp_reset_ind_handler(ke_msg_id_t const msgid,
                                          struct wxps_energy_exp_reset_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    if(param->length > 0)
		{
				app_task_msg_hdl(msgid, param);
		}

    return (KE_MSG_CONSUMED);
}

#endif // BLE_WX_SENSOR

/// @} APP_WXPS_TASK
