/**
 ****************************************************************************************
 *
 * @file usr_design.c
 *
 * @brief Product related design.
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
 * @addtogroup  USR
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "app_env.h"
#include "led.h"
#include "uart.h"
#include "lib.h"
#include "usr_design.h"
#include "gpio.h"
#include "button.h"
#include "sleep.h"

// wenxue
#include "epb.h"
#include "epb_Mmbp.h"
#include "crc32.h"
#ifdef WX_MD5ENC
#include "aes.h"
#endif
#if defined(WX_MD5ENC)||defined(WX_MD5NOENC)
#include "md5.h"
#endif
#ifdef UART0_QPPS_ENABLE  //ericyang 20160218
uint8_t msg_source = SRC_MESSAGE_NONE;  // 0: null 1:WECHAT  2: OTHER
uint8_t uart_msg[128];
uint8_t uart_msg_len;
#endif


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

#define LED_ON_DUR_ADV_FAST        2
#define LED_OFF_DUR_ADV_FAST       (uint16_t)((GAP_ADV_FAST_INTV2*0.625)/10)
#define LED_ON_DUR_ADV_SLOW        2
#define LED_OFF_DUR_ADV_SLOW       (uint16_t)((GAP_ADV_SLOW_INTV*0.625)/10)
#define LED_ON_DUR_CON          0xffff
#define LED_OFF_DUR_CON                   0
#define LED_ON_DUR_IDLE                   0
#define LED_OFF_DUR_IDLE                  0xffff

#define APP_HEART_RATE_MEASUREMENT_TO     1400 // 14s
//#define APP_HRPS_ENERGY_EXPENDED_STEP     50
#define APP_WXPS_ENERGY_EXPENDED_STEP     50 // wenxue

#define EVENT_BUTTON1_PRESS_ID            0

// wenxue
static uint16_t packet_num = 0;
static uint16_t packet_send = 0;
struct wxs_send_ind *frame ={0};
//static uint16_t wxSeq = 0;

#ifdef WX_MD5ENC
const uint8_t key[16] = DEVICE_KEY
uint8_t session_key[16] = {0};
#endif

#if defined(WX_MD5NOENC)||defined(WX_MD5ENC)
uint8_t md5_type_and_id[16];
#endif

///IOS Connection Parameter
#define IOS_CONN_INTV_MAX                              0x0010
#define IOS_CONN_INTV_MIN                              0x0008
#define IOS_SLAVE_LATENCY                              0x0000
#define IOS_STO_MULT                                   0x012c

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct usr_env_tag usr_env = {LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE};


// wenxue
#define BigLittleSwap16(A)  ((((uint16_t)(A) & 0xff00) >> 8) | \
                            (((uint16_t)(A) & 0x00ff) << 8))

 
#define BigLittleSwap32(A)  ((((uint32_t)(A) & 0xff000000) >> 24) | \
                            (((uint32_t)(A) & 0x00ff0000) >> 8) | \
                            (((uint32_t)(A) & 0x0000ff00) << 8) | \
                            (((uint32_t)(A) & 0x000000ff) << 24))

bool checkCPUendian()
{
       union{
              uint32_t i;
              uint8_t s[4];
       }c;
 
       c.i = 0x12345678;
       return (0x12 == c.s[0]);
}

uint32_t t_htonl(uint32_t h)
{
       return checkCPUendian() ? h : BigLittleSwap32(h);
}
 
uint32_t t_ntohl(uint32_t n)
{

       return checkCPUendian() ? n : BigLittleSwap32(n);
}

uint16_t htons(uint16_t h)
{
       return checkCPUendian() ? h : BigLittleSwap16(h);
}
 
uint16_t ntohs(uint16_t n)
{
       return checkCPUendian() ? n : BigLittleSwap16(n);
}

/*turn an unsigned short value to big-endian value					*/
/*for example 0x1234 in the memory of X86 is 0x34 and 0x12	*/
/*then turn it to Network Byte Order is 0x12 and 0x34				*/

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Led1 for BLE status
 ****************************************************************************************
 */
static void usr_led1_set(uint16_t timer_on, uint16_t timer_off)
{
    usr_env.led1_on_dur = timer_on;
    usr_env.led1_off_dur = timer_off;

    if (timer_on == 0 || timer_off == 0)
    {
        if (timer_on == 0)
        {
            led_set(1, LED_OFF);
        }
        if (timer_off == 0)
        {
            led_set(1, LED_ON);
        }
        ke_timer_clear(APP_SYS_LED_1_TIMER, TASK_APP);
    }
    else
    {
        led_set(1, LED_OFF);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, timer_off);
    }
}

/**
 ****************************************************************************************
 * @brief   Led 1 flash process
 ****************************************************************************************
 */
static void usr_led1_process(void)
{
    if(led_get(1) == LED_ON)
    {
        led_set(1, LED_OFF);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, usr_env.led1_off_dur);
    }
    else
    {
        led_set(1, LED_ON);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, usr_env.led1_on_dur);
    }
}


// wenxue
static int32_t wx_get_md5(void)
{
	int32_t error_code = 0;
	
	#if defined(WX_MD5ENC)||defined(WX_MD5NOENC)
	char device_type[] = DEVICE_TYPE;
	char device_id[] = DEVICE_ID;
	char argv[sizeof(DEVICE_TYPE) + sizeof(DEVICE_ID) - 1];
	memcpy(argv,device_type,sizeof(DEVICE_TYPE));
  /*when add the DEVICE_ID to DEVICE_TYPE, the offset shuld -1 to overwrite '\0'  at the end of DEVICE_TYPE */
	memcpy(argv + sizeof(DEVICE_TYPE)-1,device_id,sizeof(DEVICE_ID));
  #ifdef CATCH_LOG	
	printf ( "\r\nDEVICE_TYPE and DEVICE_ID:%s\r\n",argv);
  #endif
	error_code = md5(argv, md5_type_and_id);
  #ifdef CATCH_LOG
	printf ( "\r\nMD5:");
	for ( uint8_t i = 0; i < 16; i++ )
	printf ( " %02x", md5_type_and_id[i] );
	putchar ( '\n' );
  #endif
  #endif
	return error_code;
}

void wxps_send_data(uint8_t* datum,uint16_t length)
 {	 
	 
		// QPRINTF("fh sent len:%d,data:0x%x\r\n",length,datum[0]);    
		
		 struct wxs_send_ind *pt_data = KE_MSG_ALLOC_DYN(APP_WXPS_SEND_TIMER,
										 TASK_APP,
										 TASK_APP, wxs_send_ind,length);
		 pt_data->len = length;
		 memcpy(pt_data->data,datum,length);
		 ke_msg_send(pt_data);  	
 }

/**
 ****************************************************************************************
 * @brief   Application task message handler
 ****************************************************************************************
 */
void app_task_msg_hdl(ke_msg_id_t const msgid, void const *param)
{
    switch(msgid)
    {
        case GAP_SET_MODE_REQ_CMP_EVT:
					#if 0
            if(APP_IDLE == ke_state_get(TASK_APP))
            {
                usr_led1_set(LED_ON_DUR_ADV_FAST, LED_OFF_DUR_ADV_FAST);
                ke_timer_set(APP_ADV_INTV_UPDATE_TIMER, TASK_APP, 30 * 100);
            }
            else if(APP_ADV == ke_state_get(TASK_APP))
            {
                usr_led1_set(LED_ON_DUR_ADV_SLOW, LED_OFF_DUR_ADV_SLOW);
            }
            break;
						#endif
						
						// wenxue
						if(APP_INIT == ke_state_get(TASK_APP))
						{
							// start adv
							app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
											app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
											app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
											GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
							
						}
            else if(APP_IDLE == ke_state_get(TASK_APP))
            {
                usr_led1_set(LED_ON_DUR_ADV_FAST, LED_OFF_DUR_ADV_FAST);
                //ke_timer_set(APP_ADV_INTV_UPDATE_TIMER, TASK_APP, 30 * 100);
            }
            else if(APP_ADV == ke_state_get(TASK_APP))
            {
                usr_led1_set(LED_ON_DUR_ADV_SLOW, LED_OFF_DUR_ADV_SLOW);
            }
            break;

        case GAP_ADV_REQ_CMP_EVT:
            usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);
         //   ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP); // wenxue
            break;

        case GAP_DISCON_CMP_EVT:
            usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);

				      wxSeq = 0; // wenxue
            // start adv
            app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                    app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                    app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                    GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
            break;

        case GAP_LE_CREATE_CONN_REQ_CMP_EVT:
					#if 0
            if(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.status == CO_ERROR_NO_ERROR)
            {
                if(GAP_PERIPHERAL_SLV == app_get_role())
                { 
                    ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP);  
                    usr_led1_set(LED_ON_DUR_CON, LED_OFF_DUR_CON);

                    // Update cnx parameters
                    //if (((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.con_interval >  IOS_CONN_INTV_MAX)
                    {
                        // Update connection parameters here
                        struct gap_conn_param_update conn_par;
                        /// Connection interval minimum
                        conn_par.intv_min = IOS_CONN_INTV_MIN;
                        /// Connection interval maximum
                        conn_par.intv_max = IOS_CONN_INTV_MAX;
                        /// Latency
                        conn_par.latency = IOS_SLAVE_LATENCY;
                        /// Supervision timeout, Time = N * 10 msec
                        conn_par.time_out = IOS_STO_MULT;
                        app_gap_param_update_req(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.conhdl, &conn_par);
                    }
                }
            }
            break;
				#endif
						
						// wenxue
						 if(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.status == CO_ERROR_NO_ERROR)
            {
                if(GAP_PERIPHERAL_SLV == app_get_role())
                {
                    //ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP);
                    usr_led1_set(LED_ON_DUR_CON, LED_OFF_DUR_CON);
                    #if 1
                    // Update cnx parameters
                    if (((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.con_interval < GAP_PPCP_CONN_INTV_MIN)
                    {
                        // Update connection parameters here
                        struct gap_conn_param_update conn_par;
                        /// Connection interval minimum
                        conn_par.intv_min = 0x08;//GAP_PPCP_CONN_INTV_MIN;
                        /// Connection interval maximum
                        conn_par.intv_max = 0x10;// GAP_PPCP_CONN_INTV_MAX;
                        /// Latency
                        conn_par.latency = 0x00;//GAP_PPCP_SLAVE_LATENCY;
                        /// Supervision timeout, Time = N * 10 msec
                        conn_par.time_out = 0x12c;//GAP_PPCP_STO_MULT;
                        app_gap_param_update_req(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.conhdl, &conn_par);
                    }
										#endif
                }
            }
            break;
       // wenxue
						case WXPS_DISABLE_IND:
            //ke_timer_clear(APP_WXPS_TIMER, TASK_APP);
            break;

        case WXPS_CFG_INDNTF_IND:
				    {		
						#ifdef DEBUG_MSG_ENABLE
						    QPRINTF("start auto req\r\n");	
						#endif
						    ke_timer_set(WXPS_AUTH_REQ_TIMER,TASK_APP,5);						
					  }
            break;
				case WXPS_ENERGY_EXP_RESET_IND:// rx 
						{
							static uint8_t p_data[128] = {0};
							static uint8_t len = 0;
							static uint8_t n = 0;
							
							len += ((struct wxps_energy_exp_reset_ind *)param)->length;
							
						//	if(((struct wxps_energy_exp_reset_ind *)param)->data[0] == 0xFE) n=0;								
							//for(uint8_t k=0;k< len;k++)
							//{
							//	p_data[k+n*20] = ((struct wxps_energy_exp_reset_ind *)param)->data[k];
						//	}
							
							memcpy(&p_data[n*20],((struct wxps_energy_exp_reset_ind *)param)->data,len);
							n++;
              					//QPRINTF("received:\r\n");							
							//for(uint8_t i=0;i<6;i++)
							//QPRINTF("0x%x,",p_data[i]);
							//QPRINTF("len:%d\r\n",((p_data[2] << 8) | p_data[3]));
							//QPRINTF("\r\n");
							if(len >= ((p_data[2] << 8) | p_data[3]))
							{	
							#ifdef DEBUG_MSG_ENABLE
							 	QPRINTF("received:\r\n");//nicole							
							  	 for(uint8_t i=0;i<len;i++) //nicole
							     	QPRINTF("0x%x,",p_data[i]);//nicole
								//QPRINTF("len:%d\r\n",((p_data[2] << 8) | p_data[3]));
							  	QPRINTF("\r\n"); 
							#endif
                //if(40 >= len)
								uint16_t cmdID = (p_data[4] << 8) | p_data[5];
								uint16_t cmdTimer = 0;
								switch(cmdID)
								{
									case ECI_resp_auth:
										cmdTimer = WXPS_AUTH_RESP_TIMER;
										break;
									case ECI_resp_init:
										cmdTimer = WXPS_INIT_RESP_TIMER;
										break;
									case ECI_resp_sendData:
										cmdTimer = WXPS_SDATA_RESP_TIMER;
										break;
									case ECI_push_recvData:
										cmdTimer = WXPS_RDATA_PUSH_TIMER;
										break;
									case ECI_push_switchView:
										cmdTimer = WXPS_SVIEW_PUSH_TIMER;
										break;
									case ECI_push_switchBackgroud:
										break;
									default:
										cmdTimer = WXPS_SBG_PUSH_TIMER;
										break;
									
									
								}
                						{									
								struct BpFixHeadExt *ind = KE_MSG_ALLOC_DYN(cmdTimer,
										                TASK_APP,TASK_APP, BpFixHeadExt,len);
								
								ind->BpFixHeader.bMagicNumber = p_data[0];
								ind->BpFixHeader.bVer = p_data[1];
								ind->BpFixHeader.nLength = (p_data[2] << 8) | p_data[3];
								ind->BpFixHeader.nCmdId = (p_data[4] << 8) | p_data[5];
								ind->BpFixHeader.nSeq = (p_data[6] << 8) | p_data[7];
								
                						memcpy(ind->data,&p_data[8],ind->BpFixHeader.nLength - 8);								
								ke_msg_send(ind);
							}
								
								n=0;
								len = 0;							
							}
						
						}
					break;
				case WXPS_MEAS_SEND_CFM:
				
					if(packet_send < packet_num)
					{
						 if (app_wxps_env->ntf_sending == false && app_wxps_env->features & WXPS_WX_MEAS_NTF_CFG)
						 {              
							 app_wxps_measurement_send(app_wxps_env->conhdl,WXPS_HT_MEAS_MAX_LEN, (frame->data+packet_send*20));
							 app_wxps_env->ntf_sending = true;				 
							 packet_send++;				 
						 }
				 }				
					break;
						
						
        case QPPS_DISABLE_IND:
            break;

        case QPPS_CFG_INDNTF_IND:
            break;

        default:
            break;
    }
}

/**
 ****************************************************************************************
 * @brief Handles LED status timer.
 *
 * @param[in] msgid      APP_SYS_UART_DATA_IND
 * @param[in] param      Pointer to struct app_uart_data_ind
 * @param[in] dest_id    TASK_APP
 * @param[in] src_id     TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_led_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(msgid == APP_SYS_LED_1_TIMER)
    {
        usr_led1_process();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles advertising mode timer event.
 *
 * @param[in] msgid     APP_ADV_INTV_UPDATE_TIMER
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that first phase of adversting mode is timeout.
 ****************************************************************************************
 */
int app_gap_adv_intv_update_timer_handler(ke_msg_id_t const msgid, void const *param,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(APP_ADV == ke_state_get(TASK_APP))
    {
        usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);

        // Update Advertising Parameters
        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE, 
                                app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE), 
                                app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                                GAP_ADV_SLOW_INTV, GAP_ADV_SLOW_INTV);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   Restore peripheral setting after wakeup
 ****************************************************************************************
 */
void usr_sleep_restore(void)
{
#if QN_DBG_PRINT
   // uart_init(QN_DEBUG_UART, USARTx_CLK(0), UART_9600);
	uart_init(QN_DEBUG_UART, USARTx_CLK(0), UART_115200); // wenxue
    uart_tx_enable(QN_DEBUG_UART, MASK_ENABLE);
    uart_rx_enable(QN_DEBUG_UART, MASK_ENABLE);
#endif
}

/**
 ****************************************************************************************
 * @brief Handles button press after cancel the jitter.
 *
 * @param[in] msgid     Id of the message received
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_button_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case APP_SYS_BUTTON_1_TIMER:
            // make sure the button is pressed
            if(gpio_read_pin(BUTTON1_PIN) == GPIO_LOW)
            {
                if(APP_IDLE == ke_state_get(TASK_APP))
                {
                    if(!app_qpps_env->enabled)
                    {
                        // start adv
                        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                                app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                                app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                                GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);

#if (QN_DEEP_SLEEP_EN)
                        // prevent entering into deep sleep mode
                        sleep_set_pm(PM_SLEEP);
#endif
                    }
                }
                else if(APP_ADV == ke_state_get(TASK_APP))
                {
                    // stop adv
                    app_gap_adv_stop_req();

#if (QN_DEEP_SLEEP_EN)
                    // allow entering into deep sleep mode
                    sleep_set_pm(PM_DEEP_SLEEP);
#endif
                }
            }
            break;

        default:
            ASSERT_ERR(0);
            break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_button1_press_handler(void)
{
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    if (sleep_env.deep_sleep)
    {
        sleep_env.deep_sleep = false;
        // start 32k xtal wakeup timer
        wakeup_32k_xtal_start_timer();
    }
#endif

    // delay 20ms to debounce
    ke_timer_set(APP_SYS_BUTTON_1_TIMER, TASK_APP, 2);
    ke_evt_clear(1UL << EVENT_BUTTON1_PRESS_ID);
}

/**
 ****************************************************************************************
 * @brief   Button 1 click callback
 * @description
 *  Button 1 is used to enter adv mode.
 ****************************************************************************************
 */
void usr_button1_cb(void)
{
    // If BLE is in the sleep mode, wakeup it.
    if(ble_ext_wakeup_allow())
    {
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
        if (sleep_env.deep_sleep)
        {
            wakeup_32k_xtal_switch_clk();
        }
#endif

        sw_wakeup_ble_hw();

    }

    // key debounce:
    // We can set a soft timer to debounce.
    // After wakeup BLE, the timer is not calibrated immediately and it is not precise.
    // So We set a event, in the event handle, set the soft timer.
    ke_evt_set(1UL << EVENT_BUTTON1_PRESS_ID);
}

/**
 ****************************************************************************************
 * @brief   All GPIO interrupt callback
 ****************************************************************************************
 */
void gpio_interrupt_callback(enum gpio_pin pin)
{
    switch(pin)
    {
        case BUTTON1_PIN:
            //Button 1 is used to enter adv mode.
            usr_button1_cb();
            break;

#if (defined(QN_TEST_CTRL_PIN))
        case QN_TEST_CTRL_PIN:
            //When test controll pin is changed to low level, this function will reboot system.
            gpio_disable_interrupt(QN_TEST_CTRL_PIN);
            syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_REBOOT_SYS);
            break;
#endif

        default:
            break;
    }
}


/**
 ****************************************************************************************
 * @brief   User initialize
 ****************************************************************************************
 */
void usr_init(void)
{
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_BUTTON1_PRESS_ID, 
                                            app_event_button1_press_handler))
    {
        ASSERT_ERR(0);
    }
		wx_get_md5(); // wenxue
}


// wenxue

int app_wxps_send_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	 frame = (struct wxs_send_ind*)param;	 
	
	 if(frame->len > 0)
	 {
		 uint8_t packet_res = frame->len % WXPS_HT_MEAS_MAX_LEN;		
	  	 packet_send = 0;		 
		 if(packet_res)
		 {			 
		     	packet_num = frame->len/WXPS_HT_MEAS_MAX_LEN + 1;
			for(uint8_t i =0;i< (WXPS_HT_MEAS_MAX_LEN - packet_res);i++)
			{
				frame->data[frame->len+i] = 0x00;				   
			}
		 }
		 else
		 {
		     packet_num = frame->len/WXPS_HT_MEAS_MAX_LEN;
		 }	
#ifdef DEBUG_MSG_ENABLE
		QPRINTF("packet num:%d\r\n",packet_num);
#endif
		 if (app_wxps_env->ntf_sending == false && app_wxps_env->features & WXPS_WX_MEAS_NTF_CFG)
		 {   
		 #ifdef DEBUG_MSG_ENABLE
		 	QPRINTF("send timer handler\r\n"); //nicole
		 #endif
			app_wxps_measurement_send(app_wxps_env->conhdl,WXPS_HT_MEAS_MAX_LEN, (frame->data+packet_send*20));
     			app_wxps_env->ntf_sending = true;				 
    			packet_send++;				 
		 }			 
	 }													
	 return (KE_MSG_CONSUMED);
}


static uint8_t challeange[CHALLENAGE_LENGTH] = {0x11,0x22,0x33,0x44}; 

int app_wxps_dev2wx_proc_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	uint32_t length = 0;
	uint8_t *data=NULL;
	#ifdef FIXED_BLE_WECHAT_HEADER
	CString send_msg ={SEND_HELLO_WECHAT,strlen(SEND_HELLO_WECHAT)};
	#else
	CString send_msg ={SEND_HELLO_WECHAT,sizeof(SEND_HELLO_WECHAT)};
	#endif
//	CString send_msg ={SEND_HELLO_WECHAT,strlen(SEND_HELLO_WECHAT)};
#ifdef FIXED_BLE_WECHAT_HEADER
	//static uint16_t bleDemoHeadLen = sizeof(BpFixHead);	
#else
	static uint16_t bleDemoHeadLen = sizeof(BlueDemoHead);	
#endif
	BaseRequest basReq = {NULL};
	static uint8_t fix_head_len = sizeof(BpFixHead);
	BpFixHead fix_head = {0xFE, 1, 0, htons(ECI_req_auth), 0};
	wxSeq ++;
	
	switch(msgid)
	{
		case WXPS_AUTH_REQ_TIMER:
		{
			#if defined WX_MD5ENC
				uint8_t deviceid[] = DEVICE_ID;
				static uint32_t seq = 0x00000001;
				uint32_t ran = 0x11223344;//this is random, you may set it, currently it is set to the fixed number.
				ran = t_htonl(ran);
				seq = t_htonl(seq);
				uint8_t id_len = strlen(DEVICE_ID);
				uint8_t* id_data = malloc(id_len+8);
				if(!id_data){QPRINTF("\r\nNot enough memory!");}
				memcpy(id_data,deviceid,id_len);
				memcpy(id_data+id_len,(uint8_t*)&ran,4);
				memcpy(id_data+id_len+4,(uint8_t*)&seq,4);
				uint32_t crc = crc32(0, id_data, id_len+8);
				crc = t_htonl(crc);
				memset(id_data,0x00,id_len+8);
				memcpy(id_data,(uint8_t*)&ran,4);
				memcpy(id_data+4,(uint8_t*)&seq,4);
				memcpy(id_data+8,(uint8_t*)&crc,4);	
				uint8_t CipherText[16];
				AES_Init(key);
				AES_Encrypt_PKCS7 (id_data, CipherText, 12, key);
				if(id_data){free(id_data);id_data = NULL;}
				AuthRequest authReq = {&basReq, true,{md5_type_and_id, MD5_TYPE_AND_ID_LENGTH}, PROTO_VERSION, AUTH_PROTO, (EmAuthMethod)AUTH_METHOD, true ,{CipherText, CIPHER_TEXT_LENGTH}, false, {NULL, 0}, false, {NULL, 0}, false, {NULL, 0},true,{DEVICE_ID,sizeof(DEVICE_ID)}};
				seq++;
			#endif
			#if defined WX_MACNOENC
					QPRINTF("TEST:%d\r\n");  //TEST
				static uint8_t macAddr[MAC_ADDRESS_LENGTH];
			
			  nvds_tag_len_t name_length = MAC_ADDRESS_LENGTH;
				uint8_t tt[MAC_ADDRESS_LENGTH]={0};
				if (NVDS_OK == nvds_get(NVDS_TAG_BD_ADDRESS, &name_length, &tt[0]))
				{		
							for(uint8_t i=0;i<MAC_ADDRESS_LENGTH;i++)
									macAddr[i] = tt[MAC_ADDRESS_LENGTH-1-i];		
				}
				
        AuthRequest authReq = {&basReq, false,{NULL, 0}, PROTO_VERSION, AUTH_PROTO, (EmAuthMethod)AUTH_METHOD, false,{NULL, 0}, 
               true, {macAddr, MAC_ADDRESS_LENGTH}, false, {NULL, 0}, false, {NULL, 0},true,{DEVICE_ID,sizeof(DEVICE_ID)}};
      #endif
			
			#ifdef WX_MD5NOENC
				AuthRequest authReq = {&basReq, true,{md5_type_and_id, MD5_TYPE_AND_ID_LENGTH}, PROTO_VERSION, (EmAuthMethod)AUTH_PROTO, 
				        (EmAuthMethod)AUTH_METHOD, false ,{NULL, 0}, false, {NULL, 0}, false, {NULL, 0}, false, {NULL, 0},
				         true,{DEVICE_ID,sizeof(DEVICE_ID)}};
			#endif
				
			length = epb_auth_request_pack_size(&authReq) + fix_head_len;
				data = (uint8_t *)malloc(length);
				if(!(data)){QPRINTF("\r\nNot enough memory!\r\n");}
				if(epb_pack_auth_request(&authReq, data+fix_head_len, length-fix_head_len)<0)
				{
					data = NULL;					
				}
				fix_head.nCmdId = htons(ECI_req_auth);
				fix_head.nLength = htons(length);
				fix_head.nSeq = htons(wxSeq);
				memcpy(data, &fix_head, fix_head_len);
				#ifdef DEBUG_MSG_ENABLE
				QPRINTF("auth req len:%d\r\n",length);
				#endif
				wxps_send_data(data,length);
       
		}
			break;
		
		case WXPS_INIT_REQ_TIMER:
		{
			//has challeange
				InitRequest initReq = {&basReq,false, {NULL, 0},true, {challeange, CHALLENAGE_LENGTH}};
				length = epb_init_request_pack_size(&initReq) + fix_head_len;
				
				#if defined WX_MD5ENC
				uint8_t len = length;				
				uint8_t *p = malloc(AES_get_length( length-fix_head_len));
				if(!p){QPRINTF("\r\nNot enough memory!");}
				length = AES_get_length( length-fix_head_len)+fix_head_len;
			  #endif
			
			//pack data
				data = (uint8_t *)malloc(length);
				if(!(data)){QPRINTF("\r\nNot enough memory!");}
				if(epb_pack_init_request(&initReq, data+fix_head_len, length-fix_head_len)<0)
				{
				  data = NULL;
				  QPRINTF("epb_pack_init_request failed\r\n");
				}
				
				//encrypt body
        		#if defined WX_MD5ENC
				AES_Init(session_key);
				AES_Encrypt_PKCS7(data+fix_head_len,p,len-fix_head_len,session_key);//the original data length
				memcpy(data + fix_head_len, p, length-fix_head_len);
				if(p){free(p);}
			  #endif				
				
				fix_head.nCmdId = htons(ECI_req_init);
				fix_head.nLength = htons(length);
				fix_head.nSeq = wxSeq;
				memcpy(data, &fix_head, fix_head_len);
				#ifdef DEBUG_MSG_ENABLE
				QPRINTF("\r\n init req:%d\r\n",length);
				#endif
				wxps_send_data(data,length);	
			}
			break;
		
		case WXPS_SDATA_REQ_TIMER:
			{
				
				//  QPRINTF("msg to send:%s,%d\r\n",send_msg.str,send_msg.len);
				  #ifdef FIXED_BLE_WECHAT_HEADER
				  	//SendDataRequest sendDatReq = {&basReq, {(uint8_t*) send_msg.str, send_msg.len}, false, (EmDeviceDataType)NULL};
				  	SendDataRequest sendDatReq = {&basReq, {uart_msg, uart_msg_len}, false, (EmDeviceDataType)NULL};
					length = epb_send_data_request_pack_size(&sendDatReq) + fix_head_len;;
				  #else
					BlueDemoHead  *bleDemoHead = (BlueDemoHead*)malloc(bleDemoHeadLen+send_msg.len);
					if(!bleDemoHead){QPRINTF("1Not enough memory!\r\n");}
					bleDemoHead->m_magicCode[0] = MPBLEDEMO2_MAGICCODE_H;
					bleDemoHead->m_magicCode[1] = MPBLEDEMO2_MAGICCODE_L;
					bleDemoHead->m_version = htons( MPBLEDEMO2_VERSION);
					bleDemoHead->m_totalLength = htons(bleDemoHeadLen+send_msg.len);
					bleDemoHead->m_cmdid = htons(sendTextReq);
					bleDemoHead->m_seq = htons(wxSeq);
					QPRINTF("\r\nwxseq1%d\r\n",wxSeq); //nicole test
					bleDemoHead->m_errorCode = 0;	
					/*connect body and head.*/
					/*turn to uint8_t* befort offset.*/
					memcpy((uint8_t*)bleDemoHead+bleDemoHeadLen, send_msg.str, send_msg.len);			
					SendDataRequest sendDatReq = {&basReq, {(uint8_t*) bleDemoHead, (bleDemoHeadLen+send_msg.len)}, false, (EmDeviceDataType)NULL};
					length = epb_send_data_request_pack_size(&sendDatReq) + fix_head_len;
				#endif

					#if defined WX_MD5ENC
				  uint16_t len = length;
				  uint8_t *p = malloc(AES_get_length( length-fix_head_len));
				  if(!p){QPRINTF("\r\nNot enough memory!");}
				  length = AES_get_length( length-fix_head_len)+fix_head_len;
			    		#endif
				
					data = (uint8_t *)malloc(length);
					if(!(data)){QPRINTF("2Not enough memory!\r\n");}
					if(epb_pack_send_data_request(&sendDatReq, data+fix_head_len, length-fix_head_len)<0)
					{
						data = NULL;	
            				#if defined WX_MD5ENC
					  if(p){free(p);
					  p = NULL;}
					  #endif						
						QPRINTF("epb_pack_send_data_request error!\r\n");						
					}	
          
					#if defined WX_MD5ENC
					//encrypt body
					AES_Init(session_key);
					AES_Encrypt_PKCS7(data+fix_head_len,p,len-fix_head_len,session_key);//the original data length
					memcpy(data + fix_head_len, p, length-fix_head_len);
					if(p){free(p); p = NULL;}
				  #endif
					fix_head.nCmdId = htons(ECI_req_sendData);
					fix_head.nLength = htons(length);
					fix_head.nSeq = htons(wxSeq);
					memcpy(data, &fix_head, fix_head_len);
				//	if(bleDemoHead){free(bleDemoHead);bleDemoHead = NULL;}
				//	QPRINTF("sendata req:%d\r\n",length);
					wxps_send_data(data,length);	
				#ifdef FIXED_BLE_WECHAT_HEADER
					memset(uart_msg, 0x0, sizeof(uart_msg));
					uart_msg_len = 0;
				#else
					if(bleDemoHead) free(bleDemoHead);
				#endif
				  #if 0
					QPRINTF("##send data:\r\n");
					uint8_t *d = data;
					for(uint8_t i=0;i<length;++i){
					QPRINTF(" %x",d[i]);}
					QPRINTF("\r\n");	
					BpFixHead *fix_head = (BpFixHead *)data;
					QPRINTF("\r\n CMDID: %d",ntohs(fix_head->nCmdId));
					QPRINTF("\r\n len: %d", ntohs(fix_head->nLength ));
					QPRINTF("\r\n Seq: %d", ntohs(fix_head->nSeq));
					#endif				
				
		}
			break;		
		default:
			break;
	}
	 if((data)) 
	 free(data);
   data = NULL;	
    
	 
	return (KE_MSG_CONSUMED);
}

int app_wxps_wx2dev_proc_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
#if 1
	BpFixHead fix_head = ((struct BpFixHeadExt *)param)->BpFixHeader;
	uint8_t *data = (uint8_t *)param;
	uint16_t len = fix_head.nLength;
	uint8_t fix_head_len = sizeof(BpFixHead);
	
	switch(msgid)
	{
		
		case WXPS_AUTH_RESP_TIMER:
				{
				//	QPRINTF("fHead:0x%x,0x%x,0x%x,0x%x,0x%x\r\n",fix_head.bMagicNumber,
				//	                        fix_head.bVer,fix_head.nLength,fix_head.nCmdId,fix_head.nSeq);
				//	for(uint16_t j=0;j<fix_head.nLength;j++)
				//	QPRINTF("0x%x,",data[j]);
				//	QPRINTF("\r\n");
					AuthResponse* authResp;
					authResp = epb_unpack_auth_response(data+fix_head_len,len-fix_head_len);
				
					if(!authResp)
           				{
					   QPRINTF("unpack auth resp error\r\n");					  
					 }
				
					if(authResp->base_response)
					{
						if(authResp->base_response->err_code == 0)
						{
							// init req							
							ke_timer_set(WXPS_INIT_REQ_TIMER,TASK_APP, 6);
						}
						else
						{						
							//epb_unpack_auth_response_free(authResp);
							QPRINTF("unpack authResp error:%d\r\n",authResp->base_response->err_code);						
						}
					}
					
					#if defined WX_MD5ENC// get sessionkey
					if(authResp->aes_session_key.len)
					{
					#if 0 //def CATCH_LOG
						QPRINTF("\r\nsession_key:");
					#endif
						AES_Init(key);
						AES_Decrypt(session_key,authResp->aes_session_key.data,authResp->aes_session_key.len,key);
						#if 0 //def CATCH_LOG
						for(uint8_t i = 0;i<16;i++)
						{
							QPRINTF(" 0x%02x",session_key[i]);	
						}
						#endif
					}
				  #endif
					
					epb_unpack_auth_response_free(authResp);
				}
			
			break;
		
		case WXPS_INIT_RESP_TIMER:
			  {
			
				QPRINTF("Received 'initResp'\r\n");	

        #if defined WX_MD5ENC		
				uint32_t length = len- fix_head_len;//data length after encrypt
				uint8_t *p = malloc (length);
				if(!p){QPRINTF("\r\nNot enough memory!");if(data)free(data);data = NULL;return 0;}
				AES_Init(session_key);
				//解密数据
				AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);
				
				uint8_t temp;
				temp = p[length - 1];//填充长度
				len = len - temp;//加密前数据总长度
				memcpy(data + fix_head_len, p ,length -temp);//放回明文
				if(p){free(p);p = NULL;}
			  #endif							
			
				InitResponse *initResp = epb_unpack_init_response(data+fix_head_len, len-fix_head_len);
				if(!initResp)
				{
					QPRINTF("errorCodeUnpackInitResp\r\n");					
				}
				
					QPRINTF("unpack 'initResp' success!\r\n");
				
					if(initResp->base_response)
					{
						if(initResp->base_response->err_code == 0)
						{
							if(initResp->has_challeange_answer)
							{
								if(crc32(0,challeange,CHALLENAGE_LENGTH) == initResp->challeange_answer)
								{
									QPRINTF("1mpbledemo2Sta.init_state = true!\r\n");									
								}
							}
							else 
								QPRINTF("2mpbledemo2Sta.init_state = true!\r\n");								
							QPRINTF("mpbledemo2Sta.wechats_switch_state = true!\r\n");							
						}
						else
						{
						
							QPRINTF("errCode:%d\r\n",initResp->base_response->err_code);
						
							if(initResp->base_response->has_err_msg)
							{
							
								QPRINTF("baseResp errMsg:%s\r\n",initResp->base_response->err_msg.str);
								
							}							
						}
					}
				epb_unpack_init_response_free(initResp);
				ke_timer_set(WXPS_SDATA_REQ_TIMER,TASK_APP,100);
			}
			break;
		
		case WXPS_SDATA_RESP_TIMER:
			{		
			#ifdef DEBUG_MSG_ENABLE	
				QPRINTF("@@Received 'sendDataResp'\r\n");		
        		#endif
        		#if defined WX_MD5ENC		
				uint32_t length = len- fix_head_len;//the data length after encrypt
				uint8_t *p = malloc (length);
				if(!p){QPRINTF("\r\nNot enough memory!"); if(data)free(data);data = NULL;return 0;}
				AES_Init(session_key);
				
				//decrypt data
				AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);
				
				uint8_t temp;
				temp = p[length - 1];//计算填充长度
				len = len - temp;//取加密前数据总长度
				memcpy(data + fix_head_len, p ,length -temp);//放回明文
				if(p){free(p);p = NULL;}
			  #endif	        
				
				SendDataResponse *sendDataResp;
				sendDataResp = epb_unpack_send_data_response(data+fix_head_len,len-fix_head_len);
		
				if(!sendDataResp)
				{
					QPRINTF("errCode:%d\r\n",errorCodeUnpackSendDataResp);						
				}
			#ifdef FIXED_BLE_WECHAT_HEADER
				BpFixHead* bledemohead = (BpFixHead*)(sendDataResp->data.data);
				if(ntohs(bledemohead->nCmdId) == sendTextResp)
				{
					printf("received msg: %s\r\n",sendDataResp->data.data+sizeof(BpFixHead));
				}
			#else
				int i=0;
				BlueDemoHead *bledemohead = (BlueDemoHead*)(sendDataResp->data.data);
				printf("received msg 0x%x:\r\n",sendDataResp->data.len);

				for(i=0;i<sendDataResp->data.len;i++)
				{
					printf("%x ",sendDataResp->data.data[i]);
				}
				printf("\r\n");
			#endif
				if(sendDataResp->base_response->err_code)
				{						
					QPRINTF("baseResp err:%d\r\n",sendDataResp->base_response->err_code);						
				}
				epb_unpack_send_data_response_free(sendDataResp);
				//QPRINTF("sdata over cmdid=%d\r\n",ntohs(bledemohead->m_cmdid));//nicole	         				
			}
			break;
		case WXPS_RDATA_PUSH_TIMER:
			{
				
			#if defined WX_MD5ENC
			uint32_t length = len- fix_head_len;//加密后数据长度
			uint8_t *p = malloc (length);
			if(!p){QPRINTF("\r\nNot enough memory!");if(data)free(data); data =NULL; return 0;}
			AES_Init(session_key);
			//解密数据
			AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);
			
			uint8_t temp;
			temp = p[length - 1];//填充长度
			len = len - temp;//加密前数据总长度
			memcpy(data + fix_head_len, p ,length -temp);//放回明文
			if(p){free(p);p = NULL;}
		  	#endif
				
			RecvDataPush *recvDatPush;
			recvDatPush = epb_unpack_recv_data_push(data+fix_head_len, len-fix_head_len);
			#ifdef DEBUG_MSG_ENABLE
			QPRINTF("@@Received 'recvDataPush'\r\n");
			#endif
			if(!recvDatPush)
			{
				QPRINTF("errorCodeUnpackRecvDataPush\r\n");				
			}
			#ifdef DEBUG_MSG_ENABLE
			QPRINTF("unpack the 'recvDataPush' successfully! \r\n");
			if(recvDatPush->base_push == NULL)
			{
				QPRINTF("recvDatPush->base_push is NULL! \r\n");
			}
			else 
			{
				QPRINTF("recvDatPush->base_push is not NULL! \r\n");
			}
			#endif

			const uint8_t *d = recvDatPush->data.data;
			#if 0//
			#ifdef FIXED_BLE_WECHAT_HEADER
			QPRINTF("\r\n recvDatPush->data.len: %d \r\n",(recvDatPush->data.len));
			for(uint8_t i=0;i<recvDatPush->data.len;i++)
			 QPRINTF(" %x",d[i]);//nicole
			#else
			QPRINTF("\r\n recvDatPush->data.len: %d \r\n",(recvDatPush->data.len)-15);
			QPRINTF("\r\n recvDatPush->data.data:  \r\n");
			for(uint8_t i=0;i<recvDatPush->data.len;++i){
			 QPRINTF(" %x",d[i]);} //nicole
			 #endif
			 #endif
			//*********************************************************
			 			QPRINTF("\r\n recvDatPush->data.len: %d \r\n",(recvDatPush->data.len));
			for(uint8_t i=0;i<recvDatPush->data.len;i++)
			 QPRINTF(" %x",d[i]);//nicole
			//by nicole 20151214
			#ifdef UART0_QPPS_ENABLE
			msg_source = SRC_MESSAGE_WECHAT;
			if(recvDatPush->data.len > 15)
			{
				if(strncmp(recvDatPush->data.data, "the content is:", 15) == 0)	
					uart_write(QN_DEBUG_UART, (uint8_t *)d+15,(recvDatPush->data.len)-15, NULL);//(15 is the size of  <the content is:>)ericyang 20160204
			}
			else
			{
				uart_write(QN_DEBUG_UART, (uint8_t *)d, recvDatPush->data.len, NULL);//ericyang 20160204
			}
			#else
			msg_source = SRC_MESSAGE_WECHAT;
			QPRINTF("RX Data:\r\n");//by nicole test
			QPRINTF("%s\r\n",&d[15]);   //by nicole test
			#endif
			if((recvDatPush->data.len) == 0x10)
			{
				if(d[15] == 0x31)
				{
				    led_set(6,LED_ON);
					led_set(7,LED_OFF);
				}
				else if(d[15] == 0x32)  
				{
				    led_set(6,LED_OFF);
					led_set(7,LED_ON);
				}
				else if(d[15] == 0x33)  
				{
				    led_set(6,LED_ON);
					led_set(7,LED_ON);
				}
				else if(d[15] == 0x30)
				{
				    led_set(6,LED_OFF);
					led_set(7,LED_OFF);
				}
			}
					
			/*else if((recvDatPush->data.len) == 0x13)
			{
				if((d[0] == 0x6f) && (d[1] == 0x6e))
				{
				    led_set(6,LED_ON);
					led_set(7,LED_ON);
				}

			}
			else if((recvDatPush->data.len) == 0x03)
			{
				if((d[0] == 0x6f) && (d[1] == 0x66))
				{
				    led_set(6,LED_OFF);
					led_set(7,LED_OFF);
				}

			}*/
			else if((recvDatPush->data.len) == 0x13)
			{
				if(d[18] == 0x31)
				{
				   // led_set(6,LED_ON);
					led_set(1,LED_ON);// wenxue
				}
				if(d[18] == 0x30)
				{
					// led_set(6,LED_OFF);
					led_set(1,LED_OFF); // wenxue
				}
				if(d[18] == 0x33)
				{
				   // led_set(7,LED_ON);
					led_set(2,LED_ON); // wenxue
				}
				if(d[18] == 0x32)
				{
				//	 led_set(7,LED_OFF);
					led_set(2,LED_OFF); // wenxue
				}
			}


			//by nicole 20151214
			//*********************************************************
			if(recvDatPush->has_type)
			{
				QPRINTF("\r\n recvDatPush has type! \r\n");
				QPRINTF("\r\n type: %d\r\n",recvDatPush->type);
			}

		#ifdef FIXED_BLE_WECHAT_HEADER
			BpFixHead *bledemohead = (BpFixHead*)recvDatPush->data.data;
		#else
			BlueDemoHead *bledemohead = (BlueDemoHead*)recvDatPush->data.data;
		#endif
		#if 0 //def CATCH_LOG
			QPRINTF("\r\n magicCode: %x",bledemohead->m_magicCode[0]);
			QPRINTF(" %x",bledemohead->m_magicCode[1]);
			QPRINTF("\r\n version: %x",ntohs(bledemohead->m_version));
			QPRINTF("\r\n totalLength: %x",ntohs(bledemohead->m_totalLength));
			QPRINTF("\r\n cmdid: %x",ntohs(bledemohead->m_cmdid ));
			QPRINTF("\r\n errorCode: %x",ntohs(bledemohead->m_errorCode));
		#endif	
			//QPRINTF("\r\n ntohs(bledemohead->m_cmdid  %x \r\n",ntohs(bledemohead->m_cmdid)); //by nicole in 20160106
		#ifdef FIXED_BLE_WECHAT_HEADER
			if(ntohs(bledemohead->nCmdId) == openLightPush)
		#else
			if(ntohs(bledemohead->m_cmdid ) == openLightPush)
		#endif
			{
		
				QPRINTF("\r\n light on!! ");					
				//light_on(MPBLEDEMO2_LIGHT);			
			}
		#ifdef FIXED_BLE_WECHAT_HEADER
			else if(ntohs(bledemohead->nCmdId)  == closeLightPush)
		#else
			else if(ntohs(bledemohead->m_cmdid )  == closeLightPush)
		#endif
			{
		
					QPRINTF("\r\n light off!! ");			
					//light_off(MPBLEDEMO2_LIGHT);
					
			}
			epb_unpack_recv_data_push_free(recvDatPush);	

		//	ke_timer_set(WXPS_SDATA_REQ_TIMER,TASK_APP,100);  //ericyang test!!!!

			
		}
			
			break;
		case WXPS_SVIEW_PUSH_TIMER:
			{			
				QPRINTF("\r\n@@Received 'switchViewPush'\r\n");			
			  
				#if defined WX_MD5ENC
				uint32_t length = len- fix_head_len;//加密后数据长度
				uint8_t *p = malloc (length);
				if(!p){QPRINTF("\r\nNot enough memory!");if(data)free(data); data =NULL; return 0;}
				AES_Init(session_key);
				//解密数据
				AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);
				
				uint8_t temp;
				temp = p[length - 1];//填充长度
				len = len - temp;//加密前数据总长度
				memcpy(data + fix_head_len, p ,length -temp);//放回明文
				if(p){free(p);p = NULL;}
				#endif
				
				SwitchViewPush *swichViewPush;
				swichViewPush = epb_unpack_switch_view_push(data+fix_head_len,len-fix_head_len);
				if(!swichViewPush)
				{
					QPRINTF("errorCodeUnpackSwitchViewPush\r\n");					
				}
				epb_unpack_switch_view_push_free(swichViewPush);
			}
			break;
		case WXPS_SBG_PUSH_TIMER:
			{
			
				QPRINTF("\r\n@@Received 'switchBackgroudPush'\r\n");	
        
        #if defined WX_MD5ENC
				uint32_t length = len- fix_head_len;//加密后数据长度
				uint8_t *p = malloc (length);
				if(!p){QPRINTF("\r\nNot enough memory!");if(data)free(data); data =NULL; return 0;}
				AES_Init(session_key);
				//解密数据
				AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);
				
				uint8_t temp;
				temp = p[length - 1];//填充长度
				len = len - temp;//加密前数据总长度
				memcpy(data + fix_head_len, p ,length -temp);//放回明文
				if(p){free(p);p = NULL;}
				#endif
        				
				SwitchBackgroudPush *switchBackgroundPush = epb_unpack_switch_backgroud_push(data+fix_head_len,len-fix_head_len);
				if(! switchBackgroundPush)
				{
					QPRINTF("errorCodeUnpackSwitchBackgroundPush\r\n");						
				}	
				epb_unpack_switch_backgroud_push_free(switchBackgroundPush);
			}
			break;
		default:
			break;
	}	
	
	return (KE_MSG_CONSUMED);
#endif
}

/// @} USR

