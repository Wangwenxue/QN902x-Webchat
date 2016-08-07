/**
 ****************************************************************************************
 *
 * @file usr_design.h
 *
 * @brief Product related design header file.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef USR_DESIGN_H_
#define USR_DESIGN_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"
#include "gpio.h"

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */
 
 //about weixin encrypt: encrypt and no encrypt,
 // no encrypt has two ways:MD5 and MAC


//#define WX_MD5ENC        
//#define WX_MD5NOENC    
#define WX_MACNOENC    

#define MAC_ADDRESS_LENGTH      6

#define DEVICE_TYPE "WeChatDev" //"gh_1bafe245c2cb"
#define DEVICE_ID "code008"//"WeChatBluetoothDevice"
#define PROTO_VERSION 0x010000//0x010004
#define AUTH_PROTO 1



#ifdef WX_MACNOENC
	#define AUTH_METHOD EAM_macNoEncrypt	
#endif

#ifdef WX_MD5ENC
#define DEVICE_KEY {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};
#define MD5_TYPE_AND_ID_LENGTH 16
#define CIPHER_TEXT_LENGTH 16
#define AUTH_METHOD EAM_md5
#endif

#ifdef WX_MD5NOENC
#define AUTH_METHOD EAM_md5
#define MD5_TYPE_AND_ID_LENGTH 16
#define CIPHER_TEXT_LENGTH 0
#endif

#define CHALLENAGE_LENGTH 4

#define MPBLEDEMO2_MAGICCODE_H 0xfe
#define MPBLEDEMO2_MAGICCODE_L 0xcf
#define MPBLEDEMO2_VERSION 0x01
#define SEND_HELLO_WECHAT "Hello, WeChat!"

typedef enum
{
	errorCodeUnpackAuthResp = 0x9990,
	errorCodeUnpackInitResp = 0x9991,
	errorCodeUnpackSendDataResp = 0x9992,
	errorCodeUnpackCtlCmdResp = 0x9993,
	errorCodeUnpackRecvDataPush = 0x9994,
	errorCodeUnpackSwitchViewPush = 0x9995,
	errorCodeUnpackSwitchBackgroundPush = 0x9996,
	errorCodeUnpackErrorDecode = 0x9997,
}mpbledemo2UnpackErrorCode;


typedef enum
{
	sendTextReq = 0x01,
	sendTextResp = 0x1001,
	openLightPush = 0x2001,
	closeLightPush = 0x2002,
}BleDemo2CmdID;

typedef struct
{
	uint8_t m_magicCode[2];
	uint16_t m_version;
	uint16_t m_totalLength;
	uint16_t m_cmdid;
	uint16_t m_seq;
	uint16_t m_errorCode;
}BlueDemoHead;

typedef struct
{
	BlueDemoHead BlueDemoHeader;
	uint8_t data[1];
}BlueDemoHeadExt;

typedef struct
{
		uint8_t bMagicNumber;
		uint8_t bVer;
		uint16_t nLength;
		uint16_t nCmdId;
		uint16_t nSeq;
} BpFixHead;

struct BpFixHeadExt
{
		BpFixHead BpFixHeader;
	  uint8_t data[128];
};


struct usr_env_tag
{
    uint16_t    led1_on_dur;
    uint16_t    led1_off_dur;
};

extern struct usr_env_tag usr_env;

struct wxs_send_ind
{
	uint16_t len;
	uint8_t data[128];
};

//uint8_t flagSendData=0; //nicole

static uint16_t wxSeq = 0;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
// wenxue
extern void wxps_send_data(uint8_t* datum,uint16_t length);

extern void app_task_msg_hdl(ke_msg_id_t const msgid, void const *param);
extern int app_led_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int app_gap_adv_intv_update_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern void usr_sleep_restore(void);
extern void usr_button1_cb(void);
extern int app_button_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern void usr_init(void);
extern void gpio_interrupt_callback(enum gpio_pin pin);
extern int app_wxps_send_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int app_wxps_wx2dev_proc_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int app_wxps_dev2wx_proc_timer_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

#endif
