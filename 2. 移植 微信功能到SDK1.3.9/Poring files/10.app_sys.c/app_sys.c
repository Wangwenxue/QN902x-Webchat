/**
 ****************************************************************************************
 *
 * @file app_sys.c
 *
 * @brief Application System Functions
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h" 
#include "uart.h"

#if QN_DEMO_MENU
struct app_uart_env_tag app_uart_env;
static void app_uart_rx_done(void);
#endif

/**
 ****************************************************************************************
 * @brief Uart initialization.
 *
 ****************************************************************************************
 */
void app_uart_init(void)
{
#if QN_DEMO_MENU
	//printf("lili\r\n");  //nicole

    app_uart_env.len = 0;
    uart_read(QN_DEBUG_UART, app_uart_env.buf_rx, 1, app_uart_rx_done);
#endif
}

#if QN_DEMO_MENU
/**
 ****************************************************************************************
 * @brief UART receive call back function, input string should end with '\r''\n'.
 *
 ****************************************************************************************
 */
#ifdef UART0_QPPS_ENABLE
 void app_uart_rx_done(void)
{
//	if (app_uart_env.buf_rx[app_uart_env.len] == 0x0A)

	if((app_uart_env.len >= QN_UART_RX_LEN - 1) || (app_uart_env.buf_rx[app_uart_env.len] == 0x0A))
	{
#if 1
		struct app_uart_data_req *req = ke_msg_alloc(APP_SYS_UART_DATA_IND,
	                                    TASK_APP,
	                                    TASK_NONE,
	                                    sizeof(struct app_uart_data_req) + (app_uart_env.len - 1));

	      //app_uart_env.buf_rx[app_uart_env.len-1] = '\0';//ericyang 20160309 Wechat can't use '\0' symbol
			
		req->len = app_uart_env.len;
		memcpy(req->data, app_uart_env.buf_rx, app_uart_env.len);
		ke_msg_send(req);
#else
	 	uart_write(QN_DEBUG_UART,  app_uart_env.buf_rx, app_uart_env.len, NULL);//ericyang 20160214 for test uart debug log
#endif
		app_uart_env.len = 0;
	}
	else
	{
		app_uart_env.len += 1;
	}	
	uart_read(QN_DEBUG_UART, app_uart_env.buf_rx+app_uart_env.len, 1, app_uart_rx_done);
} 

#else
void app_uart_rx_done(void)
{

	//printf("kiki\r\n");  //nicole

    if (app_uart_env.buf_rx[app_uart_env.len] == 0x0A)
    {
        struct app_uart_data_req *req = ke_msg_alloc(APP_SYS_UART_DATA_IND,
                                                    TASK_APP,
                                                    TASK_NONE,
                                                    sizeof(struct app_uart_data_req) + (app_uart_env.len - 1));
		//printf("mimi\r\n");  //nicole
		app_uart_env.buf_rx[app_uart_env.len-1] = '\0';
        req->len = app_uart_env.len;
        memcpy(req->data, app_uart_env.buf_rx, app_uart_env.len);
        ke_msg_send(req);

 	uart_write(QN_DEBUG_UART,  app_uart_env.buf_rx, app_uart_env.len, NULL);//ericyang 20160214 for test uart debug log

		
        app_uart_env.len = 0;
    }
    else
    {
        if (app_uart_env.len == QN_UART_RX_LEN-1)
            app_uart_env.len = 0;
        else
            app_uart_env.len += 1;
    }

    uart_read(QN_DEBUG_UART, app_uart_env.buf_rx+app_uart_env.len, 1, app_uart_rx_done);
}
#endif
#endif
