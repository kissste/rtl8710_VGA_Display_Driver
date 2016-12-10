/******************************************************************************
 *
 * FileName: user_main.c
 *
 *******************************************************************************/
#include "rtl8195a/rtl_common.h"
#include "rtl8195a.h"
#include "hal_log_uart.h"

#include "FreeRTOS.h"
#include "task.h"
//#include "diag.h"
#include "osdep_service.h"
#include "device_lock.h"
#include "semphr.h"
#include "queue.h"

#include <wifi/wifi_conf.h>
#include <wifi/wifi_util.h>

#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "dhcp/dhcps.h"

#include "user/atcmd_user.h"
#include "main.h"

#define DEBUG_MAIN_LEVEL 1

#define PRIO_MAD (tskIDLE_PRIORITY + 1 + PRIORITIE_OFFSET)
#define PRIO_READER (PRIO_MAD + 7) // max 11 ?

volatile char tskmad_enable, tskreader_enable;

void tskreader(void *pvParameters);

void tskmad(void *pvParameters) {
	vTaskDelete(NULL);
}

void tskreader(void *pvParameters) {
	vTaskDelete(NULL); 
}

void connect_close(void) {
	DBG_8195A("connect_close\n");
	if (tskreader_enable == 1) {
		tskreader_enable = 0;
		while(tskreader_enable == 0) vTaskDelay(2);
		tskreader_enable = 0;
	}
}

void connect_start(void) {
	DBG_8195A("connect_start\n");
	//connect_close();
}


/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */

void main(void) {
#if DEBUG_MAIN_LEVEL > 2
	 ConfigDebugErr  = -1;
	 ConfigDebugInfo = -1;
	 ConfigDebugWarn = -1;
#endif
#if DEBUG_MAIN_LEVEL > 3
	 ConfigDebugErr  = -1;
	 ConfigDebugInfo = ~_DBG_SPI_FLASH_;
	 ConfigDebugWarn = -1;
	 CfgSysDebugErr = -1;
	 CfgSysDebugInfo = -1;
	 CfgSysDebugWarn = -1;
#endif
/*
	 if ( rtl_cryptoEngine_init() != 0 ) DBG_8195A("crypto engine init failed\r\n");
*/
#if defined(CONFIG_CPU_CLK)
		HalCpuClkConfig(CPU_CLOCK_SEL_VALUE); // 0 - 166666666 Hz, 1 - 83333333 Hz, 2 - 41666666 Hz, 3 - 20833333 Hz, 4 - 10416666 Hz, 5 - 4000000 Hz
		HAL_LOG_UART_ADAPTER pUartAdapter;
		pUartAdapter.BaudRate = RUART_BAUD_RATE_38400;
		HalLogUartSetBaudRate(&pUartAdapter);
		SystemCoreClockUpdate();
		En32KCalibration();
#endif
#if DEBUG_MAIN_LEVEL > 1
	DBG_INFO_MSG_ON(_DBG_TCM_HEAP_); // On Debug TCM MEM
#endif
#if DEBUG_MAIN_LEVEL > 0
	vPortFree(pvPortMalloc(4)); // Init RAM heap 
	fATST(NULL); // RAM/TCM/Heaps info
#endif
	/* pre-processor of application example */
	pre_example_entry();

	/* wlan intialization */
#if defined(CONFIG_WIFI_NORMAL) && defined(CONFIG_NETWORK)
	//wlan_network();
#endif
	/* Initialize log uart and at command service */
	console_init();

	/* Execute application example */
	example_entry();

	/*Enable Schedule, Start Kernel*/
#if defined(CONFIG_KERNEL) && !TASK_SCHEDULER_DISABLED
#ifdef PLATFORM_FREERTOS
	vTaskStartScheduler();
#endif
#else
	RtlConsolTaskRom(NULL);
#endif
}
