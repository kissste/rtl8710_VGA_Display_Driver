#include <platform_opts.h>

#ifdef CONFIG_AT_USR

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "at_cmd/log_service.h"
#include "at_cmd/atcmd_wifi.h"
#include <lwip_netconf.h>
#include "tcpip.h"
#include <dhcp/dhcps.h>
#include <wifi/wifi_conf.h>
#include <wifi/wifi_util.h>
#include "tcm_heap.h"
#include "user/atcmd_user.h"
#include "user/playerconfig.h"

rtw_mode_t wifi_mode = RTW_MODE_STA;
mp3_server_setings mp3_serv = {0,{0}}; //{ PLAY_PORT, { PLAY_SERVER }};

#define DEBUG_AT_USER_LEVEL 1

#include "rtl8195a.h"
#include "rtl8195a_sdio_host.h"
#include "hal_sdio_host.h"
#include "sd.h"
#include "sdio_host.h"
#include "PinNames.h"

#include <spi_api.h>
#include <spi_ex_api.h>
#include "objects.h"
#include "port_api.h"
#include "pinmap.h"
#include "gpio_api.h"
#include "PinNames.h"
#include "PortNames.h"
  
#define CPU_CLOCK_SEL_VALUE 0

// SPI0 (S0)
#define SPI0C_MOSI	PC_2
#define SPI0C_MISO	PC_3
#define SPI0C_SCLK	PC_1
#define SPI0C_CS	PC_0

// SPI2 (S2) for DEV 3V0
// Please note that PA_7 need pull high before using GPIOA group
#define SPI1A_MOSI  PA_1
#define SPI1A_MISO  PA_0
#define SPI1A_SCLK  PA_2
#define SPI1A_CS    PA_4

extern const u8 GPIO_SWPORT_DR_TBL[];
extern const u8 GPIO_SWPORT_DDR_TBL[];

gpio_t gpio_ledA5; //VSynch

//SVGA Resolution 800x600@63Hz
//http://tinyvga.com/vga-timing/800x600@60Hz
#define SSI_ROWS_BLOCK	100 //submit 12 rows in one DMA
#define SSI_VIDEO_A 	800 //800 columns (bits)
#define SSI_VIDEO_A_BYTES	100	//SSI_VIDEO_A/8 //800/8 bytes = 100 bytes
#define SSI_VIDEO_BLANK 256 //256 blanking columns(bits)
#define SSI_VIDEO_BLANK_BYTES	32 //SSI_VIDEO_BLANK/8 //256/8 = 32 bytes
#define SSI_LINE_A  	132 //800/8 + 32 blanking bytes = 132 bytes = 1056 bits
#define SSI_ROWS_A  	600 //600 rows
#define SSI_MAX_LEN_A  	SSI_LINE_A*SSI_ROWS_A //The full screen, including H Blanking
#define SSI_FREQ_C_DIV	4
#define SSI_LINE_C  	SSI_LINE_A/SSI_FREQ_C_DIV //132/4 = 33 bytes
#define SSI_HSYNC_SHIFT	14*8 //delayed HSYNC in SSI C Line bits

#if 0
//SVGA Resolution 640x480@97Hz - not tested
//http://tinyvga.com/vga-timing/640x480@100Hz
#define SSI_ROWS_BLOCK	100 //submit 12 rows in one DMA
#define SSI_VIDEO_A 	640 //640 columns (bits)
#define SSI_VIDEO_A_BYTES	80 //SSI_VIDEO_A/8 //640/8 bytes = 80 bytes
#define SSI_VIDEO_BLANK	208 //208 blanking columns(bits)
#define SSI_VIDEO_BLANK_BYTES	26 //SSI_VIDEO_BLANK/8 //208/8 = 26 bytes
#define SSI_LINE_A  	96 //SSI_VIDEO_A_BYTES + SSI_VIDEO_BLANK_BYTES //640/8 + 26 bytes = 106 bytes = 848 bits
#define SSI_ROWS_A  	480 //480 rows
#define SSI_MAX_LEN_A  	SSI_LINE_A*SSI_ROWS_A //*480 //106 bytes = 848 bits
#define SSI_FREQ_C_DIV	2
#define SSI_LINE_C  	SSI_LINE_A/SSI_FREQ_C_DIV //106/2 = 53 bytes
#define SSI_HSYNC_SHIFT	14*8 //delayed HSYNC in SSI C Line bits
#endif

#define SCLK_FREQ_A 		200000000*5/6/4 // 41.66MHz CPU@166.66MHz @SPI1 ssi_idx=1 A-port
#define SCLK_FREQ_C 		200000000*5/6/4/SSI_FREQ_C_DIV // 325.521kHz CPU@166.66MHz @SPI1 ssi_idx=1 C-port

// Spi Masters
spi_t spi_masterA;
spi_t spi_masterC;

volatile uint32_t lineV;

// DMA buffers
uint8_t *dma_bufferA;
uint8_t *dma_bufferC;
uint8_t *dma_bufferA_blank;

// Simplifier version of spi_master_write_stream_dma from SDK. The checking is taking too long
void spi_master_write_stream_dma2(spi_t *obj, char *tx_buffer, uint32_t length)
{
    PHAL_SSI_ADAPTOR pHalSsiAdaptor;
	PHAL_SSI_OP pHalSsiOp;
	
	pHalSsiAdaptor = &obj->spi_adp;
	pHalSsiOp = &obj->spi_op;

#if 0 //**** This was too slow to be execute every single time DMA is submitted, removed and put into spi_master_init_dma2 to be called separately***
	u32 abc = HalSsiGetTxFifoLevelRtl8195a(pHalSsiAdaptor);
	DBG_SSI_ERR("%d\n",abc);
	
    if ((obj->dma_en & SPI_DMA_TX_EN)==0) {
        if (HAL_OK == HalSsiTxGdmaInit(pHalSsiOp, pHalSsiAdaptor)) {
            obj->dma_en |= SPI_DMA_TX_EN;
        }
    }
#endif
 
    obj->state |= SPI_STATE_TX_BUSY;	
    HalSsiDmaSend(pHalSsiAdaptor, (u8 *) tx_buffer, length);
}

void spi_master_init_dma2(spi_t *obj)
{
    PHAL_SSI_ADAPTOR pHalSsiAdaptor;
	PHAL_SSI_OP pHalSsiOp;
	
	pHalSsiAdaptor = &obj->spi_adp;
	pHalSsiOp = &obj->spi_op;
	
    if ((obj->dma_en & SPI_DMA_TX_EN)==0) {
        if (HAL_OK == HalSsiTxGdmaInit(pHalSsiOp, pHalSsiAdaptor)) {
            obj->dma_en |= SPI_DMA_TX_EN;
        }
    }
}

#define HAL_WRITE32base(base, value32) ((*((volatile u32*)(base))) = rtk_cpu_to_le32(value32))
		
#define BITBAND_PERI_REF   0x40000000
#define BITBAND_PERI_BASE  0x42000000
#define BITBAND_PERI(a,b) ((BITBAND_PERI_BASE + (a-BITBAND_PERI_REF)*32 + (b*4)))  // Convert PERI address

#define P0_ADDR	0x40001000
#define GPIO_A0 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE,0))) 	//Port = 0, bit = 0, A0
#define GPIO_A1 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE,1))) 	//Port = 0, bit = 1, A1
#define GPIO_A2 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE+12,0))) //Port = 1, bit = 0, A2
#define GPIO_A3 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE+12,1))) //Port = 1, bit = 1, A3
#define GPIO_A4 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE+12,2))) //Port = 1, bit = 2, A4
#define GPIO_A5 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE+12,3))) //Port = 1, bit = 3, A5

#define GPIO_C0 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE+12,10))) //Port = 1, bit = 10, C0
#define GPIO_C1 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE,6))) 	 //Port = 0, bit = 6, C1
#define GPIO_C2 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE+12,11))) //Port = 1, bit = 11, C2
#define GPIO_C3 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE,7))) 	 //Port = 0, bit = 7, C3
#define GPIO_C4 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE,8))) 	 //Port = 0, bit = 8, C4
#define GPIO_C5 *((volatile unsigned char *)(BITBAND_PERI(GPIO_REG_BASE,9))) 	 //Port = 0, bit = 9, C5

void volatile __delay_xyz(uint8_t steps)
{
	for(volatile uint32_t r=0;r<steps;r++) {
		asm volatile (
			".align 4 \n\r"\
			"NOP		\n\r"\
			".align 4 \n\r"\
			"NOP		\n\r"\
			".align 4 \n\r"\
			"NOP		\n\r"\
			".align 4 \n\r"\
			"NOP		\n\r"\
			".align 4 \n\r"\
			"NOP		\n\r"\
			".align 4 \n\r"\
			"NOP		\n\r"\
			".align 4 \n\r"\
			"NOP		\n\r"\
			".align 4 \n\r"\
			"NOP		\n\r"			
		);
	}
}

/* wake up waiting tasks when DMA transfer is complete */
void volatile master_tr_done_callbackA(void *pdata, SpiIrq event)
{
    if (event == SpiTxIrq) {
		//UBaseType_t uxSavedInterruptStatus;
		//uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
		portDISABLE_INTERRUPTS();
		lineV++;
		if (lineV == SSI_ROWS_A/SSI_ROWS_BLOCK + 28) { 
			lineV = 0;
		}
		if (lineV < SSI_ROWS_A/SSI_ROWS_BLOCK) { // Video and H-Sync
			spi_master_write_stream_dma2(&spi_masterA, dma_bufferA+lineV*SSI_LINE_A*SSI_ROWS_BLOCK, SSI_LINE_A*SSI_ROWS_BLOCK); // Video
			spi_master_write_stream_dma2(&spi_masterC, dma_bufferC, SSI_LINE_C*SSI_ROWS_BLOCK); // H-Sync
		} else {
			spi_master_write_stream_dma2(&spi_masterA, dma_bufferA_blank, SSI_LINE_A); // Video Blank
			spi_master_write_stream_dma2(&spi_masterC, dma_bufferC, SSI_LINE_C); // H-Sync
			if(lineV == SSI_ROWS_A/SSI_ROWS_BLOCK + 1) { // V-Sync On
				//__delay_xyz(5);
				GPIO_A5 = 1;
			} else if(lineV == SSI_ROWS_A/SSI_ROWS_BLOCK + 5) { // V-Sync Off
				//__delay_xyz(5);
				GPIO_A5 = 0;
			}
		} 
		//taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
		portENABLE_INTERRUPTS();
	}
}

void populate_bufferA_blank() 
{
	memset(dma_bufferA_blank, 0x00, SSI_LINE_A);
}

void populate_bufferA() 
{
	for(u32 i=0; i<SSI_ROWS_A; i++) {
		memset(dma_bufferA+i*SSI_LINE_A, 0xFF, SSI_LINE_A);	//bytes
		memset(dma_bufferA+i*SSI_LINE_A+SSI_VIDEO_A_BYTES, 0x00, SSI_VIDEO_BLANK_BYTES);
	}
}


void populate_bufferC() 
{
	u32 n = SSI_HSYNC_SHIFT/8;
	for(u32 i=0; i<SSI_ROWS_BLOCK; i++) {
		memset(dma_bufferC+i*SSI_LINE_C, 0x0, SSI_LINE_C);
		//4x FF
		*(dma_bufferC+i*SSI_LINE_C+n) = 0x0F;
		*(dma_bufferC+i*SSI_LINE_C+n+1) = 0xFF;
		*(dma_bufferC+i*SSI_LINE_C+n+2) = 0xFF;
		*(dma_bufferC+i*SSI_LINE_C+n+3) = 0xFF;
		*(dma_bufferC+i*SSI_LINE_C+n+4) = 0xF0;
	}
}

void printPorts(u8 *str, u8 pin_in)
{
	uint32_t pin_name = HAL_GPIO_GetPinName(pin_in);
	uint8_t port_num = HAL_GPIO_GET_PORT_BY_NAME(pin_name);
	uint8_t pin_num = HAL_GPIO_GET_PIN_BY_NAME(pin_name);
	uint32_t addr_shiftDDR = GPIO_SWPORT_DDR_TBL[port_num];
	uint32_t addr_shiftDR = GPIO_SWPORT_DR_TBL[port_num];
	uint32_t chip_pin = GPIO_GetChipPinName_8195a(port_num, pin_num);
	
	printf("%s pin_name: %d, port_num: %d, pin_num: %d, addr_shiftDDR: %d, DR:%d, chip_pin: %d\n", str, pin_name, port_num, pin_num, addr_shiftDDR, addr_shiftDR, chip_pin);
}

int SSI_init(void)
{
	//printf("SSI_init started\n");
	fATST(NULL);

    int result = 0;
	
	// Init A5 and C4 control pin
    gpio_init(&gpio_ledA5, PA_5);
    gpio_dir(&gpio_ledA5, PIN_OUTPUT);    // Direction: Output
    gpio_mode(&gpio_ledA5, PullNone);     // PullNone
		
	// A SPI
	spi_init(&spi_masterA, SPI1A_MOSI, SPI1A_MISO, SPI1A_SCLK, SPI1A_CS);
    spi_format(&spi_masterA, 32, 3, 0); //32 bits, mode 3, master
    spi_frequency(&spi_masterA, SCLK_FREQ_A); //41.833MHz
    spi_irq_hook(&spi_masterA, master_tr_done_callbackA, NULL);
	
	// C SPI
	spi_init(&spi_masterC, SPI0C_MOSI, SPI0C_MISO, SPI0C_SCLK, SPI0C_CS);
    spi_format(&spi_masterC, 8, 3, 0); //8 bits, mode 3, master
    spi_frequency(&spi_masterC, SCLK_FREQ_C);
    spi_irq_hook(&spi_masterC, NULL, NULL);	
	
	// DMAs
	spi_master_init_dma2(&spi_masterA);
	spi_master_init_dma2(&spi_masterC);

	// A Buffer
	dma_bufferA = malloc(SSI_MAX_LEN_A);
    if (dma_bufferA != NULL) {
		populate_bufferA();
		lineV = 0;
	} else {
		printf("dma_bufferA can't be allocated\n");
		result = -1;
		goto err_out;
	}

	// A_Blank
	dma_bufferA_blank = malloc(SSI_LINE_A);
    if (dma_bufferA_blank != NULL) {	
		populate_bufferA_blank();
	} else {
		printf("dma_bufferA_blank can't be allocated\n");
		result = -1;
		goto err_out;
	}
	
	// C Buffer
	dma_bufferC = malloc(SSI_LINE_C*SSI_ROWS_BLOCK);
    if (dma_bufferC != NULL) {	
		populate_bufferC();
	} else {
		printf("dma_bufferC can't be allocated\n");
		result = -1;
		goto err_out;
	}
	
	//Don't want to create any gap between these 2 DMA channels (Video and HSYNC)
	taskENTER_CRITICAL();
	spi_master_write_stream_dma2(&spi_masterA, dma_bufferA, SSI_LINE_A*SSI_ROWS_BLOCK); //buff_len in bytes
	spi_master_write_stream_dma2(&spi_masterC, dma_bufferC, SSI_LINE_C*SSI_ROWS_BLOCK); //buff_len in bytes
	taskEXIT_CRITICAL();
		
err_out:
    if(result != 0){
		printf("SSI Init failed, freing malloc\n");
        if(dma_bufferA != NULL){
            free(dma_bufferA);
        }
        if(dma_bufferC != NULL){
            free(dma_bufferC);
        }
    }
    
    return result;
}


void pixel(u16 x, u16 y, u8 c) {
	//x [0:799]
	//y [0:599]
	//swap endian
	u16 b = x>>3;
	//printf("I:%d,%d,%d\n",x,y,b);
	if(b%2) b--; else b++;
	//find k
	u32 k = (u32)y*132+b;
	//determine mask
	u8 v = 0x80>>(x%8);
	//printf("O:%d,%d,%02X\n",b,k,v);
	if (c) {
		*(dma_bufferA+k) |= v;
	} else {
		
		*(dma_bufferA+k) &= ~v;
	}
}

/*    VGA  */
void fATVG(void *arg) {
#if 0
    ConfigDebugErr = -1;
    ConfigDebugInfo = -1;
    ConfigDebugWarn = -1;
    CfgSysDebugErr = -1;
    CfgSysDebugInfo = -1;
    CfgSysDebugWarn = -1;
#endif
	printf("ATVG started v0.01\n");
	//fATST(NULL);
	
    if(SSI_init() != 0){
        printf("[%s] SSI_init() failed\n", __func__);
        goto err_out;
    }

	printf("ATVG SSI_init done\n");
	
	vTaskDelay(1000);
	
#if 0	
	u8 shift = 0;
	u8 carry = 0;
    while(1) {
		vTaskDelay(500);
		shift++;
		//carry++;
		if(shift>7) shift = 0;
		for(u32 i=0; i<600; i++) {
			for(u32 j=0; j<100; j++) {
				u8 m = j%shift;
				carry++;
				//dma_bufferA[i*132+j] = 0x1<<m; //bytes
				*(dma_bufferA+i*132+j) = carry;
				//memset(&(dma_bufferA[i*132+100]), 0x00, 32);
			}
		}
    }
#endif	

#if 0
	u16 x[8] = {0,0,0,0,0,0,0,0};
	u16 y[8] = {0,0,0,0,0,0,0,0};
	while(1) {
		for(u8 j = 0; j<8; j++) {
			*(dma_bufferA + y[j]*132 + x[0]) = 0xFF;
			//printf("1:%d,%d,%d\n",j,y[j],x[j]);
		}
		vTaskDelay(1);
		for(u8 k = 0; k<8; k++) {
			*(dma_bufferA + y[k]*132 + x[0]) = 0x00;
			//printf("0:%d,%d,%d\n",k,y[k],x[k]);
		}
		if(++x[0] == 800/8) {
			x[0] = 0;
			if(++y[0] == 600) y[0] = 0;
				for(u8 i = 7; i>0; i--) {
					y[i] = y[i-1];
					//x[i] = x[i-1];
				}			
		}		
	}
#endif
	for(u16 y=0; y<600; y++) {
		pixel(1,y,0);
		pixel(11,y,0);
		pixel(21,y,0);
		pixel(798,y,0);
		pixel(798-10,y,0);
		pixel(798-20,y,0);
		//vTaskDelay(10);
		//printf("A:%d,%d\n",x,y);
	}
	for(u16 x=0; x<800; x++) {
		pixel(x,1,0);
		pixel(x,11,0);
		pixel(x,21,0);
		pixel(x,598,0);
		pixel(x,598-10,0);
		pixel(x,598-20,0);
	}
	
	u16 posx = 36;
	u16 posy = 36;
	const char TestText1[200] = "Bobo is the best!!!";
	TDFPutStr(posx, posy, &TestText1, 0, 1);
	posy += 26;
	const char TestText2[200] = "Pup is the better than Bobo, said Pup.";
	TDFPutStr(posx, posy, &TestText2, 0, 1);
	posy += 26;
	const char TestText3[200] = "Bobo is better than Pup no matter what!";
	TDFPutStr(posx, posy, &TestText3, 0, 1);
	posy += 26;
	const char TestText4[200] = "Pup cancels out Bobo";
	TDFPutStr(posx, posy, &TestText4, 0, 1);	
	posy += 26;
	const char TestText5[200] = "Miss Bobo says that Pup can't post messages";
	TDFPutStr(posx, posy, &TestText5, 0, 1);	
	posy += 26;	
	
err_out:
    //while(1) {
    //    vTaskDelay(1000);
    //}
	printf("ATVG finished\n");
	vTaskDelay(500);
}

/******************************************************************************/
/*
#define	_AT_WLAN_SET_SSID_          "ATW0"
#define	_AT_WLAN_SET_PASSPHRASE_    "ATW1"
#define	_AT_WLAN_SET_KEY_ID_        "ATW2"
#define	_AT_WLAN_JOIN_NET_          "ATWC"
#define	_AT_WLAN_SET_MP3_URL_       "ATWS"
*/
//extern struct netif xnetif[NET_IF_NUM];

/* fastconnect use wifi AT command. Not init_wifi_struct when log service disabled
 * static initialize all values for using fastconnect when log service disabled
 */
static rtw_network_info_t wifi = {
	{0},    // ssid
	{0},    // bssid
	0,      // security
	NULL,   // password
	0,      // password len
	-1      // key id
};

static rtw_ap_info_t ap = {0};
static unsigned char password[65] = {0};

_WEAK void connect_start(void)
{
}

_WEAK void connect_close(void)
{
}

static void init_wifi_struct(void)
{
	memset(wifi.ssid.val, 0, sizeof(wifi.ssid.val));
	memset(wifi.bssid.octet, 0, ETH_ALEN);	
	memset(password, 0, sizeof(password));
	wifi.ssid.len = 0;
	wifi.password = NULL;
	wifi.password_len = 0;
	wifi.key_id = -1;
	memset(ap.ssid.val, 0, sizeof(ap.ssid.val));
	ap.ssid.len = 0;
	ap.password = NULL;
	ap.password_len = 0;
	ap.channel = 1;
}

void fATW0(void *arg){
	if(!arg){
		printf("ATW0: Usage: ATW0=SSID\n");
		goto exit;
	}
#if	DEBUG_AT_USER_LEVEL > 1
	printf("ATW0: %s\n", (char*)arg);
#endif
	strcpy((char *)wifi.ssid.val, (char*)arg);
	wifi.ssid.len = strlen((char*)arg);
exit:
	return;
}

void fATW1(void *arg){
#if	DEBUG_AT_USER_LEVEL > 1
    printf("ATW1: %s\n", (char*)arg);
#endif
	strcpy((char *)password, (char*)arg);
	wifi.password = password;
	wifi.password_len = strlen((char*)arg);
	return;	
}

void fATW2(void *arg){
#if	DEBUG_AT_USER_LEVEL > 1
	printf("ATW2: %s\n", (char*)arg);
#endif
	if((strlen((const char *)arg) != 1 ) || (*(char*)arg <'0' ||*(char*)arg >'3')) {
		printf("ATW2: Wrong WEP key id. Must be one of 0,1,2, or 3.\n");
		return;
	}
	wifi.key_id = atoi((const char *)(arg));
	return;
}

// Test
void fATST(void *arg){
	extern u8 __HeapLimit, __StackTop;
	extern struct Heap g_tcm_heap;
		//DBG_INFO_MSG_ON(_DBG_TCM_HEAP_); // On Debug TCM MEM
#if	DEBUG_AT_USER_LEVEL > 1
		printf("ATST: Mem info:\n");
#endif
//		vPortFree(pvPortMalloc(4)); // Init RAM heap
		printf("\nCLK CPU\t\t%d Hz\nRAM heap\t%d bytes\nRAM free\t%d bytes\nTCM heap\t%d bytes\n",
				HalGetCpuClk(), xPortGetFreeHeapSize(), (int)&__StackTop - (int)&__HeapLimit, tcm_heap_freeSpace());
		printf("TCM ps_monitor\t%d bytes\n", 0x20000000 - (u32)&tcm_heap - tcm_heap_size);
		dump_mem_block_list();
		u32 saved = ConfigDebugInfo;
		DBG_INFO_MSG_ON(_DBG_TCM_HEAP_); // On Debug TCM MEM
		tcm_heap_dump();
		ConfigDebugInfo = saved;
		printf("\n");
#if (configGENERATE_RUN_TIME_STATS == 1)
		char *cBuffer = pvPortMalloc(512);
		if(cBuffer != NULL) {
			vTaskGetRunTimeStats((char *)cBuffer);
			printf("%s", cBuffer);
		}
		vPortFree(cBuffer);
#endif
}

int mp3_cfg_read(void)
{
	bzero(&mp3_serv, sizeof(mp3_serv));
	if(flash_read_cfg(mp3_serv, 0x5000, sizeof(mp3_serv.port) + 2) >= sizeof(mp3_serv.port) + 2) {
		mp3_serv.port = PLAY_PORT;
		strcpy(mp3_serv.url, PLAY_SERVER);
	}
	return mp3_serv.port;
}


// MP3 Set server, Close connect
void fATWS(void *arg){
	printf("ATWS: Usage: ATWS=URL,PORT or ATWS=close, ATWS=read, ATWS=save\n");
}


void fATWC(void *arg){
	int mode, ret;
	unsigned long tick1 = xTaskGetTickCount();
	unsigned long tick2, tick3;
	char empty_bssid[6] = {0}, assoc_by_bssid = 0;
	
	connect_close();
#if	DEBUG_AT_USER_LEVEL > 1
	printf("ATWC: Connect to AP...\n");
#endif
	if(memcmp (wifi.bssid.octet, empty_bssid, 6))
		assoc_by_bssid = 1;
	else if(wifi.ssid.val[0] == 0){
		printf("ATWC: Error: SSID can't be empty\n");
		ret = RTW_BADARG;
		goto EXIT;
	}
	if(wifi.password != NULL){
		if((wifi.key_id >= 0)&&(wifi.key_id <= 3)) {
			wifi.security_type = RTW_SECURITY_WEP_PSK;
		}
		else{
			wifi.security_type = RTW_SECURITY_WPA2_AES_PSK;
		}
	}
	else{
		wifi.security_type = RTW_SECURITY_OPEN;
	}
	//Check if in AP mode
	wext_get_mode(WLAN0_NAME, &mode);
	if(mode == IW_MODE_MASTER) {
		dhcps_deinit();
		wifi_off();
		vTaskDelay(20);
		if (wifi_on(RTW_MODE_STA) < 0){
			printf("ERROR: Wifi on failed!\n");
                        ret = RTW_ERROR;
			goto EXIT;
		}
	}

	if(assoc_by_bssid){
		printf("Joining BSS by BSSID "MAC_FMT" ...\n", MAC_ARG(wifi.bssid.octet));
		ret = wifi_connect_bssid(wifi.bssid.octet, (char*)wifi.ssid.val, wifi.security_type, (char*)wifi.password, 
						ETH_ALEN, wifi.ssid.len, wifi.password_len, wifi.key_id, NULL);		
	} else {
		printf("Joining BSS by SSID %s...\n", (char*)wifi.ssid.val);
		ret = wifi_connect((char*)wifi.ssid.val, wifi.security_type, (char*)wifi.password, wifi.ssid.len,
						wifi.password_len, wifi.key_id, NULL);
	}
	
	if(ret!= RTW_SUCCESS){
		printf("ERROR: Can't connect to AP\n");
		goto EXIT;
	}
	tick2 = xTaskGetTickCount();
	printf("Connected after %dms\n", (tick2-tick1));
	/* Start DHCPClient */
	LwIP_DHCP(0, DHCP_START);
	tick3 = xTaskGetTickCount();
	printf("Got IP after %dms\n", (tick3-tick1));
	printf("\n\r");
	connect_start();
EXIT:
	init_wifi_struct( );
}

void fATWD(void *arg){
	int timeout = 20;
	char essid[33];
	int ret = RTW_SUCCESS;

	connect_close();
#if	DEBUG_AT_USER_LEVEL > 1
	printf("ATWD: Disconnect...\n");
#endif
	printf("Dissociating AP ...\n");
	if(wext_get_ssid(WLAN0_NAME, (unsigned char *) essid) < 0) {
		printf("WIFI disconnected\n");
		goto exit;
	}

	if((ret = wifi_disconnect()) < 0) {
		printf("ERROR: Operation failed!\n");
		goto exit;
	}

	while(1) {
		if(wext_get_ssid(WLAN0_NAME, (unsigned char *) essid) < 0) {
			printf("WIFI disconnected\n");
			break;
		}

		if(timeout == 0) {
			printf("ERROR: Deassoc timeout!\n");
			ret = RTW_TIMEOUT;
			break;
		}

		vTaskDelay(1 * configTICK_RATE_HZ);
		timeout --;
	}
    dhcps_deinit();
    wifi_off();	
    printf("\n\r");
exit:
    init_wifi_struct( );
	return;
}

// Dump register
void fATSD(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

#if	DEBUG_AT_USER_LEVEL > 1
	printf("ATSD: dump registers\n");
#endif
	if(!arg){
		printf("ATSD: Usage: ATSD=REGISTER");
		return;
	}
	argc = parse_param(arg, argv);
	if(argc == 2 || argc == 3)
		CmdDumpWord(argc-1, (unsigned char**)(argv+1));
}

void fATSW(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

#if	DEBUG_AT_USER_LEVEL > 1
	printf("ATSW: write register\n");
#endif
	if(!arg){
		printf("ATSW: Usage: ATSW=REGISTER,DATA");
		return;
	}
	argc = parse_param(arg, argv);
	if(argc == 2 || argc == 3)
		CmdWriteWord(argc-1, (unsigned char**)(argv+1));
}

///// MP3 Set Mode
// MP3 Off
void fATOF(void *arg)
{
#if	DEBUG_AT_USER_LEVEL > 1
	printf("ATOF: MP3 off...\n");
#endif
	connect_close();
}


void print_wlan_help(void *arg){
		printf("WLAN AT COMMAND SET:\n");
		printf("==============================\n");
        printf(" Connect to an AES AP\n");
        printf("\t# ATW0=SSID\n");
        printf("\t# ATW1=PASSPHRASE\n");
        printf("\t# ATWC\n");
        printf(" DisConnect AP\n");
        printf("\t# ATWD\n");
}

log_item_t at_user_items[ ] = {
	{"ATVG", fATVG,},
	{"ATW0", fATW0,},
	{"ATW1", fATW1,},
	{"ATW2", fATW2,},
	{"ATWC", fATWC,},
	{"ATST", fATST,},
	{"ATSD", fATSD,},	// Dump register
	{"ATSW", fATSW,},	// Set register
	{"ATWD", fATWD,},	//
	{"ATWS", fATWS,},	// MP3 Set server, Close connect
	{"ATOF", fATOF,},	// MP3 Set Mode
};


void at_user_init(void)
{
	init_wifi_struct();
	mp3_cfg_read();
	log_service_add_table(at_user_items, sizeof(at_user_items)/sizeof(at_user_items[0]));
}

log_module_init(at_user_init);

#endif //#ifdef CONFIG_AT_USR
