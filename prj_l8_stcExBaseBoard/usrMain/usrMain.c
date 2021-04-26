#include "intrins.h"
#include "stdio.h"
#include "string.h"

#include "type_def.h"
#include "STC8xxxx.H"
#include "STC8G_H_Switch.h"

/*************************************************************************/
/*P54脚做接收脚*/
/*主频24M*/
/**/
/*************************************************************************/

//========================================================================
//                               主时钟定义
//========================================================================

//#define MAIN_Fosc		22118400L	//定义主时钟
//#define MAIN_Fosc		12000000L	//定义主时钟
//#define MAIN_Fosc		11059200L	//定义主时钟
//#define MAIN_Fosc		 5529600L	//定义主时钟
#define MAIN_Fosc		24000000L	//定义主时钟

#define UART1_BAUDRATE 	(65536UL - (MAIN_Fosc / 4) / 115200UL)

#define UART_DEVDRV_PROTOCOL_HEAD_M		0xEF
#define UART_DEVDRV_PROTOCOL_HEAD_S		0xEE
								   
#define UART_DATA_RX_TOUT				15	
#define UART_DATA_RX_QLEN 				64
								   
#define DEV_DIMMER_MODULATION_TIME_LAG	10	
#define DEV_FANS_MODULATION_TIME_LAG	10	

#define DEV_DIMMER_PIN_OUTPUT_SET(a)	P32 = a		
#define DEV_FANS_PIN_OUTPUT_SET(a)		P32 = a	

#define L8_DEVICE_TYPEDEFINE_BASE_ADDR	(0x49)

#define DEV_FAN_FULL_LOAD_EN		1
#define DEV_FAN_SHOCK_EXCUTE_EN		0		

#define DEV_DIMMER_FULL_LOAD_EN		0

#define PRJ_MODE_DBG				0x0A
#define PRJ_MODE_EXCUTE				0x0B
#define PRJ_MODE_DEF				PRJ_MODE_EXCUTE

typedef enum{

	devTypeDef_mulitSwOneBit = (L8_DEVICE_TYPEDEFINE_BASE_ADDR + 1),
	devTypeDef_mulitSwTwoBit,
	devTypeDef_mulitSwThreeBit,
	devTypeDef_dimmer,
	devTypeDef_fans,
	devTypeDef_scenario,
	devTypeDef_curtain,
	devTypeDef_thermostat = (L8_DEVICE_TYPEDEFINE_BASE_ADDR + 0x1E),
	devTypeDef_heater = (L8_DEVICE_TYPEDEFINE_BASE_ADDR + 0x1F),
}devTypeDef_enum;
								   
typedef struct __stt_uartProtocolFormat{

	uint8_t pHead;
	uint8_t command;
	uint8_t dats;
	uint8_t pTailCheck;
	
}stt_protocolFmtUartDevDrv;

typedef struct{
	uint16_t ltmrPeriod;
	uint16_t ltmrCounter;
}sttLoopTmr;

typedef struct{
	uint16_t ltmrPeriod;
	uint16_t ltmrCounter:15;
	uint16_t ltmr_tUpFlg:1;
}sttLoopTmr_exa;

bit UART1_TX_BUSY_FLG = 0;
		
static sttLoopTmr ltmrSecondSpfy = {(1000 * 20) - 1, 0};
static sttLoopTmr ltmrMsSpfy = {(1 * 20) - 1, 0};
static sttLoopTmr ltmrWdtSpfy = {100 - 1, 0}; //喂狗计时变量
static sttLoopTmr_exa ltmr_a = {1 - 1, 0, 0};

static uint16_t dataReq_actionCounter = 1;

static struct __stt_dimmer_debugAttr{

	uint8_t frepCounter;
	uint8_t frepConfirm;
}xdata devDimmer_debugParam = {0};

static struct __stt_fans_debugAttr{

	uint8_t frepCounter;
	uint8_t frepConfirm;
}xdata devFans_debugParam = {0};
			
static struct __stt_dimmer_attrFreq{ //

	u8 breightness;
	
	u8 periodBeat_cfm; 
	u8 periodBeat_counter; 
	
	u16 pwm_actEN:1;
	u16 pwm_actCounter:15;
	u8	pwm_actWaitCounter;
	
}volatile xdata devDimmer_freqParam = {0};

static struct __stt_fans_attrFreq{ //

	u8 speed;
	
	u8 periodBeat_cfm; 
	u8 periodBeat_counter; 
	
	u16 pwm_actEN:1;
	u16 pwm_actCounter:15;
	u8	pwm_actWaitCounter;
	
}volatile xdata devFans_freqParam = {0};
static bit devFans_excuteActivated_flg = FALSE;
		
static struct __stt_deviceRunningParam{

	uint8_t deviceType;
	
	struct __stt_devStatusDef{
	
		uint8_t deviceDimmer_brightness;
		uint8_t deviceFans_speed;
		uint8_t deviceOther_relayVal;
		
	}devStatus;
	
}xdata deviceRunningParam = {

	devTypeDef_dimmer,
	{0, 0, 0},
};

static struct __stt_dataRcv_attrJudge{

		  uint8_t rcvStart_flg:1;
	const uint8_t rcvTimeout_period:7;
		
		  uint8_t rcvTimeout_counter;
		  
}devUart_dataRcvJudgeAttr = {

	0, UART_DATA_RX_TOUT, 0,
};
static struct __stt_dataRcv{

	uint8_t dataRcvTout_trigFlg:1;
	uint8_t dataRcvTout_completeFlg:1;
	
	uint8_t  dataRcv_tmp[UART_DATA_RX_QLEN];
	uint8_t  dataRcvIst_tmp;
	uint8_t  dataRcv_cfm[UART_DATA_RX_QLEN];
	uint8_t  dataRcvLen_cfm;
	
}xdata devUart_dataRcvBuf = {0};
static uint8_t devUart_dataSendBuf[UART_DATA_RX_QLEN] = {0};

static unsigned char frame_Check(unsigned char frame_temp[], u8 check_num){
  
	unsigned char loop 		= 0;
	unsigned char val_Check = 0;
	
//	for(loop = 0; loop < check_num; loop ++){
//	
//		val_Check += frame_temp[loop];
//	}
//	
//	val_Check  = ~val_Check;
//	val_Check ^= 0xa7;
	
	for(loop = 0; loop < check_num; loop ++){
	
		val_Check ^= frame_temp[loop];
	}
	
	return val_Check;
}

static void sysPeriphInit_uart1(void){
	
	TR1 = 0;
	AUXR &= ~0x01;
	AUXR |= (1<<6);
	TMOD &= ~(1<<6);
	TMOD &= ~0x30;
	TH1 = (u8)(UART1_BAUDRATE / 256);
	TL1 = (u8)(UART1_BAUDRATE % 256);
	ET1 = 0;
	INT_CLKO &= ~0x02;
	TR1 = 1;
	
	/*Timer2暂时没用*/
//	AUXR &= ~(1<<4);	//Timer stop
//	AUXR |= 0x01;		//S1 BRT Use Timer2;
//	AUXR &= ~(1<<3);	//Timer2 set As Timer
//	AUXR |=  (1<<2);	//Timer2 set as 1T mode
//	TH2 = (u8)(UART1_BAUDRATE / 256);
//	TL2 = (u8)(UART1_BAUDRATE % 256);
//	IE2  &= ~(1<<2);	//禁止中断
//	AUXR |=  (1<<4);	//Timer run enable
	
	SCON = (SCON & 0x3f) | 0x40;
	// PS = 1;
	ES = 1;
	REN = 1;
#if(PRJ_MODE_DEF == PRJ_MODE_EXCUTE)
	P_SW1 &= 0x3f;
	P_SW1 |= (2 << 6);
#endif
	UART1_TX_BUSY_FLG = 0;
}

static void usrApp_uart1StringSend(u8 *puts)
{
	for(; *puts != 0; puts++){
		
		SBUF = *puts;
		UART1_TX_BUSY_FLG = 1;
		while(UART1_TX_BUSY_FLG);
	}
}

static void usrApp_uart1DataSend(uint8_t *dptr, uint16_t len){

	uint16_t loop = 0;
	uint16_t uartData_txLen = 0;
	
	(len > UART_DATA_RX_QLEN)?
		(uartData_txLen = UART_DATA_RX_QLEN):
	 	(uartData_txLen = len);

	memset(devUart_dataSendBuf, 0, sizeof(uint8_t) * UART_DATA_RX_QLEN);
	memcpy(devUart_dataSendBuf, dptr, sizeof(uint8_t) * uartData_txLen);
	
	for(loop = 0; loop < uartData_txLen; loop ++){
	
		SBUF = devUart_dataSendBuf[loop];
		UART1_TX_BUSY_FLG = 1;
		while(UART1_TX_BUSY_FLG);
	}
}

static void sysPeriphInit_exInt1(void){

    IT1 = 1; 
    EX1 = 1;
    EA = 1;
}

static void sysPeriphInit_timer0(void){  
	
	AUXR |= 0x80;		
	TMOD &= 0xF0;	
//	TL0 = 0xD7; //11.0592M
//	TH0 = 0xFD;
	TL0 = 0x50; //24M
	TH0 = 0xFB;
	ET0 = 1;	
	TR0 = 1;	
}

static void sysPeriphInit_gpio(void){

	P3M0 |= (1 << 2);
	P3M1 &= ~(1 << 2);
}

static void sysPeriphInit_wdt(void){

//	WDT_CONTR = 0x23; //0.5s
	WDT_CONTR = 0x24; //1s
//	WDT_CONTR = 0x27; //8s
}

void systemPeripheralsInitialization(void){

	EA = 0;
	
	sysPeriphInit_uart1();
	sysPeriphInit_exInt1();
	sysPeriphInit_timer0();
	sysPeriphInit_gpio();
	sysPeriphInit_wdt();
	
	Timer0_Polity(3); //定时器中断优先级最高
	INT1_Polity(2);
	UART1_Polity(1);
	
	EA = 1;
}

static void systemFunc_wdtFeed(void){

	WDT_CONTR |= 0x10;
}

void funcInterrupt_uart1(void) interrupt UART1_VECTOR
{
	if(TI){
		TI = 0;
		
		UART1_TX_BUSY_FLG = 0;
	}
	
	if(RI){
		RI = 0;
		
		devUart_dataRcvBuf.dataRcv_tmp[devUart_dataRcvBuf.dataRcvIst_tmp ++] = SBUF;
		if(devUart_dataRcvBuf.dataRcvIst_tmp >= (UART_DATA_RX_QLEN - 1))
			devUart_dataRcvBuf.dataRcvIst_tmp = (UART_DATA_RX_QLEN - 1);
		
		if(!devUart_dataRcvJudgeAttr.rcvStart_flg)
			devUart_dataRcvJudgeAttr.rcvStart_flg = 1;
		devUart_dataRcvJudgeAttr.rcvTimeout_counter = devUart_dataRcvJudgeAttr.rcvTimeout_period;
	}
}

void funcInterrupt_externalInt1(void) interrupt INT1_VECTOR
{
	switch(deviceRunningParam.deviceType){
	
		case devTypeDef_dimmer:
			
			devDimmer_freqParam.periodBeat_cfm = devDimmer_freqParam.periodBeat_counter;
			devDimmer_freqParam.periodBeat_counter = 0;
			
			devDimmer_freqParam.pwm_actEN = 1;
			devDimmer_freqParam.pwm_actWaitCounter = DEV_DIMMER_MODULATION_TIME_LAG;
			
			devDimmer_debugParam.frepCounter ++;
		
			break;
		
		case devTypeDef_fans:
			
			devFans_freqParam.periodBeat_cfm = devFans_freqParam.periodBeat_counter;
			devFans_freqParam.periodBeat_counter = 0;
			
			devFans_freqParam.pwm_actEN = 1;
			devFans_freqParam.pwm_actWaitCounter = DEV_FANS_MODULATION_TIME_LAG;
			
			devFans_debugParam.frepCounter ++;
		
			break;
		
		default:break;
	}
}

void funcInterrupt_timer0(void) interrupt TIMER0_VECTOR //50us
{	
	switch(deviceRunningParam.deviceType){
	
		case devTypeDef_dimmer:{ //调光业务
		
			const  uint16_t period_dimmerFollow = 400 - 1; //跟随速度
			static uint16_t counter_dimmerFollow = 0; 
					
			uint8_t freq_periodBeatHalf = devDimmer_freqParam.periodBeat_cfm / 2;
			
			devDimmer_freqParam.periodBeat_counter ++;
			
			if(0 != devDimmer_freqParam.breightness){

#if(1 == DEV_DIMMER_FULL_LOAD_EN)
				if(100 == devDimmer_freqParam.breightness){
					DEV_DIMMER_PIN_OUTPUT_SET(1);
				}else
#endif
				if(devDimmer_freqParam.pwm_actEN){
						
					if(devDimmer_freqParam.pwm_actWaitCounter)devDimmer_freqParam.pwm_actWaitCounter --;
					else{
					
						devDimmer_freqParam.pwm_actCounter ++;
						if(devDimmer_freqParam.pwm_actCounter <= devDimmer_freqParam.breightness && devDimmer_freqParam.pwm_actCounter < freq_periodBeatHalf){ //
								
							DEV_DIMMER_PIN_OUTPUT_SET(1);
						}
						else
						{
							DEV_DIMMER_PIN_OUTPUT_SET(0);
							
							devDimmer_freqParam.pwm_actCounter = 0;
							devDimmer_freqParam.pwm_actEN = 0;
						}
					}
				}
			}
			else
			{
				devDimmer_freqParam.pwm_actEN = 0;
				DEV_DIMMER_PIN_OUTPUT_SET(0);
			}
			
			if(counter_dimmerFollow < period_dimmerFollow)counter_dimmerFollow ++; //亮度惰性跟随
			else{
			
				counter_dimmerFollow = 0;
				
				if(devDimmer_freqParam.breightness != deviceRunningParam.devStatus.deviceDimmer_brightness){
				
					(devDimmer_freqParam.breightness < deviceRunningParam.devStatus.deviceDimmer_brightness)?
						(devDimmer_freqParam.breightness ++):
						(devDimmer_freqParam.breightness --);
				}
			}
		
		}break;
		
		case devTypeDef_fans:{
		
			const  uint16_t modulationAnchorPoint = 150; //锚定点(STC只能单独下降沿，所以锚点提前，原锚点是200，150最佳点，方便放大)
			const  uint16_t period_fansFollow = 1000 - 1; //跟随速度
			static uint16_t counter_fansFollow = 0; 
			uint16_t vatFltSpeedTarget = devFans_freqParam.speed * 3 / 2;
			
			devFans_freqParam.periodBeat_counter ++;
			
			if(0 != devFans_freqParam.speed){

#if(1 == DEV_FAN_FULL_LOAD_EN)
				if(100 == devFans_freqParam.speed){ //全开
					DEV_FANS_PIN_OUTPUT_SET(1);
				}else
#endif
				if(devFans_freqParam.pwm_actEN){	
					
					if(devFans_freqParam.periodBeat_counter > (modulationAnchorPoint - devFans_freqParam.speed)){ //后沿切相
//					if(devFans_freqParam.periodBeat_counter > (modulationAnchorPoint - vatFltSpeedTarget)){ //后沿切相
							
						if(devFans_freqParam.periodBeat_counter > modulationAnchorPoint){
							DEV_FANS_PIN_OUTPUT_SET(0);
						}
						else{
							DEV_FANS_PIN_OUTPUT_SET(1);
						}
					}
					else{
						DEV_FANS_PIN_OUTPUT_SET(0);
					}
				}
			}
			else
			{
				devFans_freqParam.pwm_actEN = 0;
				DEV_FANS_PIN_OUTPUT_SET(0);
			}
			
			if(counter_fansFollow < period_fansFollow)counter_fansFollow ++; //速度惰性跟随
			else{
			
				counter_fansFollow = 0;
				
#if(1 == DEV_FAN_SHOCK_EXCUTE_EN)
		
				if(TRUE == devFans_excuteActivated_flg){ //冲击标志：给与初速度
					
					if(devFans_freqParam.speed < 100){
						devFans_freqParam.speed ++;
					}else{
						devFans_excuteActivated_flg = FALSE;
					}
				}else{

					if(devFans_freqParam.speed != deviceRunningParam.devStatus.deviceFans_speed){
						
						(devFans_freqParam.speed < deviceRunningParam.devStatus.deviceFans_speed)?
							(devFans_freqParam.speed ++):
							(devFans_freqParam.speed --);
					}					
				}
#else

				if(devFans_freqParam.speed != deviceRunningParam.devStatus.deviceFans_speed){
					
					(devFans_freqParam.speed < deviceRunningParam.devStatus.deviceFans_speed)?
						(devFans_freqParam.speed ++):
						(devFans_freqParam.speed --);
				}	
#endif
			}
		
		}break;
		
		default:break;
	}
	
	if(ltmrMsSpfy.ltmrCounter)ltmrMsSpfy.ltmrCounter --;
	else{
		ltmrMsSpfy.ltmrCounter = ltmrMsSpfy.ltmrPeriod;
		
		if(dataReq_actionCounter)dataReq_actionCounter --;
		
		if(devUart_dataRcvJudgeAttr.rcvStart_flg){
			if(devUart_dataRcvJudgeAttr.rcvTimeout_counter)devUart_dataRcvJudgeAttr.rcvTimeout_counter --;
			else{
				devUart_dataRcvBuf.dataRcvTout_trigFlg = 1;
				devUart_dataRcvJudgeAttr.rcvStart_flg = 0;
			}
		}
		
		if(ltmrWdtSpfy.ltmrCounter)ltmrWdtSpfy.ltmrCounter --;
		else{
			ltmrWdtSpfy.ltmrCounter = ltmrWdtSpfy.ltmrPeriod;
			systemFunc_wdtFeed();
		}
	}
	
	if(ltmrSecondSpfy.ltmrCounter)ltmrSecondSpfy.ltmrCounter --;
	else{
		ltmrSecondSpfy.ltmrCounter = ltmrSecondSpfy.ltmrPeriod;
		
		if(ltmr_a.ltmrCounter)ltmr_a.ltmrCounter --;
		else{
			ltmr_a.ltmrCounter = ltmr_a.ltmrPeriod;
			ltmr_a.ltmr_tUpFlg = 1;
		}
		
		switch(deviceRunningParam.deviceType){

			case devTypeDef_dimmer:{
			
				//调光调试数据更新
				devDimmer_debugParam.frepConfirm = devDimmer_debugParam.frepCounter;
				devDimmer_debugParam.frepCounter = 0;
				
			}break;
			
			case devTypeDef_fans:{
			
				//风扇调试数据更新
				devFans_debugParam.frepConfirm = devFans_debugParam.frepCounter;
				devFans_debugParam.frepCounter = 0;
				
			}break;
			
			default:break;
		}
	}
}

static void process_uartDataRcv_timeOut_funcHandle(void){
	
	if(devUart_dataRcvBuf.dataRcvTout_trigFlg){
	
		devUart_dataRcvBuf.dataRcvTout_trigFlg = 0;
		
		memset(devUart_dataRcvBuf.dataRcv_cfm, 0, sizeof(uint8_t) * UART_DATA_RX_QLEN);
		memcpy(devUart_dataRcvBuf.dataRcv_cfm, devUart_dataRcvBuf.dataRcv_tmp, sizeof(uint8_t) * UART_DATA_RX_QLEN);
		devUart_dataRcvBuf.dataRcvLen_cfm = devUart_dataRcvBuf.dataRcvIst_tmp;
		memset(devUart_dataRcvBuf.dataRcv_tmp, 0, sizeof(uint8_t) * UART_DATA_RX_QLEN);
		devUart_dataRcvBuf.dataRcvIst_tmp = 0;
		
		devUart_dataRcvBuf.dataRcvTout_completeFlg = 1;
	}
}

static void usrApp_deviceDriverByMcu_applicationExecute(uint8_t cmd, uint8_t dat){

	uint8_t loop = 0;
	
	switch(cmd){
	
		case 1:{ //设备类型
		
			deviceRunningParam.deviceType = dat;
			memset(&deviceRunningParam.devStatus, 0, sizeof(struct __stt_devStatusDef));
			
		}break;
		
		case 2:{ //状态值
		
			switch(deviceRunningParam.deviceType){
			
				case devTypeDef_scenario:{}break;
				
				case devTypeDef_dimmer:{
				
					deviceRunningParam.devStatus.deviceDimmer_brightness = dat;
				
				}break;
				
				case devTypeDef_fans:{

#if(1 == DEV_FAN_SHOCK_EXCUTE_EN)					
					if(deviceRunningParam.devStatus.deviceFans_speed < dat)devFans_excuteActivated_flg = TRUE;
#endif
					deviceRunningParam.devStatus.deviceFans_speed = dat;
					
				}break;
				
				case devTypeDef_mulitSwOneBit:
				case devTypeDef_mulitSwTwoBit:
				case devTypeDef_mulitSwThreeBit:
				case devTypeDef_curtain:
				case devTypeDef_thermostat:
                case devTypeDef_heater:{}break;

				default:break;
			}
			
		}break;
		
		default:break;
	}
}

static void process_bussinessDeviceDriver(void){

	if(devUart_dataRcvBuf.dataRcvTout_completeFlg){ //
	
		devUart_dataRcvBuf.dataRcvTout_completeFlg = 0;
	
		if((UART_DEVDRV_PROTOCOL_HEAD_M == devUart_dataRcvBuf.dataRcv_cfm[0]) && //
		   (sizeof(stt_protocolFmtUartDevDrv) == devUart_dataRcvBuf.dataRcvLen_cfm)){ //
		
			stt_protocolFmtUartDevDrv dataTemp = {0};
			
			memcpy(&dataTemp, devUart_dataRcvBuf.dataRcv_cfm, sizeof(stt_protocolFmtUartDevDrv));
			if(frame_Check(devUart_dataRcvBuf.dataRcv_cfm, sizeof(stt_protocolFmtUartDevDrv) - 1) == dataTemp.pTailCheck){ //
			
				usrApp_deviceDriverByMcu_applicationExecute(dataTemp.command, dataTemp.dats); //

				dataTemp.pHead = UART_DEVDRV_PROTOCOL_HEAD_S;
				dataTemp.pTailCheck = frame_Check((uint8_t *)&dataTemp, sizeof(stt_protocolFmtUartDevDrv) - 1);
				usrApp_uart1DataSend((uint8_t *)&dataTemp, sizeof(stt_protocolFmtUartDevDrv)); //
			}
			else
			{
				usrApp_uart1StringSend("check err\n");
			}
		}	
		else
		{
			usrApp_uart1StringSend("format err\n");
		}
	}
}

static void devTest(void){
		
	process_uartDataRcv_timeOut_funcHandle();
	
//	if(devUart_dataRcvBuf.dataRcvTout_completeFlg){
//		
//		devUart_dataRcvBuf.dataRcvTout_completeFlg = 0;
//	
//		if(!strcmp("open", (const char *)devUart_dataRcvBuf.dataRcv_cfm)){
//		
//			usrApp_uart1StringSend("ok\n");
//		}
//		else
//		{
//			usrApp_uart1DataSend(devUart_dataRcvBuf.dataRcv_cfm, devUart_dataRcvBuf.dataRcvLen_cfm);
//		}
//	}
//
//	delay(1000);
//	UART1_PRINTF("soft mark\n");
//
//	if(!dataReq_actionCounter){
//		
//		dataReq_actionCounter = 1000;
//		
//		usrApp_uart1StringSend("hard mark\n");
//		
//		GPIO_WriteReverse(GPIOB, GPIO_PIN_5);
//	}

	if(!dataReq_actionCounter){
		
		dataReq_actionCounter = 2000;
				
		switch(deviceRunningParam.deviceType){
		
			case devTypeDef_dimmer:{
			
				memset(devUart_dataSendBuf, 0, UART_DATA_RX_QLEN * sizeof(char));
				sprintf((char *)devUart_dataSendBuf, 
						"[devDimmer]source freq:%dHz, loadVal:%d\n", 
						(int)devDimmer_debugParam.frepConfirm,
						(int)deviceRunningParam.devStatus.deviceDimmer_brightness);
				usrApp_uart1StringSend((char *)devUart_dataSendBuf);
						
			}break;
			
			case devTypeDef_fans:{
			
				memset(devUart_dataSendBuf, 0, UART_DATA_RX_QLEN * sizeof(char));
				sprintf((char *)devUart_dataSendBuf, 
						"[devFans]source freq:%dHz, beatPeriod:%d, loadVal:%d\n", 
						(int)devFans_debugParam.frepConfirm,
						(int)devFans_freqParam.periodBeat_cfm,
						(int)deviceRunningParam.devStatus.deviceFans_speed);
				usrApp_uart1StringSend((char *)devUart_dataSendBuf);
						
			}break;
			
			default:break;
		}		
	}
	
	if(devUart_dataRcvBuf.dataRcvTout_completeFlg){
	
		devUart_dataRcvBuf.dataRcvTout_completeFlg = 0;
		
		if(!strcmp("devDef fans", (const char *)devUart_dataRcvBuf.dataRcv_cfm)){
		
			deviceRunningParam.deviceType = devTypeDef_fans;
			usrApp_uart1StringSend("ok\n");
		}else
		if(!strcmp("devDef dimmer", (const char *)devUart_dataRcvBuf.dataRcv_cfm)){
		
			deviceRunningParam.deviceType = devTypeDef_dimmer;
			usrApp_uart1StringSend("ok\n");
		}else
		switch(deviceRunningParam.deviceType){
		
			case devTypeDef_dimmer:{
			
				if(!strcmp("open", (const char *)devUart_dataRcvBuf.dataRcv_cfm)){
				
					usrApp_uart1StringSend("ok\n");
				}
				else
				if(!strcmp("close", (const char *)devUart_dataRcvBuf.dataRcv_cfm)){
				
					usrApp_uart1StringSend("ok\n");
				}
				else
				if(!memcmp("brightness:", (const uint8_t *)devUart_dataRcvBuf.dataRcv_cfm, 11)){
				
					int brightness = 0;
					
//					brightness = ((devUart_dataRcvBuf.dataRcv_cfm[11] - '0') * 10) +
//								 ((devUart_dataRcvBuf.dataRcv_cfm[12] - '0') * 1);
					
//					if(brightness <= 100){
//					
//						usrApp_uart1StringSend("ok\n");
//						deviceRunningParam.devStatus.deviceDimmer_brightness = brightness;
//					}
//					else
//					{
//						usrApp_uart1StringSend("format err\n");
//					}
					
					if(sscanf(&devUart_dataRcvBuf.dataRcv_cfm[11], "%d", (int *)&brightness)){
						if(brightness <= 100){
							usrApp_uart1StringSend("ok\n");
							deviceRunningParam.devStatus.deviceDimmer_brightness = brightness;
						}else{
							usrApp_uart1StringSend("value too large!!!\n");
						}
					}else{
						usrApp_uart1StringSend("format err\n");
					}
					
				}else{
				
					usrApp_uart1StringSend("format err\n");
				}
				
			}break;
			
			case devTypeDef_fans:{
			
				if(!memcmp("speed:", (const uint8_t *)devUart_dataRcvBuf.dataRcv_cfm, 6)){
				
					int speed = 0;
					
//					speed = ((devUart_dataRcvBuf.dataRcv_cfm[6] - '0') * 10) +
//							((devUart_dataRcvBuf.dataRcv_cfm[7] - '0') * 1);
			
//					if(speed <= 100){
//					
//						usrApp_uart1StringSend("ok\n");
//						deviceRunningParam.devStatus.deviceFans_speed = speed;
//					}
//					else
//					{
//						usrApp_uart1StringSend("format err\n");
//					}
					
					if(sscanf(&devUart_dataRcvBuf.dataRcv_cfm[6], "%d", (int *)&speed)){
						if(speed <= 100){
							usrApp_uart1StringSend("ok\n");
#if(1 == DEV_FAN_SHOCK_EXCUTE_EN)
							if(deviceRunningParam.devStatus.deviceFans_speed < speed)devFans_excuteActivated_flg = TRUE;
#endif							
							deviceRunningParam.devStatus.deviceFans_speed = speed;
						}
						else{
							usrApp_uart1StringSend("value too large!!!\n");
						}
					}else{
						usrApp_uart1StringSend("format err\n");
					}
					
				}else{
				
					usrApp_uart1StringSend("format err\n");
				}
				
			}break;
			
			default:
				usrApp_uart1StringSend("format err\n");
				break;
		}
	}
}

void main(void){
	
	systemPeripheralsInitialization();
	
	for(;;){

#if(PRJ_MODE_DEF == PRJ_MODE_DBG)
		devTest();
#else
		process_uartDataRcv_timeOut_funcHandle(); //
		process_bussinessDeviceDriver();
#endif
		
//		if(ltmr_a.ltmr_tUpFlg){
//			ltmr_a.ltmr_tUpFlg = 0;
//			usrApp_uart1StringSend("hellow world!!!\n");
//		}
	}
}

























