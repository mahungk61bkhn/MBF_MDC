/***********************************************************************
*
*  FILE        : MDC.c
*  DATE        : 2023-02-22
*  DESCRIPTION : Main Program
*
*  NOTE:THIS IS A TYPICAL EXAMPLE.
*
***********************************************************************/
#include "r_smc_entry.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "r_fw_up_rx_if.h"
#include "deviceFlash.h"
#include "storage.h"


//static char version_I[40] = "MDC Dev. 2023 Feb. 28 - 10:23 I";
//static char version_V[40] = "MDC Dev. 2023 Feb. 28 - 10:23 V";
static char print_str[150];
/********************** GPIOs **************************/
#define MCU_LED_STT (PORTA.PODR.BIT.B6)
#define MCU_LED_PSU (PORTB.PODR.BIT.B0)
#define MCU_LED_MCC (PORTB.PODR.BIT.B1)
#define BUZZER 		(PORT3.PODR.BIT.B1)
#define WDI 		(PORT3.PODR.BIT.B2)
/********************** Modbus MDC Register Values (MDC as Slave)************************************/
#define MDC_ID 0x05
#define MDC_VERSION 8
#define MDC_NUM_REGS 320
uint16_t MDC_regs[MDC_NUM_REGS];
/********************** Modbus MDC ************************************/
#define RS485_M_Ctr (PORT2.PODR.BIT.B7)
#define RS485_S_Ctr (PORTC.PODR.BIT.B1)
#define FLASH_CE 	(PORTC.PODR.BIT.B4)
#define CALIB_EN	(PORTH.PIDR.BIT.B2)
uint8_t transmit_ctrl=0; // enable transmit to MCC
volatile uint8_t is_slave =0;
volatile uint8_t PSU_connect_flag =0;
extern volatile uint8_t MCC_timeout_flag;
uint8_t Rs485_RequestToSlave[8];// Request send to Slave
uint8_t Rs485_MasterResponse[(MDC_NUM_REGS*2+10)];// Response to MCC
uint8_t rx5_buff[300];
uint8_t rx1_buff[500];
uint16_t SPI_buff[270];
//extern uint8_t SCI5_rxdone;
extern uint8_t SCI1_rxdone;
extern uint16_t  g_sci5_rx_count;
uint16_t rx5_count;
uint16_t rx1_count;
extern uint32_t tick;


/**********************Slave Register (MDC as Master)*************************************/
#define ID_Accu_Shoto 0x00
#define ID_Box_Huawei 0x21
#define ID_Accu_Vision 0x00
#define ID_Accu_Huawei 214
#define ID_Accu_Narada 1
#define ID_ZZT4850 1
uint16_t BattRegs[130];
uint16_t BattRegs34[130];
uint16_t BattRegs5[25];

/**********************MDC Measurement****************************************************/
#define OFFSET_TEMP1 0
#define OFFSET_TEMP2 0
#define CURR_SENSOR_TH 2 //offset 0A
#define SAMPLES_NUM  128
extern uint16_t ADC_sample_count;
extern volatile uint8_t Sample_done;
extern uint32_t wait_time;
uint8_t discharge_start=0; //for counting discharge time
uint8_t charge_start=0;
uint8_t charge_discharge_sts =0; // indicate charge(1)/discharge(0) current
extern uint32_t discharge_time_count; //for counting discharge time
extern uint32_t charge_time_count; //for counting discharge time
uint16_t chargtimeH=0;
uint16_t chargtimeL=0;
uint16_t dischargtimeH=0;
uint16_t dischargtimeL=0;

uint16_t ADC_Temp1[SAMPLES_NUM], ADC_Temp2[SAMPLES_NUM];
uint16_t ADC_Volt1[SAMPLES_NUM], ADC_Volt2[SAMPLES_NUM], ADC_Volt3[SAMPLES_NUM], ADC_Volt4[SAMPLES_NUM];
uint16_t ADC_Curr1[SAMPLES_NUM];
uint16_t ADC_Curr2[SAMPLES_NUM];
uint16_t ADC_Curr3[SAMPLES_NUM];
uint16_t ADC_Curr4[SAMPLES_NUM];
uint16_t ADC_CurrRef1[SAMPLES_NUM];
uint16_t ADC_CurrRef2[SAMPLES_NUM];
uint16_t ADC_CurrRef3[SAMPLES_NUM];
uint16_t ADC_CurrRef4[SAMPLES_NUM];

int16_t OFFSET_V1=0;
int16_t OFFSET_V2=0;
int16_t OFFSET_V3=0;
int16_t OFFSET_V4=0;
int16_t OFFSET_I1=0;
int16_t OFFSET_I2=0;
int16_t OFFSET_I3=0;
int16_t OFFSET_I4=0;
uint8_t data_calib[16];
uint8_t data_time[12];



int32_t CALIB_INIT_V1=0;
int32_t CALIB_INIT_V2=0;
int32_t CALIB_INIT_V3=0;
int32_t CALIB_INIT_V4=0;

int32_t CALIB_INIT_I1=0;
int32_t CALIB_INIT_I2=0;
int32_t CALIB_INIT_I3=0;
int32_t CALIB_INIT_I4=0;

volatile uint32_t Sector0_address = 0;
volatile uint32_t Sector1_address = 0x1000;
volatile uint32_t Offset_addr = 0;// start address of all offsets. sector 1, 1 page = 256 bytes
volatile uint32_t charge_discharge_time_addr = 65536;// Start address of charge and discharge time, sector 2
static const uint16_t ADC_LUT[]={ //ADC LUT for temperature from (-10)C to 170C - resolution 1C - 181 values
		3432,3404,3375,3345,3315,3283,3251,3218,3184,3150,3114,3077,3040,3001,2962,2923,2882,2841,2800,2758,2715,2672,2628,2585,2541,
		2496,2452,2407,2362,2317,2272,2226,2182,2136,2092,2047,2003,1958,1914,1871,1828,1785,1742,1701,1659,1618,1578,1538,1499,1461,
		1423,1385,1349,1313,1278,1243,1209,1176,1144,1112,1081,1051,1022,993,965,937,911,885,859,835,811,788,765,743,721,700,680,660,
		641,623,605,587,570,554,538,522,507,493,479,465,452,439,426,414,403,391,380,370,360,350,340,331,322,313,304,296,288,280,272,265,
		258,251,245,238,232,226,220,214,208,203,197,192,187,182,177,172,167,163,159,154,150,146,142,139,135,131,128,125,122,118,115,112,
		110,107,104,101,99,96,94,92,89,87,85,83,81,79,77,75,74,72,70,69,67,65,64,63,61,60,58,57,56,55,54,52,51,50,49,48,47,46,45
};

/********************** Functions **************************/
void main_FW_update(void);
void main(void);
extern MD_STATUS R_SCI1_AsyncTransmit (uint8_t * const tx_buf,uint16_t tx_num);
extern MD_STATUS R_SCI5_AsyncTransmit (uint8_t * const tx_buf,uint16_t tx_num, uint8_t control);
void RS485_Slave_Mode(void);
void RS485_Master_Mode(void);
uint16_t CRC16_bytewise (uint8_t *nData, int wLength);
void uint16_to_uint8(uint16_t input, uint8_t* Hi_byte, uint8_t* Low_byte);
void RS485_M_Read_and_Receive(uint8_t slaveID, uint16_t Register, uint8_t *request, uint16_t * Value);
void RS485_M_Cmd04_and_Receive(uint8_t slaveID, uint32_t StartAdd, uint16_t NoR, uint16_t * Value);
void measure(void);
float temp_measure(uint16_t *ADC_value,uint8_t channel);
float volt_measure(uint16_t *ADC_value,uint8_t channel);
float current_measure(uint16_t *ADC_value,uint16_t *ADC_Vref, uint8_t channel);
void RS485_M_Read_Batt(uint8_t slaveID, uint16_t Start_Add, uint16_t NbRgt ,uint8_t *request, uint16_t * Response); //Store message in Response
void waittime(uint16_t time);
void Flash_write(uint32_t address, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4, uint16_t data5, uint16_t data6, uint16_t data7, uint16_t data8);
void Flash_read(uint32_t address, uint16_t *data1,uint16_t *data2,uint16_t *data3,uint16_t *data4, uint16_t *data5,uint16_t *data6,uint16_t *data7,uint16_t *data8);
void Flash_Sector_Erase(uint32_t address);
void Flash_Block_Erase(uint32_t address);
void Flash_Chip_Erase();
void Flash_Read_StsReg(uint8_t *sts);
void Buzzer(uint8_t times, uint16_t millisec);
void Led_Blink();
void main(void)
{
	MDC_regs[21] =0;
	MDC_regs[22] =0;
	MDC_regs[99] =0; //Enable send to MCC
	MDC_regs[100] =0; // Bootloader Status Register
	MDC_regs[101] =0; // Bootloader Control Register
	MDC_regs[102] = MDC_VERSION; // Bootloader Version Register
	MDC_regs[103] =0; // Huawei ACCU
	tick=0;
	charge_time_count=0;
	discharge_time_count=0;

	MCU_LED_STT=0;
	MCU_LED_PSU=0;
	MCU_LED_MCC=0;

	//WTD impulse
	WDI =1;
	R_BSP_SoftwareDelay(5, BSP_DELAY_MICROSECS);
	WDI =0;

	memset(&data_calib,0,sizeof(data_calib));
	memset(&data_time,0,sizeof(data_time));
	memset(ADC_Temp1,0,sizeof(ADC_Temp1));
	memset(ADC_Temp2,0,sizeof(ADC_Temp2));
	memset(ADC_Curr1,0,sizeof(ADC_Curr1));
	memset(ADC_Curr2,0,sizeof(ADC_Curr2));
	memset(ADC_Curr3,0,sizeof(ADC_Curr3));
	memset(ADC_Curr4,0,sizeof(ADC_Curr4));
	memset(ADC_CurrRef1,0,sizeof(ADC_CurrRef1));
	memset(ADC_CurrRef2,0,sizeof(ADC_CurrRef2));
	memset(ADC_CurrRef3,0,sizeof(ADC_CurrRef3));
	memset(ADC_CurrRef4,0,sizeof(ADC_CurrRef4));
	memset(ADC_Volt1,0,sizeof(ADC_Volt1));
	memset(ADC_Volt2,0,sizeof(ADC_Volt2));
	memset(ADC_Volt3,0,sizeof(ADC_Volt3));
	memset(ADC_Volt4,0,sizeof(ADC_Volt4));

	ADC_sample_count=0;
	R_Config_CMT0_Start();// Start Sampling ADC value for Current/Temp/Voltage measurement
	R_Config_CMT1_Start();
	R_BSP_SoftwareDelay(200, BSP_DELAY_MILLISECS);//wait sampling finish 1st round.
	R_Config_SCI5_Start();//UART5 init for RS485 - Slave to communicate with MCC - Remember to add rx5_count = g_sci5_rx_count; in r_Config_SCI5_receive_interrupt()
	R_Config_SCI1_Start();//UART1 init for RS485_M - Master to communicate with ACCU
	R_Config_SCI5_Serial_Receive((uint8_t*)&rx5_buff, sizeof(rx5_buff));
	R_Config_RSPI0_Start();
	for(int i=1;i<3;i++)
	{
		deviceFlash_readData(Offset_addr, data_calib, 16);
	}

	deviceFlash_readData(Offset_addr, data_calib, 16);
	OFFSET_V1 = (uint16_t)(data_calib[0]<<8) +(uint16_t)data_calib[1];
	OFFSET_V2 = (uint16_t)(data_calib[2]<<8) +(uint16_t)data_calib[3];
	OFFSET_V3 = (uint16_t)(data_calib[4]<<8) +(uint16_t)data_calib[5];
	OFFSET_V4 = (uint16_t)(data_calib[6]<<8) +(uint16_t)data_calib[7];
	OFFSET_I1 = (uint16_t)(data_calib[8]<<8) +(uint16_t)data_calib[9];
	OFFSET_I2 = (uint16_t)(data_calib[10]<<8) +(uint16_t)data_calib[11];
	OFFSET_I3 = (uint16_t)(data_calib[12]<<8) +(uint16_t)data_calib[13];
	OFFSET_I4 = (uint16_t)(data_calib[14]<<8) +(uint16_t)data_calib[15];


	memset(data_time,0,sizeof(data_time));
	deviceFlash_readData(charge_discharge_time_addr, data_time, 10);
	charge_time_count = ((uint32_t)(data_time[0]<<24) +(uint32_t)(data_time[1]<<16)+(uint32_t)(data_time[2]<<8)+(uint32_t)(data_time[3])); // @suppress("Symbol is not resolved")
	discharge_time_count = ((uint32_t)(data_time[4]<<24) +(uint32_t)(data_time[5]<<16)+(uint32_t)(data_time[6]<<8)+(uint32_t)(data_time[7]));
	if(charge_time_count > 900000) charge_time_count=0;
	if(discharge_time_count > 900000) discharge_time_count=0;
	MCC_timeout_flag = data_time[11];

	Buzzer(2,50);


	//print test
	sprintf(print_str,"\n\rV_OffSet= %d %d %d %d\r\n",OFFSET_V1,OFFSET_V2,OFFSET_V3,OFFSET_V4);
	RS485_S_Ctr = 1U; //RS485 Master send mode
	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
	RS485_S_Ctr = 0U; //RS485 Master receive mode
	memset(print_str, 0, sizeof(print_str));
	//end_print_test
	//print test
	sprintf(print_str,"\n\rI_OffSet= %d %d %d %d\r\n",OFFSET_I1,OFFSET_I2,OFFSET_I3,OFFSET_I4);
	RS485_S_Ctr = 1U; //RS485 Master send mode
	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
	RS485_S_Ctr = 0U; //RS485 Master receive mode
	memset(print_str, 0, sizeof(print_str));
	//end_print_test
	//print test
	sprintf(print_str,"\n\rCharge Time(ms)= %d Disharge Time(ms)= %d\r\n",charge_time_count,discharge_time_count);
	RS485_S_Ctr = 1U; //RS485 Master send mode
	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
	RS485_S_Ctr = 0U; //RS485 Master receive mode
	memset(print_str, 0, sizeof(print_str));
	//end_print_test

	RS485_Master_Mode(); // Read parameters from ACCU
	is_slave=1; // Start counting time MCC disconnect
	measure(); // Read direct

   //for BLD - OTA - Firmware update
	fw_up_return_t ret_fw_up;
    ret_fw_up = fw_up_open_flash();
	while(1)
	{
		R_BSP_SoftwareDelay(200, BSP_DELAY_MILLISECS);
		if(g_sci5_rx_count>7)
		{
			RS485_Slave_Mode();
		}
		else
		{
			//Disable transmit to MCC
			if(MDC_regs[99]==1)
			{
				transmit_ctrl=1;
			}
			else
			{
				transmit_ctrl=0;
			}

			//Boot loader Mode
			if(MDC_regs[101]==1)
			{
				main_FW_update();
			}

			// Start counting time out MCC connection
			is_slave=1;
			// read parameters from Huawei Box and Baterry ACCU
			if(MDC_regs[47]==1) //Read From Electrical Box
			{
				measure();
				RS485_Master_Mode();
			}
			else // Direct measurement
			{
				measure();
			}

			//DC Low Warning
			if(((MDC_regs[0]>3600)&&(MDC_regs[0]<(MDC_regs[46]*100)))||((MDC_regs[1]>3600)&&(MDC_regs[1]<(MDC_regs[46]*100)))||((MDC_regs[2]>3600)&&(MDC_regs[2]<(MDC_regs[46]*100)))||((MDC_regs[3]>3600)&&(MDC_regs[3]<(MDC_regs[46]*100))))
			{
				MDC_regs[43] =1; //DC low warning register
			}
			else MDC_regs[43] =0;

			//DC discharge time count
			if((MDC_regs[5]>300)||(MDC_regs[6]>300)||(MDC_regs[7]>300)||(MDC_regs[8]>300)) //discharging/charging
			{
				if(charge_discharge_sts==0)
				{
					discharge_start=1; //start count
					MDC_regs[57] =1;
					charge_start =0;
					MDC_regs[56] =0;
				}
				else
				{
					charge_start =1;
					MDC_regs[56] =1;
					discharge_start=0; //start count
					MDC_regs[57] =0;
				}
			}
			else
			{
				discharge_start=0; //stop count
				charge_start=0;
				MDC_regs[56] =0;
				MDC_regs[57] =0;
			}

			MDC_regs[21] = (uint16_t)(charge_time_count/300); // 200ms*(5*60) = 1min
			MDC_regs[22] = (uint16_t)(discharge_time_count/300); // 200ms*(5*60) = 1min
			uint32_t lasttick = tick;

			// Save time to flash
//			chargtimeH = (uint16_t)(charge_time_count>>16);
//			chargtimeL = (uint16_t)(charge_time_count& 0xFFFF);
//			dischargtimeH = (uint16_t)(discharge_time_count>>16);
//			dischargtimeL = (uint16_t)(discharge_time_count& 0xFFFF);
//			Flash_Sector_Erase(charge_discharge_time_addr);
//			Flash_write(charge_discharge_time_addr,chargtimeH, chargtimeL, dischargtimeH,dischargtimeL,MCC_timeout_flag,0,0,0);

			if(tick-lasttick>1000*60)
			{
				lasttick = tick;
				uint32_t savedata[3];
				savedata[0] = charge_time_count;
				savedata[1] = discharge_time_count;
				savedata[2] = MCC_timeout_flag;
				deviceFlash_erase4k(charge_discharge_time_addr);
				deviceFlash_writeData(charge_discharge_time_addr, savedata, sizeof(savedata));
			}
		}
	}
}
void main_FW_update(void)
{
    fw_up_return_t ret_fw_up;
    ret_fw_up = fw_up_open();
    if (FW_UP_SUCCESS != ret_fw_up)
    {
    }
    else
    {
        MCU_LED_STT =1;
        MCU_LED_PSU =1;
        MCU_LED_MCC =1;
        Buzzer(1, 300);
    	ret_fw_up = switch_start_up_and_reset();

        switch (ret_fw_up)
        {
            case FW_UP_ERR_NOT_OPEN:
                MCU_LED_STT =0;
                break;
            case FW_UP_ERR_INVALID_RESETVECT:
                MCU_LED_PSU =0;
                break;
            case FW_UP_ERR_SWITCH_AREA:
                MCU_LED_MCC =0;
                break;
            default:

                /** Do nothing */
            break;
        }

        ret_fw_up = fw_up_close();

        if (FW_UP_SUCCESS != ret_fw_up)
        {

        }
    }
}
void uint16_to_uint8(uint16_t input, uint8_t* Hi_byte, uint8_t* Low_byte)
{
	*Hi_byte = input>>8;
	*Low_byte = input & 0xFF;
}

uint16_t CRC16_bytewise (uint8_t *nData, int wLength)
{
    static const uint16_t wCRCTable[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

    uint8_t nTemp;
    uint16_t wCRCWord = 0xFFFF;

    while (wLength--)
    {
        nTemp = *nData++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord  ^= wCRCTable[(nTemp & 0xFF)];
    }
    return wCRCWord;
} // End: CRC16
void RS485_Slave_Mode()
{
	uint16_t CRC16;
	uint8_t CRC8[2];
	//SCAN BUFFER
	for(uint16_t i=0;i<g_sci5_rx_count;i++)
	{
		//CALIB MODE RS485_CHECK
		if ((rx5_buff[i]== 'R')&&(rx5_buff[i+1]== 'S')&&(rx5_buff[i+10]== 'K'))
		{
			RS485_S_Ctr = 1U; //RS485 send mode
			sprintf(print_str,"RS485_OK");
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			memset(print_str, 0, sizeof(print_str));
			RS485_S_Ctr = 0U; //RS485 send mod
		}
		//CALIB MODE LED_BUZZER_CHECK
		else if ((rx5_buff[i]== 'L')&&(rx5_buff[i+1]== 'E')&&(rx5_buff[i+15]== 'K'))
		{
			RS485_S_Ctr = 1U; //RS485 send mode
			sprintf(print_str,"LED_BUZZER_ON");
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			memset(print_str, 0, sizeof(print_str));
			RS485_S_Ctr = 0U; //RS485 send mod
			MCU_LED_MCC = 0;
			MCU_LED_PSU = 0;
			MCU_LED_STT = 0;
			for(uint8_t i = 0;i<5;i++)
			{
				BUZZER =1;
				R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
				BUZZER =0;
				R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
			}

		}
		//CALIB MODE DATA_CHECK
		else if ((rx5_buff[i]== 'D')&&(rx5_buff[i+1]== 'A')&&(rx5_buff[i+9]== 'K'))
		{
//			measure();

			RS485_S_Ctr = 1U; //RS485 send mode
			sprintf(print_str,"\r\n%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d\r\n",
					(int16_t)MDC_regs[0x00],(int16_t)MDC_regs[0x01],(int16_t)MDC_regs[0x02],(int16_t)MDC_regs[0x03],
					(int16_t)MDC_regs[0x05],(int16_t)MDC_regs[0x06],(int16_t)MDC_regs[0x07],(int16_t)MDC_regs[0x08],
					(int16_t)MDC_regs[0x0A],(int16_t)MDC_regs[0x0B],MDC_regs[21],MDC_regs[22]);
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			memset(print_str, 0, sizeof(print_str));
			RS485_S_Ctr = 0U; //RS485 send mode
		}
		//RESET_CHARGE_TIME
		else if ((rx5_buff[i]== 'R')&&(rx5_buff[i+13]== 'T')&&(rx5_buff[i+6]== 'C'))
		{
			memset(data_time,0,sizeof(data_time));
//			data_time[3]=9;
//			data_time[7]=9;
			deviceFlash_erase4k(charge_discharge_time_addr);
			deviceFlash_writeData(charge_discharge_time_addr, data_time, 10);

			memset(data_time,0,sizeof(data_time));
			deviceFlash_readData(charge_discharge_time_addr, data_time, 10);
			charge_time_count = ((uint32_t)(data_time[0]<<24) +(uint32_t)(data_time[1]<<16)+(uint32_t)(data_time[2]<<8)+(uint32_t)(data_time[3]));
			discharge_time_count = ((uint32_t)(data_time[4]<<24) +(uint32_t)(data_time[5]<<16)+(uint32_t)(data_time[6]<<8)+(uint32_t)(data_time[7]));
			MCC_timeout_flag = data_time[9];

			sprintf(print_str,"\n\rCharge Time(ms)= %d Discharge Time(ms)= %d\r\n",charge_time_count,discharge_time_count);
			RS485_S_Ctr = 1U; //RS485 Master send mode
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			RS485_S_Ctr = 0U; //RS485 Master receive mode
			memset(print_str, 0, sizeof(print_str));


		}
		//RESET_OFFSET
		else if ((rx5_buff[i]== 'R')&&(rx5_buff[i+1]== 'E')&&(rx5_buff[i+6]== 'O'))
		{
			OFFSET_V1 =0;
			OFFSET_V2 =0;
			OFFSET_V3 =0;
			OFFSET_V4 =0;

			OFFSET_I1 =0;
			OFFSET_I2 =0;
			OFFSET_I3 =0;
			OFFSET_I4 =0;

			RS485_S_Ctr = 1U; //RS485 send mode
			sprintf(print_str,"\r\nV_Offset: %d %d %d %d\r\n",OFFSET_V1,OFFSET_V2,OFFSET_V3,OFFSET_V4);
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			memset(print_str, 0, sizeof(print_str));
			RS485_S_Ctr = 0U; //RS485 send mode

			RS485_S_Ctr = 1U; //RS485 send mode
			sprintf(print_str,"\r\nI_Offset: %d %d %d %d\r\n",OFFSET_I1,OFFSET_I2,OFFSET_I3,OFFSET_I4);
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			memset(print_str, 0, sizeof(print_str));
			RS485_S_Ctr = 0U; //RS485 send mode
		}
		//CALIB MODE CALIB_vvvvv_vvvvv_vvvvv_vvvvv_iiiii_iiiii_iiiii_iiiii
		else if ((rx5_buff[i]== 'C')&&(rx5_buff[i+1]== 'A')&&(rx5_buff[i+5]== '_'))
		{
			char tmp[6];

			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,(char*)rx5_buff+i+6,(uint8_t)5);
			CALIB_INIT_V1 = atoi(tmp);

			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,(char*)rx5_buff+i+12,(uint8_t)5);
			CALIB_INIT_V2 = atoi(tmp);

			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,(char*)rx5_buff+i+18,(uint8_t)5);
			CALIB_INIT_V3 = atoi(tmp);

			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,(char*)rx5_buff+i+24,(uint8_t)5);
			CALIB_INIT_V4 = atoi(tmp);

			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,(char*)rx5_buff+i+30,(uint8_t)5);
			CALIB_INIT_I1 = atoi(tmp);

			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,(char*)rx5_buff+i+36,(uint8_t)5);
			CALIB_INIT_I2 = atoi(tmp);

			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,(char*)rx5_buff+i+42,(uint8_t)5);
			CALIB_INIT_I3 = atoi(tmp);

			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,(char*)rx5_buff+i+48,(uint8_t)5);
			CALIB_INIT_I4 = atoi(tmp);

			RS485_S_Ctr = 1U; //RS485 send mode
			sprintf(print_str,"\r\nV_Calib: %d %d %d %d\r\n",CALIB_INIT_V1,CALIB_INIT_V2,CALIB_INIT_V3,CALIB_INIT_V4);
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			memset(print_str, 0, sizeof(print_str));
			RS485_S_Ctr = 0U; //RS485 send mode

			RS485_S_Ctr = 1U; //RS485 send mode
			sprintf(print_str,"\r\nI_Calib: %d %d %d %d\r\n",CALIB_INIT_I1,CALIB_INIT_I2,CALIB_INIT_I3,CALIB_INIT_I4);
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			memset(print_str, 0, sizeof(print_str));
			RS485_S_Ctr = 0U; //RS485 send mode


			if (Sample_done)
			{
				if(CALIB_INIT_V1<99999)
				{
					OFFSET_V1=0;
					OFFSET_V1 = CALIB_INIT_V1 - (uint16_t)(volt_measure(ADC_Volt1,1) );
					data_calib[0] = (uint8_t)(OFFSET_V1>>8);
					data_calib[1] = (uint8_t)(OFFSET_V1 & 0xFF);

				}
				if(CALIB_INIT_V2<99999)
				{
					OFFSET_V2=0;
					OFFSET_V2 = CALIB_INIT_V2 - (uint16_t)(volt_measure(ADC_Volt2,2) );
					data_calib[2] = (uint8_t)(OFFSET_V2>>8);
					data_calib[3] = (uint8_t)(OFFSET_V2 & 0xFF);
				}
				if(CALIB_INIT_V3<99999)
				{
					OFFSET_V3=0;
					OFFSET_V3 = CALIB_INIT_V3 - (uint16_t)(volt_measure(ADC_Volt3,3) );
					data_calib[4] = (uint8_t)(OFFSET_V3>>8);
					data_calib[5] = (uint8_t)(OFFSET_V3 & 0xFF);
				}
				if(CALIB_INIT_V4<99999)
				{
					OFFSET_V4=0;
					OFFSET_V4 = CALIB_INIT_V4 - (uint16_t)(volt_measure(ADC_Volt4,4) );
					data_calib[6] = (uint8_t)(OFFSET_V4>>8);
					data_calib[7] = (uint8_t)(OFFSET_V4 & 0xFF);
				}

				if(CALIB_INIT_I1<99999)
				{
					OFFSET_I1=0;
					OFFSET_I1 = CALIB_INIT_I1 - (int16_t)(current_measure(ADC_Curr1,ADC_CurrRef1,1));
					data_calib[8] = (uint8_t)(OFFSET_I1>>8);
					data_calib[9] = (uint8_t)(OFFSET_I1 & 0xFF);
				}
				if(CALIB_INIT_I2<99999)
				{
					OFFSET_I2=0;
					OFFSET_I2 = CALIB_INIT_I2 - (int16_t)(current_measure(ADC_Curr2,ADC_CurrRef2,2));
					data_calib[10] = (uint8_t)(OFFSET_I2>>8);
					data_calib[11] = (uint8_t)(OFFSET_I2 & 0xFF);
				}
				if(CALIB_INIT_I3<99999)
				{
					OFFSET_I3=0;
					OFFSET_I3 = CALIB_INIT_I3 - (int16_t)(current_measure(ADC_Curr3,ADC_CurrRef3,3));
					data_calib[12] = (uint8_t)(OFFSET_I3>>8);
					data_calib[13] = (uint8_t)(OFFSET_I3 & 0xFF);
				}
				if(CALIB_INIT_I4<99999)
				{
					OFFSET_I4=0;
					OFFSET_I4 = CALIB_INIT_I4 - (int16_t)(current_measure(ADC_Curr4,ADC_CurrRef4,4));
					data_calib[14] = (uint8_t)(OFFSET_I4>>8);
					data_calib[15] = (uint8_t)(OFFSET_I4 & 0xFF);
				}


				RS485_S_Ctr = 1U; //RS485 send mode
				sprintf(print_str,"\r\nV_Offset: %d %d %d %d\r\n",OFFSET_V1,OFFSET_V2,OFFSET_V3,OFFSET_V4);
				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
				memset(print_str, 0, sizeof(print_str));
				RS485_S_Ctr = 0U; //RS485 send mode

				RS485_S_Ctr = 1U; //RS485 send mode
				sprintf(print_str,"\r\nI_Offset: %d %d %d %d\r\n",OFFSET_I1,OFFSET_I2,OFFSET_I3,OFFSET_I4);
				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
				memset(print_str, 0, sizeof(print_str));
				RS485_S_Ctr = 0U; //RS485 send mode


				deviceFlash_erase4k(Offset_addr);
				deviceFlash_writeData(Offset_addr, data_calib, 16);
				deviceFlash_readData(Offset_addr, data_calib, 16);
				OFFSET_V1 = (uint16_t)(data_calib[0]<<8) +(uint16_t)data_calib[1];
				OFFSET_V2 = (uint16_t)(data_calib[2]<<8) +(uint16_t)data_calib[3];
				OFFSET_V3 = (uint16_t)(data_calib[4]<<8) +(uint16_t)data_calib[5];
				OFFSET_V4 = (uint16_t)(data_calib[6]<<8) +(uint16_t)data_calib[7];
				OFFSET_I1 = (uint16_t)(data_calib[8]<<8) +(uint16_t)data_calib[9];
				OFFSET_I2 = (uint16_t)(data_calib[10]<<8) +(uint16_t)data_calib[11];
				OFFSET_I3 = (uint16_t)(data_calib[12]<<8) +(uint16_t)data_calib[13];
				OFFSET_I4 = (uint16_t)(data_calib[14]<<8) +(uint16_t)data_calib[15];

				sprintf(print_str,"\n\r%d %d %d %d %d %d %d %d\r\n",OFFSET_V1, OFFSET_V2, OFFSET_V3, OFFSET_V4, OFFSET_I1, OFFSET_I2, OFFSET_I3, OFFSET_I4);
				RS485_S_Ctr = 1U; //RS485 Master send mode
				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
				RS485_S_Ctr = 0U; //RS485 Master receive mode
				memset(print_str, 0, sizeof(print_str));
				Sample_done=0;
			}

			RS485_S_Ctr = 1U; //RS485 send mode
			sprintf(print_str,"\r\nMDC_CALIB_DONE\r\n");
			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
			memset(print_str, 0, sizeof(print_str));
			RS485_S_Ctr = 0U; //RS485 send mod
			Buzzer(3, 80);
		}
		//WRITE MULTIPLE REGISTERS COMMAND - write TIME register 0x002C-0x002D (02 registers - 0x02)
		// 13 Bytes: Slave_add|Func|Add_Hi|Add_Lo|Quantity_Hi|Quantity_Lo|Byte_count|(Data_Hi|Data_lo)*Quantity|CRC*2
		else if ((rx5_buff[i]== MDC_ID)&&(rx5_buff[i+1]== 0x10)&&(rx5_buff[i+3]==0x2C)) //ID & WRITE MULTIPLE REGISTERS COMMAND
		{
			CRC16 = CRC16_bytewise(rx5_buff+i, 11);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx5_buff[i+11])&&(CRC8[1]== rx5_buff[i+12])) // CRC check & TIME Address check
			{
//				print test
//				sprintf(print_str,"\n\r   RS485 WRITE Add: 0x%02X%02X, 0x%02X%02X Registers, Byte count: 0x%02X \n\r",rx5_buff[2],rx5_buff[3],rx5_buff[4],rx5_buff[5],rx5_buff[6]);
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//				memset(print_str, 0, sizeof(print_str));

				//hour
				MDC_regs[44] = (uint16_t)(rx5_buff[i+7]<<8) + (uint16_t)(rx5_buff[i+8]);
				//Minute
				MDC_regs[45] = (uint16_t)(rx5_buff[i+9]<<8) + (uint16_t)(rx5_buff[i+10]);

				// Rs485 response to master
				memset(Rs485_MasterResponse, 0, sizeof(Rs485_MasterResponse));
				Rs485_MasterResponse[0] = MDC_ID;
				Rs485_MasterResponse[1] = 0x10;
				Rs485_MasterResponse[2] = rx5_buff[i+2]; // Starting address Hi
				Rs485_MasterResponse[3] = rx5_buff[i+3]; // Starting address Lo
				Rs485_MasterResponse[4] = rx5_buff[i+4]; // Number of written registers Hi
				Rs485_MasterResponse[5] = rx5_buff[i+5]; // Number of written registers Lo
				CRC16 = CRC16_bytewise(Rs485_MasterResponse, 6);
				Rs485_MasterResponse[6] = CRC16 & 0xff;
				Rs485_MasterResponse[7] = CRC16 >> 8;
				RS485_S_Ctr = 1U; //RS485 send mode
				R_SCI5_AsyncTransmit(Rs485_MasterResponse,8,transmit_ctrl);
				RS485_S_Ctr = 0U; //RS485 receive mode
				is_slave=0; // Switch to Master mode after response to MCC

				//print test
//				uint8_t i=0;
//				for(i=0;i<8;i++)
//				{
//					sprintf(print_str,"%02X ",Rs485_MasterResponse[i]);
//					R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//					memset(print_str, 0, sizeof(print_str));
//				}
				//end print test
			}
			else
			{
				// wrong CRC
			}
		}
		// READ MULTIPLE REGISTER 0x03 Command full function
		// 8 Bytes
		else if((rx5_buff[i+0]== MDC_ID)&&(rx5_buff[i+1]== 0x03)) //Slave address & Read command
		{
			CRC16 = CRC16_bytewise(rx5_buff+i, 6);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
//			sprintf(print_str,"\n\r CRC %02X %02X", CRC8[0], CRC8[1]);
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//			memset(print_str, 0, sizeof(print_str));

			if ((CRC8[0]== rx5_buff[i+6])&&(CRC8[1]== rx5_buff[i+7])) //CRC check
			{
				//print test
//				RS485_S_Ctr = 1U;
//				sprintf(print_str,"\n\r   RS485 READ Add: 0x%02X%02X, 0x%02X%02X Registers!\n\r",rx5_buff[2],rx5_buff[3],rx5_buff[4],rx5_buff[5]);
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//				memset(print_str, 0, sizeof(print_str));
//				RS485_S_Ctr = 0U;
				// end print test

				memset(Rs485_MasterResponse, 0, sizeof(Rs485_MasterResponse));
				// Rs485 response to master
				Rs485_MasterResponse[0] = MDC_ID;
				Rs485_MasterResponse[1] = 0x03;
				Rs485_MasterResponse[2] = rx5_buff[i+5]*2; // 104 *2 =208 = 0xD0

				for(uint16_t j=0;j<rx5_buff[i+5];j++)
				{
					uint16_to_uint8(MDC_regs[rx5_buff[i+3]+j], (Rs485_MasterResponse +((j*2)+3)), (Rs485_MasterResponse +((j*2)+4)));
				}

				CRC16 = CRC16_bytewise(Rs485_MasterResponse, rx5_buff[i+5]*2+3);
				Rs485_MasterResponse[rx5_buff[i+5]*2+3] = CRC16 & 0xff;
				Rs485_MasterResponse[rx5_buff[i+5]*2+4] = CRC16 >> 8;
				RS485_S_Ctr = 1U; //RS485 send mode
				R_SCI5_AsyncTransmit(Rs485_MasterResponse,rx5_buff[i+5]*2+5,transmit_ctrl);
				RS485_S_Ctr = 0U; //RS485 receive mode
				is_slave=0;

				//print test
//						for(uint8_t i=0;i<105;i++)
//						{
//							sprintf(print_str,"%02X ",Rs485_MasterResponse[i]);
//							RS485_S_Ctr = 1U;
//							R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//							RS485_S_Ctr = 0U;
//							memset(print_str, 0, sizeof(print_str));
//						}
				// end print test
			}
			else
			{
				//wrong CRC
			}
		}
		// WRITE SINGLE REGISTER: 0x002F-READ MODE, 0x0065 - Bootloader Control
		// 8 Bytes
		else if((rx5_buff[i+0]== MDC_ID)&&(rx5_buff[i+1]== 0x06)) // write 1 register
		{
			CRC16 = CRC16_bytewise(rx5_buff+i, 6);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;

			if ((CRC8[0]== rx5_buff[i+6])&&(CRC8[1]== rx5_buff[i+7])) //CRC check && Address check
			{
				//Read mode
				MDC_regs[rx5_buff[i+3]] = (uint16_t)(rx5_buff[i+4]<<8) + (uint16_t)(rx5_buff[i+5]);
				if(MDC_regs[99]==0) transmit_ctrl=0;

				// Rs485 response to master
				memset(Rs485_MasterResponse, 0, sizeof(Rs485_MasterResponse));
				Rs485_MasterResponse[0] = MDC_ID;
				Rs485_MasterResponse[1] = 0x06;
				Rs485_MasterResponse[2] = rx5_buff[i+2]; // Starting address Hi
				Rs485_MasterResponse[3] = rx5_buff[i+3]; // Starting address Lo
				Rs485_MasterResponse[4] = rx5_buff[i+4]; // Number of written registers Hi
				Rs485_MasterResponse[5] = rx5_buff[i+5]; // Number of written registers Lo
				CRC16 = CRC16_bytewise(Rs485_MasterResponse, 6);
				Rs485_MasterResponse[6] = CRC16 & 0xff;
				Rs485_MasterResponse[7] = CRC16 >> 8;

				RS485_S_Ctr = 1U; //RS485 send mode
				R_SCI5_AsyncTransmit(Rs485_MasterResponse,8,transmit_ctrl);
				RS485_S_Ctr = 0U; //RS485 receive mode
				is_slave=0; // Switch to Master mode after response to MCC
			}
			else
			{
				//wrong CRC
			}
		}

		//WRITE MULTIPLE REGISTERS COMMAND - write 1 register 0x002E DC LOW threshold
		// 11 Bytes: Slave_add|Func|Add_Hi|Add_Lo|Quantity_Hi|Quantity_Lo|Byte_count|Data_Hi|Data_lo|CRC*2
		else if((rx5_buff[i+0]== MDC_ID)&&(rx5_buff[i+1]== 0x10)&&(rx5_buff[i+3]==0x2E)) // Write multiple register command
		{
			CRC16 = CRC16_bytewise(rx5_buff+i, 9); // Calculate CRC of 9 bytes/11 received bytes
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;

			if ((CRC8[0]== rx5_buff[i+9])&&(CRC8[1]== rx5_buff[i+10])) //CRC check && Address check
			{
				MDC_regs[46] = (uint16_t)(rx5_buff[i+7]<<8) + (uint16_t)(rx5_buff[i+8]);

				//Rs485 response to master
				memset(Rs485_MasterResponse, 0, sizeof(Rs485_MasterResponse));
				Rs485_MasterResponse[0] = MDC_ID;
				Rs485_MasterResponse[1] = 0x10;
				Rs485_MasterResponse[2] = rx5_buff[i+2]; // Starting address Hi
				Rs485_MasterResponse[3] = rx5_buff[i+3]; // Starting address Lo
				Rs485_MasterResponse[4] = rx5_buff[i+4]; // Number of written registers Hi
				Rs485_MasterResponse[5] = rx5_buff[i+5]; // Number of written registers Lo
				CRC16 = CRC16_bytewise(Rs485_MasterResponse, 6);
				Rs485_MasterResponse[6] = CRC16 & 0xff;
				Rs485_MasterResponse[7] = CRC16 >> 8;
				RS485_S_Ctr = 1U; //RS485 send mode
				R_SCI5_AsyncTransmit(Rs485_MasterResponse,8,transmit_ctrl);
				RS485_S_Ctr = 0U; //RS485 receive mode
				is_slave=0; // Switch to Master mode after response to MCC
			}
		}
	}
	g_sci5_rx_count=0;
	memset(rx5_buff, 0, sizeof(rx5_buff));
	R_Config_SCI5_Serial_Receive((uint8_t*)&rx5_buff, sizeof(rx5_buff));
}
/************************************************************************
 * Function Name: RS485_M_Read_and_Receive
 * Description  : This function is Modbus 03 Command from Master
 * @param slaveID: ID of slave device
 * @param Register: register address
 * @param request: buffer to store request massage
 * @param Value: Value of the requested register from Slave
 **************************************************************************/
void RS485_M_Read_and_Receive(uint8_t slaveID, uint16_t Register, uint8_t *request, uint16_t * Value) //Read value stores in Value
{
	memset(request, 0, sizeof(request));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x03;
	//address register
	uint16_to_uint8(Register, request+2, request+3);
	*(request+4) = 0;
	*(request+5) = 1;
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;
	R_Config_SCI1_Serial_Receive((uint8_t*)&rx1_buff, 7);
	RS485_M_Ctr = 1U; //RS485 send mode
	R_SCI1_AsyncTransmit(request,8);
	RS485_M_Ctr = 0U; //RS485 receive mode
	uint32_t lasttick = tick;
	while((!SCI1_rxdone) && (tick-lasttick < 100));
	if(SCI1_rxdone==1)
	{
		//print test
//		uint8_t i=0;
//		for(i=0;i<7;i++)
//		{
//			sprintf(print_str,"%02X ",rx1_buff[i]);
//			R_SCI1_AsyncTransmit((uint8_t*)print_str,3);
//			memset(print_str, 0, sizeof(print_str));
//		}
		if((rx1_buff[0]== slaveID)&&(rx1_buff[1]==0x03))
		{
			CRC16 = CRC16_bytewise(rx1_buff, 5);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx1_buff[5])&&(CRC8[1]== rx1_buff[6])) //CRC check
			{

				PSU_connect_flag=1; // PSU Connected
				*Value = (uint16_t)(rx1_buff[3]<<8) + (uint16_t)(rx1_buff[4]);
				//print test
//				sprintf(print_str,"\n\r  Value of %d register: %d \n\r",Register, *Value);
//				R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				memset(print_str, 0, sizeof(print_str));
				//end print test
			}
		}
		else
		{
			//wrong CRC
		}
		SCI1_rxdone =0;
	}
	else
	{
		// for timeout counting
		PSU_connect_flag=0; //PSU not connected
	}
}
void RS485_M_Read_Batt(uint8_t slaveID, uint16_t Start_Add, uint16_t NbRgt ,uint8_t *request, uint16_t * BattRegs) //Store message in Response
{
	memset(request, 0, sizeof(request));
	memset(BattRegs, 0, sizeof(BattRegs));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x03;
	//address register
	uint16_to_uint8(Start_Add, request+2, request+3);
	uint16_to_uint8(NbRgt, request+4, request+5);
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;

	R_Config_SCI1_Serial_Receive((uint8_t*)&rx1_buff, (5+NbRgt*2));
	RS485_M_Ctr = 1U; //RS485 send mode
	R_SCI1_AsyncTransmit(request,8);
	RS485_M_Ctr = 0U; //RS485 receive mode

	uint32_t lasttick = tick;
	//print test
//	sprintf(print_str,"\n\r SEND: ");
//	RS485_S_Ctr = 1U; //RS485 send mode
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//	RS485_S_Ctr = 0U; //RS485 receive mode
//	memset(print_str, 0, sizeof(print_str));
//	for(uint8_t i=0;i<8;i++)
//	{
//		sprintf(print_str,"%02X ",request[i]);
//		RS485_S_Ctr = 1U; //RS485 send mode
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//		RS485_S_Ctr = 0U; //RS485 receive mode
//		memset(print_str, 0, sizeof(print_str));
//	}
	//end_print_test

	while((!SCI1_rxdone) && (tick-lasttick< NbRgt*10));
	if(SCI1_rxdone==1)
	{
		//print test
//		sprintf(print_str,"\n\r RECEIVE: ");
//			RS485_S_Ctr = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//			RS485_S_Ctr = 0U; //RS485 receive mode
//			memset(print_str, 0, sizeof(print_str));
//		for(uint8_t i=0;i<(5+NbRgt*2);i++)
//		{
//			sprintf(print_str,"%02X ",rx1_buff[i]);
//			RS485_S_Ctr = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//			RS485_S_Ctr = 0U; //RS485 receive mode
//			memset(print_str, 0, sizeof(print_str));
//		}
		//end_print_test
		if((rx1_buff[0]== slaveID)&&(rx1_buff[1]==0x03))
		{
			CRC16 = CRC16_bytewise(rx1_buff, 3+NbRgt*2);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx1_buff[3+NbRgt*2])&&(CRC8[1]== rx1_buff[4+NbRgt*2])) //CRC check
			{
				for(uint16_t i=0;i<NbRgt;i++)
				{
					BattRegs[i] = (uint16_t)(rx1_buff[i*2+3]<<8) + (uint16_t)(rx1_buff[i*2+4]);
//					if (BattRegs[i]>32768) BattRegs[i]=0;

					//print test
//					sprintf(print_str,"%02X ",BattRegs[i]);
//					RS485_M_Ctr = 1U; //RS485 send mode
//					R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//					RS485_M_Ctr = 0U; //RS485 receive mode
//					memset(print_str, 0, sizeof(print_str));
					//end_print_test
				}
				if(Start_Add == 0xA731) //0xA731
				{
					MDC_regs[0] = ((BattRegs[1]==65535)?0:BattRegs[1])*10;
					MDC_regs[5] = ((BattRegs[3]==65535)?0:BattRegs[3]);
					MDC_regs[23] = ((BattRegs[8]==65535)?0:BattRegs[8]);
					MDC_regs[24] = ((BattRegs[53]==65535)?0:BattRegs[53]);
					MDC_regs[25] = ((BattRegs[54]==65535)?0:BattRegs[54]);
					MDC_regs[26] = ((BattRegs[9]==65535)?0:BattRegs[9]);
					MDC_regs[50] = ((BattRegs[56]==65535)?0:BattRegs[56]);

					MDC_regs[1] = ((BattRegs[65]==65535)?0:BattRegs[65])*10;
					MDC_regs[6] = ((BattRegs[67]==65535)?0:BattRegs[67]);
					MDC_regs[28] = ((BattRegs[72]==65535)?0:BattRegs[72]);
					MDC_regs[29] = ((BattRegs[117]==65535)?0:BattRegs[117]);
					MDC_regs[30] = ((BattRegs[118]==65535)?0:BattRegs[118]);
					MDC_regs[31] = ((BattRegs[73]==65535)?0:BattRegs[73]);
					MDC_regs[51] = ((BattRegs[120]==65535)?0:BattRegs[120]);

					MDC_regs[12] = ((BattRegs[58]==65535)?0:(BattRegs[58]/100));
					MDC_regs[13] = ((BattRegs[122]==65535)?0:(BattRegs[122]/100));


//					//print test
//					sprintf(print_str,"\r\n0: %d 5: %d 23: %d 24: %d 25: %d 26: %d",MDC_regs[0],MDC_regs[5],MDC_regs[23],MDC_regs[24],MDC_regs[25],MDC_regs[26]);
//					RS485_M_Ctr = 1U; //RS485 send mode
//					R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//					RS485_M_Ctr = 0U; //RS485 receive mode
//					memset(print_str, 0, sizeof(print_str));
//					//end_print_test
//
//					//print test
//					sprintf(print_str,"\r\n1: %d 6: %d 28: %d 29: %d 30: %d 31: %d",MDC_regs[1],MDC_regs[6],MDC_regs[28],MDC_regs[29],MDC_regs[30],MDC_regs[31]);
//					RS485_M_Ctr = 1U; //RS485 send mode
//					R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//					RS485_M_Ctr = 0U; //RS485 receive mode
//					memset(print_str, 0, sizeof(print_str));
//					//end_print_test
				}
				else if(Start_Add == 0xA7B1) //0xA7B1
				{
					MDC_regs[2] = ((BattRegs[1]==65535)?0:BattRegs[1])*10;
					MDC_regs[7] = ((BattRegs[3]==65535)?0:BattRegs[3]);
					MDC_regs[33] = ((BattRegs[8]==65535)?0:BattRegs[8]);
					MDC_regs[34] = ((BattRegs[53]==65535)?0:BattRegs[53]);
					MDC_regs[35] = ((BattRegs[54]==65535)?0:BattRegs[54]);
					MDC_regs[36] = ((BattRegs[9]==65535)?0:BattRegs[9]);
					MDC_regs[52] = ((BattRegs[56]==65535)?0:BattRegs[56]);

					MDC_regs[38] = ((BattRegs[72]==65535)?0:BattRegs[72]);
					MDC_regs[39] = ((BattRegs[117]==65535)?0:BattRegs[117]);
					MDC_regs[40] = ((BattRegs[118]==65535)?0:BattRegs[118]);
					MDC_regs[41] = ((BattRegs[73]==65535)?0:BattRegs[73]);

					MDC_regs[14] = ((BattRegs[58]==65535)?0:(BattRegs[58]/100));

//					//print test
//					sprintf(print_str,"\r\n2: %d 7: %d 33: %d 34: %d 35: %d 36: %d",MDC_regs[2],MDC_regs[7],MDC_regs[33],MDC_regs[34],MDC_regs[35],MDC_regs[36]);
//					RS485_M_Ctr = 1U; //RS485 send mode
//					R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//					RS485_M_Ctr = 0U; //RS485 receive mode
//					memset(print_str, 0, sizeof(print_str));
//					//end_print_test
				}
				else if(Start_Add == 0x1000)
				{
//					MDC_regs[3] = ((BattRegs[0]==65535)?0:BattRegs[0]);
//					MDC_regs[8] = ((BattRegs[1]==65535)?0:BattRegs[1]);
					MDC_regs[10] = ((BattRegs[13]==65535)?0:(BattRegs[13]/1000));
//					MDC_regs[15] = ((BattRegs[21]==65535)?0:(BattRegs[21]/100));
					MDC_regs[48] = ((BattRegs[6]==65535)?0:BattRegs[6]);
					MDC_regs[49] = ((BattRegs[9]==65535)?0:BattRegs[9]);
					MDC_regs[16] = ((BattRegs[19]==65535)?0:(BattRegs[19]/100));

//					//print test
//					sprintf(print_str,"\r\n3: %d 8: %d 10: %d 15: %d 48: %d 49: %d",MDC_regs[3],MDC_regs[8],MDC_regs[10],MDC_regs[15],MDC_regs[48],MDC_regs[49]);
//					RS485_M_Ctr = 1U; //RS485 send mode
//					R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//					RS485_M_Ctr = 0U; //RS485 receive mode
//					memset(print_str, 0, sizeof(print_str));
//					//end_print_test
				}
			}
			else
			{
				//wrong CRC
			}
		}
		SCI1_rxdone =0;
	}
}
void RS485_M_Cmd04_and_Receive(uint8_t slaveID, uint32_t StartAdd, uint16_t NoR, uint16_t * Value) //Read value stores in Value
{
	uint8_t request[8];
	memset(request, 0, sizeof(request));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x04;
	//address register
	uint16_to_uint8(StartAdd, request+2, request+3);
	uint16_to_uint8(NoR, request+4, request+5);
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;
	R_Config_SCI1_Serial_Receive((uint8_t*)&rx1_buff, (NoR*2+5));
	RS485_M_Ctr = 1U; //RS485 send mode
	R_SCI1_AsyncTransmit(request,8);
	RS485_M_Ctr = 0U; //RS485 receive mode
	uint32_t lasttick = tick;

	while((!SCI1_rxdone) && (tick-lasttick<NoR*10));
	if (SCI1_rxdone ==1)
	{
		//print test
//		uint8_t i=0;
//		for(i=0;i<(NoR*2+5);i++)
//		{
//			sprintf(print_str,"%02X ",rx5_buff[i]);
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,3);
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
//		}
		//end_print_test

		if((rx1_buff[0]== slaveID)&&(rx1_buff[1]==0x04))
		{
			CRC16 = CRC16_bytewise((uint8_t*) rx1_buff, NoR*2+3);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			//print test
//			sprintf(print_str,"\r\n CRC: %02X %02X  MSG: %02X %02X \r\n",CRC8[0],CRC8[1],rx5_buff[(NoR*2+3)],rx5_buff[(NoR*2+4)]);
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
			//end_print_test

			if ((CRC8[0]== rx1_buff[(NoR*2+3)])&&(CRC8[1]== rx1_buff[(NoR*2+4)])) //CRC check
			{
				for(uint8_t i=0;i<NoR;i++)
				{
					Value[i] = (uint16_t)(rx1_buff[(i*2)+3]<<8) + (uint16_t)(rx1_buff[(i*2)+4]);
				}

				//print test
//				sprintf(print_str,"\n\r  Value: %d \n\r",Value[22]);
//				RS485_DE2 = 1U; //RS485 send mode
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U; //RS485 send mode
//				memset(print_str, 0, sizeof(print_str));
				//end_print_test
			}
			else
			{
				//wrong CRC
			}
		}
		SCI1_rxdone =0;
	}
	else// timeout
	{

	}
}
void RS485_Master_Mode()
{
	switch(MDC_regs[103])
	{
	case 1:
		//Tu nguon Huawei
		RS485_M_Read_and_Receive(ID_Box_Huawei, 0x1103, Rs485_RequestToSlave, &MDC_regs[19]);
		RS485_M_Read_and_Receive(ID_Box_Huawei, 0x1101, Rs485_RequestToSlave, &MDC_regs[17]);
		RS485_M_Read_and_Receive(ID_Box_Huawei, 0x1100, Rs485_RequestToSlave, &MDC_regs[18]);
		//Battery 1 2 Parameters
		RS485_M_Read_Batt(ID_Box_Huawei, 0xA731, 125 ,Rs485_RequestToSlave, BattRegs); //Battery 1 2 Regs 0xA731
		//Battery 3 4 Parameters
		RS485_M_Read_Batt(ID_Box_Huawei, 0xA7B1, 125 ,Rs485_RequestToSlave, BattRegs); //Battery 3 4 Regs 0xA7B1
		RS485_M_Read_Batt(ID_Box_Huawei, 0x1000, 22 ,Rs485_RequestToSlave, BattRegs5);

		RS485_M_Read_Batt(ID_Box_Huawei,  0xA731, 181 ,Rs485_RequestToSlave, MDC_regs +130);
		break;
	case 2:
		// ACCU Vision
		RS485_M_Read_and_Receive(ID_Accu_Vision, 0x0000, Rs485_RequestToSlave, &MDC_regs[2]);
		RS485_M_Read_and_Receive(ID_Accu_Vision, 0x0001, Rs485_RequestToSlave, &MDC_regs[7]);
		RS485_M_Read_and_Receive(ID_Accu_Vision, 0x0019, Rs485_RequestToSlave, &MDC_regs[36]);
		RS485_M_Read_and_Receive(ID_Accu_Vision, 0x0023, Rs485_RequestToSlave, &MDC_regs[34]);
		RS485_M_Read_and_Receive(ID_Accu_Vision, 0x0024, Rs485_RequestToSlave, &MDC_regs[33]);
		RS485_M_Read_and_Receive(ID_Accu_Vision, 0x0025, Rs485_RequestToSlave, &MDC_regs[35]);

		RS485_M_Read_Batt(ID_Accu_Vision,  0x00, 181 ,Rs485_RequestToSlave, MDC_regs +130);
		break;

	case 3:
		// ACCU SHOTO
		RS485_M_Cmd04_and_Receive(ID_Accu_Shoto, 1000,10, BattRegs);
		MDC_regs[0] = BattRegs[0];
		MDC_regs[5] = BattRegs[1];
		MDC_regs[23] = BattRegs[5];
		MDC_regs[24] = BattRegs[6];
		MDC_regs[26] = BattRegs[9]-2730;
		MDC_regs[50] = BattRegs[2];
		RS485_M_Cmd04_and_Receive(ID_Accu_Shoto, 3040,1, BattRegs);
		MDC_regs[25] = BattRegs[0];
		break;
	case 4:
		// accu vision 100A
		RS485_M_Read_Batt(ID_Accu_Vision,  0x00, 181 ,Rs485_RequestToSlave, MDC_regs +130);
		break;

	case 5:
		//ACCU Huawei
		RS485_M_Read_and_Receive(ID_Accu_Huawei, 0x0000, Rs485_RequestToSlave, &MDC_regs[0]);
		RS485_M_Read_and_Receive(ID_Accu_Huawei, 0x0002, Rs485_RequestToSlave, &MDC_regs[5]);
		RS485_M_Read_and_Receive(ID_Accu_Huawei, 0x0204, Rs485_RequestToSlave, &MDC_regs[12]);
		RS485_M_Read_and_Receive(ID_Accu_Huawei, 0x0043, Rs485_RequestToSlave, &MDC_regs[22]);
		RS485_M_Read_and_Receive(ID_Accu_Huawei, 0x0003, Rs485_RequestToSlave, &MDC_regs[23]);
		RS485_M_Read_and_Receive(ID_Accu_Huawei, 0x004A, Rs485_RequestToSlave, &MDC_regs[25]);
		RS485_M_Read_and_Receive(ID_Accu_Huawei, 0x0005, Rs485_RequestToSlave, &MDC_regs[26]);
		RS485_M_Read_and_Receive(ID_Accu_Huawei, 0x0107, Rs485_RequestToSlave, &MDC_regs[50]);
		break;
	case 6:
		//Narada        48NPFC100
		RS485_M_Read_and_Receive(ID_Accu_Narada, 0x0FFF, Rs485_RequestToSlave, &MDC_regs[0]);
		RS485_M_Read_and_Receive(ID_Accu_Narada, 0x1000, Rs485_RequestToSlave, &MDC_regs[5]);
		RS485_M_Read_and_Receive(ID_Accu_Narada, 0x1007, Rs485_RequestToSlave, &MDC_regs[23]);
		RS485_M_Read_and_Receive(ID_Accu_Narada, 0x1009, Rs485_RequestToSlave, &MDC_regs[24]);
		RS485_M_Read_and_Receive(ID_Accu_Narada, 0x1002, Rs485_RequestToSlave, &MDC_regs[26]);
		RS485_M_Read_Batt(ID_Accu_Narada,  0xFFF, 181 ,Rs485_RequestToSlave, MDC_regs +130);
		break;

	case 7:
		//accu ztt4850
		RS485_M_Read_and_Receive(ID_ZZT4850, 0, Rs485_RequestToSlave, &MDC_regs[0]);
		RS485_M_Read_and_Receive(ID_ZZT4850, 1, Rs485_RequestToSlave, &MDC_regs[5]);
		break;

	case 8:
		//accu vision 100A_VN
		RS485_M_Read_Batt(ID_Accu_Vision,  0x00, 181 ,Rs485_RequestToSlave, MDC_regs +130);
		break;

	case 9:
		//accu postef
		break;

	case 10:
		//tủ nguồn POSTEF CSU501B (SNMP)
		break;
	}
}

void measure()
{
	uint32_t lasttick = tick;
	while(!Sample_done && (tick-lasttick<200));
	if (Sample_done)
	{
		MDC_regs[0] = (uint16_t)(volt_measure(ADC_Volt1,1) );
		MDC_regs[1] = (uint16_t)(volt_measure(ADC_Volt2,2) );
		MDC_regs[2] = (uint16_t)(volt_measure(ADC_Volt3,3) );
		MDC_regs[3] = (uint16_t)(volt_measure(ADC_Volt4,4) );

		MDC_regs[5] = (uint16_t)abs((current_measure(ADC_Curr1,ADC_CurrRef1,1)));
		if(MDC_regs[5]<100) MDC_regs[5]=0;
		MDC_regs[6] = (uint16_t)abs((current_measure(ADC_Curr2,ADC_CurrRef2,2)));
		if(MDC_regs[6]<100) MDC_regs[6]=0;
		MDC_regs[7] = (uint16_t)abs((current_measure(ADC_Curr3,ADC_CurrRef3,3)));
		if(MDC_regs[7]<100) MDC_regs[7]=0;
		MDC_regs[8] = (uint16_t)abs((current_measure(ADC_Curr4,ADC_CurrRef4,4)));
		if(MDC_regs[8]<100) MDC_regs[8]=0;


		MDC_regs[10] = (uint16_t)temp_measure(ADC_Temp1,1);
		MDC_regs[11] = (uint16_t)temp_measure(ADC_Temp2,2);

		MDC_regs[12] = (MDC_regs[0]/100)*(MDC_regs[5]/100);
		MDC_regs[13] = (MDC_regs[1]/100)*(MDC_regs[6]/100);
		MDC_regs[14] = (MDC_regs[2]/100)*(MDC_regs[7]/100);
		MDC_regs[15] = (MDC_regs[3]/100)*(MDC_regs[8]/100);

		//print test
//		RS485_M_Ctr = 1U; //RS485 Master send mode
		//TEMP + volt + Current
//		sprintf(print_str,"\nTemp1: %d Temp2: %d Volt1: %d Volt2: %d Volt3: %d Volt4: %d MDC_regs[5]: %d MDC_regs[6]: %d MDC_regs[7]: %d MDC_regs[8]: %d\n", MDC_regs[0x0A],MDC_regs[0x0B], MDC_regs[0],MDC_regs[1],MDC_regs[2],MDC_regs[3], MDC_regs[5],MDC_regs[6],MDC_regs[7],MDC_regs[8]);
		//test Voltage
//		sprintf(print_str,"\n %d %d %d %d Volt1: %d Volt2: %d Volt3: %d Volt4: %d\n", ADC_Volt1[SAMPLES_NUM-1], ADC_Volt2[SAMPLES_NUM-1], ADC_Volt3[SAMPLES_NUM-1], ADC_Volt4[SAMPLES_NUM-1], MDC_regs[0],MDC_regs[1],MDC_regs[2],MDC_regs[3]);
		//test current
//		sprintf(print_str,"\n Curr1: %d Curr2: %d Curr3: %d Curr4: %d\n", MDC_regs[5],MDC_regs[6],MDC_regs[7],MDC_regs[8]);
//		R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//		RS485_M_Ctr = 0U; //RS485 Master receive mode

//		RS485_S_Ctr = 1U; //RS485 Master send mode
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//		RS485_S_Ctr = 0U; //RS485 Master send mode
//		memset(print_str, 0, sizeof(print_str));
		//end print test

		Sample_done=0;
	}

}

float temp_measure(uint16_t *ADC_value, uint8_t channel)
{
	volatile uint16_t LUT_i=0,LUT_pos=0,i;
	float temp=0,avg;
	for(i=0;i<SAMPLES_NUM;i++)
	{
		avg +=  ADC_value[i];

	}
	avg = avg/SAMPLES_NUM;

	//print test
//	RS485_S_Ctr = 1U; //RS485 Master send mode
//	sprintf(print_str,"\n Avg%d: %.2f \n", channel, avg);
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//	RS485_S_Ctr = 0U; //RS485 Master send mode
//	memset(print_str, 0, sizeof(print_str));
	//end print test

	if(avg > 3431) temp=0;
	else if(avg <45) temp = 9999.99;
	else
	{
		LUT_i=0;
		while((avg<= ADC_LUT[LUT_i])&&(LUT_i<180))
		{
			LUT_pos=LUT_i;
			LUT_i++;
		}
		//print test
//		RS485_S_Ctr = 1U; //RS485 Master send mode
//		sprintf(print_str,"\n LUT%d: %d \n", channel,ADC_LUT[LUT_pos]);
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//		RS485_S_Ctr = 0U; //RS485 Master send mode
//		memset(print_str, 0, sizeof(print_str));
		//end print test
		if(LUT_pos==0) temp = 0;
		else
		{
			temp = (avg - ADC_LUT[LUT_pos]);
			temp= temp/(ADC_LUT[LUT_pos-1]-ADC_LUT[LUT_pos]);
			temp = LUT_pos-temp-10;
		}
	}
	if(channel==1)
	{
		temp = (temp + OFFSET_TEMP1)*100;
	}
	if(channel==2)
	{
		temp = (temp + OFFSET_TEMP2)*100;
	}
	//print test
//		RS485_S_Ctr = 1U; //RS485 Master send mode
//		sprintf(print_str,"\n\r temp%d: %f \n\r", channel,temp);
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//		RS485_S_Ctr = 0U; //RS485 Master send mode
//		memset(print_str, 0, sizeof(print_str));
	//end print test

	return temp;
}

float volt_measure(uint16_t *ADC_value, uint8_t channel)
{
	float volt=0;
	for(uint16_t i=0;i<SAMPLES_NUM;i++)
	{
		volt = volt+  (ADC_value[i])*0.083-255.1;
	}
	volt = volt/SAMPLES_NUM;
	if(volt<0) volt = -volt;

	if(channel ==1)
	{
		if(volt<35)
		{
			volt =  0.9925*volt +0.4511;
		}
		volt= volt* 100 + OFFSET_V1;
	}
	else if(channel ==2)
	{
		if(volt<35)
		{
			volt =  0.9834*volt + 0.3759;
		}
		volt= volt* 100 + OFFSET_V2;
	}
	else if(channel ==3)
	{
		if(volt<35)
		{
			volt =  1.0018*volt -0.2621;
		}
		volt= volt* 100 + OFFSET_V3;
	}
	else if(channel ==4)
	{
		if(volt<35)
		{
			volt =  0.9901*volt +0.5277;
		}
		volt= volt* 100 + OFFSET_V4;
	}

	if(volt<2000) volt =0;
	//print test
//		RS485_S_Ctr = 1U; //RS485 Master send mode
//		sprintf(print_str,"\n\r volt%d: %.2f \n\r", channel,volt);
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//		RS485_S_Ctr = 0U; //RS485 Master send mode
//		memset(print_str, 0, sizeof(print_str));
	//end print test

	return volt;
}
float current_measure(uint16_t *ADC_value,uint16_t *ADC_Vref, uint8_t channel) //channel 1-4
{
	float rms_current =0;
	float Vref =CURR_SENSOR_TH;
	uint16_t i=0;


	for(i=0;i<SAMPLES_NUM;i++)
	{
		rms_current +=  (ADC_value[i]*3.3/4096);
//		Vref +=  (ADC_Vref[i]*3.3/4096);

	}
	rms_current = rms_current/SAMPLES_NUM;
//	Vref = Vref/SAMPLES_NUM;
//	//print test
//		RS485_S_Ctr = 1U; //RS485 Master send mode
//		sprintf(print_str,"\n\r curr_ADC%d: %.2f \n\r", channel,rms_current);
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//		RS485_S_Ctr = 0U; //RS485 Master send mode
//		memset(print_str, 0, sizeof(print_str));
//	//end print test

//	if(rms_current>CURR_SENSOR_TH) rms_current = (rms_current-CURR_SENSOR_TH)/0.015; // step 15mV/A
	if(rms_current>=1) //Sensor is plugged in
	{
		if(rms_current>Vref) //Discharge Current
		{
			charge_discharge_sts =0;
			rms_current = (rms_current-Vref)/0.01; // step 10mV/A

			//CALIB
			if(channel==1)
			{
				//CALIB theo điện áp
				if (rms_current < 8.5)  rms_current= 0.9328*(0.9995*(0.996*rms_current - 1.1422) + 0.002108) + 0.41139;
				else if (rms_current < 19 &&  rms_current > 8.5) rms_current= 1.0481 * (0.9287591684 * rms_current - 0.54253252992) - 0.27373;
				else if (rms_current < 26.5 &&  rms_current > 19) rms_current= 1.0122 * rms_current - 1.21456;
				else  rms_current = 0.9799*rms_current - 0.8912;
				// CALIB THEO SENSOR
//				if (rms_current< 7) rms_current= 0.9513*rms_current + 0.12653 - 0.55;
//				else if (rms_current < 23 &&  rms_current > 8) rms_current = 0.9907 * (0.9799*rms_current - 0.4712) - 0.08856;
//					else  rms_current = 0.9799*rms_current - 0.4712;
				rms_current = rms_current*100 + OFFSET_I1;
			}
			else if(channel==2)
			{
				if (rms_current<8.5)	rms_current= 1.0255*(0.9982*(0.9479*rms_current - 0.66263) + 0.002926) - 0.13522;
				else if (rms_current < 19 &&  rms_current > 8.5) rms_current= 1.0085 * (0.9888763225 * rms_current - 0.40057670525) - 0.43182;
				else if (rms_current < 26.5 &&  rms_current > 19) rms_current= 0.9906 * rms_current - 0.83595;
				else  rms_current = 1.0055*rms_current - 1.1241;
				//CALIB THEO SENSOR
//				if (rms_current<7)	rms_current= 0.9577*rms_current + 0.22085 - 0.55;
//				else if (rms_current < 23 &&  rms_current > 8) rms_current = 1.0044 * (1.0055*rms_current - 1.1241) + 0.27506;
//					else  rms_current = 1.0055*rms_current - 1.1241;

				rms_current = rms_current*100 + OFFSET_I2;
			}
			else if(channel==3)
			{
				if (rms_current<8.5) rms_current= 0.964*(0.9999*(0.9868*rms_current - 1.0048) - 0.002822)+0.23661;
				else if (rms_current < 19 &&  rms_current > 8.5) rms_current= 0.9997 * (1.00977598462 * rms_current - 0.45463268081) - 0.70955;
				else if (rms_current < 26.5 &&  rms_current > 19) rms_current= 1.0154 * rms_current - 1.34914;
				else  rms_current = 0.9846*rms_current - 0.79;
				//CALIB THEO SENSOR
//				if (rms_current<7)	rms_current= 0.9602*rms_current - 0.1140 - 0.55;
//				else if (rms_current < 23 &&  rms_current > 8) rms_current = 1.01 * (1.0055*rms_current - 1.1241) + 0.02917;
//					else  rms_current = 1.0055*rms_current - 1.1241;

				rms_current = rms_current*100 + OFFSET_I3;
			}
			else if(channel==4)
			{
				if (rms_current<8.5)	rms_current= 1.01387*(0.9958*(0.9465*rms_current - 0.66626)+0.005483)-0.075593;
				else if (rms_current < 19 &&  rms_current > 8.5) rms_current= 1.0048 * (0.951848277 * rms_current - 0.712869431) + 0.38113;
				else if (rms_current < 26.5 &&  rms_current > 19) rms_current= 1.0059 * rms_current - 1.20233;
				else rms_current = 1.0027*rms_current - 1.035;
				//CALIB THEO SENSOR
//				if (rms_current<7)	rms_current= 0.9397*rms_current + 0.13289 - 0.55;
//				else if (rms_current < 23 &&  rms_current > 8) rms_current = 0.9605 * (1.0055*rms_current - 1.1241) + 0.66104;
//					else  rms_current = 1.0055*rms_current - 1.1241;

				rms_current = rms_current*100 + OFFSET_I4;
			}
		}
		else //Charge current
		{
			charge_discharge_sts =1;
			rms_current = (Vref-rms_current)/0.01; // step 10mV/A

			//CALIB
			if(channel==1)
			{
				rms_current = rms_current*100 + OFFSET_I1;
			}
			else if(channel==2) // CHUA CALIB
			{
				rms_current = rms_current*100 + OFFSET_I2;
			}
			else if(channel==3) //y = 1.0169x - 1.15
			{
				rms_current = rms_current*100 + OFFSET_I3;
			}
			else if(channel==4)
			{
				rms_current = rms_current*100 + OFFSET_I4;
			}
		}
	}
	else
	{
		rms_current=0;
	}

//	//print test
//	RS485_S_Ctr = 1U; //RS485 Master send mode
//	sprintf(print_str,"\n\r curr%d: %d \n\r", channel,(int16_t)rms_current);
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//	RS485_S_Ctr = 0U; //RS485 Master send mode
//	memset(print_str, 0, sizeof(print_str));
//	//end print test


	return rms_current;
}
void waittime(uint16_t time)
{
	wait_time =0;
	while(wait_time<time) // time*200ms
	{
		R_BSP_SoftwareDelay(200, BSP_DELAY_MILLISECS);
		RS485_Slave_Mode();
	}
}
void Flash_write(uint32_t address, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4, uint16_t data5, uint16_t data6, uint16_t data7, uint16_t data8)
{
	uint16_t WE = 0x06;
	uint8_t *sts;
	FLASH_CE =0;
	SPI_Send_Receive(&WE, 1, SPI_buff); // send write enable
	FLASH_CE =1;

	uint16_t tx_buf[21];
	tx_buf[0] = 0x02;
	tx_buf[1] = (address >> 16) & 0xFF;
	tx_buf[2] = (address >> 8) & 0xFF;
	tx_buf[3] = address & 0xFF;

	tx_buf[4] 	= (uint8_t)((data1>>8) & 0xFF);
	tx_buf[5] 	= (uint8_t)(data1 & 0xFF);
	tx_buf[6] 	= (uint8_t)((data2>>8) & 0xFF);
	tx_buf[7] 	= (uint8_t)(data2 & 0xFF);
	tx_buf[8] 	= (uint8_t)((data3>>8) & 0xFF);
	tx_buf[9] 	= (uint8_t)(data3 & 0xFF);
	tx_buf[10] 	= (uint8_t)((data4>>8) & 0xFF);
	tx_buf[11] 	= (uint8_t)(data4 & 0xFF);
	tx_buf[12] 	= (uint8_t)((data5>>8) & 0xFF);
	tx_buf[13] 	= (uint8_t)(data5 & 0xFF);
	tx_buf[14] 	= (uint8_t)((data6>>8) & 0xFF);
	tx_buf[15] 	= (uint8_t)(data6 & 0xFF);
	tx_buf[16] 	= (uint8_t)((data7>>8) & 0xFF);
	tx_buf[17] 	= (uint8_t)(data7 & 0xFF);
	tx_buf[18] 	= (uint8_t)((data8>>8) & 0xFF);
	tx_buf[19] 	= (uint8_t)(data8 & 0xFF);

//	//print SPI_buff
//	sprintf(print_str,"\r\nWrite: ");
//	RS485_S_Ctr = 1U; //RS485 Master send mode
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//	RS485_S_Ctr = 0U; //RS485 Master receive mode
//	memset(print_str, 0, sizeof(print_str));
//	for(uint16_t i=4;i<20;i++)
//	{
//		//print SPI_buff
//		sprintf(print_str," %02X",tx_buf[i]);
//		RS485_S_Ctr = 1U; //RS485 Master send mode
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//		RS485_S_Ctr = 0U; //RS485 Master receive mode
//		memset(print_str, 0, sizeof(print_str));
//	}
	R_BSP_SoftwareDelay(300, BSP_DELAY_MILLISECS);
	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(tx_buf, 21, SPI_buff); //write data
	FLASH_CE =1;
	R_BSP_SoftwareDelay(300, BSP_DELAY_MILLISECS);

}

void Flash_read(uint32_t address, uint16_t *data1,uint16_t *data2,uint16_t *data3,uint16_t *data4, uint16_t *data5,uint16_t *data6,uint16_t *data7,uint16_t *data8)
{
//	memset(SPI_buff, 0, sizeof(SPI_buff));
	uint16_t rx_buf[4];
	rx_buf[0] = 0x03;
	rx_buf[1] = (address >> 16) & 0xFF;
	rx_buf[2] = (address >> 8) & 0xFF;
	rx_buf[3] = address & 0xFF;

	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(rx_buf, 20, SPI_buff); // send read at address
	FLASH_CE =1;
	R_BSP_SoftwareDelay(400, BSP_DELAY_MILLISECS);
	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(rx_buf, 20, SPI_buff); // Just to make sure!!
	FLASH_CE =1;
	R_BSP_SoftwareDelay(400, BSP_DELAY_MILLISECS);

	*data1 = (uint16_t)(SPI_buff[4]<<8) +(uint16_t)SPI_buff[5];
	*data2 = (uint16_t)(SPI_buff[6]<<8) +(uint16_t)SPI_buff[7];
	*data3 = (uint16_t)(SPI_buff[8]<<8) +(uint16_t)SPI_buff[9];
	*data4 = (uint16_t)(SPI_buff[10]<<8)+(uint16_t)SPI_buff[11];
	*data5 = (uint16_t)(SPI_buff[12]<<8)+(uint16_t)SPI_buff[13];
	*data6 = (uint16_t)(SPI_buff[14]<<8)+(uint16_t)SPI_buff[15];
	*data7 = (uint16_t)(SPI_buff[16]<<8)+(uint16_t)SPI_buff[17];
	*data8 = (uint16_t)(SPI_buff[18]<<8)+(uint16_t)SPI_buff[19];

//	//print test
//	sprintf(print_str,"\r\nRead:");
//	RS485_S_Ctr = 1U; //RS485 Master send mode
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//	RS485_S_Ctr = 0U; //RS485 Master receive mode
//	memset(print_str, 0, sizeof(print_str));
//	for(uint16_t i=4;i<20;i++)
//	{
//		//print SPI_buff
//		sprintf(print_str," %02X",SPI_buff[i]);
//		RS485_S_Ctr = 1U; //RS485 Master send mode
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//		RS485_S_Ctr = 0U; //RS485 Master receive mode
//		memset(print_str, 0, sizeof(print_str));
//	}
//	//print test
//	sprintf(print_str,"\r\n data= %d %d %d %d\r\n",*data1,*data2,*data3,*data4);
//	RS485_S_Ctr = 1U; //RS485 Master send mode
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//	RS485_S_Ctr = 0U; //RS485 Master receive mode
//	memset(print_str, 0, sizeof(print_str));

}
void Flash_Sector_Erase(uint32_t address) //4kB erase
{
	uint16_t WE = 0x06;
	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(&WE, 1, SPI_buff); // send write enable
	FLASH_CE =1;
	R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);

	uint16_t rx_buf[4];
	rx_buf[0] = 0x20;
	rx_buf[1] = (address >> 16) & 0xFF;
	rx_buf[2] = (address >> 8) & 0xFF;
	rx_buf[3] = address & 0xFF;

	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(rx_buf, 4, SPI_buff);
	FLASH_CE =1;
	R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);
}
void Flash_Block_Erase(uint32_t address) //64kB erase
{
	uint16_t WE = 0x06;
	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(&WE, 1, SPI_buff); // send write enable
	FLASH_CE =1;

	uint16_t rx_buf[4];
	rx_buf[0] = 0xD8;
	rx_buf[1] = (address >> 16) & 0xFF;
	rx_buf[2] = (address >> 8) & 0xFF;
	rx_buf[3] = address & 0xFF;

	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(rx_buf, 4, SPI_buff);
	FLASH_CE =1;
}
void Flash_Chip_Erase()
{
	uint16_t WE = 0x06;
	uint16_t CE = 0xC7;

	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(&WE, 1, SPI_buff); // send write enable
	FLASH_CE =1;

	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(&CE, 1, SPI_buff);
	FLASH_CE =1;
}
void Flash_Read_StsReg(uint8_t *sts)
{
	uint16_t RSR = 0x05;
	FLASH_CE =0;
	R_BSP_SoftwareDelay(1, BSP_DELAY_MILLISECS);
	SPI_Send_Receive(&RSR, 2, SPI_buff); // send write enable
	FLASH_CE =1;

	*sts = SPI_buff[1];
//	//print test
//	sprintf(print_str,"\r\nsts = %d",*sts);
//	RS485_S_Ctr = 1U; //RS485 Master send mode
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str),transmit_ctrl);
//	RS485_S_Ctr = 0U; //RS485 Master receive mode
//	memset(print_str, 0, sizeof(print_str));
}
void Buzzer(uint8_t times, uint16_t millisec)
{
	for(uint8_t i=0;i<times;i++)
	{
    	BUZZER =1;
    	R_BSP_SoftwareDelay(millisec, BSP_DELAY_MILLISECS);
    	BUZZER =0;
    	R_BSP_SoftwareDelay(millisec, BSP_DELAY_MILLISECS);
	}
}
void Led_Blink()
{
	MCU_LED_MCC ^=1;
	MCU_LED_PSU ^=1;
	MCU_LED_STT ^=1;
	R_BSP_SoftwareDelay(300, BSP_DELAY_MILLISECS);
}
