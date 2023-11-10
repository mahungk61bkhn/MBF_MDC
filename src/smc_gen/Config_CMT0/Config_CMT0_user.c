/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Config_CMT0_user.c
* Component Version: 2.3.0
* Device(s)        : R5F51305AxFM
* Description      : This file implements device driver for Config_CMT0.
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "Config_CMT0.h"
/* Start user code for include. Do not edit comment generated here */
#include "Config_S12AD0.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
#define MCU_LED_MCC (PORTB.PODR.BIT.B1)
#define MCU_LED_PSU (PORTB.PODR.BIT.B0)
#define MCU_LED_STT (PORTA.PODR.BIT.B6)
volatile uint8_t Sample_done=0;
uint16_t ADC_sample_count;
uint32_t tick;
#define WDI 		(PORT3.PODR.BIT.B2)
#define SAMPLES_NUM  128
extern uint16_t ADC_Temp1[SAMPLES_NUM], ADC_Temp2[SAMPLES_NUM];
extern uint16_t ADC_Volt1[SAMPLES_NUM], ADC_Volt2[SAMPLES_NUM], ADC_Volt3[SAMPLES_NUM], ADC_Volt4[SAMPLES_NUM];
extern uint16_t ADC_Curr1[SAMPLES_NUM];
extern uint16_t ADC_Curr2[SAMPLES_NUM];
extern uint16_t ADC_Curr3[SAMPLES_NUM];
extern uint16_t ADC_Curr4[SAMPLES_NUM];
extern uint16_t ADC_CurrRef1[SAMPLES_NUM];
extern uint16_t ADC_CurrRef2[SAMPLES_NUM];
extern uint16_t ADC_CurrRef3[SAMPLES_NUM];
extern uint16_t ADC_CurrRef4[SAMPLES_NUM];

extern volatile uint8_t adc_completed;
extern volatile uint8_t WD_timeout_flag;
void ADC_Sample(uint16_t count);
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_CMT0_Create_UserInit
* Description  : This function adds user code after initializing the CMT0 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_CMT0_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_CMT0_cmi0_interrupt
* Description  : This function is CMI0 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_CMT0_CMI0
#pragma interrupt r_Config_CMT0_cmi0_interrupt(vect=VECT(CMT0,CMI0),fint)
#else
#pragma interrupt r_Config_CMT0_cmi0_interrupt(vect=VECT(CMT0,CMI0))
#endif
static void r_Config_CMT0_cmi0_interrupt(void)
{
    /* Start user code for r_Config_CMT0_cmi0_interrupt. Do not edit comment generated here */
	//system tick
	tick++;

	// Sampling
	if ((ADC_sample_count<SAMPLES_NUM) && (Sample_done==0))
	{
		ADC_Sample(ADC_sample_count);
		ADC_sample_count++;
	}
	if(ADC_sample_count >= SAMPLES_NUM)
	{
		ADC_sample_count=0;
		Sample_done =1; // notice to perform Calculating
	}
	// Watch Dog
	WDI =1;
	R_BSP_SoftwareDelay(5, BSP_DELAY_MICROSECS);
	WDI =0;
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
void ADC_Sample(uint16_t count)
{
	R_Config_S12AD0_Start();
	uint8_t cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL1,&ADC_Temp1[count]);
	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL0,&ADC_Temp2[count]);
	R_Config_S12AD0_Stop();

//	R_Config_S12AD0_Start();
//	cnt = 50;
//	while(S12AD.ADCSR.BIT.ADST && cnt--) {
//		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
//	}
//	R_Config_S12AD0_Get_ValueResult(ADCHANNEL2,&ADC_CurrRef1[count]);
//	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL3,&ADC_Curr1[count]);
	R_Config_S12AD0_Stop();

//	R_Config_S12AD0_Start();
//		cnt = 50;
//	while(S12AD.ADCSR.BIT.ADST && cnt--) {
//		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
//	}
//	R_Config_S12AD0_Get_ValueResult(ADCHANNEL4,&ADC_CurrRef2[count]);
//	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL5,&ADC_Curr2[count]);
	R_Config_S12AD0_Stop();

//	R_Config_S12AD0_Start();
//	cnt = 50;
//	while(S12AD.ADCSR.BIT.ADST && cnt--) {
//		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
//	}
//	R_Config_S12AD0_Get_ValueResult(ADCHANNEL6,&ADC_CurrRef3[count]);
//	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL7,&ADC_Curr3[count]);
	R_Config_S12AD0_Stop();

//	R_Config_S12AD0_Start();
//	while((!adc_completed) && (!R_BSP_SoftwareDelay(15, BSP_DELAY_MICROSECS)));
//	R_Config_S12AD0_Get_ValueResult(ADCHANNEL16,&ADC_CurrRef4[count]);
//	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL17,&ADC_Curr4[count]);
	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL18,&ADC_Volt1[count]);
	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL19,&ADC_Volt2[count]);
	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL20,&ADC_Volt3[count]);
	R_Config_S12AD0_Stop();

	R_Config_S12AD0_Start();
	cnt = 50;
	while(S12AD.ADCSR.BIT.ADST && cnt--) {
		R_BSP_SoftwareDelay(1, BSP_DELAY_MICROSECS);
	}
	R_Config_S12AD0_Get_ValueResult(ADCHANNEL21,&ADC_Volt4[count]);
	R_Config_S12AD0_Stop();


	adc_completed=0;

}



/* End user code. Do not edit comment generated here */
