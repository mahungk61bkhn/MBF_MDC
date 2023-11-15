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
* File Name        : Config_CMT1_user.c
* Component Version: 2.3.0
* Device(s)        : R5F51305AxFM
* Description      : This file implements device driver for Config_CMT1.
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
#include "Config_CMT1.h"
/* Start user code for include. Do not edit comment generated here */
#include "r_cg_macrodriver.h"
#include "apl_header_files.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
#define MCU_LED_MCC (PORTB.PODR.BIT.B1)
#define MCU_LED_PSU (PORTB.PODR.BIT.B0)
#define MCU_LED_STT (PORTA.PODR.BIT.B6)
#define MCC_DISCONNECT_TIMEOUT 45 //minutes
#define MDC_NUM_REGS 320
uint16_t MCC_timeout_count=0;
uint16_t WD_timeout_count=0;
uint32_t wait_time=0;
uint32_t runtime=0;
volatile uint8_t MCC_timeout_flag=0;
volatile uint8_t WD_timeout_flag=0;
extern volatile uint8_t is_slave;
extern volatile uint8_t PSU_connect_flag;
extern uint16_t MDC_regs[MDC_NUM_REGS];
extern uint8_t discharge_start;
extern uint8_t charge_start;
uint32_t discharge_time_count;
uint32_t charge_time_count;
uint32_t reset_chargetime_count=0;
uint32_t reset_dischargetime_count=0;
uint8_t save_flag=0; // for saving charge/discharge time to flash
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_CMT1_Create_UserInit
* Description  : This function adds user code after initializing the CMT1 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_CMT1_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_CMT1_cmi1_interrupt
* Description  : This function is CMI1 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_CMT1_CMI1
#pragma interrupt r_Config_CMT1_cmi1_interrupt(vect=VECT(CMT1,CMI1),fint)
#else
#pragma interrupt r_Config_CMT1_cmi1_interrupt(vect=VECT(CMT1,CMI1))
#endif
static void r_Config_CMT1_cmi1_interrupt(void)
{
    /* Start user code for r_Config_CMT1_cmi1_interrupt. Do not edit comment generated here */
	// for wait function
	wait_time++;
	 // MDC running time (mins)
	if(runtime<0xFFFFFF) runtime++;
	else runtime=0;
	MDC_regs[58] = (uint16_t) ((runtime / 300) >> 16);
	MDC_regs[59] = (uint16_t) ((runtime / 300));
	// LED MCC
	if(is_slave)
		MCC_timeout_count++; //No RS485 communication from MCC increase counter
	else
	{
		MCC_timeout_count = 0;
		MCC_timeout_flag = 0;
	}

	if (MCC_timeout_count==899) // 200*900 ms = 3 mins
	{
		MCC_timeout_count=0;
		MCC_timeout_flag =1; // MCC communication lost
	}

	if(MCC_timeout_flag) //BLink LED_MCC indicate lost MCC connection
	{
		MCU_LED_MCC ^=1;
		WD_timeout_count++; //Count time to stop WD pulse
	}
	else
	{
		MCU_LED_MCC =0;
		WD_timeout_count=0;
		WD_timeout_flag=0;
	}

	if (WD_timeout_count == 599 * 21) // (200 *600 ms = 2mins)*21 = 42 mins
	{
		WD_timeout_flag=1; // signal to stop WD pulse
		WD_timeout_count=0;
		PowerON_Reset_PC();
	}
	//LED PSU
	if(!PSU_connect_flag) MCU_LED_PSU ^=1;
	else MCU_LED_PSU =0;

	//DC Low
	if(MDC_regs[43]==1)
	{
		MCU_LED_STT^=1;
	}
	else MCU_LED_STT=0;

	//Discharge time
	if(discharge_start==1)
	{
		discharge_time_count++;
		reset_dischargetime_count=0;
	}
	else
	{
		reset_dischargetime_count++;
		if(reset_dischargetime_count >= 9000)
		{
			reset_dischargetime_count=0;
			discharge_time_count=0; //after 30 mins no discharge, reset discharge time count
		}
	}
	//Charge time
	if(charge_start==1)
	{
		charge_time_count++;
		reset_chargetime_count=0;
	}
	else
	{
		reset_chargetime_count++;
		if(reset_chargetime_count >= 9000)
		{
			reset_chargetime_count=0;
			charge_time_count=0; //after 30 mins no charge, reset charge time count
		}
	}

    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
