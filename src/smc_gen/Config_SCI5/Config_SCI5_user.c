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
* File Name        : Config_SCI5_user.c
* Component Version: 1.12.0
* Device(s)        : R5F51305AxFM
* Description      : This file implements device driver for Config_SCI5.
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
#include "Config_SCI5.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern volatile uint8_t * gp_sci5_tx_address;                /* SCI5 transmit buffer address */
extern volatile uint16_t  g_sci5_tx_count;                   /* SCI5 transmit data number */
extern volatile uint8_t * gp_sci5_rx_address;                /* SCI5 receive buffer address */
extern volatile uint16_t  g_sci5_rx_count;                   /* SCI5 receive data number */
extern volatile uint16_t  g_sci5_rx_length;                  /* SCI5 receive data length */
/* Start user code for global. Do not edit comment generated here */
extern uint8_t rx5_buff[500];
//extern uint16_t rx5_count; // Remember to add rx5_count = g_sci5_rx_count; in r_Config_SCI5_receive_interrupt()
/* Flag used to detect whether data is received */
volatile uint8_t SCI5_rxdone;
/* Flag used to detect completion of transmission */
static volatile uint8_t SCI5_txdone;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_SCI5_Create_UserInit
* Description  : This function adds user code after initializing the SCI5 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_SCI5_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_SCI5_transmit_interrupt
* Description  : This function is TXI5 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_SCI5_TXI5
#pragma interrupt r_Config_SCI5_transmit_interrupt(vect=VECT(SCI5,TXI5),fint)
#else
#pragma interrupt r_Config_SCI5_transmit_interrupt(vect=VECT(SCI5,TXI5))
#endif
static void r_Config_SCI5_transmit_interrupt(void)
{
    if (0U < g_sci5_tx_count)
    {
        SCI5.TDR = *gp_sci5_tx_address;
        gp_sci5_tx_address++;
        g_sci5_tx_count--;
    }
    else
    {
        SCI5.SCR.BIT.TIE = 0U;
        SCI5.SCR.BIT.TEIE = 1U;
    }
}

/***********************************************************************************************************************
* Function Name: r_Config_SCI5_transmitend_interrupt
* Description  : This function is TEI5 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_SCI5_TEI5
#pragma interrupt r_Config_SCI5_transmitend_interrupt(vect=VECT(SCI5,TEI5),fint)
#else
#pragma interrupt r_Config_SCI5_transmitend_interrupt(vect=VECT(SCI5,TEI5))
#endif
static void r_Config_SCI5_transmitend_interrupt(void)
{
    /* Set TXD5 pin */
    PORTC.PMR.BYTE &= 0xF7U;

    SCI5.SCR.BIT.TIE = 0U;
    SCI5.SCR.BIT.TE = 0U;
    SCI5.SCR.BIT.TEIE = 0U;

    r_Config_SCI5_callback_transmitend();
}

/***********************************************************************************************************************
* Function Name: r_Config_SCI5_receive_interrupt
* Description  : This function is RXI5 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_SCI5_RXI5
#pragma interrupt r_Config_SCI5_receive_interrupt(vect=VECT(SCI5,RXI5),fint)
#else
#pragma interrupt r_Config_SCI5_receive_interrupt(vect=VECT(SCI5,RXI5))
#endif
static void r_Config_SCI5_receive_interrupt(void)
{
    if (g_sci5_rx_length > g_sci5_rx_count)
    {
        *gp_sci5_rx_address = SCI5.RDR;
        gp_sci5_rx_address++;
        g_sci5_rx_count++;
    }

    if (g_sci5_rx_length <= g_sci5_rx_count)
    {
        /* All data received */
        SCI5.SCR.BIT.RIE = 0U;
        SCI5.SCR.BIT.RE = 0U;
        r_Config_SCI5_callback_receiveend();
    }
}

/***********************************************************************************************************************
* Function Name: r_Config_SCI5_receiveerror_interrupt
* Description  : This function is ERI5 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_SCI5_ERI5
#pragma interrupt r_Config_SCI5_receiveerror_interrupt(vect=VECT(SCI5,ERI5),fint)
#else
#pragma interrupt r_Config_SCI5_receiveerror_interrupt(vect=VECT(SCI5,ERI5))
#endif
static void r_Config_SCI5_receiveerror_interrupt(void)
{
    uint8_t err_type;

    r_Config_SCI5_callback_receiveerror();

    /* Clear overrun, framing and parity error flags */
    err_type = SCI5.SSR.BYTE;
    err_type &= 0xC7U;
    err_type |= 0xC0U;
    SCI5.SSR.BYTE = err_type;
}

/***********************************************************************************************************************
* Function Name: r_Config_SCI5_callback_transmitend
* Description  : This function is a callback function when SCI5 finishes transmission
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

static void r_Config_SCI5_callback_transmitend(void)
{
    /* Start user code for r_Config_SCI5_callback_transmitend. Do not edit comment generated here */
	/* Set transmit completion flag */
	SCI5_txdone = 1U;
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_SCI5_callback_receiveend
* Description  : This function is a callback function when SCI5 finishes reception
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

static void r_Config_SCI5_callback_receiveend(void)
{
    /* Start user code for r_Config_SCI5_callback_receiveend. Do not edit comment generated here */
	SCI5_rxdone = 1U;
	memset(rx5_buff, 0, sizeof(rx5_buff));
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_SCI5_callback_receiveerror
* Description  : This function is a callback function when SCI5 reception encounters error
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

static void r_Config_SCI5_callback_receiveerror(void)
{
    /* Start user code for r_Config_SCI5_callback_receiveerror. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/*******************************************************************************
* Function Name: R_SCI5_AsyncTransmit
* Description : This function sends SCI5 data and waits for the transmit end flag
* Arguments : tx_buf -
* transfer buffer pointer
* tx_num -
* buffer size
* Return Value : status -
* MD_OK or MD_ARGERROR
*******************************************************************************/
MD_STATUS R_SCI5_AsyncTransmit (uint8_t * const tx_buf,uint16_t tx_num, uint8_t control)
{
	MD_STATUS status = MD_OK;
	if(!control)
	{

		 /* Clear transmit completion flag before transmission */
		 SCI5_txdone = 0U;
		 /* Set SCI5 transmit buffer address and start transmission */
		 status = R_Config_SCI5_Serial_Send(tx_buf, tx_num);
		 /* Wait for transmit end flag */
		 while (0U == SCI5_txdone)
		 {
			 /* Wait */
		 }
	}
	else
	{

	}
	 return (status);
}
/**********************************************************************
*********
* End of function R_SCI8_AsyncTransmit
*****************************************************************
**************/
/* End user code. Do not edit comment generated here */
