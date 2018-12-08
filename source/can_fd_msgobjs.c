/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "can.h"
#include "fsl_usart.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define APP_BOARD_TEST_GPIO_PORT1 BOARD_LED3_GPIO_PORT
#define APP_BOARD_TEST_GPIO_PORT2 BOARD_LED1_GPIO_PORT
#define APP_BOARD_TEST_GPIO_PORT3 BOARD_LED2_GPIO_PORT
#define APP_BOARD_TEST_LED1_PIN BOARD_LED3_GPIO_PIN
#define APP_BOARD_TEST_LED2_PIN BOARD_LED1_GPIO_PIN
#define APP_BOARD_TEST_LED3_PIN BOARD_LED2_GPIO_PIN

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#define TICKRATE_HZ (1000)	          /* 1000 ticks per second */
#define TRANSMIT_PERIOD (500)         /* milliseconds between transmission */

static volatile uint32_t gTimCnt = 0; /* incremented every millisecond */

uint8_t VfCANH_RxMSG_Data;
uint16_t VfCANH_RxMSG_ID;

uint8_t VfUSART_Data[12];
uint8_t USART_Data[12],i;

usart_handle_t usart0_Define;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Keeps track of time
 */
void SysTick_Handler(void)
{
	// count milliseconds
	gTimCnt++;
}

/*!
 * @brief Main function
 */
int main(void)
{
    can_config_t config;
    can_frame_t txmsg = { 0 };
		
		txmsg.dataByte[0] = 0xFC;
		txmsg.dataByte[1] = 0x56;
    can_frame_t rxmsg = { 0 };
    int b;
    bool message_transmitted = false;
    uint32_t next_id = 0x4C8;

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Board pin, clock, debug console init */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    CLOCK_EnableClock(kCLOCK_Gpio2);
    CLOCK_EnableClock(kCLOCK_Gpio3);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();
		USART_EnableInterrupts(USART1, kUSART_TxLevelInterruptEnable | kUSART_RxLevelInterruptEnable);
    /* print a note to terminal */
    PRINTF("\r\n CAN-FD driver message objects example\r\n");

    /* configure for 4Mbps data 1Mbps nominal, CAN-FD */
		
		//config.disableFD = false;
    CAN_GetDefaultConfig(&config);
    config.baseAddress = 0x20010000;
    config.nominalBaudRate = 500000;                  // nominal bit rate is 500kbps
    config.dataBaudRate = 2000000;                     //the data bit rate is 2Mbps
    config.timestampClock_Hz = 100000;
    CAN_Init(CAN0, &config, SystemCoreClock);
    CAN_Init(CAN1, &config, SystemCoreClock);

    /* receive 0x100 in CAN1 rx message buffer 0 by setting mask 0 */
    CAN_SetRxIndividualMask(CAN0, 0, CAN_RX_MB_STD(0x4C9, 0));
    /* receive 0x101 in CAN1 rx message buffer 0 by setting mask 1 */
    CAN_SetRxIndividualMask(CAN1, 1, CAN_RX_MB_STD(0x4C9, 0));
    //* receive 0x102 in CAN1 rx message buffer 0 by setting mask 2 */
    CAN_SetRxIndividualMask(CAN1, 2, CAN_RX_MB_STD(0x102, 0));
    /* receive 0x00000200 (29-bit id) in CAN1 rx message buffer 1 by setting mask 3 */
    CAN_SetRxExtIndividualMask(CAN1, 3, CAN_RX_MB_EXT_LOW(0x200, 1), CAN_RX_MB_EXT_HIGH(0x200, 1));

    /* enable CAN 0 */
    CAN_Enable(CAN0, true);
    /* enable CAN 1 */
    CAN_Enable(CAN1, true);

    /* Enable SysTick Timer */
    SysTick_Config(SystemCoreClock / TICKRATE_HZ);

    /* Init output LED GPIO. */
		/*
    GPIO_PinInit(GPIO, BOARD_LED1_GPIO_PORT, BOARD_LED1_GPIO_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, BOARD_LED1_GPIO_PORT, BOARD_LED1_GPIO_PIN, 1);
    GPIO_PinInit(GPIO, BOARD_LED2_GPIO_PORT, BOARD_LED2_GPIO_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, BOARD_LED2_GPIO_PORT, BOARD_LED2_GPIO_PIN, 1);
    GPIO_PinInit(GPIO, BOARD_LED3_GPIO_PORT, BOARD_LED3_GPIO_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, BOARD_LED3_GPIO_PORT, BOARD_LED3_GPIO_PIN, 1);
		*/
		GPIO_PinInit(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_EN_GPIO_PIN, &led_config);
		GPIO_WritePinOutput(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_EN_GPIO_PIN,0);
		
		GPIO_PinInit(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_BT_GPIO_PIN, &led_config);
		GPIO_WritePinOutput(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_BT_GPIO_PIN,0);
		
		GPIO_PinInit(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_BC_GPIO_PIN, &led_config);
		GPIO_WritePinOutput(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_BC_GPIO_PIN,0);

    while (true)
    {
        /* time to send messages from CAN0 */
        
        
        /* check for any received messages on CAN1 message buffer 0 */
        if (CAN_ReadRxMb(CAN0, 0, &rxmsg) == kStatus_Success)
        {
            //PRINTF("Rx buf 0: Received message 0x%3.3X\r\n", rxmsg.id);
					
          	VfCANH_RxMSG_Data=rxmsg.dataByte[1]; //Read the Rx buffer Byte1
					VfCANH_RxMSG_ID=rxmsg.id;
					if(VfCANH_RxMSG_ID==0x4C9)
					{
						GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);
					}
					USART_WriteByte(USART0, VfCANH_RxMSG_Data);
					USART_WriteByte(USART0, 0x00);
            /* toggle LED2 */
				   GPIO_TogglePinsOutput(GPIO, BOARD_LED2_GPIO_PORT, 1u << BOARD_LED2_GPIO_PIN);
        }

        /* check for any received messages on CAN1 message buffer 1 */
        if (CAN_ReadRxMb(CAN1, 1, &rxmsg) == kStatus_Success)
        {
          //  PRINTF("Rx buf 1: Received message 0x%X (29-bit)\r\n", rxmsg.id);
          
            /* toggle LED3 */
          //  GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);
					VfCANH_RxMSG_Data=rxmsg.dataByte[1]; //Read the Rx buffer Byte1
					VfCANH_RxMSG_ID=rxmsg.id;
					if(VfCANH_RxMSG_ID==0x4C9)
					{
						GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);
					}
					USART_WriteByte(USART0, VfCANH_RxMSG_Data);
					USART_WriteByte(USART0, 0x00);
				
        }
				for(i=0;i<12;i++)
				{
				USART_Data[i]=DbgConsole_Getchar();
				}
				VfUSART_Data[0]=USART_Data[0];
					VfUSART_Data[1]=USART_Data[1];
					VfUSART_Data[2]=USART_Data[2];
						//USART_ReadByte(USART0);
			//	USART_ReadBlocking(USART0,VfUSART_Data,2);
				//USART_TransferHandleIRQ(USART0,usart0_Define);
    }
}


