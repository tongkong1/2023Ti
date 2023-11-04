/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
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
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
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

/**
 * @file    MK66F18_Ref2a.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_uart.h"
#include "fsl_rtc.h"
#include "CDK66_OLED.h"
#include "pin_mux.h"
#include <math.h>
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
#define SW1() 			GPIO_PinRead(BOARD_INITPINS_SW1_GPIO, BOARD_INITPINS_SW1_PIN)
#define SW2() 			GPIO_PinRead(BOARD_INITPINS_SW2_GPIO, BOARD_INITPINS_SW2_PIN)
#define SW3() 			GPIO_PinRead(BOARD_INITPINS_SW3_GPIO, BOARD_INITPINS_SW3_PIN)
#define SW4() 			GPIO_PinRead(BOARD_INITPINS_SW4_GPIO, BOARD_INITPINS_SW4_PIN)
#define KEY1() 			!GPIO_PinRead(BOARD_INITPINS_KEY1_GPIO, BOARD_INITPINS_KEY1_PIN)
#define KEY2() 			!GPIO_PinRead(BOARD_INITPINS_KEY2_GPIO, BOARD_INITPINS_KEY2_PIN)
#define QESA() 			GPIO_PinRead(BOARD_INITPINS_QEPA_GPIO, BOARD_INITPINS_QEPA_PIN)
#define QESB() 			GPIO_PinRead(BOARD_INITPINS_QEPB_GPIO, BOARD_INITPINS_QEPB_PIN)
#define BTN() 			!GPIO_PinRead(BOARD_INITPINS_BTN_GPIO, BOARD_INITPINS_BTN_PIN)

#define LED1_ON() 		GPIO_PortClear(BOARD_INITPINS_LED1_GPIO, BOARD_INITPINS_LED1_GPIO_PIN_MASK)
#define LED1_OFF() 		GPIO_PortSet(BOARD_INITPINS_LED1_GPIO, BOARD_INITPINS_LED1_GPIO_PIN_MASK)
#define LED1_TOGGLE() 	GPIO_PortToggle(BOARD_INITPINS_LED1_GPIO, BOARD_INITPINS_LED1_GPIO_PIN_MASK)
#define LED2_ON() 		GPIO_PortClear(BOARD_INITPINS_LED2_GPIO, BOARD_INITPINS_LED2_GPIO_PIN_MASK)
#define LED2_OFF() 		GPIO_PortSet(BOARD_INITPINS_LED2_GPIO, BOARD_INITPINS_LED2_GPIO_PIN_MASK)
#define LED2_TOGGLE() 	GPIO_PortToggle(BOARD_INITPINS_LED2_GPIO, BOARD_INITPINS_LED2_GPIO_PIN_MASK)
#define BEEP_ON() 		GPIO_PortSet(BOARD_INITPINS_BEEP_GPIO, BOARD_INITPINS_BEEP_GPIO_PIN_MASK)
#define BEEP_OFF() 		GPIO_PortClear(BOARD_INITPINS_BEEP_GPIO, BOARD_INITPINS_BEEP_GPIO_PIN_MASK)
#define BEEP_TOGGLE() 	GPIO_PortToggle(BOARD_INITPINS_BEEP_GPIO, BOARD_INITPINS_BEEP_GPIO_PIN_MASK)
#define M1_ENABLE() 	GPIO_PortSet(BOARD_INITPINS_MLEN_GPIO, BOARD_INITPINS_MLEN_GPIO_PIN_MASK)
#define M1_DISABLE() 	GPIO_PortClear(BOARD_INITPINS_MLEN_GPIO, BOARD_INITPINS_MLEN_GPIO_PIN_MASK)
#define M2_ENABLE() 	GPIO_PortSet(BOARD_INITPINS_MREN_GPIO, BOARD_INITPINS_MREN_GPIO_PIN_MASK)
#define M2_DISABLE() 	GPIO_PortClear(BOARD_INITPINS_MREN_GPIO, BOARD_INITPINS_MREN_GPIO_PIN_MASK)

#define CCD_SI_HIGH		GPIO_PinWrite(BOARD_INITPINS_CCDSI_GPIO, BOARD_INITPINS_CCDSI_PIN, 1)
#define CCD_SI_LOW		GPIO_PinWrite(BOARD_INITPINS_CCDSI_GPIO, BOARD_INITPINS_CCDSI_PIN, 0)
#define CCD_CLK_HIGH	GPIO_PinWrite(BOARD_INITPINS_CCDCLK_GPIO, BOARD_INITPINS_CCDCLK_PIN, 1)
#define CCD_CLK_LOW		GPIO_PinWrite(BOARD_INITPINS_CCDCLK_GPIO, BOARD_INITPINS_CCDCLK_PIN, 0)

#define I2C_CAT9555_BASEADDR I2C3
#define I2C_CAT9555_SLAVE_ADDR_7BIT 0x22U
#define I2C_CAT9555_DATA_LENGTH 2U
#define USB_RING_BUFFER_SIZE	(128U)
#define cmdDataLenMax			(1000U)
#define cmdDataLenMax1			(1000U)
typedef struct _Board_Analog
{
	 uint32_t	x;			// MMA7260 x
	 uint32_t	y;			// MMA7260 y
	 uint32_t	z;			// MMA7260 z
	 uint32_t	IRV1;		// IR sensor #1
	 uint32_t	IRV2;		// IR sensor #2
	 uint32_t	HALL1;		// Hall sensor #1
	 uint32_t	HALL2;		// Hall sensor #2
	 uint32_t	VSENSE;		// Battery Voltage
	 uint32_t	Temp;		// On-chip temperature sensor
	 uint32_t	CCD;		// TSL1401 Linear CCD
} Board_Analog_t;

// Variables for HMI
static uint8_t DSPTable[] = { 0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10, 0x08, 0x03, 0x27, 0x21, 0x06, 0x0E };
uint16_t QESVar=0;

// Variables for I2C3
uint8_t g_master_txBuff[I2C_CAT9555_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_CAT9555_DATA_LENGTH];
i2c_master_transfer_t masterXfer;

// Variable for RTC
rtc_datetime_t appDateTime;
uint32_t ElapsedSeconds = 0;

// Variables for ADCs
Board_Analog_t AnalogIn;

// Variables for UART
bool CmdDataReady;		// new cmd data flag, 1: new cmd received
uint8_t CmdDataLen;		// PC cmd data length counter
uint8_t CmdDataBuffer[cmdDataLenMax];	// buffer to hold cmd data from PC
uint8_t UsbRingBuffer[USB_RING_BUFFER_SIZE];
bool CmdDataReady1;		// new cmd data flag, 1: new cmd received
uint8_t CmdDataLen1;		// PC cmd data length counter
uint8_t CmdDataBuffer1[cmdDataLenMax];	// buffer to hold cmd data from PC
uint16_t g_bufferWriteByteCnt;
volatile uint16_t txIndexUSB=0;		// Index of the debug port data to send out.
volatile uint16_t rxIndexUSB=0;		// Index of the memory to save new arrived debug port data.

uint8_t status1 = 0;

// Variables for MMA7260
uint16_t MemsX, MemsY, MemsZ;
float MemsX1, MemsY1, MemsZ1, VBat;

// Variables for Servo & Motors
bool servo_update_flag = true;
uint16_t ServoDC1, ServoDC2;
uint16_t ServoDC1ref = 1470;
uint16_t ServoDC2ref = 1470;
uint16_t ServoLimitUpper = 1980;	// Adjust for a better value on your car
uint16_t ServoLimitLower = 1000;	// Adjust for a better value on your car
uint8_t MotorSpeedCmd1=0, MotorSpeedCmd2=0;
uint16_t MotorSpeedCal=0;			// debug purpose
uint32_t MotorSpeedFdbk1=0, MotorSpeedFdbk2=0;
bool MotorDir1, MotorDir2;

uint16_t r1 = 0, r2 = 0, r3 = 0,r4 = 0,r5 = 0, r6 = 0, r7 = 0,r8 = 0;
uint16_t r0x = 0, r1x = 0, r2x = 0,r3x = 0;
uint16_t r0y = 0, r1y = 0, r2y = 0,r3y = 0;
uint16_t redx = 0, redy = 0, grex = 0, grey = 0;
uint16_t Rd0x = 0,Rd0y = 0;

// Variables for Linear CCD TSL1401
uint16_t CCDData[128];	// Linear CCD raw data
uint16_t TrackIndex;	// Target tracking point index from 1 to 128

// Variables for MT9V034
uint8_t Image_Data[120][188];	// MT9V034 DMA data receiver ROW*COL: 120*188
uint8_t Image_Use[60][94];	// resized CAM data to fit OLED
uint8_t Pixle[60][94];		// 0/1 CAM data
uint16_t CAM_DMA_Counter=0;
uint8_t CamLineCounter=0;
bool CamDataReady=0;	// MT9V034 new data frame ready in buffer

// Variables for autorun
uint16_t TrackErrorCounter=0;	// Invalid detection counter for off-track protection
uint8_t TrackFlag=0;	// 0: Invalid track detection
						// 1: Valid/Normal track found
						// 2: Start/End Mark detected
						// 3: Cross-road detected
						// 4: Undefined

void DMA_CAM_Repeat(uint8_t LineNumber)
{
	DMA_DMA_BASEADDR->TCD[DMA_CH0_DMA_CHANNEL].DADDR         = (uint32_t) &Image_Data[LineNumber][0];
	DMA_DMA_BASEADDR->TCD[DMA_CH0_DMA_CHANNEL].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(188);
	DMA_DMA_BASEADDR->TCD[DMA_CH0_DMA_CHANNEL].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(188);
	//EDMA_EnableChannelRequest(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL);
	DMA_DMA_BASEADDR->SERQ = DMA_SERQ_SERQ(DMA_CH0_DMA_CHANNEL);
}

/* DMA0_DMA16_IRQn interrupt handler */
void DMA_DMA_CH_INT_DONE_0_IRQHANDLER(void) {
  uint32_t CH0_status;

  /* Reading all flags of status register */
  CH0_status = EDMA_GetChannelStatusFlags(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL);
  /* Clearing all flags of the channel status register */
  EDMA_ClearChannelStatusFlags(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL, CH0_status);

  /* Place your code here */
  //CamDataReady = 1;
  //DMA_CAM_Repeat();

  /*
  CAM_DMA_Counter++;
  if ((CAM_DMA_Counter%50)==0)
	  LED2_TOGGLE();
	*/

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

void ShowNumHEX(uint8_t num) {
	uint16_t l;
	l = DSPTable[num & 0x0F];

	// DSP: PD9~PD15
	GPIO_PortSet(BOARD_INITPINS_SEG_A_GPIO, 0x7F << BOARD_INITPINS_SEG_A_PIN);	// turn off all DSP segments
	GPIO_PortClear(BOARD_INITPINS_SEG_A_GPIO, ((~l)&0x7F) << BOARD_INITPINS_SEG_A_PIN);	// turn on the code segments
}

void ShowNumDEC(uint8_t num) {
	uint16_t l;
	l = DSPTable[num % 10];

	// DSP: PD9~PD15
	GPIO_PortSet(BOARD_INITPINS_SEG_A_GPIO, 0x7F << BOARD_INITPINS_SEG_A_PIN);	// turn off all DSP segments
	GPIO_PortClear(BOARD_INITPINS_SEG_A_GPIO, ((~l)&0x7F) << BOARD_INITPINS_SEG_A_PIN);	// turn on the code segments
}

void ShowNumOFF(void) {
	// DSP: PD9~PD15
	GPIO_PortSet(BOARD_INITPINS_SEG_A_GPIO, 0x7F << BOARD_INITPINS_SEG_A_PIN);	// turn off all DSP segments
}

/* PORTB_IRQn interrupt handler */
void GPIOB_IRQHANDLER(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOB);

  /* Place your interrupt code here */
  // Rotary Encoder Switch
  if (pin_flags & BOARD_INITPINS_QEPA_GPIO_PIN_MASK)
  {
  	  GPIO_PortClearInterruptFlags(GPIOB, BOARD_INITPINS_QEPA_GPIO_PIN_MASK);
	  if (!GPIO_PinRead(BOARD_INITPINS_QEPB_GPIO, BOARD_INITPINS_QEPB_PIN))
	  {
		  QESVar++;
	  }
	  else
	  {
		  if (QESVar>0) QESVar--;
	  }
  }
  // HREF on PTB11
  if (pin_flags & BOARD_INITPINS_CAMHREF_GPIO_PIN_MASK)
  {
	  GPIO_PortClearInterruptFlags(GPIOB, BOARD_INITPINS_CAMHREF_GPIO_PIN_MASK);
	  // Start DMA transfer on new line
	  DMA_CAM_Repeat(CamLineCounter);
	  CamLineCounter++;
	  if (CamLineCounter >= 120)
	  {
		  CamLineCounter = 0;
	  }
	  return;
  }
  // VSYN on PTB16
  if (pin_flags & BOARD_INITPINS_CAMVSYN_GPIO_PIN_MASK)
  {
	  GPIO_PortClearInterruptFlags(GPIOB, BOARD_INITPINS_CAMVSYN_GPIO_PIN_MASK);
	  //CAM_DMA_Counter = EDMA_GetRemainingMajorLoopCount(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL);
	  //DMA_CAM_Repeat();
	  //DMALeftSize = EDMA_GetRemainingMajorLoopCount(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL);
	  CamDataReady = 1;
	  CamLineCounter = 0;
	  //DMA_CAM_Repeat();
	  CAM_DMA_Counter++;
	  if ((CAM_DMA_Counter%50)==0)
		  LED2_TOGGLE();
	  return;
  }

  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOB, pin_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}



/* PC command string parsing for RTC synchronizing */
void PcCmdProcess(void) {
    int year;	/*!< Range from 1970 to 2099.*/
    int month;	/*!< Range from 1 to 12.*/
    int day;	/*!< Range from 1 to 31 (depending on month).*/
    int hour;	/*!< Range from 0 to 23.*/
    int minute;	/*!< Range from 0 to 59.*/
    int second;	/*!< Range from 0 to 59.*/
    rtc_datetime_t PCTIME;


	if (CmdDataLen == 27U)	// "#cmd01data20230405-180000\r\n"
	{
		PRINTF("Command from PC received!\r\n");
		sscanf(CmdDataBuffer, "#cmd01data%4d%2d%2d-%2d%2d%2d",
				&year, &month, &day, &hour, &minute, &second);
		PRINTF("RTC: %04hd-%02hd-%02hd %02hd:%02hd:%02hd\r\n",
				year, month, day, hour, minute, second);

	    /* Set DATE TIME registers and start RTC */
		PCTIME.year   = year;
		PCTIME.month  = month;
		PCTIME.day    = day;
		PCTIME.hour   = hour;
		PCTIME.minute = minute;
		PCTIME.second = second;
	    RTC_StopTimer(RTC);
	    RTC_SetDatetime(RTC, &PCTIME);
	    RTC_StartTimer(RTC);
	}
	else
	{
		PRINTF("Invalid RTC synchronizing command!\r\n");
	}
}

/* UART0_RX_TX_IRQn interrupt handler */
void UART0_SERIAL_RX_TX_IRQHANDLER(void) {
	uint8_t data;
	uint32_t intStatus;

	/* Reading all interrupt flags of status registers */
	intStatus = UART_GetStatusFlags(UART0_PERIPHERAL);

	/* Place your code here */
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & intStatus)	// If new data arrived?
	{
		data = UART_ReadByte(UART0_PERIPHERAL);
		//UART_WriteByte(UART0_PERIPHERAL, data);	// send data back for loop test

		/* PC command data framing */
		/* RTC calibrate example: "#cmd01data20230405-180000\r\n" */
		if (data == '#')		// start of a data frame?
		{
			CmdDataLen = 0;		// reset cmd data counter
			CmdDataBuffer[0] = data;
		}
		else
		{
			CmdDataLen++;
			if (CmdDataLen < cmdDataLenMax)
			{
				CmdDataBuffer[CmdDataLen] = data;
				if (data == '\n')	// end of a data frame?
				{
					CmdDataReady = 1;	// a complete PC cmd data frame received
					CmdDataLen++;
					PcCmdProcess();		// cmd processing
				}
			}
			else	// no cmd data frame received when data buffer overflow occur
			{
				CmdDataLen = 0;
			}
		}

		/* If ring buffer is not full, add data to ring buffer. */
		if (((rxIndexUSB + 1) % USB_RING_BUFFER_SIZE) != txIndexUSB)
		{
			UsbRingBuffer[rxIndexUSB] = data;
			rxIndexUSB++;
			rxIndexUSB %= USB_RING_BUFFER_SIZE;
		}
	}

	UART_ClearStatusFlags(UART0_PERIPHERAL, intStatus);
	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
	Store immediate overlapping exception return operation might vector to incorrect interrupt. */
	#if defined __CORTEX_M && (__CORTEX_M == 4U)
	__DSB();
	#endif
}

/* UART1_RX_TX_IRQn interrupt handler */
void UART1_SERIAL_RX_TX_IRQHANDLER(void) {
  uint32_t intStatus;
  uint32_t uart_data=0;
  uint8_t RFCmd ;
  /* Reading all interrupt flags of status registers */
  intStatus = UART_GetStatusFlags(UART1_PERIPHERAL);

  /* Flags can be cleared by reading the status register and reading/writing data registers.
    See the reference manual for details of each flag.
    The UART_ClearStatusFlags() function can be also used for clearing of flags in case the content of data regsiter is not used.
    For example:
        status_t status;
        intStatus &= ~(kUART_RxOverrunFlag | kUART_NoiseErrorFlag | kUART_FramingErrorFlag | kUART_ParityErrorFlag);
        status = UART_ClearStatusFlags(UART1_PERIPHERAL, intStatus);
  */

  /* Place your code here */

  if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & intStatus)	// If new data arrived?
  	{
	    RFCmd = UART_ReadByte(UART1_PERIPHERAL);
  		UART_WriteByte(UART0_PERIPHERAL, RFCmd);	// send data back for loop test
  		//PRINTF("RFCMD%2d\r\n",RFCmd);

  		/* PC command data framing */
  		/* RTC calibrate example: "#cmd01data20230405-180000\r\n" */
  		if (RFCmd == '#')		// start of a data frame?
  		{
  			CmdDataLen1 = 0;		// reset cmd data counter
  			CmdDataBuffer1[0] = RFCmd;
  		}
  		else
  		{
  			CmdDataLen1++;
  			if (CmdDataLen1 < cmdDataLenMax1)
  			{
  				CmdDataBuffer1[CmdDataLen1] = RFCmd;
  				if (RFCmd == '\n')	// end of a data frame?
  				{
  					CmdDataReady1 = 1;	// a complete PC cmd data frame received
  					CmdDataLen1=0;
  					//scanf("#r1x:%dy:%dr2x:%dy:%dr3x:%dy:%dr0x:%dy:%db_gx:%dy:%dEND\n",&r1x,&r1y,&r2x,&r2y,&r3x,&r3y,&r0x,&r0y,&redx,&redy);
  					//sscanf(CmdDataBuffer1,"#r1x:%dy:%dr2x:%dy:%dr3x:%dy:%dr0x:%dy:%db_gx:%dy:%d END\n",&r1x,&r1y,&r2x,&r2y,&r3x,&r3y,&r0x,&r0y,&redx,&redy);
  					//PRINTF(CmdDataBuffer1);
  					//PRINTF("\n");


  					//sscanf(CmdDataBuffer1, "%d",
  					//				&uart_data);
  					//PRINTF("UART%2d\r\n",uart_data);
  				}
  			}
  			else	// no cmd data frame received when data buffer overflow occur
  			{
  				CmdDataLen1 = 0;
  			}
  		}

  		/* If ring buffer is not full, add data to ring buffer.
  		if (((rxIndexUSB + 1) % USB_RING_BUFFER_SIZE) != txIndexUSB)
  		{
  			UsbRingBuffer[rxIndexUSB] = data;
  			rxIndexUSB++;
  			rxIndexUSB %= USB_RING_BUFFER_SIZE;
  		}*/
  	}

    status1 =UART_ClearStatusFlags(UART1_PERIPHERAL, intStatus);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


/* RTC_Seconds_IRQn interrupt handler */
void RTC_SECONDS_IRQHANDLER(void) {
  /* Get status flags */
  uint32_t status_flags = RTC_GetStatusFlags(RTC_PERIPHERAL);

  /* Place your interrupt code here */
  ElapsedSeconds++;
  /* Clear status flags */
  RTC_ClearStatusFlags(RTC_PERIPHERAL, status_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void delay_short(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 20000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

uint8_t InverseByteBits(uint8_t data)
{
	uint8_t tempChar;

	tempChar = data;
	tempChar=(tempChar<<4)|(tempChar>>4);
	tempChar=((tempChar<<2)&0xcc)|((tempChar>>2)&0x33);
	tempChar=((tempChar<<1)&0xaa)|((tempChar>>1)&0x55);

	return tempChar;
}

void BOARD_InitI2C(void) {
    uint8_t deviceAddress;

    /* CAT9555 communication protocal */
	// Polarity
    g_master_txBuff[0] = 0x0U;
    g_master_txBuff[1] = 0x0U;
    deviceAddress     = 0x04U;
    masterXfer.slaveAddress   = I2C_CAT9555_SLAVE_ADDR_7BIT;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = g_master_txBuff;
    masterXfer.dataSize       = I2C_CAT9555_DATA_LENGTH;
    masterXfer.flags          = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C_CAT9555_BASEADDR, &masterXfer);
    delay();

    // IO Configuration
    g_master_txBuff[0] = 0U;
    g_master_txBuff[1] = 0U;
    deviceAddress     = 0x06U;
    masterXfer.subaddress     = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = g_master_txBuff;
    masterXfer.dataSize       = I2C_CAT9555_DATA_LENGTH;
    I2C_MasterTransferBlocking(I2C_CAT9555_BASEADDR, &masterXfer);
    delay();
}

void BOARD_I2C_GPIO(uint16_t ctrlValue) {
    uint8_t deviceAddress;

    g_master_txBuff[0] = ~(ctrlValue&0xFFU);
    g_master_txBuff[1] = ~(InverseByteBits((ctrlValue>>8)&0xFFU));
    deviceAddress     = 0x02U;
    masterXfer.slaveAddress   = I2C_CAT9555_SLAVE_ADDR_7BIT;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = g_master_txBuff;
    masterXfer.dataSize       = I2C_CAT9555_DATA_LENGTH;
    masterXfer.flags          = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C_CAT9555_BASEADDR, &masterXfer);
}

void CDK66_Analog_Input(Board_Analog_t *AnalogInput) {
    //adc16_channel_config_t adc16ChannelConfigStruct;
    adc16_channel_mux_mode_t ADCMUX0, ADCMUX1;

    ADCMUX0 = kADC16_ChannelMuxA;
    ADCMUX1 = kADC16_ChannelMuxA;
    ADC16_SetChannelMuxMode(ADC0, ADCMUX0);
    ADC16_SetChannelMuxMode(ADC1, ADCMUX1);

    // ADC0
    // MEMSx: ADC0, se19
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH1_CONTROL_GROUP, &ADC0_channelsConfig[1]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH1_CONTROL_GROUP))) {}
    AnalogInput->x = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP) & 0xFFF;
    // MEMSy: ADC0, se3
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP, &ADC0_channelsConfig[0]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP))) {}
    AnalogInput->y = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP) & 0xFFF;
    // HALL2: ADC0, se17
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH2_CONTROL_GROUP, &ADC0_channelsConfig[2]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH2_CONTROL_GROUP))) {}
    AnalogInput->HALL2 = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH2_CONTROL_GROUP) & 0xFFF;
    // IRV2: ADC0, se23
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH3_CONTROL_GROUP, &ADC0_channelsConfig[3]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH3_CONTROL_GROUP))) {}
    AnalogInput->IRV2 = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH3_CONTROL_GROUP) & 0xFFF;
    // CCD: ADC0, se14
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP, &ADC0_channelsConfig[4]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP))) {}
    AnalogInput->CCD = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP) & 0xFFF;

    // ADC1
    // MEMSz: ADC1, se19
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP, &ADC1_channelsConfig[0]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP))) {}
    AnalogInput->z = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP) & 0xFFF;
    // IRV1: ADC1, se18
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH1_CONTROL_GROUP, &ADC1_channelsConfig[1]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, ADC1_CH1_CONTROL_GROUP))) {}
    AnalogInput->IRV1 = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, ADC1_CH1_CONTROL_GROUP) & 0xFFF;
    // HALL1: ADC1, se23
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH2_CONTROL_GROUP, &ADC1_channelsConfig[2]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, ADC1_CH2_CONTROL_GROUP))) {}
    AnalogInput->HALL1 = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, ADC1_CH2_CONTROL_GROUP) & 0xFFF;
    // Temperature: ADC1, se26
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH3_CONTROL_GROUP, &ADC1_channelsConfig[3]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, ADC1_CH3_CONTROL_GROUP))) {}
    AnalogInput->Temp = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, ADC1_CH3_CONTROL_GROUP) & 0xFFF;

    return;
}

/* PIT0_IRQn interrupt handler */
void PIT_CHANNEL_0_IRQHANDLER(void) {
  uint32_t intStatus;
  /* Reading all interrupt flags of status register */
  intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
  PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);

  /* Place your code here */
  FTM_SetSoftwareTrigger(FTM3_PERIPHERAL, true);
  //servo_update_flag = true;

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


    int16_t Error = 0;
	int32_t Kp = 5;
	int32_t Ki = 0.1;
	static int32_t PrevError = 0;
	static int32_t IntegError = 0;



// appTicks increasing at an interval of 5ms
void PIT_CHANNEL_1_IRQHANDLER(void)
{
	uint32_t intStatus;

	/* Reading all interrupt flags of status register */
	intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_1);
	PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_1, intStatus);

	/* Place your code here */
    /* Read Encoder#1 pulses value */

    MotorSpeedFdbk1 = FTM_GetQuadDecoderCounterValue(FTM1_PERIPHERAL);  //光电编码器脉冲数
                                                                        //当前车辆速度=光电编码器脉冲数/齿轮传动比*车轮周长
    /* Clear Encoder#1 pulses counter */
    FTM_ClearQuadDecoderCounterValue(FTM1_PERIPHERAL);
    /* Read Motor#1 running direction */
    if (FTM_GetQuadDecoderFlags(FTM1_PERIPHERAL) & kFTM_QuadDecoderCountingIncreaseFlag)
    {
        MotorDir1 = 1;
    }
    else if (MotorSpeedFdbk1 != 0)
    {
        MotorDir1 = 0;
        MotorSpeedFdbk1 = 5000 - MotorSpeedFdbk1;
    }




    if (MotorSpeedFdbk1 < MotorSpeedCmd1*20)
    {
        /* Calculate the error between the desired speed and the actual speed */
        Error = MotorSpeedCmd1 - MotorSpeedFdbk1/20/*/GEAR_RATIO/WHEEL_CIRCUMFERENCE*/;

        /* Update the integral term of the PID controller */
        IntegError += Error;

        /* Calculate the control value using the PID controller */
        MotorSpeedCal = (uint16_t)(Kp * Error + Ki * IntegError ) /*(Error - PrevError))*/;
        if(MotorSpeedCal > 40) MotorSpeedCal = 40;
    }
    else
    {
       	if (MotorSpeedCal>0)
       		MotorSpeedCal--;
        else
          {
              /* Reset the integral term of the PID controller */
              IntegError = 0;
          }
    }
    PrevError = Error;
    /*if (MotorSpeedFdbk1 < MotorSpeedCmd1*20)
    {
    	MotorSpeedCal++;
    }
    else
    {
    	if (MotorSpeedCal>0)
    		MotorSpeedCal--;
    }*/

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
       Store immediate overlapping exception return operation might vector to incorrect interrupt. */
    #if defined __CORTEX_M && (__CORTEX_M == 4U)
      __DSB();
    #endif
}

/*ServoMotor controller*/
void static inline Update_ServoUS(uint8_t ChanNo, uint32_t DutyCycleUS)
{
	uint32_t cv, mod;

	if ((ChanNo<2) && (DutyCycleUS<=2500) && (DutyCycleUS>=500))
	{
		mod = FTM3_PERIPHERAL->MOD;
		cv = (mod*DutyCycleUS) / 20000U;
		if (cv >= mod)
		{
			cv = mod + 1U;
		}
		FTM3_PERIPHERAL->CONTROLS[ChanNo].CnV = cv;
		//FTM_SetSoftwareTrigger(FTM3_PERIPHERAL, true);
	}
	return;
}

// Count value calibrated with 1 second timing to be 142
void delay1ms(uint16_t delayTime) {
	uint32_t delayCount;
	delayCount = delayTime * USEC_TO_COUNT(142, CLOCK_GetFreq(SYS_CLK));
	//delayCount = delayTime * USEC_TO_COUNT(100, CLOCK_GetFreq(kCLOCK_BusClk));

	while(delayCount--) {}
}

void CCD_delay()
{
	uint16_t delayCount;
	delayCount = 20;
	while(delayCount--) {}
}

void CCD_IntegratingDelay(uint16_t delayCount)
{
	if (delayCount==0) return;
	while(delayCount--) {}
}


/* Drive to and read from TSL1401CL */
void CollectCCD(void)
{
    uint8_t index=0;

    CCD_CLK_HIGH;
    CCD_delay();
    CCD_SI_LOW;
    CCD_delay();

    CCD_SI_HIGH;
    CCD_delay();
    CCD_CLK_LOW;
    CCD_delay();

    CCD_CLK_HIGH;
    CCD_delay();
    CCD_SI_LOW;
    CCD_delay();

    for(index=0; index<128; index++)
    {
        CCD_CLK_LOW;
        CCD_delay();

        ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP, &ADC0_channelsConfig[4]);
        while (0U == (kADC16_ChannelConversionDoneFlag &
                      ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP))) {}
        CCDData[index] = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP) & 0xFFF;

        CCD_CLK_HIGH;
        CCD_delay();
    }
}

// simply generate SI & CLK pulses to flush the previously integrated frame
// while integrating new frame to be read out
void LinearCameraFlush(void)
{
    uint8_t index=0;

    CCD_SI_HIGH;
    CCD_delay();
    CCD_CLK_HIGH;	// 1st Pulse
    CCD_delay();
    CCD_SI_LOW;
    CCD_delay();
    CCD_CLK_LOW;

    for(index=0; index<128; index++)
    {
        CCD_CLK_HIGH;
        //CCD_delay();
        CCD_IntegratingDelay(150);
        CCD_CLK_LOW;
        //CCD_delay();
        CCD_IntegratingDelay(150);
    }
}

void LinearCameraOneShot(void)
{
    uint8_t index=0;

    // flush previously integrated frame before capturing new frame
    LinearCameraFlush();
    // wait for TSL1401 to integrate new frame, exposure time control by delay
    delay1ms(10);

    CCD_SI_HIGH;
    CCD_delay();
    CCD_CLK_HIGH;
    CCD_delay();
    CCD_SI_LOW;
    CCD_delay();
    CCD_CLK_LOW;

    // ADC mux to CCD channel
    ADC16_SetChannelMuxMode(ADC0, kADC16_ChannelMuxA);
    for(index=0; index<128; index++)
    {
    	// read AO value using ADC
        ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP, &ADC0_channelsConfig[4]);
        while (0U == (kADC16_ChannelConversionDoneFlag &
                      ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP))) {}
        CCDData[index] = 2*ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP) & 0xFFF;
        // clock pulse to read next pixel output
        CCD_CLK_HIGH;
        CCD_delay();
        CCD_CLK_LOW;
    }

    // 129th pulse to terminate output of 128th pixel
    CCD_CLK_HIGH;
    CCD_delay();
    CCD_CLK_LOW;
}

void Draw_LinearView(void)
{
    uint8_t index=0, tempData;
    uint8_t i=0, j=0, k=0;

    for(index=0; index<128; index++)
	{
    	i = 7 - (CCDData[index]>>6)/8;
    	j = 7- (CCDData[index]>>6)%8;
    	for (k=0; k<8; k++)
    	{
    		OLED_Set_Pos(index, k);
    		if (i==k)
    		{
    			tempData = (1U << j);
    		}
    		else
    		{
    			tempData = 0;
    		}
    		OLED_WrDat(tempData);
    	}
	}
}

void Get_Use_Image()
{
	uint16_t i=0, j=0, row=0, col=0;

	for(i=0; i<120; i+=2)		// ROW
	{
		for(j=0; j<188; j+=2)	// COL
		{
			Image_Use[row][col] = Image_Data[i][j];
			col++;
		}
		col = 0;
		row++;
	}
}


void Draw_LinearViewHalf(void)
{
    uint8_t index=0, tempData;
    uint8_t i=0, j=0, k=0;

    for(index=0; index<128; index++)
	{
    	i = 7 - (CCDData[index]>>10);
    	j = 7- (CCDData[index]>>7)%8;
    	for (k=4; k<8; k++)
    	{
    		OLED_Set_Pos(index, k);
    		if (i==k)
    		{
    			tempData = (1U << j);
    		}
    		else
    		{
    			tempData = 0;
    		}
    		OLED_WrDat(tempData);
    	}
	}
}

void KC4D(void)
{
    OLED_KC4D(16,2);
    BEEP_ON();
    delay_short();
    BEEP_OFF();
    delay();
    BEEP_ON();
    delay();
    BEEP_OFF();
    delay();
    delay();
}

uint8_t GetOTSU(uint8_t image[60][94])
{
	uint16_t i, j;
	uint32_t Amount = 0;
	uint32_t PixelBack = 0;
	uint32_t PixelIntegralBack = 0;
	uint32_t PixelIntegral = 0;
	uint32_t PixelIntegralFore = 0;
	uint32_t PixelFore = 0;
	float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
	uint16_t MinValue, MaxValue;
	uint8_t Threshold = 0;
	uint8_t HistoGram[256];

	for (j=0; j<256; j++)  HistoGram[j] = 0; //初始化灰度直方图

	for (j=0; j<60; j++)
	{
		for (i = 0; i < 94; i++)
		{
			HistoGram[image[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
		}
	}

	for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
	for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

	if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色
	if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色

	for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数

	PixelIntegral = 0;
	for (j = MinValue; j <= MaxValue; j++)
	{
		PixelIntegral += HistoGram[j] * j;//灰度值总数
	}
	SigmaB = -1;
	for (j = MinValue; j < MaxValue; j++)
	{
		PixelBack = PixelBack + HistoGram[j];   //前景像素点数
		PixelFore = Amount - PixelBack;         //背景像素点数
		OmegaBack = (float)PixelBack / Amount;//前景像素百分比
		OmegaFore = (float)PixelFore / Amount;//背景像素百分比
		PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
		MicroBack = (float)PixelIntegralBack / PixelBack;   //前景灰度百分比
		MicroFore = (float)PixelIntegralFore / PixelFore;   //背景灰度百分比
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
		if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
		{
			SigmaB = Sigma;
			Threshold = j;
		}
	}
	return Threshold;                        //返回最佳阈值;

}

// threshold mode: 0 - OTSU; 1 - average
void Get_01_Value(uint8_t mode)
{
	uint16_t i=0, j=0;
	uint8_t Threshold;
	uint32_t tv=0;
	char txt[16];

	if (mode)
	{
		for (i=0; i<60; i++)
		{
			for (j=0; j<94; j++)
				{
					tv += Image_Use[i][j];   //累加
				}
		}
		Threshold = tv/60/94;        	//求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
		Threshold = Threshold*8/10+10;	//此处阈值设置，根据环境的光线来设定
	}
	else
	{
		Threshold = GetOTSU(Image_Use);	//大津法阈值
		Threshold = (uint8_t)(Threshold*0.5)+70;
	}
	sprintf(txt,"%03d", Threshold);
	OLED_P6x8Str(80, 1, (uint8_t *)txt);

	for (i=0; i<60; i++)
	{
		for(j=0; j<94; j++)
		{
			if(Image_Use[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
				Pixle[i][j] = 1;
			else
				Pixle[i][j] = 0;
		}
	}
}

void Draw_CameraView()
{
	uint8_t i=0, j=0, temp=0;

	for(i=8;i<56;i+=8)//6*8=48行
	{
		OLED_Set_Pos(18, i/8+1);	//起始位置
		for(j=0; j<94; j++)  	//列数
		{
			temp = 0;
			if(Pixle[0+i][j]) temp |= 1;
			if(Pixle[1+i][j]) temp |= 2;
			if(Pixle[2+i][j]) temp |= 4;
			if(Pixle[3+i][j]) temp |= 8;
			if(Pixle[4+i][j]) temp |= 0x10;
			if(Pixle[5+i][j]) temp |= 0x20;
			if(Pixle[6+i][j]) temp |= 0x40;
			if(Pixle[7+i][j]) temp |= 0x80;
			OLED_WrDat(temp);
		}
	}
}

void SendImage(	void)
{
	uint8_t PcImageHeader[4];

	PcImageHeader[0] = 0x00;
	PcImageHeader[1] = 0xFF;
	PcImageHeader[2] = 0x01;
	PcImageHeader[3] = 0x01;

	//发送帧头标志
	UART_WriteBlocking(UART0_PERIPHERAL, (uint8_t *)PcImageHeader, 4U);
	//发送灰度图像
	UART_WriteBlocking(UART0_PERIPHERAL, (uint8_t *)Image_Data, 22560U);
}

void MotorCtrlOL(uint8_t SpeedCmd)
{
	if(SpeedCmd>100)
		return;

	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, SpeedCmd);
	FTM_SetSoftwareTrigger(FTM0_PERIPHERAL, true);
}

// Closed-loop speed control
// feedback available on MotorSpeedFdbk1
void MotorCtrlCL(void)
{
	uint16_t CtrlValue=0;

	CtrlValue = MotorSpeedCal;		// MotorSpeedCal updated in PIT IRQ at an interval of 5ms
	if (CtrlValue > 40) CtrlValue = 40;

	//FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, SpeedCmd);
	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, CtrlValue);
	FTM_SetSoftwareTrigger(FTM0_PERIPHERAL, true);

}

// Your code here to find the target tracking point
// Result: update TrackIndex value
// uint8_t TrackFlag=0;	// 0: Invalid track detection
//						// 1: Valid/Normal track found
//						// 2: Start/End Mark detected
//						// 3: Cross-road detected
//						// 4: Undefined

int stage = 0;
int last_stage = 0;
int n = 0;
int average;
int num = 0;

void FindTrackPoint(void)
{
	int16_t DMax, DMin, DV;
	uint8_t MaxIndex, MinIndex;
	uint16_t i;
	uint16_t max;
	uint16_t i_max;
	num = 0;

	DMax = 0;
	DMin = 0;
	max = 0;
	average = 0;

	for (i=0; i<126; i++)
	{
		average = average + CCDData[i];
		if(abs(CCDData[i+2] - CCDData[i]) > 150)
		{
			num++;
		}
		if(CCDData[i]>max)
		{
			max = CCDData[i];
			i_max = i;
		}

		DV = CCDData[i+2] - CCDData[i];
		if (DV > DMax)		// Raising edge
		{
			DMax = DV;
			MaxIndex = i+1;
		}
		if (DV < DMin)		// Falling edge
		{
			DMin = DV;
			MinIndex = i+1;
		}
	}
	average = average / 126;
	if(num == 0)
	{
		if(average < 330)
			{
				stage = 2;
			}
		//stage = 2;
	}
	else if(num >= 2 && num < 6)
	{
		 stage = 0;
	}
	else if(num >= 8)
	{
		 stage = 1;
	}

	if(average < 330)
	{
		stage = 2;
	}
	// Cross-road?
	// Start/End mark?
	if ((MaxIndex<MinIndex) || (DMax<120) || (DMin>(-120)))	// Invalid cases
	{
		TrackFlag = 0;
		//TrackErrorCounter++;
	}
	else
	{
		if ((MaxIndex-MinIndex) > 30)	// One of invalid cases
		{
			TrackFlag = 0;
			//TrackErrorCounter++;
		}
		else
		{
			TrackFlag = 1;		// Normal track found
			TrackIndex = (MaxIndex+MinIndex)/2;
			//TrackErrorCounter = 0;
		}
	}
}

// indicate TrackIndex on OLED
void Draw_Mark(TrackIndex)
{
	uint8_t i=0;
	for (i=4; i<8; i++)
	{
		OLED_Set_Pos(TrackIndex, i);
		OLED_WrDat(0xFF);
	}
}

int SpeedZhi = 0;
int SpeedWan = 0;

void BEEP()
{
	BEEP_ON();
	delay();
	BEEP_OFF();
	delay();
	BEEP_ON();
	delay();
	BEEP_OFF();
}
/*
 * @brief   Application entry point.
 */
int tmx = 0, tmy = 0, tmi;

/*
void Follow_A4(){

 		//x0-r0
 		//算边(实际最后是三条边)

 		xeg1 = sqrt(1+(x0 - x0)*(x0 - x0)*0.00467*0.00467 + (y0 - y0)*(y0 - y0)*0.00467*0.00467);//a1
		xeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (y0 - y0)*(y0 - y0)*0.00467*0.00467);//c
		yeg1 = xeg2;
		yeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//a2
 		//算角
 		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2 - (x0 - r0x)*(x0 - r0x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
		delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2 - (y0 - r0y)*(y0 - r0y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
		deltx = (int)(abs(deltx));
		delty = (int)(abs(delty));
		if(deltx<400 && delty <400){
			tmx = deltx/20;
			txy = delty/20;
			for(tmi = 0; tmi<20;tmi++){
				if(r0x<b_rx){
	     			LoopCounter2+=tmx;
	     		}
	     		else{
	     			LoopCounter2-=tmx;
	     		}
	     		if(r0y>b_ry){
	     			LoopCounter+=tmy;
	     		}
	     		else{
	     			LoopCounter-=tmy;
	     		}

	         	Update_ServoUS(kFTM_Chnl_0, LoopCounter);
	      	    Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

	         	delay_short();
			}

		}
		Round++;
		delay_short();
 		//r0-r1

 		//算边(实际最后是三条边)

 		xeg1 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//a1
		xeg2 = sqrt(1*1+(r1x - x0)*(r1x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//c
		yeg1 = xeg2;
		yeg2 = sqrt(1*1+(r1x - x0)*(r1x - x0)*0.00467*0.00467 + (r1y - y0)*(r1y - y0)*0.00467*0.00467);//a2
 		//算角
 		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (r0x - r1x)*(r0x - r1x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
		delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (r0y - r1y)*(r0y - r1y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
		deltx = (int)(abs(deltx));
		delty = (int)(abs(delty));
		if(deltx<400 && delty <400){
			tmx = deltx/20;
			txy = delty/20;
			for(tmi = 0; tmi<20;tmi++){
				if(r1x<r0x){
	     			LoopCounter2+=tmx;
	     		}
	     		else{
	     			LoopCounter2-=tmx;
	     		}
	     		if(r1y>r0y){
	     			LoopCounter+=tmy;
	     		}
	     		else{
	     			LoopCounter-=tmy;
	     		}

	         	Update_ServoUS(kFTM_Chnl_0, LoopCounter);
	      	    Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

	         	delay_short();
			}

		}
		delay_short();
		Round++;
 		//r1-r2


 		//算边(实际最后是三条边)

 		xeg1 = sqrt(1*1+(r1x - x0)*(r1x - x0)*0.00467*0.00467 + (r1y - y0)*(r1y - y0)*0.00467*0.00467);//a1
		xeg2 = sqrt(1*1+(r2x - x0)*(r2x - x0)*0.00467*0.00467 + (r1y - y0)*(r1y - y0)*0.00467*0.00467);//c
		yeg1 = xeg2;
		yeg2 = sqrt(1*1+(r2x - x0)*(r2x - x0)*0.00467*0.00467 + (r2y - y0)*(r2y - y0)*0.00467*0.00467);//a2
 		//算角
 		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (r1x - r2x)*(r1x - r2x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
		delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (r1y - r2y)*(r1y - r2y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
		deltx = (int)(abs(deltx));
		delty = (int)(abs(delty));
		if(deltx<400 && delty <400){
			tmx = deltx/20;
			txy = delty/20;
			for(tmi = 0; tmi<20;tmi++){
				if(r2x<r1x){
	     			LoopCounter2+=tmx;
	     		}
	     		else{
	     			LoopCounter2-=tmx;
	     		}
	     		if(r2y>r1y){
	     			LoopCounter+=tmy;
	     		}
	     		else{
	     			LoopCounter-=tmy;
	     		}

	         	Update_ServoUS(kFTM_Chnl_0, LoopCounter);
	      	    Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

	         	delay_short();
			}

		}
		delay_short();
		Round++;
 		//red-r3


 		//算边(实际最后是三条边)

 		xeg1 = sqrt(1*1+(r2x - x0)*(r2x - x0)*0.00467*0.00467 + (r2y - y0)*(r2y - y0)*0.00467*0.00467);//a1
		xeg2 = sqrt(1*1+(r3x - x0)*(r3x - x0)*0.00467*0.00467 + (r2y - y0)*(r2y - y0)*0.00467*0.00467);//c
		yeg1 = xeg2;
		yeg2 = sqrt(1*1+(r3x - x0)*(r3x - x0)*0.00467*0.00467 + (r3y - y0)*(r3y - y0)*0.00467*0.00467);//a2
 		//算角
 		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (r2x - r3x)*(r2x - r3x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
		delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (r2y - r3y)*(r2y - r3y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
		deltx = (int)(abs(deltx));
		delty = (int)(abs(delty));
		if(deltx<400 && delty <400){
			tmx = deltx/20;
			txy = delty/20;
			for(tmi = 0; tmi<20;tmi++){
				if(r3x<r2x){
	     			LoopCounter2+=tmx;
	     		}
	     		else{
	     			LoopCounter2-=tmx;
	     		}
	     		if(r3y>r2y){
	     			LoopCounter+=tmy;
	     		}
	     		else{
	     			LoopCounter-=tmy;
	     		}

	         	Update_ServoUS(kFTM_Chnl_0, LoopCounter);
	      	    Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

	         	delay_short();
			}

		}
		delay_short();
 		//red-r0


 		//算边(实际最后是三条边)

 		xeg1 = sqrt(1*1+(r3x - x0)*(r3x - x0)*0.00467*0.00467 + (r3y - y0)*(r3y - y0)*0.00467*0.00467);//a1
		xeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r3y - y0)*(r3y - y0)*0.00467*0.00467);//c
		yeg1 = xeg2;
		yeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//a2
 		//算角
 		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (r3x - r0x)*(r3x - r0x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
		delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (r3y - r0y)*(r3y - r0y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
		deltx = (int)(abs(deltx));
		delty = (int)(abs(delty));
		if(deltx<400 && delty <400){
			tmx = deltx/20;
			txy = delty/20;
			for(tmi = 0; tmi<20;tmi++){
				if(r0x<r3x){
	     			LoopCounter2+=tmx;
	     		}
	     		else{
	     			LoopCounter2-=tmx;
	     		}
	     		if(r0y>r3y){
	     			LoopCounter+=tmy;
	     		}
	     		else{
	     			LoopCounter-=tmy;
	     		}

	         	Update_ServoUS(kFTM_Chnl_0, LoopCounter);
	      	    Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

	         	delay_short();
			}

		}
		delay_short();



	}

}*/


// Variables for PID controller
    			float error = 0;
    			float integral = 0;
    			float derivative = 0;
    			float last_error = 0;

    			// Set the initial reference and state values
    			const int TrackIndex_base = 64;
    			const int tempInt_base = 1470;
    			const int tempInt_min = 1150;
    			const int tempInt_max = 1960;

uint8_t Round = 0;
uint8_t round = 0;
uint8_t sscan = 0;
double x0 = 0, y0 = 0;//base point
float Dx = 0, Dy = 0;
double xeg1 = 0,xeg2 = 0;//横移三角的余弦公式的两边长
float yeg1 = 0,yeg2 = 0;//纵移三角的余弦公式的两边长
float deltx = 0, delty = 0;




int main(void) {
	x0 = 94;
	y0 = 71;
    uint8_t TaskID = 0;		// loop switch
    uint32_t LastSecond=0;
    //uint16_t ServoDutyCounter=0;
    //bool ServoDutyCountDir=1;		// 0: count down; 1: count up
	uint32_t tempInt = 0;
	uint16_t CCDMin = 0;
	uint8_t index = 0;
	uint8_t CCD2PC[260];	// data to be sent to PC
	//uint32_t LastSecond=0;
    uint16_t Counter=0;//通道0减速倍速位
    uint16_t Counter2=0;//通道1减速倍速位
    uint16_t LoopCounter=1440;//通道0绕Y轴转计数计数
    uint16_t LoopCounter2=1400;//通道1绕X轴转计数
	char OLEDLine1[] = "0123456789abcdef";
	char OLEDLine2[] = "0123456789abcdef";
	char OLEDLine3[] = "0123456789abcdef";
	char OLEDLine4[] = "0123456789abcdef";

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    BOARD_InitI2C();
    OLED_Init();

    KC4D();		// Welcome message
    // Header of CCD data frame, seekfree protocol
    CCD2PC[0] = 0x00;
	CCD2PC[1] = 0xFF;
	CCD2PC[2] = 0x01;
	CCD2PC[3] = 0x00;

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

//	TaskID = (~((SW1()<<3) + (SW2()<<2) + (SW3()<<1) + SW4())) & 0x0F;
//    PRINTF("Task ID: %d\r\n", TaskID);

    LastSecond = ElapsedSeconds;
    LED1_ON();    LED2_OFF();

    volatile static int i = 0 ;
    //ServoDutyCountDir = 1;	// count up
    //ServoDutyCounter = 500;	// Lower Limit
/*
	if (TaskID == 1U)		// Test RC/Servo
	{
		QESVar = ServoDC1ref;
		ServoDC1 = ServoDC1ref;
		ServoDC2 = ServoDC2ref;
	}
    if (TaskID == 7U)		// using MT9V034?
		MT9V034_Init(50);
	if ((TaskID == 4U) || (TaskID == 9U))		// Test Motor driver?
	{
		M1_DISABLE();
		M2_DISABLE();
		FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_2, kFTM_EdgeAlignedPwm, 0);
		FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, 0);
    	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_4, kFTM_EdgeAlignedPwm, 0);
    	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_7, kFTM_EdgeAlignedPwm, 0);
		M1_ENABLE();
	}*/

    OLED_Init();

    static int r1x, r1y, r2x, r2y, r3x, r3y, r0x, r0y, b_rx, b_ry;
    static int nr1x, nr1y, nr2x, nr2y, nr3x, nr3y, nr0x, nr0y, nb_rx, nb_ry;
    static int lastb_rx = 0, lastb_ry = 0;
    static int lastr1x=0, lastr1y=0, lastr2x=0, lastr2y=0, lastr3x=0, lastr3y=0, lastr0x=0, lastr0y=0;

    while(1) {

    	 delay();
   // 	 sscan++;
    //	 if(sscan < 10){
    	    	//sscanf(CmdDataBuffer1, "#rx:%d y:%d rx:%d y:%d rx:%d y:%d rx:%d y:%d brx:%d y:%dEND",
    	    	        	      //     &r1x, &r1y, &r2x, &r2y, &r3x, &r3y, &r0x, &r0y, &rx, &ry);
    	    	sscanf(CmdDataBuffer1, "#r1x:%d y:%d r2x:%d y:%d r3x:%d y:%d r0x:%d y:%d r1x:%d y:%d r2x:%d y:%d r3x:%d y:%d r0x:%d y:%d b_rx:%d y:%dEND",
    	    	           &r1x, &r1y, &r2x, &r2y, &r3x, &r3y, &r0x, &r0y, &r1x, &r1y, &r2x, &r2y, &r3x, &r3y, &r0x, &r0y, &b_rx, &b_ry);

    	// }

    	PRINTF("r1x = %d\r\n", r1x);
    	PRINTF("r1y = %d\r\n", r1y);
        PRINTF("r2x = %d\r\n", r2x);
    	PRINTF("r2y = %d\r\n", r2y);
    	PRINTF("r3x = %d\r\n", r3x);
    	PRINTF("r3y = %d\r\n", r3y);
    	PRINTF("r0x = %d\r\n", r0x);
    	PRINTF("r0y = %d\r\n", r0y);
    	PRINTF("b_rx = %d\r\n", b_rx);
    	PRINTF("b_ry = %d\r\n", b_ry);
    	PRINTF("xeg1 = %lf\r\n",xeg1);
    	PRINTF("xeg2 = %lf\r\n",xeg2);
    	PRINTF("round = %d\r\n",round);
    	PRINTF("Round = %d\r\n",Round);
    	PRINTF("x0 = %lf\r\n",x0);
    	PRINTF("y0 = %lf\r\n",y0);
    	PRINTF("deltx = %f\r\n",deltx);
    	PRINTF("delty = %f\r\n",delty);
    	PRINTF("LoopCounter = %hu\r\n", LoopCounter);
    	PRINTF("LoopCounter2 = %hu\r\n", LoopCounter2);
    	PRINTF("\r\n");

     	if (KEY1()) {
     		LoopCounter=1370;
     		LoopCounter2=1400;
     		TaskID = 0;//reset
     		sscan = 0;
     	}
     	if (KEY2()) {
     		LoopCounter=1206;
     		LoopCounter2=1240;
     		round = 0;
     		TaskID = 1;//0.5*0.5
     	}
     	if (BTN()) {
     		LoopCounter=1360;
     		LoopCounter2=1400;
     		round = 0;
     		Round = 0;
     		TaskID = 2;//a4
     	}
     	if (KEY2() && KEY1()) {
     		LoopCounter=1285;
     		LoopCounter2=1309;
     		round = 0;
     		TaskID = 3;//a4 off
     	}


     	switch(TaskID){//model0-2
     	case 0://reset

     		//蜂鸣器
     		/*
     		BEEP_ON();
     		LED1_ON();
     		LED2_ON();
     		delay_short();
     		BEEP_OFF();
     		LED1_OFF();
     		LED2_OFF();
     		delay_short();

     		BEEP_ON();
     		LED1_ON();
     		LED2_ON();
     		delay_short();
     		BEEP_OFF();
     		LED1_OFF();
     		LED2_OFF();
     		delay_short();

     		BEEP_ON();
     		LED1_ON();
     		LED2_ON();
     		delay_short();
     		BEEP_OFF();
     		LED1_OFF();
     		LED2_OFF();
     		delay_short();

     		*/


     		break;


     	case 1://0.5*0.5
     		switch(round){
     		case 0:
     			//if(Counter<5)Counter++;
     			//if(Counter>=5 )Counter = 0;
     			if(Counter == 0){
     				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter++;
     			}
     			if(LoopCounter>=1515){round++;delay_short();break;}
 				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter++;
 				if(LoopCounter>=1515){round++;delay_short();break;}
 				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter++;
 				if(LoopCounter>=1515){round++;delay_short();break;}
 				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter++;
 				if(LoopCounter>=1515){round++;delay_short();break;}
 				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter++;
 				if(LoopCounter>=1515){round++;delay_short();break;}
 				break;
     		case 1:
     			//if(Counter2<5)Counter2++;
     			//if(Counter2>=5 )Counter2 = 0;
     			if(Counter2 == 0){
     				if(LoopCounter2>=1240 && LoopCounter2<=1560)LoopCounter2++;
     			}
     			if(LoopCounter2>=1560){round++;delay_short();break;}
     			if(LoopCounter2>=1240 && LoopCounter2<=1565)LoopCounter2++;
     			if(LoopCounter2>=1560){round++;delay_short();break;}
     			if(LoopCounter2>=1240 && LoopCounter2<=1565)LoopCounter2++;
     			if(LoopCounter2>=1560){round++;delay_short();break;}
       			if(LoopCounter2>=1240 && LoopCounter2<=1565)LoopCounter2++;
         		if(LoopCounter2>=1560){round++;delay_short();break;}
           		if(LoopCounter2>=1240 && LoopCounter2<=1565)LoopCounter2++;
            	if(LoopCounter2>=1560){round++;delay_short();break;}
     			break;
     		case 2:
     			//if(Counter<5)Counter++;
     			//if(Counter>=5 )Counter = 0;
     			if(Counter == 0){
     				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter--;
     			}
     			if(LoopCounter<=1206){round++;delay_short();break;}
 				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter--;
     			if(LoopCounter<=1206){round++;delay_short();break;}
 				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter--;
     			if(LoopCounter<=1206){round++;delay_short();break;}
     			if(LoopCounter<=1206){round++;delay_short();break;}
 				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter--;
     			if(LoopCounter<=1206){round++;delay_short();break;}
 				if(LoopCounter>=1206 && LoopCounter<=1515)LoopCounter--;
     			break;
     		case 3:
     			//if(Counter2<5)Counter2++;
     			//if(Counter2>=5 )Counter2 = 0;
     			if(Counter2 == 0){
     				if(LoopCounter2>=1240 && LoopCounter2<=1560)LoopCounter2--;
     			}
     			if(LoopCounter2<=1240){round = 0;delay_short();break;}
     			if(LoopCounter2>=1240 && LoopCounter2<=1565)LoopCounter2--;
     			if(LoopCounter2<=1240){round = 0;delay_short();break;}
     			if(LoopCounter2>=1240 && LoopCounter2<=1565)LoopCounter2--;
     			if(LoopCounter2<=1240){round = 0;delay_short();break;}
     			if(LoopCounter2>=1240 && LoopCounter2<=1565)LoopCounter2--;
     			if(LoopCounter2<=1240){round = 0;delay_short();break;}
     			if(LoopCounter2>=1240 && LoopCounter2<=1565)LoopCounter2--;
     			if(LoopCounter2<=1240){round = 0;delay_short();break;}
     			break;
     		}
     		break;

     	case 2://a4

     		//xeg1 = sqrt(1*1+abs((r1x - x0))*abs((r1x - x0))*0.00467*0.00467 + abs((r1y - y0))*abs((r1y - y0))*0.00467*0.00467);

     		/*
     		//法2
     		//回到基准点x0y0
     		//识别黑框，比较数据如果上一次的黑框和这次的一样/非常接近，用这个数据
     		//分5步实现控制舵机，不要点的坐标

     		//回到基准点x0y0
     		LoopCounter=1360;
     		LoopCounter2=1500;
     		Update_ServoUS(kFTM_Chnl_0, LoopCounter);
     		Update_ServoUS(kFTM_Chnl_1, LoopCounter2);
     		delay();

     		//识别黑框，比较数据如果上一次的黑框和这次的一样/非常接近，用这个数据
     		if(){}
     		else{last}

     		//
     		if(lastb_rx ==  b_rx && lastb_ry == b_ry){
     			Round++;
     			if(Round == 10){
     				Round = 0;
     				x0 = b_rx;
     				y0 = b_ry;
     				Follow_A4();


     			}


     		}
			lastb_rx = b_rx;
			lastb_ry = b_ry;
     		break;
     		*/
     		Update_ServoUS(kFTM_Chnl_0, LoopCounter);
     		Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

     		sscan++;
     		if(sscan == 10){
    	    	sscanf(CmdDataBuffer1, "#r1x:%d y:%d r2x:%d y:%d r3x:%d y:%d r0x:%d y:%d r1x:%d y:%d r2x:%d y:%d r3x:%d y:%d r0x:%d y:%d b_rx:%d y:%dEND",
    	    	           &nr1x, &nr1y, &nr2x, &nr2y, &nr3x, &nr3y, &nr0x, &nr0y, &nr1x, &nr1y, &nr2x, &nr2y, &nr3x, &nr3y, &nr0x, &nr0y, &nb_rx, &nb_ry);

     		}



     		if(sscan==11){
     			r1x = nr1x;
     			r1y = nr1y;
     			r2x = nr2x;
     			r2y = nr2y;
     			r3x = nr3x;
     			r3y = nr3y;
     			r0x = nr0x;
     			b_rx = nb_rx;
     			b_ry = nb_ry;


     			//x0-r0
     			xeg1 = sqrt(1+(x0 - x0)*(x0 - x0)*0.00467*0.00467 + (y0 - y0)*(y0 - y0)*0.00467*0.00467);//a1
				xeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (y0 - y0)*(y0 - y0)*0.00467*0.00467);//c
				yeg1 = xeg2;
				yeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//a2
				//算角
				deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2 - (x0 - r0x)*(x0 - r0x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
				delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2 - (y0 - r0y)*(y0 - r0y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
				deltx = (int)(abs(deltx));
				delty = (int)(abs(delty));
				if(deltx<400 && delty <400){
					tmx = deltx/5;
					tmy = delty/5;
					for(tmi = 0; tmi<5;tmi++){
						if(r0x<b_rx){
							LoopCounter2+=tmx;
							LoopCounter2-=3;
						}
						else{
							LoopCounter2-=tmx;
							LoopCounter2-=3;
						}
						if(r0y>b_ry){
							LoopCounter+=tmy;
							LoopCounter+=7;
						}
						else{
							LoopCounter-=tmy;
							LoopCounter+=7;
						}

						Update_ServoUS(kFTM_Chnl_0, LoopCounter);
						Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

						delay();
						delay();
						delay();
					}

				}
				Round++;
				delay();
				delay();
				delay();
				//r0-r3


				//算边(实际最后是三条边)

				xeg1 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//a1
				xeg2 = sqrt(1*1+(r3x - x0)*(r3x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//c
				yeg1 = xeg2;
				yeg2 = sqrt(1*1+(r3x - x0)*(r3x - x0)*0.00467*0.00467 + (r3y - y0)*(r3y - y0)*0.00467*0.00467);//a2
				//算角
				deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (r3x - r0x)*(r3x - r0x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
				delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (r3y - r0y)*(r3y - r0y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
				deltx = (int)(abs(deltx));
				delty = (int)(abs(delty));
				if(deltx<400 && delty <400){
					tmx = deltx/5;
					tmy = delty/5;
					for(tmi = 0; tmi<5;tmi++){
						if(r3x<r0x){
							LoopCounter2+=tmx;
							LoopCounter2-=7;
						}
						else{
							LoopCounter2-=tmx;
							LoopCounter2-=7;
						}
						if(r3y>r0y){
							LoopCounter+=tmy;
						}
						else{
							LoopCounter-=tmy;
						}

						Update_ServoUS(kFTM_Chnl_0, LoopCounter);
						Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

						delay();
						delay();
						delay();
					}
					delay();
					delay();
					delay();
					Round++;
				}


				//r3-r2


				//算边(实际最后是三条边)

				xeg1 = sqrt(1*1+(r3x - x0)*(r3x - x0)*0.00467*0.00467 + (r3y - y0)*(r3y - y0)*0.00467*0.00467);//a1
				xeg2 = sqrt(1*1+(r2x - x0)*(r2x - x0)*0.00467*0.00467 + (r3y - y0)*(r3y - y0)*0.00467*0.00467);//c
				yeg1 = xeg2;
				yeg2 = sqrt(1*1+(r2x - x0)*(r2x - x0)*0.00467*0.00467 + (r2y - y0)*(r2y - y0)*0.00467*0.00467);//a2
				//算角
				deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (r2x - r3x)*(r2x - r3x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
				delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (r2y - r3y)*(r2y - r3y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
				deltx = (int)(abs(deltx));
				delty = (int)(abs(delty));
				if(deltx<400 && delty <400){
					tmx = deltx/5;
					tmy = delty/5;
					for(tmi = 0; tmi<5;tmi++){
						if(r2x<r3x){
							LoopCounter2+=tmx;
						}
						else{
							LoopCounter2-=tmx;
						}
						if(r2y>r3y){
							LoopCounter+=tmy;
							LoopCounter+=1;
						}
						else{
							LoopCounter-=tmy;
							LoopCounter+=1;
						}

						Update_ServoUS(kFTM_Chnl_0, LoopCounter);
						Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

						delay();
						delay();
						delay();
					}

				}
				delay();
				delay();
				delay();
				Round++;
				//r2-r1


				//算边(实际最后是三条边)

				xeg1 = sqrt(1*1+(r2x - x0)*(r2x - x0)*0.00467*0.00467 + (r2y - y0)*(r2y - y0)*0.00467*0.00467);//a1
				xeg2 = sqrt(1*1+(r1x - x0)*(r1x - x0)*0.00467*0.00467 + (r2y - y0)*(r2y - y0)*0.00467*0.00467);//c
				yeg1 = xeg2;
				yeg2 = sqrt(1*1+(r1x - x0)*(r1x - x0)*0.00467*0.00467 + (r1y - y0)*(r1y - y0)*0.00467*0.00467);//a2
				//算角
				deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (r1x - r2x)*(r1x - r2x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
				delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (r1y - r2y)*(r1y - r2y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
				deltx = (int)(abs(deltx));
				delty = (int)(abs(delty));
				if(deltx<400 && delty <400){
					tmx = deltx/5;
					tmy = delty/5;
					for(tmi = 0; tmi<5;tmi++){
						if(r1x<r2x){
							LoopCounter2+=tmx;
							LoopCounter2+=8;
						}
						else{
							LoopCounter2-=tmx;
							LoopCounter2+=8;
						}
						if(r1y>r2y){
							LoopCounter+=tmy;
						}
						else{
							LoopCounter-=tmy;
						}

						Update_ServoUS(kFTM_Chnl_0, LoopCounter);
						Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

						delay();
						delay();
						delay();
					}

				}
				delay();
				delay();
				delay();
				Round++;

				//r1-r0

				//算边(实际最后是三条边)

				xeg1 = sqrt(1*1+(r1x - x0)*(r1x - x0)*0.00467*0.00467 + (r1y - y0)*(r1y - y0)*0.00467*0.00467);//a1
				xeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r1y - y0)*(r1y - y0)*0.00467*0.00467);//c
				yeg1 = xeg2;
				yeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//a2
				//算角
				deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (r0x - r1x)*(r0x - r1x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
				delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (r0y - r1y)*(r0y - r1y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
				deltx = (int)(abs(deltx));
				delty = (int)(abs(delty));
				if(deltx<400 && delty <400){
					tmx = deltx/5;
					tmy = delty/5;
					for(tmi = 0; tmi<5;tmi++){
						if(r0x<r1x){
							LoopCounter2+=tmx;
						}
						else{
							LoopCounter2-=tmx;

						}
						if(r0y>r1y){
							LoopCounter+=tmy;
							LoopCounter+=1;
						}
						else{
							LoopCounter-=tmy;
							LoopCounter+=1;
						}

						Update_ServoUS(kFTM_Chnl_0, LoopCounter);
						Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

						delay();
						delay();
						delay();
					}

				}
				delay();
				delay();
				delay();
				Round++;


				delay();
		}
     			     		//算边(实际最后是三条边)










     		//法1
/*
     		switch(round){
     		case 0:

         		//red-r0
     			Dx = abs(b_rx - r0x);
     			Dy = abs(b_ry - r0y);
         		//算边(实际最后是三条边)

         		xeg1 = sqrt(1+(b_rx - x0)*(b_rx - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//a1
    			xeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//c
    			yeg1 = xeg2;
    			yeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//a2
         		//算角
         		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2 - (b_rx - r0x)*(b_rx - r0x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
    			delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2 - (b_ry - r0y)*(b_ry - r0y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
    			deltx = (int)(abs(deltx));
    			delty = (int)(abs(delty));
    			if((lastb_rx != b_rx || lastb_ry != b_ry) && deltx<400 && delty <400){
					if(r0x<b_rx){
	         			LoopCounter2+=deltx;
	         		}
	         		else{
	         			LoopCounter2-=deltx;
	         		}
	         		if(r0y>b_ry){
	         			LoopCounter+=delty;
	         		}
	         		else{
	         			LoopCounter-=delty;
	         		}
    			}
    			lastb_rx = b_rx;
    			lastb_ry = b_ry;
    			if(r0x<b_rx+3 && r0x>b_rx-3 && r0y>b_ry+3 && r0y>b_ry-3)round++;
     			break;
     		case 1:
         		//red-r1

     			Dx = abs(r0x - r1x);
     			Dy = abs(r0y - r1y);
         		//算边(实际最后是三条边)

         		xeg1 = sqrt(1*1+(b_rx - x0)*(b_rx - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//a1
    			xeg2 = sqrt(1*1+(r1x - x0)*(r1x - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//c
    			yeg1 = xeg2;
    			yeg2 = sqrt(1*1+(r1x - x0)*(r1x - x0)*0.00467*0.00467 + (r1y - y0)*(r1y - y0)*0.00467*0.00467);//a2
         		//算角
         		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (b_rx - r1x)*(b_rx - r1x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
    			delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (b_ry - r1y)*(b_ry - r1y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
    			deltx = (int)(abs(deltx));
    			delty = (int)(abs(delty));
    			if((lastb_rx != b_rx || lastb_ry != b_ry) && deltx<400 && delty <400){
					if(r1x<b_rx){
	         			LoopCounter2+=deltx;
	         		}
	         		else{
	         			LoopCounter2-=deltx;
	         		}
	         		if(r1y>b_ry){
	         			LoopCounter+=delty;
	         		}
	         		else{
	         			LoopCounter-=delty;
	         		}
	         		//round++;
    			}
    			lastb_rx = b_rx;
    			lastb_ry = b_ry;
    			if(r1x<b_rx+3 && r1x>b_rx-3 && r1y<b_ry+3 && r1y>b_ry-3)round++;
         		break;
     		case 2:
         		//red-r2

     			Dx = abs(r1x - r2x);
     			Dy = abs(r1y - r2y);
         		//算边(实际最后是三条边)

         		xeg1 = sqrt(1*1+(b_rx - x0)*(b_rx - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//a1
    			xeg2 =	sqrt(1*1+(r2x - x0)*(r2x - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//c
    			yeg1 = xeg2;
    			yeg2 = sqrt(1*1+(r2x - x0)*(r2x - x0)*0.00467*0.00467 + (r2y - y0)*(r2y - y0)*0.00467*0.00467);//a2
         		//算角
         		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (b_rx - r2x)*(b_rx - r2x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
    			delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (b_ry - r2y)*(b_ry - r2y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
    			deltx = (int)(abs(deltx));
    			delty = (int)(abs(delty));
    			if((lastb_rx != b_rx || lastb_ry != b_ry) && deltx<400 && delty <400){
					if(r2x<b_rx){
	         			LoopCounter2+=deltx;
	         		}
	         		else{
	         			LoopCounter2-=deltx;
	         		}
	         		if(r2y>b_ry){
	         			LoopCounter+=delty;
	         		}
	         		else{
	         			LoopCounter-=delty;
	         		}
	         		//round++;
    			}
    			lastb_rx = b_rx;
    			lastb_ry = b_ry;
    			if(r2x<b_rx+3 && r2x>b_rx-3 && r2y<b_ry+3 && r2y>b_ry-3)round++;
         		break;
     		case 3:
         		//red-r3

     			Dx = abs(r2x - r3x);
     			Dy = abs(r2y - r3y);
         		//算边(实际最后是三条边)

         		xeg1 = sqrt(1*1+(b_rx - x0)*(b_rx - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//a1
    			xeg2 = sqrt(1*1+(r3x - x0)*(r3x - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//c
    			yeg1 = xeg2;
    			yeg2 = sqrt(1*1+(r3x - x0)*(r3x - x0)*0.00467*0.00467 + (r3y - y0)*(r3y - y0)*0.00467*0.00467);//a2
         		//算角
         		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (b_rx - r3x)*(b_rx - r3x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
    			delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (b_ry - r3y)*(b_ry - r3y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
    			deltx = (int)(abs(deltx));
    			delty = (int)(abs(delty));
    			if((lastb_rx != b_rx || lastb_ry != b_ry) && deltx<400 && delty <400){
					if(r3x<b_rx){
	         			LoopCounter2+=deltx;
	         		}
	         		else{
	         			LoopCounter2-=deltx;
	         		}
	         		if(r3y>b_ry){
	         			LoopCounter+=delty;
	         		}
	         		else{
	         			LoopCounter-=delty;
	         		}
	         		//round++;
    			}
    			lastb_rx = b_rx;
    			lastb_ry = b_ry;
    			if(r3x<b_rx+3 && r3x>b_rx-3 && r3y<b_ry+3 && r3y>b_ry-3)round++;
     			break;
     		case 4:
         		//red-r0

     			Dx = abs(r3x - r0x);
     			Dy = abs(r3y - r0y);
         		//算边(实际最后是三条边)

         		xeg1 = sqrt(1*1+(b_rx - x0)*(b_rx - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//a1
    			xeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (b_ry - y0)*(b_ry - y0)*0.00467*0.00467);//c
    			yeg1 = xeg2;
    			yeg2 = sqrt(1*1+(r0x - x0)*(r0x - x0)*0.00467*0.00467 + (r0y - y0)*(r0y - y0)*0.00467*0.00467);//a2
         		//算角
         		deltx = 2000*(acos((xeg1*xeg1 + xeg2*xeg2- (b_rx - r0x)*(b_rx - r0x)*0.00467*0.00467)/(2*xeg1*xeg2)))/3.14;
    			delty = 2000*(acos((yeg1*yeg1 + yeg2*yeg2- (b_ry - r0y)*(b_ry - r0y)*0.00467*0.00467)/(2*yeg1*yeg2)))/3.14;
    			deltx = (int)(abs(deltx));
    			delty = (int)(abs(delty));
    			if((lastb_rx != b_rx || lastb_ry != b_ry) && deltx<400 && delty <400){
					if(r0x<b_rx){
	         			LoopCounter2+=deltx;
	         		}
	         		else{
	         			LoopCounter2-=deltx;
	         		}
	         		if(r0y>b_ry){
	         			LoopCounter+=delty;
	         		}
	         		else{
	         			LoopCounter-=delty;
	         		}
	         		//round=100;
    			}
    			lastb_rx = b_rx;
    			lastb_ry = b_ry;
    			if(r0x<b_rx+3 && r0x>b_rx-3 && r0y<b_ry+3 && r0y>b_ry-3)round = 100;
         		break;
     		}*/
     		break;



     	case 3:
     		switch(round){
     		case 0:
 			//if(Counter<5)Counter++;
 			//if(Counter>=5 )Counter = 0;
 			if(Counter == 0){
 				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter++;
 			}
 			if(LoopCounter>=1435){round++;delay_short();break;}
				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter++;
				if(LoopCounter>=1435){round++;delay_short();break;}
				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter++;
				if(LoopCounter>=1435){round++;delay_short();break;}
				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter++;
				if(LoopCounter>=1435){round++;delay_short();break;}
				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter++;
				if(LoopCounter>=1435){round++;delay_short();break;}
				break;
     		case 1:
 			//if(Counter2<5)Counter2++;
 			//if(Counter2>=5 )Counter2 = 0;
 			if(Counter2 == 0){
 				if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2++;
 			}
 			if(LoopCounter2>=1507){round++;delay_short();break;}
 			if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2++;
 			if(LoopCounter2>=1507){round++;delay_short();break;}
 			if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2++;
 			if(LoopCounter2>=1507){round++;delay_short();break;}
   			if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2++;
     		if(LoopCounter2>=1507){round++;delay_short();break;}
       		if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2++;
        	if(LoopCounter2>=1507){round++;delay_short();break;}
 			break;
     		case 2:
 			//if(Counter<5)Counter++;
 			//if(Counter>=5 )Counter = 0;
 			if(Counter == 0){
 				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter--;
 			}
 			if(LoopCounter<=1285){round++;delay_short();break;}
				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter--;
 			if(LoopCounter<=1285){round++;delay_short();break;}
				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter--;
 			if(LoopCounter<=1285){round++;delay_short();break;}
 			if(LoopCounter<=1285){round++;delay_short();break;}
				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter--;
 			if(LoopCounter<=1285){round++;delay_short();break;}
				if(LoopCounter>=1285 && LoopCounter<=1435)LoopCounter--;
 			break;
     		case 3:
 			//if(Counter2<5)Counter2++;
 			//if(Counter2>=5 )Counter2 = 0;
 			if(Counter2 == 0){
 				if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2--;
 			}
 			if(LoopCounter2<=1309){round = 0;delay_short();break;}
 			if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2--;
 			if(LoopCounter2<=1309){round = 0;delay_short();break;}
 			if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2--;
 			if(LoopCounter2<=1309){round = 0;delay_short();break;}
 			if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2--;
 			if(LoopCounter2<=1309){round = 0;delay_short();break;}
 			if(LoopCounter2>=1309 && LoopCounter2<=1507)LoopCounter2--;
 			if(LoopCounter2<=1309){round = 0;delay_short();break;}
 			break;
     		}
 		break;


     	}
     	Update_ServoUS(kFTM_Chnl_0, LoopCounter);
  	    Update_ServoUS(kFTM_Chnl_1, LoopCounter2);

     	delay_short();


 	   /*
 	    *     	int i = 0;
     	for(i = 0;i < 1000;i++) LoopCounter++;
     	delay_short();
     	for(i = 0;i < 1000;i++) LoopCounter2++;
     	delay_short();
     	for(i = 0;i < 1000;i++) LoopCounter--;
     	delay_short();
     	for(i = 0;i < 1000;i++) LoopCounter2--;
     	if(servo_update_flag)
 		{
  	       Update_ServoUS(kFTM_Chnl_0, LoopCounter);
  	       Update_ServoUS(kFTM_Chnl_1, LoopCounter);
  	       FTM_SetSoftwareTrigger(FTM3_PERIPHERAL, true);
  	       servo_update_flag = false;
 		}
     //	LoopCounter = (int)((angleXZ+1.57) / 3.14 * 2000);//转角换算为0-2000舵机随动计数
     	if (LoopCounter > 2500)
     		LoopCounter = 0;
     //	LoopCounter2 = (int)((angleYZ+1.57) / 3.14 * 2000);//转角换算为0-2000舵机随动计数
     	if (LoopCounter2 > 2500)
     		LoopCounter2 = 0;

 		*/

     	if (ElapsedSeconds != LastSecond)
     	{
     		LED1_TOGGLE();	LED2_TOGGLE();
     		LastSecond = ElapsedSeconds;

     		RTC_GetDatetime(RTC, &appDateTime);
     		sprintf(OLEDLine1, "Count: %d ", LoopCounter);
     		sprintf(OLEDLine2, "Count2: %d ", LoopCounter2);//OLED显示舵机转动计数
     		sprintf(OLEDLine3, "round: %d ", round);
     		sprintf(OLEDLine4, "xeg1: %2f ", xeg1);
     		OLED_P8x16Str(0, 0, (uint8_t *)OLEDLine1);
     		OLED_P8x16Str(0, 2, (uint8_t *)OLEDLine2);
     		OLED_P8x16Str(0, 4, (uint8_t *)OLEDLine3);
     		OLED_P8x16Str(0, 6, (uint8_t *)OLEDLine4);

     	}
         CDK66_Analog_Input(&AnalogIn);		// Update all analog inputs


         //MemsX1 = ((float)MemsX-2048.0)/101.3234;
         //MemsY1 = ((float)MemsY-2048.0)/101.3234;
         //MemsZ1 = ((float)MemsZ-2048.0)/101.3234;

     }
    return 0 ;
}
