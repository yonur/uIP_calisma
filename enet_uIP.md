//hello_world
/**
 * \addtogroup helloworld
 * @{
 */

/**
 * \file
 *         An example of how to write uIP applications
 *         with protosockets.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

/*
 * This is a short example of how to write uIP applications using
 * protosockets.
 */

/*
 * We define the application state (struct hello_world_state) in the
 * hello-world.h file, so we need to include it here. We also include
 * uip.h (since this cannot be included in hello-world.h) and
 * <string.h>, since we use the memcpy() function in the code.
 */
#include "hello-world.h"
#include "drivers/pinout.h"
#include "uip.h"
#include <string.h>

uint32_t time = 0;
uint32_t pinVal=0;
/*
 * Declaration of the protosocket function that handles the connection
 * (defined at the end of the code).
 */
static int handle_connection(struct hello_world_state *s);
/*---------------------------------------------------------------------------*/
/*
 * The initialization function. We must explicitly call this function
 * from the system initialization code, some time after uip_init() is
 * called.
 */
void
hello_world_init(void)
{
  /* We start to listen for connections on TCP port 1000. */
  uip_listen(HTONS(6340));
}
/*---------------------------------------------------------------------------*/
/*
 * In hello-world.h we have defined the UIP_APPCALL macro to
 * hello_world_appcall so that this funcion is uIP's application
 * function. This function is called whenever an uIP event occurs
 * (e.g. when a new connection is established, new data arrives, sent
 * data is acknowledged, data needs to be retransmitted, etc.).
 */
void
hello_world_appcall(void)
{
  /*
   * The uip_conn structure has a field called "appstate" that holds
   * the application state of the connection. We make a pointer to
   * this to access it easier.
   */
  struct hello_world_state *s = &(uip_conn->appstate);

  /*
   * If a new connection was just established, we should initialize
   * the protosocket in our applications' state structure.
   */
  if(uip_connected()) {
    PSOCK_INIT(&s->p, s->inputbuffer, sizeof(s->inputbuffer));
  }

  /*
   * Finally, we run the protosocket function that actually handles
   * the communication. We pass it a pointer to the application state
   * of the current connection.
   */
  handle_connection(s);
}
/*---------------------------------------------------------------------------*/
/*
 * This is the protosocket function that handles the communication. A
 * protosocket function must always return an int, but must never
 * explicitly return - all return statements are hidden in the PSOCK
 * macros.
 */
static int
handle_connection(struct hello_world_state *s)
{
  PSOCK_BEGIN(&s->p);

  strncpy(s->out, "abidin", sizeof(s->out));
  p_flag = 1;

  while(p_flag)
  {
	  PSOCK_SEND_STR(&s->p, "Hello. What is your name?\n");
	  PSOCK_READTO(&s->p, '\n');
	  strncpy(s->name, s->inputbuffer, sizeof(s->name));
	  if((s->name[0] == 'a') && (s->name[6] != 'q'))
	  {
		  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 1);
		  PSOCK_SEND_STR(&s->p, "Led is lighting\n");

		  if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3))
		  {
			  PSOCK_SEND_STR(&s->p, "Read PE3\n");
		  }
		  else
		  {
			  PSOCK_SEND_STR(&s->p, "No read PE3\n");
		  }

		  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
		  PSOCK_SEND_STR(&s->p, "Led is no\n");
		  s->out[3] = 'c';
		  PSOCK_SEND_STR(&s->p, s->out);
		  PSOCK_SEND_STR(&s->p, "\n");
		  TimerActivate();

		  while(t_ui32Flags)
		  {
			  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2);
			  if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3))
			  {
				  s->out[2] = 'x';
				  PSOCK_SEND_STR(&s->p, s->out);

				  PSOCK_SEND_STR(&s->p, " ");
			  }
			//  time++;
		  }

		  TimerDeActivate();


		  PSOCK_SEND_STR(&s->p, "\n");
		  PSOCK_SEND_STR(&s->p, "\n");

		  t_ui32Flags = 1;
		  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
		  TimerActivate();

		  while(t_ui32Flags)
		  {
			  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 , 0);
			  s->out[1] = 'w';
			  PSOCK_SEND_STR(&s->p, s->out);
			  PSOCK_SEND_STR(&s->p, " ");

			 // time++;
		  }

		  TimerDeActivate();

		  PSOCK_SEND_STR(&s->p, "\n");
		  PSOCK_SEND_STR(&s->p, "\n");

		  t_ui32Flags = 1;
		  //2 sn bekleme yapacaz burada.
		  TimerActivate();

		  while(t_ui32Flags)
		  {
			  if(s->name[3] == 'w')
				  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 ,1);

			  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 ,2);
			  s->out[5] = 'z';
			  PSOCK_SEND_STR(&s->p, s->out);
			  PSOCK_SEND_STR(&s->p, " ");

			//  time++;
		  }

		  TimerDeActivate();

		  PSOCK_SEND_STR(&s->p, "\n");
		  PSOCK_SEND_STR(&s->p, "\n");

		  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 ,0);
		  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 ,0);
		  t_ui32Flags = 1;

		  //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4);
	  }
	  else if(s->name[6] == 'q')
	  {
		  p_flag = 0;
		  PSOCK_SEND_STR(&s->p, "Good bye and fuck you\n");
	  }
	  else
	  {
		  PSOCK_SEND_STR(&s->p, "Please enter a sentence that first letter is 'a'\n");
		  //PSOCK_READTO(&s->p, '\n');
	  }
  }

  PSOCK_CLOSE(&s->p);
  
  PSOCK_END(&s->p);
}
/*---------------------------------------------------------------------------*/
//hello_world.h
/**
 * \addtogroup apps
 * @{
 */

/**
 * \defgroup helloworld Hello, world
 * @{
 *
 * A small example showing how to write applications with
 * \ref psock "protosockets".
 */

/**
 * \file
 *         Header file for an example of how to write uIP applications
 *         with protosockets.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#ifndef __HELLO_WORLD_H__
#define __HELLO_WORLD_H__

/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */

#include <stdbool.h>

#include "uipopt.h"

#include "psock.h"

#include "drivers/pinout.h"


#include "inc/hw_emac.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"

extern u8_t p_flag;
/* Next, we define the uip_tcp_appstate_t datatype. This is the state
   of our application, and the memory required for this state is
   allocated together with each TCP connection. One application state
   for each TCP connection. */
typedef struct hello_world_state {
  struct psock p;
  char inputbuffer[10];
  char name[40];
  char out[20];
} uip_tcp_appstate_t;

/* Finally we define the application function to be called by uIP. */
void hello_world_appcall(void);
#ifndef UIP_APPCALL
#define UIP_APPCALL hello_world_appcall
#endif /* UIP_APPCALL */

void hello_world_init(void);

#endif /* __HELLO_WORLD_H__ */
/** @} */
/** @} */
//pinout.c
//*****************************************************************************
//
// pinout.c - Function to configure the device pins on the EK-TM4C1294XL.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************


#include "drivers/pinout.h"

//*****************************************************************************
//
//! \addtogroup pinout_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Configures the device pins for the standard usages on the EK-TM4C1294XL.
//!
//! \param bEthernet is a boolean used to determine function of Ethernet pins.
//! If true Ethernet pins are  configured as Ethernet LEDs.  If false GPIO are
//! available for application use.
//! \param bUSB is a boolean used to determine function of USB pins. If true USB
//! pins are configured for USB use.  If false then USB pins are available for
//! application use as GPIO.
//!
//! This function enables the GPIO modules and configures the device pins for
//! the default, standard usages on the EK-TM4C1294XL.  Applications that
//! require alternate configurations of the device pins can either not call
//! this function and take full responsibility for configuring all the device
//! pins, or can reconfigure the required device pins after calling this
//! function.
//!
//! \return None.
//
//*****************************************************************************

uint32_t t_ui32Flags = 1;

void
PinoutSet(bool bEthernet, bool bUSB)
{
    //
    // Enable all the GPIO peripherals.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

    //
    // PA0-1 are used for UART0.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // PB0-1/PD6/PL6-7 are used for USB.
    // PQ4 can be used as a power fault detect on this board but it is not
    // the hardware peripheral power fault input pin.
    //
    if(bUSB)
    {
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0xff;
        ROM_GPIOPinConfigure(GPIO_PD6_USB0EPEN);
        ROM_GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        ROM_GPIOPinTypeUSBDigital(GPIO_PORTD_BASE, GPIO_PIN_6);
        ROM_GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);
        ROM_GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_4);
    }
    else
    {
        //
        // Keep the default config for most pins used by USB.
        // Add a pull down to PD6 to turn off the TPS2052 switch
        //
        ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6);
        MAP_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA,
                             GPIO_PIN_TYPE_STD_WPD);

    }

    //
    // PF0/PF4 are used for Ethernet LEDs.
    //
    if(bEthernet)
    {
        //
        // this app wants to configure for ethernet LED function.
        //
        ROM_GPIOPinConfigure(GPIO_PF0_EN0LED0);
        ROM_GPIOPinConfigure(GPIO_PF4_EN0LED1);

        GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    }
    else
    {

        //
        // This app does not want Ethernet LED function so configure as
        // standard outputs for LED driving.
        //
        ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

        //
        // Default the LEDs to OFF.
        //
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);
        MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4,
                             GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);


    }

    //
    // PJ0 and J1 are used for user buttons
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);

    //
    // PN0 and PN1 are used for USER LEDs.
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    MAP_GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                             GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    //
    // Default the LEDs to OFF.
    //
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
}

void PuttyInit(void)
{
	//PE3, PE4, and PE5 are for inputs.
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
	MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, 0);

	//
	// PE0, PE1 and PE2 are used for USER LEDs.
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
	//MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void TimerInit(void)
{
	//Configure timer clock source.
    //

	// Enable the peripherals used by this example.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//
	// Wait for the Timer0 module to be ready.
	//
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
	{
	}

	//
	// Enable processor interrupts.
	//
	ROM_IntMasterEnable();

	//
	// Configure the two 32-bit periodic timers.
	//
	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, (2 * ui32SysClock));

	//
	// Setup the interrupts for the timer timeouts.
	//
	ROM_IntEnable(INT_TIMER0A);
	ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Enable the timers.
	//
	//ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

void TimerActivate(void)
{
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

void TimerDeActivate(void)
{
	ROM_TimerDisable(TIMER0_BASE, TIMER_A);
}

//*****************************************************************************
//
//! This function writes a state to the LED bank.
//!
//! \param ui32LEDMask is a bit mask for which GPIO should be changed by this
//! call.
//! \param ui32LEDValue is the new value to be applied to the LEDs after the
//! ui32LEDMask is applied.
//!
//! The first parameter acts as a mask.  Only bits in the mask that are set
//! will correspond to LEDs that may change.  LEDs with a mask that is not set
//! will not change. This works the same as GPIOPinWrite. After applying the
//! mask the setting for each unmasked LED is written to the corresponding
//! LED port pin via GPIOPinWrite.
//!
//! \return None.
//
//*****************************************************************************
void
LEDWrite(uint32_t ui32LEDMask, uint32_t ui32LEDValue)
{

    //
    // Check the mask and set or clear the LED as directed.
    //
    if(ui32LEDMask & CLP_D1)
    {
        if(ui32LEDValue & CLP_D1)
        {
            GPIOPinWrite(CLP_D1_PORT, CLP_D1_PIN, CLP_D1_PIN);
        }
        else
        {
            GPIOPinWrite(CLP_D1_PORT, CLP_D1_PIN, 0);
        }
    }

    if(ui32LEDMask & CLP_D2)
    {
        if(ui32LEDValue & CLP_D2)
        {
            GPIOPinWrite(CLP_D2_PORT, CLP_D2_PIN, CLP_D2_PIN);
        }
        else
        {
            GPIOPinWrite(CLP_D2_PORT, CLP_D2_PIN, 0);
        }
    }

    if(ui32LEDMask & CLP_D3)
    {
        if(ui32LEDValue & CLP_D3)
        {
            GPIOPinWrite(CLP_D3_PORT, CLP_D3_PIN, CLP_D3_PIN);
        }
        else
        {
            GPIOPinWrite(CLP_D3_PORT, CLP_D3_PIN, 0);
        }
    }

    if(ui32LEDMask & CLP_D4)
    {
        if(ui32LEDValue & CLP_D4)
        {
            GPIOPinWrite(CLP_D4_PORT, CLP_D4_PIN, CLP_D4_PIN);
        }
        else
        {
            GPIOPinWrite(CLP_D4_PORT, CLP_D4_PIN, 0);
        }
    }
}

//*****************************************************************************
//
//! This function reads the state to the LED bank.
//!
//! \param pui32LEDValue is a pointer to where the LED value will be stored.
//!
//! This function reads the state of the CLP LEDs and stores that state
//! information into the variable pointed to by pui32LEDValue.
//!
//! \return None.
//
//*****************************************************************************
void LEDRead(uint32_t *pui32LEDValue)
{
    *pui32LEDValue = 0;

    //
    // Read the pin state and set the variable bit if needed.
    //
    if(GPIOPinRead(CLP_D4_PORT, CLP_D4_PIN))
    {
        *pui32LEDValue |= CLP_D4;
    }

    //
    // Read the pin state and set the variable bit if needed.
    //
    if(GPIOPinRead(CLP_D3_PORT, CLP_D3_PIN))
    {
        *pui32LEDValue |= CLP_D3;
    }

    //
    // Read the pin state and set the variable bit if needed.
    //
    if(GPIOPinRead(CLP_D2_PORT, CLP_D2_PIN))
    {
        *pui32LEDValue |= CLP_D2;
    }

    //
    // Read the pin state and set the variable bit if needed.
    //
    if(GPIOPinRead(CLP_D1_PORT, CLP_D1_PIN))
    {
        *pui32LEDValue |= CLP_D1;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
//pinout.h
//*****************************************************************************
//
// pinout.h - Prototype for the function to configure the device pins on the
//            EK-TM4C1294XL.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#ifndef __DRIVERS_PINOUT_H__
#define __DRIVERS_PINOUT_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Define Board LED's
//
//*****************************************************************************
#define CLP_D1              1
#define CLP_D2              2
#define CLP_D3              4
#define CLP_D4              8

#define CLP_D1_PORT         GPIO_PORTN_BASE
#define CLP_D1_PIN          GPIO_PIN_1

#define CLP_D2_PORT         GPIO_PORTN_BASE
#define CLP_D2_PIN          GPIO_PIN_0

#define CLP_D3_PORT         GPIO_PORTF_BASE
#define CLP_D3_PIN          GPIO_PIN_4

#define CLP_D4_PORT         GPIO_PORTF_BASE
#define CLP_D4_PIN          GPIO_PIN_0

//Kutuphaneler

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

//*****************************************************************************
//
// Prototypes.
//
//*****************************************************************************
extern void PinoutSet(bool bEthernet, bool bUSB);
extern void LEDWrite(uint32_t ui32LEDMask, uint32_t ui32LEDValue);
extern void LEDRead(uint32_t *pui32LEDValue);

extern void PuttyInit(void);
extern void TimerActivate(void);
extern void TimerDeActivate(void);
//extern void Timer0IntHandler(void);

extern uint32_t ui32SysClock;
extern uint32_t t_ui32Flags;
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __DRIVERS_PINOUT_H__
//enet_uip.c
//*****************************************************************************
//
// enet_uip.c - Sample WebServer Application for Ethernet Demo
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_emac.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/emac.h"
#include "driverlib/flash.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "uip/uip.h"
#include "uip/uip_arp.h"
#include "hello-world/hello-world.h"
#include "dhcpc/dhcpc.h"
#include "drivers/pinout.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Ethernet with uIP (enet_uip)</h1>
//!
//! This example application demonstrates the operation of the Tiva C Series
//! Ethernet controller using the uIP TCP/IP Stack.  DHCP is used to obtain
//! an Ethernet address.  A basic web site is served over the Ethernet port.
//! The web site displays a few lines of text, and a counter that increments
//! each time the page is sent.
//!
//! UART0, connected to the ICDI virtual COM port and running at 115,200,
//! 8-N-1, is used to display messages from this application.
//!
//! For additional details on uIP, refer to the uIP web page at:
//! http://www.sics.se/~adam/uip/
//
//*****************************************************************************

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ               CLOCK_CONF_SECOND
#define SYSTICKMS               (1000 / SYSTICKHZ)
#define SYSTICKUS               (1000000 / SYSTICKHZ)
#define SYSTICKNS               (1000000000 / SYSTICKHZ)

//*****************************************************************************
//
// Macro for accessing the Ethernet header information in the buffer.
//
//*****************************************************************************
u8_t g_pui8UIPBuffer[UIP_BUFSIZE + 2];
u8_t *uip_buf = g_pui8UIPBuffer;

#define BUF                     ((struct uip_eth_hdr *)uip_buf)

//*****************************************************************************
u8_t p_flag = 1;
uint32_t ui32SysClock;

//*****************************************************************************
//
// Ethernet DMA descriptors.
//
// Although uIP uses a single buffer, the MAC hardware needs a minimum of
// 3 receive descriptors to operate.
//
//*****************************************************************************
#define NUM_TX_DESCRIPTORS 3
#define NUM_RX_DESCRIPTORS 3
tEMACDMADescriptor g_psRxDescriptor[NUM_TX_DESCRIPTORS];
tEMACDMADescriptor g_psTxDescriptor[NUM_RX_DESCRIPTORS];

uint32_t g_ui32RxDescIndex;
uint32_t g_ui32TxDescIndex;

//*****************************************************************************
//
// Transmit and receive buffers.
//
//*****************************************************************************
#define RX_BUFFER_SIZE 1536
#define TX_BUFFER_SIZE 1536
uint8_t g_pui8RxBuffer[RX_BUFFER_SIZE];
uint8_t g_pui8TxBuffer[TX_BUFFER_SIZE];

//*****************************************************************************
//
// A set of flags.  The flag bits are defined as follows:
//
//     0 -> An indicator that a SysTick interrupt has occurred.
//     1 -> An RX Packet has been received.
//     2 -> A TX packet DMA transfer is pending.
//     3 -> A RX packet DMA transfer is pending.
//
//*****************************************************************************
#define FLAG_SYSTICK            0
#define FLAG_RXPKT              1
#define FLAG_TXPKT              2
#define FLAG_RXPKTPEND          3
static volatile uint32_t g_ui32Flags;

//*****************************************************************************
//
// A system tick counter, incremented every SYSTICKMS.
//
//*****************************************************************************
volatile uint32_t g_ui32TickCounter = 0;

//*****************************************************************************
//
// Default TCP/IP Settings for this application.
//
// Default to Link Local address ... (169.254.1.0 to 169.254.254.255).  Note:
// This application does not implement the Zeroconf protocol.  No ARP query is
// issued to determine if this static IP address is already in use.
//
// Uncomment the following #define statement to enable STATIC IP
// instead of DHCP.
//
//*****************************************************************************
#define USE_STATIC_IP

#ifndef DEFAULT_IPADDR0
#define DEFAULT_IPADDR0         169
#endif

#ifndef DEFAULT_IPADDR1
#define DEFAULT_IPADDR1         254
#endif

#ifndef DEFAULT_IPADDR2
#define DEFAULT_IPADDR2         19
#endif

#ifndef DEFAULT_IPADDR3
#define DEFAULT_IPADDR3         63
#endif

#ifndef DEFAULT_NETMASK0
#define DEFAULT_NETMASK0        255
#endif

#ifndef DEFAULT_NETMASK1
#define DEFAULT_NETMASK1        255
#endif

#ifndef DEFAULT_NETMASK2
#define DEFAULT_NETMASK2        0
#endif

#ifndef DEFAULT_NETMASK3
#define DEFAULT_NETMASK3        0
#endif

//*****************************************************************************
//
// UIP Timers (in MS)
//
//*****************************************************************************
#define UIP_PERIODIC_TIMER_MS   500
#define UIP_ARP_TIMER_MS        10000

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Increment the system tick count.
    //
    g_ui32TickCounter++;

    //
    // Indicate that a SysTick interrupt has occurred.
    //
    HWREGBITW(&g_ui32Flags, FLAG_SYSTICK) = 1;
}

//*****************************************************************************
//
// When using the timer module in UIP, this function is required to return
// the number of ticks.  Note that the file "clock-arch.h" must be provided
// by the application, and define CLOCK_CONF_SECONDS as the number of ticks
// per second, and must also define the typedef "clock_time_t".
//
//*****************************************************************************
clock_time_t
clock_time(void)
{
    return((clock_time_t)g_ui32TickCounter);
}

//*****************************************************************************
//
// Display a status string on the LCD and also transmit it via the serial port.
//
//*****************************************************************************
void
UpdateStatus(char *pcStatus)
{
    //
    // Dump that status string to the serial port.
    //
    UARTprintf("%s\n", pcStatus);
}

//*****************************************************************************
//
// Display the current IP address on the screen and transmit it via the UART.
//
//*****************************************************************************
void
ShowIPAddress(const uip_ipaddr_t sIPAddr)
{
    char pcBuffer[24];

    usprintf(pcBuffer, "IP: %d.%d.%d.%d", sIPAddr[0] & 0xff,
             sIPAddr[0] >> 8, sIPAddr[1] & 0xff, sIPAddr[1] >> 8);
    UpdateStatus(pcBuffer);
}

//*****************************************************************************
//
// The interrupt handler for the Ethernet interrupt.
//
//*****************************************************************************
void
EthernetIntHandler(void)
{
    uint32_t ui32Temp;

    //
    // Read and Clear the interrupt.
    //
    ui32Temp = MAP_EMACIntStatus(EMAC0_BASE, true);
    MAP_EMACIntClear(EMAC0_BASE, ui32Temp);

    //
    // Check to see if an RX Interrupt has occurred.
    //
    if(ui32Temp & EMAC_INT_RECEIVE)
    {
        //
        // Indicate that a packet has been received.
        //
        HWREGBITW(&g_ui32Flags, FLAG_RXPKT) = 1;
    }

    //
    // Has the DMA finished transferring a packet to the transmitter?
    //
    if(ui32Temp & EMAC_INT_TRANSMIT)
    {
        //
        // Indicate that a packet has been sent.
        //
        HWREGBITW(&g_ui32Flags, FLAG_TXPKT) = 0;
    }
}

//*************************************************
//Asagidaki interrupt handler benim SATB icin kullandigim uygulamada gecikmeyi verecegim Timer'in interrupt handler'i.
//
void Timer0AIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    t_ui32Flags  = 0;

    //
    // Update the interrupt status.
    //
    //ROM_IntMasterDisable();

    //ROM_IntMasterEnable();
}

//*****************************************************************************
//
// Callback for when DHCP client has been configured.
//
//*****************************************************************************

void
dhcpc_configured(const struct dhcpc_state *s)
{
    uip_sethostaddr(&s->ipaddr);
    uip_setnetmask(&s->netmask);
    uip_setdraddr(&s->default_router);
    ShowIPAddress(s->ipaddr);
}

//*****************************************************************************
//
// Read a packet from the DMA receive buffer into the uIP packet buffer.
//
//*****************************************************************************
int32_t
PacketReceive(uint32_t ui32Base, uint8_t *pui8Buf, int32_t i32BufLen)
{
    int_fast32_t i32FrameLen, i32Loop;

    //
    // Check the arguments.
    //
    ASSERT(ui32Base == EMAC0_BASE);
    ASSERT(pui8Buf != 0);
    ASSERT(i32BufLen > 0);

    //
    // By default, we assume we got a bad frame.
    //
    i32FrameLen = 0;

    //
    // Make sure that we own the receive descriptor.
    //
    if(!(g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus & DES0_RX_CTRL_OWN))
    {
        //
        // We own the receive descriptor so check to see if it contains a valid
        // frame.  Look for a descriptor error, indicating that the incoming
        // packet was truncated or, if this is the last frame in a packet,
        // the receive error bit.
        //
        if(!(g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus &
             DES0_RX_STAT_ERR))
        {
            //
            // We have a valid frame so copy the content to the supplied
            // buffer. First check that the "last descriptor" flag is set.  We
            // sized the receive buffer such that it can always hold a valid
            // frame so this flag should never be clear at this point but...
            //
            if(g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus &
               DES0_RX_STAT_LAST_DESC)
            {
                i32FrameLen =
                    ((g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus &
                      DES0_RX_STAT_FRAME_LENGTH_M) >>
                     DES0_RX_STAT_FRAME_LENGTH_S);

                //
                // Sanity check.  This shouldn't be required since we sized the
                // uIP buffer such that it's the same size as the DMA receive
                // buffer but, just in case...
                //
                if(i32FrameLen > i32BufLen)
                {
                    i32FrameLen = i32BufLen;
                }

                //
                // Copy the data from the DMA receive buffer into the provided
                // frame buffer.
                //
                for(i32Loop = 0; i32Loop < i32FrameLen; i32Loop++)
                {
                    pui8Buf[i32Loop] = g_pui8RxBuffer[i32Loop];
                }
            }
        }

        //
        // Move on to the next descriptor in the chain.
        //
        g_ui32RxDescIndex++;
        if(g_ui32RxDescIndex == NUM_RX_DESCRIPTORS)
        {
            g_ui32RxDescIndex = 0;
        }

        //
        // Mark the next descriptor in the ring as available for the receiver
        // to write into.
        //
        g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus = DES0_RX_CTRL_OWN;
    }

    //
    // Return the Frame Length
    //
    return(i32FrameLen);
}

//*****************************************************************************
//
// Transmit a packet from the supplied buffer.
//
//*****************************************************************************
static int32_t
PacketTransmit(uint32_t ui32Base, uint8_t *pui8Buf, int32_t i32BufLen)
{
    int_fast32_t i32Loop;

    //
    // Indicate that a packet is being sent.
    //
    HWREGBITW(&g_ui32Flags, FLAG_TXPKT) = 1;

    //
    // Wait for the previous packet to be transmitted.
    //
    while(g_psTxDescriptor[g_ui32TxDescIndex].ui32CtrlStatus &
          DES0_TX_CTRL_OWN)
    {
        //
        // Spin and waste time.
        //
    }

    //
    // Check that we're not going to overflow the transmit buffer.  This
    // shouldn't be necessary since the uIP buffer is smaller than our DMA
    // transmit buffer but, just in case...
    //
    if(i32BufLen > TX_BUFFER_SIZE)
    {
        i32BufLen = TX_BUFFER_SIZE;
    }

    //
    // Copy the packet data into the transmit buffer.
    //
    for(i32Loop = 0; i32Loop < i32BufLen; i32Loop++)
    {
        g_pui8TxBuffer[i32Loop] = pui8Buf[i32Loop];
    }

    //
    // Move to the next descriptor.
    //
    g_ui32TxDescIndex++;
    if(g_ui32TxDescIndex == NUM_TX_DESCRIPTORS)
    {
        g_ui32TxDescIndex = 0;
    }

    //
    // Fill in the packet size and tell the transmitter to start work.
    //
    g_psTxDescriptor[g_ui32TxDescIndex].ui32Count = (uint32_t)i32BufLen;
    g_psTxDescriptor[g_ui32TxDescIndex].ui32CtrlStatus =
        (DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG |
         DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_IP_ALL_CKHSUMS |
         DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_OWN);

    //
    // Tell the DMA to reacquire the descriptor now that we've filled it in.
    //
    MAP_EMACTxDMAPollDemand(EMAC0_BASE);

    //
    // Return the number of bytes sent.
    //
    return(i32BufLen);
}

//*****************************************************************************
//
// Initialize the transmit and receive DMA descriptors.  We apparently need
// a minimum of 3 descriptors in each chain.  This is overkill since uIP uses
// a single, common transmit and receive buffer so we tag each descriptor
// with the same buffer and will make sure we only hand the DMA one descriptor
// at a time.
//
//*****************************************************************************
void
InitDescriptors(uint32_t ui32Base)
{
    uint32_t ui32Loop;

    //
    // Initialize each of the transmit descriptors.  Note that we leave the OWN
    // bit clear here since we have not set up any transmissions yet.
    //
    for(ui32Loop = 0; ui32Loop < NUM_TX_DESCRIPTORS; ui32Loop++)
    {
        g_psTxDescriptor[ui32Loop].ui32Count =
            (DES1_TX_CTRL_SADDR_INSERT |
             (TX_BUFFER_SIZE << DES1_TX_CTRL_BUFF1_SIZE_S));
        g_psTxDescriptor[ui32Loop].pvBuffer1 = g_pui8TxBuffer;
        g_psTxDescriptor[ui32Loop].DES3.pLink =
            (ui32Loop == (NUM_TX_DESCRIPTORS - 1)) ?
            g_psTxDescriptor : &g_psTxDescriptor[ui32Loop + 1];
        g_psTxDescriptor[ui32Loop].ui32CtrlStatus =
            (DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG |
             DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_CHAINED |
             DES0_TX_CTRL_IP_ALL_CKHSUMS);
    }

    //
    // Initialize each of the receive descriptors.  We clear the OWN bit here
    // to make sure that the receiver doesn't start writing anything
    // immediately.
    //
    for(ui32Loop = 0; ui32Loop < NUM_RX_DESCRIPTORS; ui32Loop++)
    {
        g_psRxDescriptor[ui32Loop].ui32CtrlStatus = 0;
        g_psRxDescriptor[ui32Loop].ui32Count =
            (DES1_RX_CTRL_CHAINED |
             (RX_BUFFER_SIZE << DES1_RX_CTRL_BUFF1_SIZE_S));
        g_psRxDescriptor[ui32Loop].pvBuffer1 = g_pui8RxBuffer;
        g_psRxDescriptor[ui32Loop].DES3.pLink =
            (ui32Loop == (NUM_RX_DESCRIPTORS - 1)) ?
            g_psRxDescriptor : &g_psRxDescriptor[ui32Loop + 1];
    }

    //
    // Set the descriptor pointers in the hardware.
    //
    MAP_EMACRxDMADescriptorListSet(ui32Base, g_psRxDescriptor);
    MAP_EMACTxDMADescriptorListSet(ui32Base, g_psTxDescriptor);

    //
    // Start from the beginning of both descriptor chains.  We actually set
    // the transmit descriptor index to the last descriptor in the chain
    // since it will be incremented before use and this means the first
    // transmission we perform will use the correct descriptor.
    //
    g_ui32RxDescIndex = 0;
    g_ui32TxDescIndex = NUM_TX_DESCRIPTORS - 1;
}

//*****************************************************************************
//
// This example demonstrates the use of the Ethernet Controller with the uIP
// TCP/IP stack.
//
//*****************************************************************************
int
main(void)
{
    uip_ipaddr_t sIPAddr;
    static struct uip_eth_addr sTempAddr;
    int32_t i32PeriodicTimer, i32ARPTimer;
    uint32_t ui32User0, ui32User1;
    uint32_t ui32Temp, ui32PHYConfig;

    //
    // Run from the PLL at 120 MHz.
    //
    ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                           SYSCTL_OSC_MAIN |
                                           SYSCTL_USE_PLL |
                                           SYSCTL_CFG_VCO_480), 120000000);

    //
    // Configure the device pins.
    //
    PinoutSet(true, false);

    //Putty app pins sets
    PuttyInit();

    //Timer init.
    TimerInit();
    //
    // Initialize the UART, clear the terminal, and print banner.
    //
    UARTStdioConfig(0, 115200, ui32SysClock);
    UARTprintf("\033[2J\033[H");
    UARTprintf("Ethernet with uIP\n-----------------\n\n");

    UpdateStatus("Using Internal PHY.");
    ui32PHYConfig = (EMAC_PHY_TYPE_INTERNAL | EMAC_PHY_INT_MDIX_EN |
                     EMAC_PHY_AN_100B_T_FULL_DUPLEX);

    //
    // Read the MAC address from the user registers.
    //
    MAP_FlashUserGet(&ui32User0, &ui32User1);
    if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
    {
        //
        // We should never get here.  This is an error if the MAC address has
        // not been programmed into the device.  Exit the program.
        //
        UpdateStatus("MAC Address Not Programmed!");
        while(1)
        {
        }
    }

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
    sTempAddr.addr[0] = ((ui32User0 >> 0) & 0xff);
    sTempAddr.addr[1] = ((ui32User0 >> 8) & 0xff);
    sTempAddr.addr[2] = ((ui32User0 >> 16) & 0xff);
    sTempAddr.addr[3] = ((ui32User1 >> 0) & 0xff);
    sTempAddr.addr[4] = ((ui32User1 >> 8) & 0xff);
    sTempAddr.addr[5] = ((ui32User1 >> 16) & 0xff);

    //
    // Configure SysTick for a periodic interrupt.
    //
    MAP_SysTickPeriodSet(ui32SysClock / SYSTICKHZ);
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();

    //
    // Enable and reset the Ethernet modules.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_EMAC0);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_EPHY0);

    //
    // Wait for the MAC to be ready.
    //
    UpdateStatus("Waiting for MAC to be ready...");
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0))
    {
    }

    //
    // Configure for use with the internal PHY.
    //
    MAP_EMACPHYConfigSet(EMAC0_BASE, ui32PHYConfig);
    UpdateStatus("MAC ready.");

    //
    // Reset the MAC.
    //
    MAP_EMACReset(EMAC0_BASE);

    //
    // Initialize the MAC and set the DMA mode.
    //
    MAP_EMACInit(EMAC0_BASE, ui32SysClock,
                 EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED, 4, 4,
                 0);

    //
    // Set MAC configuration options.
    //
    MAP_EMACConfigSet(EMAC0_BASE,
                      (EMAC_CONFIG_FULL_DUPLEX | EMAC_CONFIG_CHECKSUM_OFFLOAD |
                       EMAC_CONFIG_7BYTE_PREAMBLE | EMAC_CONFIG_IF_GAP_96BITS |
                       EMAC_CONFIG_USE_MACADDR0 |
                       EMAC_CONFIG_SA_FROM_DESCRIPTOR |
                       EMAC_CONFIG_BO_LIMIT_1024),
                      (EMAC_MODE_RX_STORE_FORWARD |
                       EMAC_MODE_TX_STORE_FORWARD |
                       EMAC_MODE_TX_THRESHOLD_64_BYTES |
                       EMAC_MODE_RX_THRESHOLD_64_BYTES), 0);

    //
    // Initialize the Ethernet DMA descriptors.
    //
    InitDescriptors(EMAC0_BASE);

    //
    // Program the hardware with its MAC address (for filtering).
    //
    MAP_EMACAddrSet(EMAC0_BASE, 0, (uint8_t *)&sTempAddr);

    //
    // Wait for the link to become active.
    //
    UpdateStatus("Waiting for Link.");
    while((MAP_EMACPHYRead(EMAC0_BASE, 0, EPHY_BMSR) &
           EPHY_BMSR_LINKSTAT) == 0)
    {
    }

    UpdateStatus("Link Established.");

    //
    // Set MAC filtering options.  We receive all broadcast and multicast
    // packets along with those addressed specifically for us.
    //
    MAP_EMACFrameFilterSet(EMAC0_BASE, (EMAC_FRMFILTER_SADDR |
                                        EMAC_FRMFILTER_PASS_MULTICAST |
                                        EMAC_FRMFILTER_PASS_NO_CTRL));

    //
    // Clear any pending interrupts.
    //
    MAP_EMACIntClear(EMAC0_BASE, EMACIntStatus(EMAC0_BASE, false));

    //
    // Initialize the uIP TCP/IP stack.
    //
    uip_init();

    //
    // Set the local MAC address (for uIP).
    //
    uip_setethaddr(sTempAddr);
#ifdef USE_STATIC_IP
    uip_ipaddr(sIPAddr, DEFAULT_IPADDR0, DEFAULT_IPADDR1, DEFAULT_IPADDR2,
               DEFAULT_IPADDR3);
    uip_sethostaddr(sIPAddr);
    ShowIPAddress(sIPAddr);
    uip_ipaddr(sIPAddr, DEFAULT_NETMASK0, DEFAULT_NETMASK1, DEFAULT_NETMASK2,
               DEFAULT_NETMASK3);
    uip_setnetmask(sIPAddr);
#else
    uip_ipaddr(sIPAddr, 0, 0, 0, 0);
    uip_sethostaddr(sIPAddr);
    UpdateStatus("Waiting for IP address...");
    uip_ipaddr(sIPAddr, 0, 0, 0, 0);
    uip_setnetmask(sIPAddr);
#endif

    //
    // Enable the Ethernet MAC transmitter and receiver.
    //
    MAP_EMACTxEnable(EMAC0_BASE);
    MAP_EMACRxEnable(EMAC0_BASE);

    //
    // Enable the Ethernet interrupt.
    //
    MAP_IntEnable(INT_EMAC0);

    //
    // Enable the Ethernet RX Packet interrupt source.
    //
    MAP_EMACIntEnable(EMAC0_BASE, EMAC_INT_RECEIVE);

    //
    // Mark the first receive descriptor as available to the DMA to start
    // the receive processing.
    //
    g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus |= DES0_RX_CTRL_OWN;

    //
    // Initialize the TCP/IP Application (e.g. web server).
    //
    hello_world_init();

#ifndef USE_STATIC_IP
    //
    // Initialize the DHCP Client Application.
    //
    dhcpc_init(&sTempAddr.addr[0], 6);
    dhcpc_request();
#endif

    //
    // Main Application Loop.
    //
    i32PeriodicTimer = 0;
    i32ARPTimer = 0;
    while(true)
    {
        //
        // Wait for an event to occur.  This can be either a System Tick event,
        // or an RX Packet event.
        //
        while(!g_ui32Flags)
        {
        }

        //
        // If SysTick, Clear the SysTick interrupt flag and increment the
        // timers.
        //
        if(HWREGBITW(&g_ui32Flags, FLAG_SYSTICK) == 1)
        {
            HWREGBITW(&g_ui32Flags, FLAG_SYSTICK) = 0;
            i32PeriodicTimer += SYSTICKMS;
            i32ARPTimer += SYSTICKMS;
        }

        //
        // Check for an RX Packet and read it.
        //
        if(HWREGBITW(&g_ui32Flags, FLAG_RXPKT))
        {
            //
            // Clear the RX Packet event flag.
            //
            HWREGBITW(&g_ui32Flags, FLAG_RXPKT) = 0;

            // Get the packet and set uip_len for uIP stack usage.
            //
            uip_len = (unsigned short)PacketReceive(EMAC0_BASE, uip_buf,
                                                    sizeof(g_pui8UIPBuffer));

            //
            // Process incoming IP packets here.
            //
            if(BUF->type == htons(UIP_ETHTYPE_IP))
            {
                uip_arp_ipin();
                uip_input();

                //
                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                //
                if(uip_len > 0)
                {
                    uip_arp_out();
                    PacketTransmit(EMAC0_BASE, uip_buf, uip_len);
                    uip_len = 0;
                }
            }

            //
            // Process incoming ARP packets here.
            //
            else if(BUF->type == htons(UIP_ETHTYPE_ARP))
            {
                uip_arp_arpin();

                //
                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                //
                if(uip_len > 0)
                {
                    PacketTransmit(EMAC0_BASE, uip_buf, uip_len);
                    uip_len = 0;
                }
            }
        }

        //
        // Process TCP/IP Periodic Timer here.
        //
        if(i32PeriodicTimer > UIP_PERIODIC_TIMER_MS)
        {
            i32PeriodicTimer = 0;
            for(ui32Temp = 0; ui32Temp < UIP_CONNS; ui32Temp++)
            {
                uip_periodic(ui32Temp);

                //
                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                //
                if(uip_len > 0)
                {
                    uip_arp_out();
                    PacketTransmit(EMAC0_BASE, uip_buf, uip_len);
                    uip_len = 0;
                }
            }

#if UIP_UDP
            for(ui32Temp = 0; ui32Temp < UIP_UDP_CONNS; ui32Temp++)
            {
                uip_udp_periodic(ui32Temp);

                //
                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                //
                if(uip_len > 0)
                {
                    uip_arp_out();
                    PacketTransmit(EMAC0_BASE, uip_buf, uip_len);
                    uip_len = 0;
                }
            }
#endif
        }

        //
        // Process ARP Timer here.
        //
        if(i32ARPTimer > UIP_ARP_TIMER_MS)
        {
            i32ARPTimer = 0;
            uip_arp_timer();
        }
    }
}

#ifdef UIP_ARCH_IPCHKSUM
//*****************************************************************************
//
// Return the IP checksum for the packet in uip_buf.  This is a dummy since
// the hardware calculates this for us.
//
//*****************************************************************************
u16_t
uip_ipchksum(void)
{
    //
    // Dummy function - the hardware calculates and inserts all required
    // checksums for us.
    //
    return(0xffff);
}

//*****************************************************************************
//
// This is a dummy since the hardware calculates this for us.
//
//*****************************************************************************
u16_t
uip_chksum(u16_t *data, u16_t len)
{
    return(0xffff);
}

//*****************************************************************************
//
// This is a dummy since the hardware calculates this for us.
//
//*****************************************************************************
u16_t
uip_icmp6chksum(void)
{
    return(0xffff);
}

//*****************************************************************************
//
// This is a dummy since the hardware calculates this for us.
//
//*****************************************************************************
u16_t
uip_tcpchksum(void)
{
    return(0xffff);
}
#endif
//startup_css.c
//*****************************************************************************
//
// startup_ccs.c - Startup code for use with TI's Code Composer Studio.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
extern void _c_int00(void);

//*****************************************************************************
//
// Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern uint32_t __STACK_TOP;

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
extern void EthernetIntHandler(void);
extern void SysTickIntHandler(void);
extern void Timer0AIntHandler(void);
//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((uint32_t)&__STACK_TOP),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    IntDefaultHandler,                      // The MPU fault handler
    IntDefaultHandler,                      // The bus fault handler
    IntDefaultHandler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // SVCall handler
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    IntDefaultHandler,                      // The PendSV handler
    SysTickIntHandler,                      // The SysTick handler
    IntDefaultHandler,                      // GPIO Port A
    IntDefaultHandler,                      // GPIO Port B
    IntDefaultHandler,                      // GPIO Port C
    IntDefaultHandler,                      // GPIO Port D
    IntDefaultHandler,                      // GPIO Port E
    IntDefaultHandler,                      // UART0 Rx and Tx
    IntDefaultHandler,                      // UART1 Rx and Tx
    IntDefaultHandler,                      // SSI0 Rx and Tx
    IntDefaultHandler,                      // I2C0 Master and Slave
    IntDefaultHandler,                      // PWM Fault
    IntDefaultHandler,                      // PWM Generator 0
    IntDefaultHandler,                      // PWM Generator 1
    IntDefaultHandler,                      // PWM Generator 2
    IntDefaultHandler,                      // Quadrature Encoder 0
    IntDefaultHandler,                      // ADC Sequence 0
    IntDefaultHandler,                      // ADC Sequence 1
    IntDefaultHandler,                      // ADC Sequence 2
    IntDefaultHandler,                      // ADC Sequence 3
    IntDefaultHandler,                      // Watchdog timer
	Timer0AIntHandler,                      // Timer 0 subtimer A
    IntDefaultHandler,                      // Timer 0 subtimer B
    IntDefaultHandler,                      // Timer 1 subtimer A
    IntDefaultHandler,                      // Timer 1 subtimer B
    IntDefaultHandler,                      // Timer 2 subtimer A
    IntDefaultHandler,                      // Timer 2 subtimer B
    IntDefaultHandler,                      // Analog Comparator 0
    IntDefaultHandler,                      // Analog Comparator 1
    IntDefaultHandler,                      // Analog Comparator 2
    IntDefaultHandler,                      // System Control (PLL, OSC, BO)
    IntDefaultHandler,                      // FLASH Control
    IntDefaultHandler,                      // GPIO Port F
    IntDefaultHandler,                      // GPIO Port G
    IntDefaultHandler,                      // GPIO Port H
    IntDefaultHandler,                      // UART2 Rx and Tx
    IntDefaultHandler,                      // SSI1 Rx and Tx
    IntDefaultHandler,                      // Timer 3 subtimer A
    IntDefaultHandler,                      // Timer 3 subtimer B
    IntDefaultHandler,                      // I2C1 Master and Slave
    IntDefaultHandler,                      // CAN0
    IntDefaultHandler,                      // CAN1
    EthernetIntHandler,                     // Ethernet
    IntDefaultHandler,                      // Hibernate
    IntDefaultHandler,                      // USB0
    IntDefaultHandler,                      // PWM Generator 3
    IntDefaultHandler,                      // uDMA Software Transfer
    IntDefaultHandler,                      // uDMA Error
    IntDefaultHandler,                      // ADC1 Sequence 0
    IntDefaultHandler,                      // ADC1 Sequence 1
    IntDefaultHandler,                      // ADC1 Sequence 2
    IntDefaultHandler,                      // ADC1 Sequence 3
    IntDefaultHandler,                      // External Bus Interface 0
    IntDefaultHandler,                      // GPIO Port J
    IntDefaultHandler,                      // GPIO Port K
    IntDefaultHandler,                      // GPIO Port L
    IntDefaultHandler,                      // SSI2 Rx and Tx
    IntDefaultHandler,                      // SSI3 Rx and Tx
    IntDefaultHandler,                      // UART3 Rx and Tx
    IntDefaultHandler,                      // UART4 Rx and Tx
    IntDefaultHandler,                      // UART5 Rx and Tx
    IntDefaultHandler,                      // UART6 Rx and Tx
    IntDefaultHandler,                      // UART7 Rx and Tx
    IntDefaultHandler,                      // I2C2 Master and Slave
    IntDefaultHandler,                      // I2C3 Master and Slave
    IntDefaultHandler,                      // Timer 4 subtimer A
    IntDefaultHandler,                      // Timer 4 subtimer B
    IntDefaultHandler,                      // Timer 5 subtimer A
    IntDefaultHandler,                      // Timer 5 subtimer B
    IntDefaultHandler,                      // FPU
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // I2C4 Master and Slave
    IntDefaultHandler,                      // I2C5 Master and Slave
    IntDefaultHandler,                      // GPIO Port M
    IntDefaultHandler,                      // GPIO Port N
    0,                                      // Reserved
    IntDefaultHandler,                      // Tamper
    IntDefaultHandler,                      // GPIO Port P (Summary or P0)
    IntDefaultHandler,                      // GPIO Port P1
    IntDefaultHandler,                      // GPIO Port P2
    IntDefaultHandler,                      // GPIO Port P3
    IntDefaultHandler,                      // GPIO Port P4
    IntDefaultHandler,                      // GPIO Port P5
    IntDefaultHandler,                      // GPIO Port P6
    IntDefaultHandler,                      // GPIO Port P7
    IntDefaultHandler,                      // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,                      // GPIO Port Q1
    IntDefaultHandler,                      // GPIO Port Q2
    IntDefaultHandler,                      // GPIO Port Q3
    IntDefaultHandler,                      // GPIO Port Q4
    IntDefaultHandler,                      // GPIO Port Q5
    IntDefaultHandler,                      // GPIO Port Q6
    IntDefaultHandler,                      // GPIO Port Q7
    IntDefaultHandler,                      // GPIO Port R
    IntDefaultHandler,                      // GPIO Port S
    IntDefaultHandler,                      // SHA/MD5 0
    IntDefaultHandler,                      // AES 0
    IntDefaultHandler,                      // DES3DES 0
    IntDefaultHandler,                      // LCD Controller 0
    IntDefaultHandler,                      // Timer 6 subtimer A
    IntDefaultHandler,                      // Timer 6 subtimer B
    IntDefaultHandler,                      // Timer 7 subtimer A
    IntDefaultHandler,                      // Timer 7 subtimer B
    IntDefaultHandler,                      // I2C6 Master and Slave
    IntDefaultHandler,                      // I2C7 Master and Slave
    IntDefaultHandler,                      // HIM Scan Matrix Keyboard 0
    IntDefaultHandler,                      // One Wire 0
    IntDefaultHandler,                      // HIM PS/2 0
    IntDefaultHandler,                      // HIM LED Sequencer 0
    IntDefaultHandler,                      // HIM Consumer IR 0
    IntDefaultHandler,                      // I2C8 Master and Slave
    IntDefaultHandler,                      // I2C9 Master and Slave
    IntDefaultHandler                       // GPIO Port T
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetISR(void)
{
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}

