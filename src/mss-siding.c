/*************************************************************************
Title:    MSS-CASCADE-BASIC
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>

#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>

#include "debouncer.h"
#include "optionSwitches.h"
#include "signalHead.h"
#include "mss.h"
#include "i2c.h"
#include "light_ws2812.h"
typedef enum
{
	STATUS_RED,
	STATUS_YELLOW,
	STATUS_GREEN,
	STATUS_BLUE,
	STATUS_PURPLE,
	STATUS_WHITE,
	STATUS_OFF,
	STATUS_UNKNOWN,
} Status;


void setStatusLED(Status status)
{
	static Status oldStatus = STATUS_UNKNOWN;
	struct cRGB led;

	if(status != oldStatus)
	{
		switch(status)
		{
			case STATUS_RED:
				led.r = 64;
				led.g = 0;
				led.b = 0;
				break;
			case STATUS_YELLOW:
				led.r = 64;
				led.g = 24;
				led.b = 0;
				break;
			case STATUS_GREEN:
				led.r = 0;
				led.g = 64;
				led.b = 0;
				break;
			case STATUS_BLUE:
				led.r = 0;
				led.g = 0;
				led.b = 64;
				break;
			case STATUS_PURPLE:
				led.r = 64;
				led.g = 0;
				led.b = 64;
				break;
			case STATUS_WHITE:
				led.r = 64;
				led.g = 64;
				led.b = 64;
				break;
			case STATUS_OFF:
				led.r = 0;
				led.g = 0;
				led.b = 0;
				break;
			case STATUS_UNKNOWN:
				break;
		}
		ws2812_setleds(&led,1);
		oldStatus = status;
	}
}

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

volatile uint8_t signalHeadOptions = 0;
volatile uint32_t millis = 0;
SignalState_t signalAU;
SignalState_t signalAL;
SignalState_t signalB;
SignalState_t signalC;

// A few hardware definitions

// MSS Port Connections
// These are in the order of:
//  Adjacent address, bitmask
//  Approach address, bitmask
//  Adv Approach address, bitmask
//  Divering Approach address, bitmask

#define MSS_PORT_A_DEF   &PINA, _BV(4), &PINA, _BV(3), &PINA, _BV(2), NULL, _BV(0)
#define MSS_PORT_B_DEF   &PINA, _BV(7), &PINA, _BV(6), &PINA, _BV(5), NULL, _BV(0)

// Signal Port Connections
// These are in the order of:
//  Red address, bitmask
//  Yellow address, bitmask
//  Green address, bitmask

#define SIGNAL_HEAD_AU_DEF   &PORTA, _BV(PA7), &PORTA, _BV(PA6), &PORTA, _BV(PA5)
#define SIGNAL_HEAD_AL_DEF   &PORTA, _BV(PA4), &PORTA, _BV(PA3), &PORTB, _BV(PB0)
#define SIGNAL_HEAD_B_DEF   &PORTB, _BV(PB6), &PORTB, _BV(PB5), &PORTB, _BV(PB4)
#define SIGNAL_HEAD_C_DEF   &PORTB, _BV(PB3), &PORTB, _BV(PB2), &PORTB, _BV(PB1)

uint32_t getMillis()
{
	uint32_t retmillis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}
	return retmillis;
}

ISR(TIMER0_COMPA_vect) 
{
	static uint8_t flasherCounter = 0;
	static uint8_t flasher = 0;
	static uint8_t pwmPhase = 0;
	static uint8_t subMillisCounter = 0;
	
	// The ISR does two main things - updates the LED outputs since
	//  PWM is done through software, and updates millis which is used
	//  to trigger various events
	// We need this to run at roughly 125 Hz * number of PWM levels (32).  That makes a nice round 4kHz
	
	// First thing, output the signals so that the PWM doesn't get too much jitter

	signalHeadISR_OutputPWM(&signalAU, signalHeadOptions, pwmPhase, SIGNAL_HEAD_AU_DEF);
	signalHeadISR_OutputPWM(&signalAL, signalHeadOptions, pwmPhase, SIGNAL_HEAD_AL_DEF);
	signalHeadISR_OutputPWM(&signalB, signalHeadOptions, pwmPhase, SIGNAL_HEAD_B_DEF);
	signalHeadISR_OutputPWM(&signalC, signalHeadOptions, pwmPhase, SIGNAL_HEAD_C_DEF);

	// Now do all the counter incrementing and such
	if (++subMillisCounter >= 4)
	{
		subMillisCounter = 0;
		millis++;
	}

	pwmPhase = (pwmPhase + 1) & 0x1F;

	if (0 == pwmPhase)
	{
		pwmPhase = 0;
		flasherCounter++;
		if (flasherCounter > 94)
		{
			flasher ^= 0x01;
			flasherCounter = 0;
		}

		// We rolled over the PWM counter, calculate the next PWM widths
		// This runs at 125 frames/second essentially

		signalHeadISR_AspectToNextPWM(&signalAU, flasher, signalHeadOptions);
		signalHeadISR_AspectToNextPWM(&signalAL, flasher, signalHeadOptions);
		signalHeadISR_AspectToNextPWM(&signalB, flasher, signalHeadOptions);
		signalHeadISR_AspectToNextPWM(&signalC, flasher, signalHeadOptions);
	}
}

void initializeTimer()
{
	TIMSK = 0;                                    // Timer interrupts OFF
	// Set up Timer/Counter0 for 100Hz clock
	TCCR0A = 0b00000001;  // CTC Mode
	TCCR0B = 0b00000010;  // CS01 - 1:8 prescaler
	OCR0A = 250;           // 8MHz / 8 / 125 = 8kHz
	TIMSK = _BV(OCIE0A);
}

int main(void)
{
	DebounceState8_t optionsDebouncer;
	DebounceState8_t mssDebouncer;
	uint32_t lastReadTime = 0;
	uint32_t currentTime = 0;
	uint8_t q=0;
	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	// PORT A
	//  PA7 - Output - A Upper Signal - RED
	//  PA6 - Output - A Upper Signal - YELLOW
	//  PA5 - Output - A Upper Signal - GREEN
	//  PA4 - Output - A Lower Signal - RED
	//  PA3 - Output - A Lower Signal - YELLOW
	//  PA2 - I2C    - SCL
	//  PA1 - Output - WS2812B
	//  PA0 - I2C    - SDA

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Output - B Signal - RED
	//  PB5 - Output - B Signal - YELLOW
	//  PB5 - Output - B Signal - GREEN
	//  PB3 - Output - C Signal - RED
	//  PB2 - Output - C Signal - YELLOW
	//  PB1 - Output - C Signal - GREEN
	//  PB0 - Output - A Lower Signal - GREEN

	PORTA = 0b11111101;
	DDRA  = 0b11111110;

	PORTB = 0b11111111;
	DDRB  = 0b01111111;

	initializeTimer();
	initializeOptions(&optionsDebouncer);
	initializeOptions(&mssDebouncer);

	signalHeadInitialize(&signalAU);
	signalHeadInitialize(&signalAL);
	signalHeadInitialize(&signalB);
	signalHeadInitialize(&signalC);

	signalHeadAspectSet(&signalAU, ASPECT_RED);
	signalHeadAspectSet(&signalAL, ASPECT_RED);
	signalHeadAspectSet(&signalB, ASPECT_RED);
	signalHeadAspectSet(&signalC, ASPECT_RED);

	setStatusLED(STATUS_OFF);

	sei();
	wdt_reset();

#define TCA9555_ADDR_000  0x20
#define TCA9555_GPIN0        0
#define TCA9555_GPIN1        1
#define TCA9555_GPOUT0       2
#define TCA9555_GPOUT1       3
#define TCA9555_GPDDR0       6
#define TCA9555_GPDDR1       7

#define MSS_MAIN_AA_IN        0x80
#define MSS_TO_IS_DIVERGING   0x40
#define MSS_POINTS_S          0x20
#define MSS_POINTS_A_IN       0x10
#define MSS_POINTS_A_OUT      0x08
#define MSS_POINTS_AA_IN      0x04
#define MSS_POINTS_AA_OUT     0x02
#define MSS_SIDING_AA_IN      0x01


	// TCA9555 - GPIO 0
	//  IO07 - Output - Relay Driver (active high)
	//  IO06 - Output - Force Adv Approach on MSS Port A (points end)
	//  IO05 - Input  - Option Switch E
	//  IO04 - Input  - Option Switch D
	//  IO03 - Input  - Option Switch C
	//  IO02 - Input  - Option Switch B
	//  IO01 - Input  - Option Switch A
	//  IO00 - Input  - Signal Common Sense

	// TCA9555 - GPIO 1
	//  IO17 - Input  - Main AA In
	//  IO16 - Input  - Turnout Position (high = diverging)
	//  IO15 - Input  - Points S
	//  IO14 - Input  - Points A Out
	//  IO13 - Input  - Points A In
	//  IO12 - Input  - Points AA In
	//  IO11 - Input  - Points AA Out
	//  IO10 - Input  - Siding AA In

	// Set TCA9555 direction registers
	writeByte(TCA9555_ADDR_000, TCA9555_GPDDR0, 0b00111111);
	writeByte(TCA9555_ADDR_000, TCA9555_GPDDR1, 0b11111111);
	writeByte(TCA9555_ADDR_000, TCA9555_GPOUT0, 0b00000000);
	
	while(1)
	{
		wdt_reset();

		currentTime = getMillis();

		// Because debouncing and such is built into option reading and the MSS library, only 
		//  run the updates every 10mS or so.

		if (((uint32_t)currentTime - lastReadTime) > 10)
		{
			uint8_t optionJumpers = 0;
			uint8_t mssOptions = 0;
			uint8_t ack = 0;
			uint8_t i = 0;

			SignalAspect_t aspectAU = ASPECT_RED;
			SignalAspect_t aspectAL = ASPECT_RED;
			SignalAspect_t aspectB = ASPECT_RED;
			SignalAspect_t aspectC = ASPECT_RED;

			lastReadTime = currentTime;

			if (++q>100)
			{
				signalHeadAspectSet(&signalAU, ASPECT_RED);
			}
			else
			{
				signalHeadAspectSet(&signalAU, ASPECT_GREEN);
			}

			if (q>=200)
				q = 0;

			// Read PCA9555
			ack = readByte(TCA9555_ADDR_000, TCA9555_GPIN0, &i);
			if (ack)
				debounce8(i, &optionsDebouncer);

			ack = readByte(TCA9555_ADDR_000, TCA9555_GPIN1, &i);
			if (ack)
				debounce8(i, &mssDebouncer);

			setStatusLED((i==0)?STATUS_GREEN:STATUS_RED);

			optionJumpers = getDebouncedState(&optionsDebouncer);
			optionJumpers |= OPTION_B_FOUR_ASPECT | OPTION_COMMON_ANODE | OPTION_C_SEARCHLIGHT_MODE;

			// Convert global option bits to signal head option bits
			if (optionJumpers & OPTION_C_SEARCHLIGHT_MODE)
				signalHeadOptions |= SIGNAL_OPTION_SEARCHLIGHT;
			else 
				signalHeadOptions &= ~SIGNAL_OPTION_SEARCHLIGHT;

			if (optionJumpers & OPTION_COMMON_ANODE)
				signalHeadOptions |= SIGNAL_OPTION_COMMON_ANODE;
			else 
				signalHeadOptions &= ~SIGNAL_OPTION_COMMON_ANODE;

			// Convert global option bits to MSS option bits
			if (optionJumpers & OPTION_A_APPROACH_LIGHTING)
				mssOptions |= MSS_ASPECT_OPTION_APPRCH_LIGHTING;

			if (optionJumpers & OPTION_B_FOUR_ASPECT)
				mssOptions |= MSS_ASPECT_OPTION_FOUR_INDICATION;


			// Read state of MSS bus ports coming in
			i = getDebouncedState(&mssDebouncer);

			if ((i & MSS_POINTS_S) || (i & MSS_TO_IS_DIVERGING))
				aspectB = ASPECT_RED;
			else if (i & MSS_POINTS_A_IN)
				aspectB = ASPECT_YELLOW;
			else if ((i & MSS_POINTS_AA_IN) && (mssOptions & MSS_ASPECT_OPTION_FOUR_INDICATION))
				aspectB = ASPECT_FL_YELLOW;
			else
				aspectB = ASPECT_GREEN;


			if (i & MSS_POINTS_S || !(i & MSS_TO_IS_DIVERGING))
				aspectC = ASPECT_RED;
			else if (i & MSS_POINTS_A_IN)
				aspectC = ASPECT_YELLOW;
			else if ((i & MSS_POINTS_AA_IN) && (mssOptions & MSS_ASPECT_OPTION_FOUR_INDICATION))
				aspectC = ASPECT_FL_YELLOW;
			else
				aspectC = ASPECT_GREEN;


			if (i && MSS_TO_IS_DIVERGING)
			{
				aspectAU = ASPECT_RED;
			
				if (i & MSS_POINTS_A_OUT)
					aspectAL = ASPECT_RED;
				else if (i & MSS_POINTS_AA_OUT)
					aspectAL = ASPECT_YELLOW;
				else if ((i && MSS_SIDING_AA_IN) && (mssOptions & MSS_ASPECT_OPTION_FOUR_INDICATION))
					aspectAL = ASPECT_FL_YELLOW;
				else
					aspectAL = ASPECT_GREEN;

			} else {
				aspectAL = ASPECT_RED;
				if (i & MSS_POINTS_A_OUT)
					aspectAU = ASPECT_RED;
				else if ((i & MSS_POINTS_AA_OUT) && (mssOptions & MSS_ASPECT_OPTION_FOUR_INDICATION))
					aspectAU = ASPECT_YELLOW;
				else if (i & MSS_MAIN_AA_IN)
					aspectAU = ASPECT_FL_YELLOW;
				else
					aspectAU = ASPECT_GREEN;
			}
/*
			signalHeadAspectSet(&signalAU, aspectAU);
			signalHeadAspectSet(&signalAL, aspectAL);
			signalHeadAspectSet(&signalB, aspectB);
			signalHeadAspectSet(&signalC, aspectC);*/
		}
	}
}



