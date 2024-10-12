/*************************************************************************
Title:    Hardware Configuration for MSS-SIDING
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

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#define LOOP_UPDATE_TIME_MS       50
#define TURNOUT_LOCKOUT_TIME_MS  200
#define STARTUP_LOCKOUT_TIME_MS  500

#define TCA9555_ADDR_000  0x20
#define TCA9555_GPIN0        0
#define TCA9555_GPIN1        1
#define TCA9555_GPOUT0       2
#define TCA9555_GPOUT1       3
#define TCA9555_GPDDR0       6
#define TCA9555_GPDDR1       7

// TCA9555 - GPIO 0
//  IO07 - Output - Relay Driver (active high)
//  IO06 - Output - Force Adv Approach on MSS Port A (points end)
//  IO05 - Input  - Option Switch E
//  IO04 - Input  - Option Switch D
//  IO03 - Input  - Option Switch C
//  IO02 - Input  - Option Switch B
//  IO01 - Input  - Option Switch A
//  IO00 - Input  - Signal Common Sense

#define OPTION_COMMON_ANODE          0x01
#define OPTION_A_APPROACH_LIGHTING   0x02
#define OPTION_B_FOUR_ASPECT         0x04
#define OPTION_C_SEARCHLIGHT_MODE    0x08
#define OPTION_D_LIMIT_DIVERGING     0x10
#define OPTION_E_RESERVED            0x20
#define MSS_SET_FORCE_POINTS_AA      0x40
#define MSS_SET_ROUTE_RELAY          0x80


// TCA9555 - GPIO 1
//  IO17 - Input  - Main AA In
//  IO16 - Input  - Turnout Position (high = diverging)
//  IO15 - Input  - Points S
//  IO14 - Input  - Points A Out
//  IO13 - Input  - Points A In
//  IO12 - Input  - Points AA In
//  IO11 - Input  - Points AA Out
//  IO10 - Input  - Siding AA In

#define MSS_MAIN_AA_IN        0x80
#define MSS_TO_IS_DIVERGING   0x40
#define MSS_POINTS_S          0x20
#define MSS_POINTS_A_OUT      0x10
#define MSS_POINTS_A_IN       0x08
#define MSS_POINTS_AA_IN      0x04
#define MSS_POINTS_AA_OUT     0x02
#define MSS_SIDING_AA_IN      0x01

// Signal Port Connections
// These are in the order of:
//  Red address, bitmask
//  Yellow address, bitmask
//  Green address, bitmask

#define SIGNAL_HEAD_AU_DEF   &PORTA, _BV(PA7), &PORTA, _BV(PA6), &PORTA, _BV(PA5)
#define SIGNAL_HEAD_AL_DEF   &PORTA, _BV(PA4), &PORTA, _BV(PA3), &PORTB, _BV(PB0)
#define SIGNAL_HEAD_B_DEF   &PORTB, _BV(PB6), &PORTB, _BV(PB5), &PORTB, _BV(PB4)
#define SIGNAL_HEAD_C_DEF   &PORTB, _BV(PB3), &PORTB, _BV(PB2), &PORTB, _BV(PB1)


#endif
