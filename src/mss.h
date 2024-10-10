/*************************************************************************
Title:    Modular Signal System Support Library
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     mss.h
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

#ifndef _MSS_H_
#define _MSS_H_

#include <stdint.h>
#include <stdbool.h>
#include "debouncer.h"
#include "signalAspect.h"

//  Definitions:
//  Physical   - The state of the MSS wires
//  Indication - The "meaning" to be conveyed to a train viewing the signal
//  Aspect     - The appearance of the signal - color, flashing, etc.

#define MSS_ASPECT_OPTION_APPRCH_LIGHTING   0x01
#define MSS_ASPECT_OPTION_FOUR_INDICATION   0x02


#define MSS_MASK_ADJACENT        0x01
#define MSS_MASK_APPROACH        0x02
#define MSS_MASK_ADV_APPROACH    0x04
#define MSS_MASK_DIV_APPROACH    0x08

typedef enum 
{
	INDICATION_STOP               = 0,
	INDICATION_APPROACH           = 1,
	INDICATION_APPROACH_DIVERGING = 2,
	INDICATION_ADVANCE_APPROACH   = 3,
	INDICATION_CLEAR              = 4
} MSSPortIndication_t;

typedef struct
{
	MSSPortIndication_t indication;
	DebounceState8_t debounce;
} MSSPort_t;


void mssReadPort(MSSPort_t* port, volatile uint8_t* adjacentPort, uint8_t adjacentMask,
	volatile uint8_t* approachPort, uint8_t approachMask,
	volatile uint8_t* advApproachPort, uint8_t advApproachMask,
	volatile uint8_t* divApproachPort, uint8_t divApproachMask);

void mssIndicationToSingleHeadAspect(MSSPortIndication_t indication, SignalAspect_t* aspect, uint8_t options, bool approachActive);

void mssPortInitialize(MSSPort_t* port);

bool mssPortApproach(MSSPort_t* port);


#endif
