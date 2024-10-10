/*************************************************************************
Title:    MSS-CASCADE-BASIC
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     optionSwitches.h
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

#ifndef _OPTIONSWITCHES_H_
#define _OPTIONSWITCHES_H_

#include "debouncer.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define OPTION_COMMON_ANODE          0x01
#define OPTION_A_APPROACH_LIGHTING   0x02
#define OPTION_B_FOUR_ASPECT         0x04
#define OPTION_C_SEARCHLIGHT_MODE    0x08
#define OPTION_D_RESERVED            0x10
#define OPTION_E_RESERVED            0x20

void initializeOptions(DebounceState8_t* optionsDebouncer);
void readOptions(DebounceState8_t* optionsDebouncer);

#endif
