/*************************************************************************
Title:    Big-Bang I2C Library
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     i2c.h
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

#ifndef _I2C_H_
#define _I2C_H_

#include <avr/io.h>
#include <util/delay.h>

#define   SDA   PA0
#define   SCL   PA2

uint8_t writeByte(uint8_t addr, uint8_t cmd, uint8_t writeVal);
uint8_t readByte(uint8_t addr, uint8_t cmd, uint8_t* data);

#endif

