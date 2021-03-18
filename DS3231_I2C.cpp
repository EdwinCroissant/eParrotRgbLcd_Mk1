/*
 * Very simple library for the DS3231 RTC.
 *
 * uses the I2C library from Wayne Truchsess
 *
 * Copyright (C) 2017 Edwin Croissant
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * See the README.md file for additional information.
 */

#include "DS3231_I2C.h"


SimpleDS3231I2C::SimpleDS3231I2C(void) {
	year = 0;
	month = 0;
	day = 0;
	hour = 0;
	minute = 0;
	second = 0;
}

void SimpleDS3231I2C::readClock(void) {
	uint8_t temp;
	I2c.read(I2Caddr, RegStart, 7);
	temp = I2c.receive();
	second = (temp & 0x0F) + ((temp >> 4) * 10);
	temp = I2c.receive();
	minute = (temp & 0x0F) + ((temp >> 4) * 10);
	temp = I2c.receive();
	hour = (temp & 0x0F) + ((temp >> 4) * 10);
	I2c.receive(); // day number
	temp = I2c.receive();
	day = (temp & 0x0F) + ((temp >> 4) * 10);
	temp = I2c.receive();
	month = (temp & 0x0F) + (((temp >> 4) & 0x03) * 10);	// discard century bit
	temp = I2c.receive();
	year = (temp & 0x0F) + ((temp >> 4) * 10);
}

void SimpleDS3231I2C::setClock(void) {
	I2c._start();
	I2c._sendAddress(SLA_W(I2Caddr));
	I2c._sendByte(RegStart);
	I2c._sendByte(((second % 10) | ((second / 10) << 4)) & 0x7F);
	I2c._sendByte(((minute % 10) | ((minute / 10) << 4)) & 0x7F);
	I2c._sendByte(((hour % 10) | ((hour / 10) << 4)) & 0x3F); // force 24h mode
	I2c._sendByte(1);
	I2c._sendByte(((day % 10) | ((day / 10) << 4)) & 0x3F);
	I2c._sendByte(((month % 10) | ((month / 10) << 4)) & 0x1F);
	I2c._sendByte((year % 10) | ((year / 10) << 4));
	I2c._stop();
	I2c._start();
	I2c._sendAddress(SLA_W(I2Caddr));
	I2c._sendByte(RegControl);
	I2c._sendByte(0b00000100); // enable oscillator
	I2c._sendByte(0b00000000); // clear Oscillator Stop Flag (OSF)
	I2c._stop();
}

bool SimpleDS3231I2C::isRunning(void) {
	// Returns false if the oscillator has been stopped.
	uint8_t temp = 0;
	I2c._start();
	I2c._sendAddress(SLA_W(I2Caddr));
	I2c._sendByte(RegStatus);
	I2c._start();	// repeated start
	I2c._sendAddress(SLA_R(I2Caddr));
	I2c._receiveByte(0, &temp);
	I2c._stop();
	if (bitRead(temp,7))	// Oscillator Stop Flag (OSF) set
		return false;
		else return true;
}

SimpleDS3231I2C rtc = SimpleDS3231I2C();
