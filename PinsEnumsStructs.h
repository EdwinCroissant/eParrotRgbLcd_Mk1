/* Copyright (C) 2017 Edwin Croissant
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 */

#ifndef PINSENUMSSTRUCTS_H_
#define PINSENUMSSTRUCTS_H_
#include <SingleDS18B20.h>		//https://github.com/EdwinCroissantArduinoLibraries/SingleDS18B20

/* -----------------IMPORTANT-----------------
 * To use a Nano, flash a Uno bootloader otherwise
 * the program will not fit.
 */

/* -----------------IMPORTANT-----------------
 * Remove pin 9 from the LCD RGB 16x2 Keypad Shield when using the
 * Robotdyn Data logger shield as this pin is connected to the LCD
 * contrast trimmer and the logger shield uses this pin for the CS signal.
 */

/*----( pin assignments )----*/

enum pins {
	pinTX = 0,					// Serial TX
	pinRX = 1,					// Serial RX
	pinBeeper = 2,				// Passive beeper use tone library
	pinInterrupt = 3,			// Interrupt CH376S --> INT
	pinBussy = 4,				// Bussy CH376S --> D4
	pinVent2 = 5,				// DS18B20 only
	pinVent1 = 6,				// DS18B20 only
	pinBoiler = 7,				// DS18B20 only
	pinVapor = 8,				// SMT172 or DS18B20
	pinLCD = 9,					// connected to the LCD contrast trimmer? may be spare
	pinCS = 10,					// SPI CS CH376S --> D3
	pinMOSI = 11,				// SPI MOSI CH376S --> D6
	pinMISO = 12,				// SPI MISO CH376 --> D7
	pinLed = 13,				// SPI SCK CH376S --> D5
	pinHealthy= A0,				// output pin: high when healthy, low when alarm
	pinSpare = A1,				// spare pin for future use
	pinBoilerPressure = A2,		// MPXV7002DP or equivalent
	pinCoolant = A3,			// DS18B20 only
	pinSDA = A4,				// I2C SDA
	pinSCL = A5,				// I2C SCL
};

/*
 * Connect the CH376S RD & WR pin to ground and remove jumper.
 * Remove 3.3V regulator and connect input to output.
 * Please note that there are two types of CH376S modules
 * with different pinouts.
 */

/*-----( EEPROM helper )-----*/

template <class T> int EEPROM_updateAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.update(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}

/*
 * To use enumerated types and structures as parameters for functions
 * their declarations must be done in a separate header file.
 */

/*----( Recognizable names for the sensor types )----*/
enum sensorType {
	NoSensor = 0,
	smt172 = 1,
	DS18B20 = 2
};

/*----( Recognizable names for the alarm status )----*/
enum alarmStatus {
	alarmArmed,
	alarmTriggered,
	alarmSilenced,
	alarmAcknowledged
};

enum warningStatus {
	warningDisabled,
	warningArmed,
	warningTriggered,
	warningSilenced,
	warningAcknowledged
};

enum healthAlarm {
	healthOk,
	healthBoiler,
	healthVent1,
	healthVent2,
	healthCoolant
};

struct temperatureSensor {
	sensorType Type;
	float Temperature;
};

#endif /* PINSENUMSSTRUCTS_H_ */
