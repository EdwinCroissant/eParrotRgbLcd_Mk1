/* Copyright (C) 2021 Edwin Croissant
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 */
#include "Arduino.h"
#include <I2C.h>				//https://github.com/rambo/I2C
#include <OneWire.h>			//https://github.com/bigjosh/OneWireNoResistor
#include <RgbLcdKeyShieldI2C.h>	//https://github.com/EdwinCroissantArduinoLibraries/RgbLcdKeyShieldI2C
#include <SimpleBMP280I2C.h>	//https://github.com/EdwinCroissantArduinoLibraries/SimpleBMP280I2C
#include <SingleDS18B20.h>		//https://github.com/EdwinCroissantArduinoLibraries/SingleDS18B20
#include <SMT172_T1.h>			//https://github.com/EdwinCroissantArduinoLibraries/SMT172
#include <EEPROM.h>
#include <avr/wdt.h>

#include "PinsEnumsStructs.h"

/*
 * When Settings structure changed increment SETTINGSVERSION
 * this will load the default values and starts the program
 * in the first settings screen
  */
#define SETTINGSVERSION 1

/*
 * With LOG_TO_FLASHDRIVE = 1 logs to flashdrive instead of
 * serial. Needs a a 24LC256 eeprom, CH376S module and a DS3231 rtc
 * otherwise use LOG_TO_FLASHDRIVE = 0.
 */
#define LOG_TO_FLASHDRIVE 0

/*----( strings in flash )----*/
const char msgSplash1[] PROGMEM = 				"eParrot  RGB LCD";
const char msgNo[] PROGMEM = 					"No";
const char msgCanceled[] PROGMEM = 				"Canceled";
const char msgSaved[] PROGMEM = 				"Saved";
const char msghPa[] PROGMEM = 					" hPa";
const char msgdPa[] PROGMEM = 					"cmH2O";
const char msgBaro[] PROGMEM = 					"Baro";
const char msgBPWater[] PROGMEM = 				"BP H2O";
const char msgNoValueABV[] PROGMEM = 			"--.-%";
const char msgNoValueC6[] PROGMEM = 			"---.--";
const char msgNoValueC4[] PROGMEM = 			"--.-";
const char msgBoilerOffset[] PROGMEM = 			"Blr Offs";
const char msgVaporOffset[] PROGMEM = 			"Vpr Offs";
const char msgSilent[] PROGMEM = 				"Silent";
const char msgAudial[] PROGMEM = 				"Beeps";
const char msgForesWarning[] PROGMEM = 			" FS";
const char msgFores[] PROGMEM = 				"FORES";
const char msgAlarm[] PROGMEM = 				"Alarm";
const char msgForesImminent[] PROGMEM = 		"Fores imminent";
const char msgLowABV[] PROGMEM = 				"Low ABV";
const char msgWarmedUp[] PROGMEM = 				"Warmed up";
const char msgBoilerP[] PROGMEM = 				"BlrP ";
const char msgBoiler[] PROGMEM = 				"Boiler ";
const char msgVapor[] PROGMEM = 				"Vapor ";
const char msgVent1[] PROGMEM = 				"Vnt1 ";
const char msgVent2[] PROGMEM = 				"Vnt2 ";
const char msgCoolant[] PROGMEM = 				"Clnt ";
const char msgToLog[] PROGMEM = 				"S to start log";
const char msgToStop[] PROGMEM = 				"S to stop log ";
const char msgSettings[] PROGMEM = 				"S to settings";
const char msgMain[] PROGMEM = 					"S to return";
const char msgWarnings1[] PROGMEM = 			"1 %.2hd%% 2 %.2hd%%  BLR";
const char msgWarnings2[] PROGMEM = 			"3 %.2hd%% 4 %.2hd%%  %.2hdC";
const char msgCoolantSetting[] PROGMEM = 		"Clnt    %.2hd C";
const char msgBoilerPressureSetting[] PROGMEM = "BlrP    %.2hd cmH2O";
const char msgBoilerPressure[] PROGMEM = 		"BlrP %s cmH2O";
const char msgBlrPressSens1[] PROGMEM =			"BlrP sensitivity";
const char msgBlrPressSenSetting[] PROGMEM =	"%.4hd mV/kPa";
const char msgLogHeader[] PROGMEM =				"Time,Baro,VaporT,VaporABV,BoilerT,BoilerABV,Vent1T,Vent2T,CoolantT,BoilerP,Alarm";

#if LOG_TO_FLASHDRIVE == 1

const char msgSplash2[] PROGMEM = 				"MK1 0.0.1  Flash";
const char msgLogging[] PROGMEM = 				"Log to USB";
const char msgNoFlash[] PROGMEM =	 			"No flash drive";
const char msgFlashMounted[] PROGMEM = 			"Flash mounted";
const char msgFlashDetached[] PROGMEM = 		"Device detached";
const char logFilename[] PROGMEM =				"/RUN_00.CSV";
const char msgTime1[] PROGMEM = 				"%.2hd-%.2hd-20%.2hd";
const char msgTime2[] PROGMEM = 				"%.2hd:%.2hd:%.2hd";
const char msgSetTime1[] PROGMEM = 				"YYMMDD  %.2hd/%.2hd/%.2hd";
const char msgSetTime2[] PROGMEM = 				"HHMMSS  %.2hd:%.2hd:%.2hd";
const char msgTimeStamp[] PROGMEM =				"20%.2hd-%.2hd-%.2hdT%.2hd:%.2hd:%.2hd";

#include "CH376_SPI.h"
#include "DS3231_I2C.h"
#include "T2ABV_SMT172_I2C.h"


CH376_SPI flashDrive(pinCS, pinInterrupt, pinBussy);

char Filename[12];

#else

const char msgSplash2[] PROGMEM = 				"MK1 0.0.1 Serial";
const char msgLogging[] PROGMEM = 				"Serial 8N1";
const uint16_t baudrates[] PROGMEM =			{9600, 14400, 19200, 28800, 31250, 38400, 57600};

#include "T2ABV_SMT172.h"
#endif


/*----( make instances for the Dallas sensors, BMP280, etc. )----*/
OneWire pinBoilerSensor(pinBoiler), pinVaporSensor(pinVapor),
		pinVent1Sensor(pinVent1), pinVent2Sensor(pinVent2),
		pinCoolantSensor(pinCoolant);
SingleDS18B20 BoilerDS18B20(&pinBoilerSensor), VaporDS18B20(&pinVaporSensor),
		Vent1DS18B20(&pinVent1Sensor), Vent2DS18B20(&pinVent2Sensor),
		CoolantDS18B20(&pinCoolantSensor);

SimpleBMP280I2C baro; // autodetect I2C address in setup

// RgbLcdKeyShieldI2C lcd(true);	//inverted backlight
RgbLcdKeyShieldI2C lcd;				// non inverted backlight

/*-----( Declare Variables )-----*/

struct sensors {
	float BaroPressure;				// in hPa
	float H2OBoilingPoint;			// in C
	int16_t BoilerPressureRaw;
	float BoilerPressure;			// in cmH2O

	temperatureSensor Vapor;
	float VaporABV;					// in %

	temperatureSensor Boiler;
	float BoilerABV;				// in %
	float BoilerLastTemperature;	// in C
	int16_t WarmupTime;				// in minutes

	temperatureSensor Vent1;
	float Vent1LastTemperature;		// in C

	temperatureSensor Vent2;
	float Vent2LastTemperature;		// in C

	temperatureSensor Coolant;
} Sensors;

struct settings {
		uint8_t Warning[4] {92, 93, 94, 95};	// in %
		uint8_t WarmedUpWarning = 70;			// in C
		int16_t VaporOffset;					// in cC
		int16_t BoilerOffset;					// in cC
		int16_t BoilerPressureOffset;
		uint8_t MaxCoolant = 60;				// in C
		uint8_t MaxBoilerPressure = 10;			// in cmWk
		bool Vent1Enable;
		bool Vent2Enable;
		bool CoolantEnable;
		bool BoilerPressureEnable;
		int8_t ForesWarning = -10;				// 10 C below azeotrope
		int8_t VentAlarmTemperature = 60;		// in C
		int16_t VentAlarmDeltaTemperature = 50;	// in cC
		uint16_t BoilerPressSens = 800;			// in mV/kPa
		uint8_t Baudrate = 6;
} Settings;

warningStatus WarningStatusVapor;
warningStatus WarningStatusBoiler;
alarmStatus AlarmStatusHealth;
healthAlarm HealthAlarm;

bool IsFirstRun;

bool IsLogging;

char lineBuffer[20];

void (*AutoPageFastRefresh)();
void (*AutoPageRefresh)();
void (*ReturnScreen)();
void (*PrintOffset)();

int16_t* Offset;
int16_t OldTemp;

bool Backlight = true;
bool FlashBacklight;
bool Silent;
uint32_t LastSMT172Update;
uint32_t LastSensorUpdate;
uint32_t AlternateBacklightUpdate;
uint32_t StartTimeLog;
uint32_t LastAlarmUpdate;
uint32_t LastWarmingupUpdate;
uint32_t LastInfoScreenUpdate;
uint8_t CurrentWarning = 4; // set to FORES at startup
bool ShowingInfoScreen;


void saveSettings() {
	EEPROM.update(0, SETTINGSVERSION);
	EEPROM_updateAnything(2, Settings);
}

bool loadSettings() {
	if (SETTINGSVERSION != EEPROM.read(0))
		return false;
	EEPROM_readAnything(2, Settings);
	return true;
}

// Helper function to initialize the DS18B20 and start the conversion
// Useful when multiple sensors are used

sensorType initDS18B20(SingleDS18B20 &sensor) {
	if (sensor.read() && sensor.setResolution(SingleDS18B20::res12bit)
			&& sensor.convert())
		return DS18B20;
	else return NoSensor;
}

sensorType initVaporSensor() {
	// disable internal pull up resistor for vapor pin
	pinMode(pinVapor, INPUT);
	digitalWrite(pinVapor, LOW);
	SMT172_T1::startTemperatureByTime(1);
	delay(5); // timeout within 5 millis with no signal present
	if (SMT172_T1::getStatus() != 2)
		return smt172;
	else
		return initDS18B20(VaporDS18B20);
}

// Helper function to print the temperature to the lcd ##.##C
void printTemp6(temperatureSensor &sensor) {
	if (sensor.Type == NoSensor)
		lcd.printP(msgNoValueC6);
	else
		lcd.print(dtostrf(sensor.Temperature, 6, 2, lineBuffer));
	lcd.print('C');
}

// Helper function to print the temperature to the lcd  ##.# C
void printTemp4(temperatureSensor &sensor) {
	if (sensor.Type == NoSensor)
		lcd.printP(msgNoValueC4);
	else
		lcd.print(dtostrf(sensor.Temperature, 4, 1, lineBuffer));
	lcd.print(' ');
	lcd.print('C');
}

//The setup function is called once at startup of the sketch
void setup()
{
	pinMode(pinHealthy, OUTPUT);

	// initialize I2C
	I2c.begin();
	I2c.setSpeed(1);
	I2c.timeOut(10);

	IsFirstRun = !loadSettings();

#if LOG_TO_FLASHDRIVE == 1

	flashDrive.init();
	flashDrive.onStateChange = flashDriveStateChange;
	if (!rtc.isRunning()) IsFirstRun = true;

#endif

	// initialize lcd
	lcd.begin();

	// test barometer
	if (!baro.begin())
		if (!baro.begin(0x77)) {
			lcd.printP(msgNo);
			lcd.print(' ');
			lcd.printP(msgBaro);
			while (true) {
			};
		}

	// Initialize the temperature sensors
	Sensors.Vapor.Type = initVaporSensor();
	Sensors.Boiler.Type = initDS18B20(BoilerDS18B20);
	if (Settings.Vent1Enable) Sensors.Vent1.Type = initDS18B20(Vent1DS18B20);
	if (Settings.Vent2Enable) Sensors.Vent2.Type = initDS18B20(Vent2DS18B20);
	if (Settings.CoolantEnable) Sensors.Coolant.Type = initDS18B20(CoolantDS18B20);

	// show splash
	lcd.setColor(lcd.clWhite);
	lcd.printP(msgSplash1);
	lcd.setCursor(0,1);
	lcd.printP(msgSplash2);

	// read the sensors again to get a valid
	// Vent1LastTemperature and Vent2LastTemperature
	delay(1000);
	readSensors();
	delay(1000);

	// enable the watch dog timer
	wdt_enable(WDTO_2S);

#if LOG_TO_FLASHDRIVE == 1

	if (IsFirstRun) {
		saveSettings();
		screenTime();
	} else
		screenMain();

#else

	if (IsFirstRun) {
		saveSettings();
		screenOffsetVapor();
	} else
		screenMain();

#endif

}

void doFunctionAtInterval(void (*callBackFunction)(), uint32_t &lastEvent,
		uint32_t Interval) {
	uint32_t now = millis();
	if ((now - lastEvent) >= Interval) {
		lastEvent = now;
		callBackFunction();
	}
}

// The loop function is called in an endless loop
void loop()
{
	wdt_reset();
	doFunctionAtInterval(readSMT172, LastSMT172Update, 250);
	doFunctionAtInterval(readSensors, LastSensorUpdate, 1000);	// read the baro and DS18B20's every second
	doFunctionAtInterval(handleAlarms, LastAlarmUpdate, 1000);	// handle the alarms every second
	doFunctionAtInterval(handleWarmingup, LastWarmingupUpdate, 60000);	// check warming up every minute
	if (FlashBacklight) doFunctionAtInterval(alternateBacklight, AlternateBacklightUpdate, 500);
	if (ShowingInfoScreen) doFunctionAtInterval(endInfoScreen, LastInfoScreenUpdate, 1000);
	lcd.readKeys();

#if LOG_TO_FLASHDRIVE == 1
	if (flashDrive.processMessages()) tone(pinBeeper, 880, 250);
#endif
}

#if LOG_TO_FLASHDRIVE == 1

void flashDriveStateChange(uint8_t _state) {
	switch (_state) {
		case flashDrive.mounted:
			tone(pinBeeper, 880, 250);
			infoScreen(msgFlashMounted);
			if (createFile()) {
				StartTimeLog = millis();
				IsLogging = true;
			}
			break;
		case flashDrive.mountfailure:
			tone(pinBeeper, 220, 250);
			infoScreen(msgNoFlash);
			break;
		case flashDrive.detached:
			tone(pinBeeper, 220, 250);
			infoScreen(msgFlashDetached);
			IsLogging = false;
			break;
		default:
			break;
	}
}

bool createFile() {
	bool created = false;
	strcpy_P(Filename, logFilename);
	// create a new file
	for (uint8_t i = 0; i < 100; ++i) {
		Filename[5] = i / 10 + '0';
		Filename[6] = i % 10 + '0';

		if (!flashDrive.fileExists(Filename)) {
			flashDrive.createFile(Filename);
			rtc.readClock();
			flashDrive.setTimeStamp(rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute,
					rtc.second);
			flashDrive.print(Filename);
			flashDrive.print(' ');
			sprintf_P(lineBuffer, msgTimeStamp,
							rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute,
							rtc.second);
			flashDrive.println(lineBuffer);
			flashDrive.printP(msgLogHeader);
			flashDrive.println();
			flashDrive.closeFile(true);
	        created = true;
			break;
		}
	}
	return created;
}

#endif

void endInfoScreen() {
	ShowingInfoScreen = false;
	ReturnScreen();
}

void alternateBacklight() {
	Backlight = !Backlight;
	if (Backlight)
		lcd.setColor(RgbLcdKeyShieldI2C::clRed);
	else
		lcd.setColor(RgbLcdKeyShieldI2C::clViolet);

}

void handleWarmingup() {
	float DeltaT;
	// calculate the warmup time
	if ((Sensors.Boiler.Type != NoSensor) && (Sensors.BoilerLastTemperature > 0)
			&& (Sensors.Boiler.Temperature < Settings.WarmedUpWarning)) {
		DeltaT = Sensors.Boiler.Temperature - Sensors.BoilerLastTemperature;
		Sensors.WarmupTime = int16_t(constrain((float(Settings.WarmedUpWarning)
					- Sensors.Boiler.Temperature) / DeltaT + 0.5, 0, 5999));
	} else
		Sensors.WarmupTime = 0;

	if (Sensors.WarmupTime == 5999)
		Sensors.WarmupTime = 0;

	Sensors.BoilerLastTemperature = Sensors.Boiler.Temperature;
}

void handleWarnings() {
	bool WarningCondition;

	if (WarningStatusVapor == warningDisabled
			&& WarningStatusBoiler == warningDisabled) {
		lcd.setColor(RgbLcdKeyShieldI2C::clWhite);
		FlashBacklight = false;
		return;
	}

	WarningCondition = (Sensors.Boiler.Type != NoSensor)
			&& (Sensors.Boiler.Temperature > Settings.WarmedUpWarning);

	switch (WarningStatusBoiler) {
	case warningArmed:
		if (WarningCondition)
			WarningStatusBoiler = warningTriggered;
		break;
	case warningTriggered:
		ShowingInfoScreen = false; // prevent infoscreen from hijacking warning
		screenWarningWarmedUp();
		break;
	case warningSilenced:
		FlashBacklight = true;
		break;
	default:
		break;
	}

	WarningCondition = (((Sensors.Vapor.Type != NoSensor)
			&& (Sensors.VaporABV < 0 && CurrentWarning > 3
					&& Sensors.VaporABV > Settings.ForesWarning))
			|| (Sensors.VaporABV >= 0 && CurrentWarning < 4
					&& Sensors.VaporABV < Settings.Warning[CurrentWarning]));

	switch (WarningStatusVapor) {
	case warningArmed:
		if (WarningCondition)
			WarningStatusVapor = warningTriggered;
		break;
	case warningTriggered:
		ShowingInfoScreen = false; // prevent infoscreen from hijacking warning
		screenWarningVapor();
		break;
	case warningSilenced:
		FlashBacklight = true;
		break;
	case warningAcknowledged:
		if (!WarningCondition)
			WarningStatusVapor = warningArmed;
		break;
	default:
		break;
	}

	FlashBacklight = (WarningStatusBoiler == warningTriggered
			|| WarningStatusBoiler == warningSilenced
			|| WarningStatusVapor == warningTriggered
			|| WarningStatusVapor == warningSilenced);

	if (!Silent
			&& (WarningStatusBoiler == warningTriggered
					|| WarningStatusVapor == warningTriggered))
		tone(pinBeeper, 440, 500);

	if (!FlashBacklight && WarningStatusVapor == warningAcknowledged) {
		lcd.setColor(RgbLcdKeyShieldI2C::clRed);
		return;
	}

	if (Silent)
		lcd.setColor(RgbLcdKeyShieldI2C::clBlue);
	else
		lcd.setColor(RgbLcdKeyShieldI2C::clGreen);
}

void handleAlarms() {
	if (Settings.BoilerPressureEnable
			&& (abs(Sensors.BoilerPressure) > Settings.MaxBoilerPressure))
		HealthAlarm = healthBoiler;

	else if (Settings.Vent1Enable
			&& (Sensors.Vent1.Type == NoSensor
					|| Sensors.Vent1.Temperature > Settings.VentAlarmTemperature
					|| (Sensors.Vent1.Temperature
							- Sensors.Vent1LastTemperature) * 100
							> Settings.VentAlarmDeltaTemperature))
		HealthAlarm = healthVent1;

	else if (Settings.Vent2Enable
			&& (Sensors.Vent2.Type == NoSensor
					|| Sensors.Vent2.Temperature > Settings.VentAlarmTemperature
					|| (Sensors.Vent2.Temperature
							- Sensors.Vent2LastTemperature) * 100
							> Settings.VentAlarmDeltaTemperature))
		HealthAlarm = healthVent2;

	else if (Settings.CoolantEnable
			&& (Sensors.Coolant.Type == NoSensor
					|| Sensors.Coolant.Temperature > Settings.MaxCoolant))
		HealthAlarm = healthCoolant;

	else if (AlarmStatusHealth == alarmAcknowledged)
		HealthAlarm = healthOk;

	switch (AlarmStatusHealth) {
	case alarmArmed:
		if (HealthAlarm != healthOk) {
			AlarmStatusHealth = alarmTriggered;
			ShowingInfoScreen = false; // prevent infoscreen from hijacking warning
			screenAlarm();
		}
		break;
	case alarmTriggered:
		FlashBacklight = true;
		tone(pinBeeper, 440, 500);
		break;
	case alarmSilenced:
		FlashBacklight = true;
		break;
	case alarmAcknowledged:
		FlashBacklight = false;
		if (HealthAlarm != healthOk)
			lcd.setColor(RgbLcdKeyShieldI2C::clRed);
		else {
			AlarmStatusHealth = alarmArmed;
			lcd.setColor(RgbLcdKeyShieldI2C::clWhite);
			}
		break;
	}

	if (HealthAlarm == healthOk) {
		handleWarnings();
	} else digitalWrite(pinHealthy, false);
}

void readSMT172() {
	if (Sensors.Vapor.Type == smt172) {
		switch (SMT172_T1::getStatus()) {
		case 0:
			break;
		case 1:
			Sensors.Vapor.Temperature = SMT172_T1::getTemperature()
					+ float(Settings.VaporOffset) / 100;
			Sensors.VaporABV = TtoVaporABV(Sensors.Vapor.Temperature,
							Sensors.BaroPressure);
			SMT172_T1::startTemperature(0.002);
			break;
		case 2:
			Sensors.Vapor.Type = NoSensor;
		}
	}
	if (AutoPageFastRefresh)
		AutoPageFastRefresh();
}

bool readDS18B20(temperatureSensor &sensor, int16_t offset,
		SingleDS18B20 &DS18B20Sensor) {
	if (sensor.Type == DS18B20) {
		if (DS18B20Sensor.read() && DS18B20Sensor.convert()) {
			sensor.Temperature = DS18B20Sensor.getTempAsC() + float(offset) / 100;
			return true;
		} else
			sensor.Type = NoSensor;
	}
	return false;
}

void readSensors() {
	// Retrieve the current pressure in Pascal.
	Sensors.BaroPressure = float(baro.getPressure()) / 100;
	// calculate the boiling point of water
	Sensors.H2OBoilingPoint = h2oBoilingPoint(Sensors.BaroPressure);

	if (Sensors.Vapor.Type == DS18B20) {
		if (readDS18B20(Sensors.Vapor, Settings.VaporOffset, VaporDS18B20))
			Sensors.VaporABV = TtoVaporABV(Sensors.Vapor.Temperature,
					Sensors.BaroPressure);
	}
	else if (Sensors.Vapor.Type == NoSensor)
			Sensors.Vapor.Type = initVaporSensor();


	if (readDS18B20(Sensors.Boiler, Settings.BoilerOffset, BoilerDS18B20))
		Sensors.BoilerABV = TtoLiquidABV(Sensors.Boiler.Temperature,
				Sensors.BaroPressure);
	else
		Sensors.Boiler.Type = initDS18B20(BoilerDS18B20);

/*
 * @5000mV n = 1024, 1kPa = 10.2 cm H2O
 * p = n * 5000 / 1024 / sensitivity * 10.2 -> p = n * 49.8 / sensitivity
 */
	if (Settings.BoilerPressureEnable) {
		Sensors.BoilerPressureRaw = analogRead(pinBoilerPressure); // 0->1024
		Sensors.BoilerPressure = (Sensors.BoilerPressureRaw
				- Settings.BoilerPressureOffset) * 49.8 / Settings.BoilerPressSens;
	}

	if (Settings.Vent1Enable) {
		Sensors.Vent1LastTemperature = Sensors.Vent1.Temperature;
		if (!readDS18B20(Sensors.Vent1, 0, Vent1DS18B20))
			Sensors.Vent1.Type = initDS18B20(Vent1DS18B20);
	}

	if (Settings.Vent2Enable) {
		Sensors.Vent2LastTemperature = Sensors.Vent2.Temperature;
		if (!readDS18B20(Sensors.Vent2, 0, Vent2DS18B20))
			Sensors.Vent2.Type = initDS18B20(Vent2DS18B20);
	}

	if (Settings.CoolantEnable)
		if (!readDS18B20(Sensors.Coolant, 0, CoolantDS18B20))
			Sensors.Coolant.Type = initDS18B20(CoolantDS18B20);

	if (AutoPageRefresh)
		AutoPageRefresh();

	if (IsLogging) writeToLog();
}


/*-----------------------------helper functions for display-------------------------------*/

void incDigit() {
	lcd.noCursor();
	uint8_t value = lcd.read();
	lcd.moveCursorLeft();
	if (value < (9 + 0x30))
		lcd.print(char(++value));
	else
		lcd.print('0');
	lcd.moveCursorLeft();
	lcd.cursor();
}

void decDigit() {
	lcd.noCursor();
	uint8_t value = lcd.read();
	lcd.moveCursorLeft();
	if (value > (0 + 0x30))
		lcd.print(char(--value));
	else
		lcd.print('9');
	lcd.moveCursorLeft();
	lcd.cursor();
}

void nextWarning() {
	if (CurrentWarning < 4) {
		CurrentWarning++;
	} else CurrentWarning = 0;
}

void prevWarning() {
	if (CurrentWarning > 0) {
		CurrentWarning--;
	} else CurrentWarning = 4;
}

void toggleWarnings() {
	if (WarningStatusVapor == warningDisabled) {
		if (Sensors.Boiler.Temperature < Settings.WarmedUpWarning)
			WarningStatusBoiler = warningArmed;
		WarningStatusVapor = warningArmed;
	} else {
		WarningStatusVapor = warningDisabled;
		WarningStatusBoiler = warningDisabled;
	}
}

void acknowledgeWarningVapor() {
	if (Silent || WarningStatusVapor == warningSilenced) {
		WarningStatusVapor = warningAcknowledged;
		ReturnScreen();
	}
	else WarningStatusVapor = warningSilenced;
}

void toggleSilent() {
	Silent = !Silent;
	if (Silent)
		infoScreen(msgSilent);
	else
		infoScreen(msgAudial);
}

void printVaporValues() {
	printTemp6(Sensors.Vapor);
	lcd.print(' ');
	if (Sensors.Vapor.Type == NoSensor || Sensors.VaporABV < Settings.ForesWarning)
		lcd.printP(msgNoValueABV);
	else if (Sensors.VaporABV < 0)
		lcd.printP(msgFores);
	else {
		lcd.print(dtostrf(Sensors.VaporABV, 4, 1, lineBuffer));
		lcd.print('%');
	}

	if (CurrentWarning == 4)
		lcd.printP(msgForesWarning);
	else
		lcd.print(dtostrf(Settings.Warning[CurrentWarning], 3, 0, lineBuffer));
}

void printBoilerValues() {
	printTemp6(Sensors.Boiler);
	lcd.print(' ');
	if (Sensors.WarmupTime > 0) {
		sprintf(lineBuffer, "%.2hd:%.2hd %.2hd", Sensors.WarmupTime / 60,
				Sensors.WarmupTime % 60, Settings.WarmedUpWarning);
		lcd.print(lineBuffer);
	}
	else if (Sensors.Boiler.Type == NoSensor || Sensors.BoilerABV < 0)
		lcd.printP(msgNoValueABV);
	else {
		lcd.print(dtostrf(Sensors.BoilerABV, 4, 1, lineBuffer));
		lcd.print('%');
	}
	for (int i = 0; i < 3; ++i) {
		lcd.print(' ');
	}
}

void zeroBoilerPressure() {
	Settings.BoilerPressureOffset = Sensors.BoilerPressureRaw;
	saveSettings();
	infoScreenSaved();
}

/*-----------------------------Screens--------------------------------------*/

void emptyScreen(void (*_returnScreen)()) {
	if (_returnScreen)
		ReturnScreen = _returnScreen;
	lcd.clear();
	lcd.noCursor();
	lcd.clearKeys();
	AutoPageRefresh = nullptr;
	AutoPageFastRefresh = nullptr;
}

void screenWarnings() {
	emptyScreen(screenWarnings);
	sprintf_P(lineBuffer, msgWarnings1, Settings.Warning[0], Settings.Warning[1]);
	lcd.print(lineBuffer);
	lcd.setCursor(0,1);
	sprintf_P(lineBuffer, msgWarnings2, Settings.Warning[2], Settings.Warning[3], Settings.WarmedUpWarning);
	lcd.print(lineBuffer);
	lcd.keyUp.onShortPress = screenGotoSettings;
	lcd.keyDown.onShortPress = screenMain;
	lcd.keyDown.onLongPress = toggleHealthPin;
	lcd.keySelect.onShortPress = keyRemapWarnings;
}

void screenMain() {
	emptyScreen(screenMain);
	lcd.keyUp.onShortPress = screenWarnings;
	lcd.keyRight.onShortPress = nextWarning;
	lcd.keyRight.onRepPress = nextWarning;
	lcd.keyLeft.onShortPress = prevWarning;
	lcd.keyLeft.onRepPress = prevWarning;
	lcd.keySelect.onShortPress = toggleWarnings;
	lcd.keySelect.onLongPress = toggleSilent;
	lcd.keyDown.onShortPress = screenCoolant;
	lcd.keyDown.onLongPress = toggleHealthPin;
	AutoPageFastRefresh = screenMainRefreshFast;
	AutoPageRefresh = screenMainRefresh;
	screenMainRefresh();
}

void screenCoolant() {
	emptyScreen(screenCoolant);
	lcd.keyUp.onShortPress = screenMain;
	lcd.keySelect.onLongPress = zeroBoilerPressure;
	lcd.keyDown.onShortPress = screenBaro;
	lcd.keyRight.onShortPress = toggleShowVent;
	lcd.keyLeft.onShortPress = toggleShowVent;
	lcd.keyDown.onLongPress = toggleHealthPin;
	AutoPageRefresh = showHealthRefresh;
	showHealthRefresh();
}

void screenBaro() {
	emptyScreen(screenBaro);
	lcd.keyUp.onShortPress = screenCoolant;
	lcd.keyDown.onShortPress = screenLogging;
	lcd.keyDown.onLongPress = toggleHealthPin;
	AutoPageRefresh = screenBaroRefresh;
	screenBaroRefresh();
}

#if LOG_TO_FLASHDRIVE == 1

void screenLogging() {
	emptyScreen(screenLogging);
	lcd.keyUp.onShortPress = screenBaro;;
	lcd.keyDown.onShortPress = screenGotoSettings;
	lcd.keyDown.onLongPress = toggleHealthPin;
	lcd.keySelect.onShortPress = toggleLogging;
	AutoPageRefresh = showLogStatus;
	showLogStatus();
}

void screenGotoSettings() {
	emptyScreen(screenGotoSettings);
	lcd.printP(msgSettings);
	lcd.keyUp.onShortPress = screenLogging;;
	lcd.keySelect.onShortPress = screenTime;
	lcd.keyDown.onShortPress = screenWarnings;
}

void screenTime() {
	emptyScreen(screenTime);
	lcd.keyUp.onShortPress = screenGotoMain;
	lcd.keyDown.onShortPress = screenOffsetVapor;
	lcd.keySelect.onShortPress = setTime;
	lcd.keyLeft.onShortPress = screenMain;
	screenTimeRefresh();
	AutoPageRefresh = screenTimeRefresh;
}

void screenOffsetVapor() {
	emptyScreen(screenOffsetVapor);
	lcd.printP(msgVaporOffset);
	printOffsetVapor();
	lcd.setCursor(15,0);
	lcd.print('C');
	lcd.keyUp.onShortPress = screenTime;
	lcd.keySelect.onShortPress = keyRemapOffset;
	lcd.keyDown.onShortPress = screenOffsetBoiler;
	lcd.keyLeft.onShortPress = screenMain;
	AutoPageFastRefresh = showOffsetVaporRefresh;
	OldTemp = Settings.VaporOffset;
	Offset = &Settings.VaporOffset;
	PrintOffset = printOffsetVapor;
}

#else

void screenLogging() {
	emptyScreen(screenLogging);
	lcd.keyUp.onShortPress = screenBaro;;
	lcd.keyDown.onShortPress = screenGotoSettings;
	lcd.keyDown.onLongPress = toggleHealthPin;
	lcd.keySelect.onShortPress = toggleLogging;
	lcd.keySelect.onLongPress = saveLogSettings;
	AutoPageFastRefresh = showLogStatus;
	showLogStatus();
}

void screenGotoSettings() {
	emptyScreen(screenGotoSettings);
	lcd.printP(msgSettings);
	lcd.keyUp.onShortPress = screenLogging;;
	lcd.keySelect.onShortPress = screenOffsetVapor;
	lcd.keyDown.onShortPress = screenWarnings;
}

void screenOffsetVapor() {
	emptyScreen(screenOffsetVapor);
	lcd.printP(msgVaporOffset);
	printOffsetVapor();
	lcd.setCursor(15,0);
	lcd.print('C');
	lcd.keyUp.onShortPress = screenGotoMain;
	lcd.keySelect.onShortPress = keyRemapOffset;
	lcd.keyDown.onShortPress = screenOffsetBoiler;
	lcd.keyLeft.onShortPress = screenMain;
	AutoPageFastRefresh = showOffsetVaporRefresh;
	OldTemp = Settings.VaporOffset;
	Offset = &Settings.VaporOffset;
	PrintOffset = printOffsetVapor;
}

#endif

void screenOffsetBoiler() {
	emptyScreen(screenOffsetBoiler);
	lcd.printP(msgBoilerOffset);
	printOffsetBoiler();
	lcd.setCursor(15,0);
	lcd.print('C');
	lcd.keyUp.onShortPress = screenOffsetVapor;
	lcd.keySelect.onShortPress = keyRemapOffset;
	lcd.keyDown.onShortPress = screenEnable;
	lcd.keyLeft.onShortPress = screenMain;
	AutoPageFastRefresh = showOffsetBoilerRefresh;;
	OldTemp = Settings.BoilerOffset;
	Offset = &Settings.BoilerOffset;
	PrintOffset = printOffsetBoiler;
}

void screenEnable() {
	emptyScreen(screenEnable);
	lcd.keyUp.onShortPress = screenOffsetBoiler;
	lcd.keyDown.onShortPress = screenSetCoolant;
	lcd.keySelect.onShortPress = keyRemapEnable;
	lcd.keyLeft.onShortPress = screenMain;
	printEnableScreen();
}

void screenSetCoolant() {
	emptyScreen(screenSetCoolant);
	lcd.keyUp.onShortPress = screenEnable;
	lcd.keyDown.onShortPress = screenSetPressureSensitivity;
	lcd.keySelect.onShortPress = keyRemapSetCoolant;
	lcd.keyLeft.onShortPress = screenMain;
	printSetCoolantScreen();
}


void screenSetPressureSensitivity() {
	emptyScreen(screenSetPressureSensitivity);
	lcd.keyUp.onShortPress = screenSetCoolant;
	lcd.keyDown.onShortPress = screenGotoMain;
	lcd.keySelect.onShortPress = keyRemapSetPressSens;
	lcd.keyLeft.onShortPress = screenMain;
	lcd.setCursor(0,0);
	lcd.printP(msgBlrPressSens1);
	screenSetPressureSensitivityRefresh();
}


#if LOG_TO_FLASHDRIVE == 1

void screenGotoMain() {
	emptyScreen(screenGotoMain);
	lcd.printP(msgMain);
	lcd.keyUp.onShortPress = screenSetPressureSensitivity;
	lcd.keySelect.onShortPress = screenMain;
	lcd.keyDown.onShortPress = screenTime;
	if (IsFirstRun) saveSettings();
}

#else

void screenGotoMain() {
	emptyScreen(screenGotoMain);
	lcd.printP(msgMain);
	lcd.keyUp.onShortPress = screenSetPressureSensitivity;
	lcd.keySelect.onShortPress = screenMain;
	lcd.keyDown.onShortPress = screenOffsetVapor;
	if (IsFirstRun) saveSettings();
}

#endif


/*----------------------------Refresh screens-------------------------------------*/

void screenMainRefresh() {
	if (Sensors.Vapor.Type != smt172) {
		lcd.setCursor(0, 0);
		printVaporValues();
	}
	lcd.setCursor(0, 1);
	printBoilerValues();
}

void screenMainRefreshFast() {
	if (Sensors.Vapor.Type == smt172) {
		lcd.setCursor(0, 0);
		printVaporValues();
	}
}

void screenBaroRefresh() {
	lcd.setCursor(0,0);
	lcd.printP(msgBaro);
	lcd.setCursor(8,0);
	lcd.print(dtostrf(Sensors.BaroPressure, 4, 0, lineBuffer));
	lcd.printP(msghPa);

	lcd.setCursor(0,1);
	lcd.printP(msgBPWater);
	lcd.setCursor(8,1);
	lcd.print(dtostrf(Sensors.H2OBoilingPoint, 6, 2, lineBuffer));
	lcd.print(' ');
	lcd.print('C');
}

#if LOG_TO_FLASHDRIVE == 1

void screenTimeRefresh() {
	rtc.readClock();
	lcd.setCursor(3,0);
	sprintf_P(lineBuffer, msgTime1, rtc.day, rtc.month, rtc.year );
	lcd.print(lineBuffer);
	lcd.setCursor(4,1);
	sprintf_P(lineBuffer, msgTime2, rtc.hour, rtc.minute, rtc.second);
	lcd.print(lineBuffer);
}

#endif

void screenSetPressureSensitivityRefresh() {
	lcd.setCursor(2,1);
	sprintf_P(lineBuffer, msgBlrPressSenSetting , Settings.BoilerPressSens);
	lcd.print(lineBuffer);
}

/*-------------------------------Info screens----------------------------------------*/

void infoScreenCanceled() {
	infoScreen(msgCanceled);
}

void infoScreenSaved() {
	infoScreen(msgSaved);
}

void screenWarningWarmedUp() {
	emptyScreen(nullptr);
	lcd.keySelect.onShortPress = acknowledgeWarningBoiler;
	lcd.printP(msgWarmedUp);
	AutoPageRefresh = screenWarningWarmedUpRefresh;
	screenWarningWarmedUpRefresh();
}

void screenWarningVapor() {
	emptyScreen(nullptr);
	lcd.keySelect.onShortPress = acknowledgeWarningVapor;
	AutoPageRefresh = screenWarningVaporRefresh;
	screenWarningVaporRefresh();
}

void screenWarningVaporRefresh() {
	if (CurrentWarning == 4) {
		lcd.setCursor(0, 0);
		lcd.printP(msgForesImminent);
		lcd.setCursor(0, 1);
		lcd.printP(msgVapor);
		printTemp6(Sensors.Vapor);
	} else {
		lcd.setCursor(0, 0);
		lcd.printP(msgLowABV);
		lcd.setCursor(0, 1);
		if (Sensors.Vapor.Type == NoSensor
				|| Sensors.VaporABV < Settings.ForesWarning)
			lcd.printP(msgNoValueABV);
		else if (Sensors.VaporABV < 0)
			lcd.printP(msgFores);
		else {
			lcd.print(dtostrf(Sensors.VaporABV, 4, 1, lineBuffer));
			lcd.print('%');
		};
		lcd.print(F(" < "));
		lcd.print(dtostrf(Settings.Warning[CurrentWarning], 2, 0, lineBuffer));
		lcd.print('%');
	}
}

void screenWarningWarmedUpRefresh() {
	lcd.setCursor(0, 1);
	lcd.printP(msgBoiler);
	printTemp6(Sensors.Boiler);
}

void acknowledgeWarningBoiler() {
	if (Silent || WarningStatusBoiler == warningSilenced) {
		WarningStatusBoiler = warningDisabled;
		ReturnScreen();
	}
	else WarningStatusBoiler = warningSilenced;
}

void screenAlarm() {
	emptyScreen(nullptr);
	lcd.keySelect.onShortPress = acknowledgeHealthAlarm;
	lcd.printP(msgAlarm);
	AutoPageRefresh = screenAlarmRefresh;
	screenAlarmRefresh();
}

void screenAlarmRefresh() {
	char floatString[10];
	lcd.setCursor(0, 1);
	switch (HealthAlarm) {
		case healthBoiler:
			dtostrf(Sensors.BoilerPressure, 5, 1, floatString);
			sprintf_P(lineBuffer, msgBoilerPressure, floatString);
			lcd.print(lineBuffer);
			break;
		case healthCoolant:
			lcd.printP(msgCoolant);
			lcd.setCursor(6, 1);
			printTemp4(Sensors.Coolant);
			break;
		case healthVent1:
			lcd.printP(msgVent1);
			printTemp6(Sensors.Vent1);
			break;
		case healthVent2:
			lcd.printP(msgVent2);
			printTemp6(Sensors.Vent2);
			break;
		default:

			break;
	}
}

void infoScreen(const char* msg1) {
	emptyScreen(nullptr);
	lcd.printP(msg1);
	ShowingInfoScreen = true;
	LastInfoScreenUpdate = millis();
}

void infoScreen(const char* msg1, const char* msg2) {
	emptyScreen(nullptr);
	lcd.printP(msg1);
	lcd.setCursor(0, 1);
	lcd.printP(msg2);
	ShowingInfoScreen = true;
	LastInfoScreenUpdate = millis();
}

uint16_t lcdGetDigits(uint8_t n) {
	uint16_t value = 0;
	while (n > 0) {
		value *= 10;
		value += (lcd.read() - 0x30);
		n--;
	}
	return value;
}

void keyRemapOffset() {
	lcd.setCursor(14,0);
	lcd.keyRight.onShortPress = nextDigitOffset;
	lcd.keyLeft.onShortPress = prevDigitOffset;
	lcd.keyUp.onShortPress = incDigitOffset;
	lcd.keyUp.onRepPress = incDigitOffset;
	lcd.keyDown.onShortPress = decDigitOffset;
	lcd.keyDown.onRepPress = decDigitOffset;
	lcd.keySelect.onShortPress = offsetCancel;
	lcd.keySelect.onLongPress = offsetSave;
}

void offsetCancel() {
	*Offset = OldTemp;
	infoScreenCanceled();
}

void offsetSave() {
	saveSettings();
	infoScreenSaved();
}

void printOffsetVapor() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	lcd.setCursor(9,0);
	lcd.print(dtostrf(float(Settings.VaporOffset) / 100, 6, 2, lineBuffer));
	lcd.setCursor(pos,0);
	lcd.cursor();
}

void showOffsetVaporRefresh() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	lcd.setCursor(0,1);
	printVaporValues();
	lcd.setCursor(pos,0);
	lcd.cursor();
}

void printOffsetBoiler() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	lcd.setCursor(9,0);
	lcd.print(dtostrf(float(Settings.BoilerOffset) / 100, 6, 2, lineBuffer));
	lcd.setCursor(pos,0);
	lcd.cursor();
}

void showOffsetBoilerRefresh() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	lcd.setCursor(0,1);
	printBoilerValues();
	lcd.setCursor(pos,0);
	lcd.cursor();
}

void nextDigitOffset() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 11:
		lcd.setCursor(13, 0);
		break;
	case 15:
		lcd.setCursor(10, 0);
		break;
	default:
		lcd.moveCursorRight();
		break;
	}
	lcd.cursor();
}

void prevDigitOffset() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 13:
		lcd.setCursor(11, 0);
		break;
	case 10:
		lcd.setCursor(15, 0);
		break;
	default:
		lcd.moveCursorLeft();
		break;
	}
	lcd.cursor();
}

void incDigitOffset() {
	uint8_t pos = lcd.getCursor();
	switch (pos) {
	case 10:
		*Offset += 1000;
		break;
	case 11:
		*Offset += 100;
		break;
	case 13:
		*Offset += 10;
		break;
	case 14:
		*Offset += 1;
		break;
	case 15:
		*Offset = 0;
		break;
	default:
		break;
	}
	*Offset = constrain(*Offset, -9999, 9999);
	PrintOffset();
}

void decDigitOffset() {
	uint8_t pos = lcd.getCursor();
	switch (pos) {
	case 10:
		*Offset -= 1000;
		break;
	case 11:
		*Offset -= 100;
		break;
	case 13:
		*Offset -= 10;
		break;
	case 14:
		*Offset -= 1;
		break;
	case 15:
		*Offset = 0;
		break;
	default:
		break;
	}
	*Offset = constrain(*Offset, -9999, 9999);
	PrintOffset();
}

void keyRemapWarnings() {
	lcd.setCursor(2,0);
	lcd.cursor();
	lcd.clearKeys();
	lcd.keyUp.onShortPress = incDigit;
	lcd.keyUp.onRepPress = incDigit;
	lcd.keyDown.onShortPress = decDigit;
	lcd.keyDown.onRepPress = decDigit;
	lcd.keyRight.onShortPress = nextDigitWarning;
	lcd.keyRight.onRepPress = nextDigitWarning;
	lcd.keyLeft.onShortPress = prevDigitWarning;
	lcd.keyLeft.onRepPress = prevDigitWarning;
	lcd.keySelect.onShortPress = infoScreenCanceled;
	lcd.keySelect.onLongPress = saveWarnings;
}

void nextDigitWarning() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 3:
		lcd.setCursor(8, 0);
		break;
	case 9:
		lcd.setCursor(2, 1);
		break;
	case 3 + 0x40:
		lcd.setCursor(8, 1);
		break;
	case 9 + 0x40:
		lcd.setCursor(13, 1);
		break;
	case 14 + 0x40:
		lcd.setCursor(2, 0);
		break;
	default:
		lcd.moveCursorRight();
		break;
	}
	lcd.cursor();
}

void prevDigitWarning() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 2:
		lcd.setCursor(14, 1);
		break;
	case 8:
		lcd.setCursor(3, 0);
		break;
	case 2 + 0x40:
		lcd.setCursor(9, 0);
		break;
	case 8 + 0x40:
		lcd.setCursor(3, 1);
		break;
	case 13 + 0x40:
		lcd.setCursor(9, 1);
		break;
	default:
		lcd.moveCursorLeft();
		break;
	}
	lcd.cursor();
}

void saveWarnings() {
	lcd.noCursor();

	lcd.setCursor(2,0);
	Settings.Warning[0] = lcdGetDigits(2);

	lcd.setCursor(8,0);
	Settings.Warning[1] = lcdGetDigits(2);

	lcd.setCursor(2,1);
	Settings.Warning[2] = lcdGetDigits(2);

	lcd.setCursor(8,1);
	Settings.Warning[3] = lcdGetDigits(2);

	lcd.setCursor(13,1);
	Settings.WarmedUpWarning = lcdGetDigits(2);

	saveSettings();
	infoScreenSaved();
}


void showHealthRefresh() {
	lcd.setCursor(0, 0);
	lcd.printP(msgCoolant);
	lcd.setCursor(6, 0);
	printTemp4(Sensors.Coolant);
	lcd.setCursor(0, 1);
	char floatString[] = " --.-";
	if (Settings.BoilerPressureEnable)
		dtostrf(Sensors.BoilerPressure, 5, 1, floatString);
	sprintf_P(lineBuffer, msgBoilerPressure, floatString);
	lcd.print(lineBuffer);
	lcd.setCursor(20, 0);
	lcd.printP(msgVent1);
	printTemp6(Sensors.Vent1);
	lcd.setCursor(20, 1);
	lcd.printP(msgVent2);
	printTemp6(Sensors.Vent2);
}

void toggleShowVent() {
	/*
	 * The internal screen is 40 char width. By placing the vent values
	 * midway at 20 one command shows one or the other.
	 */
	for (int i = 0; i < 20; ++i) {
		lcd.scrollDisplayRight();
	}
}

void toggleHealthPin() {
	if (digitalRead(pinHealthy))
		digitalWrite(pinHealthy, LOW);
	else digitalWrite(pinHealthy, HealthAlarm == healthOk);
}

void acknowledgeHealthAlarm() {
	switch (AlarmStatusHealth) {
		case alarmTriggered:
			AlarmStatusHealth = alarmSilenced;
			break;
		case alarmSilenced:
			AlarmStatusHealth = alarmAcknowledged;
			ReturnScreen();
			break;
		default:
			break;
	}
}


/*-------------------------------Logging-------------------------------------*/

#if LOG_TO_FLASHDRIVE == 1

void showLogStatus() {
	lcd.clear();
	if (flashDrive.getState() == flashDrive.mounted) {
		lcd.keySelect.onShortPress = toggleLogging;
		if (IsLogging) {
			lcd.setCursor(0, 0);
			lcd.print(Filename);
			lcd.setCursor(0, 1);
			lcd.printP(msgToStop);
		} else {
			lcd.setCursor(0, 0);
			lcd.printP(msgLogging);
			lcd.setCursor(0, 1);
			lcd.printP(msgToLog);
		}
	} else {
		lcd.setCursor(0, 0);
		lcd.printP(msgNoFlash);
		lcd.keySelect.onShortPress = nullptr;
	}
}

void toggleLogging() {
	if (IsLogging) {
		IsLogging = false;
	} else {
		if (createFile()) {
			StartTimeLog = millis();
			IsLogging = true;
		}
	}
	showLogStatus();
}

void writeToLog() {
	if (flashDrive.openFile(Filename, true) == USB_INT_SUCCESS) {
		flashDrive.print((millis() - StartTimeLog) / 1000, DEC);
		flashDrive.print(',');
		flashDrive.print(Sensors.BaroPressure, 0);
		flashDrive.print(',');
		if (Sensors.Vapor.Type != NoSensor)
			flashDrive.print(Sensors.Vapor.Temperature, 2);
		flashDrive.print(',');
		if (Sensors.VaporABV > 0)
			flashDrive.print(Sensors.VaporABV, 1);
		flashDrive.print(',');
		if (Sensors.Boiler.Type != NoSensor)
			flashDrive.print(Sensors.Boiler.Temperature, 2);
		flashDrive.print(',');
		if (Sensors.BoilerABV > 0)
			flashDrive.print(Sensors.BoilerABV, 1);
		flashDrive.print(',');
		if (Settings.Vent1Enable && Sensors.Vent1.Type != NoSensor)
			flashDrive.print(Sensors.Vent1.Temperature, 2);
		flashDrive.print(',');
		if (Settings.Vent2Enable && Sensors.Vent2.Type != NoSensor)
			flashDrive.print(Sensors.Vent2.Temperature, 2);
		flashDrive.print(',');
		if (Settings.CoolantEnable && Sensors.Coolant.Type != NoSensor)
			flashDrive.print(Sensors.Coolant.Temperature, 2);
		flashDrive.print(',');
		if (Settings.BoilerPressureEnable)
			flashDrive.print(Sensors.BoilerPressure, 1);
		flashDrive.print(',');
		switch (HealthAlarm) {
			case healthOk:
				flashDrive.print(F("Ok"));
				break;
			case healthBoiler:
				flashDrive.printP(msgBoilerP);
				break;
			case healthVent1:
				flashDrive.printP(msgVent1);
				break;
			case healthVent2:
				flashDrive.printP(msgVent2);
				break;
			case healthCoolant:
				flashDrive.printP(msgCoolant);
		}
		flashDrive.println();
		flashDrive.closeFile(true);
	} else IsLogging = false;
}

#else

void showLogStatus() {

	lcd.setCursor(0, 0);
	lcd.printP(msgLogging);
	char floatString[8];
	dtostrf(pgm_read_word(&baudrates[Settings.Baudrate]), 6, 0, floatString);
	lcd.setCursor(10, 0);
	lcd.print(floatString);
	lcd.setCursor(0, 1);
	if (IsLogging) {
		lcd.printP(msgToStop);
		lcd.keyLeft.onShortPress = nullptr;
		lcd.keyRight.onShortPress = nullptr;
	} else {
		lcd.printP(msgToLog);
		lcd.keyLeft.onShortPress = decBaudRate;
		lcd.keyRight.onShortPress = incBaudRate;
	}
}

void toggleLogging() {
	IsLogging = !IsLogging;
	if (IsLogging) {
		Serial.begin(pgm_read_word(&baudrates[Settings.Baudrate]));
		StartTimeLog = millis();
		SerialPrintP(msgLogHeader);
		Serial.println();
	} else Serial.end();
	showLogStatus();
}

void incBaudRate() {
	if (Settings.Baudrate == (sizeof(baudrates) / sizeof(baudrates[0]) -1))
		Settings.Baudrate = 0;
	else Settings.Baudrate++;
}

void decBaudRate() {
	if (Settings.Baudrate == 0)
		Settings.Baudrate = sizeof(baudrates) / sizeof(baudrates[0]) -1;
	else Settings.Baudrate--;
}

void saveLogSettings() {
	saveSettings();
	infoScreenSaved();
}

// helper function for serial to print strings from program memory
size_t SerialPrintP(const char str[]) {
	size_t n = 0;
	char c = pgm_read_byte(&str[n]);
	while (c) {
		Serial.print(c);
		c = pgm_read_byte(&str[++n]);
	};
	return n;
}

void writeToLog() {
		Serial.print((millis() - StartTimeLog) / 1000, DEC);
		Serial.print(',');
		Serial.print(Sensors.BaroPressure, 0);
		Serial.print(',');
		if (Sensors.Vapor.Type != NoSensor)
			Serial.print(Sensors.Vapor.Temperature, 2);
		Serial.print(',');
		if (Sensors.VaporABV > 0)
			Serial.print(Sensors.VaporABV, 1);
		Serial.print(',');
		if (Sensors.Boiler.Type != NoSensor)
			Serial.print(Sensors.Boiler.Temperature, 2);
		Serial.print(',');
		if (Sensors.BoilerABV > 0)
			Serial.print(Sensors.BoilerABV, 1);
		Serial.print(',');
		if (Settings.Vent1Enable && Sensors.Vent1.Type != NoSensor)
			Serial.print(Sensors.Vent1.Temperature, 2);
		Serial.print(',');
		if (Settings.Vent2Enable && Sensors.Vent2.Type != NoSensor)
			Serial.print(Sensors.Vent2.Temperature, 2);
		Serial.print(',');
		if (Settings.CoolantEnable && Sensors.Coolant.Type != NoSensor)
			Serial.print(Sensors.Coolant.Temperature, 2);
		Serial.print(',');
		if (Settings.BoilerPressureEnable)
			Serial.print(Sensors.BoilerPressure, 1);
		Serial.print(',');
		switch (HealthAlarm) {
			case healthOk:
				Serial.print(F("Ok"));
				break;
			case healthBoiler:
				SerialPrintP(msgBoilerP);
				break;
			case healthVent1:
				SerialPrintP(msgVent1);
				break;
			case healthVent2:
				SerialPrintP(msgVent2);
				break;
			case healthCoolant:
				SerialPrintP(msgCoolant);
		}
		Serial.println();
	}

#endif

void printEnabeledDisabeled(bool v) {
	if (v) lcd.print('E');
	else lcd.print('D');
}

void printEnableScreen() {
	lcd.printP(msgVent1);
	printEnabeledDisabeled(Settings.Vent1Enable);
	lcd.setCursor(8, 0);
	lcd.printP(msgVent2);
	printEnabeledDisabeled(Settings.Vent2Enable);
	lcd.setCursor(0, 1);
	lcd.printP(msgCoolant);
	printEnabeledDisabeled(Settings.CoolantEnable);
	lcd.setCursor(8, 1);
	lcd.printP(msgBoilerP);
	printEnabeledDisabeled(Settings.BoilerPressureEnable);
}

void keyRemapEnable() {
	lcd.setCursor(5,0);
	lcd.cursor();
	lcd.keyRight.onShortPress = nextEnable;
	lcd.keyLeft.onShortPress = prevEnable;
	lcd.keyUp.onShortPress = toggleEnable;
	lcd.keyDown.onShortPress = toggleEnable;
	lcd.keySelect.onShortPress = infoScreenCanceled;
	lcd.keySelect.onLongPress = saveEnable;
}

void saveEnable() {
	lcd.noCursor();

	lcd.setCursor(5,0);
	if (lcd.read() == 'E') {
		Settings.Vent1Enable = true;
	} else {
		Settings.Vent1Enable = false;
		Sensors.Vent1.Type = NoSensor;
	}

	lcd.setCursor(13,0);
	if (lcd.read() == 'E') {
		Settings.Vent2Enable = true;
	} else {
		Settings.Vent2Enable = false;
		Sensors.Vent2.Type = NoSensor;
	}

	lcd.setCursor(5,1);

	if (lcd.read() == 'E') {
		Settings.CoolantEnable = true;
	} else {
		Settings.CoolantEnable = false;
		Sensors.Coolant.Type = NoSensor;
	}

	lcd.setCursor(13,1);
	Settings.BoilerPressureEnable = lcd.read() == 'E';

	saveSettings();

	infoScreenSaved();
}

void toggleEnable() {
	lcd.noCursor();
	uint8_t value = lcd.read();
	lcd.moveCursorLeft();
	if (value == 'E') lcd.print('D');
			else lcd.print('E');
	lcd.moveCursorLeft();
	lcd.cursor();
}

void nextEnable() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 5:
		lcd.setCursor(13, 0);
		break;
	case 13:
		lcd.setCursor(5, 1);
		break;
	case 5 + 0x40:
		lcd.setCursor(13, 1);
		break;
	default:
		lcd.setCursor(5, 0);
		break;
	}
	lcd.cursor();
}

void prevEnable() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 5:
		lcd.setCursor(13, 1);
		break;
	case 13 + 0x40:
		lcd.setCursor(5, 1);
		break;
	case 5 + 0x40:
		lcd.setCursor(13, 0);
		break;
	default:
		lcd.setCursor(5, 0);
		break;
	}
	lcd.cursor();
}

void keyRemapSetCoolant() {
	lcd.setCursor(8,0);
	lcd.cursor();
	lcd.keyRight.onShortPress = nextDigitCoolant;
	lcd.keyLeft.onShortPress = prevDigitCoolant;
	lcd.keyUp.onShortPress =  incDigit;
	lcd.keyDown.onShortPress = decDigit;
	lcd.keySelect.onShortPress = infoScreenCanceled;
	lcd.keySelect.onLongPress = saveCoolant;
}

void printSetCoolantScreen() {
	sprintf_P(lineBuffer, msgCoolantSetting, Settings.MaxCoolant);
	lcd.print(lineBuffer);
	lcd.setCursor(0,1);
	sprintf_P(lineBuffer, msgBoilerPressureSetting, Settings.MaxBoilerPressure);
	lcd.print(lineBuffer);
}

void nextDigitCoolant() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 8:
		lcd.setCursor(9, 0);
		break;
	case 9:
		lcd.setCursor(8, 1);
		break;
	case 8 + 0x40:
		lcd.setCursor(9, 1);
		break;
	default:
		lcd.setCursor(8, 0);
		break;
	}
	lcd.cursor();
}

void prevDigitCoolant() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 8:
		lcd.setCursor(9, 1);
		break;
	case 9 + 0x40:
		lcd.setCursor(8, 1);
		break;
	case 8 + 0x40:
		lcd.setCursor(9, 0);
		break;
	default:
		lcd.setCursor(8, 0);
		break;
	}
	lcd.cursor();
}

void saveCoolant() {
	lcd.noCursor();
	lcd.setCursor(8,0);
	Settings.MaxCoolant = lcdGetDigits(2);
	lcd.setCursor(8,1);
	Settings.MaxBoilerPressure = lcdGetDigits(2);
	saveSettings();
	infoScreenSaved();
}

void keyRemapSetPressSens() {
	lcd.setCursor(2,1);
	lcd.cursor();
	lcd.keyLeft.onShortPress = prevDigitPressSens;
	lcd.keyRight.onShortPress = nextDigitPressSens;
	lcd.keyUp.onShortPress =  incDigit;
	lcd.keyUp.onRepPress = incDigit;
	lcd.keyDown.onShortPress = decDigit;
	lcd.keyDown.onRepPress = decDigit;
	lcd.keySelect.onShortPress = infoScreenCanceled;
	lcd.keySelect.onLongPress = savePressSens;
}

void nextDigitPressSens() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 5 + 0x40:
		lcd.setCursor(1, 2);
		break;
	default:
		lcd.moveCursorRight();
		break;
	}
	lcd.cursor();
}


void prevDigitPressSens() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 2 + 0x40:
		lcd.setCursor(5, 1);
		break;
	default:
		lcd.moveCursorLeft();
		break;
	}
	lcd.cursor();
}


void savePressSens() {
	lcd.setCursor(2, 1);
	Settings.BoilerPressSens = lcdGetDigits(4);
	Settings.BoilerPressSens = constrain(Settings.BoilerPressSens, 1, 9999);
	saveSettings();
	infoScreenSaved();
}

#if LOG_TO_FLASHDRIVE == 1
void setTime() {
	emptyScreen(nullptr);
	sprintf_P(lineBuffer, msgSetTime1, rtc.year, rtc.month, rtc.day);
	lcd.print(lineBuffer);
	lcd.setCursor(0,1);
	sprintf_P(lineBuffer, msgSetTime2, rtc.hour, rtc.minute, rtc.second);
	lcd.print(lineBuffer);
	lcd.setCursor(8,0);
	lcd.cursor();
	lcd.clearKeys();
	lcd.keyUp.onShortPress = incDigit;
	lcd.keyUp.onRepPress = incDigit;
	lcd.keyDown.onShortPress = decDigit;
	lcd.keyDown.onRepPress = decDigit;
	lcd.keyRight.onShortPress = nextDigitTime;
	lcd.keyRight.onRepPress = nextDigitTime;
	lcd.keyLeft.onShortPress = prevDigitTime;
	lcd.keyLeft.onRepPress = prevDigitTime;
	lcd.keySelect.onShortPress = infoScreenCanceled;
	lcd.keySelect.onLongPress = saveTime;
}

void nextDigitTime() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 9:
	case 12:
	case 9 + 0x40:
	case 12 + 0x40:
		lcd.setCursor(pos + 2, 0);
		break;
	case 15:
		lcd.setCursor(8, 1);
		break;
	case 15 + 0x40:
		lcd.setCursor(8, 0);
		break;
	default:
		lcd.moveCursorRight();
		break;
	}
	lcd.cursor();
}

void prevDigitTime() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 11:
	case 14:
	case 11 + 0x40:
	case 14 + 0x40:
		lcd.setCursor(pos - 2, 0);
		break;
	case 8:
		lcd.setCursor(15, 1);
		break;
	case 8 + 0x40:
		lcd.setCursor(15, 0);
		break;
	default:
		lcd.moveCursorLeft();
		break;
	}
	lcd.cursor();
}

void saveTime() {
	lcd.noCursor();

	lcd.setCursor(8,0);
	rtc.year = lcdGetDigits(2);

	lcd.setCursor(11,0);
	rtc.month = lcdGetDigits(2);

	lcd.setCursor(14,0);
	rtc.day = lcdGetDigits(2);

	lcd.setCursor(8,1);
	rtc.hour = lcdGetDigits(2);

	lcd.setCursor(11,1);
	rtc.minute = lcdGetDigits(2);

	lcd.setCursor(14,1);
	rtc.second = lcdGetDigits(2);

	rtc.setClock();
	saveSettings();
	infoScreenSaved();
}

#endif
