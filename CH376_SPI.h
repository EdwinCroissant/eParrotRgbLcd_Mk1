/*
 * CH376_SPI.h
 *
 *  Created on: 19 feb. 2021
 *      Author: Ed
 */

#ifndef CH376_SPI_H
#define CH376_SPI_H

#include <inttypes.h>
#include <Arduino.h>
#include <SPI.h>
#include "CH376_CMD.h"

#define CH376_SPI_SETTINGS SPISettings(14000000, MSBFIRST, SPI_MODE0)

#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))

class CH376_SPI: public Print {
public:
	enum CH376_States: uint8_t {
		error,
		detached,
		attached,
		mounted,
		mountfailure
	} ;

	CH376_SPI(uint8_t CSPin, uint8_t intPin, uint8_t bussyPin);
//	virtual ~CH376_SPI(); //destructor
	using Print::write; // pull in write(str) and write(buf, size) from Print
	virtual size_t write(uint8_t c);
	size_t write(const uint8_t *buffer, size_t size) override;
#ifdef __AVR__
	size_t printP(const char str[]);
#endif // __AVR__
	bool init();
	void reset();
	uint8_t getIcVersion();
	bool checkDevice();
	uint8_t processMessages();
	void (*onStateChange)(uint8_t);
	CH376_States getState();
	uint8_t getStatus();
	void setUSBMode(uint8_t mode);
	void mountDisk();
	uint8_t openFile(const char* filename, bool append = true);
	void setFilename(const char* filename);
	uint8_t createFile(const char* filename);
	bool fileExists(const char* filename);
	uint8_t closeFile(bool update = true);
	uint8_t setCursor(uint32_t position);
	int16_t writeFile(const char* buffer, uint16_t length);
	uint8_t setTimeStamp(uint8_t yy, uint8_t mm, uint8_t dd, uint8_t hh, uint8_t mi, uint8_t ss);


private:
	struct pinDef {
			volatile uint8_t *reg;
			uint8_t mask;
		} 	_spiSelect, _intPin, _bussyPin;
	CH376_States _state;
	uint8_t _mountAttemps;

};

#endif /* CH376_SPI_H */
