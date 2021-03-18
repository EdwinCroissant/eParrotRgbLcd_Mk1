/*
 * CH376_SPI.cpp
 *
 *  Created on: 19 feb. 2021
 *      Author: Ed
 */
#include "CH376_SPI.h"
#include "CH376_CMD.h"

CH376_SPI::CH376_SPI(uint8_t CSPin, uint8_t intPin, uint8_t bussyPin){
	_spiSelect.reg = PIN_TO_BASEREG(CSPin);
	_spiSelect.mask = PIN_TO_BITMASK(CSPin);
	_intPin.reg = PIN_TO_BASEREG(intPin);
	_intPin.mask = PIN_TO_BITMASK(intPin);
	_bussyPin.reg = PIN_TO_BASEREG(bussyPin);
	_bussyPin.mask = PIN_TO_BITMASK(bussyPin);
	_state = error;
	_mountAttemps = 0;
	onStateChange = nullptr;
}


bool CH376_SPI::init() {
	delay(100); //wait for VCC to normalize
	DIRECT_MODE_INPUT(_intPin.reg, _intPin.mask);
	DIRECT_WRITE_HIGH(_intPin.reg, _intPin.mask); // enable pullup interruptpin

	DIRECT_MODE_OUTPUT(_spiSelect.reg, _spiSelect.mask);
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask); // CS pin output and high
	SPI.begin();
	SPI.beginTransaction(CH376_SPI_SETTINGS);

	CH376_SPI::reset();

	CH376_SPI::setUSBMode(MODE_HOST_6);

	return CH376_SPI::checkDevice();
}

size_t CH376_SPI::write(uint8_t c) {
	const char _c = c;
	writeFile(&_c, 1);
	return 1;
}

size_t CH376_SPI::write(const uint8_t *buffer, size_t size) {
	return writeFile((char *) buffer, size);
}

void CH376_SPI::reset() {
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_RESET_ALL);
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	delay(100);// wait until reset command is done.
	}

uint8_t CH376_SPI::getIcVersion() {
	uint8_t tmp;
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_GET_IC_VER);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	tmp = SPI.transfer(0x00);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	return tmp;
}

bool CH376_SPI::checkDevice() {
	uint8_t tmp;
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_CHECK_EXIST);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(0xF0);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	tmp = SPI.transfer(0);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	return (tmp == (uint8_t) ~0xF0);
}

uint8_t CH376_SPI::processMessages() {
	uint8_t msg = 0;
	if (!DIRECT_READ(_intPin.reg, _intPin.mask)) { // interrupt pin is low
		msg = getStatus();
		switch (msg) {
			case USB_INT_DISCONNECT:
				_state = detached;
				if (onStateChange)
					onStateChange(detached);
				msg = 0;
				break;
			case USB_INT_CONNECT:
				_state = attached;
				if (onStateChange)
					onStateChange(attached);
				_mountAttemps = 0;
				mountDisk();
				msg = 0;
				break;
			case USB_INT_SUCCESS:
				if (_state == attached) {
						_state = mounted;
					if (onStateChange)
						onStateChange(mounted);
					msg = 0;
				}
				break;
			case USB_INT_DISK_ERR:
				if (_state == attached) {
					if (_mountAttemps < 5) {
						mountDisk();
						_mountAttemps++;
					} else {
						_state = mountfailure;
						if (onStateChange)
							onStateChange(mountfailure);
					}
					msg = 0;
				}
				break;
			default:
				break;
		}
	}
	return msg; // unexpected message if !0
}


uint8_t CH376_SPI::getStatus() {
	uint8_t tmp;
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_GET_STATUS);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	tmp = SPI.transfer(0x00);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	return tmp;
}

void CH376_SPI::setUSBMode(uint8_t mode) {
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_SET_USB_MODE);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(mode);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
}

void CH376_SPI::mountDisk() {
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_DISK_MOUNT);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
}

CH376_SPI::CH376_States CH376_SPI::getState() {
	return _state;
}

uint8_t CH376_SPI::openFile(const char *filename, bool append) {
	uint8_t msg;
	setFilename(filename);
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_FILE_OPEN);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	msg = getStatus();
	if ((msg == USB_INT_SUCCESS) && append)
		setCursor(0xFFFFFFFF);
	return msg;
}

void CH376_SPI::setFilename(const char *filename) {
	uint8_t n = 0;
	char c;
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_SET_FILE_NAME);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	while(1) {
		c = filename[n++];
		if (c == 0 || n > 14) break;
		SPI.transfer(c);
		while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		}
	SPI.transfer(0x00);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
}

uint8_t CH376_SPI::createFile(const char *filename) {
	setFilename(filename);
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_FILE_CREATE);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	return getStatus();
}

bool CH376_SPI::fileExists(const char *filename) {
	setFilename(filename);
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_FILE_OPEN);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	if (getStatus() == USB_INT_SUCCESS) {
		closeFile(false);
		return true;
	}
	else return false;
}

uint8_t CH376_SPI::closeFile(bool update) {
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_FILE_CLOSE);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(update ? 0x01 : 0x00);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	return getStatus();
}

uint8_t CH376_SPI::setCursor(uint32_t position) {
	union union32bits {
	  uint32_t num;
	  uint8_t arr[sizeof(uint32_t)];
	} _pos;
	_pos.num = position;

	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_BYTE_LOCATE);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	for (uint8_t i = 0; i < sizeof(uint32_t); ++i) {
		SPI.transfer(_pos.arr[i]);
		while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	}
	return getStatus();
}

int16_t CH376_SPI::writeFile(const char *buffer, uint16_t length) {
	uint16_t count = 0;
	uint8_t requested;
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_BYTE_WRITE);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(lowByte(length));
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(highByte(length));
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);

	while (DIRECT_READ(_intPin.reg, _intPin.mask)); // wait for interrupt
	while (getStatus() == USB_INT_DISK_WRITE) {
		SPI.beginTransaction(CH376_SPI_SETTINGS);
		DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
		SPI.transfer(CMD_WR_REQ_DATA);
		while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		requested = SPI.transfer(0x00);
		while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		while (requested > 0) {
			SPI.transfer(buffer[count]);
			requested--;
			count++;
			while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		};
		DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
		DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
		SPI.transfer(CMD_BYTE_WR_GO);
		while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
		while (DIRECT_READ(_intPin.reg, _intPin.mask)); // wait for interrupt
	}
	return getStatus() == USB_INT_SUCCESS ?  count: 0;
}

uint8_t CH376_SPI::setTimeStamp(uint8_t yy, uint8_t mm, uint8_t dd, uint8_t hh,
		uint8_t mi, uint8_t ss) {

	uint8_t _status;
	union union32bits {
	  uint32_t as32;
	  uint8_t arr[sizeof(uint32_t)];
	} _fatDateTime;

	/*<------- 0x19 --------> <------- 0x18 --------> <------- 0x17 --------> <------- 0x16 -------->
	 *15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
	 * y  y  y  y  y  y  y  m  m  m  m  d  d  d  d  d  h  h  h  h  h  m  m  m  m  m  m  s  s  s  s  s
	 */
	uint32_t temp = 20 + yy;
	_fatDateTime.as32 = (temp << 25);		//Year
	temp = mm;
	_fatDateTime.as32 |= (temp << 21);		//Month
	temp = dd;
	_fatDateTime.as32 |= (temp << 16);		//Day
	temp = hh;
	_fatDateTime.as32 |= (temp << 11);		//Hour
	temp = mi;
	_fatDateTime.as32 |= (temp << 5);		//Minute
	temp = ss;
	_fatDateTime.as32 |= (temp >> 1);		//Second

	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_DIR_INFO_READ);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(0xFF);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);

	while (DIRECT_READ(_intPin.reg, _intPin.mask)); // wait for interrupt
	_status = getStatus();
	if (_status == USB_INT_SUCCESS) {
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_WR_OFS_DATA);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(0x16); //22
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(sizeof(uint32_t));
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(_fatDateTime.arr[0]);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(_fatDateTime.arr[1]);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(_fatDateTime.arr[2]);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(_fatDateTime.arr[3]);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);

	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_DIR_INFO_SAVE);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
	while (DIRECT_READ(_intPin.reg, _intPin.mask)); // wait for interrupt
	return getStatus();
	} else return _status;
}

#ifdef __AVR__
/*
 * Writes a string in program memory to the flashdrive
 */
size_t CH376_SPI::printP(const char str[]) {
	uint16_t length = 0;
	uint16_t count = 0;
	uint8_t requested;
	char c = pgm_read_byte(&str[length]);
	while (c) {
		c = pgm_read_byte(&str[++length]);
	};
	DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
	SPI.transfer(CMD_BYTE_WRITE);
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(lowByte(length));
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	SPI.transfer(highByte(length));
	while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
	DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);

	while (DIRECT_READ(_intPin.reg, _intPin.mask)); // wait for interrupt
	while (getStatus() == USB_INT_DISK_WRITE) {
		DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
		SPI.transfer(CMD_WR_REQ_DATA);
		while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		requested = SPI.transfer(0x00);
		while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		while (requested > 0) {
			SPI.transfer(pgm_read_byte(&str[count]));
			requested--;
			count++;
			while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		};
		DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
		DIRECT_WRITE_LOW(_spiSelect.reg, _spiSelect.mask);
		SPI.transfer(CMD_BYTE_WR_GO);
		while(DIRECT_READ(_bussyPin.reg, _bussyPin.mask));
		DIRECT_WRITE_HIGH(_spiSelect.reg, _spiSelect.mask);
		while (DIRECT_READ(_intPin.reg, _intPin.mask)); // wait for interrupt
	}
	return getStatus() == USB_INT_SUCCESS ?  count: 0;
}

#endif // __AVR__
