/*
 * CH376_CMD.h
 *
 *  Created on: 20 feb. 2021
 *      Author: Ed
 */

#ifndef CH376_CMD_H_
#define CH376_CMD_H_

#define	CMD_GET_IC_VER		0x01	// Obtain chip and firmware version number
#define	CMD_SET_BAUDRATE	0x02	// Set serial communication baud rate
#define	CMD_ENTER_SLEEP		0x03	// Go to low-power
#define	CMD_RESET_ALL		0x05	// Execute hardware reset
#define	CMD_CHECK_EXIST		0x06	// Test communication interface and working status
#define	CMD_SET_SD0_INT		0x0B	// Set interrupt Mode of SD0 in SPI
#define	CMD_GET_FILE_SIZE	0x0C	// Get the current file length
#define	CMD_SET_USB_MODE	0x15	// Configure the USB work mode
#define	CMD_GET_STATUS		0x22	// Get interruption status and cancel requirement
#define	CMD_RD_USB_DATA0	0x27	// Read data from current interrupt port buffer of USB or receive buffer of host port
#define	CMD_WR_USB_DATA		0x2C	// Write data to transfer buffer of USB host
#define	CMD_WR_REQ_DATA		0x2D	// Write requested data block to internal appointed buffer
#define	CMD_WR_OFS_DATA		0x2E	// Write data block to internal buffer with appointed excursion address
#define	CMD_SET_FILE_NAME	0x2F	// Set the file name which will be operated
#define	CMD_DISK_CONNECT	0x30	// Check the disk connection status
#define	CMD_DISK_MOUNT		0x31	// Initialize disk and test disk ready
#define	CMD_FILE_OPEN		0x32	// Open file or catalog, enumerate file and catalog
#define	CMD_FILE_ENUM_GO	0x33	// Go on to enumerate file and catalog
#define	CMD_FILE_CREATE		0x34	// Create file
#define	CMD_FILE_ERASE		0x35	// Delete file
#define	CMD_FILE_CLOSE		0x36	// Close the open file or catalog
#define	CMD_DIR_INFO_READ	0x37	// Read the catalog information of file
#define	CMD_DIR_INFO_SAVE	0x38	// Save catalog information of file
#define	CMD_BYTE_LOCATE		0x39	// Move the file pointer take byte as unit
#define	CMD_BYTE_READ		0x3A	// Read data block from current location take byte as unit
#define	CMD_BYTE_RD_GO		0x3B	// Continue byte read
#define	CMD_BYTE_WRITE		0x3C	// Write data block from current location take unit as unit
#define	CMD_BYTE_WR_GO		0x3D	// Continue byte write
#define	CMD_DISK_CAPACITY	0x3E	// Check disk physical capacity
#define	CMD_DISK_QUERY		0x3F	// Check disk space
#define	CMD_DIR_CREATE		0x40	// Create catalog and open or open the existed catalog
#define	CMD_SEC_LOCATE		0x4A	// Move file pointer from current location take sector as unit
#define	CMD_SEC_READ		0x4B	// Read data block from current location take sector as unit
#define	CMD_SEC_WRITE		0x4C	// Write data block from current location take sector as unit
#define	CMD_DISK_BOC_CMD	0x50	// Execute B0 transfer protocol command to USB Storage
#define	CMD_DISK_READ		0x54	// Read physical sector from USB storage device
#define	CMD_DISK_RD_GO		0x55	// Go on reading operation of USB storage device
#define	CMD_DISK_WRITE		0x56	// Write physical sector to USB storage device
#define	CMD_DISK_WR_GO		0x57	// Go on writing operation of USB storage device
#define CMD_RET_SUCCESS		0x51	// Operation successful
#define CMD_RET_ABORT		0x5F	// Operation failure

#define USB_INT_SUCCESS		0x14	// USB transaction or transfer operation was successful
#define USB_INT_CONNECT		0x15	// USB device connection event detected, may be a new connection or reconnect after disconnecting
#define USB_INT_DISCONNECT	0x16	// USB device disconnect event detected
#define USB_INT_BUF_OVER	0x17	// USB data transmission error or too much data buffer overflow
#define USB_INT_USB_READY	0x18	// USB device has been initialized (USB address assigned)
#define USB_INT_DISK_READ	0x1D	// USB memory request data read out
#define USB_INT_DISK_WRITE	0x1E	// USB memory request data write
#define USB_INT_DISK_ERR	0x1F	// USB memory operation failed

#define	ERR_OPEN_DIR		0x41	// Open directory address is appointed
#define	ERR_MISS_FILE		0x42	// File doesn�t be found which address is appointed, maybe the name is error
#define	ERR_FOUND_NAME		0x43	// Search suited file name, or open directory according the request, open file in actual
#define	ERR_DISK_DISCON		0x82	// Disk doesn�t connect, maybe the disk has cut down
#define	ERR_LARGE_SECTOR	0x84	// Sector is too big, only support 512 bytes
#define	ERR_TYPE_ERROR		0x92	// Disk partition doesn�t support, re-prartition by tool
#define	ERR_BPB_ERROR		0xA1	// Disk doesn�t format, or parameter is errot, re-formate by WINDOWS with default parameter
#define	ERR_DISK_FULL		0xB1	// File in disk is full, spare space is too small
#define	ERR_FDT_OVER		0xB2	// Many file in directory, no spare directory, clean up the disk, less than 512 in FAT12/FAT16 root directory
#define	ERR_FILE_CLOSE		0xB4	// File is closed, re-open file if need

#define MODE_HOST_1			0x01	// switch to valid USB-DEVICE, peripheral firmware mode. (not support serial connection)
#define MODE_HOST_2			0x02	// switch to valid USB-DEVICE, inner firmware mode.
#define MODE_HOST_3			0X03	// means switch to SD host mode, manage and storage/get file in SD card.
#define MODE_HOST_4			0x04	// switch to invalid USB-HOST mode.
#define MODE_HOST_5			0x05	// switch to valid USB-HOST, non-generate SOF package.
#define MODE_HOST_6			0x06	// switch to valid USB-HOST, produce SOF package automatically.
#define MODE_HOST_7			0x07	// means switch to valid USB-HOST, and reset USB bus.




#endif /* CH376_CMD_H_ */
