#include "data_logging.h"

bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);

static uint32_t address = 0;
int dumpAddr = 0x00000000;
uint8_t buffer[64]; // Initialize read buffer
int txCount;

/**
 * @author Thomas Makryniotis
 * 
 * Writes a log 'row' in the SPI Flash. In our case it is usually 28 bytes.
 * 
 * @param data pointer to uint8_t type array containing data.
 * @param size number of bytes to be stored.
 *
 * @return
 * @retval true if operation is successful.
 * @retval false if operation is failed.
 */

bool writeLog(uint8_t* data, uint8_t size) {

	// Check if the address is valid

	int ret;
	
	// Address is invalid
	if(address >= 0x01FFFF){
		//printf("Invalid address\n");
		ret = false;
	}

	// Address is valid and size is also good
	if(address + size <= 0x01FFFF) {
		flash_write_bytes(address, data, size);
		address += size;
		//printf("Current address -> %ld \n", address);
		ret = true;
	
	// Address is valid but adding with size goes out of bounds
	} else if (address + size >= 0x01FFFF) {
		
		// Erase memory
		flash_chip_erase();
		// Reset address
		address = 0;
		// Write new log line
		flash_write_bytes(address, data, size);
		address += size;

		ret = true;
	}
	
	return ret;

}

/**
 * @author Thomas Makryniotis
 * 
 * Reads data from the SPI Flash and transmits them over serial in 64-byte chunks.
 *
 * @return
 * @retval true if operation is successful (aka, all data was successfully read from memory).
 * @retval false if operation is failed.
 */

bool dumpLog(void){

	printf("Dumping SPI Flash contents...\n");

	bool success = true;

	// We send data in 64-byte chunks

	// Signal dump of data by sending two 0xAA bytes
	uart_put(0xAA);
	uart_put(0xAA);

	while(dumpAddr < 0xFFFFFFFF){		

		// Read 64 bytes from the SPI Flash
		bool fread = flash_read_bytes(dumpAddr, &buffer[0], 64);

		// Send 64 bytes with uart_put()
		for(txCount = 0; txCount < 63; txCount++){
			uart_put(buffer[txCount]);
		}

		// Read from next offset address
		dumpAddr += 64;
		// Update error
		success = success & fread;
		
	}

	// Erase memory
	flash_chip_erase();

	return success;
}

// TODO: Maybe we need a readLog() function?