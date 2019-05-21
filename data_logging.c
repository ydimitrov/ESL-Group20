bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);

static uint32_t address = 0;



- es system time (control loop time in ms)
- system mode
- joystick/keyboard data
- actuator data (motor values)
- response time
- sensor data (phi, psi, theta)
- angle readings




- LITTLE ENDIAN 



- telemtry (done with printf)
- rotor speed
- battery




void eightTo32bit(){

}

uint8_t *data;

uint8_t writeLog(uint8_t* data, uint8_t size){

	if (address >= 0x01FFFF)
		return;

	if (address + size <= 0x01FFFF)
		flash_write_bytes(address, data, size);
		address += size;
}



void readLog(){
	
	

}