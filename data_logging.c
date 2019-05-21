bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);


- es system time (control loop time in ms)
- system mode
- joystick/keyboard data
- actuator data (motor values)
- response time
- sensor data (phi, psi, theta)
- angle readings




- LITTLE ENDIAN 

You need to count how much has been storred in the flash to avoid overflow


- telemtry (done with printf)
- rotor speed
- battery




eightTo32bit()

uint8_t *data;

void write_log(uint8_t* data, uint8_t size)
{

	static bool isfull = false;
	static uint32_t address = 0;
	if (isfull)
		return;

	flash_write_bytes(data, size);
	address += size;
	if (address >= 0x01FFFF)
	{
		isfull = true;
	}
}



void read_log()
{
	static uint32_t address = 0;

}