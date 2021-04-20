# STM32H7 Example. I2C EEPROM.

This example includes simple communication with external EEPROM on I2C bus. Example written for AT24C08C EEPROM (16 byte
page, 64 pages total which gives 1024 bytes) but should work with others with some modifications. I2C bus is set to 1MHz
frequency since AT24C08C supports it with VCC >= 2.7V

Tested with STM32H743ZI.
