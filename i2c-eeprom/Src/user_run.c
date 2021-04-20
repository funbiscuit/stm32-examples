#include "user_run.h"

#include <stdbool.h>
#include <string.h>

#include "stm32h7xx_hal.h"

#define EEPROM_PAGE_SIZE 16
// delay in ms required for eeprom finish its writes
#define EEPROM_WRITE_DELAY 6

extern I2C_HandleTypeDef hi2c2;

static uint8_t buf[1024];

static void randomWrite(uint16_t address, uint8_t data);

static void randomPageWrite(uint16_t address, const uint8_t *data, uint8_t dataLen);

static void randomWriteArray(uint16_t address, const uint8_t *data, uint16_t dataLen);

static uint8_t randomRead(uint16_t address);

static void randomReadArray(uint16_t address, uint8_t *data, uint16_t dataLen);

static uint8_t currentAddressRead(void);

_Noreturn void userRun(void)
{
    memset(buf, 0, 1024);
    // clear data in eeprom
    randomWriteArray(0x00, buf, 1024);

    // read all data
    randomReadArray(0x00, buf, 1024);

    // write big array eeprom
    for (int i = 0; i < 512; ++i)
        buf[i] = i;
    randomWriteArray(234, buf, 512);

    // read all data
    memset(buf, 0, 1024);
    randomReadArray(0x00, buf, 1024);

    // write some data with single writes
    for (int i = 513; i < 520; ++i)
    {
        randomWrite(i, i);
        HAL_Delay(EEPROM_WRITE_DELAY); // at least 5ms
    }

    // read
    memset(buf, 0, 1024);
    randomReadArray(0x00, buf, 1024);

    // write some data with page write
    uint8_t data[] = {
            0x11, 0x13, 0x15, 0x17,
            0x21, 0x23, 0x25, 0x27,
            0x31, 0x33, 0x35, 0x37,
            0x41, 0x43, 0x45, 0x47
    };
    randomPageWrite(702, data, 16);
    HAL_Delay(EEPROM_WRITE_DELAY);

    // read
    memset(buf, 0, 1024);
    randomReadArray(0x00, buf, 1024);

    while (true)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        HAL_Delay(500);
    }
}

static void randomWrite(uint16_t address, uint8_t data)
{
    // address is 10bit with 2 MSB being page number
    uint8_t page = (address >> 8) & 0x03;
    address &= 0xFF;

    uint8_t devAddress = (0xA << 4) | (page << 1);

    uint8_t buffer[2] = {address, data};

    HAL_I2C_Master_Transmit(&hi2c2, devAddress, buffer, 2, 1);
}

static void randomPageWrite(uint16_t address, const uint8_t *data, uint8_t dataLen)
{
    //write up to 16 bytes at once (single page)
    // data is written inside single page, so writing more just overwrites
    // previous data
    if (dataLen > EEPROM_PAGE_SIZE)
        dataLen = EEPROM_PAGE_SIZE;

    uint8_t page = (address >> 8) & 0x03;
    address &= 0xFF;

    uint8_t devAddress = (0xA << 4) | (page << 1);

    uint8_t buffer[EEPROM_PAGE_SIZE + 1];
    buffer[0] = address;
    memcpy(&buffer[1], data, dataLen);

    HAL_I2C_Master_Transmit(&hi2c2, devAddress, buffer, dataLen + 1, 1);
}

static void randomWriteArray(uint16_t address, const uint8_t *data, uint16_t dataLen)
{
    uint8_t size = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);

    while (dataLen > 0)
    {
        if (dataLen < size)
            size = dataLen;
        randomPageWrite(address, data, size);
        dataLen -= size;
        address += size;
        data += size;
        size = EEPROM_PAGE_SIZE;
        HAL_Delay(EEPROM_WRITE_DELAY); // at least 5 ms
    }
}

static uint8_t randomRead(uint16_t address)
{
    uint8_t page = (address >> 8) & 0x03;
    uint8_t addr = address;

    uint8_t devAddress = (0xA << 4) | (page << 1);

    // dummy write to set current address
    HAL_I2C_Master_Transmit(&hi2c2, devAddress, &addr, 1, 10);

    uint8_t data = 0;
    // actual read
    HAL_I2C_Master_Receive(&hi2c2, devAddress | 0x01, &data, 1, 10);
    return data;
}

static void randomReadArray(uint16_t address, uint8_t *data, uint16_t dataLen)
{
    data[0] = randomRead(address);
    for (int i = 1; i < dataLen; ++i)
        data[i] = currentAddressRead();
}

static uint8_t currentAddressRead(void)
{
    // read data at current address.
    // address is autoincremented after this type of read

    // page bits are not used during current address read
    uint8_t devAddress = (0xA << 4);

    uint8_t data = 0;
    HAL_I2C_Master_Receive(&hi2c2, devAddress | 0x01, &data, 1, 10);
    return data;
}
