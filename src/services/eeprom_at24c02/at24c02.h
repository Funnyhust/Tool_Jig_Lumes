#ifndef AT24C02_H
#define AT24C02_H

#include <Arduino.h>

#define AT24C02_ADDRESS 0x50

void at24c02_init(void);
void at24c02_write(uint8_t address, uint8_t data);
void at24c02_write_block(uint8_t address, uint8_t* data, uint8_t length);
uint8_t at24c02_read(uint8_t address);
void at24c02_read_block(uint8_t address, uint8_t* data, uint8_t length);


#endif