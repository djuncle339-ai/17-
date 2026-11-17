#ifndef SWI2C_H
#define SWI2C_H
#include <stdint.h>

typedef enum I2C_STATUS {
    I2C_OK = 0,
    I2C_ERR = 1,
}I2C_STATUS;
I2C_STATUS I2C_Write_Reg(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t* data);
I2C_STATUS I2C_Read_Reg(uint8_t dev_addr, uint8_t reg_addr,uint16_t len, uint8_t* data);
#endif