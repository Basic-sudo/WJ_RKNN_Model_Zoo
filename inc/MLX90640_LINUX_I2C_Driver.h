#ifndef _MLX90640_LINUX_I2C_DRIVER_H_
#define _MLX90640_LINUX_I2C_DRIVER_H_
#include <Head.hpp>

int MLX90640_I2CRead(uint8 slaveAddr, uint16 startAddress, uint16 nMemAddressRead, uint16 *data);

int MLX90640_I2CWrite(uint8 slaveAddr, uint16 writeAddress, uint16 data);

int MLX90640_I2CGeneralReset(void);

#endif
