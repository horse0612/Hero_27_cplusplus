#ifndef BMI088_MIDDLEWARE_H_
#define BMI088_MIDDLEWARE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
uint8_t BMI088_SPI_Transfer(uint8_t accel, const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
