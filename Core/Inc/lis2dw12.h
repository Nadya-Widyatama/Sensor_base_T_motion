/*
 * lis2dw12.h
 *
 *  Created on: Nov 5, 2024
 *      Author: win10
 */

/* lis2dw12.h */
#ifndef LIS2DW12_H
#define LIS2DW12_H

#include "main.h"

/* Register addresses */
#define LIS2DW12_WHO_AM_I     0x0F
#define LIS2DW12_CTRL1        0x20
#define LIS2DW12_CTRL2        0x21
#define LIS2DW12_CTRL3        0x22
#define LIS2DW12_CTRL4_INT1   0x23
#define LIS2DW12_CTRL5_INT2   0x24
#define LIS2DW12_CTRL6        0x25
#define LIS2DW12_STATUS       0x27
#define LIS2DW12_OUT_X_L      0x28
#define LIS2DW12_OUT_X_H      0x29
#define LIS2DW12_OUT_Y_L      0x2A
#define LIS2DW12_OUT_Y_H      0x2B
#define LIS2DW12_OUT_Z_L      0x2C
#define LIS2DW12_OUT_Z_H      0x2D
#define LIS2DW12_OUT_T_L      0x0D

/* Device ID */
#define LIS2DW12_ID           0x44

/* SPI read/write commands */
#define LIS2DW12_READ         0x80
#define LIS2DW12_WRITE        0x00

/* Scale selection */
typedef enum {
    LIS2DW12_2G  = 0x00,
    LIS2DW12_4G  = 0x01,
    LIS2DW12_8G  = 0x02,
    LIS2DW12_16G = 0x03
} LIS2DW12_Scale_t;

/* Output Data Rate */
typedef enum {
    LIS2DW12_POWER_DOWN  = 0x00,
    LIS2DW12_ODR_12_5Hz  = 0x02,
    LIS2DW12_ODR_25Hz    = 0x03,
    LIS2DW12_ODR_50Hz    = 0x04,
    LIS2DW12_ODR_100Hz   = 0x05,
    LIS2DW12_ODR_200Hz   = 0x06,
    LIS2DW12_ODR_400Hz   = 0x07,
    LIS2DW12_ODR_800Hz   = 0x08,
    LIS2DW12_ODR_1600Hz  = 0x09
} LIS2DW12_ODR_t;

typedef struct {
    float x;
    float y;
    float z;
} LIS2DW12_Data_t;

/* Function prototypes */
uint8_t LIS2DW12_Init(SPI_HandleTypeDef *hspi);
uint8_t LIS2DW12_SetScale(SPI_HandleTypeDef *hspi, LIS2DW12_Scale_t scale);
uint8_t LIS2DW12_SetODR(SPI_HandleTypeDef *hspi, LIS2DW12_ODR_t odr);
uint8_t LIS2DW12_ReadAccel(SPI_HandleTypeDef *hspi, LIS2DW12_Data_t *data);
uint8_t LIS2DW12_ReadID(SPI_HandleTypeDef *hspi);

/**
 * @brief  Membaca suhu dari sensor
 * @param  hspi: Handler SPI
 * @param  temp: Pointer ke variabel float untuk menyimpan hasil suhu dalam °C
 * @retval 1 jika berhasil, 0 jika gagal
 */
uint8_t LIS2DW12_ReadTemp(SPI_HandleTypeDef *hspi, float *temp);

#endif /* LIS2DW12_H */
