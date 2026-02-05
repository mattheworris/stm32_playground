/**
  ******************************************************************************
  * @file    accelerometer.hpp
  * @brief   LIS3DSH accelerometer driver
  ******************************************************************************
  */

#ifndef ACCELEROMETER_HPP
#define ACCELEROMETER_HPP

#include "stm32f4xx_hal.h"
#include <cstdint>

/**
 * @brief LIS3DSH 3-axis MEMS accelerometer driver
 *
 * Port of Rust Accelerometer from src/accelerometer.rs
 * Communicates via SPI Mode 3 (CPOL=1, CPHA=1)
 */
class Accelerometer {
public:
    /**
     * @brief Constructor
     * @param hspi Pointer to SPI handle
     * @param cs_port Chip select GPIO port
     * @param cs_pin Chip select GPIO pin
     */
    Accelerometer(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);

    /**
     * @brief Initialize accelerometer
     * @return true if initialization successful
     */
    bool init();

    /**
     * @brief Read X and Y axis values
     * @param x Reference to store X value in g's
     * @param y Reference to store Y value in g's
     * @return true if read successful
     */
    bool readXY(float& x, float& y);

private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;

    // LIS3DSH registers
    static constexpr uint8_t WHO_AM_I = 0x0F;
    static constexpr uint8_t CTRL_REG4 = 0x20;
    static constexpr uint8_t CTRL_REG5 = 0x24;
    static constexpr uint8_t OUT_X_L = 0x28;
    static constexpr uint8_t OUT_X_H = 0x29;
    static constexpr uint8_t OUT_Y_L = 0x2A;
    static constexpr uint8_t OUT_Y_H = 0x2B;
    static constexpr uint8_t OUT_Z_L = 0x2C;
    static constexpr uint8_t OUT_Z_H = 0x2D;

    // Expected WHO_AM_I value
    static constexpr uint8_t WHO_AM_I_VALUE = 0x3F;

    // SPI read/write bits
    static constexpr uint8_t READ_BIT = 0x80;
    static constexpr uint8_t WRITE_BIT = 0x00;
    static constexpr uint8_t MULTI_BYTE_BIT = 0x40;  // For auto-increment

    // ±2g range: 16384 LSB/g
    static constexpr float LSB_PER_G = 16384.0f;

    void chipSelectLow();
    void chipSelectHigh();
    bool readRegister(uint8_t reg, uint8_t* data);
    bool writeRegister(uint8_t reg, uint8_t data);
    bool readRawData(int16_t& x, int16_t& y, int16_t& z);
};

#endif /* ACCELEROMETER_HPP */
