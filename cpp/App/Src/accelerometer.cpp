/**
  ******************************************************************************
  * @file    accelerometer.cpp
  * @brief   LIS3DSH accelerometer driver implementation
  ******************************************************************************
  */

#include "accelerometer.hpp"
#include "debug.hpp"

Accelerometer::Accelerometer(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : hspi_(hspi)
    , cs_port_(cs_port)
    , cs_pin_(cs_pin)
{
}

bool Accelerometer::init()
{
    DEBUG_PRINTF("Accel: Initializing LIS3DSH...");

    // Ensure CS is high (idle)
    chipSelectHigh();
    HAL_Delay(10);

    // CRITICAL: Soft reset first (matches Rust lis3dsh library)
    DEBUG_PRINTF("Accel: Performing soft reset (CTRL_REG3=0x01)");
    static constexpr uint8_t CTRL_REG3 = 0x23;
    if (!writeRegister(CTRL_REG3, 0x01)) {
        DEBUG_PRINTF("Accel: Soft reset failed!");
        return false;
    }

    // Wait 5ms for reset to complete (per Rust library)
    HAL_Delay(5);

    // Now read WHO_AM_I to verify communication
    DEBUG_PRINTF("Accel: Reading WHO_AM_I...");
    uint8_t who_am_i = 0;
    if (!readRegister(WHO_AM_I, &who_am_i)) {
        DEBUG_PRINTF("Accel: WHO_AM_I read failed!");
        return false;
    }

    DEBUG_PRINTF("Accel: WHO_AM_I = 0x%02X (expected 0x%02X)", who_am_i, WHO_AM_I_VALUE);
    if (who_am_i != WHO_AM_I_VALUE) {
        DEBUG_PRINTF("Accel: WHO_AM_I mismatch!");
        return false;
    }

    // Configure CTRL_REG4: Enable XYZ at 100 Hz, ±2g
    // ODR = 100Hz (0b0110), BDU=1, Enable X,Y,Z (0b111)
    // 0b01101111 = 0x6F
    DEBUG_PRINTF("Accel: Configuring CTRL_REG4=0x6F (with BDU=1)");
    if (!writeRegister(CTRL_REG4, 0x6F)) {
        DEBUG_PRINTF("Accel: CTRL_REG4 write failed!");
        return false;
    }

    // Verify CTRL_REG4 was written correctly
    uint8_t ctrl_reg4_readback = 0;
    if (readRegister(CTRL_REG4, &ctrl_reg4_readback)) {
        DEBUG_PRINTF("Accel: CTRL_REG4 readback = 0x%02X", ctrl_reg4_readback);
    }

    // Configure CTRL_REG3: Enable DRDY signal (matches Rust: 0xE8)
    DEBUG_PRINTF("Accel: Configuring CTRL_REG3=0xE8");
    if (!writeRegister(CTRL_REG3, 0xE8)) {
        DEBUG_PRINTF("Accel: CTRL_REG3 write failed!");
        return false;
    }

    // Verify CTRL_REG3 was written correctly
    uint8_t ctrl_reg3_readback = 0;
    if (readRegister(CTRL_REG3, &ctrl_reg3_readback)) {
        DEBUG_PRINTF("Accel: CTRL_REG3 readback = 0x%02X", ctrl_reg3_readback);
    }

    // Check CTRL_REG5 (just to see default state)
    static constexpr uint8_t CTRL_REG5 = 0x24;
    uint8_t ctrl_reg5_val = 0;
    if (readRegister(CTRL_REG5, &ctrl_reg5_val)) {
        DEBUG_PRINTF("Accel: CTRL_REG5 (default) = 0x%02X", ctrl_reg5_val);
    }

    // Wait a bit for sensor to stabilize
    HAL_Delay(100);

    DEBUG_PRINTF("Accel: Init complete!");
    return true;
}

bool Accelerometer::readXY(float& x, float& y)
{
    int16_t raw_x, raw_y, raw_z;
    if (!readRawData(raw_x, raw_y, raw_z)) {
        return false;
    }

    // Convert to g's
    x = static_cast<float>(raw_x) / LSB_PER_G;
    y = static_cast<float>(raw_y) / LSB_PER_G;

    return true;
}

void Accelerometer::chipSelectLow()
{
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
}

void Accelerometer::chipSelectHigh()
{
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

bool Accelerometer::readRegister(uint8_t reg, uint8_t* data)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg | READ_BIT), 0x00};
    uint8_t rx[2] = {0, 0};

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 100; i++);  // CS setup delay

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, tx, rx, 2, 100);

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    for (volatile int i = 0; i < 100; i++);  // CS hold delay

    DEBUG_PRINTF("  REG[0x%02X] = 0x%02X (status=%d)", reg, rx[1], status);

    if (status == HAL_OK) {
        *data = rx[1];
        return true;
    }
    return false;
}

bool Accelerometer::writeRegister(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg | WRITE_BIT), data};

    chipSelectLow();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi_, tx, 2, 100);
    chipSelectHigh();

    return (status == HAL_OK);
}

bool Accelerometer::readRawData(int16_t& x, int16_t& y, int16_t& z)
{
    // CRITICAL: Rust library does TWO separate SPI transfers, not one!
    // First transfer: send command byte
    // Second transfer: read data bytes (NO multi-byte bit needed!)

    uint8_t data[6] = {0};

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 100; i++);  // CS setup

    // Transfer 1: Send command (address with read bit, NO multi-byte bit)
    uint8_t cmd = OUT_X_L | READ_BIT;  // 0x28 | 0x80 = 0xA8
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi_, &cmd, 1, 100);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        DEBUG_PRINTF("  Command transmit failed: %d", status);
        return false;
    }

    // Transfer 2: Read 6 data bytes
    status = HAL_SPI_Receive(hspi_, data, 6, 100);

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    for (volatile int i = 0; i < 100; i++);  // CS hold

    // DEBUG_PRINTF("  Raw read: status=%d, data=%02X %02X %02X %02X %02X %02X",
    //              status, data[0], data[1], data[2], data[3], data[4], data[5]);

    if (status != HAL_OK) {
        return false;
    }

    // Combine low and high bytes (little-endian)
    x = static_cast<int16_t>((data[1] << 8) | data[0]);
    y = static_cast<int16_t>((data[3] << 8) | data[2]);
    z = static_cast<int16_t>((data[5] << 8) | data[4]);

    return true;
}
