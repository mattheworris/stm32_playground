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
    DEBUG_PRINTF("Accel: Setting CS high");
    // Ensure CS is high (idle)
    chipSelectHigh();
    HAL_Delay(10);

    // Verify SPI configuration
    DEBUG_PRINTF("Accel: SPI Configuration:");
    DEBUG_PRINTF("  CLKPolarity: %d (expect 2=SPI_POLARITY_HIGH)", hspi_->Init.CLKPolarity);
    DEBUG_PRINTF("  CLKPhase: %d (expect 2=SPI_PHASE_2EDGE)", hspi_->Init.CLKPhase);
    DEBUG_PRINTF("  BaudRate Prescaler: %d (expect 8)", hspi_->Init.BaudRatePrescaler);
    DEBUG_PRINTF("  CS Pin State: %d (expect 1=HIGH)", HAL_GPIO_ReadPin(cs_port_, cs_pin_));

    // Check GPIO configurations
    DEBUG_PRINTF("Accel: GPIO Register Checks:");
    DEBUG_PRINTF("  GPIOA MODER (PA5/6/7): 0x%08lX", GPIOA->MODER);
    DEBUG_PRINTF("  GPIOA AFR[0] (PA5/6/7): 0x%08lX", GPIOA->AFR[0]);
    DEBUG_PRINTF("  GPIOE MODER (PE3): 0x%08lX", GPIOE->MODER);
    DEBUG_PRINTF("  GPIOE ODR (PE3 bit3): %d", (GPIOE->ODR >> 3) & 1);

    // Check SPI peripheral state
    DEBUG_PRINTF("Accel: SPI1 State:");
    DEBUG_PRINTF("  CR1: 0x%04X", SPI1->CR1);
    DEBUG_PRINTF("  SR: 0x%04X", SPI1->SR);
    DEBUG_PRINTF("  CR1.SPE (enabled): %d", (SPI1->CR1 & SPI_CR1_SPE) ? 1 : 0);

    DEBUG_PRINTF("Accel: Reading WHO_AM_I register (0x0F)...");
    // Verify WHO_AM_I register
    uint8_t who_am_i = 0;
    if (!readRegister(WHO_AM_I, &who_am_i)) {
        DEBUG_PRINTF("Accel: WHO_AM_I read failed!\n");
        return false;
    }

    DEBUG_PRINTF("Accel: WHO_AM_I = 0x%02X (expected 0x%02X)\n", who_am_i, WHO_AM_I_VALUE);
    if (who_am_i != WHO_AM_I_VALUE) {
        DEBUG_PRINTF("Accel: WHO_AM_I mismatch!\n");
        return false;
    }

    DEBUG_PRINTF("Accel: Configuring CTRL_REG4...\n");
    // Configure CTRL_REG4: Enable all axes, 100 Hz ODR, ±2g range
    // ODR = 100Hz (0b0110), Enable X,Y,Z (0b111)
    // Register layout: [ODR3:ODR0][BDU][ZEN][YEN][XEN]
    // 0b01100111 = 0x67
    if (!writeRegister(CTRL_REG4, 0x67)) {
        DEBUG_PRINTF("Accel: CTRL_REG4 write failed!\n");
        return false;
    }

    DEBUG_PRINTF("Accel: Configuring CTRL_REG5...\n");
    // Configure CTRL_REG5: ±2g range (default, but set explicitly)
    // FSCALE = 00 (±2g), Anti-aliasing bandwidth = 800Hz
    // 0b00000000 = 0x00
    if (!writeRegister(CTRL_REG5, 0x00)) {
        DEBUG_PRINTF("Accel: CTRL_REG5 write failed!\n");
        return false;
    }

    DEBUG_PRINTF("Accel: Init complete, waiting for stabilization\n");
    HAL_Delay(10);  // Allow sensor to stabilize
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
    DEBUG_PRINTF("  >>> readRegister ENTRY: reg=0x%02X", reg);

    uint8_t tx[2] = {static_cast<uint8_t>(reg | READ_BIT), 0x00};
    uint8_t rx[2] = {0};

    DEBUG_PRINTF("  SPI TX: 0x%02X 0x%02X", tx[0], tx[1]);

    DEBUG_PRINTF("  About to set CS low (inline)");
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    DEBUG_PRINTF("  CS is now low (inline)");

    // Small delay for CS setup time
    DEBUG_PRINTF("  Starting delay loop");
    for (volatile int i = 0; i < 10; i++);
    DEBUG_PRINTF("  Delay complete");

    DEBUG_PRINTF("  About to call HAL_SPI_TransmitReceive");
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, tx, rx, 2, 100);
    DEBUG_PRINTF("  HAL_SPI_TransmitReceive returned status=%d", status);

    chipSelectHigh();

    DEBUG_PRINTF("  SPI RX: 0x%02X 0x%02X (status=%d)", rx[0], rx[1], status);

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
    // Read 6 bytes starting from OUT_X_L with auto-increment
    // LIS3DSH stores data in little-endian format: L, H for each axis
    // Must set bit 6 (multi-byte) for address auto-increment
    uint8_t tx[7] = {static_cast<uint8_t>(OUT_X_L | READ_BIT | MULTI_BYTE_BIT), 0, 0, 0, 0, 0, 0};
    uint8_t rx[7] = {0};

    chipSelectLow();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, tx, rx, 7, 100);
    chipSelectHigh();

    if (status != HAL_OK) {
        return false;
    }

    // Combine low and high bytes (little-endian)
    x = static_cast<int16_t>((rx[2] << 8) | rx[1]);
    y = static_cast<int16_t>((rx[4] << 8) | rx[3]);
    z = static_cast<int16_t>((rx[6] << 8) | rx[5]);

    return true;
}
