/**
  ******************************************************************************
  * @file    led_controller.hpp
  * @brief   LED PWM controller for 4 directional LEDs
  ******************************************************************************
  */

#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

#include "stm32f4xx_hal.h"
#include <cstdint>

/**
 * @brief Controls 4 LEDs via PWM (TIM4 channels 1-4)
 *
 * Port of Rust LedController from src/led_controller.rs
 * Hardware mapping:
 *   - West:  TIM4_CH1 (PD12)
 *   - North: TIM4_CH2 (PD13)
 *   - East:  TIM4_CH3 (PD14)
 *   - South: TIM4_CH4 (PD15)
 */
class LedController {
public:
    /**
     * @brief Constructor
     * @param htim Pointer to TIM handle (must be TIM4)
     */
    explicit LedController(TIM_HandleTypeDef* htim);

    /**
     * @brief Initialize PWM channels
     */
    void init();

    /**
     * @brief Set individual LED brightness
     * @param north North LED brightness (0-255)
     * @param east East LED brightness (0-255)
     * @param south South LED brightness (0-255)
     * @param west West LED brightness (0-255)
     */
    void setBrightness(uint8_t north, uint8_t east, uint8_t south, uint8_t west);

    /**
     * @brief Turn all LEDs on at full brightness
     */
    void allOn();

    /**
     * @brief Turn all LEDs off
     */
    void allOff();

private:
    TIM_HandleTypeDef* htim_;
    uint32_t max_duty_;

    // Channel mapping for TIM4
    static constexpr uint32_t CHANNEL_WEST  = TIM_CHANNEL_1;  // PD12
    static constexpr uint32_t CHANNEL_NORTH = TIM_CHANNEL_2;  // PD13
    static constexpr uint32_t CHANNEL_EAST  = TIM_CHANNEL_3;  // PD14
    static constexpr uint32_t CHANNEL_SOUTH = TIM_CHANNEL_4;  // PD15

    /**
     * @brief Convert 0-255 brightness to PWM duty cycle
     */
    uint32_t brightnessToDuty(uint8_t brightness) const;
};

#endif /* LED_CONTROLLER_HPP */
