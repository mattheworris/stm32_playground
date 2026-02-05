/**
  ******************************************************************************
  * @file    led_controller.cpp
  * @brief   LED PWM controller implementation
  ******************************************************************************
  */

#include "led_controller.hpp"

LedController::LedController(TIM_HandleTypeDef* htim)
    : htim_(htim)
    , max_duty_(0)
{
}

void LedController::init()
{
    // Get the auto-reload value (PWM period)
    max_duty_ = __HAL_TIM_GET_AUTORELOAD(htim_);

    // Start PWM on all channels
    HAL_TIM_PWM_Start(htim_, CHANNEL_WEST);
    HAL_TIM_PWM_Start(htim_, CHANNEL_NORTH);
    HAL_TIM_PWM_Start(htim_, CHANNEL_EAST);
    HAL_TIM_PWM_Start(htim_, CHANNEL_SOUTH);

    // Initialize all LEDs to off
    allOff();
}

void LedController::setBrightness(uint8_t north, uint8_t east, uint8_t south, uint8_t west)
{
    __HAL_TIM_SET_COMPARE(htim_, CHANNEL_NORTH, brightnessToDuty(north));
    __HAL_TIM_SET_COMPARE(htim_, CHANNEL_EAST,  brightnessToDuty(east));
    __HAL_TIM_SET_COMPARE(htim_, CHANNEL_SOUTH, brightnessToDuty(south));
    __HAL_TIM_SET_COMPARE(htim_, CHANNEL_WEST,  brightnessToDuty(west));
}

void LedController::allOn()
{
    setBrightness(255, 255, 255, 255);
}

void LedController::allOff()
{
    setBrightness(0, 0, 0, 0);
}

uint32_t LedController::brightnessToDuty(uint8_t brightness) const
{
    // Scale 0-255 brightness to 0-max_duty_ PWM value
    return (static_cast<uint32_t>(brightness) * max_duty_) / 255;
}
