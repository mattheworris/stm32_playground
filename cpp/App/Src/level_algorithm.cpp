/**
  ******************************************************************************
  * @file    level_algorithm.cpp
  * @brief   Level detection algorithm implementation
  ******************************************************************************
  */

#include "level_algorithm.hpp"
#include <cmath>

// Pi constant
static constexpr float PI = 3.14159265358979323846f;

LevelAlgorithm::LevelAlgorithm()
    : filtered_x_(0.0f)
    , filtered_y_(0.0f)
{
}

LedBrightness LevelAlgorithm::update(float accel_x, float accel_y)
{
    // Apply exponential moving average filter
    filtered_x_ = EMA_ALPHA * accel_x + (1.0f - EMA_ALPHA) * filtered_x_;
    filtered_y_ = EMA_ALPHA * accel_y + (1.0f - EMA_ALPHA) * filtered_y_;

    // Calculate tilt angles (can be positive or negative)
    // Accelerometer orientation: X points toward USB (top), Y points LEFT
    float tilt_x = calculateTilt(filtered_x_, 1.0f);  // North/South axis
    float tilt_y = calculateTilt(filtered_y_, 1.0f);  // West/East axis

    // Invert the logic: when a side tips UP, that LED stays bright
    // This matches the Rust implementation exactly
    return LedBrightness{
        .north = angleToBrightness(-tilt_x),  // Inverted
        .east  = angleToBrightness(tilt_y),   // Inverted
        .south = angleToBrightness(tilt_x),   // Inverted
        .west  = angleToBrightness(-tilt_y)   // Inverted
    };
}

float LevelAlgorithm::calculateTilt(float axis_reading, float gravity) const
{
    float angle_rad = atan2f(axis_reading, gravity);
    return angle_rad * (180.0f / PI);
}

uint8_t LevelAlgorithm::angleToBrightness(float angle) const
{
    // Negative angle = tilting toward this LED = stay bright
    // Positive angle = tilting away from this LED = dim it

    // Deadband: within ±3° is considered level (full brightness)
    if (angle <= 3.0f) {
        return 255;
    }

    // Above threshold = LED off
    if (angle >= MAX_TILT) {
        return 0;
    }

    // Linear fade from 3° to threshold
    float brightness = 255.0f * (1.0f - (angle - 3.0f) / (MAX_TILT - 3.0f));
    return static_cast<uint8_t>(brightness);
}
