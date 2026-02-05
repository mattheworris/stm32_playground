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
    // Accelerometer orientation CORRECTED: Y points toward USB (North), X points RIGHT (East)
    // Swap X and Y to fix 90° rotation offset
    float tilt_north_south = calculateTilt(filtered_y_, 1.0f);  // Y axis = North/South
    float tilt_east_west = calculateTilt(filtered_x_, 1.0f);    // X axis = East/West

    // When a side tips UP (away from gravity), that LED should stay bright
    // Positive tilt angle = that direction tips DOWN (toward gravity) = LED dims
    return LedBrightness{
        .north = angleToBrightness(-tilt_north_south),  // Y axis, inverted (swap with south)
        .east  = angleToBrightness(-tilt_east_west),    // X axis, inverted for correct L/R
        .south = angleToBrightness(tilt_north_south),   // Y axis (swap with north)
        .west  = angleToBrightness(tilt_east_west)      // X axis
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

    // Deadband: within ±8° is considered level (full brightness)
    static constexpr float DEADBAND = 8.0f;
    if (angle <= DEADBAND) {
        return 255;
    }

    // Above threshold = LED off
    if (angle >= MAX_TILT) {
        return 0;
    }

    // Linear fade from 8° to threshold
    float brightness = 255.0f * (1.0f - (angle - DEADBAND) / (MAX_TILT - DEADBAND));
    return static_cast<uint8_t>(brightness);
}
