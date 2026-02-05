/**
  ******************************************************************************
  * @file    level_algorithm.hpp
  * @brief   Level detection algorithm - port from Rust
  ******************************************************************************
  */

#ifndef LEVEL_ALGORITHM_HPP
#define LEVEL_ALGORITHM_HPP

#include <cstdint>

/**
 * @brief LED brightness values for each compass direction
 */
struct LedBrightness {
    uint8_t north;
    uint8_t east;
    uint8_t south;
    uint8_t west;
};

/**
 * @brief Detects board tilt and calculates LED brightness
 *
 * Port of Rust LevelDetector from src/level_algorithm.rs
 */
class LevelAlgorithm {
public:
    LevelAlgorithm();

    /**
     * @brief Update with new accelerometer readings
     * @param accel_x X-axis reading in g's (points toward USB/top)
     * @param accel_y Y-axis reading in g's (points LEFT)
     * @return LED brightness values for each direction
     */
    LedBrightness update(float accel_x, float accel_y);

private:
    float filtered_x_;
    float filtered_y_;

    // Constants from Rust version
    static constexpr float EMA_ALPHA = 0.15f;        // Smoothing factor
    static constexpr float MAX_TILT = 15.0f;         // Maximum tilt angle for full dimming
    static constexpr float LEVEL_THRESHOLD_G = 0.05f; // ~3.5 degrees

    /**
     * @brief Calculate tilt angle from accelerometer reading
     * @param axis_reading Accelerometer axis reading in g's
     * @param gravity Gravity component (typically 1.0)
     * @return Tilt angle in degrees
     */
    float calculateTilt(float axis_reading, float gravity) const;

    /**
     * @brief Convert tilt angle to LED brightness
     * @param angle Tilt angle in degrees
     * @return Brightness value (0-255)
     *
     * Deadband: ±3° = full brightness (255)
     * Linear fade: 3° to 15° (MAX_TILT)
     * Above 15° = LED off (0)
     */
    uint8_t angleToBrightness(float angle) const;
};

#endif /* LEVEL_ALGORITHM_HPP */
