#![allow(unused)]

use libm::atan2f;

/// Threshold for considering board "level" (in g-force).
/// 0.05g ~ 3.5 degrees tilt
const LEVEL_THRESHOLD_G: f32 = 0.05;

/// Maximum tilt angle for full LED dimming
const MAX_TILT: f32 = 15.0;

/// Exponential moving average smoothing factor.
/// 0.15 provides stronger smoothing to reduce flashing from vibrations.
/// Lower = smoother but slower response, Higher = faster but more jitter.
const EMA_ALPHA: f32 = 0.15;

/// LED brightness values for each compass direction.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct LedBrightness {
    pub north: u8,
    pub east: u8,
    pub south: u8,
    pub west: u8,
}

/// Detects board tilt and calculates LED brightness.
pub struct LevelDetector {
    filtered_x: f32,
    filtered_y: f32,
    alpha: f32,
    tilt_threshold: f32,
}

impl LevelDetector {
    pub fn new() -> Self {
        Self {
            filtered_x: 0.0,
            filtered_y: 0.0,
            alpha: EMA_ALPHA,
            tilt_threshold: MAX_TILT,
        }
    }

    pub fn update(&mut self, accel_x: f32, accel_y: f32) -> LedBrightness {
        // Apply exponential moving average filter
        self.filtered_x = self.alpha * accel_x + (1.0 - self.alpha) * self.filtered_x;
        self.filtered_y = self.alpha * accel_y + (1.0 - self.alpha) * self.filtered_y;

        // Calculate tilt angles (can be positive or negative)
        // Accelerometer orientation: X points toward USB (top), Y points LEFT
        let tilt_x = self.calculate_tilt(self.filtered_x, 1.0);  // North/South axis
        let tilt_y = self.calculate_tilt(self.filtered_y, 1.0);  // West/East axis

        // Invert the logic: when a side tips UP, that LED stays bright
        LedBrightness {
            north: self.angle_to_brightness(tilt_x),  // Inverted
            south: self.angle_to_brightness(-tilt_x),   // Inverted
            east: self.angle_to_brightness(-tilt_y),    // Inverted
            west: self.angle_to_brightness(tilt_y),   // Inverted
        }
    }

    fn calculate_tilt(&self, axis_reading: f32, gravity: f32) -> f32 {
        let angle_rad = atan2f(axis_reading, gravity);
        angle_rad * (180.0 / core::f32::consts::PI)
    }

    fn angle_to_brightness(&self, angle: f32) -> u8 {
        // Negative angle = tilting toward this LED = stay bright
        // Positive angle = tilting away from this LED = dim it

        // Deadband: within ±3° is considered level (full brightness)
        if angle <= 3.0 {
            return 255;
        }

        // Above threshold = LED off
        if angle >= self.tilt_threshold {
            return 0;
        }

        // Linear fade from 3° to threshold
        let brightness = 255.0 * (1.0 - (angle - 3.0) / (self.tilt_threshold - 3.0));
        brightness as u8
    }
}