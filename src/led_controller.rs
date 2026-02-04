#![allow(unused)]
use stm32f4xx_hal::timer::PwmChannel;
use stm32f4xx_hal::pac::TIM4;

/// Controls the 4 LEDs using PWM for brightness control.
/// 
/// LED Mapping (Discovery board orientation: USB away from you):
/// - West (left):    PD12 -> TIM4_CH1 -> Green LED (LD4)
/// - North (top):    PD13 -> TIM4_CH2 -> Orange LED (LD3)
/// - East (right):   PD14 -> TIM4_CH3 -> Red LED (LD5)
/// - South (bottom): PD15 -> TIM4_CH4 -> Blue LED (LD6)
pub struct LedController {
    ch1_west: PwmChannel<TIM4, 0>,
    ch2_north: PwmChannel<TIM4, 1>,
    ch3_east: PwmChannel<TIM4, 2>,
    ch4_south: PwmChannel<TIM4, 3>,
    max_duty: u16,
}

impl LedController {
    /// Create a new LED controller from 
    /// 
    /// Assumes PWM is already configured at 1 kHz with 8-bit resolution.
    pub fn new(
        ch1: PwmChannel<TIM4, 0>,
        ch2: PwmChannel<TIM4, 1>,
        ch3: PwmChannel<TIM4, 2>,
        ch4: PwmChannel<TIM4, 3>,
    ) -> Self {
        let max_duty = ch1.get_max_duty();

        let mut controller = Self {
            ch1_west: ch1,
            ch2_north: ch2,
            ch3_east: ch3,
            ch4_south: ch4,
            max_duty,
        };

        // Enable all channels
        controller.ch1_west.enable();
        controller.ch2_north.enable();
        controller.ch3_east.enable();
        controller.ch4_south.enable();

        controller
    }

    pub fn set_brightness(&mut self, west: u8, north: u8, east: u8, south: u8) {
        let scale = |brightness: u8| -> u16 {
            ((brightness as u32 * self.max_duty as u32) / 255) as u16
        };

        self.ch1_west.set_duty(scale(west));
        self.ch2_north.set_duty(scale(north));
        self.ch3_east.set_duty(scale(east));
        self.ch4_south.set_duty(scale(south));
    }

    /// Turn all LEDs to full brightness.
    pub fn all_on(&mut self) {
        self.set_brightness(255, 255, 255, 255);
    }

    /// Turn all LEDs off.
    pub fn all_off(&mut self) {
        self.set_brightness(0, 0, 0, 0);
    }
}
