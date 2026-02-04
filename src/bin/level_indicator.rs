#![no_std]
#![no_main]

use panic_probe as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    rcc::Config,
    spi::{Spi, Mode, Phase, Polarity},
};

use aero_app::{
    accelerometer::Accelerometer,
    led_controller::LedController,
    level_algorithm::LevelDetector,
};

#[entry]
fn main() -> ! {
    // Take ownership of device peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())
            .sysclk(84.MHz())
            .pclk1(42.MHz())
    );

    // Configure GPIO
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);
    let gpioe = dp.GPIOE.split(&mut rcc);

    // Configure PWM for LEDs (TIM4, channels 1-4 on PD12-PD15)
    let (_pwm_manager, (ch1, ch2, ch3, ch4)) = dp.TIM4.pwm_hz(1.kHz(), &mut rcc);

    // Bind GPIO pins to PWM channels
    let pwm0 = ch1.with(gpiod.pd12);  // CH1 - West
    let pwm1 = ch2.with(gpiod.pd13);  // CH2 - North
    let pwm2 = ch3.with(gpiod.pd14);  // CH3 - East
    let pwm3 = ch4.with(gpiod.pd15);  // CH4 - South

    // Create LED controller (enables channels internally)
    let mut leds = LedController::new(pwm0, pwm1, pwm2, pwm3);
    leds.all_on();  // Test: all LEDs on at startup

    // Configure SPI1 for LIS3DSH accelerometer (PA5=SCK, PA6=MISO, PA7=MOSI)
    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    let spi_mode = Mode {
        polarity: Polarity::IdleHigh,
        phase: Phase::CaptureOnSecondTransition,
    };

    let spi = Spi::new(
        dp.SPI1,
        (Some(sck), Some(miso), Some(mosi)),
        spi_mode,
        10.MHz(),
        &mut rcc,
    );

    // Chip select for accelerometer (PE3)
    let cs = gpioe.pe3.into_push_pull_output();

    // Configure SysTick delay (needed for accelerometer init)
    let mut delay = cp.SYST.delay(&rcc.clocks);

    // Create accelerometer wrapper
    let mut accel = Accelerometer::new(spi, cs, &mut delay).unwrap();

    // Create level detector
    let mut level_detector = LevelDetector::new();

    // Main loop
    loop {
        // Read accelerometer
        if let Ok((x, y)) = accel.read_xy_g() {
            // Update level detection algorithm
            let brightness = level_detector.update(x, y);

            // Update LED brightness
            leds.set_brightness(
                brightness.north,
                brightness.east,
                brightness.south,
                brightness.west,
            );
        }

        // Wait 10ms for next update (100 Hz)
        delay.delay_ms(10u32);
    }
}
