# In-Depth Study Guide: Embedded Rust Implementation

This guide walks through every significant line of code to ensure you deeply understand the implementation.

---

## Part 1: Main Entry Point (`level_indicator.rs`)

### Lines 1-5: Bare Metal Setup

```rust
#![no_std]        // Don't link standard library
#![no_main]       // We provide our own entry point

use panic_probe as _;  // Panic handler for debugging
use cortex_m_rt::entry;
```

**What's happening:**

1. **`#![no_std]`** - This is an *inner attribute* (note the `!`). It tells the compiler not to link the standard library. The standard library assumes an OS (file I/O, networking, heap allocation, threads). We don't have any of that. We only get `core`, which has:
   - Primitives: `i32`, `f32`, `bool`
   - Smart pointers: `Option<T>`, `Result<T, E>`
   - Traits: `Copy`, `Clone`, `Iterator`, `Debug`
   - Slice operations, basic math
   - **No heap allocation, no String, no Vec**

2. **`#![no_main]`** - Normally, Rust expects `fn main()` and provides startup code. In embedded, WE control startup. The `cortex_m_rt` crate will call our entry point after initializing RAM.

3. **`use panic_probe as _;`** - When a panic happens (`.unwrap()` on `None`, array out of bounds), we need a *panic handler*. The `panic_probe` crate implements this by breaking into the debugger. The `as _` means we're importing it but not using it directly—its presence is enough.

4. **`use cortex_m_rt::entry;`** - This imports the `#[entry]` attribute macro.

### Lines 19-23: Peripheral Singleton Pattern

```rust
#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
```

**What's happening:**

1. **`#[entry]`** - This macro generates:
   - The vector table (interrupt handlers)
   - Startup code (copy .data from flash to RAM, zero .bss)
   - A call to our `main()` function
   - The actual chip entry point is `Reset_Handler`, which calls this

2. **`fn main() -> !`** - The `!` is the "never" type. This function never returns. Why? Because we're the only code running. If we returned, what would we return to? The chip would just jump to garbage memory. So we loop forever or trigger a reset.

3. **`pac::Peripherals::take()`** - PAC = Peripheral Access Crate. This is auto-generated from the STM32 SVD (System View Description) file. It gives us type-safe access to every peripheral register.

   The magic: `take()` returns `Option<Peripherals>`:
   ```rust
   static mut PERIPHERALS_TAKEN: bool = false;

   pub fn take() -> Option<Peripherals> {
       cortex_m::interrupt::free(|_| {
           if unsafe { PERIPHERALS_TAKEN } {
               None  // Already taken!
           } else {
               unsafe { PERIPHERALS_TAKEN = true; }
               Some(Peripherals { /* all the registers */ })
           }
       })
   }
   ```

   **Why this matters:** You can only call `take()` once. This prevents you from creating two `&mut` references to the same hardware, which would violate Rust's aliasing rules and could cause hardware conflicts.

4. **`.unwrap()`** - Panics if `None`. We're asserting "this MUST be the first call to take()". If it's not, the program crashes loudly rather than continuing with undefined behavior.

### Lines 26-30: Clock Configuration

```rust
let mut rcc = dp.RCC.freeze(
    Config::hse(8.MHz())
        .sysclk(84.MHz())
        .pclk1(42.MHz())
);
```

**What's happening:**

1. **`dp.RCC`** - RCC = Reset and Clock Control peripheral. This peripheral controls all clocks on the chip.

2. **`.freeze(Config::hse(8.MHz())...)`** - The "freeze" name comes from the type-state pattern. Before calling `freeze()`, the RCC is in an "unconfigured" state. After, it's "frozen" (configured and locked). You can't reconfigure clocks at runtime—they're set once.

3. **Clock tree:**
   ```
   HSE (8 MHz external crystal)
     ↓
   PLL (Phase-Locked Loop)
     ↓
   SYSCLK (84 MHz) ← Main system clock
     ↓
   AHB (84 MHz) ← Bus for CPU, DMA, etc.
     ↓
   APB1 (42 MHz) ← Peripheral bus 1 (TIM4, SPI1)
   APB2 (84 MHz) ← Peripheral bus 2 (faster peripherals)
   ```

4. **Why 84 MHz?** The STM32F407 can go up to 168 MHz, but we don't need that much performance. Lower frequency = lower power consumption.

5. **Return value: `Rcc`** - The `rcc` variable is now a "configured RCC". We need to pass `&mut rcc` to `.split()` on GPIO ports because `split()` needs to enable the GPIO clocks.

### Lines 33-35: GPIO Splitting

```rust
let gpioa = dp.GPIOA.split(&mut rcc);
let gpiod = dp.GPIOD.split(&mut rcc);
let gpioe = dp.GPIOE.split(&mut rcc);
```

**What's happening:**

1. **Before split:** `dp.GPIOA` is a single struct representing the entire GPIO port (16 pins: PA0-PA15). You have full control but no safety.

2. **After split:** You get individual pin structs:
   ```rust
   struct Parts {
       pa0: PA0<Input<Floating>>,
       pa1: PA1<Input<Floating>>,
       pa2: PA2<Input<Floating>>,
       // ... pa15
   }
   ```

3. **`split()` consumes `GPIOA`** - After calling `split()`, you can't access `GPIOA` anymore. The HAL has "split" it into individual pins. This prevents you from configuring the port-level registers while also using individual pins (which would be a conflict).

4. **`&mut rcc` parameter** - Why do we pass this? Because `split()` needs to enable the GPIOA clock by writing to `RCC.AHB1ENR` register. By requiring `&mut rcc`, the type system ensures:
   - You've configured clocks (can't split before clock config)
   - Only one thing can modify RCC at a time (no race conditions)

5. **Pin states:** Notice `Input<Floating>` in the type. Each pin starts in this default state (high-impedance input). To use a pin, you call methods like:
   - `.into_push_pull_output()` → `PA0<Output<PushPull>>`
   - `.into_alternate()` → `PA0<Alternate<AF5>>`

   These methods *consume* the old pin and *return* a new pin with a different type. The compiler tracks the state.

### Lines 38-47: PWM Configuration

```rust
let (_pwm_manager, (ch1, ch2, ch3, ch4)) = dp.TIM4.pwm_hz(1.kHz(), &mut rcc);

let pwm0 = ch1.with(gpiod.pd12);  // CH1 - West
let pwm1 = ch2.with(gpiod.pd13);  // CH2 - North
let pwm2 = ch3.with(gpiod.pd14);  // CH3 - East
let pwm3 = ch4.with(gpiod.pd15);  // CH4 - South

let mut leds = LedController::new(pwm0, pwm1, pwm2, pwm3);
leds.all_on();
```

**What's happening:**

1. **`dp.TIM4.pwm_hz(1.kHz(), &mut rcc)`** - This configures TIM4 (a hardware timer) for PWM output at 1 kHz frequency.

   Internally, it calculates:
   ```
   Timer clock = 84 MHz (from APB1 × 2)
   Desired frequency = 1 kHz

   Prescaler + Period values such that:
   84,000,000 / (prescaler × period) = 1,000

   Example: prescaler=84, period=1000
   → 84,000,000 / (84 × 1000) = 1,000 Hz ✓
   ```

2. **Return value: `(PwmHz<TIM4>, (C1, C2, C3, C4))`**
   - `_pwm_manager`: The timer struct itself (unused, so we prefix with `_`)
   - `(ch1, ch2, ch3, ch4)`: The four PWM channel objects

3. **`ch1.with(gpiod.pd12)`** - This *binds* PWM channel 1 to physical pin PD12. The pin must support this alternate function (TIM4_CH1). If you tried to bind to the wrong pin, it wouldn't compile.

   This consumes both `ch1` and `pd12`, returning a `PwmChannel<TIM4, 0>` that's bound to a specific pin.

4. **Type safety:** Notice the channel index is in the TYPE: `PwmChannel<TIM4, 0>`, `PwmChannel<TIM4, 1>`, etc. You can't pass channel 0 to a function expecting channel 1—the compiler prevents it.

5. **`LedController::new()`** - We give ownership of all four PWM channels to the LED controller. Now only the LED controller can modify those PWM outputs.

### Lines 50-75: SPI and Accelerometer

```rust
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

let cs = gpioe.pe3.into_push_pull_output();
let mut delay = cp.SYST.delay(&rcc.clocks);
let mut accel = Accelerometer::new(spi, cs, &mut delay).unwrap();
```

**What's happening:**

1. **`.into_alternate()`** - This configures the pin for alternate function mode. Each pin on STM32 can serve multiple functions (GPIO, UART, SPI, I2C, etc.). The specific function is determined by which peripheral you connect it to. By calling `into_alternate()`, we're saying "this pin is no longer GPIO, it's controlled by a peripheral."

   Type changes: `PA5<Input<Floating>>` → `PA5<Alternate<PushPull>>`

2. **SPI Mode 3 (`CPOL=1, CPHA=1`):**
   - `Polarity::IdleHigh`: Clock line sits HIGH when idle
   - `Phase::CaptureOnSecondTransition`: Data is sampled on the falling edge of the clock

   This is what the LIS3DSH datasheet specifies. Wrong mode = garbage data.

3. **`Spi::new(...)`:**
   - Consumes `dp.SPI1` peripheral
   - Consumes the three pins (or `None` if you only need TX or RX)
   - Configures baud rate: 10 MHz
   - Requires `&mut rcc` to enable SPI1 clock

4. **Chip Select (CS):** SPI is a multi-device bus. CS (chip select) determines which device is active. We configure PE3 as a regular GPIO output that we'll manually toggle.

5. **SysTick delay:** The Cortex-M SysTick is a simple countdown timer. We configure it as a blocking delay provider (needed for accelerometer initialization delays).

6. **`Accelerometer::new(spi, cs, &mut delay)`:**
   - Takes ownership of `spi` and `cs`
   - Uses `delay` to wait during initialization
   - Returns `Result<Accelerometer, SPI::Error>`
   - `.unwrap()` panics if init fails (sensor not responding)

### Lines 80-99: Main Loop

```rust
let mut level_detector = LevelDetector::new();

defmt::println!("Starting level indicator...");

loop {
    if let Ok((x, y)) = accel.read_xy_g() {
        let brightness = level_detector.update(x, y);

        leds.set_brightness(
            brightness.north,
            brightness.east,
            brightness.south,
            brightness.west,
        );
    }

    delay.delay_ms(10u32);  // 100 Hz
}
```

**What's happening:**

1. **`LevelDetector::new()`** - Creates the algorithm object with initial state (filtered_x = 0, filtered_y = 0).

2. **`defmt::println!(...)`** - Efficient logging. Instead of sprintf on the target, this sends a format string index over RTT (Real-Time Transfer) and the host computer does the formatting. Zero overhead.

3. **`if let Ok((x, y)) = ...`** - Pattern matching on `Result`. If `read_xy_g()` succeeds, extract the `(x, y)` tuple. If it fails, skip this iteration.

4. **`level_detector.update(x, y)`** - Runs the EMA filter, calculates tilt, returns brightness struct.

5. **`leds.set_brightness(...)`** - Updates all four PWM duty cycles.

6. **`delay.delay_ms(10u32)`** - Blocking delay for 10 milliseconds. This gives us a 100 Hz update rate (1000ms / 10ms = 100 updates per second).

   Why blocking? In a simple application like this, we don't need interrupts or async. We just loop, read, compute, update, wait, repeat.

---

## Part 2: Accelerometer Module

### Generic Structure

```rust
pub struct Accelerometer<SPI, CS>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    driver: Lis3dsh<Lis3dshSpi<SPI, CS>>,
}
```

**Deep dive:**

1. **Generics:** This struct is generic over TWO types: `SPI` and `CS`. It doesn't care what specific types they are, as long as they implement the required traits.

2. **Trait bounds (`where` clause):**
   - `SPI: Transfer<u8>` - The SPI type must implement the `Transfer<u8>` trait from `embedded_hal`. This trait has one method:
     ```rust
     fn transfer<'w>(&mut self, words: &'w mut [u8])
         -> Result<&'w [u8], Self::Error>;
     ```
     This is a full-duplex SPI transfer: send bytes while simultaneously receiving bytes.

   - `CS: OutputPin` - The chip select type must implement `OutputPin` from `embedded_hal`:
     ```rust
     fn set_low(&mut self) -> Result<(), Self::Error>;
     fn set_high(&mut self) -> Result<(), Self::Error>;
     ```

3. **Why generics?** Portability. This code works with:
   - `stm32f4xx_hal::spi::Spi` (STM32F4)
   - `nrf52_hal::spi::Spi` (nRF52)
   - `esp32_hal::spi::Spi` (ESP32)
   - Any HAL that implements `embedded_hal` traits

   **Zero runtime cost:** The compiler generates specialized versions for each concrete type used. If you use `stm32f4xx_hal::spi::Spi`, the compiler generates code as if you had written it specifically for that type. No virtual dispatch, no function pointers, no overhead.

4. **`Lis3dsh<Lis3dshSpi<SPI, CS>>`** - The `lis3dsh` crate provides a driver that's also generic. We're wrapping it.

### Constructor

```rust
pub fn new(spi: SPI, cs: CS, delay: &mut impl DelayMs<u8>)
    -> Result<Self, SPI::Error>
{
    let mut driver = Lis3dsh::new_spi(spi, cs);
    driver.init(delay)?;
    Ok(Self { driver })
}
```

**Deep dive:**

1. **Ownership transfer:** The parameters `spi: SPI` and `cs: CS` are taken *by value*, not by reference. This means we're taking OWNERSHIP. The caller can't use those peripherals anymore.

2. **`&mut impl DelayMs<u8>`** - "impl Trait" in argument position means "any type that implements DelayMs". The `&mut` means we borrow it mutably for the duration of the function. After `new()` returns, the caller can still use their delay.

3. **`driver.init(delay)?`** - The `?` operator:
   - If `init()` returns `Ok(())`, continue
   - If `init()` returns `Err(e)`, immediately return `Err(e)` from `new()`

   This is like "try/catch" but checked at compile time. You can't forget to handle errors.

4. **What does `init()` do?**
   - Soft reset the sensor (write to CTRL_REG3)
   - Wait 5ms for reset to complete (using `delay`)
   - Verify WHO_AM_I register (should be 0x3F for LIS3DSH)
   - Configure CTRL_REG4 (enable axes, set data rate, enable BDU)
   - Configure CTRL_REG3 (enable data-ready signal)

### Reading Data

```rust
pub fn read_xy_g(&mut self) -> Result<(f32, f32), SPI::Error> {
    let [x, y, _z] = self.driver.read_data()?;

    const LSB_PER_G: f32 = 16_384.0;
    Ok((x as f32 / LSB_PER_G, y as f32 / LSB_PER_G))
}
```

**Deep dive:**

1. **`&mut self`** - We need mutable access because reading from SPI modifies the peripheral's state (internal buffers, status registers).

2. **Array destructuring:** `let [x, y, _z] = ...` extracts the three i16 values. The `_z` means we're ignoring Z-axis (we don't need it for 2D tilt).

3. **`self.driver.read_data()?`** - This calls the lis3dsh crate's method which:
   - Pulls CS low
   - Sends command byte: `0x28 | 0x80` (OUT_X_L register, read bit set)
   - Reads 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
   - Pulls CS high
   - Combines bytes into three i16 values (little-endian)

4. **LSB_PER_G constant:** The LIS3DSH in ±2g mode has 16-bit output:
   - Full scale = ±2g
   - Resolution = 2^16 = 65536 levels
   - Each g = 65536 / 4 = 16384 LSB

   So: `1.0g = 16384 LSB`, `0.5g = 8192 LSB`, etc.

5. **Conversion to float:** Dividing the raw integer by 16384.0 gives us acceleration in g-force units. Example:
   - Raw reading: `x = 8192`
   - In g's: `8192 / 16384.0 = 0.5g`

---

## Part 3: Level Algorithm

### Structure

```rust
pub struct LevelDetector {
    filtered_x: f32,
    filtered_y: f32,
    alpha: f32,
    tilt_threshold: f32,
}
```

**Deep dive:**

1. **State storage:** This struct holds the filter state. The `filtered_x` and `filtered_y` values persist between calls to `update()`.

2. **Memory layout:** Four f32 values = 4 bytes each = 16 bytes total. This struct is tiny and lives on the stack.

3. **No heap allocation:** In `no_std`, we can't use `Vec` or `Box`. Everything must be statically sized or stack-allocated.

### EMA Filter

```rust
pub fn update(&mut self, accel_x: f32, accel_y: f32) -> LedBrightness {
    self.filtered_x = self.alpha * accel_x
                    + (1.0 - self.alpha) * self.filtered_x;
    self.filtered_y = self.alpha * accel_y
                    + (1.0 - self.alpha) * self.filtered_y;
    // ...
}
```

**Deep dive:**

1. **EMA formula:**
   ```
   y[n] = α × x[n] + (1 - α) × y[n-1]

   Where:
   y[n]   = new filtered value
   x[n]   = new raw sample
   y[n-1] = previous filtered value
   α      = smoothing factor (0 < α < 1)
   ```

2. **Why EMA instead of moving average?**

   Moving average:
   ```rust
   let mut buffer = [0.0; 10];  // Keep last 10 samples
   let avg = buffer.iter().sum::<f32>() / 10.0;
   ```
   - Pros: Simple, predictable
   - Cons: Requires buffer (memory), O(N) computation

   EMA:
   ```rust
   let filtered = alpha * new + (1.0 - alpha) * old;
   ```
   - Pros: O(1) computation, constant memory (one float)
   - Cons: Less intuitive, harder to tune

3. **Alpha = 0.15 analysis:**
   - 15% weight on new sample
   - 85% weight on accumulated history
   - Smooths out vibrations and hand tremor
   - Still responsive enough for user interaction

   If alpha = 0.9:
   - Very responsive (90% new data)
   - Lots of jitter/noise
   - LEDs would flicker

   If alpha = 0.05:
   - Very smooth (95% old data)
   - Sluggish response
   - Feels laggy

4. **FPU advantage:** The STM32F407 has a hardware FPU (Floating Point Unit). These multiply-add operations are single-cycle instructions. If we were on a Cortex-M0 (no FPU), we'd want to use fixed-point math instead.

### Tilt Calculation

```rust
let tilt_x = self.calculate_tilt(self.filtered_x, 1.0);
let tilt_y = self.calculate_tilt(self.filtered_y, 1.0);

fn calculate_tilt(&self, axis_reading: f32, gravity: f32) -> f32 {
    let angle_rad = atan2f(axis_reading, gravity);
    angle_rad * (180.0 / core::f32::consts::PI)
}
```

**Deep dive:**

1. **Geometry of tilt:**
   ```
   When board is level:
     Z-axis: 1.0g (down)
     X-axis: 0.0g
     Y-axis: 0.0g

   When tilted θ degrees around X-axis:
     Z-axis: cos(θ) × 1.0g
     X-axis: sin(θ) × 1.0g
     Y-axis: 0.0g

   Therefore:
     tan(θ) = X / Z
     θ = atan(X / Z)
   ```

2. **Why atan2 instead of atan?**

   `atan(y/x)` has problems:
   - Division by zero when x=0
   - Loses quadrant information (can't tell 45° from -135°)
   - Range: -90° to +90°

   `atan2(y, x)` solves this:
   - No division (handles x=0 gracefully)
   - Full quadrant information
   - Range: -180° to +180°

3. **Gravity = 1.0 parameter:** We're passing gravity as a parameter for future flexibility. If you wanted to do 3D tilt (using Z-axis too), you'd pass the filtered Z value here. For 2D, we assume Z ≈ 1.0g.

4. **libm::atan2f:** In `no_std`, we don't have std::f32::atan2. The `libm` crate provides pure-Rust implementations of math functions. The compiler optimizes these aggressively—often as good as libc.

5. **Radians to degrees:** `atan2f` returns radians. We multiply by `180/π` to get degrees (more intuitive for thresholds like "15 degrees").

### Brightness Mapping

```rust
fn angle_to_brightness(&self, angle: f32) -> u8 {
    if angle <= 3.0 {
        return 255;
    }

    if angle >= self.tilt_threshold {
        return 0;
    }

    let brightness = 255.0 * (1.0 - (angle - 3.0)
                                   / (self.tilt_threshold - 3.0));
    brightness as u8
}
```

**Deep dive:**

1. **Piecewise function:**
   ```
   brightness(θ) =
     255           if θ ≤ 3°
     0             if θ ≥ 15°
     linear fade   if 3° < θ < 15°
   ```

2. **Deadband (3°):** Within ±3° of level, just return full brightness. This prevents:
   - LED flicker from accelerometer noise
   - Distracting brightness changes when nearly level
   - User frustration ("it's never perfectly level!")

3. **Linear interpolation math:**
   ```
   At θ = 3°:  brightness = 255 × (1 - 0/12) = 255
   At θ = 9°:  brightness = 255 × (1 - 6/12) = 127.5
   At θ = 15°: brightness = 255 × (1 - 12/12) = 0
   ```

   General formula:
   ```
   t = (angle - min) / (max - min)  // Normalize to 0..1
   brightness = 255 × (1 - t)       // Invert (more tilt = less bright)
   ```

4. **`as u8` cast:** Explicit conversion from f32 to u8. Truncates decimal part. Rust requires explicit casts—no silent conversions.

5. **Sign inversion in update():**
   ```rust
   LedBrightness {
       north: self.angle_to_brightness(-tilt_x),
       south: self.angle_to_brightness(tilt_x),
       // ...
   }
   ```

   If tilt_x is POSITIVE (USB side tipped up):
   - North LED: gets -tilt_x (negative angle) → brightness = 255 (stays bright)
   - South LED: gets +tilt_x (positive angle) → brightness fades

   This creates the intuitive behavior: "high side stays bright, low side dims."

---

## Part 4: LED Controller

### Structure

```rust
pub struct LedController {
    ch1_west: PwmChannel<TIM4, 0>,
    ch2_north: PwmChannel<TIM4, 1>,
    ch3_east: PwmChannel<TIM4, 2>,
    ch4_south: PwmChannel<TIM4, 3>,
    max_duty: u16,
}
```

**Deep dive:**

1. **Const generics:** `PwmChannel<TIM4, 0>` vs `PwmChannel<TIM4, 1>` are DIFFERENT types. The channel number is part of the type signature.

   Why this matters:
   ```rust
   fn set_channel_0(ch: PwmChannel<TIM4, 0>) { ... }

   let ch1: PwmChannel<TIM4, 1> = ...;
   set_channel_0(ch1);  // COMPILE ERROR: expected 0, got 1
   ```

   This prevents wiring mistakes at compile time.

2. **Ownership:** Once created, `LedController` owns all four PWM channels. Nothing else can modify those timer channels.

3. **`max_duty`:** PWM timers have a period (also called ARR - Auto-Reload Register). When the counter reaches this value, it resets to 0. The duty cycle is what percentage of the period the output is HIGH.

   Example:
   ```
   Timer frequency: 84 MHz
   Prescaler: 84 (divide by 84) → 1 MHz timer clock
   Period: 1000 → 1 kHz PWM frequency

   max_duty = 1000

   duty = 500 → 50% brightness
   duty = 1000 → 100% brightness
   duty = 0 → 0% brightness
   ```

### Setting Brightness

```rust
pub fn set_brightness(&mut self, west: u8, north: u8,
                      east: u8, south: u8) {
    let scale = |brightness: u8| -> u16 {
        ((brightness as u32 * self.max_duty as u32) / 255) as u16
    };

    self.ch1_west.set_duty(scale(west));
    self.ch2_north.set_duty(scale(north));
    self.ch3_east.set_duty(scale(east));
    self.ch4_south.set_duty(scale(south));
}
```

**Deep dive:**

1. **Closure:** `let scale = |brightness| { ... }` defines a closure (anonymous function). It captures `self.max_duty` from the enclosing scope.

   Why a closure instead of a function?
   - Avoids repeating the scaling math four times
   - Compiler inlines it (zero overhead)
   - Can capture variables from scope

2. **Scaling math:**
   ```
   Input: 0..255 (u8 brightness)
   Output: 0..max_duty (u16 timer value)

   Formula: duty = (brightness × max_duty) / 255
   ```

   **Overflow prevention:**
   ```rust
   (brightness as u32 * self.max_duty as u32) / 255
   ```

   If we didn't upcast to u32:
   - `255 × 1000 = 255,000`
   - `u16::MAX = 65,535`
   - Overflow! 💥

   By casting to u32 first, we have room (u32::MAX = 4 billion).

3. **Example calculation:**
   ```
   brightness = 128 (50%)
   max_duty = 1000

   duty = (128 × 1000) / 255
        = 128,000 / 255
        = 502

   Timer count: 0..1000
   Output HIGH: 0..502 (50.2%)
   Output LOW: 502..1000 (49.8%)
   ```

4. **LED perception:** Human perception of brightness is logarithmic, not linear. Ideally, we'd use gamma correction:
   ```rust
   let gamma = |x: f32| x.powf(2.2);
   ```

   But for this application, linear is good enough.

---

## Part 5: Key Rust Concepts

### Ownership

**Rule 1: Every value has exactly one owner**

```rust
let x = String::from("hello");
let y = x;  // Ownership moved from x to y
println!("{}", x);  // COMPILE ERROR: x no longer valid
```

In embedded:
```rust
let spi = Spi::new(...);
let accel = Accelerometer::new(spi, cs);  // spi moved into accel
spi.transfer(...);  // COMPILE ERROR: spi was moved
```

**Rule 2: When the owner goes out of scope, the value is dropped**

```rust
{
    let spi = Spi::new(...);
    // Use spi
}  // spi goes out of scope, drop() called, peripheral released
```

### Borrowing

**Immutable borrow (&T):**
```rust
fn read_config(cfg: &Config) {
    // Can read cfg, cannot modify
}

let cfg = Config::new();
read_config(&cfg);
read_config(&cfg);  // Can have multiple & refs
```

**Mutable borrow (&mut T):**
```rust
fn update_state(state: &mut State) {
    state.value = 42;  // Can modify
}

let mut state = State::new();
update_state(&mut state);
```

**The Rules:**
- Either ONE &mut T OR many &T
- Never both at the same time
- References must not outlive the value

**Why this matters in embedded:**
```rust
// BAD (prevented by compiler):
let spi = &mut peripherals.SPI1;
let sensor1 = Sensor::new(spi);  // Moved!
let sensor2 = Sensor::new(spi);  // ERROR: spi already moved

// GOOD (use SPI bus sharing):
let spi = Spi::new(...);
let bus = shared_bus::BusManager::new(spi);
let sensor1 = Sensor::new(bus.acquire());
let sensor2 = Sensor::new(bus.acquire());
```

### Traits

**Trait definition:**
```rust
pub trait Transfer<W> {
    type Error;

    fn transfer<'w>(&mut self, words: &'w mut [W])
        -> Result<&'w [W], Self::Error>;
}
```

**Trait implementation:**
```rust
impl Transfer<u8> for Spi<SPI1> {
    type Error = Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8])
        -> Result<&'w [u8], Self::Error>
    {
        // STM32-specific implementation
    }
}
```

**Using traits as bounds:**
```rust
fn communicate<SPI>(spi: &mut SPI) -> Result<(), SPI::Error>
where
    SPI: Transfer<u8>
{
    let mut data = [0x00, 0x01, 0x02];
    spi.transfer(&mut data)?;
    Ok(())
}
```

**Monomorphization (zero cost):**

When you write:
```rust
communicate(&mut stm32_spi);
communicate(&mut nrf52_spi);
```

The compiler generates TWO specialized versions:
```rust
// Generated for STM32:
fn communicate_stm32(spi: &mut Spi<SPI1>) -> Result<(), Error> {
    // Direct calls, no indirection
}

// Generated for nRF52:
fn communicate_nrf52(spi: &mut nrf52_hal::Spi) -> Result<(), Error> {
    // Direct calls, no indirection
}
```

No runtime cost. Same performance as if you hand-coded for each platform.

### Type States

**Example: GPIO pin states**

```rust
struct Pin<MODE> {
    pin_number: u8,
    _mode: PhantomData<MODE>,
}

struct Input;
struct Output;

impl Pin<Input> {
    fn read(&self) -> bool { /* ... */ }
}

impl Pin<Output> {
    fn set_high(&mut self) { /* ... */ }
    fn set_low(&mut self) { /* ... */ }
}

impl Pin<Input> {
    fn into_output(self) -> Pin<Output> {
        // Configure hardware registers
        Pin { pin_number: self.pin_number, _mode: PhantomData }
    }
}
```

Usage:
```rust
let pin: Pin<Input> = Pin::new(5);
pin.set_high();  // COMPILE ERROR: no method set_high on Pin<Input>

let pin: Pin<Output> = pin.into_output();
pin.set_high();  // OK!
pin.read();  // COMPILE ERROR: no method read on Pin<Output>
```

**Why this is powerful:**
- Invalid states are unrepresentable
- State transitions are explicit
- Compiler enforces correct usage
- Zero runtime cost (types erased)

---

## Study Checklist

Before the interview, make sure you can explain:

- [ ] What `#![no_std]` means and why we use it
- [ ] Why `Peripherals::take()` returns `Option`
- [ ] How the clock tree works (HSE → PLL → SYSCLK → APB)
- [ ] What `split()` does to GPIO ports
- [ ] Why we pass `&mut rcc` to peripheral initialization
- [ ] Type states and how pins track their mode
- [ ] Generic structs and trait bounds
- [ ] Ownership transfer in `Accelerometer::new()`
- [ ] The `?` operator and error propagation
- [ ] EMA filter formula and why we use it
- [ ] `atan2` vs `atan` and why quadrants matter
- [ ] Deadband purpose (prevent flicker)
- [ ] PWM duty cycle calculation
- [ ] Why Rust worked first try vs C++ debugging
- [ ] The SPI protocol bug (multi-byte bit issue)
- [ ] How traits enable zero-cost abstraction

You've got this! The key is understanding the *why* behind each decision, not just memorizing syntax.
