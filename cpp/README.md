# C++ Level Indicator Port

This is a C++ port of the Rust level indicator application for STM32F407VG, created for comparison and interview discussion at HighQ.aero.

## Overview

Port of the embedded Rust level indicator to C++ using STM32 HAL, demonstrating equivalent functionality with a traditional embedded C++ toolchain.

## Hardware

- **MCU:** STM32F407VG (Cortex-M4F, 168 MHz, 1MB Flash, 192KB RAM)
- **Accelerometer:** LIS3DSH (SPI1: PA5=SCK, PA6=MISO, PA7=MOSI, PE3=CS)
- **LEDs:** 4x PWM-controlled (TIM4 CH1-4: PD12-PD15)

## Project Structure

```text
cpp/
├── CMakeLists.txt              # Build configuration
├── STM32F407VGTx_FLASH.ld      # Linker script
├── arm-none-eabi-gcc.cmake     # Toolchain file
│
├── Core/                       # STM32 system files
│   ├── Inc/                    # HAL config, interrupts, main.h
│   └── Src/                    # System init, interrupts, MSP, startup
│
├── Drivers/                    # ST libraries
│   ├── STM32F4xx_HAL_Driver/   # HAL peripheral drivers
│   ├── CMSIS_Device/           # STM32F4 device files
│   └── CMSIS/                  # ARM CMSIS core
│
└── App/                        # Application code (C++)
    ├── Inc/                    # Headers
    │   ├── level_algorithm.hpp
    │   ├── accelerometer.hpp
    │   └── led_controller.hpp
    └── Src/                    # Implementation
        ├── main.cpp
        ├── level_algorithm.cpp
        ├── accelerometer.cpp
        └── led_controller.cpp
```

## Build System

**CMake + ARM GCC Toolchain**

### Prerequisites

```bash
# ARM GNU Toolchain (installed in ~/arm-toolchain/)
curl -L "https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi.tar.xz" -o /tmp/arm-toolchain.tar.xz
mkdir -p ~/arm-toolchain && cd ~/arm-toolchain && tar -xf /tmp/arm-toolchain.tar.xz

# CMake
brew install cmake
```

### Build

```bash
cd aero-app
git submodule update --init --recursive

cd cpp
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi-gcc.cmake ..
make -j4
```

### Flash

```bash
probe-rs run --chip STM32F407VG level_indicator.elf
```

## Implementation Details

### LevelAlgorithm (level_algorithm.cpp)

Direct port from `src/level_algorithm.rs`:

- **EMA Filter:** `α = 0.15` for vibration smoothing
- **Tilt Calculation:** `atan2f(accel, gravity) * 180/π`
- **Brightness Mapping:**
  - `≤3°`: Full brightness (255)
  - `3°-15°`: Linear fade
  - `≥15°`: LED off (0)

### Accelerometer (accelerometer.cpp)

LIS3DSH driver via SPI Mode 3:

- **WHO_AM_I:** 0x0F → 0x3F verification
- **CTRL_REG4:** 0x67 (100 Hz ODR, ±2g range, all axes)
- **Conversion:** `g = raw_value / 16384.0f`

### LedController (led_controller.cpp)

PWM control via TIM4:

- **Frequency:** 1 kHz (84 MHz / 84000)
- **Channels:** West(CH1), North(CH2), East(CH3), South(CH4)
- **Scaling:** `duty = (brightness * ARR) / 255`

### Main (main.cpp)

- **Clock:** 84 MHz from 8 MHz HSE (PLL: M=8, N=336, P=4)
- **Loop:** 100 Hz update rate (10ms delay)
- **Peripherals:** HAL-based SPI1, TIM4, GPIO initialization

## Size Comparison

```
Rust version:    7,752 bytes flash, 1,032 bytes RAM
C++ version:     9,384 bytes flash, 2,076 bytes RAM
```

The C++ version is ~21% larger due to HAL overhead vs. Rust's zero-cost abstractions.

## Key Differences: Rust vs C++

| Aspect | Rust | C++ |
|--------|------|-----|
| **Memory Safety** | Compile-time guarantees | Manual management (RAII) |
| **HAL Abstraction** | embedded-hal traits | Function pointers, callbacks |
| **Dependencies** | Cargo ecosystem | Git submodules, manual integration |
| **Code Size** | Smaller (zero-cost) | Larger (HAL overhead) |
| **Build Speed** | Slower (initial) | Faster (incremental) |
| **Debugging** | probe-rs + defmt | GDB + semihosting |

## Interview Discussion Points

1. **Safety:** Rust's borrow checker vs C++ RAII patterns
2. **Abstractions:** embedded-hal traits vs HAL callbacks
3. **Algorithm Port:** Line-by-line translation demonstrates equivalence
4. **Performance:** Both run at 100 Hz with identical behavior
5. **Ecosystem:** Rust's cargo vs C++'s fragmented tooling
6. **Adoption:** C++ mature ecosystem vs Rust's growing embedded support

## Files Ported

- `src/level_algorithm.rs` → `App/Src/level_algorithm.cpp`
- `src/accelerometer.rs` → `App/Src/accelerometer.cpp` (implemented directly, not using lib)
- `src/led_controller.rs` → `App/Src/led_controller.cpp`
- `src/bin/level_indicator.rs` → `App/Src/main.cpp`

## License

Same as parent project
