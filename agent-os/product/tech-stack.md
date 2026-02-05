# Tech Stack

## Hardware Platform

- **Microcontroller**: STM32F407VG (ARM Cortex-M4F with FPU)
- **Development Board**: STM32F4 Discovery
- **Target**: `thumbv7em-none-eabihf` (Cortex-M4F with hard float ABI)

## Language & Runtime

- **Language**: Rust (Edition 2024)
- **Environment**: `no_std` bare-metal (no operating system)
- **Core Libraries**:
  - `cortex-m` (v0.7) - ARM Cortex-M processor support
  - `cortex-m-rt` (v0.7) - Runtime and startup code
  - `panic-probe` (v1.0) - Panic handler for debugging

## Hardware Abstraction

- **HAL**: `stm32f4xx-hal` (v0.23) - STM32F4 Hardware Abstraction Layer
- **Embedded HAL**: `embedded-hal` (v0.2.7) - Peripheral trait abstractions
- **Math Library**: `libm` (v0.2) - Math functions for `no_std`

## Peripherals & Sensors

- **Accelerometer**: LIS3DSH via SPI1
  - Driver: `lis3dsh` (v0.1.0)
  - Interface: SPI with manual chip select
- **LEDs**: 4x directional LEDs via TIM4 PWM
  - Channels: PD12-PD15 (West, North, East, South)

## Development Tools

- **Build System**: Cargo with custom linker
- **Linker**: `flip-link` - Stack overflow protection
- **Debugger**: `probe-rs` - Flashing and debugging
- **Logging**: `defmt` (v1.0) + `defmt-rtt` (v1.0) - Efficient logging over RTT
- **Testing**: `defmt-test` (v0.3) - On-target unit testing

## Build Configuration

- **Optimization**: Size-optimized (`opt-level = "s"`)
- **LTO**: Fat LTO for release builds
- **Debug Info**: Full debug symbols retained in release
- **Memory Layout**: Custom `memory.x` linker script
