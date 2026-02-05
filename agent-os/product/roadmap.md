# Product Roadmap

## Phase 1: Rust Implementation ✓ Complete

A working digital level indicator built with embedded Rust:

- **Accelerometer integration**: LIS3DSH sensor communication via SPI
- **LED control**: Directional LED indicators using PWM brightness control (TIM4)
- **Tilt detection algorithm**: EMA filtering and angle calculation for stable level detection
- **Clean architecture**: Modular code structure separating hardware, algorithms, and control logic

## Phase 2: C++ Implementation (Planned)

Port the level indicator functionality to C++ to demonstrate:

- Dual-language proficiency for embedded systems
- Ability to work with HighQ.aero's existing C++ codebase
- Comparative analysis of Rust vs C++ approaches in embedded context
- Trade-offs between memory safety, performance, and development velocity
