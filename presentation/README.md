# Embedded Rust Presentation for HighQ.aero

## How to Present

### Opening the Presentation

1. **Open in browser:**

   ```bash
   cd /Users/mattheworris/projects/aero-app/presentation
   open index.html
   ```

   Or just double-click `index.html`

2. **Keyboard Controls:**
   - `→` or `Space` - Next slide
   - `←` - Previous slide
   - `S` - Open speaker notes window (IMPORTANT!)
   - `Esc` - Overview mode (see all slides)
   - `F` - Fullscreen
   - `B` - Blackout screen

### Speaker Notes

Press `S` to open a separate window with detailed speaker notes for each slide. This window shows:

- Current slide preview
- Next slide preview
- **Detailed talking points and explanations**
- Timer

**Practice with the speaker notes open!** They contain the deep technical explanations you need to master.

---

## Study Guide: Deep Understanding

### Before the Interview

#### 1. **Run Through the Code**

Navigate through each source file and understand:

**Main entry (`src/bin/level_indicator.rs`):**

- Why `#![no_std]` and `#![no_main]`?
- What does `Peripherals::take()` return and why `Option`?
- Clock tree: HSE → PLL → SYSCLK → APB1/APB2
- Why must we pass `&mut rcc` to `split()`?

**Accelerometer (`src/accelerometer.rs`):**

- What are `Transfer<u8>` and `OutputPin` traits?
- Why is the struct generic over `SPI` and `CS`?
- What happens when you call `new()`? (Ownership transfer)
- Why use `lis3dsh` crate vs manual implementation?

**LevelDetector (`src/level_algorithm.rs`):**

- How does EMA filter work? Write the formula.
- Why is alpha = 0.15? What if alpha = 0.9?
- What is `atan2(x, z)` calculating geometrically?
- Why invert signs for north/south mapping?

**LedController (`src/led_controller.rs`):**

- What is `PwmChannel<TIM4, 0>` vs `PwmChannel<TIM4, 1>`?
- Why store `max_duty`?
- How does duty cycle relate to brightness?

#### 2. **Understand the Key Concepts**

**Ownership:**

- Every value has ONE owner
- When owner goes out of scope, value is dropped
- You can borrow (`&T`) or mutably borrow (`&mut T`)
- Only ONE `&mut T` OR many `&T`, never both

**Traits:**

- Like interfaces in other languages
- Define shared behavior
- Zero-cost: monomorphized at compile time
- `embedded-hal` traits enable portability

**Type State Pattern:**

- Encode state in type system
- `Pin<Input>` vs `Pin<Output>` are different types
- Compiler enforces state transitions
- Makes invalid states unrepresentable

**`no_std`:**

- No heap (`Vec`, `String`, `Box`)
- No standard library
- Only `core` (primitives, `Option`, `Result`, traits)
- All allocation must be stack or static

#### 3. **Prepare for Questions**

**"Why did Rust work first try but C++ didn't?"**

- Driver crate encapsulated correct SPI protocol
- Type system caught initialization errors
- Borrow checker prevented simultaneous peripheral access
- Compiler enforced peripheral ownership

**"How do you handle interrupts in Rust?"**

- Critical section abstractions (`cortex_m::interrupt::free`)
- RTIC framework for interrupt-driven concurrency
- Type system prevents data races between ISRs and main
- `Mutex` types that are interrupt-safe

**"What about code size compared to C?"**

- With LTO and `opt-level="s"`, comparable to C
- Monomorphization can increase size (generic specialization)
- Can use `opt-level="z"` for even smaller binaries
- Trade-off: abstractions vs size (choose wisely)

**"How do you debug Rust embedded?"**

- `probe-rs` for flashing and GDB server
- `defmt` for efficient logging (formatting on host)
- Panic messages via RTT (Real-Time Transfer)
- Standard GDB/LLDB work fine

**"Can Rust call C libraries? Can C call Rust?"**

- Yes! FFI via `extern "C"`
- Zero overhead, uses C ABI
- Unsafe blocks required for FFI calls
- Can wrap unsafe C in safe Rust API

**"What's the state of Rust in aerospace/safety-critical?"**

- Ferrocene: Qualified compiler for ISO 26262 / IEC 61508
- ESA funding Rust for space applications
- Airbus exploring Rust for avionics
- NASA using Rust in some projects
- Growing but not yet mainstream

---

## Technical Deep Dives

### EMA Filter Math

```rust
// Exponential Moving Average
filtered[n] = α × raw[n] + (1 - α) × filtered[n-1]

// Where:
// α = smoothing factor (0 < α < 1)
// α = 0.15 means 15% new data, 85% old data
// Lower α = smoother but slower response
// Higher α = faster but more noise
```

**Why it works:**

- Each sample contributes exponentially less as it ages
- No buffer needed (unlike moving average)
- Constant memory (two floats)
- Fast (one multiply-add per axis)

### Tilt Angle Calculation

```
When board is level:
  X = 0g, Y = 0g, Z = 1g (gravity pointing down)

When tilted by angle θ around X-axis:
  X = sin(θ) × 1g
  Z = cos(θ) × 1g

Therefore:
  θ = atan2(X, Z)

atan2 handles quadrants correctly (unlike atan)
```

### SPI Mode 3 (CPOL=1, CPHA=1)

```
CPOL=1: Clock idle HIGH
CPHA=1: Data captured on SECOND edge (falling edge)

Timing:
  CS goes LOW
  CLK idles HIGH
  Data sampled on falling CLK edge
  Data shifted on rising CLK edge
  CS goes HIGH
```

**LIS3DSH Protocol:**

- Bit 7 = R/W (1=read, 0=write)
- Bit 6 = Multi-byte (NOT USED, use separate transfers)
- Bits 5-0 = Register address

**Correct multi-byte read:**

```rust
// Transfer 1: Send command
let cmd = 0x28 | 0x80;  // OUT_X_L | READ_BIT (no multi-byte!)
spi.transfer(&mut [cmd])?;

// Transfer 2: Read data
let mut data = [0u8; 6];
spi.transfer(&mut data)?;
```

### PWM Duty Cycle

```
Brightness (0-255) → Duty Cycle (0-max_duty)

duty = (brightness × max_duty) / 255

Example: 1 kHz PWM, 84 MHz timer clock
  Prescaler = 84 (gives 1 MHz timer)
  Period = 1000 (1 MHz / 1000 = 1 kHz)
  max_duty = 1000

  brightness=255 → duty=1000 (100%)
  brightness=128 → duty=502 (50%)
  brightness=0 → duty=0 (0%)
```

---

## Practice Talking Points

### Opening (30 seconds)

"Good morning. I built a digital level indicator to demonstrate Rust's value in embedded systems. The Rust implementation worked on first compile, while the C++ port required significant debugging. This isn't luck—it's Rust's compile-time guarantees preventing entire bug categories."

### Rust Value (1 minute)

"For embedded C++ experts, Rust offers memory safety without runtime cost. The borrow checker enforces rules you already follow manually—one mutable reference OR many immutable references. This prevents data races, use-after-free, and null pointer dereferences at compile time. The type system makes invalid hardware states unrepresentable."

### Architecture (1 minute)

"I structured the code in three layers: hardware abstraction for the accelerometer and LEDs, algorithm layer for filtering and tilt calculation, and control logic in the main loop. Each module is generic over traits from embedded-hal, making the code portable across different microcontrollers."

### SPI Story (1 minute)

"The C++ debugging story is revealing. I initially used a single SPI transaction with the multi-byte bit set. The sensor returned data but it never changed—frozen values. After examining the Rust lis3dsh driver source, I discovered it uses TWO separate transfers. This is the correct protocol for the LIS3DSH. The Rust driver crate prevented this mistake from the start."

### Closing (30 seconds)

"I bring Rust expertise your team doesn't have. For safety-critical systems, Rust's compile-time guarantees reduce debugging time and enable fearless refactoring. But I'm also fluent in C++—I can work in your existing codebase. Rust isn't a replacement, it's a force multiplier."

---

## Time Management (15-20 minutes)

- Slides 1-2: Opening (2 min)
- Slides 3-4: Why Rust + Ecosystem (5 min)
- Slides 5-10: Architecture Deep-Dive (8 min)
- Slide 11: SPI Story (3 min)
- Slides 12-13: Concepts + Results (3 min)
- Slide 14: Closing (1 min)
- Total: 18 minutes + Q&A

**Pacing Tips:**

- Don't rush the SPI debugging story—it's your key differentiator
- If time is short, skim slide 12 (advanced concepts)
- If they engage with questions mid-presentation, let it flow naturally
- Keep slide 15 (backup questions) ready but don't present unless asked

---

## Additional Resources

If they ask about specific topics:

- **RTIC Framework:** <https://rtic.rs> (interrupt-driven concurrency)
- **Embedded Rust Book:** <https://docs.rust-embedded.org/book/>
- **Ferrocene:** <https://ferrous-systems.com/ferrocene/> (qualified compiler)
- **embedded-hal:** <https://github.com/rust-embedded/embedded-hal>
- **probe-rs:** <https://probe.rs> (debugging tool)

---

## Final Checklist

Before the interview:

- [ ] Practice with speaker notes (press `S`)
- [ ] Run through each slide's talking points
- [ ] Be able to explain ownership, borrowing, traits
- [ ] Know the EMA formula and atan2 calculation
- [ ] Understand the SPI debugging story deeply
- [ ] Review backup slide for common questions
- [ ] Test presentation in the browser you'll use
- [ ] Have code repository open in another window for reference
- [ ] Prepare 2-3 questions about HighQ.aero's tech stack

Good luck! You've got this.
