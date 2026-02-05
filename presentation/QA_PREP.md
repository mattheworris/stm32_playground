# Interview Q&A Preparation

Likely questions and strong answers to demonstrate your expertise.

---

## Technical Implementation Questions

### Q: "Walk me through how your accelerometer reading works at the hardware level."

**Answer:**

"The LIS3DSH communicates over SPI Mode 3—that's clock idle high, data sampled on falling edge. Here's the sequence:

1. I pull CS (chip select) low to activate the sensor
2. Send a command byte: 0x28 with the read bit (0x80), so 0xA8
3. The sensor auto-increments the address, so I can read 6 bytes in sequence
4. Receive X_LOW, X_HIGH, Y_LOW, Y_HIGH, Z_LOW, Z_HIGH
5. Pull CS high to end the transaction
6. Combine the bytes (little-endian): X = (X_HIGH << 8) | X_LOW

The sensor returns 16-bit signed integers in ±2g range. I divide by 16,384 to convert to g-force units—that's the fixed-point scale factor for this sensitivity.

The critical detail I learned: the LIS3DSH requires TWO separate SPI transfers—one to send the command, one to receive data. Using a single transaction with the multi-byte bit doesn't work reliably. This is what caused the bug in the C++ version."

**Why this answer works:**

- Shows low-level hardware understanding
- Demonstrates debugging experience
- Explains the SPI protocol issue clearly
- Uses specific numbers and register addresses

---

### Q: "How does your filtering algorithm work, and why did you choose it?"

**Answer:**

"I use an Exponential Moving Average filter with alpha = 0.15. The formula is:

  filtered[n] = 0.15 × new_sample + 0.85 × filtered[n-1]

This means each new reading contributes 15%, and the accumulated history contributes 85%.

Why EMA over a simple moving average? Two reasons:

1. **Constant memory**: I only need to store one float per axis, not a buffer of samples. In embedded, memory is precious.

2. **O(1) computation**: One multiply-add per axis, every update. A moving average would require summing N samples—O(N) every time.

I chose alpha = 0.15 through testing. Lower values (like 0.05) were too sluggish—the LEDs felt unresponsive. Higher values (like 0.5) let too much noise through—the LEDs would flicker. 0.15 gives smooth, stable output while still feeling responsive to user input.

Additionally, I have a 3-degree deadband around level. Within ±3°, brightness stays at maximum. This prevents LED flutter from tiny vibrations or hand tremor."

**Why this answer works:**

- Shows understanding of trade-offs (memory vs. computation)
- Demonstrates iterative testing and tuning
- Explains the math clearly
- Considers user experience, not just technical correctness

---

### Q: "Your C++ port had bugs but the Rust version didn't. Was that just luck?"

**Answer:**

"No, it's Rust's compile-time guarantees working exactly as designed. Let me give specific examples:

**SPI Protocol Bug in C++:**
I tried `HAL_SPI_TransmitReceive` with a 7-byte buffer, setting the multi-byte bit. The code compiled fine—C++ has no way to know this is wrong. The sensor returned data, but it never changed. After debugging with an oscilloscope and reading the Rust driver source, I discovered the LIS3DSH needs TWO separate transactions.

In Rust, I used the `lis3dsh` crate, which encodes the correct protocol. The crate author already solved this problem. Rust's culture emphasizes reusable, well-tested libraries.

**Initialization Order in C++:**
If you initialize SPI before enabling the clock, the behavior is undefined. C++ compiles it. Rust won't—the HAL requires `&mut rcc` to prove clocks are configured.

**Peripheral Conflicts:**
If you try to create two mutable references to SPI1, Rust's borrow checker stops you at compile time. C++ lets you do it—undefined behavior at runtime.

The pattern: Rust catches entire categories of bugs before the code ever runs. C++ relies on developer discipline and runtime debugging."

**Why this answer works:**

- Specific technical examples, not vague claims
- Shows you debugged the C++ version (oscilloscope detail)
- Explains WHY Rust prevented bugs (borrow checker, type system)
- Balanced—not bashing C++, just explaining differences

---

### Q: "How would you handle interrupt-driven I/O in Rust?"

**Answer:**

"Great question. This project uses a simple polling loop, but for production I'd use the RTIC framework—Real-Time Interrupt-driven Concurrency.

RTIC gives you:

1. **Tasks with priorities**: Each interrupt handler is a task. RTIC statically analyzes which tasks can run concurrently and ensures data race freedom.

2. **Resource locking**: Shared state between interrupts uses RTIC's resource system. The framework computes the minimum priority level to mask interrupts, preventing races with zero overhead.

3. **Compile-time scheduling**: All priority analysis happens at compile time. No runtime scheduler.

Example structure:
```rust
#[task(bindings = [SPI1], priority = 2)]
fn spi_handler(cx: spi_handler::Context) {
    // Access hardware
    // Share data with other tasks via resources
}

#[task(priority = 1)]
fn main_task(cx: main_task::Context) {
    // Lower priority, can be preempted
}
```

If `spi_handler` and `main_task` both access shared state, RTIC's macros insert critical sections only where needed—computed at compile time.

Alternative: For simpler needs, `cortex_m::interrupt::free(|_| { ... })` creates a critical section by disabling interrupts."

**Why this answer works:**

- Shows knowledge beyond the current project
- Mentions specific framework (RTIC)
- Explains the safety guarantees
- Provides code example for clarity

---

### Q: "What happens when Rust code panics on an embedded system?"

**Answer:**

"In `no_std` Rust, you must provide a panic handler since there's no OS to catch it. I use the `panic-probe` crate:

```rust
use panic_probe as _;
```

When a panic occurs—like `.unwrap()` on `None` or array out of bounds—the panic handler:

1. Logs the panic message via defmt (over RTT)
2. Triggers a breakpoint instruction (`bkpt`)
3. Halts the processor

If a debugger is attached, it stops at the panic location. If not, the chip freezes (safe failure mode).

In production, you'd use different handlers:

- `panic-abort`: Terminate and reset
- `panic-halt`: Spin loop forever (for debugging)
- `panic-rtt-target`: Log and halt, for post-mortem analysis
- Custom handler: Write to EEPROM, blink an LED pattern, trigger watchdog reset

The key: panics are deterministic. If code panics in testing, it will panic in production—so you find bugs before deployment. Compare to C++ undefined behavior that might work in debug and fail in release."

**Why this answer works:**

- Specific about the panic mechanism
- Shows you understand production vs. development needs
- Contrasts with C++ undefined behavior
- Demonstrates thinking about failure modes

---

## Rust Language Questions

### Q: "Explain Rust's ownership system to someone who knows C++."

**Answer:**

"In C++, you manage lifetimes manually:

- `new`/`delete` for heap
- Smart pointers like `unique_ptr`, `shared_ptr` for RAII
- Raw pointers for flexibility (and danger)

Rust enforces three rules at compile time:

1. **Every value has exactly one owner**. When you pass a value to a function, ownership transfers unless you explicitly borrow.

2. **When the owner goes out of scope, the value is dropped**. Like C++ destructors, but guaranteed—no leaks unless you explicitly opt out with `forget()`.

3. **You can borrow in two ways**:
   - Many immutable borrows (`&T`)
   - ONE mutable borrow (`&mut T`)
   - Never both simultaneously

Example in embedded:
```rust
let spi = Spi::new(peripherals.SPI1);
let accel = Accelerometer::new(spi);  // spi moved
// Can't use spi here anymore—compiler error
```

This prevents:

- Use after free (no dangling pointers)
- Double free (can't drop twice)
- Data races (multiple writers or reader during write)

For embedded, this is huge: no null pointer dereferences, no buffer overflows (checked at compile time), no concurrent access bugs."

**Why this answer works:**

- Starts from C++ knowledge they have
- Three clear rules
- Concrete embedded example
- Lists specific prevented bug types

---

### Q: "How does Rust achieve zero-cost abstraction?"

**Answer:**

"Zero-cost abstraction means 'what you don't use, you don't pay for'—and 'what you do use is as fast as hand-written code.'

Rust achieves this through monomorphization and compile-time evaluation:

**Monomorphization:**
When you write a generic function:
```rust
fn read<SPI: Transfer<u8>>(spi: &mut SPI) { ... }
```

The compiler generates specialized versions for each concrete type used:
```rust
read(&mut stm32_spi);  // Generates optimized code for STM32
read(&mut nrf_spi);    // Generates different optimized code for nRF52
```

No vtables, no dynamic dispatch, no runtime cost. Each version is as if you hand-coded it for that type.

**Const evaluation:**
Things like channel indices in `PwmChannel<TIM4, 0>` are compile-time constants. The compiler optimizes them away completely—zero runtime cost.

**Example from my code:**
The LED controller's `set_brightness` uses a closure to scale values. The compiler inlines it—examining the assembly, it's identical to writing the math four times manually.

This is how Rust can have nice abstractions (traits, generics) that compile to the same code as low-level C."

**Why this answer works:**

- Explains the mechanism (monomorphization)
- Concrete code example
- References actual assembly inspection
- Compares to C (their baseline)

---

### Q: "What are the downsides of Rust for embedded?"

**Answer:**

"I'll be honest about the trade-offs:

**1. Learning curve:**
Rust's borrow checker is unfamiliar if you're coming from C. You fight the compiler initially. But once you internalize the rules, it becomes a productivity tool—it catches bugs you'd spend hours debugging.

**2. Compile times:**
Rust is slower to compile than C. Monomorphization generates specialized code for every generic instantiation. On a large project, incremental builds help, but cold builds are slow.

**3. Ecosystem maturity:**
For STM32, nRF, ESP32, the ecosystem is great. For more obscure MCUs, you might need to write your own HAL or use auto-generated PAC bindings. C/C++ has decades of vendor support.

**4. Binary size with generics:**
Each monomorphized instance increases code size. You can mitigate with `opt-level = 's'` and LTO, but it's something to watch.

**5. Certification:**
Safety-critical aerospace needs qualified tools. Ferrocene is working on ISO 26262/IEC 61508 qualification, but it's not as mature as MISRA C compliance.

That said: the bugs Rust prevents are worth the trade-offs. I'd rather spend time fighting the compiler than debugging a HardFault from a null pointer."

**Why this answer works:**

- Honest about real downsides (shows maturity)
- Provides context and mitigations
- Balances critique with value judgment
- Relevant to their domain (certification)

---

## Behavioral/Cultural Questions

### Q: "Why do you want to work at HighQ.aero?"

**Answer:**

"Three reasons:

**1. Safety-critical embedded systems align with Rust's strengths.**
Rust's memory safety guarantees are exactly what aerospace needs. I want to work where those guarantees matter—where bugs don't just crash an app, they have real consequences.

**2. I bring skills your team doesn't have.**
You're experts in embedded C++. I bring Rust expertise. For new development, especially safety-critical modules, I can prototype in Rust and demonstrate the productivity and safety benefits. If needed, I can also work in your existing C++ codebase—I've ported code both directions.

**3. I want to be challenged.**
This is complex work: real-time constraints, hardware abstraction, signal processing. That's exactly what I want to be doing. I learn best when I'm solving hard problems with real stakes."

**Why this answer works:**

- Specific to their company (aerospace, safety)
- Shows what you bring (unique skill)
- Demonstrates motivation (challenge)
- Humble but confident

---

### Q: "Have you ever had to debug a really difficult embedded bug? Walk me through it."

**Answer:**

"Yes—the SPI freezing issue in the C++ port.

**Problem:** Accelerometer initialization succeeded (WHO_AM_I returned 0x3F), but when reading sensor data, the values never changed. Always the same bytes: `FF E9 A7 9E CC 49 E2`.

**Initial hypothesis:** Timing issue. Maybe I was reading too fast, before new data was ready. I added delays—no change.

**Second hypothesis:** Wrong register address. I triple-checked the datasheet—0x28 for OUT_X_L was correct.

**Breakthrough:** I hooked up a logic analyzer to the SPI bus. The transactions looked correct at the protocol level—correct clock polarity, correct chip select timing. But I noticed the command byte was 0xE8, not 0xA8. That's because I was setting the multi-byte bit (0x40).

**Investigation:** I went to the Rust `lis3dsh` crate source code. I expected it would use the multi-byte bit for efficiency. But it didn't—it used TWO separate `transfer()` calls.

**Root cause:** The LIS3DSH auto-increments addresses internally. The multi-byte bit (bit 6) apparently interferes with this. Two separate transactions work: send command byte 0xA8, then receive 6 bytes.

**Fix:** Changed from one `TransmitReceive` call to `Transmit` then `Receive`. Data started updating immediately.

**Lesson:** When debugging, look at working code. Open-source drivers are a goldmine—they've already solved the protocol quirks."

**Why this answer works:**

- Structured narrative (problem → hypotheses → breakthrough)
- Shows systematic debugging
- Mentions tools (logic analyzer)
- Explains root cause
- Extracts a lesson (read working code)

---

### Q: "How do you stay current with Rust and embedded development?"

**Answer:**

"Several ways:

**1. I read embedded Rust books and blogs:**

- The Embedded Rust Book (official documentation)
- Ferrous Systems blog (they're leading Rust in embedded)
- James Munns' blog (embedded-hal maintainer)

**2. I study crates.io source code:**
When I use a crate like `lis3dsh`, I read its source. It's often the best documentation for how to structure embedded code idiomatically.

**3. I follow the Embedded Working Group:**
They meet regularly to discuss HAL design, ecosystem improvements, etc. Their Matrix channel is active.

**4. I build projects:**
This level indicator is one example. I learn best by implementing real hardware. Next, I want to tackle CAN bus communication or a simple RTOS.

**5. I'm active in the Rust community:**
I read r/rust, watch talks from RustConf and Oxidize (embedded Rust conference).

The embedded Rust ecosystem moves fast, so staying engaged is important."

**Why this answer works:**

- Specific resources (not vague "I read blogs")
- Shows proactive learning (community involvement)
- Mentions next project (forward-looking)
- Balance of theory and practice

---

## Questions YOU Should Ask THEM

### About the team:

**"What embedded platforms do you primarily work with? Any plans to explore new architectures?"**

- Shows interest in their tech stack
- Opens door to discuss Rust's portability

**"How do you handle code review and testing for safety-critical code?"**

- Demonstrates you're thinking about their domain
- Learn about their quality process

**"What's the biggest embedded systems challenge your team is currently tackling?"**

- Shows you want to contribute to real problems
- Gauges complexity and scope of work

### About Rust:

**"Is there existing Rust code in the codebase, or would I be pioneering that?"**

- Sets expectations about your role
- Understand if you'll be alone or have support

**"How does HighQ.aero think about introducing new technologies? Is there room for experimentation?"**

- Understand culture (conservative vs. innovative)
- Assess if Rust adoption is realistic

**"What's the certification path for flight software here? Any experience with qualified Rust toolchains?"**

- Shows you understand aerospace constraints
- Learn if Ferrocene or similar is on their radar

### About growth:

**"What does career progression look like for senior embedded engineers here?"**

- Shows you're thinking long-term
- Understand if this is a growth opportunity

**"Are there opportunities to present technical work, either internally or at conferences?"**

- Demonstrates ambition
- Learn about their visibility/recognition culture

---

## Final Prep Checklist

Before the interview:

- [ ] Can explain ownership, borrowing, traits without notes
- [ ] Know the EMA formula and can derive it
- [ ] Understand SPI Mode 3 timing diagram
- [ ] Can describe the type-state pattern with example
- [ ] Ready to walk through each line of accelerometer code
- [ ] Have 2-3 questions prepared for them
- [ ] Know your "why HighQ.aero" answer
- [ ] Practiced the SPI debugging story out loud
- [ ] Reviewed what Ferrocene is (certification)
- [ ] Can explain when to use `&` vs `&mut` vs ownership transfer

---

## Mindset Tips

**1. You're interviewing them too.**
This is a two-way evaluation. Are they excited about Rust? Do they ask good technical questions? You want a team that values learning.

**2. It's okay to say "I don't know, but here's how I'd find out."**
If they ask about something you haven't encountered (e.g., "How does Rust handle misaligned memory access?"), be honest: "I haven't faced that specific issue, but I'd start by reading the compiler docs and checking if there's a lint or sanitizer for it."

**3. Show your process, not just answers.**
They want to see how you think. Walk through your reasoning: "First, I'd check X. If that didn't work, I'd try Y. As a last resort, Z."

**4. Enthusiasm is contagious.**
Your excitement about Rust and embedded systems is part of your value. Don't hide it—they're hiring for culture fit too.

**5. Be yourself.**
You built something cool. You learned from debugging. You understand the trade-offs. That's exactly what they're looking for.

---

Good luck! You're going to do great.
