# C++ Level Indicator - Debugging Guide

## LED Debug Patterns

The application uses LED blink patterns to indicate status without needing a debugger:

### Startup Checkpoints (Fast blinks on PD13/North LED)

- **1 blink**: HAL initialized
- **2 blinks**: Clock configured (84 MHz)
- **3 blinks**: GPIO initialized
- **4 blinks**: SPI1 initialized
- **5 blinks**: TIM4 initialized
- **6 blinks**: Objects created (Accel, LED, Level)
- **7 blinks**: LED controller initialized
- **8 blinks**: LED test complete
- **9 blinks**: Accelerometer initialized successfully

After checkpoint 9, you should see **3 quick blinks** indicating the system is ready and entering the main loop.

### Error Patterns (Repeating)

- **2 long blinks (500ms)**: Accelerometer initialization failed
  - Check SPI connections
  - Verify WHO_AM_I register reads 0x3F

- **3 short blinks (200ms)**: Accelerometer read errors (10+ consecutive failures)
  - SPI communication issue
  - Check CS pin, clock polarity/phase

### Normal Operation

When working correctly:
- All 4 LEDs turn on briefly at startup (500ms test)
- LEDs respond to board tilt
- No blinking error patterns

## Manual Debugging Steps

### 1. Flash with Debug Build

```bash
cd /Users/mattheworris/projects/aero-app/cpp/build
probe-rs run --chip STM32F407VG level_indicator.elf
```

### 2. Observe LED Pattern

Watch the North LED (PD13) for checkpoint blinks:

```bash
# Count the fast blinks (100ms each)
# The number tells you how far the code progressed

# If it stops at checkpoint 4: SPI init issue
# If it stops at checkpoint 8: Accelerometer init issue
# If repeating error pattern: See error patterns above
```

### 3. Use GDB for Detailed Debugging

```bash
# Terminal 1: Start probe-rs in GDB mode
cd /Users/mattheworris/projects/aero-app/cpp/build
probe-rs gdb --chip STM32F407VG level_indicator.elf

# Terminal 2: Connect with GDB
~/arm-toolchain/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi/bin/arm-none-eabi-gdb level_indicator.elf

# In GDB:
(gdb) target remote :1337
(gdb) monitor reset halt
(gdb) load
(gdb) break main
(gdb) continue

# Inspect accelerometer init:
(gdb) break Accelerometer::init
(gdb) continue
(gdb) step
(gdb) print who_am_i
(gdb) print /x who_am_i  # Hex format

# Check SPI handle:
(gdb) print hspi1
(gdb) print *hspi1.Instance

# Watch variables:
(gdb) watch x
(gdb) watch y
(gdb) continue
```

### 4. Check Hardware Connections

**SPI1 (Accelerometer):**
- PA5 → SCK (clock)
- PA6 → MISO (data from sensor)
- PA7 → MOSI (data to sensor)
- PE3 → CS (chip select, active low)
- Mode 3: CPOL=1, CPHA=1

**TIM4 (LEDs):**
- PD12 → West LED
- PD13 → North LED
- PD14 → East LED
- PD15 → South LED

**Power:**
- VDD → 3.3V
- GND → Ground

### 5. Logic Analyzer Debugging

If you have a logic analyzer:

```bash
# Capture SPI signals on:
# - PA5 (SCK)
# - PA6 (MISO)
# - PA7 (MOSI)
# - PE3 (CS)

# Expected WHO_AM_I transaction:
# CS Low → Send 0x8F (read WHO_AM_I) → Receive 0x3F → CS High
```

### 6. Live Variable Inspection

For printf-style debugging (requires semihosting):

```bash
# Rebuild with semihosting:
# Edit CMakeLists.txt, add: -DDEBUG_SEMIHOSTING
# Change --specs=nosys.specs to --specs=rdimon.specs

# Run with semihosting:
probe-rs run --chip STM32F407VG level_indicator.elf

# You'll see printf output like:
# "Accel: Reading WHO_AM_I register..."
# "Accel: WHO_AM_I = 0x3F (expected 0x3F)"
# "Loop: 100, X: 0.012, Y: -0.985"
```

## Common Issues

### Issue: Stuck at checkpoint 8 (accelerometer init)

**Symptoms:**
- Checkpoints 1-8 blink normally
- Then 2 long blinks repeating

**Causes:**
- WHO_AM_I register not reading 0x3F
- SPI not configured correctly (wrong mode/speed)
- CS pin not working
- Accelerometer not powered

**Debug:**
```bash
# Check SPI configuration
(gdb) print hspi1.Init.CLKPolarity   # Should be SPI_POLARITY_HIGH
(gdb) print hspi1.Init.CLKPhase      # Should be SPI_PHASE_2EDGE

# Check CS pin state
(gdb) print GPIOE->ODR  # Bit 3 should be high when idle
```

### Issue: LEDs don't turn on

**Symptoms:**
- Checkpoints work
- No LED test at startup

**Causes:**
- TIM4 not running
- GPIO alternate function not configured
- PWM duty cycle incorrect

**Debug:**
```bash
(gdb) print htim4.Init.Period  # Should be 83999
(gdb) print htim4.Instance->CR1  # Bit 0 (CEN) should be 1
(gdb) print GPIOD->MODER  # PD12-15 should be alternate function (10)
```

### Issue: LEDs don't respond to tilt

**Symptoms:**
- LEDs stay at same brightness
- No response to movement

**Causes:**
- Accelerometer reads returning same values
- Accelerometer not enabled
- EMA filter not updating

**Debug:**
```bash
(gdb) break level_detector->update
(gdb) continue
# Tilt the board
(gdb) print x
(gdb) print y
(gdb) print this->filtered_x_
(gdb) print this->filtered_y_

# Check if values change when you tilt
```

## Quick Test Procedure

1. **Flash the firmware**
   ```bash
   probe-rs run --chip STM32F407VG level_indicator.elf
   ```

2. **Count checkpoint blinks** on North LED
   - Should reach checkpoint 9

3. **Look for 3 quick blinks** (ready signal)

4. **Verify LED test**
   - All 4 LEDs should turn on for 500ms then off

5. **Test tilt response**
   - Tilt board north → South LED should dim
   - Tilt board south → North LED should dim
   - Tilt board east → West LED should dim
   - Tilt board west → East LED should dim
   - Level board → All LEDs bright

6. **If not working**, note which checkpoint it stops at and refer to issues above

## Comparing with Rust Version

To verify behavior matches:

```bash
# Flash Rust version
cd /Users/mattheworris/projects/aero-app
cargo rb level_indicator

# Test tilt behavior, note LED response

# Flash C++ version
cd cpp/build
probe-rs run --chip STM32F407VG level_indicator.elf

# Compare tilt behavior
# Both should respond identically
```

## Advanced: Using probe-rs Tracing

```bash
# Enable RTT (Real-Time Transfer) for printf-style debugging
# Requires modifying code to use probe-rs-rtt instead of semihosting

# Monitor RTT output:
probe-rs run --chip STM32F407VG level_indicator.elf --log-format "{t} {L} {s}"
```

## Getting Help

If debugging shows unexpected behavior:

1. Note the checkpoint number where it stops
2. Note any error patterns (blink counts)
3. Capture logic analyzer traces if available
4. Check hardware connections with multimeter
5. Compare register values with Rust working version
