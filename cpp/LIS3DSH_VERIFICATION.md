# LIS3DSH Accelerometer - Verification Guide

## Register Addresses (from datasheet Table 17)

```
Register Name    | Address | Access | Description
-----------------|---------|--------|---------------------------
WHO_AM_I         | 0x0F    | R      | Device identification (should read 0x3F)
CTRL_REG4        | 0x20    | R/W    | Control register 4
CTRL_REG5        | 0x24    | R/W    | Control register 5
OUT_X_L          | 0x28    | R      | X-axis acceleration data (low byte)
OUT_X_H          | 0x29    | R      | X-axis acceleration data (high byte)
OUT_Y_L          | 0x2A    | R      | Y-axis acceleration data (low byte)
OUT_Y_H          | 0x2B    | R      | Y-axis acceleration data (high byte)
OUT_Z_L          | 0x2C    | R      | Z-axis acceleration data (low byte)
OUT_Z_H          | 0x2D    | R      | Z-axis acceleration data (high byte)
```

## SPI Protocol (from datasheet Section 6)

### SPI Mode

- **Mode 3**: CPOL=1 (clock idle high), CPHA=1 (data captured on second/falling edge)
- **Max frequency**: 10 MHz
- **Bit order**: MSB first
- **Chip Select**: Active LOW

### Read/Write Bit (MSB of address byte)

```
Bit 7 (RW) | Operation
-----------|----------
0          | WRITE
1          | READ
```

### Multi-byte Read (Bit 6 of address byte)

```
Bit 6 (MS) | Operation
-----------|----------
0          | Single byte
1          | Multiple bytes (auto-increment address)
```

## Current Implementation Analysis

### 1. WHO_AM_I Read (Line 87-101)

**What we send:**

```c
tx[0] = 0x0F | 0x80 = 0x8F  // Read WHO_AM_I (bit 7 = 1 for read)
tx[1] = 0x00                  // Dummy byte
```

**What we expect:**

```c
rx[0] = 0xXX  // Don't care (while sending address)
rx[1] = 0x3F  // WHO_AM_I value
```

**SPI Transaction:**

```
CS Low
MOSI: [0x8F] [0x00]
MISO: [0xXX] [0x3F]  <- Should be 0x3F
CS High
```

### 2. Register Write (Line 103-112)

**Example: CTRL_REG4 = 0x67**

```c
tx[0] = 0x20 | 0x00 = 0x20  // Write to CTRL_REG4 (bit 7 = 0 for write)
tx[1] = 0x67                 // Data to write
```

**SPI Transaction:**

```
CS Low
MOSI: [0x20] [0x67]
CS High
```

### 3. Multi-byte Read (Line 114-135)

**Reading OUT_X_L through OUT_Z_H (6 bytes):**

```c
tx[0] = 0x28 | 0x80 = 0xA8  // Read from OUT_X_L (bit 7 = 1)
                             // Note: Should also set bit 6 for multi-byte!
tx[1-6] = 0x00              // Dummy bytes
```

**⚠️ POTENTIAL ISSUE:** Missing multi-byte bit (bit 6)!

Should be:

```c
tx[0] = 0x28 | 0x80 | 0x40 = 0xE8  // Read + multi-byte
```

## Debugging Steps

### Step 1: Verify SPI Configuration

Add to init() before WHO_AM_I read:

```cpp
DEBUG_PRINTF("SPI Config:");
DEBUG_PRINTF("  CLKPolarity: %d (expect 1)", hspi_->Init.CLKPolarity);
DEBUG_PRINTF("  CLKPhase: %d (expect 1)", hspi_->Init.CLKPhase);
DEBUG_PRINTF("  BaudRate: %d", hspi_->Init.BaudRatePrescaler);
```

Expected values:

- CLKPolarity: `SPI_POLARITY_HIGH` (1)
- CLKPhase: `SPI_PHASE_2EDGE` (1)
- BaudRate: 8 (for 10.5 MHz)

### Step 2: Verify CS Pin State

Add to init():

```cpp
DEBUG_PRINTF("CS Pin before init: %d (expect 1)",
             HAL_GPIO_ReadPin(cs_port_, cs_pin_));
```

Should read 1 (HIGH) when idle.

### Step 3: Enhanced WHO_AM_I Read with Raw Data

Replace readRegister call with:

```cpp
uint8_t tx[2] = {0x8F, 0x00};
uint8_t rx[2] = {0, 0};

DEBUG_PRINTF("Sending: 0x%02X 0x%02X", tx[0], tx[1]);
chipSelectLow();
HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, tx, rx, 2, 100);
chipSelectHigh();
DEBUG_PRINTF("Received: 0x%02X 0x%02X (status=%d)", rx[0], rx[1], status);
DEBUG_PRINTF("WHO_AM_I value: 0x%02X (expected 0x3F)", rx[1]);
```

### Step 4: Check What You Actually Receive

Common issues and what they mean:

| rx[1] Value | Likely Cause |
|-------------|--------------|
| 0x00        | No SPI communication (check wiring, clock) |
| 0xFF        | SPI receiving garbage (check MISO connection) |
| Same as tx[1] | MISO/MOSI shorted or loopback enabled |
| Other value | Wrong chip or register address |
| 0x3F        | ✅ SUCCESS! |

### Step 5: Verify Pin Connections

**SPI1 Pins (from STM32F407VG datasheet):**

```
PA5 -> SPI1_SCK  (Clock)
PA6 -> SPI1_MISO (Master In, Slave Out)
PA7 -> SPI1_MOSI (Master Out, Slave In)
PE3 -> CS        (Chip Select, manual GPIO)
```

**LIS3DSH Pins (from accelerometer datasheet):**

```
Pin 1  -> CS   (connect to PE3)
Pin 2  -> MOSI (connect to PA7)
Pin 3  -> MISO (connect to PA6)
Pin 4  -> SCL  (connect to PA5)
```

### Step 6: Test with Known Good Values

Try reading other registers to verify SPI works:

```cpp
// Read CTRL_REG4 (should be 0x07 after reset, or 0x67 after we write it)
uint8_t ctrl_reg4 = 0;
readRegister(0x20, &ctrl_reg4);
DEBUG_PRINTF("CTRL_REG4: 0x%02X", ctrl_reg4);

// Read CTRL_REG5
uint8_t ctrl_reg5 = 0;
readRegister(0x24, &ctrl_reg5);
DEBUG_PRINTF("CTRL_REG5: 0x%02X", ctrl_reg5);
```

## Known Issue: Multi-byte Read

**Current code (Line 118):**

```cpp
uint8_t tx[7] = {static_cast<uint8_t>(OUT_X_L | READ_BIT), 0, 0, 0, 0, 0, 0};
```

**Should be (for auto-increment):**

```cpp
uint8_t tx[7] = {static_cast<uint8_t>(OUT_X_L | READ_BIT | 0x40), 0, 0, 0, 0, 0, 0};
//                                                           ^^^^
//                                                     Multi-byte bit
```

Without bit 6 set, the LIS3DSH will NOT auto-increment the address, so all reads will return OUT_X_L repeatedly.

## Quick Test in GDB

```gdb
# Break at WHO_AM_I read
break accelerometer.cpp:28

# Continue and inspect
continue
next
print /x tx[0]    # Should be 0x8F
print /x tx[1]    # Should be 0x00
next
print /x rx[0]    # Don't care
print /x rx[1]    # Should be 0x3F if working
print status      # Should be 0 (HAL_OK)

# Check CS pin
print cs_port_
print cs_pin_
print /x cs_port_->ODR  # Bit 3 should toggle during transaction
```

## What to Report

Run the firmware and tell me:

1. **What value does WHO_AM_I return?** (from RTT log)
2. **What does `rx[0]` and `rx[1]` show?**
3. **Does CS pin toggle?** (measure with multimeter or scope)
4. **SPI configuration values** from the debug output

This will tell us exactly where the issue is!
