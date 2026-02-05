#!/bin/bash
# Interactive GDB debugging for C++ level indicator

echo "=================================="
echo "GDB Debugging Mode"
echo "=================================="
echo ""
echo "This will start a GDB server and connect to it."
echo "Useful for stepping through code and inspecting variables."
echo ""
echo "Common GDB commands:"
echo "  break main              - Set breakpoint at main"
echo "  break Accelerometer::init - Break in accelerometer init"
echo "  continue (or c)         - Continue execution"
echo "  next (or n)             - Step to next line"
echo "  step (or s)             - Step into function"
echo "  print variable          - Print variable value"
echo "  print /x variable       - Print in hexadecimal"
echo "  info locals             - Show local variables"
echo "  backtrace (or bt)       - Show call stack"
echo "  quit                    - Exit GDB"
echo ""
echo "Starting GDB session..."
echo ""

# Start GDB with automatic commands
~/arm-toolchain/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi/bin/arm-none-eabi-gdb \
  -ex "target extended-remote | probe-rs gdb --chip STM32F407VG" \
  -ex "load" \
  -ex "monitor reset" \
  -ex "break main" \
  -ex "continue" \
  level_indicator.elf

