cmake_minimum_required(VERSION 3.15)

# Project definition
project(level_indicator C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MCU-specific settings
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F407xx)
set(CPU_PARAMETERS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Compiler flags
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -fdata-sections
    -ffunction-sections
)

add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

set(CMAKE_ASM_FLAGS "-x assembler-with-cpp")

# Linker flags
add_link_options(
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nano.specs
    --specs=nosys.specs
)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(
    $<$<CONFIG:DEBUG>:-Og>
    $<$<CONFIG:DEBUG>:-g3>
    $<$<CONFIG:DEBUG>:-ggdb>
    $<$<CONFIG:RELEASE>:-Os>
    $<$<CONFIG:RELEASE>:-g>
)

# Definitions
add_compile_definitions(
    ${MCU_MODEL}
    USE_HAL_DRIVER
)

# Include directories
include_directories(
    Core/Inc
    App/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS_Device/Include
    Drivers/CMSIS/Core/Include
    RTT
)

# Source files
file(GLOB_RECURSE SOURCES
    "Core/Src/*.c"
    "App/Src/*.cpp"
    "App/Src/*.c"
)

# HAL driver sources (exclude templates)
file(GLOB HAL_SOURCES "Drivers/STM32F4xx_HAL_Driver/Src/*.c")
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*template\\.c$")
list(APPEND SOURCES ${HAL_SOURCES})

# RTT sources
file(GLOB RTT_SOURCES "RTT/*.c")
list(APPEND SOURCES ${RTT_SOURCES})

# Startup file
set(STARTUP_SCRIPT "Core/Src/startup_stm32f407xx.s")

# Linker script
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F407VGTx_FLASH.ld")

# Executable
add_executable(${CMAKE_PROJECT_NAME}.elf
    ${SOURCES}
    ${STARTUP_SCRIPT}
)

target_compile_options(${CMAKE_PROJECT_NAME}.elf PRIVATE
    ${CPU_PARAMETERS}
    $<$<COMPILE_LANGUAGE:C>:${CMAKE_C_FLAGS}>
    $<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CXX_FLAGS}>
)

target_link_options(${CMAKE_PROJECT_NAME}.elf PRIVATE
    ${CPU_PARAMETERS}
    -T${LINKER_SCRIPT}
    ${CMAKE_EXE_LINKER_FLAGS}
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -Wl,--end-group
)

# Print size after build
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
)

# Generate hex and bin files
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
)
