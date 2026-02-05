/**
  ******************************************************************************
  * @file    main.cpp
  * @brief   Level indicator application - C++ port
  ******************************************************************************
  */

#include "main.h"
#include "rtt_log.h"

// Uncomment to enable full application
#define ENABLE_FULL_APP

#ifdef ENABLE_FULL_APP
#include "accelerometer.hpp"
#include "led_controller.hpp"
#include "level_algorithm.hpp"
#include "debug.hpp"
#endif

// Hardware handles
static TIM_HandleTypeDef htim4;

#ifdef ENABLE_FULL_APP
static SPI_HandleTypeDef hspi1;
static Accelerometer* accel = nullptr;
static LedController* leds = nullptr;
static LevelAlgorithm* level_detector = nullptr;
#endif

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);

#ifdef ENABLE_FULL_APP
static void MX_SPI1_Init(void);
#endif

int main(void)
{
    // HAL initialization
    if (HAL_Init() != HAL_OK) {
        Error_Handler();
    }

    // Initialize RTT for logging
    rtt_init();
    rtt_println("=== C++ Level Indicator ===");
    rtt_println("HAL initialized");

    // Configure system clock
    SystemClock_Config();
    rtt_println("System clock: 84 MHz");

    // Initialize peripherals
    MX_GPIO_Init();
    rtt_println("GPIO initialized");

    MX_TIM4_Init();
    rtt_println("TIM4 initialized for PWM");

    // Start PWM on all 4 channels
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    rtt_println("PWM started on all 4 channels");

    // Set all LEDs to full brightness
    uint32_t max_brightness = __HAL_TIM_GET_AUTORELOAD(&htim4);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, max_brightness);  // West
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, max_brightness);  // North
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, max_brightness);  // East
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, max_brightness);  // South
    rtt_println("All LEDs set to full brightness (max=%lu)", max_brightness);
    HAL_Delay(1000);

#ifdef ENABLE_FULL_APP
    // Full application code
    rtt_println("\n--- Starting Full Application ---");
    MX_SPI1_Init();
    rtt_println("SPI1 initialized");

    accel = new Accelerometer(&hspi1, GPIOE, GPIO_PIN_3);
    leds = new LedController(&htim4);
    level_detector = new LevelAlgorithm();
    rtt_println("Objects created");

    leds->init();
    rtt_println("LED controller initialized");

    rtt_println("About to call accel->init()...");
    bool accel_init_result = accel->init();
    rtt_println("accel->init() returned: %d", accel_init_result);

    if (!accel_init_result) {
        rtt_println("ERROR: Accelerometer init failed!");
        Error_Handler();
    }
    rtt_println("Accelerometer initialized successfully");
    rtt_println("Entering main loop...\n");

    // Main loop
    uint32_t loop_count = 0;
    while (1)
    {
        float x, y;
        if (accel->readXY(x, y)) {
            LedBrightness brightness = level_detector->update(x, y);
            leds->setBrightness(
                brightness.north,
                brightness.east,
                brightness.south,
                brightness.west
            );

            // Log every second
            if (loop_count % 100 == 0) {
                rtt_println("X: %.3f, Y: %.3f | LEDs: N=%d E=%d S=%d W=%d",
                           x, y,
                           brightness.north, brightness.east,
                           brightness.south, brightness.west);
            }
        }
        loop_count++;
        HAL_Delay(10);
    }
#else
    // Simple LED test - just keep them on
    rtt_println("\n--- LED Test Mode ---");
    rtt_println("All LEDs should be ON");
    rtt_println("Looping...\n");

    while (1)
    {
        HAL_Delay(1000);
    }
#endif
}

/**
  * @brief System Clock Configuration
  * 84 MHz from 8 MHz HSE
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Enable Power Control clock
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Configure HSE oscillator
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;   // 8 MHz / 8 = 1 MHz
    RCC_OscInitStruct.PLL.PLLN = 336; // 1 MHz * 336 = 336 MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // 336 MHz / 4 = 84 MHz
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Configure clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // 84 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // 42 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // 84 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    // CRITICAL: Update SystemCoreClock variable and reconfigure SysTick
    SystemCoreClockUpdate();

    // Reconfigure SysTick for new clock frequency (1ms tick)
    HAL_SYSTICK_Config(SystemCoreClock / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    // Ensure SysTick has proper priority
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief GPIO Initialization
  */
static void MX_GPIO_Init(void)
{
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

#ifdef ENABLE_FULL_APP
/**
  * @brief SPI1 Initialization
  * 10 MHz, Mode 3 (CPOL=1, CPHA=1)
  */
static void MX_SPI1_Init(void)
{
    rtt_println("SPI1: Configuring...");

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;  // CPOL = 1 (value = 2)
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;       // CPHA = 1 (value = 4)
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 84 MHz / 8 = 10.5 MHz
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    rtt_println("SPI1: Before HAL_SPI_Init:");
    rtt_println("  CLKPolarity = %d (SPI_POLARITY_HIGH = %d)",
                hspi1.Init.CLKPolarity, SPI_POLARITY_HIGH);
    rtt_println("  CLKPhase = %d (SPI_PHASE_2EDGE = %d)",
                hspi1.Init.CLKPhase, SPI_PHASE_2EDGE);
    rtt_println("  BaudRate = %d (SPI_BAUDRATEPRESCALER_8 = %d)",
                hspi1.Init.BaudRatePrescaler, SPI_BAUDRATEPRESCALER_8);

    HAL_StatusTypeDef status = HAL_SPI_Init(&hspi1);
    rtt_println("SPI1: HAL_SPI_Init returned %d (0=OK)", status);

    if (status != HAL_OK) {
        rtt_println("ERROR: HAL_SPI_Init failed!");
        Error_Handler();
    }

    // Read back actual hardware register values
    rtt_println("SPI1: Hardware register CR1 = 0x%04X", SPI1->CR1);
    rtt_println("  Bit 0 (CPHA): %d", (SPI1->CR1 & SPI_CR1_CPHA) ? 1 : 0);
    rtt_println("  Bit 1 (CPOL): %d", (SPI1->CR1 & SPI_CR1_CPOL) ? 1 : 0);
    rtt_println("  Bits 3-5 (BR): %d", (SPI1->CR1 >> 3) & 0x7);

    // Explicitly enable SPI peripheral (HAL should do this, but let's be sure)
    __HAL_SPI_ENABLE(&hspi1);
    rtt_println("SPI1: Explicitly enabled, CR1 = 0x%04X", SPI1->CR1);
    rtt_println("  CR1.SPE (should be 1): %d", (SPI1->CR1 & SPI_CR1_SPE) ? 1 : 0);

    rtt_println("SPI1: Init complete");
}
#endif

/**
  * @brief TIM4 Initialization
  * 1 kHz PWM on channels 1-4
  */
static void MX_TIM4_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    // TIM4 clock = 84 MHz (APB1 * 2 due to prescaler)
    // For 1 kHz PWM: prescaler = 0, period = 84000
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 84000 - 1;  // 84 MHz / 84000 = 1 kHz
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }

    // Configure PWM channels
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* Infinite loop */
    while (1)
    {
    }
}
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif
