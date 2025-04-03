#include "stm32f1xx_hal.h"

// Define I2C handle
I2C_HandleTypeDef hi2c1;

// ADXL343 I2C Address (default 0x53 if ALT address pin is LOW)
#define ADXL343_ADDR 0x53 << 1  // Shifted left for STM32 HAL

// Register addresses for ADXL343
#define ADXL343_REG_POWER_CTL 0x2D  // Power control register
#define ADXL343_REG_DATAX0 0x32     // X-axis data register (LSB)
#define ADXL343_REG_INT_SOURCE 0x30 // Interrupt source register
#define ADXL343_REG_THRESH_ACT 0x24 // Activity threshold register
#define ADXL343_REG_ACT_INACT_CTL 0x27 // Activity control

// Define LED pin
#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOA

void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_I2C1_Init(void);
void ADXL343_Init(void);
uint8_t ADXL343_ReadRegister(uint8_t reg);
void ADXL343_WriteRegister(uint8_t reg, uint8_t value);
void Error_Handler(void);

int main(void) {
    // Initialize HAL library
    HAL_Init();
    // Configure system clock
    //SystemClock_Config();
    // Initialize GPIO and I2C peripherals
    //MX_GPIO_Init();
    //MX_I2C1_Init();
    // Initialize ADXL343 accelerometer
    //ADXL343_Init();

    // Flash the LED 3 times at startup
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
/*
    while (1) {
        // Read activity status from the interrupt source register
        uint8_t int_source = ADXL343_ReadRegister(ADXL343_REG_INT_SOURCE);

        // Check if Activity bit (D4) is set, indicating motion
        if (int_source & 0x10) {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);  // Turn ON LED
        } else {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET); // Turn OFF LED
        }

        HAL_Delay(100); // Small delay to avoid excessive polling
    }
}

void ADXL343_Init(void) {
    // Enable measurement mode
    ADXL343_WriteRegister(ADXL343_REG_POWER_CTL, 0x08);
    // Set activity threshold (adjust sensitivity)
    ADXL343_WriteRegister(ADXL343_REG_THRESH_ACT, 10); // 10 (~0.625g per LSB)
    // Enable activity detection on all axes
    ADXL343_WriteRegister(ADXL343_REG_ACT_INACT_CTL, 0x70); // Enable activity detection on X, Y, Z
}

void ADXL343_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    HAL_I2C_Master_Transmit(&hi2c1, ADXL343_ADDR, data, 2, HAL_MAX_DELAY);
}

uint8_t ADXL343_ReadRegister(uint8_t reg) {
    uint8_t value;
    HAL_I2C_Master_Transmit(&hi2c1, ADXL343_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, ADXL343_ADDR, &value, 1, HAL_MAX_DELAY);
    return value;
}

static void MX_I2C1_Init(void) {
    // Configure I2C peripheral
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_GPIO_Init(void) {
    // Enable clock for GPIOA
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure LED pin as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure High-Speed External (HSE) oscillator
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Configure system clock source and dividers
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    */
}
