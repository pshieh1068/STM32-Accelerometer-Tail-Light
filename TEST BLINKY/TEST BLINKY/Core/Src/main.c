#include "stm32f1xx_hal.h"

// Define I2C handle
I2C_HandleTypeDef hi2c1;

// Define LED pin
#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOC

void SystemClock_Config(void);
void MX_GPIO_Init();

int main(void) {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, 0);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, 1);
        HAL_Delay(200);

}
