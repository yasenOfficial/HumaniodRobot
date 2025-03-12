#include "DRV8825.h"

/**
 * @brief Simple busy-wait delay for microseconds.
 *        Note: This implementation is crude and depends on SystemCoreClock.
 */
void DRV8825_DelayMicroseconds(uint32_t us)
{
    volatile uint32_t count = us * (SystemCoreClock / 1000000 / 5);
    while(count--) {}
}

void DRV8825_Begin(DRV8825_HandleTypeDef *hdrv)
{
    // Enable the GPIO clocks for the used ports.
    // (Ensure these clocks are enabled. If not already done, for example, enable GPIOA clock:)
    // LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    // Similarly enable clocks for other GPIO ports as needed.

    // Configure the pins as output.
    LL_GPIO_SetPinMode(hdrv->dirPort, hdrv->dirPin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(hdrv->stepPort, hdrv->stepPin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(hdrv->enablePort, hdrv->enablePin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(hdrv->resetPort, hdrv->resetPin, LL_GPIO_MODE_OUTPUT);

    // Reset sequence (mirroring Arduino code):
    LL_GPIO_ResetOutputPin(hdrv->resetPort, hdrv->resetPin);   // Set reset LOW
    LL_GPIO_SetOutputPin(hdrv->enablePort, hdrv->enablePin);     // Set enable HIGH
    LL_mDelay(10);  // 10 ms delay

    LL_GPIO_SetOutputPin(hdrv->resetPort, hdrv->resetPin);      // Set reset HIGH
    LL_mDelay(5);   // 5 ms delay
    LL_GPIO_ResetOutputPin(hdrv->enablePort, hdrv->enablePin);   // Set enable LOW
    LL_mDelay(5);   // 5 ms delay

    // Set default direction (LOW)
    LL_GPIO_ResetOutputPin(hdrv->dirPort, hdrv->dirPin);
}

void DRV8825_SetDirection(DRV8825_HandleTypeDef *hdrv, uint8_t direction)
{
    if (direction)
    {
        LL_GPIO_SetOutputPin(hdrv->dirPort, hdrv->dirPin);
    }
    else
    {
        LL_GPIO_ResetOutputPin(hdrv->dirPort, hdrv->dirPin);
    }
}

void DRV8825_MoveMotor(DRV8825_HandleTypeDef *hdrv)
{
    for (int i = 0; i < hdrv->stepsPerRevolution; i++)
    {
        // Set step HIGH
        LL_GPIO_SetOutputPin(hdrv->stepPort, hdrv->stepPin);
        DRV8825_DelayMicroseconds(50);
        // Set step LOW
        LL_GPIO_ResetOutputPin(hdrv->stepPort, hdrv->stepPin);
        DRV8825_DelayMicroseconds(50);
    }
}
