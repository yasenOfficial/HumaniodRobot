#ifndef DRV8825_H
#define DRV8825_H

#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_utils.h"  // For LL_mDelay()
#include "stm32f1xx.h"           // For SystemCoreClock

// Handle structure for DRV8825 configuration
typedef struct {
    GPIO_TypeDef *dirPort;
    GPIO_TypeDef *stepPort;
    GPIO_TypeDef *enablePort;
    GPIO_TypeDef *resetPort;
    uint32_t dirPin;
    uint32_t stepPin;
    uint32_t enablePin;
    uint32_t resetPin;
    uint16_t stepsPerRevolution;
} DRV8825_HandleTypeDef;


void DRV8825_Begin(DRV8825_HandleTypeDef *hdrv);

void DRV8825_SetDirection(DRV8825_HandleTypeDef *hdrv, uint8_t direction);

void DRV8825_MoveMotor(DRV8825_HandleTypeDef *hdrv);

void DRV8825_DelayMicroseconds(uint32_t us);

#endif // DRV8825_H
