#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "main.h"
#include "tb6612.h"
#include <stdint.h>
#include <stdbool.h>

#define BSP_MOTOR_MAX_SPEED 1000

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} Motor_ID_t;

void BSP_Motor_Init(void);
void BSP_Motor_SetSpeed(Motor_ID_t motor, int16_t speed);
void BSP_Motor_SetSpeeds(int16_t left_speed, int16_t right_speed);
void BSP_Motor_Stop(Motor_ID_t motor);
void BSP_Motor_Brake(Motor_ID_t motor);
void BSP_Motor_Standby(bool enable_standby);
tb6612_t* BSP_Motor_GetHandle(Motor_ID_t motor);

#endif /* BSP_MOTOR_H */
