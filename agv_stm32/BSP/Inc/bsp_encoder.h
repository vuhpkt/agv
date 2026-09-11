#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "main.h"
#include "encoder.h"
#include <stdint.h>

typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT = 1
} Encoder_ID_t;

void BSP_Encoder_Init(void);
void BSP_Encoder_Update(void);
int32_t BSP_Encoder_GetCount(Encoder_ID_t encoder);
int16_t BSP_Encoder_GetDelta(Encoder_ID_t encoder);
void BSP_Encoder_Reset(Encoder_ID_t encoder);
void BSP_Encoder_ResetAll(void);
encoder_t* BSP_Encoder_GetHandle(Encoder_ID_t encoder);

#endif /* BSP_ENCODER_H */
