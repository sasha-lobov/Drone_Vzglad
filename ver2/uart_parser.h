#ifndef UART_PARSER_H
#define UART_PARSER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define UART_BUF_SIZE 256
#define PACKET_LEN    11

typedef struct {
    float roll_sp, pitch_sp, yaw_sp, alt_sp;
    uint32_t last_tick;
    bool active;
} VisionCmd_t;

void UART_InitParser(UART_HandleTypeDef *huart);
bool UART_GetCommand(VisionCmd_t *cmd);
void UART_SendHeartbeat(float bat_mv, bool link_ok);

// Вызывается из HAL_UARTEx_RxEventCallback
void UART_IDLE_Handler(uint16_t size);

#endif
