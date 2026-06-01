#include "uart_parser.c"

#include <string.h>

static UART_HandleTypeDef *h_uart;
static uint8_t rx_buf[UART_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static VisionCmd_t cmd = {0};
static uint8_t parse_state = 0;
static uint8_t temp_pkt[PACKET_LEN];

static uint16_t CRC16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for(size_t i=0; i<len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for(int j=0; j<8; j++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

void UART_InitParser(UART_HandleTypeDef *huart) {
    h_uart = huart;
    __HAL_UART_ENABLE_IT(h_uart, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(h_uart, rx_buf, UART_BUF_SIZE);
}

bool UART_GetCommand(VisionCmd_t *out) {
    __disable_irq();
    if(cmd.active) { *out = cmd; cmd.active = false; }
    __enable_irq();
    return out->active;
}

void UART_SendHeartbeat(float bat_mv, bool link_ok) {
    uint8_t tx[6];
    tx[0] = 0xCC;
    tx[1] = link_ok ? 1 : 0;
    tx[2] = (uint8_t)((uint16_t)bat_mv >> 8);
    tx[3] = (uint8_t)((uint16_t)bat_mv);
    uint16_t c = CRC16(tx, 4);
    tx[4] = c >> 8; tx[5] = c;
    HAL_UART_Transmit_DMA(h_uart, tx, 6);
}

void UART_IDLE_Handler(uint16_t size) {
    if(size == 0) return;

    // Копируем новые данные во временный буфер
    uint8_t local_buf[UART_BUF_SIZE];
    uint16_t tail = (rx_head + size) % UART_BUF_SIZE;
    if(rx_head <= tail) {
        memcpy(local_buf, &rx_buf[rx_head], size);
    } else {
        uint16_t first_part = UART_BUF_SIZE - rx_head;
        memcpy(local_buf, &rx_buf[rx_head], first_part);
        memcpy(local_buf + first_part, rx_buf, size - first_part);
    }
    rx_head = tail;

    // Парсинг пакетов
    uint16_t pos = 0;
    while(pos <= size - PACKET_LEN) {
        if(local_buf[pos] == 0xAA && local_buf[pos+1] == 0x55) {
            uint16_t calc = CRC16(&local_buf[pos], PACKET_LEN-2);
            uint16_t recv = (local_buf[pos+PACKET_LEN-2]<<8) | local_buf[pos+PACKET_LEN-1];
            if(calc == recv) {
                __disable_irq();
                cmd.roll_sp  = (int16_t)((local_buf[pos+2]<<8)|local_buf[pos+3]) / 100.0f;
                cmd.pitch_sp = (int16_t)((local_buf[pos+4]<<8)|local_buf[pos+5]) / 100.0f;
                cmd.yaw_sp   = (int16_t)((local_buf[pos+6]<<8)|local_buf[pos+7]) / 100.0f;
                cmd.alt_sp   = local_buf[pos+8] / 10.0f;
                cmd.last_tick = HAL_GetTick();
                cmd.active = true;
                __enable_irq();
            }
            pos += PACKET_LEN; continue;
        }
        pos++;
    }

    // Перезапуск DMA с новой позиции
    HAL_UARTEx_ReceiveToIdle_DMA(h_uart, rx_buf, UART_BUF_SIZE);
}
