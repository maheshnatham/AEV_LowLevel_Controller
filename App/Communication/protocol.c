#include "protocol.h"
#include "usart.h" // Includes auto-generated huart3 or huart1
#include <string.h>

#define RX_DMA_BUF_SIZE 128
#define MAX_PAYLOAD_LEN 32

static uint8_t rx_dma_buf[RX_DMA_BUF_SIZE];
static uint32_t rx_read_ptr = 0;

volatile ControlPayload_t latest_command = {0};
static uint32_t last_valid_msg_time = 0; // Tracks the 200ms watchdog
static uint32_t system_uptime_ms = 0;

// Parser State Machine
typedef enum {
    WAIT_SOF1, WAIT_SOF2, READ_MSG_ID, READ_SEQ, READ_LEN,
    READ_PAYLOAD, READ_CRC_L, READ_CRC_H, WAIT_EOF1, WAIT_EOF2
} ParserState_t;

static ParserState_t state = WAIT_SOF1;
static uint8_t msg_id, seq, payload_len, payload_idx;
static uint8_t payload_buf[MAX_PAYLOAD_LEN];
static uint8_t crc_low, crc_high;

// Standard CRC-16-CCITT (Poly 0x1021, Init 0xFFFF)
uint16_t Comm_CalculateCRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc = (crc << 1);
        }
    }
    return crc;
}

void Comm_Init(void) {
    // Start circular DMA reception in the background
    // IMPORTANT: Change &huart3 to match your selected UART (e.g., &huart1)
    HAL_UART_Receive_DMA(&huart3, rx_dma_buf, RX_DMA_BUF_SIZE);
}

volatile uint32_t comm_rx_bytes = 0;
volatile uint32_t comm_rx_packets = 0;
volatile uint32_t comm_rx_crc_errs = 0;

void Comm_Update_100Hz(void) {
    system_uptime_ms += 10;

    // Self-healing Watchdog: If UART DMA reception was aborted by line noise, ST-Link boot break,
    // or framing error, clear error flags and re-arm DMA reception automatically.
    if (huart3.RxState != HAL_UART_STATE_BUSY_RX) {
        __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
        rx_read_ptr = 0;
        state = WAIT_SOF1;
        HAL_UART_Receive_DMA(&huart3, rx_dma_buf, RX_DMA_BUF_SIZE);
        return;
    }

    // Determine how many bytes the DMA has written since we last checked
    // Ndtr holds the number of bytes REMAINING in the DMA buffer
    uint32_t dma_write_ptr = (RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx)) % RX_DMA_BUF_SIZE;

    while (rx_read_ptr != dma_write_ptr) {
        uint8_t b = rx_dma_buf[rx_read_ptr];
        rx_read_ptr = (rx_read_ptr + 1) % RX_DMA_BUF_SIZE;
        comm_rx_bytes++;

        switch (state) {
            case WAIT_SOF1:   if (b == 0xAA) state = WAIT_SOF2; break;
            case WAIT_SOF2:   state = (b == 0x55) ? READ_MSG_ID : WAIT_SOF1; break;
            case READ_MSG_ID: msg_id = b; state = READ_SEQ; break;
            case READ_SEQ:    seq = b; state = READ_LEN; break;
            case READ_LEN:
                payload_len = b;
                if (payload_len > MAX_PAYLOAD_LEN) state = WAIT_SOF1;
                else { payload_idx = 0; state = READ_PAYLOAD; }
                break;
            case READ_PAYLOAD:
                payload_buf[payload_idx++] = b;
                if (payload_idx >= payload_len) state = READ_CRC_L;
                break;
            case READ_CRC_L:  crc_low = b; state = READ_CRC_H; break;
            case READ_CRC_H:  crc_high = b; state = WAIT_EOF1; break;
            case WAIT_EOF1:   state = (b == 0x55) ? WAIT_EOF2 : WAIT_SOF1; break;
            case WAIT_EOF2:
                if (b == 0xAA) {
                    // Packet fully received. Verify CRC.
                    uint8_t crc_calc_buf[3 + MAX_PAYLOAD_LEN];
                    crc_calc_buf[0] = msg_id;
                    crc_calc_buf[1] = seq;
                    crc_calc_buf[2] = payload_len;
                    memcpy(&crc_calc_buf[3], payload_buf, payload_len);

                    uint16_t calc_crc = Comm_CalculateCRC16(crc_calc_buf, 3 + payload_len);
                    uint16_t rx_crc = ((uint16_t)crc_high << 8) | crc_low;

                    if (calc_crc == rx_crc && msg_id == MSG_ID_CMD_CONTROL && payload_len == sizeof(ControlPayload_t)) {
                        memcpy((void*)&latest_command, payload_buf, sizeof(ControlPayload_t));
                        last_valid_msg_time = system_uptime_ms; // Feed Watchdog
                        comm_rx_packets++;
                    } else {
                        comm_rx_crc_errs++;
                    }
                }
                state = WAIT_SOF1;
                break;
        }
    }
}

bool Comm_IsLinkActive(void) {
    // 200ms Failsafe Watchdog
    return (system_uptime_ms - last_valid_msg_time) <= COMM_TIMEOUT_MS;
}

uint32_t Comm_GetWatchdogRemainingMs(void) {
    if (last_valid_msg_time == 0 || (system_uptime_ms - last_valid_msg_time) >= COMM_TIMEOUT_MS) {
        return 0;
    }
    return (uint32_t)(COMM_TIMEOUT_MS - (system_uptime_ms - last_valid_msg_time));
}

ControlPayload_t Comm_GetLatestCommand(void) {
    return latest_command;
}

// Telemetry & Test transmission buffers
static uint8_t tx_seq = 0;
static uint8_t tx_frame_buf[64];

bool Comm_SendTelemetry(const TelemetryPayload_t *telemetry) {
    if (telemetry == NULL) return false;

    // Do not clobber if UART TX is currently busy
    if (huart3.gState != HAL_UART_STATE_READY) {
        return false;
    }

    uint8_t payload_len = (uint8_t)sizeof(TelemetryPayload_t);
    uint8_t idx = 0;

    tx_frame_buf[idx++] = 0xAA; // SOF1
    tx_frame_buf[idx++] = 0x55; // SOF2
    tx_frame_buf[idx++] = MSG_ID_TLM_STATUS; // 0x81
    tx_frame_buf[idx++] = tx_seq++;
    tx_frame_buf[idx++] = payload_len;

    memcpy(&tx_frame_buf[idx], telemetry, payload_len);
    idx += payload_len;

    // CRC-16 calculated over [MSG_ID, SEQ, LEN, PAYLOAD] (starts at byte index 2, length 3 + payload_len)
    uint16_t crc = Comm_CalculateCRC16(&tx_frame_buf[2], 3 + payload_len);
    tx_frame_buf[idx++] = (uint8_t)(crc & 0xFF);        // CRC Low
    tx_frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF); // CRC High
    tx_frame_buf[idx++] = 0x55; // EOF1
    tx_frame_buf[idx++] = 0xAA; // EOF2

    // Non-blocking transmission via interrupt
    return (HAL_UART_Transmit_IT(&huart3, tx_frame_buf, idx) == HAL_OK);
}

static bool telemetry_enabled = false;

void Comm_EnableTelemetry(bool enable) {
    telemetry_enabled = enable;
}

bool Comm_IsTelemetryEnabled(void) {
    return telemetry_enabled;
}

void Comm_SendString(const char *str) {
    if (str == NULL) return;
    // Wait for any background IT transmission to finish before sending ASCII
    uint32_t timeout = HAL_GetTick() + 100;
    while (huart3.gState != HAL_UART_STATE_READY && HAL_GetTick() < timeout) {
        __NOP();
    }
    HAL_UART_Transmit(&huart3, (uint8_t*)str, (uint16_t)strlen(str), 200);
}

// Automatically re-arm DMA reception if hardware line noise or framing error occurs
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
        rx_read_ptr = 0;
        state = WAIT_SOF1;
        HAL_UART_Receive_DMA(huart, rx_dma_buf, RX_DMA_BUF_SIZE);
    }
}

