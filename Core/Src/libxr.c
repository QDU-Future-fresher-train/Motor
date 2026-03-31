#include "libxr.h"

void XR_SystemInit(void) {
    // Stub implementation; replace with actual platform initialization code.
}

void XR_GPIO_Init(void) {
    // Stub GPIO init.
}

void XR_CAN_Init(void) {
    // Stub CAN init.
}

void XR_UART_Init(void) {
    // Stub UART init.
}

int XR_CAN_Receive(uint32_t id, uint8_t *data, uint8_t len) {
    (void)id;
    (void)data;
    (void)len;
    return -1; // indicate no data received
}

void XR_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len) {
    (void)id;
    (void)data;
    (void)len;
}

void XR_Delay_ms(uint32_t ms) {
    (void)ms;
}
