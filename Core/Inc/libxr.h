#ifndef LIBXR_H
#define LIBXR_H

#include <stdint.h>

void XR_SystemInit(void);
void XR_GPIO_Init(void);
void XR_CAN_Init(void);
void XR_UART_Init(void);
int XR_CAN_Receive(uint32_t id, uint8_t *data, uint8_t len);
void XR_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);
void XR_Delay_ms(uint32_t ms);

#endif // LIBXR_H
