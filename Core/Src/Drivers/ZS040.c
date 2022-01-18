#include <Drivers/ZS040.h>
#include <Lib/str.h>

/* globals */
UART_HandleTypeDef* uart_dev;
void(*reception_callback)(char*);

/* private function declarations */
void ZS040_process_char(UART_HandleTypeDef *huart);

/* 
public function definitions 
*/
void ZS040_init(UART_HandleTypeDef *huart, void(*callback)(char*))
{
    uart_dev = huart;
    uart_dev->RxISR = ZS040_process_char;
    reception_callback = callback;
}

void ZS040_print(char* str)
{
    uint32_t len = strlen(str);
    HAL_UART_Transmit(uart_dev, str, len, 100);
}

void ZS040_print_DMA(char* str)
{
    uint32_t len = strlen(str);
    HAL_UART_Transmit_DMA(uart_dev, str, len);
}

/* 
private function definitions 
*/
void ZS040_process_char(struct __UART_HandleTypeDef *huart)
{
    static unsigned char buf[10];
    static int i = 0;

    // store incoming chars
    while (huart->Instance->ISR & USART_ISR_RXNE) {
        buf[i] = huart->Instance->RDR;
        i++;
    }

    // call callback, when line end received
    if (buf[i-1] == '\r') {
        buf[i-1] = 0;
        reception_callback(buf);
        for (uint32_t j=0; j<sizeof(buf); j++)
            buf[j] = 0;
        i = 0;

    }
}





