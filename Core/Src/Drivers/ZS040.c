#include <stdint.h>

#include <Drivers/ZS040.h>
#include <Lib/str.h>


/* globals */
UART_HandleTypeDef* uart_dev;
void(*reception_callback)(uint8_t, char**);

/* private function declarations */
void ZS040_process_char(UART_HandleTypeDef *huart);

/* 
public function definitions 
*/
void ZS040_init(UART_HandleTypeDef *huart, void(*callback)(uint8_t, char**))
{
    uart_dev = huart;
    uart_dev->RxISR = ZS040_process_char;
    reception_callback = callback;
}

void ZS040_print(char* str)
{
    uint32_t len = strlen(str);
    HAL_UART_Transmit(uart_dev, (uint8_t*)str, len, 100);
}

void ZS040_print_DMA(char* str)
{
    uint32_t len = strlen(str);
    HAL_UART_Transmit_DMA(uart_dev, (uint8_t*)str, len);
}

/* 
private function definitions 
*/
void ZS040_process_char(struct __UART_HandleTypeDef *huart)
{
    static unsigned char buf[50];
    static uint8_t argc = 1;
    static char* argv[10] = {&(buf[0])};
    static int i = 0;

    // store incoming chars
    while (huart->Instance->ISR & USART_ISR_RXNE) {
        buf[i] = huart->Instance->RDR;
        if (buf[i] == ' ') {
            argv[argc] = &(buf[i+1]);
            argc++;
            buf[i] = 0;
        }
            
        i++;
    }

    // call callback, when line end received
    if (buf[i-1] == '\r') {
        buf[i-1] = 0;
        reception_callback(argc, argv);
        for (uint32_t j=0; j<sizeof(buf); j++)
            buf[j] = 0;
        i = 0;

        argc = 1;
    }
}





