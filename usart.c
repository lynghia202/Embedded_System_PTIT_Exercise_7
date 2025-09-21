#include "usart.h"

static const char HEXLUT[] = "0123456789ABCDEF";

void UART1_Init(uint32_t baud){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    // TX PA9 AF_PP
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    // RX PA10 IN_FLOATING
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef us;
    us.USART_BaudRate            = baud;
    us.USART_WordLength          = USART_WordLength_8b;
    us.USART_StopBits            = USART_StopBits_1;
    us.USART_Parity              = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &us);
    USART_Cmd(USART1, ENABLE);
}

void Print_Char(char c){
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint16_t)c);
}

void Print_String(const char *s){
    while (*s) { Print_Char(*s++); }
}

void print_hex8(uint8_t v){
    Print_Char(HEXLUT[(v>>4)&0xF]);
    Print_Char(HEXLUT[v&0xF]);
}

void print_uid4(const uint8_t uid[4]){
    for (int i=0;i<4;i++){
        print_hex8(uid[i]);
        if (i<3) Print_String(" ");
    }
}
