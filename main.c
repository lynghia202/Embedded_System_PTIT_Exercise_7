#include "stm32f10x.h"
#include <stdint.h>
#include "rcc522.h"
#include "usart.h"

/* === SysTick delay === */
static volatile uint32_t msTick=0;
void SysTick_Handler(void){ msTick++; }
static void delay_ms(uint32_t ms){
    uint32_t t = msTick + ms;
    while ((int32_t)(t - msTick) > 0);
}

/* === demo === */
static void RC522_ReadUID_PrintOnce(void){
    uint8_t atqa[2], uid[4];
    if (RC522_RequestA(atqa)==0){
        if (RC522_AntiColl_CL1(uid)==0){
						Print_String("UID: ");
            print_uid4(uid);
						Print_String("\r\n");
            Print_String("Ready. Present a card near antenna...\r\n");
        }
    }
}


int main(void){
    SystemInit();
    SysTick_Config(SystemCoreClock/1000);

    UART1_Init(9600); 
    Print_String("\r\n=== RC522 UID Demo (SPI1) ===\r\n");

    SPI1_Init_RC522();
    RC522_Init();

    uint8_t ver = RC522_ReadVersion();
    Print_String("VersionReg=0x"); print_hex8(ver);
    if (ver==0x91 || ver==0x92) Print_String(" OK\r\n");
    else                        Print_String("Connection Failed\r\n");

    Print_String("Ready. Present a card near antenna...\r\n");

    while (1){
        RC522_ReadUID_PrintOnce();
        delay_ms(100);
				
    }
}
