#include "rcc522.h"

/* ====== RC522 register map & commands ====== */
#define PCD_Idle           0x00
#define PCD_CalcCRC        0x03
#define PCD_Transceive     0x0C
#define PCD_SoftReset      0x0F

#define PICC_CMD_REQA      0x26
#define PICC_ANTICOLL_CL1  0x93

#define CommandReg      0x01
#define CommIrqReg      0x04
#define ErrorReg        0x06
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define ControlReg      0x0C
#define BitFramingReg   0x0D
#define ModeReg         0x11
#define TxControlReg    0x14
#define TxASKReg        0x15
#define CRCResultRegH   0x21
#define CRCResultRegL   0x22
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D
#define VersionReg      0x37

/* ====== Pin helpers ====== */
#define RC522_CS_LOW()     GPIO_ResetBits(RC522_CS_PORT, RC522_CS_PIN)
#define RC522_CS_HIGH()    GPIO_SetBits(RC522_CS_PORT, RC522_CS_PIN)
#define RC522_RST_LOW()    GPIO_ResetBits(RC522_RST_PORT, RC522_RST_PIN)
#define RC522_RST_HIGH()   GPIO_SetBits(RC522_RST_PORT, RC522_RST_PIN)

/* ====== local SPI helper ====== */
static uint8_t SPI1_TxRx(uint8_t d){
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, d);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

/* ====== low-level reg R/W ====== */
static void RC522_WriteReg(uint8_t reg, uint8_t val){
    RC522_CS_LOW();
    SPI1_TxRx((reg << 1) & 0x7E);          // write
    SPI1_TxRx(val);
    RC522_CS_HIGH();
}
static uint8_t RC522_ReadReg(uint8_t reg){
    uint8_t v;
    RC522_CS_LOW();
    SPI1_TxRx(((reg << 1) & 0x7E) | 0x80); // read
    v = SPI1_TxRx(0x00);
    RC522_CS_HIGH();
    return v;
}
static void RC522_SetBitMask(uint8_t reg, uint8_t mask){
    RC522_WriteReg(reg, RC522_ReadReg(reg) | mask);
}
static void RC522_ClearBitMask(uint8_t reg, uint8_t mask){
    RC522_WriteReg(reg, RC522_ReadReg(reg) & (~mask));
}

/* ====== public: SPI1 + GPIO init ====== */
void SPI1_Init_RC522(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    GPIO_InitTypeDef gpio;
    // SCK PA5, MOSI PA7 -> AF_PP
    gpio.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    // MISO PA6 -> input floating
    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    // NSS (PA4) -> GPIO out (t? qu?n)
    gpio.GPIO_Pin   = RC522_CS_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RC522_CS_PORT, &gpio);
    RC522_CS_HIGH();

    // RST (PB0) -> GPIO out
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin   = RC522_RST_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RC522_RST_PORT, &gpio);
    RC522_RST_HIGH();

    SPI_InitTypeDef spi;
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_Low;     // Mode 0
    spi.SPI_CPHA              = SPI_CPHA_1Edge;   // Mode 0
    spi.SPI_NSS               = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // an toàn khi test
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}

/* ====== core controls ====== */
static void RC522_Reset(void){
    RC522_WriteReg(CommandReg, PCD_SoftReset);
    for (volatile uint32_t i=0;i<30000;i++); // tiny wait
}
static void RC522_AntennaOn(void){
    uint8_t v = RC522_ReadReg(TxControlReg);
    if ((v & 0x03) != 0x03) RC522_SetBitMask(TxControlReg, 0x03);
}

/* ====== public ====== */
void RC522_Init(void){
    RC522_Reset();
    RC522_WriteReg(TModeReg,      0x8D); // TAuto=1
    RC522_WriteReg(TPrescalerReg, 0x3E);
    RC522_WriteReg(TReloadRegL,   30);
    RC522_WriteReg(TReloadRegH,   0);
    RC522_WriteReg(TxASKReg,      0x40); // 100% ASK
    RC522_WriteReg(ModeReg,       0x3D); // CRC preset 0x6363
    RC522_AntennaOn();
}

uint8_t RC522_ReadVersion(void){
    return RC522_ReadReg(VersionReg);
}

/* helper transceive */
static uint8_t RC522_Transceive(uint8_t *send, uint8_t sendLen,
                                uint8_t *back, uint8_t *backLen, uint8_t *validBits){
    uint8_t waitIRq = 0x30; // RxIRq|IdleIRq
    RC522_WriteReg(CommIrqReg, 0x7F);   // clear IRQs
    RC522_WriteReg(FIFOLevelReg, 0x80); // flush FIFO
    for (uint8_t i=0;i<sendLen;i++) RC522_WriteReg(FIFODataReg, send[i]);

    RC522_WriteReg(CommandReg, PCD_Transceive);
    RC522_SetBitMask(BitFramingReg, 0x80); // StartSend

    uint16_t i=6000; uint8_t n;
    do { n = RC522_ReadReg(CommIrqReg); i--; } while (i && !(n & waitIRq));

    RC522_ClearBitMask(BitFramingReg, 0x80); // StopSend
    if (i==0) return 1;                      // timeout
    if (RC522_ReadReg(ErrorReg) & 0x1B) return 2; // proto/parity/buffer errors

    uint8_t fifoLevel = RC522_ReadReg(FIFOLevelReg);
    if (fifoLevel > *backLen) fifoLevel = *backLen;
    for (uint8_t j=0;j<fifoLevel;j++) back[j] = RC522_ReadReg(FIFODataReg);
    *backLen = fifoLevel;
    if (validBits) *validBits = (RC522_ReadReg(ControlReg) & 0x07);
    return 0;
}

uint8_t RC522_RequestA(uint8_t atqa[2]){
    uint8_t cmd = PICC_CMD_REQA;
    uint8_t back[4]; uint8_t backLen=4; uint8_t validBits=7; // 7-bit frame
    RC522_WriteReg(BitFramingReg, 0x07); // TxLastBits=7
    uint8_t st = RC522_Transceive(&cmd, 1, back, &backLen, &validBits);
    RC522_WriteReg(BitFramingReg, 0x00);
    if (st==0 && backLen==2){ atqa[0]=back[0]; atqa[1]=back[1]; return 0; }
    return 1;
}

uint8_t RC522_AntiColl_CL1(uint8_t uid[4]){
    uint8_t cmd[2] = {PICC_ANTICOLL_CL1, 0x20};
    uint8_t back[10]; uint8_t backLen=10; uint8_t vb=0;
    RC522_WriteReg(BitFramingReg, 0x00);
    if (RC522_Transceive(cmd, 2, back, &backLen, &vb)) return 1;
    if (backLen < 5) return 2;
    for (int i=0;i<4;i++) uid[i]=back[i]; // back[4] = BCC (có th? ki?m tra)
    return 0;
}
