#ifndef AFE4490_H
#define AFE4490_H


#include "Arduino.h" 


const int SPISTE = 7; //SPI Serial Interface Enable, Chip select
const int PWDN = 4; //AFE-only power-down input, active low
const int SPIDRDY = 2; //ADC Conversion Completion, Data Ready

volatile int drdy_trigger = false;
volatile boolean AFE4490_data_ready = false;

unsigned long IRtemp,REDtemp;
unsigned long ired, red;


//AFE4490 Register
#define CONTROL0    0x00
#define LED2STC     0x01
#define LED2ENDC    0x02
#define LED2LEDSTC    0x03
#define LED2LEDENDC   0x04
#define ALED2STC    0x05
#define ALED2ENDC   0x06
#define LED1STC     0x07
#define LED1ENDC    0x08
#define LED1LEDSTC    0x09
#define LED1LEDENDC   0x0a
#define ALED1STC    0x0b
#define ALED1ENDC   0x0c
#define LED2CONVST    0x0d
#define LED2CONVEND   0x0e
#define ALED2CONVST   0x0f
#define ALED2CONVEND  0x10
#define LED1CONVST    0x11
#define LED1CONVEND   0x12
#define ALED1CONVST   0x13
#define ALED1CONVEND  0x14
#define ADCRSTSTCT0   0x15
#define ADCRSTENDCT0  0x16
#define ADCRSTSTCT1   0x17
#define ADCRSTENDCT1  0x18
#define ADCRSTSTCT2   0x19
#define ADCRSTENDCT2  0x1a
#define ADCRSTSTCT3   0x1b
#define ADCRSTENDCT3  0x1c
#define PRPCOUNT    0x1d
#define CONTROL1    0x1e
#define SPARE1      0x1f
#define TIAGAIN     0x20
#define TIA_AMB_GAIN  0x21
#define LEDCNTRL    0x22
#define CONTROL2    0x23
#define SPARE2      0x24
#define SPARE3      0x25
#define SPARE4      0x26
#define SPARE4      0x26
#define RESERVED1   0x27
#define RESERVED2   0x28
#define ALARM     0x29
#define LED2VAL     0x2a
#define ALED2VAL    0x2b
#define LED1VAL     0x2c
#define ALED1VAL    0x2d
#define LED2ABSVAL    0x2e
#define LED1ABSVAL    0x2f
#define DIAG      0x30






#endif

