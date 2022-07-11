#include <SPI.h>

#include "src/AFE4490.h"
#include "src/bp_filter.h"
#include "src/lp_filter.h"
#include "src/wave_filter.h"

void testInit();
void initAFE4490();
void afe44xx_drdy_event();
void writeAFE4490(uint8_t address, uint32_t data);
unsigned long readAFE4490(uint8_t address);

void user_delay(int count) { delay(count); }

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 5;

void setup()
{
  startMillis = millis();  //initial start time

  //Init Arduino UART========================
  Serial.begin(115200);
  //Serial.println("Start Respactory SpO2 Module Communication!");
  //=========================================

  //Init Arduino SPI=========================
  SPI.begin();
  SPI.setClockDivider (SPI_CLOCK_DIV8); // set Speed as 2MHz , 16MHz/ClockDiv
  SPI.setDataMode (SPI_MODE0);          //Set SPI mode as 0
  SPI.setBitOrder (MSBFIRST);           //MSB first
  //=========================================

  //Init AFE4490 SPI and Interrupt===========
  pinMode(SPISTE, OUTPUT);
  pinMode(PWDN, OUTPUT);
  pinMode(SPIDRDY,INPUT);// data ready
  attachInterrupt(digitalPinToInterrupt(SPIDRDY), afe44xx_drdy_event, RISING);
  //=========================================

  //initAFE4490();
  testInit();
}

//pulse rate-------------------------------------
#define BP_ARRAY_SIZE 200
int arrayBP[BP_ARRAY_SIZE];
int ired_bp_filter;
int maxBP_IRed, minBP_IRed;
int WindowIRed[3];
int WindowCount;
int WindowCount_prev;
int pulseRateArray[5];
int pulseRate;
bool ired_ok = false;

void measurePulseRate(int tempIRed);
//pulse rate-------------------------------------

//perfusion index--------------------------------
int perfusion_index = 0;
//perfusion index--------------------------------

//SpO2 saturation--------------------------------
#define LP_ARRAY_SIZE 100
int arrayIRed[LP_ARRAY_SIZE];
int arrayRed[LP_ARRAY_SIZE];
int averageIRed, averageRed;
int maxIRed, minIRed;
int maxRed, minRed;
int saturation;

void measureSaturation(void);
//SpO2 saturation--------------------------------


int waveIRed;

void loop()
{
  currentMillis = millis();
  if(currentMillis - startMillis >= period)
  {
    writeAFE4490(CONTROL0, 0x000001);
    ired = readAFE4490(LED1VAL)/0xFF;
    writeAFE4490(CONTROL0, 0x000001);
    red = readAFE4490(LED2VAL)/0xFF;

    measurePulseRate(ired);

    measureSaturation();

    waveIRed = wave_iir(ired);
    waveIRed = waveIRed - 1000;
    waveIRed = abs(waveIRed);

    startMillis = currentMillis;
  }

  if(Serial.available()){
    String tempString = Serial.readStringUntil('\n');
    if(tempString == "pulseRate"){
      Serial.println(tempString+"="+pulseRate);
    }else if(tempString == "saturation"){
      Serial.println(tempString+"="+saturation);
    }else if(tempString == "ired"){
      Serial.println(tempString+"="+waveIRed);
    }else if(tempString == "perfusionIndex"){
      Serial.println(tempString+"="+perfusion_index);
    }
  }

}


void measurePulseRate(int tempIRed)
{
  if(tempIRed<8000) ired_ok = true;
  else ired_ok = false;

  ired_bp_filter = bp_iir(tempIRed);

  //IRed: Input array
  for(int i=BP_ARRAY_SIZE-1; i>0; i--){ arrayBP[i] = arrayBP[i-1]; }
  arrayBP[0] = ired_bp_filter;

  //IRed: min max 
  maxBP_IRed = arrayBP[0];
  minBP_IRed = arrayBP[0];
  for(int i=0; i<BP_ARRAY_SIZE; i++){
    if(arrayBP[i]>maxBP_IRed) maxBP_IRed = arrayBP[i];
    if(arrayBP[i]<minBP_IRed) minBP_IRed = arrayBP[i];
  }

  //IRed: pulse rate
  WindowIRed[2] = WindowIRed[1];
  WindowIRed[1] = WindowIRed[0];
  WindowIRed[0] = ired_bp_filter;
  int iTemp = (maxBP_IRed+minBP_IRed)/2;
  if( (WindowIRed[2] < iTemp) && (WindowIRed[0] > iTemp) ){
    WindowCount = 0;
    if( (WindowCount_prev!=0) && (WindowCount==0) ){
      for(int i=5-1; i>0; i--){
        pulseRateArray[i] = pulseRateArray[i-1];
      }
      pulseRateArray[0] = (60*200)/WindowCount_prev; // (60sec*200Hz) / measured_count

      //5개의 배열에서 최대 최소를 제외한 나머지 값을 평균내어 pulse rate에 반영한다.
      int tempMax = pulseRateArray[0];
      int tempMin = pulseRateArray[0];
      for(int i=0; i<5; i++){
        if(pulseRateArray[i]>tempMax) tempMax = pulseRateArray[i];
        if(pulseRateArray[i]<tempMin) tempMin = pulseRateArray[i];
      }
      long lTemp = 0;
      for(int i=0; i<5; i++){
        lTemp += pulseRateArray[i];
      }
      lTemp = lTemp - tempMax - tempMin;
      lTemp = lTemp/3;
      pulseRate = lTemp;

      /*
      Serial.print(maxIRed-minIRed);
      Serial.print(",");
      Serial.print(averageIRed);
      Serial.print(",");
      Serial.println( (100000*(maxIRed-minIRed))/averageIRed);
      */
      long tempPI = 100000*(maxIRed-minIRed);
      perfusion_index = tempPI/averageIRed;
    }
  }
  WindowCount_prev = WindowCount;

  if(ired_ok) WindowCount++;
  else{
    WindowCount = 0;
    pulseRate = 0;
  }
/*
  Serial.print(maxBP_IRed);
  Serial.print(",");
  Serial.print(minBP_IRed);
  Serial.print(",");
  Serial.print(iTemp);
  Serial.print(",");
  Serial.print(ired_bp_filter);
  Serial.print(",");
  Serial.print(WindowCount_prev);
  Serial.print(",");
  Serial.println(pulseRate);
*/

  if(!ired_ok) {
    pulseRate = -1;
    perfusion_index = -1;
  }
}

void measureSaturation(void)
{
  int ired_lp_filter = ired_iir(ired);
  int red_lp_filter = red_iir(red);

  //Input array
  for(int i=LP_ARRAY_SIZE-1; i>0; i--){
    arrayIRed[i] = arrayIRed[i-1]; 
    arrayRed[i] = arrayRed[i-1]; 
  }
  arrayIRed[0] = ired_lp_filter;
  arrayRed[0] = red_lp_filter;

  //find min max 
  maxIRed = arrayIRed[0];
  minIRed = arrayIRed[0];
  for(int i=0; i<LP_ARRAY_SIZE; i++){
    if(arrayIRed[i]>maxIRed) maxIRed = arrayIRed[i];
    if(arrayIRed[i]<minIRed) minIRed = arrayIRed[i];
  }
  maxRed = arrayRed[0];
  minRed = arrayRed[0];
  for(int i=0; i<LP_ARRAY_SIZE; i++){
    if(arrayRed[i]>maxRed) maxRed = arrayRed[i];
    if(arrayRed[i]<minRed) minRed = arrayRed[i];
  }

  //sum and average
  
  long lTempIRed = 0;
  long lTempRed = 0;
  for(int i=LP_ARRAY_SIZE; i>0; i--){
    lTempIRed += arrayIRed[i]; 
    lTempRed += arrayRed[i]; 
  }
  //averageIRed = lTempIRed/LP_ARRAY_SIZE;
  //averageRed = lTempRed/LP_ARRAY_SIZE;
  
  averageIRed = (maxIRed+minIRed)/2;
  averageRed = (maxRed+minRed)/2;

  /*
      Serial.print(maxRed);
      Serial.print(",");
      Serial.print(minRed);
      Serial.print(",");
      Serial.print(averageRed);
      Serial.print(",");
      Serial.print(red);
      Serial.print(",");
  
      Serial.print(maxIRed);
      Serial.print(",");
      Serial.print(minIRed);
      Serial.print(",");
      Serial.print(averageIRed);
      Serial.print(",");
      Serial.println(ired);
  */

  if(ired_ok){
    float ac_red = (maxRed-minRed);
    float dc_red = averageRed;
    float ac_ired = (maxIRed-minIRed);
    float dc_ired = averageIRed;
    
    float tempR = (ac_red/dc_red) / (ac_ired/dc_ired);
    //saturation = 110-25*tempR;
    #define A -25.01735
    #define B 1.414907
    #define C 104.825874

    saturation = A*tempR*tempR + B*tempR + C;

    if(saturation > 100) saturation = 100;
    //Serial.println(saturation);
  }
  else{
    saturation = -1;
  }
}




void initAFE4490()
{
  //Init AFE4490 Power On and Configuration==
  digitalWrite(PWDN, LOW);
  user_delay(100);
  digitalWrite(PWDN, HIGH);
  user_delay(100);

/*
CONTROL0: Control Register 0

D3 SW_RST: Software reset
0 = No Action
1 = Software reset applied; resets all internal registers to the default values and self-clears to '0'

D2 DIAG_EN: Diagnostic enable
0 = No Action (default after reset)
1 = Diagnostic mode is enabled

D1 TIM_CNT_RST: Timer counter reset
0 = Disables timer counter reset, required for normal timer operation (default after reset)
1 = Timer counters are in reset state

D0 SPI READ: SPI read
0 = SPI read is disabled (default after reset)
1 = SPI read is enabled
*/
  writeAFE4490(CONTROL0, 0x000000); //SW_RST = 0
  writeAFE4490(CONTROL0, 0x000008); //SW_RST = 1


/*
LED2STC: Sample LED2 Start Count Register

D[15:0] LED2STC[15:0]: Sample LED2 start count
*/
  writeAFE4490(LED2STC, 0X001770); //0X001770 = 6000


/*
LED2ENDC: Sample LED2 End Count Register

D[15:0] LED2ENDC[15:0]: Sample LED2 end count
*/
  writeAFE4490(LED2ENDC, 0X001F3E); //0X001F3E = 7998


/*
LED2LEDSTC: LED2 LED Start Count Register

D[15:0] LED2LEDSTC[15:0]: LED2 start count
*/
  writeAFE4490(LED2LEDSTC, 0X001770);//0X001770 = 6000


/*
LED2LEDENDC: LED2 LED End Count Register

D[15:0] LED2LEDENDC[15:0]: LED2 end count
*/
  writeAFE4490(LED2LEDENDC, 0X001F3F); //0X001F3E = 7998


/*
ALED2STC: Sample Ambient LED2 Start Count Register

D[15:0] ALED2STC[15:0]: Sample ambient LED2 start count
*/
  writeAFE4490(ALED2STC, 0X000000);


/*
ALED2ENDC: Sample Ambient LED2 End Count Register

D[15:0] ALED2ENDC[15:0]: Sample ambient LED2 end count
*/
  writeAFE4490(ALED2ENDC, 0X0007CE); //0X0007CE = 1998


/*
LED2CONVST: LED2 Convert Start Count Register

D[15:0] LED2CONVST[15:0]: LED2 convert start count
*/
  writeAFE4490(LED2CONVST, 0X000002);


/*
LED2CONVEND: LED2 Convert End Count Register

D[15:0] LED2CONVEND[15:0]: LED2 convert end count
*/
  writeAFE4490(LED2CONVEND, 0X0007CF); //0X0007CE = 1998


/*
ALED2CONVST: LED2 Ambient Convert Start Count Register

D[15:0] ALED2CONVST[15:0]: LED2 ambient convert start count
*/
  writeAFE4490(ALED2CONVST, 0X0007D2); //0X0007D2 = 2002


/*
ALED2CONVEND: LED2 Ambient Convert End Count Register

D[15:0] ALED2CONVEND[15:0]: LED2 ambient convert end count
*/
  writeAFE4490(ALED2CONVEND, 0X000F9F); //0X000F9F = 3999


/*
LED1STC: Sample LED1 Start Count Register

D[16:0] LED1STC[15:0]: Sample LED1 start count
*/
  writeAFE4490(LED1STC, 0X0007D0); //0X0007D0 = 2000


/*
LED1ENDC: Sample LED1 End Count

D[16:0] LED1ENDC[15:0]: Sample LED1 end count
*/
  writeAFE4490(LED1ENDC, 0X000F9E); //0X000F9E = 3998


/*
LED1LEDSTC: LED1 LED Start Count Register

D[15:0] LED1LEDSTC[15:0]: LED1 start count
*/
  writeAFE4490(LED1LEDSTC, 0X0007D0); //0X0007D0 = 2000

/*
LED1LEDENDC: LED1 LED End Count Register

D[15:0] LED1LEDENDC[15:0]: LED1 end count
*/
  writeAFE4490(LED1LEDENDC, 0X000F9F); //0X000F9F = 3999


/*
ALED1STC: Sample Ambient LED1 Start Count Register

D[15:0] ALED1STC[15:0]: Sample ambient LED1 start count
*/
  writeAFE4490(ALED1STC, 0X000FA0); //0X000FA0 = 4000


/*
ALED1ENDC: Sample Ambient LED1 End Count Register

D[15:0] ALED1ENDC[15:0]: Sample ambient LED1 end count
*/
  writeAFE4490(ALED1ENDC, 0X00176E); //0X00176E = 5998


/*
LED1CONVST: LED1 Convert Start Count Register

D[15:0] LED1CONVST[15:0]: LED1 convert start count
*/
  writeAFE4490(LED1CONVST, 0X000FA2); //0X000FA2 = 4002


/*
LED1CONVEND: LED1 Convert End Count Register

D[15:0] LED1CONVEND[15:0]: LED1 convert end count
*/
  writeAFE4490(LED1CONVEND, 0X00176F); //0X00176F = 5999


/*
ALED1CONVST: LED1 Ambient Convert Start Count Register

D[15:0] ALED1CONVST[15:0]: LED1 ambient convert start count
*/
  writeAFE4490(ALED1CONVST, 0X001772); //0X001772 = 6002


/*
ALED1CONVEND: LED1 Ambient Convert End Count Register

D[15:0] ALED1CONVEND[15:0]: LED1 ambient convert end count
*/
  writeAFE4490(ALED1CONVEND, 0X001F3F); //0X001F3F = 7999


/*
ADCRSTSTCT0: ADC Reset 0 Start Count Register

D[15:0] ADCRSTSTCT0[15:0]: ADC RESET 0 start count
*/
  writeAFE4490(ADCRSTSTCT0, 0X000000);


/*
ADCRSTENDCT0: ADC Reset 0 End Count Register

D[15:0] ADCRSTENDCT0[15:0]: ADC RESET 0 end count
*/
  writeAFE4490(ADCRSTENDCT0, 0X000000);


/*
ADCRSTSTCT1: ADC Reset 1 Start Count Register

D[15:0] ADCRSTSTCT1[15:0]: ADC RESET 1 start count
*/
  writeAFE4490(ADCRSTSTCT1, 0X0007D0); //0X0007D0 = 2000


/*
ADCRSTENDCT1: ADC Reset 1 End Count Register

D[15:0] ADCRSTENDCT1[15:0]: ADC RESET 1 end count
*/
  writeAFE4490(ADCRSTENDCT1, 0X0007D0); //0X0007D0 = 2000


/*
ADCRSTSTCT2: ADC Reset 2 Start Count Register

D[15:0] ADCRSTSTCT2[15:0]: ADC RESET 2 start count
*/
  writeAFE4490(ADCRSTSTCT2, 0X000FA0); //0X000FA0 = 4000


/*
ADCRSTENDCT2: ADC Reset 2 End Count Register

D[15:0] ADCRSTENDCT2[15:0]: ADC RESET 2 end count
*/
  writeAFE4490(ADCRSTENDCT2, 0X000FA0); //0X000FA0 = 4000


/*
ADCRSTSTCT3: ADC Reset 3 Start Count Register

D[15:0] ADCRSTSTCT3[15:0]: ADC RESET 3 start count
*/
  writeAFE4490(ADCRSTSTCT3, 0X001770); //0X001770 = 6000

/*
ADCRSTENDCT3: ADC Reset 3 End Count Register

D[15:0] ADCRSTENDCT3[15:0]: ADC RESET 3 end count
*/
  writeAFE4490(ADCRSTENDCT3, 0X001770); //0X001770 = 6000


/*
PRPCOUNT: Pulse Repetition Period Count Register

D[15:0] PRPCOUNT[15:0]: Pulse repetition period count, 800 ~ 64000
*/
  writeAFE4490(PRPCOUNT, 0X001F3F); //0X001F3F = 7999


/*
CONTROL1: Control Register 1

D[11:9] CLKALMPIN[2:0]: Clocks on ALM pins

D8 TIMEREN: Timer enable

D[7:0] NUMAV[7:0]: Number of averages
*/
//0x000707 = 0b 0000 0000 0000 011 1 00000111
  //origin source set -> writeAFE4490(CONTROL1, 0x010707); // Timers ON, average 3 samples
  writeAFE4490(CONTROL1, 0x000707); // Timers ON, average 8 samples


/*
TIAGAIN: Transimpedance Amplifier Gain Setting Register

D15 ENSEPGAIN: Enable separate gain mode

D14 STAGE2EN1: Enable Stage 2 for LED 1
0 = Stage 2 is bypassed (default after reset)
1 = Stage 2 is enabled with the gain value specified by the STG2GAIN1[2:0] bits

D[10:8] STG2GAIN1[2:0]: Program Stage 2 gain for LED1
000 = 0dB

D[7:3] CF_LED1[4:0]: Program CF for LED1
00000 = 5pF

D[2:0] RF_LED1[2:0]: Program RF for LED1
000 = 500 k Ohm
*/
  writeAFE4490(TIAGAIN, 0x000000);


/*
TIA_AMB_GAIN: Transimpedance Amplifier and Ambient Cancellation Stage Gain Register

D[19:16] AMBDAC[3:0]: Ambient DAC value
0000 = 0uA

D15 FLTRCNRSEL: Filter corner selection
0 = 500-Hz filter corner (default after reset)
1 = 1000-Hz filter corner

D14 STAGE2EN2: Stage 2 enable for LED 2
0 = Stage 2 is bypassed (default after reset)
1 = Stage 2 is enabled with the gain value specified by the STG2GAIN2[2:0] bits

D[10:8] STG2GAIN2[2:0]: Stage 2 gain setting for LED 2
000 = 0 dB

D[7:3] CF_LED2[4:0]: Program CF for LED2
00000 = 5 pF

D[2:0] RF_LED2[2:0]: Program RF for LED2
001 = 250 kΩ
*/
  writeAFE4490(TIA_AMB_GAIN, 0x000001);


/*
LEDCNTRL: LED Control Register

D[17:16] LED_RANGE[1:0]: LED range

D[15:8] LED1[7:0]: Program LED current for LED1 signal
0b 0001 0100 = 0x14 = 20

D[7:0] LED2[7:0]: Program LED current for LED2 signal
0b 0001 0100 = 0x14 = 20
*/
//0x001414 = 0b 0000 0000   0001 0100   0001 0100
  writeAFE4490(LEDCNTRL, 0x001414);
  


/*
CONTROL2: Control Register 2

D[18:17] TX_REF[1:0]: Tx reference voltage
00 = 0.75-V Tx reference voltage

D16 RST_CLK_ON_PD_ALM: Reset clock onto PD_ALM pin
0 = Normal mode

D15 EN_ADC_BYP: ADC bypass mode enable
0 = Normal mode

D11 TXBRGMOD: Tx bridge mode
0 = LED driver is configured as an H-bridge

D10 DIGOUT_TRISTATE: Digital output 3-state mode
0 = Normal operation

D9 XTALDIS: Crystal disable mode
0 = The crystal module is enabled

D8 EN_SLOW_DIAG: Fast diagnostics mode enable
0 = Fast diagnostics mode

D2 PDN_TX: Tx power-down
0 = The Tx is powered up

D1 PDN_RX: Rx power-down
0 = The Rx is powered up

D0 PDN_AFE: AFE power-down
0 = The AFE is powered up
*/
  writeAFE4490(CONTROL2, 0x000000); // LED_RANGE=100mA, LED=50mA


  user_delay(100);
  //=========================================
}



void testInit()
{
  //AFE4490 Manual 36 Page Setting

  //Init AFE4490 Power On and Configuration==
  digitalWrite(PWDN, LOW);
  user_delay(100);
  digitalWrite(PWDN, HIGH);
  user_delay(100);

  writeAFE4490(CONTROL0, 0x000000); //SW_RST = 0
  writeAFE4490(CONTROL0, 0x000008); //SW_RST = 1

  writeAFE4490(0x01, 6050);
  writeAFE4490(0x02, 7998);
  writeAFE4490(0x03, 6000);
  writeAFE4490(0x04, 7999);
  writeAFE4490(0x05, 50);

  writeAFE4490(0x06, 1998);
  writeAFE4490(0x07, 2050);
  writeAFE4490(0x08, 3998);
  writeAFE4490(0x09, 2000);
  writeAFE4490(0x0A, 3999);
  writeAFE4490(0x0B, 4050);
  writeAFE4490(0x0C, 5998);
  writeAFE4490(0x0D, 4);

  writeAFE4490(0x0E, 1999);
  writeAFE4490(0x0F, 2004);
  writeAFE4490(0x10, 3999);
  writeAFE4490(0x11, 4004);
  writeAFE4490(0x12, 5999);
  writeAFE4490(0x13, 6004);
  writeAFE4490(0x14, 7999);
  writeAFE4490(0x15, 0);
  writeAFE4490(0x16, 3);

  writeAFE4490(0x17, 2000);
  writeAFE4490(0x18, 2003);
  writeAFE4490(0x19, 4000);
  writeAFE4490(0x1A, 4003);
  writeAFE4490(0x1B, 6000);
  writeAFE4490(0x1C, 6003);
  writeAFE4490(0x1D, 7999);

  writeAFE4490(CONTROL1, 0x000707); // Timers ON, average 8 samples

  writeAFE4490(TIAGAIN, 0x000000);

  writeAFE4490(TIA_AMB_GAIN, 0x000001);

  //origin setting writeAFE4490(LEDCNTRL, 0x001414);
  //writeAFE4490(LEDCNTRL, 0x000101); //red 8000 ired 35000
  //writeAFE4490(LEDCNTRL, 0x000202); //red 20000 ired 90000
  //writeAFE4490(LEDCNTRL, 0x000404); //red 45000 ired 240000
  //writeAFE4490(LEDCNTRL, 0x000808); //red 80000 ired 520000
  writeAFE4490(LEDCNTRL, 0x001414);

  writeAFE4490(CONTROL2, 0x000000); // LED_RANGE=100mA, LED=50mA

  user_delay(100);
}


void afe44xx_drdy_event() 
{
  drdy_trigger = true; 
}

void writeAFE4490(uint8_t address, uint32_t data)
{
  digitalWrite (SPISTE, LOW); // enable device
  SPI.transfer (address); // send address to device
  SPI.transfer ((data >> 16) & 0xFF); // write top 8 bits
  SPI.transfer ((data >> 8) & 0xFF); // write middle 8 bits
  SPI.transfer (data & 0xFF); // write bottom 8 bits
  digitalWrite (SPISTE, HIGH); // disable device
}

unsigned long readAFE4490(uint8_t address)
{
  unsigned long data = 0;
  digitalWrite (SPISTE, LOW); // enable device
  SPI.transfer (address); // send address to device
  
  data |= ((unsigned long)SPI.transfer (0) << 16); // read top 8 bits data
  data |= ((unsigned long)SPI.transfer (0) << 8); // read middle 8 bits  data
  data |= SPI.transfer (0); // read bottom 8 bits data
  digitalWrite (SPISTE, HIGH); // disable device

  return data; // return with 24 bits of read data
}



