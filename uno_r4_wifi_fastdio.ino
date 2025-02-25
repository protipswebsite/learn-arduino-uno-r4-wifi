//  Title - Arduino Uno R4 WiFi Fast Digital IO Read Write to Registers Example
//  Author - Pro.Tips
//  Arduino UNO R4 code for fast digital pin operation
//  https://pro.tips/courses/learn-arduino-uno-r4-wifi-free-online-course/lesson/fast-digital-io-read-write-operation-on-arduino-uno-r4-wifi/

// ARM-developer - Accessing memory-mapped peripherals
// https://developer.arm.com/documentation/102618/0100
// =========== Ports ============
// 19.2.5 Port mn Pin Function Select Register (PmnPFS/PmnPFS_HA/PmnPFS_BY) (m = 0 to 9; n = 00 to 15)

// DIO port mapping is based on Susan Parker's pin allocation is as per R4 Minima (non-wifi) version
// https://github.com/TriodeGirl/RA4M1_Arduino_UNO-R4_Processor_Direct_Register_Addressing_Defines/blob/main/susan_ra4m1_minima_register_defines.h
// Used R4 WiFi schematics https://docs.arduino.cc/resources/schematics/ABX00087-schematics.pdf to figure out PIN register mapping
// All DIO's below were tested and verified on actual hardware

#define PORTBASE 0x40040000 /* Port Base */

#define P100PFS 0x0843  // Port 1 Pin Function Select Register
#define PFS_P102PFS_BY ((volatile unsigned char  *)(PORTBASE + P100PFS + ( 2 * 4))) // D13          Confirmed for R4 WiFi
#define PFS_P103PFS_BY ((volatile unsigned char  *)(PORTBASE + P100PFS + ( 3 * 4))) // D10          Confirmed for R4 WiFi
#define PFS_P104PFS_BY ((volatile unsigned char  *)(PORTBASE + P100PFS + ( 4 * 4))) // D2           Confirmed for R4 WiFi
#define PFS_P105PFS_BY ((volatile unsigned char  *)(PORTBASE + P100PFS + ( 5 * 4))) // D3           Confirmed for R4 WiFi
#define PFS_P106PFS_BY ((volatile unsigned char  *)(PORTBASE + P100PFS + ( 6 * 4))) // D4           Confirmed for R4 WiFi
#define PFS_P107PFS_BY ((volatile unsigned char  *)(PORTBASE + P100PFS + ( 7 * 4))) // D5           Confirmed for R4 WiFi
#define PFS_P111PFS_BY ((volatile unsigned char  *)(PORTBASE + P100PFS + (11 * 4))) // D6           Confirmed for R4 WiFi
#define PFS_P112PFS_BY ((volatile unsigned char  *)(PORTBASE + P100PFS + (12 * 4))) // D7           Confirmed for R4 WiFi

#define P300PFS 0x08C3  // Port 3 Pin Function Select Register
#define PFS_P301PFS_BY ((volatile unsigned char  *)(PORTBASE + P300PFS + (01 * 4))) // D0           Confirmed for R4 WiFi
#define PFS_P302PFS_BY ((volatile unsigned char  *)(PORTBASE + P300PFS + (02 * 4))) // D1           Confirmed for R4 WiFi
#define PFS_P303PFS_BY ((volatile unsigned char  *)(PORTBASE + P300PFS + (03 * 4))) // D9           Confirmed for R4 WiFi
#define PFS_P304PFS_BY ((volatile unsigned char  *)(PORTBASE + P300PFS + (04 * 4))) // D8           Confirmed for R4 WiFi


#define P400PFS 0x0900  // Port 4 Pin Function Select Register
#define PFS_P410PFS ((volatile unsigned int *)(PORTBASE + P400PFS + (10 * 4)))      // D12          Confirmed for R4 WiFi
#define PFS_P411PFS ((volatile unsigned int *)(PORTBASE + P400PFS + (11 * 4)))      // D11          Confirmed for R4 WiFi

#define CPU_RESET_CYCLECOUNTER    do { CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
                                       ITM->LAR = 0xc5acce55; \
                                       DWT->CYCCNT = 0; \
									                     DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; } while(0)

void setup()
{
  Serial.begin(115200);
  Serial.println("Fast PIN test");
  /* Reset CPU cycle counter */
  CPU_RESET_CYCLECOUNTER;
}

unsigned char char_val; 

unsigned int cnt=0;
unsigned long startTime, endTime, elapsedTime=0;

double digitalReadTime, digitalWriteHighTime, digitalWriteLowTime;
double regDigitalReadTime, regDigitalWriteHighTime, regDigitalWriteLowTime;

#define FREQ_CPU 48000000.0f
#define CYCLE_TIME (1 / FREQ_CPU) * 1000000000.0f

void loop() {

#define NMB_OF_WRITE_OPS 10000

  // Measure digitalRead() elapsed time
  startTime = DWT->CYCCNT;
  for (int i=0; i<NMB_OF_WRITE_OPS; i++) {
    digitalRead(5);
  }
  elapsedTime = DWT->CYCCNT-startTime;
  digitalReadTime = ((elapsedTime / FREQ_CPU) * 1000000000.0f) / NMB_OF_WRITE_OPS;

  // Measure digitalWrite(pin, LOW) elapsed time
  startTime = DWT->CYCCNT;
  for (int i=0; i<NMB_OF_WRITE_OPS; i++) {
    digitalWrite(5,LOW);
  }
  elapsedTime = DWT->CYCCNT-startTime;
  digitalWriteLowTime = ((elapsedTime / FREQ_CPU) * 1000000000.0f) / NMB_OF_WRITE_OPS;

  // Measure digitalWrite(pin#, HIGH) elapsed time
  startTime = DWT->CYCCNT;
  for (int i=0; i<NMB_OF_WRITE_OPS; i++) {
    digitalWrite(5,HIGH);
  }
  elapsedTime = DWT->CYCCNT-startTime;
  digitalWriteHighTime = ((elapsedTime / FREQ_CPU) * 1000000000.0f) / NMB_OF_WRITE_OPS;

  // Read status directly from pin D9 register
  startTime = DWT->CYCCNT;
  for (int i=0; i<NMB_OF_WRITE_OPS; i++) {
    char_val = *PFS_P102PFS_BY;
  }
  elapsedTime = DWT->CYCCNT-startTime;
  regDigitalReadTime = ((elapsedTime / FREQ_CPU) * 1000000000.0f) / NMB_OF_WRITE_OPS;

  // Write LOW (clear) directly to pin D9 register
  startTime = DWT->CYCCNT;
  for (int i=0; i<NMB_OF_WRITE_OPS; i++) {
    *PFS_P102PFS_BY = 0x05;
  }
  elapsedTime = DWT->CYCCNT-startTime;
  regDigitalWriteLowTime = ((elapsedTime / FREQ_CPU) * 1000000000.0f) / NMB_OF_WRITE_OPS;

  // Write High (set) directly to pin D9 register
  startTime = DWT->CYCCNT;
  for (int i=0; i<NMB_OF_WRITE_OPS; i++) {
    *PFS_P102PFS_BY = 0x04;
  }
  elapsedTime = DWT->CYCCNT-startTime;
  regDigitalWriteHighTime = ((elapsedTime / FREQ_CPU) * 1000000000.0f) / NMB_OF_WRITE_OPS;

  Serial.print("digitalReadTime=");
  Serial.print(digitalReadTime,10);
  Serial.print(" digitalWriteLowTime=");
  Serial.print(digitalWriteLowTime,10);
  Serial.print(" digitalWriteHighTime=");
  Serial.print(digitalWriteHighTime,10);
  Serial.print(" regDigitalReadTime=");
  Serial.print(regDigitalReadTime,10);
  Serial.print(" regDigitalWriteLowTime=");
  Serial.print(regDigitalWriteLowTime,10);
  Serial.print(" regDigitalWriteHighTime=");
  Serial.println(regDigitalWriteHighTime,10);
}
