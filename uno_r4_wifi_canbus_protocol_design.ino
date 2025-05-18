/*
  Title - Arduino Read/Write Digital IO using CAN Bus
  Designer - Pro.Tips website
  Website - https://pro.tips
  Date - 05/10/2025
  Link - https://pro.tips/courses/learn-arduino-uno-r4-wifi-free-online-course/lesson/arduino-uno-r4-wifi-can-bus-protocol-design-and-implementation/
*/

#include <Arduino_CAN.h>
#include "FspTimer.h"       // Used for 10ms timer
FspTimer T10ms_timer;
uint64_t t10ms_cnt = 0;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }

  if (!CAN.begin(CanBitRate::BR_1000k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }

  // Setup D0 to D4 as inputs
  for (int pin_no=0; pin_no<=4; pin_no++)
  {
    pinMode(pin_no, INPUT);
  }

  // Setup D5 to D9 as outputs
  for (int pin_no=5; pin_no<=9; pin_no++)
  {
    pinMode(pin_no, OUTPUT);
  }

  beginT10msTimer(100);   // 1/100Hz = 10ms
}


unsigned int msg_cnt = 0;
bool msg_rcvd = false;

byte inputsStatus = 0;
byte outputsStatus = 0;
// List of messages from protocol document
#define MSG_HEARTBEAT   0x01
#define MSG_READ_INPUTS 0x02
#define MSG_SET_OUTPUTS 0x03

// List of devices on CAN network from protocol document
#define DEV_ADDRESS_MAIN_CTRLR  0x01
#define DEV_ADDRESS_ARDUINO     0x02

// Bitmap for output signals
#define D5_OUT_BIT1 0x01
#define D6_OUT_BIT1 0x02
#define D7_OUT_BIT1 0x04
#define D8_OUT_BIT1 0x08
#define D9_OUT_BIT1 0x10

bool CAN_comm_alive = 0;

uint64_t prevHeartBeatMsgTime = 0;


// callback method used by T10ms_timer
void T10ms_callback(timer_callback_args_t __attribute((unused)) *p_args) {
  t10ms_cnt++;
}

bool beginT10msTimer(float rate) {
  uint8_t timer_type = GPT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0) {
    tindex = FspTimer::get_available_timer(timer_type, true);
  }
  if (tindex < 0) {
    return false;
  }

  FspTimer::force_use_of_pwm_reserved_timer();

  if (!T10ms_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, T10ms_callback)) {
    return false;
  }

  if (!T10ms_timer.setup_overflow_irq()) {
    return false;
  }

  if (!T10ms_timer.open()) {
    return false;
  }

  if (!T10ms_timer.start()) {
    return false;
  }
  return true;
}

void loop()
{
  if (CAN.available())    // CAN data received
  {
    CAN_receive();
  }
  else
  {
    if (CAN_comm_alive != 0)
    {
      if (t10ms_cnt-prevHeartBeatMsgTime > 30) {
        CAN_comm_alive = 0;
        Serial.println("CAN comm lost heartbeat");
        clearAllOutputs();
        Serial.println("Cleared all outputs");
      }

      CAN_transmit(MSG_READ_INPUTS);
    }
  }
}

void CAN_transmit(byte msg_nmb)
{
  CanMsg msg_tx;
  switch(msg_nmb)
  {
    case MSG_READ_INPUTS:

        msg_tx.id = DEV_ADDRESS_ARDUINO << 21 | MSG_READ_INPUTS << 13;
        msg_tx.id |= CanMsg::CAN_EFF_FLAG;
        msg_tx.data_length = 1;
        
        // Read inputs
        inputsStatus = 0;
        inputsStatus =  (digitalRead(0) != 0) |           // D0 Input
                        (digitalRead(1) != 0) << 1 |      // D1 Input
                        (digitalRead(2) != 0) << 2 |      // D2 Input
                        (digitalRead(3) != 0) << 3 |      // D3 Input
                        (digitalRead(4) != 0) << 4;       // D4 Input
        msg_tx.data[0] = inputsStatus;    
        break;

      default: break;

  }

  if (int const rc = CAN.write(msg_tx); rc < 0)
  {
    Serial.print  ("CAN.write(...) failed with error code ");
    Serial.println(rc);
  }
  delayMicroseconds(125);
}

void CAN_receive(void)
{
  CanMsg msg_rx;
  if (CAN.available())
  {
    msg_rx = CAN.read();

    // Make sure can message is extended message
    byte msg_nmb = (byte)(msg_rx.id >> 13 & 0x000000FF);
    byte dev_nmb = (byte)(msg_rx.id >> 21 & 0x000000FF);
    if (dev_nmb == DEV_ADDRESS_MAIN_CTRLR)
    {
      switch(msg_nmb)
      {
        case MSG_HEARTBEAT : 
                  Serial.print(msg_rx);
                  Serial.print("MSG_HEARTBEAT ");
                  Serial.print("msg_nmb=");
                  Serial.print(msg_nmb);
                  Serial.print(", dev_nmb=");
                  Serial.println(dev_nmb);
                  CAN_comm_alive = 1;
                  prevHeartBeatMsgTime = t10ms_cnt;
                  break;

        case MSG_SET_OUTPUTS : 
                  Serial.print(msg_rx);
                  Serial.print(" MSG_SET_OUTPUTS ");
                  Serial.print("msg_nmb=");
                  Serial.print(msg_nmb);
                  Serial.print(", dev_nmb=");
                  Serial.println(dev_nmb);

                  outputsStatus = msg_rx.data[0];
                  if ((outputsStatus & D5_OUT_BIT1) != 0) { digitalWrite(5,HIGH);  } else { digitalWrite(5,LOW); };
                  if ((outputsStatus & D6_OUT_BIT1) != 0) { digitalWrite(6,HIGH);  } else { digitalWrite(6,LOW); };
                  if ((outputsStatus & D7_OUT_BIT1) != 0) { digitalWrite(7,HIGH);  } else { digitalWrite(7,LOW); };
                  if ((outputsStatus & D8_OUT_BIT1) != 0) { digitalWrite(8,HIGH);  } else { digitalWrite(8,LOW); };
                  if ((outputsStatus & D9_OUT_BIT1) != 0) { digitalWrite(9,HIGH);  } else { digitalWrite(9,LOW); };
                  
                  break;

        default:  break;
      }
    }
  }
}

void clearAllOutputs(void)
{
  for (int pin_no=5;pin_no<=9;pin_no++)
  {
    digitalWrite(pin_no,LOW);
  }
}