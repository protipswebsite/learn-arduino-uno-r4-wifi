//  Title - Arduino UNO R4 WiFi Flash Data Memory as emulated EEPROM
//  Author - Pro.Tips
//  Arduino UNO R4 WiFi Flash Data Memory as emulated EEPROM
//  https://pro.tips/courses/learn-arduino-uno-r4-wifi-free-online-course/lesson/arduino-uno-r4-wifi-flash-data-memory-as-emulated-eeprom/

#include <EEPROM.h>

#define ADDR_VALID_EEPROM_DATA_MARKER_BYTE_0 0
#define ADDR_VALID_EEPROM_DATA_MARKER_BYTE_1 1
#define ADDR_FIRE_ALARM_STATUS_BYTE 2

#define EEPROM_VALID_MARKER_BYTE_0  0xAA
#define EEPROM_VALID_MARKER_BYTE_1  0xBD

byte fireAlarmStatus = 0;

byte bitTable[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

// on power-up initialize eeprom saved data
void init_eeprom()
{
  byte eepromMarkerByte0 = EEPROM.read(ADDR_VALID_EEPROM_DATA_MARKER_BYTE_0);
  byte eepromMarkerByte1 = EEPROM.read(ADDR_VALID_EEPROM_DATA_MARKER_BYTE_1);

  if ((eepromMarkerByte0 == EEPROM_VALID_MARKER_BYTE_0) &&
      (eepromMarkerByte1 == EEPROM_VALID_MARKER_BYTE_1) )
  {
    fireAlarmStatus = EEPROM.read(ADDR_FIRE_ALARM_STATUS_BYTE);
    Serial.println("Valid data read from EEPROM upon powerup");
    Serial.print("fireAlarmStatus=");
    Serial.println(fireAlarmStatus);
  }
  else
  {
    Serial.println("First time powered up EEPROM written");
    EEPROM.write(ADDR_VALID_EEPROM_DATA_MARKER_BYTE_0, EEPROM_VALID_MARKER_BYTE_0);
    EEPROM.write(ADDR_VALID_EEPROM_DATA_MARKER_BYTE_1, EEPROM_VALID_MARKER_BYTE_1);
    EEPROM.write(ADDR_FIRE_ALARM_STATUS_BYTE,0);
  }
}

void setup() {
  Serial.begin(115200);

  delay(1000);

  // put your setup code here, to run once:
  init_eeprom();

  // configure digital pins from D0 to D7 as inputs 
  for (int pin_no=0; pin_no<=7; pin_no++)
  {
    pinMode(pin_no, INPUT);
  }

  // configure digital pin D8 as fire alarm reset input
  pinMode(8, INPUT);
}

void loop() {
  
  // Read fire alarm input signal and set fireAlarmStatus
  for (int i=0; i<8; i++)
  {
    if (digitalRead(i) != 0)
    {
      if ((fireAlarmStatus & bitTable[i]) == 0)
      {
        fireAlarmStatus |= bitTable[i];
        EEPROM.write(ADDR_FIRE_ALARM_STATUS_BYTE,fireAlarmStatus);
        Serial.print("fireAlarmStatus=");
        Serial.println(fireAlarmStatus);
      }
    }
  }

  // Read fire alarm reset input signal and clear fireAlarmStatus
  if (digitalRead(8) != 0)
  { 
    if (fireAlarmStatus != 0)
    {
      fireAlarmStatus = 0;
      EEPROM.write(ADDR_FIRE_ALARM_STATUS_BYTE,fireAlarmStatus);
      Serial.print("fireAlarmStatus=");
      Serial.println(fireAlarmStatus);
    }
  }

}


