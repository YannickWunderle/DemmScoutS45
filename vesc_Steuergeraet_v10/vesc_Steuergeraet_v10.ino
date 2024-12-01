/*Autor: Yannick Wunderle, Jona Mainberger
   Datum: 14.03.2021
   Funktion: Steuergerät DEMM mit CMDV93S Motor


   RX Gelb
   TX Grün
   5V Braun
   SDA Schwarz
   SCL Weiß

   VESC Library Timeout von 100 auf 5 und wieder zurück auf 100.
   Wire.h Library Timeout von 50 auf 5ms
   I2c taskt vom ESP hat vierfachen Speicherplatz zugewiesen
*/
#include <Wire.h>
#include "vesc_uart/VescUart.h"
#include <EEPROM.h>

#define EEPROM_SIZE 2

//PINS

#define BEEPER_PIN A2
#define RELAIS_PIN A3
#define BRAKE_PIN_L 2
#define BRAKE_PIN_R 3
#define THROTTLE_PIN A6
#define DEBUG_I2C_LED 13
//#define SDA_PIN 21
//#define SCL_PIN 5

//VESC Motor and Battery
#define BATTERY_CELLS 15
#define LOW_THROTTLE_VALUE 190
#define HIGH_THROTTLE_VALUE 850
#define MAX_CURRENT 120
#define BRAKE_CURRENT_WEAK 20
#define BRAKE_CURRENT_MEDIUM 40
#define BRAKE_CURRENT_HARD 70

#define CYCLE_TIME 30
#define FAULT_TIME 1000
#define BEEP_TIME 200
#define TEMP_NC -50

//Gearing
#define Z1 26
#define Z2 171
#define MagnetPairs 5
#define TacPerErev 6 //constant of the vesc, three states going through two magnets (one pole pair)

//Makes ESP go silent if there is no communication instead of beeping forever and shutting everything off. Useful for testing with vesc tool.
#define DEBUG (false)

#define I2C_ADDRESS 8

const float WHEEL_CIR = 1.66; //m
const float LOW_VOLTAGE = 2.7;
const float PRE_CHARGE_DROPOFF = 2; // when charging the capacitors through the resistor the voltage should at least reach the minimum voltage minus this value
unsigned long previousMillisFault = 0;
unsigned long previousMillisPrint = 0;
unsigned long previousMillisCycle = 0;

const long erotPerKm = MagnetPairs * 1000L * Z2 / Z1 / WHEEL_CIR;

unsigned long lastI2CMillis = 0;


// setting PWM properties

uint16_t kmAbs = 0;
uint16_t Km = 0;

VescUart UART;

bool serving_request = false;

void serveRequest();
void resetBus();

void setup() {
  pinMode(BEEPER_PIN, OUTPUT);
  pinMode(RELAIS_PIN, OUTPUT);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(BRAKE_PIN_L, INPUT);
  pinMode(BRAKE_PIN_R, INPUT);
  
  pinMode(DEBUG_I2C_LED, OUTPUT);

  digitalWrite(BEEPER_PIN, LOW);

  Serial.begin(115200);


  while (!Serial) ; //VESC

  UART.setSerialPort(&Serial);

  digitalWrite(BEEPER_PIN, HIGH);
  delay(BEEP_TIME);
  digitalWrite(BEEPER_PIN, LOW);

  if (!DEBUG) {
    while (UART.data.inpVoltage < BATTERY_CELLS * LOW_VOLTAGE - PRE_CHARGE_DROPOFF) {
      UART.getVescValues();
      delay(100);
    }
  }
  else {
    delay(1000);
  }

  digitalWrite(RELAIS_PIN, HIGH);
  digitalWrite(BEEPER_PIN, HIGH);
  delay(BEEP_TIME);
  digitalWrite(BEEPER_PIN, LOW);

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(serveRequest);

  kmAbs = getKmAbs();
}

uint16_t getKmAbs() {
  uint8_t MSB_Read = EEPROM.read(0);
  uint8_t LSB_Read = EEPROM.read(1);
  uint16_t kmAbs_Read = LSB_Read + (MSB_Read << 8);
  return kmAbs_Read;

}

void saveKmAbs() {
  kmAbs = constrain(kmAbs + Km, 0, 0xFFFF);
  uint8_t LSB = kmAbs;
  uint8_t MSB = kmAbs >> 8;
  EEPROM.write(0, MSB);
  EEPROM.write(1, LSB);
}



void serveRequest() {
  if (serving_request) {
    return;
  }
  serving_request = true;
  digitalWrite(DEBUG_I2C_LED,1);

  const int baseInputVoltage = 2;
  const int baseMotorTemp = -20;
  const int AhFactor = 10;
  const int voltFactor = 100;
  const int kmhFactor = 3;
  const int kmFactor = 100;

  const int msgLen = 9;

  uint16_t km = constrain((long)UART.data.tachometer / TacPerErev * kmFactor / erotPerKm, 0, 0xFFFF);
  Km = km / kmFactor;
  uint16_t kmabs = constrain(kmAbs + Km, 0, 0xFFFF);

  uint8_t msg[msgLen + 1] = {
    constrain(abs(UART.data.avgInputCurrent), 0, 255),
    km,
    km >> 8,
    constrain((UART.data.ampHours * AhFactor), 0, 255),
    kmabs,
    kmabs >> 8,
    constrain((UART.data.inpVoltage / BATTERY_CELLS - baseInputVoltage)*voltFactor, 0, 255),
    constrain(UART.data.rpm * 3 * 60 / erotPerKm, 0, 255),
    constrain(UART.data.tempMotor - baseMotorTemp, 0, 255),
    0 //csum
  };

  for (int i = 0; i < msgLen; i++) {
    msg[msgLen] += msg[i];
  }


  Wire.write(msg, sizeof(msg));
  serving_request = false;
  digitalWrite(DEBUG_I2C_LED,0);
  
  lastI2CMillis = millis();
}

void fault(int faultType) {
  /* 0 LOw voltage
     1 no communication
     2 no TempSensor
  */
  saveKmAbs();
  UART.setCurrent(float(0));

  for (int i = faultType; i >= 0; i--) {
    digitalWrite(BEEPER_PIN, LOW);
    delay(200);
    digitalWrite(BEEPER_PIN, HIGH);
    delay(200);
  }
  digitalWrite(RELAIS_PIN, LOW);
  while (1);
}

void checkTempSensor() {
  if (UART.data.tempMotor <  TEMP_NC) {
    fault(2);
  }
}

bool getIsBreaking() {
  bool ret = (!digitalRead(BRAKE_PIN_L) or !digitalRead(BRAKE_PIN_R));
  return ret;
}

void SetGas() {
  int current = analogRead(THROTTLE_PIN);
  current = constrain(current, LOW_THROTTLE_VALUE, HIGH_THROTTLE_VALUE);
  current = map(current, LOW_THROTTLE_VALUE , HIGH_THROTTLE_VALUE, 0, MAX_CURRENT);
  UART.setCurrent((float)current);
}

void SetBreak() {
  float brakeCurrent = 0;
  bool Brake_L = digitalRead(BRAKE_PIN_L);
  bool Brake_R = digitalRead(BRAKE_PIN_R);

  if (!Brake_L and Brake_R) {
    brakeCurrent = BRAKE_CURRENT_WEAK;
  }
  if (Brake_L and !Brake_R) {
    brakeCurrent = BRAKE_CURRENT_MEDIUM;
  }
  if (!Brake_L and !Brake_R) {
    brakeCurrent = BRAKE_CURRENT_HARD;
  }
  UART.setBrakeCurrent(brakeCurrent);
}


void loop() {
  previousMillisCycle = millis();
  static unsigned long lastValues = millis();
  //Serial.println(analogRead(THROTTLE_PIN)); for reading min and max throttle

  const int max_retries = 1;
  for (int retry = 0; retry < max_retries; retry++) {
    if (getIsBreaking()) {
      SetBreak();
    }
    else {
      SetGas();
    }
    if (UART.getVescValues()){
      lastValues = millis();

      if (UART.data.inpVoltage < BATTERY_CELLS * LOW_VOLTAGE) {
        fault(0);
      }
      break;
    }
  }

  if (millis() - lastI2CMillis > 500){
    Wire.end();
    Wire.begin(I2C_ADDRESS);
  }
  if (millis() - lastValues > 30000) {
    if (!DEBUG) {
      fault(1);
    }
  }
  checkTempSensor();
  while (millis() - previousMillisCycle < CYCLE_TIME);
}
