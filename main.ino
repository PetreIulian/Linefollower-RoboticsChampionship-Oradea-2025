/*
    * Copyright (C) 2025 by Petre Ioan-Iulian
    * 611AB UNSTPB-FIIR SIA-II
*/

#include <DRV8835MotorShield.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define Max_Speed 400

//Sensor Config
#define SENSOR_COUNT 7
const int sensorPins[SENSOR_COUNT] = {16, 17, 18, 19, 21, 22, 23};

//Sensor Weights
int sensorWeights[SENSOR_COUNT] = {-3, -2, -1, 0, 1, 2, 3};

//----------------------- PID CONFIG -----------------------
double Kp = 56.35; //56.35
double Kd = 135.445;//137.4455
double Ki = 0.1525; //0.155  

double baseSpeed = 250; //175
double error = 0, lastError = 0, integral = 0;

bool robotActive = false;

//Motor Config
#define M1DIR 32
#define M1PWM 33
#define M2DIR 26
#define M2PWM 25
DRV8835MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);

//Print Values via Bluetooth
void print_values(float PID, float left, float right) {
  Serial.println("------------------------");
  Serial.println("Sensor Values:");
  for(uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    uint8_t sensorValue = digitalRead(sensorPins[i]);
    BTSerial.print("Sensor ");
    BTSerial.print(i + 1);
    BTSerial.print(" (Pin ");
    BTSerial.print(sensorPins[i]);
    BTSerial.print("): ");
    BTSerial.println(sensorValue);
  }
  BTSerial.print("PID: ");
  BTSerial.println(PID);
  BTSerial.print("Left Motor Speed: ");
  BTSerial.println(left);
  BTSerial.print("Right Motor Speed: ");
  BTSerial.println(right);
  BTSerial.println("------------------------");
}

//Error Function
double Calculate_Error() {
  int activeCount = 0;
  double weightedSum = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (digitalRead(sensorPins[i])) {
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }

  if (activeCount > 0) {
    error = weightedSum / (float)activeCount;
  }

  //Deadzone
  if (abs(error) < 0.15) { //Adjustable filed depending on your linesensor array specification
    error = 0;
  }

  return error;
}

//PID Function
double PID(double error) {
  integral +=error ;
  float derivative = error - lastError;
  float PID = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  return PID;
}

void setup() {
  
  Serial.begin(115200);
  delay(400);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("ON");

  SerialBT.begin("ESP32Muerta");
  Serial.println("Waiting for Bluetooth");
}

void loop() {

  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    if (cmd == 's') { 
      robotActive = false;
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      Serial.println("Robot ON");
    } else if (cmd == 'g') { 
          robotActive = true;
          Serial.println("Robot ON");
        }
  }

  if(robotActive) {
    float error = Calculate_Error();
    float correction = PID(error);


    float left = baseSpeed - correction;
    float right = baseSpeed + correction;

    left= constrain(left, -Max_Speed, Max_Speed);
    right = constrain(right, -Max_Speed, Max_Speed);

    motors.setM1Speed(left);
    motors.setM2Speed(right);

    print_values(correction, left, right);

    delay(10);
  }  
}
