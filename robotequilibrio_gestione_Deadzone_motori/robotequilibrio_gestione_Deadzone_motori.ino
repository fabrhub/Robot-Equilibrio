#include <QuickPID.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

#define enA 9
#define IN1 4
#define IN2 5
#define enB 10
#define IN3 6
#define IN4 7

MPU6050 mpu6050(Wire);

float SetpointPID, InputPID, OutputPID;
float Kp = 10, Ki = 0.1, Kd = 100;

float sensorValue;
bool fallDirection; //true-avanti, false-dietro
float speedMot1, speedMot2; //valore PWM senza deadzone

QuickPID myQuickPID(&InputPID, &OutputPID, &SetpointPID, Kp, Ki, Kd, QuickPID::DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); //calibrazione automatica (metti true come parametro se vuoi stampare lo stato)
  //mpu6050.setGyroOffsets(0.00, -1.43, 0.00); //aggiusta offset asse Y manualmente, ci sono problemi con la calibrazione automatica

  SetpointPID = 0; //obiettivo target - equilibrio
  myQuickPID.SetMode(QuickPID::AUTOMATIC);

}

void loop() {
  readValueFromSensor();
  myQuickPID.Compute();
  removeMotorDeadZone(OutputPID);
  moveMotors(speedMot1, speedMot2, fallDirection);
  printStatus();
}


void readValueFromSensor() {
  mpu6050.update();
  sensorValue = mpu6050.getAngleY();
  if (sensorValue > 2.5) { //togliere roba hard coded, questo è una tolleranza dei valori letti dal sensore
    //caduta in avanti
    InputPID = sensorValue;
    fallDirection = true;
    myQuickPID.SetControllerDirection(QuickPID::REVERSE);
  } else if (sensorValue < -2.5) {
    //caduta in dietro
    InputPID = sensorValue;
    fallDirection = false;
    myQuickPID.SetControllerDirection(QuickPID::DIRECT);
  } else {
    InputPID = 0;
  }
}

void removeMotorDeadZone(float OutputPID) {
  if (OutputPID == 0) {
    speedMot1 = 0;
    speedMot2 = 0;
  } else {
    speedMot1 = map(OutputPID, 1, 255, 60, 255); //60 soglia di attivazione motore1
    speedMot2 = map(OutputPID, 1, 255, 90, 255); //90 soglia di attivazione motore2
  }
}

void moveMotors(float speedMot1, float speedMot2, bool fallDirection) {
  analogWrite(enA, speedMot1);
  analogWrite(enB, speedMot2);
  if (fallDirection == true) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

void printStatus() {
  Serial.print("InputPID:");
  Serial.print(InputPID);
  Serial.print(",OutputPID:");
  Serial.print(OutputPID);
  Serial.print(",speedMot1:");
  Serial.print(speedMot1);
  Serial.print(",speedMot2:");
  Serial.print(speedMot2);
  Serial.print(",sensorValue:");
  Serial.print(sensorValue);
  Serial.print(",SetpointPID:");
  Serial.println(SetpointPID);
}
