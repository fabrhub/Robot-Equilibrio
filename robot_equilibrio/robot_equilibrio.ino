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
float Kp = 10, Ki = 0.2, Kd = 0.1;
bool fallDirection; //true-avanti, false-dietro

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
  mpu6050.calcGyroOffsets(); //calibrazione (metti true come parametro se vuoi stampare lo stato)

  SetpointPID = 0; //obiettivo target -> punto di equilibrio
  myQuickPID.SetMode(QuickPID::AUTOMATIC);

}

void loop() {
  readValueFromSensor();
  myQuickPID.Compute();
  moveMotor(OutputPID, fallDirection);
  printStatus();
}


void readValueFromSensor() {
  mpu6050.update();
  InputPID = mpu6050.getAngleY();
  if (InputPID > SetpointPID) {
    //caduta in avanti
    fallDirection = true;
    myQuickPID.SetControllerDirection(QuickPID::REVERSE);
  } else if (InputPID < SetpointPID) {
    //caduta in dietro
    fallDirection = false;
    myQuickPID.SetControllerDirection(QuickPID::DIRECT);
  }
}

void moveMotor(float OutputPID, bool fallDirection) {
  analogWrite(enA, OutputPID);
  analogWrite(enB, OutputPID);
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
  Serial.print(",SetpointPID:");
  Serial.println(SetpointPID);
}
