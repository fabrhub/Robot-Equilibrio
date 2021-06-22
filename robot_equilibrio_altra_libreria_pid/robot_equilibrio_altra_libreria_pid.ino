#include <PID_v1.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

#define enA 9
#define IN1 4
#define IN2 5
#define enB 10
#define IN3 6
#define IN4 7

MPU6050 mpu6050(Wire);

double SetpointPID, InputPID, OutputPID;
double Kp = 22, Ki = 0.01, Kd = 0.4;
bool fallDirection; //true-avanti, false-dietro

PID myPID(&InputPID, &OutputPID, &SetpointPID , Kp, Ki, Kd, DIRECT);

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
  mpu6050.calcGyroOffsets(true); //calibrazione (metti true come parametro se vuoi stampare lo stato)

  SetpointPID = -0.8; //obiettivo target -> punto di equilibrio da cambiare in base al sensore, valore intorno allo 0
  myPID.SetMode(AUTOMATIC);

}

void loop() {
  readValueFromSensor();
  myPID.Compute();
  moveMotor(OutputPID, fallDirection);
  printStatus();
}


void readValueFromSensor() {
  mpu6050.update();
  InputPID = mpu6050.getAngleY();
  if (InputPID > SetpointPID) {
    //caduta in avanti
    fallDirection = true;
    myPID.SetControllerDirection(REVERSE);
  } else if (InputPID < SetpointPID) {
    //caduta in dietro
    fallDirection = false;
    myPID.SetControllerDirection(DIRECT);
  }
}

void moveMotor(double OutputPID, bool fallDirection) {
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
