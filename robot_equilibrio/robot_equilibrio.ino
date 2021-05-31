#include <MPU6050_tockn.h>
#include <Wire.h>

#define enA 9
#define IN1 4
#define IN2 5
#define enB 10
#define IN3 6
#define IN4 7

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(enA, 150);
  analogWrite(enB, 150);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); //calibrazione
}

void loop() {
  mpu6050.update();
  Serial.println(mpu6050.getAngleY());
  correzioneEquilibrio(mpu6050.getAngleY());
}

//-------

void correzioneEquilibrio(float angolo) {
  if (angolo > 7) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(200);
    brake();
  } else if (angolo < -7) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(200);
    brake();
  }
}

void brake()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
