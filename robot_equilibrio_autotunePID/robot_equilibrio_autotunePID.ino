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
float Kp = 22, Ki = 0.1, Kd = 0.4;
float POn = 0.5;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0

bool fallDirection; //true-avanti, false-dietro
bool pidLoop = false;
bool printOrPlotter = 0;  // on(1) monitor, off(0) plotter

byte hysteresis = 1;
byte outputStep = 5;

int setpoint = 341;       // 1/3 of range for symetrical waveform
int output = 85;        // 1/3 of range for symetrical waveform

const uint32_t sampleTimeUs = 10000; // 10ms

QuickPID myQuickPID(&InputPID, &OutputPID, &SetpointPID, Kp, Ki, Kd, POn, DOn, QuickPID::DIRECT);

void setup() {
  Serial.begin(115200);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); //calibrazione (metti true come parametro se vuoi stampare lo stato)

  SetpointPID = 0; //obiettivo target -> punto di equilibrio

  myQuickPID.AutoTune(tuningMethod::ZIEGLER_NICHOLS_PID);
  myQuickPID.autoTune->autoTuneConfig(outputStep, hysteresis, setpoint, output, QuickPID::DIRECT, printOrPlotter, sampleTimeUs);
}

void loop() {
  //----
  if (myQuickPID.autoTune) // Avoid dereferencing nullptr after _myPID.clearAutoTune()
  {
    switch (myQuickPID.autoTune->autoTuneLoop()) {
      case myQuickPID.autoTune->AUTOTUNE:
        readValueFromSensor();
        moveMotor(OutputPID, fallDirection);
        break;

      case myQuickPID.autoTune->TUNINGS:
        myQuickPID.autoTune->setAutoTuneConstants(&Kp, &Ki, &Kd); // set new tunings
        myQuickPID.SetMode(QuickPID::AUTOMATIC); // setup PID
        myQuickPID.SetSampleTimeUs(sampleTimeUs);
        myQuickPID.SetTunings(Kp, Ki, Kd, POn, DOn); // apply new tunings to PID
        break;

      case myQuickPID.autoTune->CLR:
        if (!pidLoop) {
          myQuickPID.clearAutoTune(); // releases memory used by AutoTune object
          pidLoop = true;
        }
        break;
    }
  }
  //----
  if (pidLoop) {
    if (printOrPlotter == 0) { // plotter
      Serial.print("Setpoint:");  Serial.print(SetpointPID);  Serial.print(",");
      Serial.print("Input:");     Serial.print(InputPID);     Serial.print(",");
      Serial.print("Output:");    Serial.print(OutputPID);    Serial.println(",");
    }
    readValueFromSensor();
    myQuickPID.Compute();
    moveMotor(OutputPID, fallDirection);
    //printStatus();
  }
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
  Serial.print(SetpointPID);
  Serial.print(",Kp:");
  Serial.print(Kp);
  Serial.print(",Ki:");
  Serial.print(Ki);
  Serial.print(",Kd:");
  Serial.println(Kd);
}
