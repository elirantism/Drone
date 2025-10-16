#include <Wire.h>
#include <PulsePosition.h>

PulsePositionInput receiverInput(RISING);

float measuredRateRoll, measuredRatePitch, measuredRateYaw;
float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;

float receiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int channelNumber = 0;

float desiredPitchRate;
float desiredRollRate;
float desiredYawRate;

float pConstantRollPitch = 0.6f;
float iConstantRollPitch = 3.5f;
float dConstantRollPitch = 0.03f;

float pConstantYaw = 2.0f;
float iConstantYaw = 12.0f;
float dConstantYaw = 0.0f;

float maxITerm = 400.0f;

uint32_t loopTimer;

float prevErrorPitch = 0.0;
float prevErrorRoll = 0.0;
float prevErrorYaw = 0.0;

float tempTotalITerm = 0;
float totalITermPitch = 0.0;
float totalITermRoll = 0.0;
float totalITermYaw = 0.0;

float outputMotor1;
float outputMotor2;
float outputMotor3;
float outputMotor4;

void readReceiver() {
  channelNumber = receiverInput.available();
  if (channelNumber > 0) {
    for (int i = 1; i < channelNumber; i++) {
      receiverValue[i - 1] = receiverInput.read(i);
    }
  }
}

void readGyro() {
  // Low Pass Filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configures sensitivity +-500 degrees per second
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
 
  // Begins reading from register 43 to 49, which contains all gyro values
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  measuredRateRoll = (float)gyroX / 65.5;
  measuredRatePitch = (float)gyroY / 65.5;
  measuredRateYaw = (float)gyroZ / 65.5;
}

// One func for all PID
float pidControlLoop(float desiredRate, float measuredRate, float prevError, float totalITerm,
                     float pConstant, float iConstant, float dConstant) {
  float error = desiredRate - measuredRate;

  float pTerm = pConstant * error;
  totalITerm += (iConstant * (error + prevError) * 0.004 * 0.5);
  // In future maybe introduce I term decay, I think that will make flight smoother
  totalITerm = constrain(totalITerm, -400, 400);
  tempTotalITerm = totalITerm;
  float dTerm = dConstant * (error - prevError) / 0.004;

  float totalOutput = pTerm + totalITerm + dTerm;
  totalOutput = constrain(totalOutput, -400, 400);

  return totalOutput;
}

void applyMotorOutputs(float pitchCorrection, float rollCorrection, float yawCorrection) {
  outputMotor1 = receiverValue[2] - rollCorrection - pitchCorrection - yawCorrection;
  outputMotor2 = receiverValue[2] - rollCorrection + pitchCorrection + yawCorrection;
  outputMotor3 = receiverValue[2] + rollCorrection + pitchCorrection - yawCorrection;
  outputMotor4 = receiverValue[2] + rollCorrection - pitchCorrection + yawCorrection;

  outputMotor1 = constrain(outputMotor1, 1000, 2000);
  outputMotor2 = constrain(outputMotor2, 1000, 2000);
  outputMotor3 = constrain(outputMotor3, 1000, 2000);
  outputMotor4 = constrain(outputMotor4, 1000, 2000);

  if (receiverValue[2] < 1050) {
    outputMotor1 = 1000;
    outputMotor2 = 1000;
    outputMotor3 = 1000;
    outputMotor4 = 1000;
    prevErrorPitch = 0.0;
    prevErrorRoll = 0.0;
    prevErrorYaw = 0.0;
    totalITermPitch = 0.0;
    totalITermRoll = 0.0;
    totalITermYaw = 0.0;
  }

  analogWriteResolution(12);

  analogWrite(1, 1.024 * outputMotor1);
  analogWrite(2, 1.024 * outputMotor2);
  analogWrite(3, 1.024 * outputMotor3);
  analogWrite(4, 1.024 * outputMotor4);
}

void setup() {
  Serial.begin(57600);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  analogWriteFrequency(1, 250);
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);

  for (int i = 0; i < 2000; i++) {
    readGyro();
    rateCalibrationRoll += measuredRateRoll;
    rateCalibrationPitch += measuredRatePitch;
    rateCalibrationYaw += measuredRateYaw;
    delay(1);
  }

  rateCalibrationRoll /= 2000;
  rateCalibrationPitch /= 2000;
  rateCalibrationYaw /= 2000;

  receiverInput.begin(14);

  while (receiverValue[2] > 1020 && receiverValue[2] < 1050) {
    readReceiver();
    delay(4);
  }
 
  loopTimer = micros();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  readReceiver();

  desiredPitchRate = 0.15 * (receiverValue[1] - 1500);
  desiredRollRate = 0.15 * (receiverValue[0] - 1500);
  desiredYawRate = 0.15 * (receiverValue[3] - 1500);

  readGyro();
  measuredRateRoll -= rateCalibrationRoll;
  measuredRatePitch -= rateCalibrationPitch;
  measuredRateYaw -= rateCalibrationYaw;

  Serial.print("Gyro Readings: ");
  Serial.print(measuredRatePitch);
  Serial.print(" | ");
  Serial.print(measuredRateRoll);
  Serial.print(" | ");
  Serial.println(measuredRateYaw);

  float pitchCorrection = pidControlLoop(desiredPitchRate, measuredRatePitch, prevErrorPitch, totalITermPitch, pConstantRollPitch, iConstantRollPitch, dConstantRollPitch);
  prevErrorPitch = desiredPitchRate - measuredRatePitch;
  totalITermPitch = tempTotalITerm;

  float rollCorrection = pidControlLoop(desiredRollRate, measuredRateRoll, prevErrorRoll, totalITermRoll, pConstantRollPitch, iConstantRollPitch, dConstantRollPitch);
  prevErrorRoll = desiredRollRate - measuredRateRoll;
  totalITermRoll = tempTotalITerm;

  float yawCorrection = pidControlLoop(desiredYawRate, measuredRateYaw, prevErrorYaw, totalITermYaw, pConstantYaw, iConstantYaw, dConstantYaw);
  prevErrorYaw = desiredYawRate - measuredRateYaw;
  totalITermYaw = tempTotalITerm;

  applyMotorOutputs(pitchCorrection, rollCorrection, yawCorrection);

  while (micros() - loopTimer < 4000);
  loopTimer = micros();
}
