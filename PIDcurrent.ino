#include <Adafruit_INA219.h>
// Create an instance of the INA219 sensor
Adafruit_INA219 ina;
#define NUM_SAMPLES 2000 // number of samples to take for the moving average
const float targetCurrent = 2.0; // target current in milliamps
const float Kp = 1.0; // proportional gain
const float Ki = 1; // integral gain
const float Kd = 1; // derivative gain

float error = 0.0;
float prevError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pwmValue = 0.0;
float currentAvg = 0.0;

void setup() {
  ina.begin();
  Serial.begin(9600);
  pinMode(9, OUTPUT);

}

void loop() {
  float currentSum = 0.0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    currentSum += ina.getCurrent_mA();
    
  }
  currentAvg = currentSum / NUM_SAMPLES;

  error = targetCurrent - currentAvg;
  integral += error;
  derivative = error - prevError;
  pwmValue = Kp * error + Ki * integral + Kd * derivative;
  pwmValue = constrain(pwmValue, 0, 255);

  analogWrite(9, pwmValue);


  Serial.println(currentAvg);
  //Serial.println(",");
  //Serial.println(pwmValue);

  prevError = error;

}
