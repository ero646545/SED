/*
  Stereoelectrochemical Printer v1.0
  Sorbonne University, LISE, René Meng and Mixtral 8x7b
*/

// Pin definitions for stepper motor
#define PinDir A7
#define PinStep A6
#define PinEn A2
#define PinEndStop 2
#define PinCathode 4
#define PinFan 10
//#define Cathode 4.41//5-ddp

// Pin definitions for electrode array (example using pins 22 to 53)
#define ElectrodeStartPin 14
#define ElectrodeEndPin 52

// Libraries
#include <Stepper.h>

///////////////////////////////////////////////

// Global variables
const int stepsPerRevolution = 200; // Number of steps per revolution for the stepper motor
Stepper myStepper(stepsPerRevolution, PinStep, PinDir); // Define stepper motor object

///////////////////////////////////////////////
#include <Adafruit_INA219.h>
// Create an instance of the INA219 sensor
Adafruit_INA219 ina;
#define NUM_SAMPLES 1000 // number of samples to take for the moving average
const float targetCurrent = 1.0; // target current in milliamps
const float Kp = 1.5; // proportional gain
const float Ki = 1.5; // integral gain
const float Kd = 1.0; // derivative gain

float error = 0.0;
float prevError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pwmValue = 0.0;
float currentAvg = 0.0;
float Thickness;
int Target=40;
bool stop=false;

void setup() {
  ina.begin();
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Printer ready");
  // Initialize pins
  pinMode(PinDir, OUTPUT);
  pinMode(PinStep, OUTPUT);
  pinMode(PinEndStop, INPUT_PULLUP);
  pinMode(PinCathode, INPUT);
  
  // Enable the stepper 
  pinMode(PinEn, OUTPUT);
  digitalWrite(PinEn, LOW);
  // Set initial stepper motor speed and direction
  myStepper.setSpeed(400);
  digitalWrite(PinDir, HIGH); // Change direction if needed


  // Set all electrode array pins as outputs
  for (int pin = ElectrodeStartPin; pin <= ElectrodeEndPin; ++pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  
  pinMode(10, OUTPUT);
  digitalWrite(PinFan,LOW);
}
///////////////////////////////////////////////

// Function to home the Z-axis using the endstop
void homeZAxis() {
  bool Stop=false;
  // Rapidly move the Z-axis down until it hits the endstop
  while (digitalRead(PinEndStop) == HIGH) {
    myStepper.setSpeed(1000); // Set a fast speed
    myStepper.step(-4);
    delay(1);
    if (Serial.available()) {
        String message1;
        message1 = Serial.readStringUntil('\n');
        if (message1.equals("stop")){
          deactivateAllElectrodes();
          Stop=true;
          break;
        }
        }
  }

  for (int i = 0; i < 100; i++) {
        myStepper.setSpeed(1000); // Set a fast speed
        myStepper.step(4);
        delay(1);
        
    }
    // Slowly home the Z-axis using the endstop
    while (digitalRead(PinEndStop) == HIGH and Stop==false) {
      myStepper.setSpeed(1000); // Set a slower speed
      myStepper.step(-4);
      delay(10);
      if (Serial.available()) {
          String message1;
          message1 = Serial.readStringUntil('\n');
          if (message1.equals("stop")){
            deactivateAllElectrodes();
            break;
          }
          }
  }

  //myStepper.setCurrentPosition(0);
}
///////////////////////////////////////////////

// Function to send an image via serial and activate corresponding electrodes
void UpdateMatrix() {
  int n_electrode;
  deactivateAllElectrodes();
  // Activate corresponding electrodes based on the received data ex:
  String message="";// le milieu fonctionne pas
  for (int port = 14; port <= 52; port++) {
    if (port != 20 && port != 21) {
    if (message[port-14]=='0'){
      digitalWrite(port, 0);
      pinMode(port, INPUT);
    }
    else if (message[port-14]=='1') {
      pinMode(port, INPUT);
      digitalWrite(port, 0);
    }
    else if (port==52) {
      pinMode(port, OUTPUT);
      digitalWrite(port, HIGH);
    }
    else {
      digitalWrite(port, HIGH);
      pinMode(port, INPUT);
    }
      Serial.print(message[port-14]);
    }
    }
    Serial.println();
    //return n_electrode;//set target current
}


///////////////////////////////////////////////

// Function to deactivate all electrodes
void deactivateAllElectrodes() {
  // Turn off all electrode array pins
  for (int pin = ElectrodeStartPin; pin <= ElectrodeEndPin; ++pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);
  }
  analogWrite(PinCathode, 255);//Turn off Cathode
  pinMode(PinCathode, INPUT);
  digitalWrite(PinFan,LOW);


}
void(* reset) (void) = 0; //declare reset function @ address 0

///////////////////////////////////////////////
void PrintLayers(){
  for (int i = 0; i < 50; i++) {//height 550µm
          myStepper.setSpeed(1000);
          myStepper.step(+4);
          delay(1);
        }
        Thickness=0;pwmValue=0;currentAvg=0;
  UpdateMatrix();// One layer test first
  while(1) {
  float currentSum = 0.0;
  if (Serial.available() || Thickness >=1000) {
        String message1;
        message1 = Serial.readStringUntil('\n');
        if (message1.equals("stop")|| Thickness >=1000){
          Serial.println("Stopped");
          deactivateAllElectrodes();Thickness=0;pwmValue=0;currentAvg=0;
          
      delay(1000);
      break;
        }
        }
  if (stop==false){

    for (int i = 0; i < NUM_SAMPLES; i++) {
      currentSum += ina.getCurrent_mA();
      
    }
    currentAvg = currentSum / NUM_SAMPLES;
    error = targetCurrent - currentAvg;
    integral += error;
    derivative = error - prevError;
    pwmValue = Kp * error + Ki * integral + Kd * derivative;
    pwmValue = 255 - constrain(pwmValue, 0, 255);
    if (currentAvg<targetCurrent*2 || abs(derivative) < 0.5){// && currentAvg>0
    pinMode(PinCathode, OUTPUT);
    Thickness+=currentAvg*2/(28.74*targetCurrent);//mA 2s 0.5mm,0.65mm, 28°, 2electrodes
  }
  else{Shake();pinMode(PinCathode, INPUT);}
    analogWrite(PinCathode, pwmValue);
    Serial.print("Courant(mA):");Serial.print(currentAvg);
    Serial.print(",");
    Serial.print("Potentiel(V):");Serial.print(pwmValue/256*5);
    Serial.print(",");
    Serial.print("Derivee(mA):");Serial.print(derivative);
    Serial.print(",");
    Serial.print("Epaisseur(µm):");Serial.println(Thickness);
    prevError = error;
    delay(933);//time correction
    if (Thickness>Target) {
      myStepper.setSpeed(1000);
      myStepper.step(+4);
      Target+=40;
  }
 
  }
  else{deactivateAllElectrodes();Thickness=0;pwmValue=0;currentAvg=0;delay(1000);}
    }
}
void Shake(){
  String message1;
  myStepper.setSpeed(1000);
     for (int i = 0; i < 1000; i++) {//printing 10µm with 1 layers          
          myStepper.step(+4);
          delay(1);  
      }
      for (int i = 0; i < 1000; i++) {//printing 10µm with 1 layers          
          myStepper.step(-4);
          delay(1);   
      }
  
   for (int i = 0; i < 25; i++) {//printing 10µm with 1 layers          
      
          myStepper.step(+4);
          delay(10);
        
          myStepper.step(-4);
          delay(10);

      }
         for (int i = 0; i < 50; i++) {//printing 10µm with 1 layers          
      
          myStepper.step(+4);
          delay(2);
        
          myStepper.step(-4);
          delay(2);

      }
       for (int i = 0; i < 100; i++) {//printing 10µm with 1 layers          
      
          myStepper.step(+4);
          delay(1);
        
          myStepper.step(-4);
          delay(1);

      }
      
        
        
      }    
    

void loop() {
 
  if (Serial.available()) {
    String message1;

    message1 = Serial.readStringUntil('\n');
    
    if (message1.equals("home")) {
      Serial.println("homing");
      homeZAxis();
      Serial.println("ready");
    }
    else if (message1.equals("upi")) {
      for (int i = 0; i < 1; i++) {
        myStepper.setSpeed(1000);
        myStepper.step(+4);
        delay(1);
      Serial.println("ready");
      }}
    else if (message1.equals("up")) {
      for (int i = 0; i < 8000; i++) {
        myStepper.setSpeed(1000);
        myStepper.step(+4);
        delay(1);
        if (Serial.available()) {
        String message1;
        message1 = Serial.readStringUntil('\n');
        if (message1.equals("stop")){
          deactivateAllElectrodes();
          break;
        }
        }
      }
      Serial.println("ready");
      }
     
      
      else if (message1.equals("shake")) {
         
          Shake();
        
        
       
      Serial.println("ready");
      }
      else if (message1.equals("print")) {
          Serial.println("received, start printing ('stop' to abort)");
          PrintLayers();
          deactivateAllElectrodes();
          for (int i = 0; i < 8000; i++) {//remonte la cathode 
          myStepper.setSpeed(1000);
          myStepper.step(+4);
          delay(1);
          if (Serial.available()) {
          String message1;
          message1 = Serial.readStringUntil('\n');
          if (message1.equals("stop")){
            deactivateAllElectrodes();Thickness=0;pwmValue=0;currentAvg=0;delay(1000);reset();
            break;
          }
          }
          }
          Serial.println("ready");
        }
        
    Serial.println("ready");
  }
}






