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
#include <List.hpp>
///////////////////////////////////////////////

// Global variables
const int stepsPerRevolution = 200; // Number of steps per revolution for the stepper motor
Stepper myStepper(stepsPerRevolution, PinStep, PinDir); // Define stepper motor object

///////////////////////////////////////////////
#include <Adafruit_INA219.h>
// Create an instance of the INA219 sensor
Adafruit_INA219 ina;
#define NUM_SAMPLES 1000 // number of samples to take for the moving average
// target current in milliamps

float targetCurrent = 1;
float electrode_current = 1;
float error = 0.0;
float prevError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pwmValue = 0.0;
float currentAvg = 0.0;
float Thickness;
int Target=40;//premiere couche plus diffuse
bool stop=false;
String matrix="";
float lheight=40;
List<int> active_anode;
List<int> depositHeights;
int ieg=80;
int height = 0;
int n_electrode=0;
int eh=0;
int shake=0;
void setup() {
  ina.begin();
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Printer ready");
  // Initialize pins
  pinMode(PinDir, OUTPUT);
  pinMode(PinStep, OUTPUT);
  pinMode(PinEndStop, INPUT_PULLUP);
  pinMode(PinCathode, INPUT_PULLUP);
  
  // Enable the stepper 
  pinMode(PinEn, OUTPUT);
  digitalWrite(PinEn, LOW);
  // Set initial stepper motor speed and direction
  myStepper.setSpeed(400);
  digitalWrite(PinDir, HIGH); // Change direction if needed


  // Set all electrode array pins as outputs
  for (int pin = ElectrodeStartPin; pin <= ElectrodeEndPin; ++pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
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
    myStepper.setSpeed(8000); // Set a fast speed
    myStepper.step(-4);
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
        myStepper.setSpeed(800); // Set a fast speed
        myStepper.step(4);
        
    }
    // Slowly home the Z-axis using the endstop
    while (digitalRead(PinEndStop) == HIGH and Stop==false) {
      myStepper.setSpeed(80); // Set a slower speed
      myStepper.step(-4);
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
void ehome() {//RETIRER Update matrix causé par le shake

  float currentSum = 0;
  float currentAvg;
  bool stopDescent=false;
  int hcount=0;//no need to reset mask because it's already deactivated, here it's just remask
  depositHeights.clear();
  List<int> save_anode;
  deactivateAllElectrodes();   
  pinMode(PinCathode, OUTPUT);
  analogWrite(PinCathode, 240);
  for (int i = 0; i < 60*8; i++) {//descend un coup
              myStepper.setSpeed(800);
              myStepper.step(-4);
            }
  for (int i = 0; i < active_anode.getSize(); i++) {
  pinMode(active_anode[i], OUTPUT);
  digitalWrite(active_anode[i], HIGH);
  Serial.print("next electrode");
  Serial.println(active_anode[i]);
  stopDescent=false;
  delay(2000);
  while (!stopDescent) {
        
        currentSum = 0;
        for (int j = 0; j < 50; j++) {
          currentSum += ina.getCurrent_mA();
        }
        
        currentAvg = currentSum / 50;
        if (currentAvg > electrode_current *2 ) {
          depositHeights.add(hcount); // Store the height of the deposit
          Serial.print("Electrode ");
          Serial.print(active_anode[i]);
          Serial.print(" Count: ");
          Serial.print(hcount);
          Serial.print("Current Avg: ");
          Serial.print(currentAvg);
          Serial.print(" Height: ");
          Serial.println(height+ieg-hcount-60*8);
          stopDescent = true;
          for (int i = 0; i < hcount; i++) {//remonte
          myStepper.setSpeed(400);
          myStepper.step(+4);
                }
          hcount=0;
          }
          else if (hcount > ieg*8) {
            stopDescent = false;
            Serial.println("Not found, trying again");
            for (int i = 0; i < hcount; i++) {//remonte 
              myStepper.setSpeed(400);
              myStepper.step(+4);
            }
            Shake();
            shake+=1;
            hcount=0;
            if (shake>=2){stopDescent = true;shake=0;}
          
          }
          else {hcount += 1;
            myStepper.setSpeed(800);//descend a step
            myStepper.step(-4);}
          }
         pinMode(active_anode[i], INPUT_PULLUP);
     
        
    if (Serial.available()) {
      String message1 = Serial.readStringUntil('\n');
      if (message1.equals("stop")) {
        deactivateAllElectrodes();
        break;
      }
    }
    
    }
    // Go back to working distance
    for (int i = 0; i < 60*8; i++) {//remonte en haut
              myStepper.setSpeed(800);
              myStepper.step(+4);
              
            }
  Serial.println("Descent stopped");
  UpdateMatrix();
  return depositHeights;
  delay(1000);
}
      


// Function to send an image via serial and activate corresponding electrodes
void UpdateMatrix() {
  targetCurrent = 0.0;
  n_electrode=0;
  active_anode.clear();  

  deactivateAllElectrodes();
  pinMode(PinCathode, OUTPUT);
  // Activate corresponding electrodes based on the received data ex:
  //String message="";// le milieu fonctionne pas
  if (matrix==""){
  Serial.println("Please send matrix");
  
  while (!Serial.available()) {
  delay(1); 
  }   

  matrix = (Serial.readStringUntil('\n'));
  }
  for (int port = 14; port <= 53; port++) {
    if (port != 20 && port != 21) {//SDA SCL port
    if (matrix[port-14]=='0'){
      pinMode(port, INPUT_PULLUP);
    }
    else if (matrix[port-14]=='1') {
      pinMode(port, OUTPUT);
      digitalWrite(port, HIGH);
      active_anode.add(port);
      n_electrode+=1;
      //save active electrode list, then mask out the shorting electrode to continue depositing what missing int the layer.
    }
    /*else if (port==23) {//no need now
      pinMode(port, OUTPUT);
      digitalWrite(port, HIGH);
      n_electrode+=1;
    }*/
    else {
      digitalWrite(port, HIGH);
      pinMode(port, INPUT_PULLUP);
    }
    
    }
    }
    
    for(int i=0; i<depositHeights.getSize();i++){
      if (depositHeights[i]>=height){
      pinMode(active_anode[i], INPUT_PULLUP);
      depositHeights.remove(i);
      n_electrode-=1;
      Serial.print(active_anode[i]);
      }
    }
    
    Serial.println();
    //return n_electrode;//set target current
    targetCurrent = electrode_current*n_electrode;
    
    Serial.println(n_electrode);
    pinMode(PinCathode, OUTPUT);
    delay(500);
    
    return targetCurrent;
}


///////////////////////////////////////////////

// Function to deactivate all electrodes
void deactivateAllElectrodes() {
  // Turn off all electrode array pins
  for (int pin = ElectrodeStartPin; pin <= ElectrodeEndPin; ++pin) {
    digitalWrite(pin,HIGH);
    pinMode(pin, INPUT_PULLUP);
  }
  digitalWrite(PinCathode,HIGH);
  pinMode(PinCathode, INPUT_PULLUP);
  digitalWrite(PinFan,LOW);


}
void(* reset) (void) = 0; //declare reset function @ address 0
void Shake(){
  deactivateAllElectrodes();
  myStepper.setSpeed(3000);
     for (int i = 0; i < 6000; i++) {//printing 10µm with 1 layers          
          myStepper.step(+4);  
      }
      myStepper.setSpeed(80);
      for (int i = 0; i < 16; i++) {//printing 10µm with 1 layers          
      
          myStepper.step(+4);
        
          myStepper.step(-4);

      }
      myStepper.setSpeed(3000);
      for (int i = 0; i < 6000; i++) {//printing 10µm with 1 layers          
          myStepper.step(-4);   
      }
  myStepper.setSpeed(80);
   for (int i = 0; i < 16; i++) {//printing 10µm with 1 layers          
      
          myStepper.step(+4);
        
          myStepper.step(-4);

      }
      myStepper.setSpeed(160);
         for (int i = 0; i < 40; i++) {//printing 10µm with 1 layers          
      
          myStepper.step(+4);
        
          myStepper.step(-4);

      }myStepper.setSpeed(800);
       for (int i = 0; i < 200; i++) {//printing 10µm with 1 layers          
      
          myStepper.step(+4);
        
          myStepper.step(-4);

      }
      UpdateMatrix(); 
      delay(1000);
      }    
///////////////////////////////////////////////
void PrintLayers(){
  UpdateMatrix();// One layer test first
  Serial.println(targetCurrent);
  for (int i = 0; i < ieg*8; i++) {//height 800µm
          myStepper.setSpeed(1000);
          myStepper.step(+4);
        }
        Thickness=0;pwmValue=0;currentAvg=0;
  int timer=0;
  float Kp = 1/electrode_current/(targetCurrent*0.5); // proportional gain
  float Ki = 1.3/electrode_current/(targetCurrent*0.5); // integral gain
  float Kd = 0.4/electrode_current/(targetCurrent*0.5); // derivative gain
  
  delay(1000);
  while(1) {
  float currentSum = 0.0;
  timer+=2;
  delay(400);
  if (Serial.available() || Thickness >=10000) {
        String message1;
        message1 = Serial.readStringUntil('\n');
        if (message1.equals("stop")|| Thickness >=10000){
          Serial.println("Stopped");
          deactivateAllElectrodes();targetCurrent=0;integral=0;derivative=0;Thickness=0;pwmValue=0;currentAvg=0;matrix="";
        
      delay(1000);
      break;
        }
        else if(message1.equals("ehome")){ehome();}
        
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
    pwmValue = 220 - constrain(pwmValue,0,235);
    if (abs(derivative) < 20){
    if (currentAvg>0){Thickness+=currentAvg*2/(28.74*targetCurrent);}//mA 2s 0.5mm,0.65mm, 28°, 2electrodes
  }
  else{
  Serial.println("Pic de courant");shake+=1;
  }
  if (shake==2){//contact
  Shake();
  Serial.println("shake");
  shake=0;
        }// Montée de 20µm
  if (shake>2){//contact
  ehome();
  Serial.println("ehome");
  shake=0;
        }
    analogWrite(PinCathode, pwmValue);
    Serial.print("Courant(mA):");Serial.print(currentAvg);
    Serial.print(",");
    Serial.print("Potentiel(V):");Serial.print(pwmValue/256*5);
    Serial.print(",");
    //Serial.print("Derivee(mA):");Serial.print(derivative);
    //Serial.print(",");
    Serial.print("Epaisseur(µm):");Serial.println(Thickness);
    prevError = error;
    delay(333);//time correction
    if (Thickness>Target) {
      for (int i = 0; i < 1; i++) {
      myStepper.setSpeed(1000);
      myStepper.step(+4);//1.25µm
      UpdateMatrix();
      height+=1;
      }
      Target+=lheight;//1.25µm will80 be smaller for later deposits will get some error estimating height after changed
      //lheight ++ = monte plus lentement//lheight -- = monte plus rapidement
  }
 
  }
  else{deactivateAllElectrodes();Thickness=0;pwmValue=0;currentAvg=0;delay(1000);}
  if (timer>=50 &&Thickness>=1){Shake();
  timer=0;shake=0;}

  if (Thickness>eh){
    eh+=200;
  UpdateMatrix();
  ehome();
  Serial.print("bravo");
  delay(1000);   
  }

  /*if (Thickness>100 && lheight==0.25){//TWICE

    lheight=5;
    electrode_current += 0.25;//new growing current
    UpdateMatrix();
    for (int i = 0; i <2*8; i++) {//start height 2X250µm+200µm
          myStepper.setSpeed(1000);
          myStepper.step(+4);
        }
        
  }
  if (Thickness>100 && electrode_current==1){//Reset height once
    lheight=6;
    electrode_current = 2;//new growing current
    UpdateMatrix();
    for (int i = 0; i <20*8; i++) {//start height 2X250µm+200µm+200µm
          myStepper.setSpeed(1000);
          myStepper.step(+4);
        }
        
  }*/
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
    if (message1.equals("ehome")) {
      Serial.println("ehoming");
      ehome();
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
      for (int i = -32000; i < 32000; i++) {
        myStepper.setSpeed(8000);
        myStepper.step(+4);
        
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
     
      else if (message1.equals("down")) {
      for (int i = -32000; i < 32000; i++) {
        myStepper.setSpeed(8000);
        myStepper.step(-4);
        
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
          for (int i = -32000; i < 32000; i++) {
          myStepper.setSpeed(8000);
          myStepper.step(+4);
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