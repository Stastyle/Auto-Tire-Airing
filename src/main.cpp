#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h> 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


#define upAirPin 16      //  Inlet solenoid (add air)
#define downAirPin 10    //  outlet solenoid (remove air)
#define sensPin A0       //  Pressure sensor 
#define setPressure A1   // Potentiometer to set pressure
#define sensHigh 4       // Set 5v for pressure sensor
#define potHigh 8        // Set 5v for potentiometer
#define potLow 6         // Set 0v for potentiometer
#define switchLock 7     // Switch for airing
#define buzzerPin 17
#define RxP 14           // Set Rx Pin for Bluetooth module
#define TxP 15           // Set Tx Pin for Bluetooth module
#define btStatePin 5    // HC-05 State

#define sUp 1
#define sDown 2
#define sFinished 3
#define sMeasure 4
#define sReady 5
#define sAvg 200     // Number of iteration for avarage function
#define sBT 6


unsigned long setTime, tempTime2 = 0, tempTime3 = 0;

int  tInput = 0, tSetpoint, tSetpointTemp;
unsigned int tError;

int currentSetPressure, tActivateTemp;
bool switchLockState, btState = 0, tActivate=0;
int sensVal;
int sStatus = 0;

//SoftwareSerial BT(RxP,TxP);

void setup() {
  Serial.begin (9600);
  Serial1.begin (9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
   Serial.println("SSD1306 allocation failed");
   for(;;); // Don't proceed, loop forever
  }

  delay(500);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.println("Auto Tire Airing"); 
  display.println("Initialting...."); 
  display.println("Please wait"); 
  display.display();
  delay(2000);


  pinMode (sensHigh, OUTPUT);
  pinMode (potHigh, OUTPUT);
  pinMode (potLow, OUTPUT);
  digitalWrite (sensHigh, 1);
  digitalWrite (potHigh, 1);
  digitalWrite (potLow, 0);
  
  pinMode (btStatePin, INPUT);
  pinMode (sensPin, INPUT);
  pinMode (setPressure, INPUT);
  pinMode (switchLock, INPUT);
  pinMode (upAirPin, OUTPUT);
  pinMode (downAirPin, OUTPUT);
  pinMode (buzzerPin, OUTPUT);
  
  

  //initialize the variables we're linked to
  currentSetPressure = analogRead(setPressure);
  tSetpoint = map(currentSetPressure, 0, 1023, 0, 50);

  sensVal = analogRead(sensPin);
  switchLockState = digitalRead(switchLock);

  while (!switchLockState) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.println("Release"); 
  display.println(" the "); 
  display.println("switch!"); 
  display.display();
  switchLockState = digitalRead(switchLock);
  }
}


//////////////////////////////////////////       Functions          //////////////////////////////////////////

void beep (int x){
  if ((millis()/1000)%5){
    for (int i=1; i<=x; i++) {
      int y = i%2 ;
      digitalWrite(buzzerPin,y);
      delay(100);
    }
    digitalWrite(buzzerPin,0);
  }
}

void relay(bool up, bool down){
  digitalWrite (upAirPin, up);
  digitalWrite (downAirPin, down);
  if (btState) sStatus = sBT;
  else {
    if (up) sStatus =  sUp;
    if (down) sStatus = sDown;
    if (!up && !down && (!switchLockState && (abs(tInput-tSetpoint)<=1))) sStatus = sFinished;
    if (!up && !down && !switchLockState && (abs(tInput-tSetpoint)>1)) sStatus = sMeasure;
    if (!up && !down && switchLockState) sStatus = sReady;
  }
}

int PSI(int xp){
  /*
    from graph: Ymax = 921[bit] or 4.5[v]
                Ymin = 102[bit] or 0.5[v]
                Xmax = 174 [PSI] or 1.2[MPa]
                Xmin = 0 ;

                m = (Ymax-Ymin) / (Xmax-Xmin) = 4.706
                n = 102
                y = 4.706x + 102
  */
//  float voltage = xp*5.0/1024.0;
//  int actualPSI = (float)3.0*((float)voltage-0.475)*1.45038;
  int actualPSI = (float)(xp-102) / (float)4.706;
  return actualPSI;
}

int avgMeasure(){
  unsigned long tTemp = 0;
  for (int i=0; i < sAvg; i++){
    sensVal = analogRead(sensPin);
    tTemp += sensVal;
  }
  unsigned long finalTemp = tTemp/sAvg ;
       // int finalTemp2 = map(finalTemp, 102, 921, 0, 174)
  int finalTemp2 = PSI(finalTemp);
  return finalTemp2;
}

void debug(){
  Serial.print("Raw: ");
  Serial.println(sensVal);

  Serial.print("Average Raw: ");
  Serial.println(avgMeasure());
    
  Serial.print("Set pressure: ");
  Serial.println(tSetpoint);
  
  Serial.print("Actual PSI: ");
  Serial.println(tInput);

  Serial.print("Serial Read: ");
  Serial.println(Serial1.read());

  Serial.print("tInput: ");
  Serial.println(tInput);

  Serial.print("tActivate: ");
  Serial.println(tActivate);

  Serial.println(" ");
}

void purgeAir(){
  digitalWrite(downAirPin,HIGH);
  delay(100);
  digitalWrite(downAirPin, LOW);
}

void showPSI(){
        display.setCursor(0,9);
        display.setTextSize(2);
        display.println(" SET  TIRE");
        display.setTextSize(4);
        display.setCursor(0,32);
        display.print((unsigned int)tSetpoint);
        display.setCursor(50,32);
        display.print(" ");
        if (tInput >= 65) display.print("ER");
        if (tInput < 0) display.print("0");
        if (tInput < 65 || tInput >= 0) display.print(tInput);
        display.display();
  }

void displayStatus(){
  display.clearDisplay();
  switch (sStatus){
    case sBT:
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("Bluetooth Controlled!");
      showPSI();
      display.display();
      break;

    case sReady:
      display.setTextSize(1);
      display.setCursor(45,0);
      display.println("Ready!");
      showPSI();
      display.display();
      break;

    case sUp:
      display.setTextSize(1);
      display.setCursor(20,0);
      display.println("Working UP!");
      showPSI();
      break;

    case sDown:
       display.setTextSize(1);
       display.setCursor(20,0);
       display.println("Working DOWN!");
       showPSI();
      break;

    case sFinished:
        display.setTextSize(4);
        display.setCursor(0,20);
        display.println("DONE!");
        display.display();
        beep(6);
      break;

    case sMeasure:
        display.setTextSize(1);
        display.setCursor(30,0);
        display.println("Measuring...");
        showPSI();
      break;

  }
  
  
}

void standaloneRoutine(){
  switchLockState = digitalRead(switchLock);
  if (switchLockState) currentSetPressure = analogRead(setPressure);
  tSetpoint = map(currentSetPressure, 0, 1023, 0, 50);

  if (!switchLockState && abs(tInput-tSetpoint)>1){
        tError = abs((int)tInput-(int)tSetpoint)*1000;
        setTime = millis();
        while ((millis() - setTime < tError) && !switchLockState) {
        if (tInput < tSetpoint) {
          relay (1,0);
        }
        if (tInput > tSetpoint) {
          relay (0,1);
        }
        displayStatus();
        switchLockState = digitalRead(switchLock);
       } 
  }  
  relay(0,0);
  displayStatus(); 
}


void bluetoothRoutine(){
  if (Serial1.available()>0) tSetpoint = Serial1.read();
   
    if (tSetpoint>100) {
      tActivate=1; 
      tSetpoint = tSetpoint-100;
    }
    else tActivate=0;
  

    tError = abs(tInput-tSetpoint)*1000;
    setTime = millis();
    while ((millis() - setTime < tError) && tActivate) {
     if (tInput < tSetpoint) {
       relay (1,0);
     }
     if (tInput > tSetpoint) {
       relay (0,1);
     }
    displayStatus();
    } 

  relay(0,0);
  displayStatus();   

  if (millis()-tempTime3>200){
    Serial1.write(tInput);
    Serial1.flush();
    tempTime3 = millis();
  }

  if (Serial1.available()>0){
    for(int j=0; j<3; j++){
      Serial1.read();
    }
  } 
  
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {

  tInput = avgMeasure();
  btState = digitalRead(btStatePin);

  if (btState) bluetoothRoutine();
  if (!btState) standaloneRoutine();

  
  if (sStatus == sMeasure) delay (1500);
  
/*
if (millis() - tempTime2 > 2000){
  debug();
  tempTime2 = millis();
}
 */

  delay(1);
 }