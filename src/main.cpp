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

#define sUp 1
#define sDown 2
#define sFinished 3
#define sMeasure 4
#define sReady 5
#define sAvg 200


//Define Variables we'll be connecting to
double newSetpoint;
double tnewSetpoint;
unsigned long setTime;

unsigned int tInput, tSetpoint, tOutput, tSens, tSet, tError;

int currentSetPressure;
bool switchLockState;
int sensVal;
int sStatus = 0;


void setup() {
   Serial.begin (9600);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
   Serial.println("SSD1306 allocation failed");
   for(;;); // Don't proceed, loop forever
  }

  delay(2000);
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
  
  pinMode (sensPin, INPUT);
  pinMode (setPressure, INPUT);
  pinMode (switchLock, INPUT_PULLUP);
  pinMode (upAirPin, OUTPUT);
  pinMode (downAirPin, OUTPUT);
  pinMode (buzzerPin, OUTPUT);

  //initialize the variables we're linked to
  currentSetPressure = analogRead(setPressure);
  tSetpoint = map(currentSetPressure, 0, 1023, 0, 50);
  tnewSetpoint = tSetpoint;

  sensVal = analogRead(sensPin);
  tInput = 0;

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
  if (up) sStatus =  sUp;
  if (down) sStatus = sDown;
  if (!up && !down && (!switchLockState && (abs(tInput-tSetpoint)<=1))) sStatus = sFinished;
  if (!up && !down && !switchLockState && (abs(tInput-tSetpoint)>1)) sStatus = sMeasure;
  if (!up && !down && switchLockState) sStatus = sReady;
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
  int tTemp = 0;
  for (int i=0; i < sAvg; i++){
    sensVal = analogRead(sensPin);
    tTemp += sensVal;
  }
  int finalTemp = tTemp/sAvg;
  return finalTemp;
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
        display.print((unsigned int)tInput);
        display.display();
  }

void displayStatus(){
  display.clearDisplay();
  switch (sStatus){
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {

  
  switchLockState = digitalRead(switchLock);
  if (switchLockState) currentSetPressure = analogRead(setPressure);
  tSetpoint = map(currentSetPressure, 0, 1023, 0, 50);
  tInput = PSI(avgMeasure());

 
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
  
  if (sStatus == sMeasure) delay (1500);
  

// debug();

  delay(1);
 }
 


