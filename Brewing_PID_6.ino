#include <max6675.h>
#include <limits.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RelayPin 9    //Relay pin to control heating element

///////////////////////////////// OLED I2C display stuff

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16

/////////////////////////////// Push buttons

const int buttonApin = 2;  // Button A
const int buttonBpin = 3;  // Button B
const int buttonCpin = 4;  // Button C
const int buttonDpin = 5;  // Button D
/////////////////////////////// Potentiometer /////////////////

const int potentiometer = A0;     //Analog input pin for potentiometer

/////////////////////////////// Thermocouple pins /////////////////

const int ktcCLK = 6; //Clock input (SCK)
const int ktcCS = 7; //Chip Select (CS)
const int ktcSO = 8; //Signal out (SO)

MAX6675 ktc(ktcCLK, ktcCS, ktcSO); //Create DHT sensor instance.

double temp = 0;

//////////////////////////////// Timing parameters ///////////////////////////

const int BUTTON_INTERVAL = 50; // ms
const int iteration_time = 50; //Delay time at the end of each loop
unsigned long counterB = 0;

//////////////////////////////// PID  ///////////////////////////
int KP=30;     //Proportional
int KI=0.2;     //Integral
int KD=1;    //Derivative

double Setpoint, Input, Output;   // Parameters for PID control
PID myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT);

String mode = "Auto";

int OutputLimit = 50;    // Max value of Output. Above this value, the PID output will be clamped to the max.
static const unsigned long PID_INTERVAL = 1000; // Max duty cycle in millis
int ONtime = 0;      // Time in millis during which the relay will be ON


void setup()
{
  
  Serial.begin(9600);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  pinMode(RelayPin, OUTPUT);

  //initialize the variables we're linked to
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  //tell the PID to range between 0 and the output limit
  myPID.SetOutputLimits(0, OutputLimit);

  //turn the buttons on as pullup
  pinMode(buttonApin, INPUT_PULLUP);  
  pinMode(buttonBpin, INPUT_PULLUP);  
  pinMode(buttonCpin, INPUT_PULLUP);
  pinMode(buttonDpin, INPUT_PULLUP);  
}

void drawPercentbar(int x,int y, int width,int height, int progress){

   progress = progress > 100 ? 100 : progress;
   progress = progress < 0 ? 0 :progress;

   float bar = ((float)(width-4) / 100) * progress; 
  
   display.drawRect(x, y, width, height, WHITE);
   display.fillRect(x+2, y+2, bar , height-4, WHITE);

  // Display progress text
  if( height >= 15){
    display.setCursor((width/2) -3, y+5 );
    display.setTextSize(1);
    display.setTextColor(WHITE);
   if( progress >=50)
     display.setTextColor(BLACK, WHITE); // 'inverted' text

     display.print(progress);
     display.print("%");
  }
}

void printscreen () {
  display.clearDisplay();
  
  //Prints temperature and setpoint
  display.setCursor(0,0); 
  display.print("Temperature: ");
  display.print(temp);
  display.print((char)247);
  display.println("C");

  display.print("Setpoint: ");
  display.print(Setpoint);
  display.print((char)247);
  display.println("C");

  display.print("Mode: ");
  display.println(mode);
  
  display.print("Output: ");
  display.print(Output*100/OutputLimit);
  display.println(" %");
  
  //drawPercentbar(0, 40, 128, 15, Output*100/OutputLimit);
  display.display();
}

void checkTemp()
{
  static const unsigned long REFRESH_INTERVAL = 600; // ms
  static const unsigned long THERMO_INTERVAL = 200; // ms
  static unsigned long lastRefreshTime = 0;
  static unsigned long thermoRefreshTime = 0;
  static unsigned long temp_readings = 0;
  static unsigned long last_temp = 0;
  static unsigned long avg_temp = 0;
  
  //if (millis() - thermoRefreshTime >= THERMO_INTERVAL){
    //thermoRefreshTime += THERMO_INTERVAL;
    //last_temp = ktc.readCelsius();  //Running average
    //avg_temp += last_temp;
    //temp_readings++;
  //}
  if(millis() - lastRefreshTime >= REFRESH_INTERVAL) //Save temperature value every REFRESH_INTERVAL time
  {
    lastRefreshTime += REFRESH_INTERVAL;
    temp = ktc.readCelsius();
    //temp = avg_temp/temp_readings;
    //temp_readings = 0;
    //avg_temp = 0;
  } 
}



void checkButtonA() {
  static const unsigned long Refresh1ButtonA = 300; // ms
  static const unsigned long Speed1ButtonA = 300; // ms
  static const unsigned long Refresh2ButtonA = 3000; // ms
  static const unsigned long Speed2ButtonA = 100; // ms
  static unsigned long lastRefresh_A = 0;
  static unsigned long timer_A = 0; 
  static unsigned long counterA = 0;
  
  //Read if button A is pressed 
  if (digitalRead(buttonApin) == LOW && digitalRead(buttonBpin) == HIGH)  {

    if (counterA == 0) {
      incrementSP();
      counterA = 1; //Store present time in counter
      lastRefresh_A = millis();
      timer_A = millis();
    } else {
      
      if (millis()-lastRefresh_A >= Refresh1ButtonA && millis()-timer_A < Refresh2ButtonA) {  //If button is pressed longer than Refresh time 1, increase SP
         incrementSP();
         lastRefresh_A += Speed1ButtonA;
        }
 
      else if (millis()-timer_A >= Refresh2ButtonA) {
        
        if (millis()-lastRefresh_A >= Speed2ButtonA) {
          incrementSP();
          lastRefresh_A += Speed2ButtonA;
        }
      }
    }
  }
  if (digitalRead(buttonApin) == HIGH)  counterA = 0;
}


void checkButtonB()
{
  static const unsigned long Refresh1ButtonB = 300;       // ms
  static const unsigned long Speed1ButtonB = 300;         // ms
  static const unsigned long Refresh2ButtonB = 3000;      // ms
  static const unsigned long Speed2ButtonB = 100;         // ms
  static unsigned long lastRefresh_B = 0;
  static unsigned long timer_B = 0; 
  static unsigned long counterB = 0;
  
  //Read if button A is pressed 
  if (digitalRead(buttonBpin) == LOW && digitalRead(buttonApin) == HIGH)  {

    if (counterB == 0) {
      decrementSP();
      counterB = 1; 
      lastRefresh_B = millis();             //Store present time
      timer_B = millis();                   //Store present time
    } else {
      
      if (millis()-lastRefresh_B >= Refresh1ButtonB && millis()-timer_B < Refresh2ButtonB) {  //If button is pressed longer than Refresh time 1, increase SP
         decrementSP();
         lastRefresh_B += Speed1ButtonB;
        }
 
      else if (millis()-timer_B >= Refresh2ButtonB) {
        
        if (millis()-lastRefresh_B >= Speed2ButtonB) {
          decrementSP();
          lastRefresh_B += Speed2ButtonB;
        }
      }
    }
  }
  if (digitalRead(buttonBpin) == HIGH)  counterB = 0;
}

void checkButtonsCD () {
  if (digitalRead(buttonCpin) == LOW && digitalRead(buttonDpin) == HIGH) {
    mode = "PID";
    myPID.SetMode(AUTOMATIC);
  } else if (digitalRead(buttonCpin) == LOW && digitalRead(buttonDpin) == LOW) {
    mode = "Manual";
    myPID.SetMode(MANUAL);
    Output = float(analogRead(potentiometer))*OutputLimit/1023;
  } else if (digitalRead(buttonCpin) == HIGH) {
    mode = "Off";
    myPID.SetMode(MANUAL);
    Output = 0;
  }
}


void incrementSP() {
  Setpoint++;
  if (Setpoint >= 100) {
    Setpoint = 0;
  }
}

void decrementSP() {
  Setpoint--;
  if (Setpoint < 0) {
    Setpoint = 100;
  }
}


///////////////Refresh PID interval and turn heater ON for a duration

void checkPID()
{
  static unsigned long lastPIDRefreshTime = 0;
  if(millis() - lastPIDRefreshTime >= PID_INTERVAL)
  {
    lastPIDRefreshTime += PID_INTERVAL;
    myPID.Compute();
    ONtime = (PID_INTERVAL/OutputLimit)*Output;
  } else {
    
    if(millis() - lastPIDRefreshTime < ONtime) {
      digitalWrite(RelayPin, HIGH);
    } else {
      digitalWrite(RelayPin, LOW);
    }
  }
}



void loop()
{
  //Reads temperature from sensor every REFRESH_INTERVAL
  checkTemp();
  
  //Updates LCD
  printscreen();
  

  //check which buttons are pressed
  
  checkButtonA();
  checkButtonB();
  checkButtonsCD();

//PID
  Input = temp;
  //myPID.Compute();  
  //analogWrite(RelayPin,Output);
  
  checkPID();

  //delay(iteration_time);
}
