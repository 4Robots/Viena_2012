/*
 * Written by Konstantin Kostyuk et al., Sep 05, 2010.
 * Copyright (c) 2010 4Robots. For more information, see
 * Ver: 0.1 01.01.2011
 *
 *   http://www.4Robots.ru
 *   http://www.Ardubot.ru
 *   http://www.roboforum.ru
 *   http://www.open-robotics.ru
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the 4 links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 */

//-- Old code include, need refactoring
//#include <OR128Servo.h> 
//#include <PCRobot.h>
//#include <ORDMotors.h>
//#include <I2C_RGBC_Reader.h>
#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include <IRremote.h>


//Main state machine state list
#define MAINSTATE_CONFIGURATION  0;
#define MAINSTATE_WAIT_START_PRESS 1;
#define MAINSTATE_WAIT_START_RELEASE 2;
#define MAINSTATE_COLLECT_PUCKS 3;
#define MAINSTATE_SEARCH_BASE 4;
#define MAINSTATE_PARK_NEAR_BASE 5;
#define MAINSTATE_RELEASE_PUCKS 6;
#define MAINSTATE_WAIT_NEAR_BASE 7;

//Pin configuration
//int startBtn=16; //Start button on 16 digital port (PortA0), between GND and SIG pins
int RECV_PIN = 10; //IR Remote reciver pin

//Time constants for algorithm
long int roundLenght=180*1000; //Round length in 1msec units, i.e. 180*1000 => 180sec => 3min
long int collectTime=120*1000; //Time to collect pucks in 1msec units, after that we must park near our base
long int releaseTime=170*1000; //Time to release enemy pucks in 1msec units, after that we must park to base and release our pucks
long int StartTK; //Start Time in sec Unix style
char buffer[16]; // For LCD 16x2 output

//Debug state, if set - State machine is simplified to MAINSTATE_WAIT_START_PRESS => MAINSTATE_WAIT_START_RELEASE => DEBUGSTATE => EN
int debugState=0; //int debugState=MAINSTATE_CONFIGURATION

int mainState,processingState,processedState;

// Objects
//Mega   RS, E, DB4, DB5, DB6, DB7
LiquidCrystal lcd(32, 30, 28, 26, 24, 22);
//RTC
RTC_DS1307 RTC;
//IR Remote
IRrecv irrecv(RECV_PIN);
decode_results results;

void setup()  
{
  //Setup All Pins
  //pinMode(startBtn,INPUT);
  //digitalWrite(startBtn,HIGH);
  //  pinMode(LED, OUTPUT);  
 
  Serial.begin(115200);// Start the UART
  lcd.begin(16, 2);    // Start the LCD
  irrecv.enableIRIn(); // Start the IR receiver

  //Setup main state machine
  mainState=MAINSTATE_CONFIGURATION;
  processedState=mainState;
  processingState=mainState;
  
  //RTC init
  Wire.begin();
  RTC.begin();  
  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    lcd.print("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  else {
    Serial.println("RTC is running!");
    lcd.print("RTC is running!");
  };
  delay(1000);
  lcd.clear();   
}

//int eps=30;
//int r, g, b, c; 
//int _r, _g, _b, _c;

void loop() //16Hz speed main cycle;
{
  DateTime now = RTC.now();  
  sprintf(buffer,"%02d.%02d.%02d",now.day(),now.month(),now.year());
  lcd.setCursor(0,0);
  lcd.print(buffer); 
  
  sprintf(buffer,"%02d:%02d:%02d",now.hour(),now.minute(),now.second());              
  lcd.setCursor(0,1);
  lcd.print(buffer);
  
  processingState=mainState;

  switch (mainState) {
   case 0: //MAINSTATE_CONFIGURATION - config and calibrate equipment
     Serial.println("CONFIGURATION");
    //if(Bot.startPressed()) mainState=MAINSTATE_WAIT_START_RELEASE;
    break;   
  case 1: //WAIT_START_PRESS
     Serial.println("WAIT_START_PRESS");
     //if(Bot.startPressed()) mainState=MAINSTATE_WAIT_START_RELEASE;
    break;
  case 2: //WAIT_START_RELEASE
     Serial.println("WAIT_START_RELEASE");
    //if(!Bot.startPressed()){
    //  Bot.setStartTime();//remember time of start
    //  mainState=MAINSTATE_COLLECT_PUCKS;
    //};
    break;
  case 3: //COLLECT_PUCKS
     Serial.println("COLLECT_PUCKS");
    //if(processedState!=mainState)
    //  collectPucksInit();
    //collectPucksIteration();
    //if(Bot.getTaimTime()>=collectTime) mainState=MAINSTATE_SEARCH_BASE;
    //if(Bot.getTaimTime()/10>=releaseTime) mainState = MAINSTATE_RELEASE_PUCKS;
    //Bot.L_RGBC.getRGBC(_r, _g, _b, _c);
    
    //if( (Bot.getTaimTime()>=collectTime) && (abs(g-_g)<eps) && (abs(b-_b)<eps) && (abs(c-_c)<eps) ) mainState = MAINSTATE_RELEASE_PUCKS;
     break;
  case 4: //SEARCH_BASE
     Serial.println("SEARCH_BASE");
    //if(processedState!=mainState)
    //  searchBaseInit();
    //searchBaseIteration();
    break;
  case 5: //PARK_NEAR_BASE
     Serial.println("MAINSTATE_CONFIGURATION");
    //if(processedState!=mainState)
    //  parkNearBaseInit();
    //parkNearBaseIteration();
    break;
  case 6: //RELEASE_PUCKS
     Serial.println("PARK_NEAR_BASE");
    //if(Bot.getTaimTime()>=releaseTime) mainState=MAINSTATE_RELEASE_PUCKS;
    break;
  case 7: //WAIT_NEAR_BASE
     Serial.println("WAIT_NEAR_BASE");
    //if(processedState!=mainState)
    //  releasePucksInit();
    //releasePucksIteration();
    break;
  };

  processedState=processingState;

  //Debug state check and process if need
  if(debugState!=0){
    if(mainState>1){
      if(processingState!=debugState)
        mainState=debugState;
      else
        mainState=MAINSTATE_WAIT_START_PRESS;
    }
  };

  delay(10);
}


void collectPucksInit()
{
  /*
    Bot.L_RGBC.calibrate(); delay(100);
    Bot.L_RGBC.offset(); delay(100);
    Bot.L_RGBC.white(); delay(100);
    digitalWrite(LED_PIN, HIGH);
    Bot.L_RGBC.getRGBC(r, g, b, c);   
  */
}

void collectPucksIteration()
{
  
}

void searchBaseInit()
{
}

void searchBaseIteration()
{
}

void parkNearBaseInit()
{
}

void parkNearBaseIteration()
{
}

void releasePucksInit()
{
}

void releasePucksIteration()
{
}

