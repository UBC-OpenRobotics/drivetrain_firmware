#include <Arduino.h>
#include <functions.h>

// PIN DEFINITION
#define LED 2
#define DIR 18  
#define PUL 19 //pulse
#define SW  15
#define ENA 21

//CONSTANT
#define PRESCALAR 20
#define COUNT 50
#define MOTOR_ENABLE 0
#define MOTOR_DISABLE 1

// GLOBAL ASSIGNMENT
bool  volatile state;
int   volatile counter, timer_counter;


void setup() 
{
  Serial.begin(115200);
  
  //PIN SETUP
  pinMode(LED, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(SW,  INPUT_PULLUP);
  digitalWrite(ENA,MOTOR_DISABLE);
  state =0;
  counter =50;
  timer_counter =0;
  
  // Timer ISR
  hw_timer_t* timer = timerBegin(0, PRESCALAR, true);
  timerAttachInterrupt(timer, &timer_isr, true);
  timerAlarmWrite(timer, COUNT, true);
  timerAlarmEnable(timer);
}
 