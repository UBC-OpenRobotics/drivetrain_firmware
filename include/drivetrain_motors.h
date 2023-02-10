
#ifndef DRIVETRAIN_MOTORS_H
#define DRIVETRAIN_MOTORS_H

#include <Arduino.h>

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
bool volatile state;
int volatile counter, timer_counter;

//function that uses the internal RAM of the ESP32 to store the counter and timer data
void IRAM_ATTR timer_isr()
{
  if(timer_counter >= counter)
  {
    state = !state;
    timer_counter = 0;
    digitalWrite(PUL, state);
  }
  else
    timer_counter++;
}

//function to setup motors
void setup_motors(){
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

void enableMotor(){
  digitalWrite(ENA, MOTOR_ENABLE);
}

void disableMotor(){
  digitalWrite(ENA, MOTOR_DISABLE);
}

//return required counter for the isr for a 200 step motor
int speed_count_cal(int speed)
{
  float f = speed*200/60;
  return 40000/f +0.5;
}

//drive stepper motor with a certain direction and speed
void driveMotor(u_int16_t rpm, bool dir){
  digitalWrite(DIR, dir);
  counter = speed_count_cal(rpm) -1;
}

#endif
