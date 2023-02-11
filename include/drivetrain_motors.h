
#ifndef DRIVETRAIN_MOTORS_H
#define DRIVETRAIN_MOTORS_H

#include <Arduino.h>

// STEPPER MOTOR DRIVER PIN DEFINITION
#define DIR_R 19  //  RIGHT MOTOR DIRECTION
#define PUL_R 18  //  RIGHT MOTOR PULSE
#define ENA_R 5   //  RIGHT MOTOR ENABLE

#define DIR_L 33  //  LEFT MOTOR DIRECTION
#define PUL_L 25  //  LEFT MOTOR PULSE
#define ENA_L 26  //  LEFT MOTOR ENABLE

#define LEFT_MOTOR  0
#define RIGHT_MOTOR 1

//CONSTANT
#define COUNT           50
#define PRESCALAR       20    //  TIMER PRESCALER
#define MOTOR_ENABLE    0  
#define MOTOR_DISABLE   1
#define STEP_PER_REV    200
#define SEC_PER_MIN     60
#define TIMER_ISR_FREQ  4000

// GLOBAL ASSIGNMENT
bool volatile state_left, state_right;
int volatile counter_left, timer_counter_left;
int volatile counter_right, timer_counter_right;

//function that uses the internal RAM of the ESP32 to store the counter and timer data
void IRAM_ATTR timer_isr()
{
  if(timer_counter_left >= counter_left || timer_counter_right >= counter_right )
  {
    if(timer_counter_left >= counter_left)
    {
      state_left = !state_left;
      timer_counter_left = 0;
      digitalWrite(PUL_L, state_left);
    }
    else
    {
      timer_counter_left ++;
      if(timer_counter_right >= counter_right)
      {
        state_right = !state_tight;
        timer_counter_right = 0;
        digitalWrite(PUL_R, state_right);
        
      else timer_counter_right ++;
      }
    }
  }
  else
  {
      timer_counter_left ++;
      timer_counter_right ++;
  }
}

//function to setup motors
void setup_motors()
{

  //PIN SETUP
  pinMode(DIR_R, OUTPUT);
  pinMode(PUL_R, OUTPUT);
  pinMode(ENA_R, OUTPUT);

  pinMode(DIR_L, OUTPUT);
  pinMode(PUL_L, OUTPUT);
  pinMode(ENA_L, OUTPUT);


  state_left  = 0;
  state_right = 0;
  counter_left  = 0;
  counter_right = 0;
  timer_counter_left  = 0; 
  timer_counter_right = 0;
  
  // Timer ISR
  hw_timer_t* timer = timerBegin(0, PRESCALAR, true);
  timerAttachInterrupt(timer, &timer_isr, true);
  timerAlarmWrite(timer, COUNT, true);
  timerAlarmEnable(timer);
}

void enableMotor()
{
  digitalWrite(ENA_R, MOTOR_ENABLE);
  digitalWrite(ENA_L, MOTOR_ENABLE);
}

void disableMotor()
{
  digitalWrite(ENA_R, MOTOR_DISABLE);
  digitalWrite(ENA_L, MOTOR_DISABLE);
}

//return required counter for the isr for a 200 step motor
int speed_count_cal(int speed)
{
  float f = speed*STEP_PER_REV/SEC_PER_MIN;
  return TIMER_ISR_FREQ/f +0.5;   
}

//drive stepper motor with a certain direction and speed
void driveMotor(u_int16_t* rpm, bool* dir)
{
  digitalWrite(ENA_R, dir[0]);
  digitalWrite(ENA_L, dir[1]);
  counter_left  = speed_count_cal(rpm[1]);
  counter_right = speed_count_cal(rpm[0]);

}

#endif
