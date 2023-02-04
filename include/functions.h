
//function that uses the internal RAM of the ESP32 to store the counter and timer data
void IRAM_ATTR timer_isr()
{

   if(timer_counter >= counter)
   {
        state = !state;
        timer_counter = 0;
        digitalWrite(PUL, state);
   }
   else timer_counter ++;

}

//initating the motor if commanded with a specific speed and direction
void loop()
{

    if(Serial.available())
    {
      String cmd = Serial.readStringUntil('\n');
      if(cmd =="enable" || cmd =="disable")
      {
        if(cmd =="disable") digitalWrite(ENA,MOTOR_DISABLE);
        else digitalWrite(ENA,MOTOR_ENABLE);
      }
      else
      {
        int speed = cmd.toInt();
        digitalWrite(DIR, (speed>0));
        counter = speed_count_cal(abs(speed)) -1 ;
      }

    }
  }


//return required counter for the isr for a 200 step motor
int speed_count_cal(int speed)
   {
       float f = speed*200/60;
       return 40000/f +0.5;
   }
