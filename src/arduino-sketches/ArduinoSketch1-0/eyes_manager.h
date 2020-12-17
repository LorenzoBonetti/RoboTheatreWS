#ifndef eyes_manager_h
#define eyes_manager_h

#include <Servo.h>
#include "Arduino.h"
#include "defines.h"
class Eyes_manager{
  private:
    Servo low_r;
    Servo high_r;
    Servo low_l;
    Servo high_l;

    void move_eye_by_degree(Servo,Servo, int, int);
  public:
    Eyes_manager( int low_r_servo_pin,int high_r_servo_pin,int low_l_servo_pin,int high_l_servo_pin);
    void move_eyes_in_position(int Speed, int r_high_final_position, int r_low_final_position, int l_high_final_position, int l_low_final_position);

};

#endif
