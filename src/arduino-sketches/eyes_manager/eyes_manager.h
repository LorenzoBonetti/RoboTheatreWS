#ifndef eyes_manager.h
#define eyes_manager.h

#include <Servo.h>
#include "Arduino.h"
#define R_LOW_SERVO_PIN 6
#define R_HIGH_SERVO_PIN 7
#define L_LOW_SERVO_PIN 9
#define L_HIGH_SERVO_PIN 10

#define SPEED_MULTIPLIER 10

#define STANDARD_LOW_R 90
#define STANDARD_HIGH_R 65
#define LOOK_RIGH_HIGH_R 10
#define LOOK_RIGH_LOW_R 100
#define LOOK_LEFT_HIGH_R 120
#define LOOK_LEFT_LOW_R 150
#define LOOK_DOWN_LOW_R 180
#define LOOK_DOWN_HIGH_R 30
#define LOOK_UP_LOW_R 125
#define LOOK_UP_HIGH_R 90

#define STANDARD_LOW_L 80
#define STANDARD_HIGH_L 45
#define LOOK_RIGH_HIGH_L 90
#define LOOK_RIGH_LOW_L  130
#define LOOK_LEFT_HIGH_L 30
#define LOOK_LEFT_LOW_L 30
#define LOOK_DOWN_LOW_L 130
#define LOOK_DOWN_HIGH_L 0
#define LOOK_UP_LOW_L 65
#define LOOK_UP_HIGH_L 60

class Eyes_manager{
  private:
    int high_r_position;
    int low_r_position;
    int high_l_position;
    int low_l_position;
    int low_r_servo_pin;
    int high_r_servo_pin;
    int low_l_servo_pin;
    int high_l_servo_pin;
    Servo low_r;
    Servo high_r;
    Servo low_l;
    Servo high_l;

    void move_eye_by_degree(Servo,Servo, int, int);
  public:
    Eyes_manager( int low_r_servo_pin,int high_r_servo_pin,int low_l_servo_pin,int high_l_servo_pin);
    void move_eyes_in_position(int Speed, int r_high_final_position, int r_low_final_position, int l_high_final_position, int l_low_final_position){

}
#endif
