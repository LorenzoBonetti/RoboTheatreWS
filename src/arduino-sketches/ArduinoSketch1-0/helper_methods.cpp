#include "helper_methods.h"
int move_servo_by_one_degree(Servo servo, int final_angle){
  int angle=servo.read();
  if(angle<final_angle){
    angle++;
  }else if (angle>final_angle){
    angle--;
  }
  servo.write(angle);
  if(servo.read()==angle)return 1;
  return 0;
}
