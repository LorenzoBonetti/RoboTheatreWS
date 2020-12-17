#include "body_manager.h"




Body_manager::Body_manager(){
  front_right_servo.attach(FRONT_RIGHT_SERVO_PIN);
  front_left_servo.attach(FRONT_LEFT_SERVO_PIN);
  back_servo.attach(BACK_SERVO_PIN);
  
  front_right_servo.write(FR_OPEN);//OPEN
  front_left_servo.write(FL_OPEN);//OPEN
  back_servo.write(B_OPEN);//OPEN
}

void Body_manager::move_servos(int fr_final_angle, int fl_final_angle, int b_final_angle, int Speed){
  int fr_angle,fl_angle,b_angle;
  fr_angle=front_right_servo.read();
  fl_angle=front_left_servo.read();
  b_angle=back_servo.read();
  while(fr_angle!=fr_final_angle||fl_angle!=fl_final_angle||b_angle!=b_final_angle){
    move_servo_by_one_degree(front_right_servo,fr_final_angle);
    move_servo_by_one_degree(front_left_servo,fl_final_angle);
    move_servo_by_one_degree(back_servo,b_final_angle);
    fr_angle=front_right_servo.read();
    fl_angle=front_left_servo.read();
    b_angle=back_servo.read();
    delay(Speed*SPEED_MULTIPLIER);
  }
}
void Body_manager::move_body(int movement_type, int Speed){
   switch(movement_type){
    case NORMAL:
          move_servos(FR_OPEN,FL_OPEN,B_OPEN, Speed);
          break;
    case BOW:
          move_servos(FR_CLOSE,FL_CLOSE,B_OPEN,Speed);
          break;
    case BOW_RIGHT:
          move_servos(FR_CLOSE,FL_OPEN,B_OPEN,Speed);
          break;
    case BOW_LEFT:
          move_servos(FR_OPEN,FL_CLOSE,B_OPEN,Speed);
          break;
    case BOW_BACK:
          move_servos(FR_OPEN,FL_OPEN,B_CLOSE,Speed);
          break;
}
}
