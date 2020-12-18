#include "eyes_manager.h"

Eyes_manager::Eyes_manager(){
  low_r.attach(R_LOW_SERVO_PIN);
  high_r.attach(R_HIGH_SERVO_PIN);
  low_l.attach(L_LOW_SERVO_PIN);
  high_l.attach(L_HIGH_SERVO_PIN);

  low_r.write(STANDARD_LOW_R);
  high_r.write(STANDARD_HIGH_R);
  low_l.write(STANDARD_LOW_L);
  high_l.write(STANDARD_HIGH_L);
}


void Eyes_manager::move_eye_by_degree(Servo high_servo,Servo low_servo, int high_final_position, int low_final_position){
   int low_position=low_servo.read();
  int high_position=high_servo.read();
  if(high_position>=high_final_position&&low_position>=low_final_position){
     if(high_position!=high_final_position)high_position--;
     if(low_position!=low_final_position)low_position--;
     low_servo.write(low_position);
     high_servo.write(high_position);
  }
  if(high_position>=high_final_position&&low_position<=low_final_position){ 
     if(high_position!=high_final_position)high_position--;
     if(low_position!=low_final_position)low_position++;
     low_servo.write(low_position);
     high_servo.write(high_position);
  }
  if(high_position<=high_final_position&&low_position>=low_final_position){
 
     if(high_position!=high_final_position)high_position++;
     if(low_position!=low_final_position)low_position--;
     low_servo.write(low_position);
     high_servo.write(high_position);
   
  }
  if(high_position<=high_final_position&&low_position<=low_final_position){
     if(high_position!=high_final_position)high_position++;
     if(low_position!=low_final_position)low_position++;
     low_servo.write(low_position);
     high_servo.write(high_position);
  }
}


void Eyes_manager::move_eyes_in_position(int Speed, int r_high_final_position, int r_low_final_position, int l_high_final_position, int l_low_final_position){
  int r_low_position=low_r.read();
  int r_high_position=high_r.read();
  int l_low_position=low_l.read();
  int l_high_position=high_r.read();
  while(r_high_position!=r_high_final_position||r_low_position!=r_low_final_position||l_high_position!=l_high_final_position||l_low_position!=l_low_final_position){
     move_eye_by_degree(high_r,low_r,r_high_final_position,r_low_final_position);
     move_eye_by_degree(high_l,low_l,l_high_final_position,l_low_final_position);
     r_low_position=low_r.read();
     r_high_position=high_r.read();
     l_low_position=low_l.read();
     l_high_position=high_r.read();
     delay(Speed*SPEED_MULTIPLIER);
    }
}
