#include "eyes_manager.h"
Eyes_manager::Eyes_manager( int low_r_servo_pin,int high_r_servo_pin,int low_l_servo_pin,int high_l_servo_pin){
  this->low_r_servo_pin=low_r_servo_pin;
  this->high_r_servo_pin=high_r_servo_pin;
  this->low_l_servo_pin=low_r_servo_pin;
  this->high_l_servo_pin=high_r_servo_pin;
  
  low_r.attach(low_r_servo_pin);
  high_r.attach(high_r_servo_pin);// attaches the servo on pin 9 to the servo object
  high_r_position=high_r.read();
  low_r_position=low_r.read();
  low_l.attach(low_l_servo_pin);
  high_l.attach(high_l_servo_pin);// attaches the servo on pin 9 to the servo object
  high_l_position=high_l.read();
  low_l_position=low_l.read();
}
void Eyes_manager::move_eye_by_degree(Servo high_servo,Servo low_servo, int high_final_position, int low_final_position){
   low_position=low_servo.read();
   high_position=high_servo.read();
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
  r_low_position=r_low_servo.read();
  r_high_position=r_high_servo.read();
  l_low_position=l_low_servo.read();
  l_high_position=l_high_servo.read();
  while(r_high_position!=r_high_final_position||r_low_position!=r_low_final_position||l_high_position!=l_high_final_position||l_low_position!=l_low_final_position){
     move_eye_by_degree(r_high_servo,r_low_servo,r_high_final_position,r_low_final_position);
      move_eye_by_degree(l_high_servo,l_low_servo,l_high_final_position,l_low_final_position);
     r_low_position=r_low_servo.read();
     r_high_position=r_high_servo.read();
     l_low_position=l_low_servo.read();
     l_high_position=l_high_servo.read();
     delay(Speed*SPEED_MULTIPLIER);
    }
}

//questo pezzo non serve, poi si vedrà
/*void Eyes_manager::move_eye_in_position(int Speed,  Servo high_servo,Servo low_servo, int high_final_position, int low_final_position){
  int low_position=low_servo.read();
  int high_position=high_servo.read();
  if(high_position>=high_final_position&&low_position>=low_final_position){
    while(high_position!=high_final_position||low_position!=low_final_position){
     if(high_position!=high_final_position)high_position--;
     if(low_position!=low_final_position)low_position--;
     low_servo.write(low_position);
     high_servo.write(high_position);
     delay(Speed*SPEED_MULTIPLIER);
    }
  }
  if(high_position>high_final_position&&low_position<low_final_position){
    while(high_position!=high_final_position||low_position!=low_final_position){
     if(high_position!=high_final_position)high_position--;
     if(low_position!=low_final_position)low_position++;
     low_servo.write(low_position);
     high_servo.write(high_position);
     delay(Speed*SPEED_MULTIPLIER);
    }
  }*/
  if(high_position<high_final_position&&low_position>low_final_position){
    while(high_position!=high_final_position||low_position!=low_final_position){
     if(high_position!=high_final_position)high_position++;
     if(low_position!=low_final_position)low_position--;
     low_servo.write(low_position);
     high_servo.write(high_position);
     delay(Speed*SPEED_MULTIPLIER);
    }
  }
  if(high_position<high_final_position&&low_position<low_final_position){
    while(high_position!=high_final_position||low_position!=low_final_position){
     if(high_position!=high_final_position)high_position++;
     if(low_position!=low_final_position)low_position++;
     low_servo.write(low_position);
     high_servo.write(high_position);
     delay(Speed*SPEED_MULTIPLIER);
    }
  }
}
