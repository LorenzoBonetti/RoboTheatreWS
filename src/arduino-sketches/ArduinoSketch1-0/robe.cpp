/*
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <Servo.h>
#include "defines.h"
#include "helper_methods.h"

void eyes_callback(const std_msgs::Int8MultiArray&);
int move_eyes(int Speed, int r_low_final_angle, int r_high_final_angle, int l_low_final_angle, int l_high_final_angle );
void body_callback (const std_msgs::Int8MultiArray&msg);
void move_body(int fr_final_angle, int fl_final_angle, int b_final_angle, int Speed);

Servo low_r, high_r, low_l, high_l;
Servo body_right, body_left, body_back;

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int8MultiArray> body_sub("arduino/body", body_callback);
ros::Subscriber<std_msgs::Int8MultiArray> eyes_sub("arduino/eyes", eyes_callback);

unsigned long last_battery_check_time=0;
unsigned long last_battery_beep_time=0;
bool should_beep;
unsigned long last_eyes_move_time=0;
unsigned long last_body_move_time=0;



void setup()
{
  //Serial.begin(9600);
  pinMode(BAT_PROBE_LED,OUTPUT);
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  delay(1000);
  nh.subscribe(eyes_sub);
  nh.subscribe(body_sub);
  delay(1000);

  low_r.attach(R_LOW_SERVO_PIN);
  high_r.attach(R_HIGH_SERVO_PIN);
  low_l.attach(L_LOW_SERVO_PIN);
  high_l.attach(L_HIGH_SERVO_PIN);

  move_eyes(5,STANDARD_LOW_R, STANDARD_HIGH_R, STANDARD_LOW_L, STANDARD_HIGH_L);

  body_right.attach(BODY_RIGH_SERVO_PIN);
  body_left.attach(BODY_LEFT_SERVO_PIN);
  body_back.attach(BODY_BACK_SERVO_PIN);

  move_body(FR_OPEN,FL_OPEN,B_OPEN, 5);

  
}

void loop()
{
   // Publish battery state every 60 seconds!
    if (millis() - last_battery_check_time > BAT_CHK_TIMEOUT_TRH){
        last_battery_check_time = millis();
        float voltage = checkBatteryLevel();
        if (voltage != 0){
          digitalWrite(BAT_PROBE_LED, HIGH);
        }
        else{
          digitalWrite(BAT_PROBE_LED, LOW);
        }
    }
    
    nh.spinOnce();
   
   
}




// Reads voltage level at battery pin (analog). This
// voltage comes from the voltage divider circuit used to monitor the battery.
float getVoltageLevel(){
  float voltage = (analogRead(BATTERY_PIN) * 5.015) / 1024.0;
  return (LOW_BATTERY_VOLTAGE*voltage)/VOLTAGE_TRH;  // Converted voltage (approx).
}

// Checks the battery level and produces a beep in case it is low. Also writes
// the voltage level to serial port.
float checkBatteryLevel(){
  float voltage = getVoltageLevel();
  
  if (voltage <= LOW_BATTERY_VOLTAGE && voltage >= (LOW_BATTERY_VOLTAGE/2)){
    if ((millis() - last_battery_beep_time) > BEEPING_INTERVAL){
      last_battery_beep_time = millis();
      if (!should_beep) {
        should_beep = true;
        tone(BUZZER_PIN, NOTE_TO_PLAY,NOTE_DURATION);
      } else {
        should_beep = false;
        noTone(BUZZER_PIN);
      }
    }
  }else{
    digitalWrite(BUZZER_PIN, LOW);
  }
  return voltage;
}

void body_callback (const std_msgs::Int8MultiArray&msg){
   int movement_type=msg.data[0];
   int Speed=msg.data[1];
   switch(movement_type){
    case NORMAL:
          move_body(FR_OPEN,FL_OPEN,B_OPEN, Speed);
          break;
    case BOW:
          move_body(FR_CLOSE,FL_CLOSE,B_OPEN,Speed);
          break;
    case BOW_RIGHT:
          move_body(FR_CLOSE,FL_OPEN,B_OPEN,Speed);
          break;
    case BOW_LEFT:
          move_body(FR_OPEN,FL_CLOSE,B_OPEN,Speed);
          break;
    case BOW_BACK:
          move_body(FR_OPEN,FL_OPEN,B_CLOSE,Speed);
          break;
    }
}

void move_body(int fr_final_angle, int fl_final_angle, int b_final_angle, int Speed){
  int fr_angle,fl_angle,b_angle;
  fr_angle=body_right.read();
  fl_angle=body_left.read();
  b_angle=body_back.read();
  while(fr_angle!=fr_final_angle||fl_angle!=fl_final_angle||b_angle!=b_final_angle){
    move_servo_by_one_degree(body_right,fr_final_angle);
    move_servo_by_one_degree(body_left,fl_final_angle);
    move_servo_by_one_degree(body_back,b_final_angle);
    fr_angle=body_right.read();
    fl_angle=body_left.read();
    b_angle=body_back.read();
    delay(Speed*SPEED_MULTIPLIER);
  }
}


int move_eyes(int Speed, int r_low_final_angle, int r_high_final_angle, int l_low_final_angle, int l_high_final_angle ){
 int r_low_angle=low_r.read();
  int r_high_angle=high_r.read();
  int l_low_angle=low_l.read();
  int l_high_angle=high_l.read();
  while(r_high_angle!=r_high_final_angle||r_low_angle!=r_low_final_angle||l_high_angle!=l_high_final_angle||l_low_angle!=l_low_final_angle){
     move_servo_by_one_degree(low_r, r_low_final_angle);
     move_servo_by_one_degree(high_r, r_high_final_angle);
     move_servo_by_one_degree(low_l, l_low_final_angle);
     move_servo_by_one_degree(high_l, l_high_final_angle);
     
     r_low_angle=low_r.read();
     r_high_angle=high_r.read();
     l_low_angle=low_l.read();
     l_high_angle=high_l.read();
     delay(Speed*SPEED_MULTIPLIER);
    }
}

void eyes_callback(const std_msgs::Int8MultiArray&msg){
  int value=msg.data[0];
  int Speed=msg.data[1];
    switch(value){
    case EYES_STANDARD:
          move_eyes(Speed,STANDARD_LOW_R, STANDARD_HIGH_R, STANDARD_LOW_L, STANDARD_HIGH_L);
          break;
    case EYES_RIGH:
          move_eyes(Speed,LOOK_RIGH_LOW_R, LOOK_RIGH_HIGH_R, LOOK_RIGH_LOW_L, LOOK_RIGH_HIGH_L);
          break;
    case EYES_LEFT:
          move_eyes(Speed,LOOK_LEFT_LOW_R, LOOK_LEFT_HIGH_R, LOOK_LEFT_LOW_L, LOOK_LEFT_HIGH_L);
          break;
    case EYES_UP:
          move_eyes(Speed,LOOK_DOWN_LOW_R, LOOK_DOWN_HIGH_R, LOOK_DOWN_LOW_L, LOOK_DOWN_HIGH_L);
          break;
    case EYES_DOWN:
          move_eyes(Speed,LOOK_UP_LOW_R, LOOK_UP_HIGH_R, LOOK_UP_LOW_L, LOOK_UP_HIGH_L);
          break;
  }
}*/
