
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Bool.h>
#include <Servo.h>
#include "defines.h"
#include "helper_methods.h"

void eyes_callback(const std_msgs::Int8MultiArray&);
int move_eyes();
void body_callback (const std_msgs::Int8MultiArray&msg);
void move_body();
void reset_callback (const std_msgs::Bool&msg);

Servo low_r, high_r, low_l, high_l;
Servo body_right, body_left, body_back;

std_msgs::Bool eyes_msg;
std_msgs::Bool body_msg;

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int8MultiArray> body_sub("arduino/body", body_callback);
ros::Subscriber<std_msgs::Int8MultiArray> eyes_sub("arduino/eyes", eyes_callback);
ros::Subscriber<std_msgs::Bool>reset_sub("arduino/reset", reset_callback);
ros::Publisher eyes_pub("arduino/eyes_response", &eyes_msg);
ros::Publisher body_pub("arduino/body_response", &body_msg);

unsigned long last_battery_check_time=0;
unsigned long last_battery_beep_time=0;
bool should_beep;
unsigned long last_eyes_move_time=0;
unsigned long last_body_move_time=0;

int r_low_final_angle, r_high_final_angle, l_low_final_angle, l_high_final_angle;
int eyes_speed;
bool should_move_eyes;

int fr_final_angle, fl_final_angle, b_final_angle;
int body_speed;
bool should_move_body;


void setup()
{
  //Serial.begin(9600);
  pinMode(BAT_PROBE_LED,OUTPUT);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  delay(500);
  nh.subscribe(eyes_sub);
  nh.subscribe(body_sub);
  nh.subscribe(reset_sub);
  nh.advertise(eyes_pub);
  nh.advertise(body_pub);

  low_r.attach(R_LOW_SERVO_PIN);
  high_r.attach(R_HIGH_SERVO_PIN);
  low_l.attach(L_LOW_SERVO_PIN);
  high_l.attach(L_HIGH_SERVO_PIN);

  r_low_final_angle=STANDARD_LOW_R;
  r_high_final_angle=STANDARD_HIGH_R;
  l_low_final_angle=STANDARD_LOW_L;
  l_high_final_angle=STANDARD_HIGH_L;
  
  should_move_eyes=true;
  eyes_speed=5;

  body_right.attach(BODY_RIGH_SERVO_PIN);
  body_left.attach(BODY_LEFT_SERVO_PIN);
  body_back.attach(BODY_BACK_SERVO_PIN);

  fr_final_angle=FR_OPEN;
  fl_final_angle=FL_OPEN;
  b_final_angle=B_MIDDLE;
  body_speed=5;
  should_move_body=true;

  
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

    if(should_move_eyes){
      move_eyes();
    }
    if(should_move_body){
      move_body();
    }
    nh.spinOnce();
   
   
}

// Checks the battery level and produces a beep in case it is low. Also writes
// the voltage level to serial port.
float checkBatteryLevel(){
  float voltage = getVoltageLevel(BATTERY_PIN,LOW_BATTERY_VOLTAGE, VOLTAGE_TRH);
  
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
void reset_callback (const std_msgs::Bool&msg){
  bool reset=msg.data;
  if(reset){
    fr_final_angle=FR_OPEN;
    fl_final_angle=FL_OPEN;
    b_final_angle=B_MIDDLE;
    body_speed=5;
    should_move_body=true;

    r_low_final_angle=STANDARD_LOW_R;
    r_high_final_angle=STANDARD_HIGH_R;
    l_low_final_angle=STANDARD_LOW_L;
    l_high_final_angle=STANDARD_HIGH_L;
    eyes_speed=5;
    should_move_eyes=true;
  }
}

void body_callback (const std_msgs::Int8MultiArray&msg){
   int movement_type=msg.data[0];
   body_speed=msg.data[1];
   switch(movement_type){
    case NORMAL:
          fr_final_angle=FR_OPEN;
          fl_final_angle=FL_OPEN;
          b_final_angle=B_MIDDLE;
          break;
    case BOW:
          fr_final_angle=FR_CLOSE;
          fl_final_angle=FL_CLOSE;
          b_final_angle=B_OPEN;
          break;
    case BOW_RIGHT:
          fr_final_angle=FR_CLOSE;
          fl_final_angle=FL_OPEN;
          b_final_angle=B_OPEN;
          break;
    case BOW_LEFT:
          fr_final_angle=FR_OPEN;
          fl_final_angle=FL_CLOSE;
          b_final_angle=B_OPEN;
          break;
    case BOW_BACK:
          fr_final_angle=FR_OPEN;
          fl_final_angle=FL_OPEN;
          b_final_angle=B_CLOSE;
          break;
    }
    should_move_body=true;
}

void move_body(){
  int fr_angle,fl_angle,b_angle;
  fr_angle=body_right.read();
  fl_angle=body_left.read();
  b_angle=body_back.read();
  if(fr_angle!=fr_final_angle||fl_angle!=fl_final_angle||b_angle!=b_final_angle){
    if((millis()-last_body_move_time)>(body_speed*SPEED_MULTIPLIER)){
      last_body_move_time=millis();
      move_servo_by_one_degree(body_right,fr_final_angle);
      move_servo_by_one_degree(body_left,fl_final_angle);
      move_servo_by_one_degree(body_back,b_final_angle);
    }
  }else{
    should_move_body=false;
    body_msg.data = true;
    body_pub.publish(&body_msg);
  }
}


int move_eyes(){
  int r_low_angle=low_r.read();
  int r_high_angle=high_r.read();
  int l_low_angle=low_l.read();
  int l_high_angle=high_l.read();
  if(r_high_angle!=r_high_final_angle||r_low_angle!=r_low_final_angle||l_high_angle!=l_high_final_angle||l_low_angle!=l_low_final_angle){
    if((millis()-last_eyes_move_time)>(eyes_speed*SPEED_MULTIPLIER)){
      last_eyes_move_time=millis();
      move_servo_by_one_degree(low_r, r_low_final_angle);
      move_servo_by_one_degree(high_r, r_high_final_angle);
      move_servo_by_one_degree(low_l, l_low_final_angle);
      move_servo_by_one_degree(high_l, l_high_final_angle);
    }
  }else{
      should_move_eyes=false;
      eyes_msg.data = true;
      eyes_pub.publish(&eyes_msg);
    }
}

void eyes_callback(const std_msgs::Int8MultiArray&msg){
  int value=msg.data[0];
  eyes_speed=msg.data[1];
    switch(value){
    case EYES_STANDARD:
          r_low_final_angle=STANDARD_LOW_R;
          r_high_final_angle=STANDARD_HIGH_R;
          l_low_final_angle=STANDARD_LOW_L;
          l_high_final_angle=STANDARD_HIGH_L;
          break;
    case EYES_RIGH:
          r_low_final_angle=LOOK_RIGH_LOW_R;
          r_high_final_angle=LOOK_RIGH_HIGH_R;
          l_low_final_angle=LOOK_RIGH_LOW_L;
          l_high_final_angle=LOOK_RIGH_HIGH_L;
          break;
    case EYES_LEFT:
          r_low_final_angle=LOOK_LEFT_LOW_R;
          r_high_final_angle=LOOK_LEFT_HIGH_R;
          l_low_final_angle=LOOK_LEFT_LOW_L;
          l_high_final_angle=LOOK_LEFT_HIGH_L;
          break;
    case EYES_UP:
          r_low_final_angle=LOOK_DOWN_LOW_R;
          r_high_final_angle=LOOK_DOWN_HIGH_R;
          l_low_final_angle=LOOK_DOWN_LOW_L;
          l_high_final_angle=LOOK_DOWN_HIGH_L;
          break;
    case EYES_DOWN:
          r_low_final_angle=LOOK_UP_LOW_R;
          r_high_final_angle=LOOK_UP_HIGH_R;
          l_low_final_angle=LOOK_UP_LOW_L;
          l_high_final_angle=LOOK_UP_HIGH_L;
          break;
  }
  should_move_eyes=true;
}
