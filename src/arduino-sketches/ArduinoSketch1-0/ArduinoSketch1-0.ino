/*
 * Sketch for Arduino
 * Lorenzo Bonetti
 * LEFT SERVO: CLOSED AT 0  OPEN AT 180
 * RIGHT SERVO CLOSED AT 180 OPEN AT 0
 * BACK SERVO CLOSED AT 0 OPEN AT 180
 */
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <Servo.h>

#define FRONT_RIGHT_SERVO_PIN 10
#define FRONT_LEFT_SERVO_PIN 11
#define BACK_SERVO_PIN 12
#define BUZZER_PIN  8
#define BAT_PROBE_LED 13
#define BATTERY_PIN A0

#define NORMAL 0
#define BOW 1
#define BOW_RIGHT 2
#define BOW_LEFT 3
#define BOW_BACK 4

#define BAT_CHK_TIMEOUT_TRH 10000     // publishes battery data every 10 secs.
#define BEEPING_INTERVAL 5000
#define LOW_BATTERY_VOLTAGE 21
#define VOLTAGE_TRH 3.70  //3.70 correspond to battery level at 21V.

#define NOTE_TO_PLAY 262 //corresponds to C4
#define NOTE_DURATION 1000

void servoCallback (const std_msgs::Int32&);
void eyesCallback(const std_msgs::Int8MultiArray);

ros::NodeHandle  nh;
std_msgs::String response;
std_msgs::Int8 bat_msg;
ros::Publisher bat_pub("arduino/battery", &bat_msg);
ros::Subscriber<std_msgs::Int32> servo_sub("Arduino/servo", servoCallback);
ros::Publisher response_pub("Arduino/response",&response);

Servo front_right_servo;
Servo front_left_servo;
Servo back_servo;

unsigned long last_battery_check_time=0;
unsigned long last_battery_beep_time=0;
bool should_beep;


void setup()
{
  pinMode(BAT_PROBE_LED,OUTPUT);
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  delay(1000);
  nh.advertise(bat_pub);
  nh.subscribe(servo_sub);
  nh.advertise(response_pub);
  delay(1000);
  front_right_servo.attach(FRONT_RIGHT_SERVO_PIN);
  front_left_servo.attach(FRONT_LEFT_SERVO_PIN);
  back_servo.attach(BACK_SERVO_PIN);
  delay(250);

  front_right_servo.write(0);//OPEN
  front_left_servo.write(180);//OPEN
  back_servo.write(180);//OPEN
}

void loop()
{
    // Publish battery state every 3 seconds!
    if (millis() - last_battery_check_time > BAT_CHK_TIMEOUT_TRH){
        last_battery_check_time = millis();
        float voltage = checkBatteryLevel();
        if (voltage != 0){
          digitalWrite(BAT_PROBE_LED, HIGH);
        }
        else{
          digitalWrite(BAT_PROBE_LED, LOW);
        }
        bat_msg.data = voltage;
        bat_pub.publish(&bat_msg);
    }
    
    nh.spinOnce();
    delay(3);

}


void servoCallback (const std_msgs::Int32& msg){
  int value=msg.data;
  switch(value){
    case NORMAL:
          front_right_servo.write(0);//OPEN
          front_left_servo.write(180);//OPEN
          back_servo.write(180);//OPEN
          response.data="bow";
          break;
    case BOW:
          front_right_servo.write(180);//CLOSED
          front_left_servo.write(0);//CLOSED
          back_servo.write(180);//OPEN
          response.data="bow";
          break;
    case BOW_RIGHT:
          response.data="bow_right";
          front_right_servo.write(180);//CLOSED
          front_left_servo.write(180);//OPEN
          back_servo.write(180);//OPEN
          break;
    case BOW_LEFT:
          response.data="bow_left";
          front_right_servo.write(0);//OPEN
          front_left_servo.write(0);//CLOSED
          back_servo.write(180);//OPEN
          break;
    case BOW_BACK:
          response.data="bow_back";
          front_right_servo.write(0);//OPEN
          front_left_servo.write(180);//OPEN
          back_servo.write(0);//CLOSED
          break;
    default:
          response.data="invalid_command";
    
  }
  response_pub.publish(&response);
  
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
