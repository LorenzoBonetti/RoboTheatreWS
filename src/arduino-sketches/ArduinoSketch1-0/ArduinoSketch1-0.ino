/*
 * Sketch for Arduino
 * Lorenzo Bonetti
 */
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <Servo.h>

#define FRONT_RIGHT_SERVO_PIN 9
#define FRONT_LEFT_SERVO_PIN 10
#define BACK_SERVO_PIN 11
#define BUZZER_PIN  6
#define BAT_PROBE_LED 8
#define BATTERY_PIN A2

#define BOW 0
#define BOW_RIGHT 1
#define BOW_LEFT 2
#define BOW_BACK 3



#define OFF 0
#define ON 1
#define BAT_CHK_TIMEOUT_TRH 3000     // publishes battery data every 3 secs.
#define BEEPING_INTERVAL 400
#define LOW_BATTERY_VOLTAGE 22
#define VOLTAGE_TRH 1.947   // A voltage less then 1.77 correspond to battery 
                            // level at 20V, 1.947 correspond to battery level at 22V.

void servoCallback (const std_msgs::Int32&);

ros::NodeHandle  nh;
std_msgs::String response;
std_msgs::Int8 bat_msg;
ros::Publisher bat_pub("arduino/battery", &bat_msg);
ros::Subscriber<std_msgs::Int32> servo_sub("Arduino/servo", servoCallback);
ros::Publisher response_pub("Arduino/response",&response);

Servo front_right_servo;
Servo front_left_servo;
Servo back_servo;

unsigned long last_battery_check_time;
unsigned long last_battery_beep_time;
unsigned long last_acc_time;          // last acc sample time that we saw. 
int should_beep;


void setup()
{
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
        bat_msg.data = voltage;
        bat_pub.publish(&bat_msg);
    }
    
    nh.spinOnce();
    delay(3);
    digitalWrite(BAT_PROBE_LED, LOW);
}


void servoCallback (const std_msgs::Int32& msg){
  int value=msg.data;
  switch(value){
    case BOW:
          front_right_servo.write(0);
          front_left_servo.write(0);
          response.data="bow";
          break;
    case BOW_RIGHT:
          response.data="bow_right";
          break;
    case BOW_LEFT:
          response.data="bow_left";
          break;
    case BOW_BACK:
          response.data="bow_back";
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
      if (should_beep == OFF) {
        should_beep = ON;
        beep(200);
      } else {
        should_beep = OFF;
        beep(0);
      }
    }
  }else{
    digitalWrite(BUZZER_PIN, LOW);
  }
  return voltage;
}

void beep(unsigned char delayms){
  analogWrite(BUZZER_PIN, delayms);      // Almost any value can be used except 0 and 255
}
