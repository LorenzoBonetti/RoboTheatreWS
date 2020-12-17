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
#include "defines.h"
#include "prova.cpp"
#include "body_manager.h"

void body_callback (const std_msgs::Int32&);

ros::NodeHandle  nh;
std_msgs::String response;
std_msgs::Int8 bat_msg;
ros::Publisher bat_pub("arduino/battery", &bat_msg);
ros::Subscriber<std_msgs::Int32> body_sub("Arduino/body", body_callback);
ros::Publisher response_pub("Arduino/response",&response);

Body_manager body_manager;


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
  nh.subscribe(body_sub);
  nh.advertise(response_pub);
  delay(1000);
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

void body_callback (const std_msgs::Int32& msg){
  int value=msg.data;
  body_manager.move_body(value, 5);
}
