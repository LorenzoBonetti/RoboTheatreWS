#include "helper_methods.h"
int move_servo_by_one_degree(Servo&servo, int final_angle){
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
void writeServo(Servo &theServo, int angle){
  theServo.write(angle);
}

// Reads voltage level at battery pin (analog). This
// voltage comes from the voltage divider circuit used to monitor the battery.
float getVoltageLevel(int battery_pin, int low_battery_voltage, float voltage_th){
  float voltage = (analogRead(battery_pin) * 5.015) / 1024.0;
  return (low_battery_voltage*voltage)/voltage_th;  // Converted voltage (approx).
}
