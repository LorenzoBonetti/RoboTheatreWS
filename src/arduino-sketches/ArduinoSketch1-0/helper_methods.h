#include <Servo.h>
#include <Arduino.h>
int move_servo_by_one_degree(Servo&servo, int final_angle);
void writeServo(Servo &theServo, int angle);
float getVoltageLevel(int battery_pin, int low_battery_voltage, int voltage_th);
