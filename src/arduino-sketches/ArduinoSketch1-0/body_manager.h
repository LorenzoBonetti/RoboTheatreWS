#ifndef body_manager_h
#define body_manager_h
#include <ros.h>
#include <std_msgs/Int32.h>
#include "helper_methods.h"
#include "defines.h"
class Body_manager{
  private:
    Servo front_right_servo;
    Servo front_left_servo;
    Servo back_servo;
    
    void move_servos(int fr_final_angle, int fl_final_angle, int b_final_angle, int Speed);
  
  public:
    Body_manager();
    void move_body(int movement_type, int Speed);
  
};
#endif
