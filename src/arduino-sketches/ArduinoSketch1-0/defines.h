#ifndef defines_h
#define defines_h


//Arduino Pins

//Battery Pins
#define BUZZER_PIN  8
#define BAT_PROBE_LED 13
#define BATTERY_PIN A0

//Eyes Pins
#define R_LOW_SERVO_PIN 4
#define R_HIGH_SERVO_PIN 5
#define L_LOW_SERVO_PIN 7
#define L_HIGH_SERVO_PIN 6

//Body Pins

#define BODY_RIGH_SERVO_PIN 10
#define BODY_LEFT_SERVO_PIN 11
#define BODY_BACK_SERVO_PIN 12

//Hardware Parts defines


#define SPEED_MULTIPLIER 10

//battery defines

#define BAT_CHK_TIMEOUT_TRH 5000     // publishes battery data every 60 secs.
#define BEEPING_INTERVAL 5000
#define LOW_BATTERY_VOLTAGE 21
#define VOLTAGE_TRH 3.70  //3.70 correspond to battery level at 21V.

#define NOTE_TO_PLAY 262 //corresponds to C4
#define NOTE_DURATION 1000

//body defines
//servo angles
#define FR_OPEN 0
#define FR_CLOSE 180
#define FL_OPEN 180
#define FL_CLOSE 0
#define B_OPEN 180
#define B_MIDDLE 120
#define B_CLOSE 0

#define NORMAL 0
#define BOW 1
#define BOW_RIGHT 2
#define BOW_LEFT 3
#define BOW_BACK 4

//eyes defines
//right eye angles
#define STANDARD_LOW_R 70
#define STANDARD_HIGH_R 70
#define LOOK_RIGH_HIGH_R 100
#define LOOK_RIGH_LOW_R 110
#define LOOK_LEFT_HIGH_R 0
#define LOOK_LEFT_LOW_R 50
#define LOOK_DOWN_LOW_R 0
#define LOOK_DOWN_HIGH_R 90
#define LOOK_UP_LOW_R 100
#define LOOK_UP_HIGH_R 40

//left eye angles
#define STANDARD_LOW_L 70
#define STANDARD_HIGH_L 90
#define LOOK_RIGH_HIGH_L 180
#define LOOK_RIGH_LOW_L  110
#define LOOK_LEFT_HIGH_L 90
#define LOOK_LEFT_LOW_L 40
#define LOOK_DOWN_LOW_L 130
#define LOOK_DOWN_HIGH_L 60
#define LOOK_UP_LOW_L 65
#define LOOK_UP_HIGH_L 150

#define EYES_STANDARD 0
#define EYES_RIGH 1
#define EYES_LEFT 2
#define EYES_UP 3
#define EYES_DOWN 4
#define EYES_CROSS 5
#define EYES_DIVIDE 6

#endif
