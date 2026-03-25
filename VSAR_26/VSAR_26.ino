/*
 /\_/\  /\_/\  /\_/\  /\_/\        VSAR_26.ino:
( o.o )( o.o )( o.o )( o.o )       |___CONFIGURATION: Constants and channels
 > ^ <  > ^ <  > ^ <  > ^ <        |___PWM DRIVER: Adafruit PWM Servo Driver
#######              #######       |___HARDWARE API: dc_control()
 /\_/\    ghelopax    /\_/\        |___DRIVETRAIN: Mecanum Drive
( o.o )              ( o.o )       |___ARDUINO FUNCTIONS: setup(), loop()
 > ^ <   @itsmevjnk   > ^ <
#######              #######
 /\_/\  /\_/\  /\_/\  /\_/\
( o.o )( o.o )( o.o )( o.o )
 > ^ <  > ^ <  > ^ <  > ^ <
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// #############
// CONFIGURATION
// #############

/* CONSTANTS */
// Drive motor speed
#define SPD_DRIVE           3072
#define SPD_DEAD            80
// Servo PW
#define PW_MIN              440
#define PW_MAX              2270
#define CR_PW_MIN           400
#define CR_PW_MID           1400

/* PWM channels */
// DC Motor
#define DRIVE_LF_A 1
#define DRIVE_LF_B 2

#define DRIVE_LB_A 3
#define DRIVE_LB_B 4

#define DRIVE_RF_A 7
#define DRIVE_RF_B 8

#define DRIVE_RB_A 5
#define DRIVE_RB_B 6

/* PS2 pins */
#define PS2_DAT             12
#define PS2_CMD             13
#define PS2_ATT             15
#define PS2_CLK             14


// ##########
// PWM DRIVER
// ##########

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void PWMDriver_init() {
  Serial.print(F("Initializing PCA9685..."));

  // PWM Init
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  // Wire Init
  Wire.setClock(400000);

  Serial.println(F("done."));
}


// ############
// HARDWARE API
// ############

/* PS2 Controller */
PS2X ps2;

void PS2_init() {
  Serial.print(F("Initializing PS2 controller..."));

  uint8_t error = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);
  while (error != 0) {
    switch (error) {
      case 1:
        Serial.println("\nError code 1: No controller found, check wiring.");
        break;
      case 2:
        Serial.println("\nError code 2: Controller found but not accepting commands.");
        break;
      case 3:
        Serial.println("\nError code 3: Controller refusing to enter Pressures mode, may not support it.");
        break;
    }

    error = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);
  }

  Serial.println(F("done."));
}

/* Control */
void dc_control(uint8_t channelA, uint8_t channelB, int16_t speed) {
  pwm.setPWM(channelA, 0, ((speed > 0) ?   speed  : 0));
  pwm.setPWM(channelB, 0, ((speed < 0) ? (-speed) : 0));
}


// #########################
// DRIVETRAIN: Mecanum Drive
// #########################

void drivetrain_update() {
  
}


// #################
// ARDUINO FUNCTIONS
// #################

void setup() {
  Serial.begin(115200);  // Arduino Uno R3 baud rate (bps)

  PWMDriver_init();
  PS2_init();
}

void loop() {
  ps2.read_gamepad();  // update from controller

  drivetrain_update();
}