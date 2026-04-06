/*
 /\_/\  /\_/\  /\_/\  /\_/\        VSAR_26.ino:
( o.o )( o.o )( o.o )( o.o )       |___CONFIGURATION: Constants and channels
 > ^ <  > ^ <  > ^ <  > ^ <        |___PWM DRIVER: Adafruit PWM Servo Driver
#######              #######       |___HARDWARE API: dc_control()
 /\_/\    ghelopax    /\_/\        |___DRIVETRAIN: Mecanum Drive
( o.o )     ntm      ( o.o )       |___SUBSYSTEMS: Linear Slide, Sushi-roll Intake
 > ^ <   @itsmevjnk   > ^ <        |___ARDUINO FUNCTIONS: setup(), loop()
#######              #######
 /\_/\  /\_/\  /\_/\  /\_/\
( o.o )( o.o )( o.o )( o.o )
 > ^ <  > ^ <  > ^ <  > ^ <
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// #define DEBUG
#define RUN

// #############
// CONFIGURATION
// #############

/* CONSTANTS */
// Motor speed
#define SPD_MAX          4095
#define PER(percentage)  (int16_t)(SPD_MAX * percentage)

#define SPD_DRIVE        PER(1.0)
#define SPD_SLIDE        PER(0.8)
#define SPD_INTAKE       PER(0.8)
#define SPD_CONVEY_LOAD  PER(0.3)
#define SPD_CONVEY_SHOOT PER(1.0)

/* PWM channels */
// DC Motor
// Drivetrain
#define LF_A             1      // Mecanum Drive
#define LF_B             2
#define LB_A             3
#define LB_B             4
#define RF_A             7
#define RF_B             8
#define RB_A             5
#define RB_B             6

// Subsystem
#define LS_A             9      // Linear Slide
#define LS_B             10

#define IT_A             11     // Intake
#define IT_B             12

#define CB_A             13     // Conveyor Belt
#define CB_B             14

/* PS2 pins */
#define PS2_DAT          13
#define PS2_CMD          11
#define PS2_ATT          10
#define PS2_CLK          12


// ##########
// PWM DRIVER
// ##########

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void init_PWMDriver() {
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

void init_PS2() {
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

    delay(1000);

    error = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);
  }

  Serial.println(F("done."));
}

/* Control */
// speed = 0...4095
void dc_control(uint8_t channelA, uint8_t channelB, int16_t speed, bool reverse = false) {
  if (reverse) speed = -speed;

  #ifdef RUN
  pwm.setPWM(channelA, 0, ((speed > 0) ?   speed  : 0));
  pwm.setPWM(channelB, 0, ((speed < 0) ? (-speed) : 0));
  #endif
}


// #########################
// DRIVETRAIN: Mecanum Drive
// #########################

void update_drivetrain(uint8_t stra, uint8_t forw, uint8_t rota) {
  int16_t x = map(stra, 0, 255, -SPD_DRIVE,  SPD_DRIVE);
  int16_t y = map(forw, 0, 255,  SPD_DRIVE, -SPD_DRIVE);
  int16_t r = map(rota, 0, 255,  SPD_DRIVE, -SPD_DRIVE);
  int16_t d = max(abs(x) + abs(y) + abs(r), SPD_DRIVE);

  dc_control(LF_A, LF_B, (double)( x + y - r) / d * SPD_DRIVE);
  dc_control(LB_A, LB_B, (double)(-x + y - r) / d * SPD_DRIVE);
  dc_control(RB_A, RB_B, (double)( x + y + r) / d * SPD_DRIVE, true);
  dc_control(RF_A, RF_B, (double)(-x + y + r) / d * SPD_DRIVE, true);

  #ifdef DEBUG
  Serial.println("MECANUM:");
  Serial.println(x);
  Serial.println(y);
  Serial.println(r);
  Serial.println(d);
  Serial.print((double)( x + y - r) / d); Serial.print(" "); Serial.print((double)(-x + y + r) / d); Serial.print("\n"); // LF RF
  Serial.print((double)(-x + y - r) / d); Serial.print(" "); Serial.print((double)( x + y + r) / d); Serial.print("\n"); // LB RB
  delay(500);
  #endif
}


// ##########
// SUBSYSTEMS
// ##########

/* Linear Slide */
void update_linearslide(bool exte, bool rest) {
  if (!(exte ^ rest)) return;

  if (exte) dc_control(LS_A, LS_B, SPD_SLIDE);
  if (rest) dc_control(LS_A, LS_B, 0);
}

/* Sushi-roll intake */
bool state_intake;

void init_intake() {
  state_intake = false;
}

void update_intake(bool togg) {
  state_intake ^= togg;

  if (togg) dc_control(IT_A, IT_B, (state_intake ? SPD_INTAKE : 0));
}

/* Conveyor Belt */
bool conveyorbelt_shoot;

void init_conveyorbelt() {
  conveyorbelt_shoot = false;
}

void update_conveyorbelt(bool run, bool rest, bool togg) {
  conveyorbelt_shoot ^= togg;

  if (run)  dc_control(CB_A, CB_B, (conveyorbelt_shoot ? SPD_CONVEY_SHOOT : SPD_CONVEY_LOAD));
  if (rest) dc_control(CB_A, CB_B, 0);
}


// #################
// ARDUINO FUNCTIONS
// #################

void setup() {
  Serial.begin(115200);  // Arduino Uno R3 baud rate (bps)

  Serial.println("VSAR 2026 : RIAN C");

  init_PWMDriver();
  init_PS2();

  update_linearslide(0, 1);
  init_intake();
  init_conveyorbelt();
}

void loop() {
  ps2.read_gamepad();  // update from controller

  update_drivetrain(ps2.Analog(PSS_LX), ps2.Analog(PSS_LY), ps2.Analog(PSS_RX));
  update_linearslide(ps2.ButtonPressed(PSB_R1), ps2.ButtonReleased(PSB_R1));
  update_intake(ps2.ButtonPressed(PSB_L1));
  update_conveyorbelt(ps2.ButtonPressed(PSB_R2), ps2.ButtonReleased(PSB_R2), ps2.ButtonPressed(PSB_TRIANGLE));
}