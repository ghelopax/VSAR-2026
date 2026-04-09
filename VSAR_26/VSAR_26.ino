/*
 /\_/\  /\_/\  /\_/\  /\_/\        VSAR_26.ino:
( o.o )( o.o )( o.o )( o.o )       |___CONFIGURATION: Constants and channels
 > ^ <  > ^ <  > ^ <  > ^ <        |___PWM DRIVER: Adafruit PWM Servo Driver
#######              #######       |___HARDWARE API: dc_control()
 /\_/\    ghelopax    /\_/\        |___DRIVETRAIN: Mecanum Drive
( o.o )     ntm      ( o.o )       |___SUBSYSTEMS: Linear Slide, Sushi-roll Intake, Conveyor Belt
 > ^ <   @itsmevjnk   > ^ <        |___ARDUINO FUNCTIONS: setup(), loop()
#######              #######
 /\_/\  /\_/\  /\_/\  /\_/\
( o.o )( o.o )( o.o )( o.o )
 > ^ <  > ^ <  > ^ <  > ^ <
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#define SPC Serial.print(" ")
#define EL  Serial.print("\n");

// #define DEBUG
#define RUN

// #############
// CONFIGURATION
// #############

/* CONSTANTS */
// Motor speed
#define SPD_MAX          4095
#define SPD_DEAD         50
#define PER(percentage)  (int16_t)(SPD_MAX * percentage)

#define SPD_DRIVE        PER(1.0)
#define SPD_SLIDE        PER(0.8)
#define SPD_INTAKE       PER(0.8)
#define SPD_CONVEY_LOAD  PER(0.3)
#define SPD_CONVEY_SHOOT PER(1.0)

/* PWM channels */
// DC Motor
// Drivetrain
#define LF_A             0      // Mecanum Drive
#define LF_B             1
#define LB_A             2
#define LB_B             3
#define RF_A             6
#define RF_B             7
#define RB_A             4
#define RB_B             5

// Subsystem
#define LS_A             8      // Linear Slide
#define LS_B             9

#define IT_A             10     // Intake
#define IT_B             11

#define CB_A             12     // Conveyor Belt
#define CB_B             13

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
  if (abs(speed) < SPD_DEAD) speed = 0;
  if (reverse) speed = -speed;

  #ifdef RUN
  pwm.setPWM(channelA, 0, ((speed > 0) ?   speed  : 0));
  pwm.setPWM(channelB, 0, ((speed < 0) ? (-speed) : 0));
  #endif
}


// #########################
// DRIVETRAIN: Mecanum Drive
// #########################

struct Drivetrain {
  void test() {
    dc_control(LF_A, LF_B, SPD_DRIVE);
    dc_control(LB_A, LB_B, SPD_DRIVE);
    dc_control(RF_A, RF_B, SPD_DRIVE, true);
    dc_control(RB_A, RB_B, SPD_DRIVE, true);
  }

  void update(uint8_t stra, uint8_t forw, uint8_t rota) {
    int16_t x = map(stra, 0, 255, -SPD_DRIVE,  SPD_DRIVE);
    int16_t y = map(forw, 0, 255,  SPD_DRIVE, -SPD_DRIVE);
    int16_t r = map(rota, 0, 255,  SPD_DRIVE, -SPD_DRIVE);
    int16_t d = max(abs(x) + abs(y) + abs(r), SPD_DRIVE);

    int16_t lf = (int32_t)( x + y - r) * SPD_DRIVE / d;
    int16_t lb = (int32_t)(-x + y - r) * SPD_DRIVE / d;
    int16_t rf = (int32_t)(-x + y + r) * SPD_DRIVE / d;
    int16_t rb = (int32_t)( x + y + r) * SPD_DRIVE / d;

    dc_control(LF_A, LF_B, lf);
    dc_control(LB_A, LB_B, lb);
    dc_control(RF_A, RF_B, rf, true);
    dc_control(RB_A, RB_B, rb, true);
    
    #ifdef DEBUG
    Serial.println("MECANUM:");
    Serial.println(x);
    Serial.println(y);
    Serial.println(r);
    Serial.println(d);
    Serial.print(lf); SPC; Serial.println(rf);
    Serial.print(lb); SPC; Serial.println(rb);
    delay(500);
    #endif
  }
} drivetrain;

// ##########
// SUBSYSTEMS
// ##########

/* Linear Slide (PUSH: Multistate) */
struct Linear_Slide {
  void update(bool exte, bool retr) {
    if (!(exte ^ retr)) dc_control(LS_A, LS_B, 0);

    if (exte) dc_control(LS_A, LS_B,  SPD_SLIDE);
    if (retr) dc_control(LS_A, LS_B, -SPD_SLIDE);
  }
} linearslide;

/* Sushi-roll intake (TOGGLE) */
struct Intake {
  bool state;

  void init() {
    state = false;
  }

  void update(bool togg) {
    state ^= togg;

    dc_control(IT_A, IT_B, (state ? SPD_INTAKE : 0));
  }
} intake;

/* Conveyor Belt (PUSH + TOGGLE Mode) */
struct Conveyor_Belt {
  bool shootmode;

  void init() {
    shootmode = false;
  }

  void update(bool run, bool togg) {
    shootmode ^= togg;

    if (run) dc_control(CB_A, CB_B, (shootmode ? SPD_CONVEY_SHOOT : SPD_CONVEY_LOAD));
    else     dc_control(CB_A, CB_B, 0);
  }

} conveyorbelt;

// #################
// ARDUINO FUNCTIONS
// #################

void setup() {
  Serial.begin(115200);  // Arduino Uno R3 baud rate (bps)

  Serial.println("VSAR 2026 : RIAN C");

  init_PWMDriver();
  init_PS2();

  linearslide.update(0, 1);
  intake.init();
  conveyorbelt.init();
}

void loop() {
  ps2.read_gamepad();  // update from controller

  drivetrain.update(
    ps2.Analog(PSS_LX),
    ps2.Analog(PSS_LY),
    ps2.Analog(PSS_RX)
  );
  // drivetrain.test();

  linearslide.update(
    ps2.Button(PSB_PAD_UP),
    ps2.Button(PSB_PAD_DOWN)
  );

  intake.update(
    ps2.ButtonPressed(PSB_L1)
  );

  conveyorbelt.update(
    ps2.Button(PSB_R1),
    ps2.ButtonPressed(PSB_TRIANGLE)
  );
}