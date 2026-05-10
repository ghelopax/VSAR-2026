/*
 /\_/\  /\_/\  /\_/\  /\_/\        VSAR_26.ino:
( o.o )( o.o )( o.o )( o.o )       |___CONFIGURATION: Constants and channels
 > ^ <  > ^ <  > ^ <  > ^ <        |___PWM DRIVER: Adafruit PWM Servo Driver
#######              #######       |___HARDWARE API: DCMotor, Servo, CRServo
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

// #define DEBUG_MECANUM
// #define DEBUG_SETPWM
#define RUN

// #############
// CONFIGURATION
// #############

/* CONSTANTS */
// Motor speed
#define SPD_MAX          4095
#define SPD_DEAD         70
#define PER(percentage)  (int16_t)(SPD_MAX * percentage)

#define SPD_DRIVE_LF     PER(0.70)
#define SPD_DRIVE_LB     PER(0.67)
#define SPD_DRIVE_RF     PER(0.67)
#define SPD_DRIVE_RB     PER(0.67)
#define SPD_SLIDE        PER(1.00)
#define SPD_INTAKE       PER(1.00)
#define SPD_CONVEY       PER(0.80)

// Servo position 
#define POS_MIN          440
#define POS_MAX          2270

// CRServo speed
#define CRSPD_L          400
#define CRSPD_M          1400

/* PWM channels */
// DC Motor
// Drivetrain
#define LF_A             6      // Mecanum Drive
#define LF_B             7
#define LB_A             4
#define LB_B             5
#define RF_A             2
#define RF_B             3
#define RB_A             0
#define RB_B             1

// Subsystem
#define LS_A             8      // Linear Slide
#define LS_B             9

#define IT_A             12     // Intake
#define IT_B             13

#define CB_A             10     // Conveyor Belt
#define CB_B             11

// Servo
#define S_OT             14

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
struct DCMotor {
  private:
  uint8_t channelA, channelB;
  bool reverse;

  public:
  DCMotor(uint8_t _channelA, uint8_t _channelB, bool _reverse = false) : 
    channelA(_channelA), 
    channelB(_channelB), 
    reverse(_reverse) 
  {}

  void control(int16_t speed) {
    if (abs(speed) < SPD_DEAD) speed = 0;
    if (reverse) speed = -speed;

    #ifdef RUN
    pwm.setPWM(channelA, 0, ((speed > 0) ?   speed  : 0));
    pwm.setPWM(channelB, 0, ((speed < 0) ? (-speed) : 0));
    #endif

    #ifdef DEBUG_SETPWM
    Serial.print(channelA); Serial.print(": "); Serial.print(pwm.getPWM(channelA, true)); SPC; 
    Serial.print(channelB); Serial.print(": "); Serial.println(pwm.getPWM(channelB, true));
    #endif
  }
};

struct Servo {
  private:
  uint8_t channel;

  public:
  Servo(uint8_t _channel) : channel(_channel) {}

  void control(int16_t position) {
    #ifdef RUN
    pwm.writeMicroseconds(channel, position);
    #endif
  }

  // rel_pos = R[0...1]
  void relControl(float rel_pos) {
    control(POS_MIN + rel_pos * (POS_MAX - POS_MIN));
  }
};

struct CRServo {
  private:
  uint8_t channel;

  public:
  CRServo(uint8_t _channel) : channel(_channel) {}

  void control(int16_t speed) {
    #ifdef RUN
    pwm.writeMicroseconds(channel, speed);
    #endif
  }

  // rel_speed = R[-1...1]
  void relControl(float rel_speed) {
    control(CRSPD_M + rel_speed * (CRSPD_M - CRSPD_L));
  }
};


// #########################
// DRIVETRAIN: Mecanum Drive
// #########################

struct Drivetrain {
  private:
  DCMotor leftfront, leftback, rightfront, rightback;

  public:
  Drivetrain() : 
    leftfront(LF_A, LF_B), 
    leftback(LB_A, LB_B), 
    rightfront(RF_A, RF_B, true), 
    rightback(RB_A, RB_B, true)
  {}

  void test() { // default to forward movement
    leftfront .control(SPD_DRIVE_LF);
    leftback  .control(SPD_DRIVE_LB);
    rightfront.control(SPD_DRIVE_RF);
    rightback .control(SPD_DRIVE_RB);
  }

  void update(uint8_t stra, uint8_t forw, uint8_t rota) {
    int16_t x = map(stra, 0, 255, -SPD_MAX,  SPD_MAX);
    int16_t y = map(forw, 0, 255,  SPD_MAX, -SPD_MAX);
    int16_t r = map(rota, 0, 255,  SPD_MAX, -SPD_MAX);
    int16_t d = max(abs(x) + abs(y) + abs(r), SPD_MAX);

    int16_t lf = (int32_t)( x + y - r) * SPD_DRIVE_LF / d;
    int16_t lb = (int32_t)(-x + y - r) * SPD_DRIVE_LB / d;
    int16_t rf = (int32_t)(-x + y + r) * SPD_DRIVE_RF / d;
    int16_t rb = (int32_t)( x + y + r) * SPD_DRIVE_RB / d;

    leftfront .control(lf);
    leftback  .control(lb);
    rightfront.control(rf);
    rightback .control(rb);
  
    #ifdef DEBUG_MECANUM
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
  private:
  DCMotor slide;

  public:
  Linear_Slide() : slide(LS_A, LS_B) {}

  void update(bool exte, bool retr) {
    if (!(exte ^ retr)) slide.control(0);

    if (exte) slide.control( SPD_SLIDE);
    if (retr) slide.control(-SPD_SLIDE);
  }
} linearslide;

/* Sushi-roll intake (TOGGLE) */
struct Intake {
  private:
  bool state;
  DCMotor in;

  public:
  Intake() : state(false), in(IT_A, IT_B) {}

  void update(bool togg) {
    state ^= togg;

    in.control((state ? SPD_INTAKE : 0));
  }
} intake;

/* Conveyor Belt (TOGGLE) */
struct Conveyor_Belt {
  private:
  bool state;
  DCMotor convey;

  public:
  Conveyor_Belt() : state(false), convey(CB_A, CB_B) {}

  void update(bool togg) {
    state ^= togg;

    convey.control(state ? SPD_CONVEY : 0);
  }

} conveyorbelt;

/* Outtake (FSM) */
struct Outtake {
  private:
  Servo out;
  char state;
  /*
  '3' Block 1 2 3
  '2' Block 1 2
  '1' Block 1
  '0' Block -
  */
  char aut[4];
  float pos[4];

  public:
  Outtake() : state('3'), out(S_OT) {
    aut[3] = '2';
    aut[2] = '1';
    aut[1] = '0';
    aut[0] = '3';

    pos[3] = 0.43;
    pos[2] = 0.63;
    pos[1] = 0.76;
    pos[0] = 0.83;
  }

  void update(bool togg) {
    if (togg) state = aut[state - '0'];

    out.relControl(pos[state - '0']);
  }
} outtake;

// #################
// ARDUINO FUNCTIONS
// #################

void setup() {
  Serial.begin(115200);  // Arduino Uno R3 baud rate (bps)

  Serial.println("Robots In A Nutshell\nVSAR 2026 : RIAN C\nBy ghelopax w/ @ntm. Original: @itsmevjnk/VRC-2022.\n");

  init_PWMDriver();
  init_PS2();
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
    ps2.ButtonPressed(PSB_R1)
  );

  outtake.update(
    ps2.ButtonPressed(PSB_R2)
  );

  #ifdef DEBUG_SETPWM
  delay(100);
  #endif
}