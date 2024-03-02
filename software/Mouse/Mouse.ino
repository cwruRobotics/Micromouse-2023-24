/* --- Includes --- */
// Wire.h for I2C communication with TCA chip and LiDAR
#include <Wire.h>
// Arduino standard encoder library
#include <Encoder.h>
// Adafruit library for VL6180X LiDAR sensor
#include "Adafruit_VL6180X.h"
// Pin definitions
#include "micromouse_pins_2023.h"
// Robot Logic and Algorithm definitions
#include "Algo/FMicro.cpp"
/* ---- Defines ---- */
typedef enum motor_t {
    LEFT_MOTOR = 0,
    RIGHT_MOTOR = 1
} motor_t;

#define TEST
#undef log
#undef logf
#undef logln

#ifdef TEST
  #define log(...) Serial.print(__VA_ARGS__)
  #define logf(...) Serial.printf(__VA_ARGS__)
  #define logln(...) Serial.println(__VA_ARGS__)
  #define LOGGING 1
#else
  #define log(...)
  #define logf(...)
  #define logln(...)
  #define LOGGING 0
#endif

#define POWER_DEADBAND 6

#define LIDAR_COUNT 4
#define LIDAR_ADDR_BASE 0x50

// The physical distance between the sensors
#define LIDAR_SEPERATION 123 // 123 mm between sensors

#define ANGLE_TOLERANCE 5

const double wheelSeparation = 9.5; // 9.5 cm between wheels
const double wheelRadius = 3; // 3 cm radius
const double turnRatio = (wheelSeparation / 2.0) / wheelRadius / 360 * 380 * 12; // degree to encoder tick conversion ratio

// The LiDAR sensors return a running average of readings,
//  so when we move past a wall, the LiDAR returns a value greater than the previous value but less than an overflow.
// (After a certain amount of time, the running average overflows the maximum and only then does the LiDAR throw a read error)
// If the LiDAR is greater than this value, we assume that it's not sensing the wall.
#define SENSOR_RANGE_MAX 110

// When centered, there should be 60mm in front of the ultrasonic
#define ULTRASONIC_FRONT 60

// Squares are 10in by 10in, but we work in cm. 10in = 25.4cm
#define SQUARE_SIZE 25.4

/* ---- User Variables ---- */

// GPIO pin numbers for the CS line on each LiDAR sensor
// We have another 2 pins on the board, but nothing's plugged into them
const int lidar_cs_pins[LIDAR_COUNT] = {LIDAR_CS1, LIDAR_CS2, LIDAR_CS3, LIDAR_CS4};

Adafruit_VL6180X lidar_sensors[LIDAR_COUNT];

bool back_right_errored, front_right_errored, back_left_errored, front_left_errored;
uint8_t back_right;
uint8_t front_right;
uint8_t back_left;
uint8_t front_left;

Encoder rightEncoder (ENCODER_RIGHT_1, ENCODER_RIGHT_2);
Encoder leftEncoder (ENCODER_LEFT_1, ENCODER_LEFT_2);

double ultrasonic;
double ultrasonic_running_average;
const double ultrasonic_ave_factor = 0.1;

// Magic constant that converts the time the ultrasonic takes to read the sensors into millimeters
// Based on the speed of sound
double ultrasonic_distance_factor = 0.17;

bool ultrasonic_errored;

/**
 * Convert a value in range [-127..127] to a motor power value
 *
 * @param p The input power [-127..127]
 * @return Output power [0..255]
 */
uint8_t convertPower(int8_t p) {
  if (p == 0) {
    return 255;
  }
  if (p < 0) {
    p = -p;
  }
  return 255 - (((uint8_t) p) * 2);
}

/**
 * Set motor power for a specified motor
 *
 * @param m The motor to modify
 * @param power The power and direction of the motor
 *              (range: [-127..127])
 *              Positive is "forward"
 *              Negative is "backward"
 */
void setMotor (motor_t m, int power) {
  if (power < -127) {
    power = -127;
  }else if (power > 127) {
    power = 127;
  }
  int m1, m2;

  // Determine motor
  if (m == LEFT_MOTOR) {
    m1 = MOTORLEFT_1;
    m2 = MOTORLEFT_2;
  }else if (m == RIGHT_MOTOR) {
    m1 = MOTORRIGHT_1;
    m2 = MOTORRIGHT_2;
  }else {
    return;
  }
  // Set power
  if (power < POWER_DEADBAND && power > -POWER_DEADBAND) {
      analogWrite(m1, 255);
      analogWrite(m2, 255);
  } else if (power > 0) {
      analogWrite(m1, convertPower(power));
      analogWrite(m2, 255);
  } else {
      analogWrite(m1, 255);
      analogWrite(m2, convertPower(power));
  }
}

int wallLeft() {
  if (lidar_sensors[3].readRangeStatus() != VL6180X_ERROR_NONE || front_left > SENSOR_RANGE_MAX) {
    coolLights();
    coolLights();
  }
  else {
    coolLights();
    coolLights();
    coolLights();
    coolLights();
  }
  return lidar_sensors[3].readRangeStatus() != VL6180X_ERROR_NONE || front_left > SENSOR_RANGE_MAX;
}

int wallRight() {
  if (lidar_sensors[2].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX) {
    redLights();
    redLights();
  }
  else {
    redLights();
    redLights();
    redLights();
    redLights();
  }
  return lidar_sensors[2].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX;
}

int wallFront(){
  ultrasonic = pulseIn(SONIC_ECHO1, HIGH) * ultrasonic_distance_factor;
  return ultrasonic > 254;
}

void updateSensors () {
  // Read the right LIDAR sensors and update their values
  back_right = lidar_sensors[1].readRange();
  back_right_errored = lidar_sensors[1].readRangeStatus() != VL6180X_ERROR_NONE || back_right > SENSOR_RANGE_MAX;
  front_right = lidar_sensors[2].readRange();
  front_right_errored = lidar_sensors[2].readRangeStatus() != VL6180X_ERROR_NONE || front_right > SENSOR_RANGE_MAX;

  // Read the left LIDAR sensors and update their values
  back_left = lidar_sensors[0].readRange();
  back_left_errored = lidar_sensors[0].readRangeStatus() != VL6180X_ERROR_NONE || back_left > SENSOR_RANGE_MAX;
  front_left = lidar_sensors[3].readRange();
  front_left_errored = lidar_sensors[3].readRangeStatus() != VL6180X_ERROR_NONE || front_left > SENSOR_RANGE_MAX;

  // Read front ultrasonic sensor
  digitalWrite(SONIC_TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONIC_TRIG1, LOW);
  ultrasonic = pulseIn(SONIC_ECHO1, HIGH) * ultrasonic_distance_factor;
  ultrasonic_errored = ultrasonic > 254;
  // And keep a running average
  if (!ultrasonic_errored) {
    ultrasonic_running_average = ultrasonic * ultrasonic_ave_factor + ultrasonic_running_average * (1 - ultrasonic_ave_factor);
  }

  // logf("Ultrasonic: %f (avg: %f); back_left (%d): %d, front_right (%d): %d, back_left (%d): %d, front_left (%d): %d\n", ultrasonic, ultrasonic_running_average, back_right_errored, back_right, front_right_errored, front_right, back_left_errored, back_left, front_left_errored, front_left);
}

// p_controller(80.0, currentAngle, 0, -127.0, 127.0);
double p_controller(double p, double current, double goal, double min, double max) {
  double out = (goal - current) * p;
  if (out > max) {
    out = max;
  }else if (out < min) {
    out = min;
  }
  return out;
}

/**
 * @return angle given by lidar sensors
 */
double getAngle()
{
  // arctan((lidarDistanceBL - lidarDistanceFL) / lidarSeparation);
  // Average left and right sensors
  double leftAngle = -atan2(front_left - back_left, LIDAR_SEPERATION);
  double rightAngle = atan2(front_right - back_right, LIDAR_SEPERATION);
  // logf("left angle: %f\tright angle: %f; ", leftAngle * 180.0 / PI, rightAngle * 180.0 / PI);

  if ((back_left_errored || front_left_errored) && (back_right_errored || front_right_errored)) {
    // If we have no good data, assume we're going straight
    // logf("Using 0 as angle\n");
    return 0;
  } else if (back_left_errored || front_left_errored) {
    // logf("Using right angle\n");
    return rightAngle;
  } else if (back_right_errored || front_right_errored) {
    // logf("Using left angle\n");
    return leftAngle;
  } else {
    // logf("Averaging angles\n");
    return (leftAngle + rightAngle) / 2;
  }
}

/**
 * Turn robot by a given angle in place
 *
 * @param angle     The angle to turn (in degrees)
 * @param direction The direction to turn (LEFT or RIGHT)
 */
void turn(double angle, turning_direction_t direction) {
  // Encoder to turn
  Encoder *turnEncoder;
  Encoder *otherTurnEncoder;

  // target point
  double target = angle * turnRatio;

  // Direction constant
  int dir = 1;

  if (direction == LEFT)
  {
    // Turn left
    turnEncoder = &rightEncoder;
    otherTurnEncoder = &leftEncoder;
  } else {
    // Turn right
    turnEncoder = &leftEncoder;
    otherTurnEncoder = &rightEncoder;
    dir = -1;
  }

  turnEncoder->write(0);
  otherTurnEncoder->write(0);

  // Turn right wheel backwards if left, forwards if right
  // Scale by 0.7 to compensate for over-volted motors
  setMotor(RIGHT_MOTOR, 45.125 * dir * 0.7);
  // Turn left wheel forwards if left, backwards if right
  setMotor(LEFT_MOTOR, -45.125 * dir * 0.7);

  // Turn until within margin of error

  int encoderAverage;
  do {
    encoderAverage = (turnEncoder->read() - otherTurnEncoder->read()) / 2;
  } while (encoderAverage < target - ANGLE_TOLERANCE);

  // Stop both motors
  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}

void movingTurn(double angle, turning_direction_t dir){
  Encoder *turnEncoder;
  Encoder *otherTurnEncoder;

  leftEncoder.write(0);
  rightEncoder.write(0);

  
  double leftTurnRatio = (SQUARE_SIZE + wheelSeparation) / 2.0 / wheelRadius / 360 * 190 * 10.5;
  double rightTurnRatio = (SQUARE_SIZE - wheelSeparation) / 2.0 / wheelRadius / 360 * 190 * 8.5;

  double target;

  double FAST_SPEED = 45.125 * (SQUARE_SIZE + wheelSeparation) / wheelSeparation * 0.35;
  double SLOW_SPEED = 45.125 * (SQUARE_SIZE - wheelSeparation) / wheelSeparation * 0.35;
  
  if(dir == LEFT){
    turnEncoder = &rightEncoder;
    target = angle * leftTurnRatio;
    // otherTurnEncoder = &leftEncoder;
    setMotor(RIGHT_MOTOR, FAST_SPEED);
    setMotor(LEFT_MOTOR, SLOW_SPEED);
  }
  else{
    turnEncoder = &rightEncoder;
    target = angle * rightTurnRatio;
    // otherTurnEncoder = &rightEncoder;
    setMotor(RIGHT_MOTOR, SLOW_SPEED);
    setMotor(LEFT_MOTOR, FAST_SPEED);
  }
  int encoderAverage = 0;
  while (turnEncoder->read() < target){}

  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
}

// Rotate 90 degrees right
void turnRight(){
  movingTurn(90.0 + getAngle() * 180.0 / PI, RIGHT);
}

// Rotate 90 degrees left
void turnLeft(){
  movingTurn(90.0 + getAngle() * 180.0 / PI, LEFT);
}

// Rotate 45 degrees right
void turnRight45(){
  turn(45.0 + getAngle() * 180.0 / PI, RIGHT);
}

// Rotate 45 degrees left
void turnLeft45(){
  turn(45.0 + getAngle() * 180.0 / PI, LEFT);
}

void turn180(){
  turn(180.0 + getAngle() * 180.0 / PI, LEFT);
}

// Moves the robot forward 1 square in the direction the robot is currently facing
int moveForward(int number) {
  // Reset encoders
  leftEncoder.write(0);
  rightEncoder.write(0);

  // Create angle variables
  double currentAngle, angularVelocity;

  // Create speed variables
  // (currentDistance is the distance inside the current square.)
  double currentDistance = 0;
  double goalDistance = SQUARE_SIZE * 10 * number;
  double velocity;

  // Center variables
  double centerVelocity, centerOffset;

  // loop quickly
  while (1) {
    // update currentDistance and currentAngle
    {
      // read LiDARs
      updateSensors();
      // Update angle
      currentAngle = getAngle();

      // Update current distance
      // 4560 ticks per revolution (380:1 gearbox * 12 ticks per rev normally)
      int ticksPerRevolution = 1750; // initially 4560
      // num revolutions * pi * diameter (Zach says 60mm)
      long leftRevs = leftEncoder.read();
      long rightRevs = rightEncoder.read();
      // ((Num ticks of both wheels / 2) / num ticks per revolution) * mm per revolution  
      currentDistance = (leftRevs + rightRevs) / 2.0 / ticksPerRevolution * PI * 60.0;
    }
    logf("Current: %lf\n", currentDistance);
    logf("Goal: %lf\n", goalDistance);

    // logf("Moving forward. current: %d, ultra: %d, goal: %d, cond: %d, %d\n", currentDistance, ultrasonic, goalDistance, ultrasonic < 150, ultrasonic > 95);

    // We're also not allowed to break out of the loop (stop going forward), if we're farther than 95 mm from a wall
    // If we think we're there, but we're not, go farther
    if (currentDistance >= goalDistance && ultrasonic < 150 && ultrasonic > 95) {
      logf("Moving goalDistance forward.\n");
      // Increase goal distance such that the ultrasonic ends up ULTRASONIC_FRONT (60mm) away from the wall in front of us
      goalDistance += ultrasonic - ULTRASONIC_FRONT;
    }

    // check if currentDistance and currentAngle are within tolerance
    // For the ultrasonic, 60 is 60 mm from the wall. This is about
    // the distance when the robot is centered in the tile
    if (currentDistance >= goalDistance || (!ultrasonic_errored && ultrasonic < ULTRASONIC_FRONT)) {
      setMotor(LEFT_MOTOR, 0);
      setMotor(RIGHT_MOTOR, 0);
      logf("Stopped. Ultrasonic: %d, %d, %lf\n", !ultrasonic_errored, ultrasonic < ULTRASONIC_FRONT, ultrasonic);
      return 0;
    }

    // How far away from the center we are
    // Right is positive
    // (ASCII art by Zach)
    // |              | <--  MAZE    |
    // |              |   CENTERLINE |
    // |              |              |
    // |      ERROR   |              |
    // |           \  |              |
    // |           |<>|              |
    // |       +-------+ <--- X ---> |
    // |       | FRONT |             |
    // |       |L  |  R|             |
    // |       |   |   |             |
    // |       | ROBOT |             |
    // |       +-------+             |
    // |       |<- R ->|             |
    // |              |              |
    // |              |              |
    // |              |              |
    // |              |              |
    // |              |              |
    // | <------ MAZE WIDTH -------> |
    //
    //              MAZE WIDTH - R
    // ERROR = X - ----------------
    //                    2
    centerOffset = (double)front_right - (double)front_left;
    if (front_left_errored && front_right_errored) {
        // logf("both errored, setting offset to 0\n");
        centerOffset = 0;
    }else if (front_left_errored) {
        // If we don't have a left value
        // (We're targeting to an offset of 0)
        // sensors are 84 mm apart, maze is 240mm wide
        centerOffset = (double)front_right - (240 - 84) / 2.0;
    }else if (front_right_errored) {
        centerOffset = (240 - 84) / 2.0 - (double)front_left;
    }

    // logf("left %d; right %d: center offset: %f\n", front_left, front_right, centerOffset);

    // p, current, goal, min, max
    // When the angle is 0.1, we need to bump left power by like 5, so P of 20
    // (A positive angle means that we're turned left)
    angularVelocity = p_controller(80.0, currentAngle, 0, -127.0, 127.0);

    // With a distance of 254 (one square), we've chose a P of 12.25
    //  so it saturates velocity for the majority of the distance
    velocity = p_controller(12.25, currentDistance, goalDistance, -64, 64);

    // With a center off set of 10mm, that's a velocity of 5
    centerVelocity = p_controller(0.5, centerOffset, 0, -50, 50);

    angularVelocity += centerVelocity;

    // NOTE: We're assuming that at angles close to 0, angularVelocity has a linear relationship with velocity.

    // update motor values
    // Scale by 0.7 to compensate for over-volted motors
    int velocityLeft = (int)(-angularVelocity / 2.0 * 0.7) + velocity;
    int velocityRight = (int)(angularVelocity / 2.0 * 0.7) + velocity;

    setMotor(LEFT_MOTOR, velocityLeft);
    setMotor(RIGHT_MOTOR, velocityRight);
  }
}

/* ---- SETUP ---- */
void setup(void) {
  // Debug led on the board itself
  pinMode(DEBUG_LED, OUTPUT);

  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(DEBUG_LED, HIGH);

  // Start serial
  Serial.begin(115200);
  logln("Serial ready!");

  // Starts I2C on the default pins (18 (SDA), 19 (SCL))
  // (I think, I can't find docs on it)
  I2C_LIDAR.begin();
  logln("I2C ready!");

  // Setup LiDARs
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    pinMode(lidar_cs_pins[i], OUTPUT);
  }
  // Disable all sensors except the first
  for (size_t i = 1; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], LOW);
  }

  // Set address for each sensor
  // Write the CS line high (turning it on)
  // Set the address
  for (size_t i = 0; i < LIDAR_COUNT; ++i) {
    digitalWrite(lidar_cs_pins[i], HIGH);
    // Pass pointer to the Wire2 object since we're running on I2C bus 2
    if (!lidar_sensors[i].begin(&I2C_LIDAR)) {
      log("Failed init on sensor ");
      logln(i);
    } else {
      lidar_sensors[i].setAddress(LIDAR_ADDR_BASE + i);
    }
  }
  logln("LiDAR sensors ready!");

  pinMode(SONIC_TRIG1, OUTPUT);
  pinMode(SONIC_ECHO1, INPUT);
  logln("Ultrasonic sensor ready!");

  // Setup motors
  pinMode(MOTORLEFT_1, OUTPUT);
  pinMode(MOTORLEFT_2, OUTPUT);
  pinMode(MOTORRIGHT_1, OUTPUT);
  pinMode(MOTORRIGHT_2, OUTPUT);

  setMotor(RIGHT_MOTOR, 0);
  setMotor(LEFT_MOTOR, 0);
  logln("Motors ready!");

  pinMode(START_BUTTON, INPUT);
  int t = 0;
  // Spin until start button is pressed
  // t is ms
  // On for 300 (0-300) off for 500 (300-800)
  while(!digitalRead(START_BUTTON)) {
    if (t == 0) {
      digitalWrite(YELLOW_LED, HIGH);
    }else if (t == 300) {
      digitalWrite(YELLOW_LED, LOW);
    }

    t = (t + 10) % 800;
    delay(10);
  }
  digitalWrite(YELLOW_LED, LOW);

  delay(20); // Switch "debounce"
  
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  long pressTime = millis();
  long endPressTime = pressTime;
  while (digitalRead(START_BUTTON)) {
    /* spin, waiting for button release */
    endPressTime = millis();
    if (endPressTime - pressTime > 1000) {
      digitalWrite(LED2, LOW);
    }
    if (endPressTime - pressTime > 3000) {
      digitalWrite(LED1, LOW);
    }
  }
  if (endPressTime - pressTime < 1000) {
  }else if (endPressTime - pressTime < 3000) {
  }else {
  }
  delay(500);
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  initialize();
  
}

void coolLights(){
  if(digitalRead(BLUE_LED) == LOW)
    digitalWrite(BLUE_LED, HIGH);
  else
    digitalWrite(BLUE_LED, LOW);
    delay(500);
}

void redLights(){
  if(digitalRead(LED3) == LOW)
    digitalWrite(LED3, HIGH);
  else
    digitalWrite(LED3, LOW);
    delay(500);
}
/* ---- MAIN ---- */
void testMotors() {
  int m1 = MOTORLEFT_1;
  int m2 = MOTORLEFT_2;
  for (int i = 0; i < 255; i = i + 10)
  {
    analogWrite(m1, i);
    analogWrite(m2, i);
    redLights();
  }
  
  while(1){
    coolLights();
  }
}

void loop(){
  /*
  digitalWrite(LED0, HIGH);
  delay(500);
  digitalWrite(LED0, LOW);
  delay(500);
  digitalWrite(LED0, HIGH);
  delay(500);
  digitalWrite(LED0, LOW);
  delay(500);
  // updateSensors();
  doRun();
  */
  wallLeft();
  wallRight();
  wallFront();
    // moveForward(1);
    // turnLeft();
    // coolLights();
    // coolLights();
    // while(!digitalRead(START_BUTTON)){}
    // moveForward(1);
    // turnRight();
    // coolLights();
    // coolLights();
    // while(!digitalRead(START_BUTTON)){}
}