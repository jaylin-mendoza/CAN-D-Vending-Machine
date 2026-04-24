#include <Adafruit_MCP2515.h>
#include "BasicStepperDriver.h"
#include "StepperAxis.cpp"
#include <Servo.h>

#define MICROSTEPS  16
#define MOTOR_STEPS 200
#define INCHES_PER_MOVE 2      // 2 inches between positions (to be adjusted maybe switch to mm)
#define STEPS_PER_INCH 100     // Example steps per inch (to be adjusted based on setup)

StepperAxis axis(MOTOR_STEPS, Pins::DIR_X, Pins::STEP_X, Pins::HOME_X);
Servo bucket;

int step = 0;
int servoCurrentPos = 10;
int servoHomePos = 10;
int servoMaxPos = 180;
int servoSpeed = 6;

void setup() {
  Serial.begin(115200);            // Initializes serial connection for console printing
  pinMode(8, OUTPUT);              // Initializes stepper driver enable to pin 8

  bucket.attach(11);               // Initializes servo signal wire to pin 11
  bucket.write(servoHomePos);      // Moves bucket to park position

  axis.setName("Axis 1");          // Sets string name of axis for console printing
  axis.init(300, MICROSTEPS);      // Initializes axis with starting speed of 300rpm and specified microstepping mode
  axis.setHomeOffset(250);         // Sets offset to drive to after homing
  axis.home();                     // Homes the axis
  axis.printPosition();            // Prints the current axis position to console
}

void loop() {
  axis.setSpeed(800);                           // Sets the axis speed
  axis.setMotionProfile(1, 2500, 2500);         // Sets accel/decel values for the axis

  // Move to the first position (2 inches to the right)
  axis.moveABS(STEPS_PER_INCH * INCHES_PER_MOVE);  // Moves to position 1 (2 inches)
  axis.printPosition();                           // Print position

  delay(1000);  // Delay to observe

  // Move to the second position (another 2 inches to the right, total 8 inches from home)
  axis.moveABS(STEPS_PER_INCH * INCHES_PER_MOVE * 2);  // Moves to position 2 (4 inches)
  axis.printPosition();                            // Print position

  delay(1000);  // Delay to observe

  // Move back to home position (optional)
  axis.moveABS(0);
  axis.printPosition();

  Serial.println("Done.");
  while (1) {}  // Infinite loop to stop the program
}
