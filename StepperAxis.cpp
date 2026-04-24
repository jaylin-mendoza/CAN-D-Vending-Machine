#include <Adafruit_MCP2515.h>

/**
 * StepperAxis class
 * @author Chris Darnell
 *
 * Wrapper class for {{ BasicStepperDriver.h }}. Uses the StepperDriver library by Laurentiu Badea (v1.4.1).
 * Adds additional functionalities, e.g. home method, absolute position tracking, programmed positioning, etc. 
 *
 * Pin assignments for Arduino Uno CNC Shield 3.0 and A4988 stepper motor drivers.
 *
*/

#include <Arduino.h>
#include <BasicStepperDriver.h>

/**
 * Enumeration for pin assignments of the Aruduino Uno CNC Shield and A4988 stepper motor drivers.
*/
enum Pins {
  DIR_X = 5,
  DIR_Y = 6,
  DIR_Z = 7,
  STEP_X = 2,
  STEP_Y = 3,
  STEP_Z = 4,
  HOME_X = 9,
  HOME_Y = 10,
  HOME_Z = 11,
  STEP_ENABLE = 8
};

class StepperAxis : private BasicStepperDriver {

private:

  short homePin;
  short homeSpeed = 30;
  short homeOffset = 50;
  short homeDirection = 1;
  long currentPos = 0;
  long positions[10];
  String name;

public:

  /**
  * Constructor - creates a new stepper motor axis.
  *
  * @param MOTOR_STEPS The number of steps/revolution - usually 200.
  * @param DIR The pin assignment that controls the direction of the motor.
  * @param STEP The pin assignment that controls the motor stepping function.
  * @param homePin The pin assignment that the home/limit switch is attached t0.
  */
  StepperAxis::StepperAxis(short MOTOR_STEPS, short DIR, short STEP, short homePin)
    : BasicStepperDriver(MOTOR_STEPS, DIR, STEP) {

    this->homePin = homePin;
    this->setSpeedProfile(StepperAxis::LINEAR_SPEED, 1000, 1000);
  }

  /**
  * Initializes this stepper motor axis.
  * Configures home switch as active-low.
  *
  * @param rpm The initial speed in rotations/min 
  * @param microsteps The microstepping mode (1, 2, 4, 8, 16)
  */
  void init(float rpm, short microsteps = 1) {
    pinMode(homePin, INPUT_PULLUP);
    this->begin(rpm, microsteps);
  }

  /**
  * Homing sequence for this stepper axis.
  * Sets current position tracking to 0.
  * Use setHomeDirection() if homing backwards.
  */
  void home() {

    /* Set homing speed and drive to home switch */
    setRPM(homeSpeed);
    startRotate(100 * 360 * homeDirection);
    do {
      if (digitalRead(homePin) == LOW) {
        stop();
        break;
      }
      nextAction();
    } while (digitalRead(homePin) == HIGH);

    /* Set slower speeed and drive off of home switch */
    delay(500);
    setRPM(homeSpeed / 8);
    startRotate(-100 * 360 * homeDirection);
    do {
      if (digitalRead(homePin) == HIGH) {
        stop();
        break;
      }
      nextAction();
    } while (digitalRead(homePin) == LOW);

    /* Set creeping speed and drive back to home switch */
    delay(500);
    setRPM(homeSpeed / 16);
    startRotate(100 * 360 * homeDirection);
    do {
      if (digitalRead(homePin) == LOW) {
        stop();
        break;
      }
      nextAction();
    } while (digitalRead(homePin) == HIGH);

    /* Drive to offset off of home switch and zero current position */
    delay(500);
    setRPM(homeSpeed);
    move(homeOffset);
    currentPos = 0;
    delay(500);

  }

  /**
  * Sets the homing direction for this axis.
  *
  * @param dir The direction to use for homing. (1 = default, 0 = reverse).
  */
  void setHomeDirection(bool dir = 1) {
    homeDirection = (dir == 1) ? 1 : -1;
  }

  /**
  * Sets the zero offset to apply after homing.
  *
  * @param offset The offset in motor steps.
  */
  void setHomeOffset(long offset) {
    this->homeOffset = offset;
  }

  /**
  * Returns the current position of this axis.
  *
  * @return The current tracked position.
  */
  long getCurrentPosition() {
    return currentPos;
  }

  /**
  * Sets the speed to use for this axis
  *
  * @param rpm The motor speed in rotations/minute
  */
  void setSpeed(float rpm) {
    this->setRPM(rpm);
  }

  /**
  * Sets acceleration / deceleration profile for this axis
  *
  * @param profile The mode to apply (0 = constant, 1 = linear).
  * @param accel The acceleration speed to apply.
  * @param decel The deceleration speed to apply.
  */
  void setMotionProfile(int profile, int accel, int decel) {
    enum StepperAxis::Mode mode = (profile == 1) ? StepperAxis::Mode::LINEAR_SPEED : StepperAxis::Mode::CONSTANT_SPEED;
    this->setSpeedProfile(mode, accel, decel);
  }

  /**
  * Moves the axis incrementally relative to it's current position.
  *
  * @param pos The number of steps to move (positive = forward, neg = backwards).
  */
  void moveINC(long pos) {
    this->move(pos);
    this->currentPos = currentPos + pos;
  }

  /**
  * Moves the axis to an absolute position relative to zero (home).
  *
  * @param pos The position in steps to move to.
  */
  void moveABS(long pos) {
    pos = (pos < 0) ? 0 : pos;  // truncate negative position values
    this->move(pos - currentPos);
    currentPos = pos;
  }

  /**
  * Moves the axis to a position previously stored with setPositionIndex().
  *
  * @param idx The stored position index to move to.
  */
  void moveIDX(short idx) {
    long pos = (positions[idx] - currentPos);
    this->move(pos);
    currentPos = positions[idx];
  }

  /**
  * Stores a position at a given index for later motion instructions.
  *
  * @param idx The index of the position to store (0-10).
  * @param value The position in motor steps to store.
  */
  void setPositionIndex(short idx, long value) {
    positions[idx] = value;
  }

  /**
  * Gets the position stored at a given index in motor steps.
  *
  * @param idx The index of the position to retreive.
  * @return The position in motor steps.
  */
  long getPositionIndex(short idx) {
    return positions[idx];
  }

  /**
  * Prints the current position of this axis.
  */
  void printPosition() {
    char s[80];
    sprintf(s, " Current position: %d", currentPos);
    Serial.print(name);
    Serial.println(s);
  }

  /**
  * Sets the name of this axis.
  *
  * @param val The string name of this axis.
  */
  void setName(String val) {
    name = val;
  }

  /**
  * Gets the name of this axis.
  *
  * @return The string name of this axis.
  */
  String getName() {
    return name;
  }
  
};