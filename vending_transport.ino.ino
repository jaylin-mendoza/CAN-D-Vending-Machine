/*
 * Program: Vending Machine Servo
 * Description: This program controls a stepper motor using a CAN bus interface by listening for position
 *              commands via CAN bus and moves the stepper motor accordingly and dumps candy using a servo.
 * Authors: Kamdon Bird and Jaylin Mendoza
 * Last Updated: 11/26/2024 @ 5:31 AM
 */

#include "StepperAxis.cpp"    // Include custom library for stepper motor control
#include <Servo.h>            // Include servo library for bucket motor control 
#include <Adafruit_MCP2515.h> // Include library for MCP2515 CAN controller
#include <pins_arduino.h>     // Include Arduino standard pin definitions

// =========================== Constants and Pin Configuration ==========================
#define MOTOR_STEPS 200 // Number of steps per revolution for the stepper motor
#define MICROSTEPS 16   // Microstepping mode (e.g., 1/16 steps for smoother movement)
#define ENABLE_PIN 8    // Pin to enable stepper motor driver (low to enable motor)
#define HOME_X_PIN 9    // Pin connected to the home switch
#define DIR_X_PIN 5     // Pin controlling the direction of the stepper motor
#define STEP_X_PIN 2    // Pin controlling the stepping signal for the motor
#define SERVO_PIN 11    // Pin for controlling a servo motor using PWM signals

// CAN bus pin configuration
#define CS_PIN 13  // Chip Select pin for CAN controller (SPI communication)
#define SI_PIN 17  // Serial Input (MOSI) pin for SPI communication
#define SO_PIN 12  // Serial Output (MISO) pin for SPI communication
#define SCK_PIN 16 // Serial Clock pin for SPI communication

// CAN Bus Addresses
#define SCREEN_ADDRESS 1    // CAN address assigned to the LCD screen
#define DISPENSER_ADDRESS 3 // CAN address assigned to the item dispenser 
#define MY_CAN_ID 2         // Unique CAN address for this device

// ========================== Motor and CAN Object Initialization ========================
Adafruit_MCP2515 CAN(CS_PIN, SI_PIN, SO_PIN, SCK_PIN);            // CAN controller instance
StepperAxis axis(MOTOR_STEPS, DIR_X_PIN, STEP_X_PIN, HOME_X_PIN); // Stepper motor axis object
Servo bucket;                                                     // Create a Servo object to control the bucket

// =================== Predefined Position Values for the Stepper & Servo ================
const long positions[] = {36500, 28000, 17000, 8500}; // Position values for 4 different items
const int dropPosition = 0;                  		 		 // Position to drop items
const int delayPosition = 3000;                      // Delay to allow motor to reach position
const int completionValue = 5;                       // Value to signal completion
constexpr int servoDelay = 500;                      // Delay in milliseconds to allow the motor to complete its movement
int servoCurrentPos = 10;                            // Current angle of the servo
int servoHomePos = 10;                               // Default home angle for the servo
int servoMaxPos = 180;                               // Maximum angle for the servo
int servoSpeed = 6;                                  // Speed for servo movement (delay between angle changes)


// ================================ Setup Function =======================================
void setup() {
  Serial.begin(9600);                   // Initialize serial communication for debugging
  pinMode(ENABLE_PIN, OUTPUT);          // Set the enable pin as output to control the motor state

  // Initialize CAN bus with 500 kbps baud rate and check if successful
  if (!CAN.begin(500E3)) {                          
    while (1) delay(10);                // Stop execution if CAN bus fails to initialize
  }
  CAN.filter(MY_CAN_ID);                // Set the CAN bus filter for this device's address
  
  axis.init(300, MICROSTEPS);           // Initialize stepper motor with max current and microstepping mode
  axis.setHomeOffset(250);              // Set the home position offset (motor's zero position)
  axis.home();                          // Move motor to the home position (zero point)

  axis.setPositionIndex(0, 0);          // Set initial position index and value
  axis.setSpeed(10000);                 // Set the maximum speed of the stepper motor
  axis.setMotionProfile(1, 2500, 2500); // Configure motion profile (mode, acceleration, deceleration)

  bucket.attach(SERVO_PIN);             // Attach the servo to the specified pin
  bucket.write(servoHomePos);           // Ensure the servo is in its home position

  sendCANMessage(1, 5);
}

// =============================== Loop Function ========================================
void loop() {
  // Listen for CAN bus messages and act on them
  int packetSize = CAN.parsePacket();               // Check for incoming CAN packets
  if (packetSize) {                                 // If a packet is received
    Serial.println("[INFO] CAN Packet Received.");
    
    int packetContents[8];                          // 8 byte array, matching the CAN frame size limit.
    int i = 0;
    while (CAN.available()) {                       // Read packet contents
      packetContents[i] = CAN.read();
      i++;
    }
    readPacket(packetContents);   // Read the packet data into the array
    handlePacket(packetContents); // Handle the packet based on its contents
  }
}

// =============================== Helper Functions ====================================

// Reads the CAN packet and stores the data
void readPacket(int packetContents[]) {
  
  int maxSize = sizeof(packetContents) / sizeof(packetContents[0]);
  for (int i = 0; i < maxSize && CAN.available(); ++i) {
    packetContents[i] = CAN.read(); // Read data from CAN bus into the array
  }
}

// Handles the packet based on the received command
void handlePacket(int packetContents[]) {
  switch (packetContents[0]) {

    // If the packet is intended for the screen
    case SCREEN_ADDRESS: 
      if (packetContents[1] >= 1 && packetContents[1] <= 4) {
        moveToPosition(packetContents[1] - 1);
      } else if(packetContents[1] == 5) {
        sendCANMessage(1, 5);
      }
      
      else {
        Serial.println("[ERROR] Invalid position command.");
      }
      break;

    // If the packet is for the dispenser
    case DISPENSER_ADDRESS:  
      if (packetContents[1] == 1) {
        dropItem(); 
      } else {
        Serial.println("[ERROR] Unknown dispenser command.");
      }
      break;
      
    default:
      Serial.println("[ERROR] Unknown command.");
  }
}

// Moves the stepper motor to the requested position
void moveToPosition(int positionIndex) {
  if (positionIndex < 0 || positionIndex >= (4)) {
    Serial.println("[ERROR] Invalid position index.");
    return;
  }
  Serial.println(positionIndex);
  Serial.println("help");
  axis.moveABS(positions[positionIndex]);           // Move the stepper motor to the requested position
  //delay(delayPosition);                             // Wait for motor to reach the position
  sendCANMessage(DISPENSER_ADDRESS, positionIndex + 1); // Send position information to the dispenser via CAN bus
}

// Drops the item by moving the motor to the drop position
void dropItem() {
  axis.moveABS(dropPosition);      // Move the motor to the drop position
  delay(delayPosition);            // Wait for motor to reach the drop position
  dumpBucket();                    // Performs the bucket dump for the item
  CAN.beginPacket(SCREEN_ADDRESS); // Start building the CAN packet for the screen
  CAN.write(MY_CAN_ID);            // Include this device's unique ID in the packet
  CAN.write(completionValue);      // Include the completion value to indicate item was dropped
  CAN.endPacket();                 // Send the CAN packet to the screen
  // After sending a CAN packet, check if the operation was successful
  if (!CAN.endPacket()) {
    Serial.println("[ERROR] Failed to send CAN packet.");
  }
}

// Sends the current position to the given address via CAN
void sendCANMessage(int toAddress, int positionIndex) {
  CAN.beginPacket(toAddress); // Start building the CAN packet for the target address
  CAN.write(MY_CAN_ID);       // Add the device's unique ID to the packet
  CAN.write(positionIndex);   // Add the position index to the packet
  CAN.endPacket();            // Send the completed CAN packet
  // After sending a CAN packet, check if the operation was successful
  // if (!CAN.endPacket()) {
  //   Serial.println("[ERROR] Failed to send CAN packet.");
  // }
}

// Tilts the servo bucket to its maximum position and then returns to the home position
void dumpBucket() {
    tiltBucket(servoMaxPos);   // Tilt the bucket to the maximum position
    delay(servoDelay);         // Wait for the bucket to hold at the max position
    tiltBucket(servoHomePos);  // Return the bucket to the home position
}

// Moves the bucket to the specified target angle step by step
void tiltBucket(int targetAngle) {
  // Determine the step direction: positive for increasing angle, negative for decreasing
  int step = (targetAngle > servoCurrentPos) ? 1 : -1;
  
  // Gradually move the bucket from the current position to the target position
  for (int i = servoCurrentPos; i != targetAngle; i += step) {
    bucket.write(i);           // Update the servo position to the current step
    servoCurrentPos = i;       // Update the current position tracker
    delay(servoSpeed);         // Add a delay to control the movement speed
  }
  
  // Ensure the bucket reaches and remains at the exact target position
  bucket.write(targetAngle);   
  servoCurrentPos = targetAngle; // Update the current position to the target angle
}