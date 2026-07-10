/*************************************************************
   Integrated Micromouse: Enhanced PID Movement + Flood-Fill
   Features:
   - Multiple VL53L0X sensors with XSHUT switching
   - Enhanced PID control for straight movement
   - Flood-fill maze solving algorithm
   - Proper turn handling for 20x20cm cells
   - Fixed I2C conflicts and MPU6050 initialization
   - Updated for 8x8 maze
*************************************************************/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <MPU6050.h>

// ------------------------------------------------------------------------
//                          Pin Definitions & Constants
// ------------------------------------------------------------------------

// Motor pins
#define IN1 16  // Left Motor Direction 1
#define IN2 17  // Left Motor Direction 2
#define IN3 18  // Right Motor Direction 1
#define IN4 19  // Right Motor Direction 2
#define ENA 32  // Left Motor PWM
#define ENB 33  // Right Motor PWM

// Encoder pins
#define ENCODER_L_A 34
#define ENCODER_L_B 35
#define ENCODER_R_A 14
#define ENCODER_R_B 12

// VL53L0X XSHUT pins for sensor switching
#define XSHUT_FRONT  25   // Front sensor shutdown pin
#define XSHUT_LEFT   26   // Left sensor shutdown pin  
#define XSHUT_RIGHT  27   // Right sensor shutdown pin

// LED pin for debugging
#define LED_PIN 2

// I2C addresses for the three sensors
#define FRONT_SENSOR_ADDRESS  0x30
#define LEFT_SENSOR_ADDRESS   0x31
#define RIGHT_SENSOR_ADDRESS  0x32

// MPU6050 default address
#define MPU6050_ADDRESS 0x68

// Enhanced PID constants for straight movement
#define KP_STRAIGHT 1.0
#define KI_STRAIGHT 0.05
#define KD_STRAIGHT 0.2

// PID constants for turning
#define KP_TURN 15
#define KI_TURN 0.05
#define KD_TURN 3

// Motor speeds
#define BASE_SPEED_LEFT  80
#define BASE_SPEED_RIGHT 85
#define MOTOR_NOMINAL    80
#define MOTOR_DELTAMAX   45

// Control timing
#define CONTROL_PERIOD 150
#define MAX_CORRECTION 20
#define MAX_INTEGRAL 50

// Maze constants - Updated for 8x8 maze
#define MAZE_SIZE 8

// Distance thresholds for walls (in mm) - adjusted for 20cm cells with robot centered
#define WALL_MIN_DISTANCE  15   // Minimum distance to detect wall
#define WALL_MAX_DISTANCE  100  // Maximum distance for wall detection (front walls)
#define SIDE_WALL_DISTANCE 100  // Expected distance to side walls when centered (10cm from center to wall)

// Center navigation constants - keep robot 3cm from walls (30mm)
#define TARGET_SIDE_DISTANCE 30  // 3cm from walls to keep robot centered
#define WALL_FOLLOW_TOLERANCE 5  // Tolerance for centering (±5mm)
#define CENTER_CORRECTION_SPEED 15 // Speed adjustment for centering

// Movement constants - adjusted for 20cm cells
const long TARGET_FORWARD_COUNT = 450; // Adjusted for exact 20cm movement

// Turn timing constants - reduced for more precise turns
#define TURN_90_TIME 600    // milliseconds for exact 90-degree turn (reduced)
#define TURN_180_TIME 1200  // milliseconds for exact 180-degree turn (reduced)
#define TURN_SPEED 85       // Reduced turning speed for more control

// Turn settling time
#define TURN_SETTLE_TIME 300 // Time to settle after turn

// I2C timing constants
#define I2C_SENSOR_DELAY 50   // Delay between sensor readings
#define I2C_INIT_DELAY 200    // Delay during initialization

// ------------------------------------------------------------------------
//                         Data Structures
// ------------------------------------------------------------------------

typedef struct {
  short int x;
  short int y;
} Cell;

typedef enum {
  FRONT = 0,
  LEFT  = 1,
  BEHIND= 2,
  RIGHT = 3
} Heading;

typedef struct {
  short int x;
  short int y;
  Heading heading;
} Mouse;

typedef struct Node {
  Cell cell;
  struct Node* next;
} Node;

typedef struct {
  Node* front;
  Node* rear;
} Queue;

// ------------------------------------------------------------------------
//                         Global Variables
// ------------------------------------------------------------------------

// Sensor objects
Adafruit_VL53L0X frontSensor = Adafruit_VL53L0X();
Adafruit_VL53L0X leftSensor = Adafruit_VL53L0X();
Adafruit_VL53L0X rightSensor = Adafruit_VL53L0X();
MPU6050 mpu;

// Sensor status flags
bool frontSensorActive = false;
bool leftSensorActive = false;
bool rightSensorActive = false;
bool mpuActive = false;

// Encoder counts
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
long prevEncoderLeft = 0;
long prevEncoderRight = 0;

// PID variables for straight movement
float speedDifference = 0;
float prevSpeedDifference = 0;
float integralErrorStraight = 0;

// PID variables for turning
float pInput, iInput, dInput;
float prevTurnError = 0;
float targetYaw;

// Current motor speeds
int currentLeftSpeed = BASE_SPEED_LEFT;
int currentRightSpeed = BASE_SPEED_RIGHT;

// Flood-fill arrays - Updated for 8x8 maze
byte dp_matrix[MAZE_SIZE][MAZE_SIZE];
byte vertical_barrier[MAZE_SIZE - 1][MAZE_SIZE]   = { 0 };
byte horizontal_barrier[MAZE_SIZE][MAZE_SIZE - 1] = { 0 };

// Mouse position and heading
Mouse mouse = { 0, 0, FRONT};

// ------------------------------------------------------------------------
//                    Function Declarations
// ------------------------------------------------------------------------

// Setup functions
void setupMotors();
void setupEncoders();
void setupSensors();
void setupMPU6050();
void scanI2C();
bool checkI2CDevice(uint8_t address);

// Motor control
void setLeftMotor(int speed);
void setRightMotor(int speed);
void stopMotors();
void moveForward();
void moveForwardWithPID();
void moveForwardCentered();
void turnRight();
void turnLeft();
void turn180();
void moveForwardOneCell();
void motorTest();
void centerRobot();
bool isRobotCentered();

// Sensor functions
void enableSensor(int xshutPin);
void disableSensor(int xshutPin);
void disableAllSensors();
int readFrontDistance();
int readLeftDistance();
int readRightDistance();
bool wallFront();
bool wallLeft();
bool wallRight();
void resetI2CBus();

// Encoder interrupts
void encoderISR_L_A();
void encoderISR_R_A();

// PID functions
void pidBeginStraight();
void pidBeginTurn();
int pidStraight(float error);
int pidTurn(float error);

// MPU6050 functions
float getMPUYaw();
bool initializeMPU6050();

// Maze solving functions
void fillDpMatrix();
void floodFill();
void updateWalls();
void getNeighbors(short int x, short int y, Cell* cells, int* size);
Cell getMinCell(Cell cells[], int size);
void changeDirection(Cell minCell);
void updateHeadingAfterTurn(int turnType);
void solveMaze();

// Queue functions
void enqueue(Queue* queue, Cell cell);
Cell dequeue(Queue* queue);
int isQueueEmpty(Queue* queue);

// ------------------------------------------------------------------------
//                               setup()
// ------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("=== INTEGRATED MICROMOUSE WITH FLOOD-FILL (8x8 MAZE) ===");
  
  // Initialize I2C with slower speed for better reliability
  Wire.begin(21, 22);
  Wire.setClock(50000);  // Reduced to 50kHz for better reliability
  delay(1000);
  
  setupMotors();
  setupEncoders();
  
  // Scan I2C bus first
  Serial.println("Scanning I2C bus...");
  scanI2C();
  
  // Setup sensors with improved error handling
  setupSensors();
  
  // Initialize MPU6050 with better conflict handling
  setupMPU6050();
  
  // Initialize maze flood-fill for 8x8 maze
  fillDpMatrix();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Setup completed!");
  Serial.print("Active sensors - Front: ");
  Serial.print(frontSensorActive ? "OK" : "FAIL");
  Serial.print(", Left: ");
  Serial.print(leftSensorActive ? "OK" : "FAIL");
  Serial.print(", Right: ");
  Serial.print(rightSensorActive ? "OK" : "FAIL");
  Serial.print(", MPU6050: ");
  Serial.println(mpuActive ? "OK" : "FAIL");
  
  // ENABLE THIS LINE TO TEST MOTORS FIRST:
  // motorTest();
  
  Serial.println("Starting flood-fill maze solving...");
  delay(2000);
}

// ------------------------------------------------------------------------
//                               loop()
// ------------------------------------------------------------------------
void loop() {
  solveMaze();
}

// ------------------------------------------------------------------------
//                         Setup Functions
// ------------------------------------------------------------------------
void setupMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  stopMotors();
  Serial.println("Motors initialized");
}

void setupEncoders() {
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderISR_L_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderISR_R_A, RISING);
  
  Serial.println("Encoders initialized");
}

void setupSensors() {
  Serial.println("Setting up VL53L0X sensors...");
  
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  
  disableAllSensors();
  delay(500);
  
  Serial.println("Attempting to initialize sensors sequentially...");
  
  // Initialize front sensor
  enableSensor(XSHUT_FRONT);
  delay(I2C_INIT_DELAY);
  if (frontSensor.begin(FRONT_SENSOR_ADDRESS)) {
    Serial.println("Front sensor initialized successfully");
    frontSensorActive = true;
  } else {
    Serial.println("Failed to initialize front sensor!");
    frontSensorActive = false;
  }
  
  // Initialize left sensor
  enableSensor(XSHUT_LEFT);
  delay(I2C_INIT_DELAY);
  if (leftSensor.begin(LEFT_SENSOR_ADDRESS)) {
    Serial.println("Left sensor initialized successfully");
    leftSensorActive = true;
  } else {
    Serial.println("Failed to initialize left sensor!");
    leftSensorActive = false;
  }
  
  // Initialize right sensor
  enableSensor(XSHUT_RIGHT);
  delay(I2C_INIT_DELAY);
  if (rightSensor.begin(RIGHT_SENSOR_ADDRESS)) {
    Serial.println("Right sensor initialized successfully");
    rightSensorActive = true;
  } else {
    Serial.println("Failed to initialize right sensor!");
    rightSensorActive = false;
  }
  
  Serial.println("Sensor setup completed");
}

void setupMPU6050() {
  Serial.println("Setting up MPU6050...");
  
  // Check if MPU6050 is present on I2C bus
  if (!checkI2CDevice(MPU6050_ADDRESS)) {
    Serial.println("MPU6050 not found on I2C bus!");
    mpuActive = false;
    return;
  }
  
  // Reset I2C bus before MPU6050 initialization
  resetI2CBus();
  delay(100);
  
  // Initialize MPU6050
  if (initializeMPU6050()) {
    Serial.println("MPU6050 initialized successfully");
    mpuActive = true;
  } else {
    Serial.println("Failed to initialize MPU6050!");
    mpuActive = false;
  }
}

bool initializeMPU6050() {
  // Try multiple times with different approaches
  for (int attempt = 0; attempt < 3; attempt++) {
    Serial.print("MPU6050 initialization attempt ");
    Serial.println(attempt + 1);
    
    mpu.initialize();
    delay(100);
    
    if (mpu.testConnection()) {
      Serial.println("MPU6050 connection test passed");
      return true;
    }
    
    delay(200);
    resetI2CBus();
    delay(100);
  }
  
  return false;
}

void resetI2CBus() {
  Wire.end();
  delay(50);
  Wire.begin(21, 22);
  Wire.setClock(50000);
  delay(50);
}

bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return (error == 0);
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning for I2C devices...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" I2C devices");
  }
  Serial.println();
}

// ------------------------------------------------------------------------
//                         Motor Control Functions
// ------------------------------------------------------------------------
void setLeftMotor(int speed) {
  if (speed == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(speed));
  }
}

void setRightMotor(int speed) {
  if (speed == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  } else if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(speed));
  }
}

void stopMotors() {
  Serial.println("=== STOPPING ALL MOTORS ===");
  setLeftMotor(0);
  setRightMotor(0);
  delay(100);
}

void moveForward() {
  Serial.println("=== MOVING FORWARD ===");
  setLeftMotor(80);
  setRightMotor(80);
}

void turnLeft() {
  Serial.println("=== TURNING LEFT 90 DEGREES ===");
  stopMotors();
  delay(TURN_SETTLE_TIME);
  
  // Check if we have walls to avoid spinning
  bool leftWallBefore = wallLeft();
  bool frontWallBefore = wallFront();
  
  Serial.print("Before turn - Left wall: ");
  Serial.print(leftWallBefore ? "YES" : "NO");
  Serial.print(", Front wall: ");
  Serial.println(frontWallBefore ? "YES" : "NO");
  
  // Perform the turn with reduced speed
  setLeftMotor(-TURN_SPEED);
  setRightMotor(TURN_SPEED);
  
  delay(TURN_90_TIME);
  stopMotors();
  delay(TURN_SETTLE_TIME);
  
  // Verify the turn was successful
  bool leftWallAfter = wallLeft();
  bool frontWallAfter = wallFront();
  
  Serial.print("After turn - Left wall: ");
  Serial.print(leftWallAfter ? "YES" : "NO");
  Serial.print(", Front wall: ");
  Serial.println(frontWallAfter ? "YES" : "NO");
  
  // If turn seems incorrect, try to correct
  if (leftWallBefore == leftWallAfter && frontWallBefore == frontWallAfter) {
    Serial.println("Turn may be incomplete, adjusting...");
    setLeftMotor(-TURN_SPEED);
    setRightMotor(TURN_SPEED);
    delay(TURN_90_TIME / 4); // Small correction
    stopMotors();
    delay(TURN_SETTLE_TIME);
  }
  
  Serial.println("Left turn completed");
}

void turnRight() {
  Serial.println("=== TURNING RIGHT 90 DEGREES ===");
  stopMotors();
  delay(TURN_SETTLE_TIME);
  
  // Check if we have walls to avoid spinning
  bool rightWallBefore = wallRight();
  bool frontWallBefore = wallFront();
  
  Serial.print("Before turn - Right wall: ");
  Serial.print(rightWallBefore ? "YES" : "NO");
  Serial.print(", Front wall: ");
  Serial.println(frontWallBefore ? "YES" : "NO");
  
  // Perform the turn with reduced speed
  setLeftMotor(TURN_SPEED);
  setRightMotor(-TURN_SPEED);
  
  delay(TURN_90_TIME);
  stopMotors();
  delay(TURN_SETTLE_TIME);
  
  // Verify the turn was successful
  bool rightWallAfter = wallRight();
  bool frontWallAfter = wallFront();
  
  Serial.print("After turn - Right wall: ");
  Serial.print(rightWallAfter ? "YES" : "NO");
  Serial.print(", Front wall: ");
  Serial.println(frontWallAfter ? "YES" : "NO");
  
  // If turn seems incorrect, try to correct
  if (rightWallBefore == rightWallAfter && frontWallBefore == frontWallAfter) {
    Serial.println("Turn may be incomplete, adjusting...");
    setLeftMotor(TURN_SPEED);
    setRightMotor(-TURN_SPEED);
    delay(TURN_90_TIME / 4); // Small correction
    stopMotors();
    delay(TURN_SETTLE_TIME);
  }
  
  Serial.println("Right turn completed");
}

void turn180() {
  Serial.println("=== TURNING 180 DEGREES (U-TURN) ===");
  stopMotors();
  delay(TURN_SETTLE_TIME);
  
  // Perform the 180-degree turn
  setLeftMotor(TURN_SPEED);
  setRightMotor(-TURN_SPEED);
  
  delay(TURN_180_TIME);
  stopMotors();
  delay(TURN_SETTLE_TIME);
  
  Serial.println("180-degree turn completed");
}

void moveForwardOneCell() {
  Serial.println("=== MOVING FORWARD ONE CELL ===");
  
  // First, try to center the robot
  centerRobot();
  
  encoderCountLeft = 0;
  encoderCountRight = 0;
  
  // Move forward with centering control
  moveForwardCentered();
  
  while (encoderCountLeft < TARGET_FORWARD_COUNT && encoderCountRight < TARGET_FORWARD_COUNT) {
    if (frontSensorActive) {
      int frontDist = readFrontDistance();
      if (frontDist < 80) {
        Serial.println("EMERGENCY STOP - Too close to wall!");
        break;
      }
    }
    
    // Continue centered movement
    moveForwardCentered();
    delay(50);
  }
  
  stopMotors();
  Serial.print("Moved - Left encoder: ");
  Serial.print(encoderCountLeft);
  Serial.print(", Right encoder: ");
  Serial.println(encoderCountRight);
  
  // Try to center again after movement
  centerRobot();
  delay(300);
}

void moveForwardCentered() {
  int leftDist = readLeftDistance();
  int rightDist = readRightDistance();
  
  int baseSpeedLeft = BASE_SPEED_LEFT;
  int baseSpeedRight = BASE_SPEED_RIGHT;
  
  // Calculate centering correction
  int correction = 0;
  
  if (leftSensorActive && rightSensorActive && 
      leftDist < SIDE_WALL_DISTANCE && rightDist < SIDE_WALL_DISTANCE) {
    
    // Both walls detected - use them for centering
    int centerError = leftDist - rightDist;
    correction = centerError * 0.3; // Gentle correction
    
    Serial.print("Centering - Left: ");
    Serial.print(leftDist);
    Serial.print("mm, Right: ");
    Serial.print(rightDist);
    Serial.print("mm, Error: ");
    Serial.print(centerError);
    Serial.print("mm, Correction: ");
    Serial.println(correction);
    
  } else if (leftSensorActive && leftDist < SIDE_WALL_DISTANCE) {
    // Only left wall - maintain distance
    int wallError = leftDist - TARGET_SIDE_DISTANCE;
    correction = -wallError * 0.4;
    
    Serial.print("Left wall follow - Distance: ");
    Serial.print(leftDist);
    Serial.print("mm, Target: ");
    Serial.print(TARGET_SIDE_DISTANCE);
    Serial.print("mm, Correction: ");
    Serial.println(correction);
    
  } else if (rightSensorActive && rightDist < SIDE_WALL_DISTANCE) {
    // Only right wall - maintain distance
    int wallError = rightDist - TARGET_SIDE_DISTANCE;
    correction = wallError * 0.4;
    
    Serial.print("Right wall follow - Distance: ");
    Serial.print(rightDist);
    Serial.print("mm, Target: ");
    Serial.print(TARGET_SIDE_DISTANCE);
    Serial.print("mm, Correction: ");
    Serial.println(correction);
  }
  
  // Limit correction
  correction = constrain(correction, -CENTER_CORRECTION_SPEED, CENTER_CORRECTION_SPEED);
  
  // Apply correction
  baseSpeedLeft -= correction;
  baseSpeedRight += correction;
  
  // Ensure speeds are within limits
  baseSpeedLeft = constrain(baseSpeedLeft, 40, 120);
  baseSpeedRight = constrain(baseSpeedRight, 40, 120);
  
  setLeftMotor(baseSpeedLeft);
  setRightMotor(baseSpeedRight);
}

void centerRobot() {
  Serial.println("=== CENTERING ROBOT ===");
  
  if (!isRobotCentered()) {
    Serial.println("Robot not centered, adjusting...");
    
    for (int attempts = 0; attempts < 5; attempts++) {
      int leftDist = readLeftDistance();
      int rightDist = readRightDistance();
      
      if (leftSensorActive && rightSensorActive && 
          leftDist < SIDE_WALL_DISTANCE && rightDist < SIDE_WALL_DISTANCE) {
        
        int centerError = leftDist - rightDist;
        
        if (abs(centerError) <= WALL_FOLLOW_TOLERANCE) {
          Serial.println("Robot is now centered");
          break;
        }
        
        Serial.print("Center error: ");
        Serial.print(centerError);
        Serial.println("mm");
        
        // Small adjustment movement
        if (centerError > WALL_FOLLOW_TOLERANCE) {
          // Too close to left wall, move right
          setLeftMotor(60);
          setRightMotor(40);
          delay(100);
        } else if (centerError < -WALL_FOLLOW_TOLERANCE) {
          // Too close to right wall, move left
          setLeftMotor(40);
          setRightMotor(60);
          delay(100);
        }
        
        stopMotors();
        delay(200);
      } else {
        Serial.println("Cannot center - insufficient wall detection");
        break;
      }
    }
  }
  
  stopMotors();
}

bool isRobotCentered() {
  int leftDist = readLeftDistance();
  int rightDist = readRightDistance();
  
  if (leftSensorActive && rightSensorActive && 
      leftDist < SIDE_WALL_DISTANCE && rightDist < SIDE_WALL_DISTANCE) {
    
    int centerError = abs(leftDist - rightDist);
    return (centerError <= WALL_FOLLOW_TOLERANCE);
  }
  
  return true; // Assume centered if we can't measure
}

void motorTest() {
  Serial.println("=== COMPREHENSIVE MOTOR TEST ===");
  
  Serial.println("\n1. Testing STOP:");
  stopMotors();
  delay(2000);
  
  Serial.println("\n2. Testing FORWARD:");
  moveForward();
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\n3. Testing LEFT TURN:");
  turnLeft();
  delay(1000);
  
  Serial.println("\n4. Testing RIGHT TURN:");
  turnRight();
  delay(1000);
  
  Serial.println("\n5. Testing 180 TURN:");
  turn180();
  delay(1000);
  
  Serial.println("=== MOTOR TEST COMPLETED ===");
}

// ------------------------------------------------------------------------
//                         Sensor Functions
// ------------------------------------------------------------------------
void enableSensor(int xshutPin) {
  digitalWrite(xshutPin, HIGH);
}

void disableSensor(int xshutPin) {
  digitalWrite(xshutPin, LOW);
}

void disableAllSensors() {
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
}

int readFrontDistance() {
  if (!frontSensorActive) return 999;
  
  VL53L0X_RangingMeasurementData_t measure;
  frontSensor.rangingTest(&measure, false);
  delay(I2C_SENSOR_DELAY);
  
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  }
  return 999;
}

int readLeftDistance() {
  if (!leftSensorActive) return 999;
  
  VL53L0X_RangingMeasurementData_t measure;
  leftSensor.rangingTest(&measure, false);
  delay(I2C_SENSOR_DELAY);
  
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  }
  return 999;
}

int readRightDistance() {
  if (!rightSensorActive) return 999;
  
  VL53L0X_RangingMeasurementData_t measure;
  rightSensor.rangingTest(&measure, false);
  delay(I2C_SENSOR_DELAY);
  
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  }
  return 999;
}

bool wallFront() {
  int dist = readFrontDistance();
  bool hasWall = (dist > WALL_MIN_DISTANCE && dist < WALL_MAX_DISTANCE);
  Serial.print("Front distance: ");
  Serial.print(dist);
  Serial.print(" mm, Wall: ");
  Serial.println(hasWall ? "YES" : "NO");
  return hasWall;
}

bool wallLeft() {
  int dist = readLeftDistance();
  bool hasWall = (dist > WALL_MIN_DISTANCE && dist < SIDE_WALL_DISTANCE);
  Serial.print("Left distance: ");
  Serial.print(dist);
  Serial.print(" mm, Wall: ");
  Serial.println(hasWall ? "YES" : "NO");
  return hasWall;
}

bool wallRight() {
  int dist = readRightDistance();
  bool hasWall = (dist > WALL_MIN_DISTANCE && dist < SIDE_WALL_DISTANCE);
  Serial.print("Right distance: ");
  Serial.print(dist);
  Serial.print(" mm, Wall: ");
  Serial.println(hasWall ? "YES" : "NO");
  return hasWall;
}

// ------------------------------------------------------------------------
//                         Encoder Interrupts
// ------------------------------------------------------------------------
void encoderISR_L_A() {
  encoderCountLeft++;
}

void encoderISR_R_A() {
  encoderCountRight++;
}

// ------------------------------------------------------------------------
//                         PID Functions
// ------------------------------------------------------------------------
void pidBeginStraight() {
  integralErrorStraight = 0;
  prevSpeedDifference = 0;
}

void pidBeginTurn() {
  iInput = 0;
  prevTurnError = 0;
}

int pidStraight(float error) {
  integralErrorStraight += error;
  integralErrorStraight = constrain(integralErrorStraight, -MAX_INTEGRAL, MAX_INTEGRAL);
  
  float derivative = error - prevSpeedDifference;
  prevSpeedDifference = error;
  
  return (KP_STRAIGHT * error) + (KI_STRAIGHT * integralErrorStraight) + (KD_STRAIGHT * derivative);
}

int pidTurn(float error) {
  pInput = error;
  iInput = constrain(iInput + error, -50, +50);
  dInput = error - prevTurnError;
  prevTurnError = error;
  
  return pInput * KP_TURN + iInput * KI_TURN + dInput * KD_TURN;
}

// ------------------------------------------------------------------------
//                         MPU6050 Functions
// ------------------------------------------------------------------------
float getMPUYaw() {
  if (!mpuActive) {
    return 0.0;
  }
  
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  
  float gyroZ = gz / 131.0;
  
  static float yaw = 0;
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  
  if (lastTime != 0) {
    float dt = (currentTime - lastTime) / 1000.0;
    yaw += gyroZ * dt;
  }
  lastTime = currentTime;
  
  return yaw;
}

// ------------------------------------------------------------------------
//                    Flood-Fill Maze Solving Functions
// ------------------------------------------------------------------------
void fillDpMatrix() {
  // For 8x8 maze, center is at (3,3), (3,4), (4,3), (4,4)
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      int dist1 = abs(i - 3) + abs(j - 3);
      int dist2 = abs(i - 3) + abs(j - 4);
      int dist3 = abs(i - 4) + abs(j - 3);
      int dist4 = abs(i - 4) + abs(j - 4);
      
      dp_matrix[i][j] = min(min(dist1, dist2), min(dist3, dist4));
    }
  }
  Serial.println("Flood-fill matrix initialized for 8x8 maze");
}

void solveMaze() {
  if (dp_matrix[mouse.x][mouse.y] != 0) {
    Serial.print("Current position: (");
    Serial.print(mouse.x);
    Serial.print(", ");
    Serial.print(mouse.y);
    Serial.print(") - dp value: ");
    Serial.print(dp_matrix[mouse.x][mouse.y]);
    Serial.print(" - heading: ");
    Serial.println(mouse.heading);
    
    updateWalls();
    
    Cell accessibleCells[4];
    int size = 0;
    getNeighbors(mouse.x, mouse.y, accessibleCells, &size);
    
    if (size == 0) {
      Serial.println("DEAD END! No accessible neighbors - turning 180°");
      turn180();
      updateHeadingAfterTurn(2);
      floodFill();
      return;
    }
    
    Cell minCell = getMinCell(accessibleCells, size);
    
    if (dp_matrix[mouse.x][mouse.y] > dp_matrix[minCell.x][minCell.y]) {
      Serial.print("Moving to better cell (");
      Serial.print(minCell.x);
      Serial.print(", ");
      Serial.print(minCell.y);
      Serial.print(") with dp value: ");
      Serial.println(dp_matrix[minCell.x][minCell.y]);
      
      changeDirection(minCell);
      delay(500);
      
      moveForwardOneCell();
      
      mouse.x = minCell.x;
      mouse.y = minCell.y;
    } else {
      Serial.println("Current position not optimal - recalculating flood-fill");
      floodFill();
    }
  } else {
    Serial.println("🎉 MAZE SOLVED! Reached the center! 🎉");
    digitalWrite(LED_PIN, HIGH);
    stopMotors();
    while(1) {
      delay(1000);
      Serial.println("Victory! Center reached!");
    }
  }
}

void updateWalls() {
  Serial.println("Scanning walls...");
  
  // Center the robot before scanning for more accurate readings
  centerRobot();
  delay(200);
  
  bool frontWall = wallFront();
  bool leftWall = wallLeft();
  bool rightWall = wallRight();
  
  Serial.print("Wall scan results - Front: ");
  Serial.print(frontWall ? "YES" : "NO");
  Serial.print(", Left: ");
  Serial.print(leftWall ? "YES" : "NO");
  Serial.print(", Right: ");
  Serial.println(rightWall ? "YES" : "NO");
  
  if (mouse.heading == FRONT) {
    if (mouse.x > 0 && leftWall)
      vertical_barrier[mouse.x - 1][mouse.y] = 1;
    if (mouse.x < MAZE_SIZE - 1 && rightWall)
      vertical_barrier[mouse.x][mouse.y] = 1;
    if (mouse.y < MAZE_SIZE - 1 && frontWall)
      horizontal_barrier[mouse.x][mouse.y] = 1;
  } else if (mouse.heading == LEFT) {
    if (mouse.x < MAZE_SIZE - 1 && frontWall)
      vertical_barrier[mouse.x][mouse.y] = 1;
    if (mouse.y > 0 && rightWall)
      horizontal_barrier[mouse.x][mouse.y - 1] = 1;
    if (mouse.y < MAZE_SIZE - 1 && leftWall)
      horizontal_barrier[mouse.x][mouse.y] = 1;
  } else if (mouse.heading == BEHIND) {
    if (mouse.x > 0 && rightWall)
      vertical_barrier[mouse.x - 1][mouse.y] = 1;
    if (mouse.x < MAZE_SIZE - 1 && leftWall)
      vertical_barrier[mouse.x][mouse.y] = 1;
    if (mouse.y > 0 && frontWall)
      horizontal_barrier[mouse.x][mouse.y - 1] = 1;
  } else if (mouse.heading == RIGHT) {
    if (mouse.x > 0 && frontWall)
      vertical_barrier[mouse.x - 1][mouse.y] = 1;
    if (mouse.y > 0 && leftWall)
      horizontal_barrier[mouse.x][mouse.y - 1] = 1;
    if (mouse.y < MAZE_SIZE - 1 && rightWall)
      horizontal_barrier[mouse.x][mouse.y] = 1;
  }
}

void getNeighbors(short int x, short int y, Cell* cells, int* size) {
  *size = 0;
  
  if (x > 0 && !vertical_barrier[x - 1][y]) {
    cells[*size].x = x - 1;
    cells[*size].y = y;
    (*size)++;
  }
  
  if (x < MAZE_SIZE - 1 && !vertical_barrier[x][y]) {
    cells[*size].x = x + 1;
    cells[*size].y = y;
    (*size)++;
  }
  
  if (y > 0 && !horizontal_barrier[x][y - 1]) {
    cells[*size].x = x;
    cells[*size].y = y - 1;
    (*size)++;
  }
  
  if (y < MAZE_SIZE - 1 && !horizontal_barrier[x][y]) {
    cells[*size].x = x;
    cells[*size].y = y + 1;
    (*size)++;
  }
}

Cell getMinCell(Cell cells[], int size) {
  Cell minCell = cells[0];
  for (int i = 1; i < size; i++) {
    if (dp_matrix[cells[i].x][cells[i].y] < dp_matrix[minCell.x][minCell.y]) {
      minCell = cells[i];
    }
  }
  return minCell;
}

void changeDirection(Cell minCell) {
  Serial.print("Changing direction to reach cell (");
  Serial.print(minCell.x);
  Serial.print(", ");
  Serial.print(minCell.y);
  Serial.println(")");
  
  Serial.print("Current heading: ");
  Serial.print(mouse.heading);
  
  Heading targetHeading;
  
  if (mouse.x == minCell.x) {
    if (mouse.y < minCell.y) {
      targetHeading = FRONT;
    } else {
      targetHeading = BEHIND;
    }
  } else if (mouse.y == minCell.y) {
    if (mouse.x < minCell.x) {
      targetHeading = LEFT;
    } else {
      targetHeading = RIGHT;
    }
  }
  
  Serial.print(" → Target heading: ");
  Serial.println(targetHeading);
  
  int turnDiff = (targetHeading - mouse.heading + 4) % 4;
  
  switch(turnDiff) {
    case 0:
      Serial.println("Already facing correct direction - no turn needed");
      break;
    case 1:
      Serial.println("Need to turn LEFT 90°");
      turnLeft();
      updateHeadingAfterTurn(-1);
      break;
    case 2:
      Serial.println("Need to turn 180°");
      turn180();
      updateHeadingAfterTurn(2);
      break;
    case 3:
      Serial.println("Need to turn RIGHT 90°");
      turnRight();
      updateHeadingAfterTurn(1);
      break;
  }
}

void updateHeadingAfterTurn(int turnType) {
  Heading oldHeading = mouse.heading;
  
  if (turnType == -1) {
    mouse.heading = (Heading)((mouse.heading + 3) % 4);
  } else if (turnType == 1) {
    mouse.heading = (Heading)((mouse.heading + 1) % 4);
  } else if (turnType == 2) {
    mouse.heading = (Heading)((mouse.heading + 2) % 4);
  }
  
  Serial.print("Heading updated: ");
  Serial.print(oldHeading);
  Serial.print(" → ");
  Serial.println(mouse.heading);
}

void floodFill() {
  Serial.println("Running flood-fill algorithm...");
  Queue queue = { NULL, NULL };
  Cell currentCell = { mouse.x, mouse.y };
  
  if (dp_matrix[mouse.x][mouse.y] == 0) {
    Serial.println("Already at goal - no flood-fill needed");
    return;
  }
  
  enqueue(&queue, currentCell);
  while (!isQueueEmpty(&queue)) {
    Cell consideredCell = dequeue(&queue);
    Cell accessibleCells[4];
    int size = 0;
    getNeighbors(consideredCell.x, consideredCell.y, accessibleCells, &size);
    
    if (size > 0) {
      Cell minElement = getMinCell(accessibleCells, size);
      
      if (dp_matrix[consideredCell.x][consideredCell.y] - 1 != dp_matrix[minElement.x][minElement.y]) {
        int minNeighbor = dp_matrix[minElement.x][minElement.y];
        dp_matrix[consideredCell.x][consideredCell.y] = minNeighbor + 1;
        for (int i = 0; i < size; i++) {
          enqueue(&queue, accessibleCells[i]);
        }
      }
    }
  }
  Serial.println("Flood-fill completed");
}

// ------------------------------------------------------------------------
//                    Queue Implementation
// ------------------------------------------------------------------------
void enqueue(Queue* queue, Cell cell) {
  Node* newNode = (Node*)malloc(sizeof(Node));
  newNode->cell = cell;
  newNode->next = NULL;
  
  if (queue->rear == NULL) {
    queue->front = queue->rear = newNode;
  } else {
    queue->rear->next = newNode;
    queue->rear = newNode;
  }
}

Cell dequeue(Queue* queue) {
  Cell cell = { 0, 0 };
  if (queue->front == NULL) return cell;
  
  Node* temp = queue->front;
  cell = queue->front->cell;
  queue->front = queue->front->next;
  
  if (queue->front == NULL) queue->rear = NULL;
  free(temp);
  return cell;
}

int isQueueEmpty(Queue* queue) {
  return (queue->front == NULL);
}

// ------------------------------------------------------------------------
//                    Additional Helper Functions
// ------------------------------------------------------------------------

void moveForwardWithPID() {
  static unsigned long lastControlTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastControlTime >= CONTROL_PERIOD) {
    lastControlTime = currentTime;
    
    long leftDiff = encoderCountLeft - prevEncoderLeft;
    long rightDiff = encoderCountRight - prevEncoderRight;
    
    prevEncoderLeft = encoderCountLeft;
    prevEncoderRight = encoderCountRight;
    
    speedDifference = leftDiff - rightDiff;
    
    integralErrorStraight += speedDifference;
    integralErrorStraight = constrain(integralErrorStraight, -MAX_INTEGRAL, MAX_INTEGRAL);
    
    float derivative = speedDifference - prevSpeedDifference;
    float correction = (KP_STRAIGHT * speedDifference) + 
                      (KI_STRAIGHT * integralErrorStraight) + 
                      (KD_STRAIGHT * derivative);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    
    currentLeftSpeed = BASE_SPEED_LEFT - (int)(correction * 0.7);
    currentRightSpeed = BASE_SPEED_RIGHT + (int)(correction * 0.3);
    
    currentLeftSpeed = constrain(currentLeftSpeed, 50, 120);
    currentRightSpeed = constrain(currentRightSpeed, 50, 120);
    
    setLeftMotor(currentLeftSpeed);
    setRightMotor(currentRightSpeed);
    
    prevSpeedDifference = speedDifference;
  }
}