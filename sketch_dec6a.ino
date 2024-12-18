#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Math.h>

// LCD setup
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Create an instance of the MPU6050 class
Adafruit_MPU6050 mpu;

// Motor control pins
int enA = 10;
int enB = 11;
int IN1 = 2;
int IN2 = 3;
int IN3 = 12;
int IN4 = 13;

float yawAngle = 0;      // Global variable for yaw angle
unsigned long lastTime;  // Global variable for timing


// Encoder pins
int encoderLeft = A0;  // Left encoder
int encoderRight = A1; // Right encoder

// IR sensor pins
int IR_L = A2; // Left IR sensor
int IR_R = A3; // Right IR sensor

// Wheel and encoder parameters
const float wheelDiameter = 6.0;  // Wheel diameter in cm
const int encoderPPR = 20;        // Encoder pulses per revolution
const float distancePerPulse = (((3.14159 * wheelDiameter) / encoderPPR)*2); // Distance per pulse in cm

// Distance variables
int EncoderLsig = 0; // Steps for left encoder
int EncoderLsig_old = 0; // Steps for right encoder
int EncoderRsig = 0; // Steps for left encoder
int EncoderRsig_old = 0; // Steps for right encoder
float steps1 = 0;     // Total distance for left encoder
float steps2 = 0;     // Total distance for right encoder
float distance1 = 0;     // Total distance for left encoder
float distance2 = 0;     // Total distance for right encoder
float totalDistance = 0; // Total distance (left + right)
float showDistance=0;
int stppedGrpNum = 0;

// Calibration and smoothing variables
float pitchOffset = 0;
float smoothedPitch = 0;
float alpha = 0.9; // Smoothing factor (0 < alpha < 1)

// Timing variables
unsigned long startTime = 0;      // Tracks when the program started
bool measurementStarted = false;  // Tracks whether distance measurement has started

// Function to calculate the pitch offset during calibration
float calculatePitchOffset() {
  float totalPitch = 0;
  int samples = 100; // Number of readings for averaging

  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    float pitch = atan2(-accel.acceleration.x, 
                        sqrt(accel.acceleration.y * accel.acceleration.y + 
                             accel.acceleration.z * accel.acceleration.z)) * 180 / PI;
    totalPitch += pitch;
    delay(10); // Small delay between readings
  }

  return totalPitch / samples; // Return the average pitch as the offset
}

// Initialization function
// Function prototypes
void moveForward();
void stopMotors();
void followline(int speed);
void turn360();
void encoderLeftISR();
void encoderRightISR();

void setup() {
  // Initialize LCD
  lcd.begin(16, 2); // Set up the LCD with 16 columns and 2 rows
  lcd.print("START");

  // Serial monitor for debugging
  Serial.begin(9600);

  // Motor pin setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Encoder pin setup
  pinMode(encoderLeft, INPUT);
  pinMode(encoderRight, INPUT);

  // IR sensor pin setup
  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);

  // Ensure motors are off initially
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    lcd.clear();
    lcd.print("MPU6050 Error");
    while (1) delay(10); // Stay in error state
  }

  // Calibrate pitch offset
  lcd.clear();
  lcd.print("Calibrating...");
  pitchOffset = calculatePitchOffset();
  lcd.clear();
  lcd.print("Offset: ");
  lcd.print(pitchOffset, 1); // Display offset
  delay(2000); // Pause for display
  lcd.clear();

  // Record the start time
  startTime = millis();

  //attachInterrupt(digitalPinToInterrupt(encoderLeft), encoderLeftISR, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoderRight), encoderRightISR, RISING);
}

void loop() {
  // Get MPU6050 sensor events (acceleration, gyroscope, and temperature)
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  int speed;

  // Calculate elapsed time in seconds since the program started
  unsigned long currentTime = (millis() - startTime) / 1000;

  // Calculate pitch (angle)
  float pitch = atan2(-accel.acceleration.x, 
                      sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;
  pitch -= pitchOffset; // Apply offset correction

  // Apply smoothing to pitch angle
  smoothedPitch = alpha * smoothedPitch + (1 - alpha) * pitch;

  //totalDistance = distance1 + distance2; // Sum left and right distances

  // Actions based on time
  if (currentTime >= 0 && currentTime <= 0.5) {
    //followline(200);
    moveForward();

/*} else if (currentTime > 1 && currentTime <= 1.5) {
    moveForward();*/

  } else if (currentTime > 0.5 && currentTime <= 4.5) { //ok
    stopMotors();

  }  else if (currentTime > 4.5 && currentTime <= 5) { //ok
    moveForward();

  } else if (currentTime > 5 && currentTime <= 9) {//ok
    stopMotors();

  } else if (currentTime > 9 && currentTime < 10) { // ok
    turn360();

  } else if (currentTime >= 10 && currentTime <= 13) {
    stopMotors();

  } else if (currentTime > 13 && currentTime <= 14) {
    //moveForward();
    followline(150);

  } else if (currentTime > 15 && currentTime < 20) {
    stopMotors();
    
  } else if (currentTime >= 22 && !measurementStarted) {
    measurementStarted = true;
    steps1 = 0; // Reset encoder steps
    steps2 = 0;
    distance1 = 0.0;
    distance2 = 0.0;

  }

  if (measurementStarted) {
    // Update distance only if steps change
    

      if (digitalRead(encoderLeft)) { 
    steps1++;
    while (digitalRead(encoderLeft)); // Avoid duplicate counts
  }

  if (digitalRead(encoderRight)) { 
    steps2++;
    while (digitalRead(encoderRight)); // Avoid duplicate counts
  }

  // Calculate distances
  distance1 = steps1 * distancePerPulse;
  distance2 = steps2 * distancePerPulse;
  totalDistance = (distance1 + distance2) / 2;

    if ((fabs(totalDistance - 70) < 2 )&& stppedGrpNum == 0 ){
      stppedGrpNum = 1;
      stopMotors();
      delay(3000);
    }

    // Debug print distances
    Serial.print("Distance1: ");
    Serial.print(distance1);
    Serial.print(" Distance2: ");
    Serial.println(distance2);
    followline(70);
    
  }
  


  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(currentTime);
  lcd.print("s "); // Append "s" for seconds
  lcd.print("A:");
  lcd.print(smoothedPitch, 1); // Show angle with 1 decimal place
  lcd.print((char)223); // Degree symbol

  lcd.setCursor(0, 1);
  lcd.print("Dist:");
  lcd.print(totalDistance, 2); // Display with 2 decimal places
  lcd.print("cm   "); // Add spaces to overwrite previous text
  }


void turn360() {
yawAngle = 0; // Reset yaw angle
  lastTime = millis(); // Initialize time

  // Start the motor for turning
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  while (fabs(yawAngle) < 360) { // Keep turning until yaw angle is 360 degrees
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Get angular velocity (z-axis)
    float angularVelocity = gyro.gyro.z * 180 / PI; // Convert rad/s to degrees/s

    // Calculate delta time (in seconds)
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Update yaw angle
    yawAngle += angularVelocity * deltaTime;

    // Debugging: Print yaw angle
    Serial.print("Yaw Angle: ");
    Serial.println(yawAngle);
  }
  }

 void moveForward() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

 void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

 void followline(int speed) {
  int Right = analogRead(IR_L); // Read the value from left IR sensor
  int Left = analogRead(IR_R);  // Read the value from right IR sensor

  Serial.print("Right IR: ");
  Serial.print(Right);
  Serial.print(" Left IR: ");
  Serial.println(Left);

  if ((Right > 100) && (Left > 50)) { // Move straight
    analogWrite(enA, speed);
    analogWrite(enB, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    

  } else if ((Right < 100) && (Left > 50)) { // Turn right
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(enA, 255);
    analogWrite(enB, 255);
    delay(50);

  } else if ((Right > 100) && (Left < 50)) { // Turn left
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(enA, 255);
    analogWrite(enB, 255);
    delay(50);
  } 
    else if ((Right < 100) && (Left < 50)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
}
// Example ISR for encoder
 void encoderLeftISR() {
  EncoderLsig++;
}

 void encoderRightISR() {
  EncoderRsig++;
}

