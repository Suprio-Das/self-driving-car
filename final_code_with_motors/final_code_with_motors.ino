#include <Servo.h>

// Servo pin for steering
Servo steeringServo;
const int servoPin = 3;

// Motor Driver pins
const int motorEnableA = 5; // Speed control for left motor
const int motorEnableB = 6; // Speed control for right motor
const int motorLeft1 = 10;  // IN1 for left motor direction
const int motorLeft2 = 11;  // IN2 for left motor direction
const int motorRight1 = 7;  // IN3 for right motor direction
const int motorRight2 = 8;  // IN4 for right motor direction

// Ultrasonic sensor pins
const int trigFront = 4;
const int echoFront = 2;
const int trigLeft = A0;
const int echoLeft = A1;
const int trigRight = A2;
const int echoRight = A3;

// Buzzer pin
const int buzzer = 12;

// Variables for distances
long duration;
int distanceFront, distanceLeft, distanceRight;

// Function to calculate distance
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert to cm
}

void setup() {
  // Set pin modes for ultrasonic sensors
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  // Set pin modes for motor driver and buzzer
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);
  pinMode(motorEnableA, OUTPUT);
  pinMode(motorEnableB, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // Attach the servo motor
  steeringServo.attach(servoPin);
  steeringServo.write(90); // Center the servo

  Serial.begin(9600);

  // Enable motors
  analogWrite(motorEnableA, 255); // Full speed for left motor
  analogWrite(motorEnableB, 255); // Full speed for right motor
}

void loop() {
  // Measure distances from ultrasonic sensors
  distanceFront = getDistance(trigFront, echoFront);
  distanceLeft = getDistance(trigLeft, echoLeft);
  distanceRight = getDistance(trigRight, echoRight);

  // Print distances for debugging
  Serial.print("Front: "); Serial.print(distanceFront);
  Serial.print(" Left: "); Serial.print(distanceLeft);
  Serial.print(" Right: "); Serial.println(distanceRight);

  // Buzzer logic: If all sides are blocked, stop and activate buzzer
  if (distanceFront < 20 && distanceLeft < 20 && distanceRight < 20) {
    Serial.println("All sides blocked - Buzzer ON");
    digitalWrite(buzzer, HIGH);
    stopMotors();
    return;
  } else {
    digitalWrite(buzzer, LOW);
  }

  // Obstacle avoidance logic
  if (distanceFront < 20) { // Obstacle in front
    slowMotors();
    if (distanceLeft > distanceRight) {
      Serial.println("Front Blocked - Turning Left");
      steeringServo.write(45); // Turn left
    } else {
      Serial.println("Front Blocked - Turning Right");
      steeringServo.write(135); // Turn right
    }
    delay(500); // Allow the turn to complete
    moveForward(); // Resume normal speed
  } else {
    // No obstacle in front, move forward
    Serial.println("Path Clear - Moving Forward");
    moveForward();
    steeringServo.write(90); // Center the servo
  }

  delay(100); // Delay for stability
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, LOW);
}

// Function to slow down the motors
void slowMotors() {
  analogWrite(motorEnableA, 150); // Reduce speed for left motor
  analogWrite(motorEnableB, 150); // Reduce speed for right motor
}

// Function to move forward at normal speed
void moveForward() {
  analogWrite(motorEnableA, 255); // Full speed for left motor
  analogWrite(motorEnableB, 255); // Full speed for right motor
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
}