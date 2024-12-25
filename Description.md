# Self-Driving and Obstacle Avoidance Robotic Car

## Project Overview

This project involves designing and programming an obstacle avoidance robotic car using an Arduino microcontroller, servo motor for steering, L298N motor driver for motor control, ultrasonic sensors for obstacle detection, and a buzzer for alert signals. The car autonomously navigates its environment, avoids obstacles, and takes appropriate actions based on sensor readings.

---

## Components Required

1. **Arduino Uno** - Microcontroller board for controlling the car.
2. **L298N Motor Driver** - For driving the left and right motors.
3. **Servo Motor** - For steering control.
4. **Ultrasonic Sensors (3)** - For detecting obstacles (front, left, and right).
5. **DC Motors (2)** - For moving the car.
6. **Buzzer** - For alert signals.
7. **Power Supply** - For powering the components.
8. **Chassis and Wheels** - For the robotic car structure.
9. **Wires and Connectors** - For connections.

---

## Circuit Diagram

- **Ultrasonic Sensors**:

  - Front sensor: `Trig -> Pin 4`, `Echo -> Pin 2`
  - Left sensor: `Trig -> A0`, `Echo -> A1`
  - Right sensor: `Trig -> A2`, `Echo -> A3`

- **Motors**:

  - Left motor: `Enable -> Pin 5`, `IN1 -> Pin 10`, `IN2 -> Pin 11`
  - Right motor: `Enable -> Pin 6`, `IN3 -> Pin 7`, `IN4 -> Pin 8`

- **Servo Motor**:

  - Control pin: `Pin 3`

- **Buzzer**:

  - Control pin: `Pin 12`

---

## Working Principle

The car uses three ultrasonic sensors to measure distances from obstacles. Based on these measurements, the Arduino executes decisions to avoid collisions:

1. **All sides blocked**: Stops the car and activates the buzzer.
2. **Front blocked**: Slows down the motors, steers left or right based on the clearer path, and resumes normal speed.
3. **Path clear**: Moves forward at full speed.

---

## Arduino Code

```cpp
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
```

---

## Features

- **Autonomous Navigation**: The car adjusts its movement dynamically based on sensor readings.
- **Collision Avoidance**: Detects obstacles and reroutes.
- **Buzzer Alerts**: Warns when all sides are blocked.

---

## Applications

- Robotics learning and experiments.
- Autonomous navigation research.
- Practical implementation of obstacle avoidance algorithms.

---

## Future Improvements

- Integrate a camera for image-based navigation.
- Add GPS for outdoor navigation.
- Use advanced sensors for better obstacle detection.

---

## Conclusion

This project demonstrates the integration of multiple hardware components to create an autonomous robotic system. The modular design allows for easy upgrades and enhancements, making it a valuable learning experience for robotics enthusiasts.
