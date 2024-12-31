# Self-Driving Car with Obstacle Avoidance

## Introduction
This project involves designing and implementing a self-driving car with obstacle avoidance functionality using an Arduino Uno. The system uses ultrasonic sensors to detect obstacles and motor drivers to control the car's movement. A buzzer provides audible feedback when the car encounters obstacles on all sides. This README details the components, design, functionality, and code implementation of the project.

---

## Objectives
- Develop an autonomous car that avoids obstacles using ultrasonic sensors.
- Implement motor controls to allow forward motion, left and right turns, and stopping.
- Provide audible alerts when obstacles are detected on all sides.
- Debug and monitor system performance using the Arduino Serial Monitor.

---

## Components Used
1. **Arduino Uno**: Microcontroller for processing sensor data and controlling motors.
2. **Ultrasonic Sensors** (3 units): Detect obstacles in front, left, and right directions.
   - Pins Used: 
     - Trig: 4, A0, A2
     - Echo: 2, A1, A3
3. **Motor Driver (L298N)**:
   - Controls two DC motors for car movement.
   - Pins Used:
     - Enable A: 5 (Left motors)
     - Enable B: 6 (Right motors)
     - IN1/IN2: 10/11 (Left motor directions)
     - IN3/IN4: 7/8 (Right motor directions)
4. **DC Motors**: Drive the car’s wheels.
5. **Buzzer**: Alerts when obstacles block all sides.
   - Pin Used: 12
6. **Power Supply**: Provides power to motors and Arduino.

---

## System Design
### Circuit Diagram
1. The ultrasonic sensors are connected to the respective trigger and echo pins of the Arduino for detecting obstacles.
2. The motor driver is connected to the Arduino and DC motors to control their direction and speed.
3. The buzzer is connected to the Arduino for obstacle alerts.

### Workflow
1. Ultrasonic sensors send ultrasonic waves and receive the reflected waves to calculate distances.
2. The Arduino processes distance data from the sensors and determines the appropriate movement or action:
   - Move forward if no obstacle is detected.
   - Turn left or right if an obstacle is detected ahead.
   - Stop and activate the buzzer if obstacles block all paths.
3. Motor driver receives signals from Arduino to control motor directions and speeds.

---

## Code Implementation
### Key Functions
1. **`getDistance(trigPin, echoPin)`**:
   - Calculates the distance of obstacles using the speed of sound.
2. **`moveForward()`**:
   - Moves the car forward.
3. **`turnLeft()`**:
   - Turns the car left by reversing left motors and moving right motors forward.
4. **`turnRight()`**:
   - Turns the car right by reversing right motors and moving left motors forward.
5. **`stopMotors()`**:
   - Stops the car by deactivating all motor pins.

### Control Logic
- The main control loop (`loop()`) measures distances from the sensors and determines the appropriate movement or action.
- If all paths are blocked, the buzzer is activated, and the car stops.
- The system prints sensor readings to the Serial Monitor for debugging.

### Code
```cpp
// Motor Driver Pins
const int motorEnableA = 5; // Speed control for left motors
const int motorEnableB = 6; // Speed control for right motors
const int motorLeft1 = 10;  // IN1 for left motor direction
const int motorLeft2 = 11;  // IN2 for left motor direction
const int motorRight1 = 7;  // IN3 for right motor direction
const int motorRight2 = 8;  // IN4 for right motor direction

// Ultrasonic Sensor Pins
const int trigFront = 4;
const int echoFront = 2;
const int trigLeft = A0;
const int echoLeft = A1;
const int trigRight = A2;
const int echoRight = A3;

// Buzzer Pin
const int buzzer = 12;

// Variables for distances
long duration;
int distanceFront, distanceLeft, distanceRight;

// Function to calculate distance from ultrasonic sensor
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
  // Set ultrasonic sensor pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  // Set motor driver pins
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);
  pinMode(motorEnableA, OUTPUT);
  pinMode(motorEnableB, OUTPUT);

  // Set buzzer pin
  pinMode(buzzer, OUTPUT);

  Serial.begin(9600);

  // Enable motors at full speed
  analogWrite(motorEnableA, 255);
  analogWrite(motorEnableB, 255);
}

void loop() {
  // Measure distances
  distanceFront = getDistance(trigFront, echoFront);
  distanceLeft = getDistance(trigLeft, echoLeft);
  distanceRight = getDistance(trigRight, echoRight);

  // Print distances for debugging
  Serial.print("Front: "); Serial.print(distanceFront);
  Serial.print(" Left: "); Serial.print(distanceLeft);
  Serial.print(" Right: "); Serial.println(distanceRight);

  // Collision detection
  if (distanceFront < 20 && distanceLeft < 20 && distanceRight < 20) {
    Serial.println("All sides blocked. Stopping!");
    stopMotors();
    digitalWrite(buzzer, HIGH);
    return;
  } else {
    digitalWrite(buzzer, LOW);
  }

  // Obstacle avoidance
  if (distanceFront < 20) {
    if (distanceLeft > distanceRight) {
      Serial.println("Obstacle ahead. Turning left.");
      turnLeft();
    } else {
      Serial.println("Obstacle ahead. Turning right.");
      turnRight();
    }
    delay(500); // Allow the car to complete the turn
  } else {
    Serial.println("Path clear. Moving forward.");
    moveForward();
  }

  delay(100); // Delay for stability
}

// Function to stop motors
void stopMotors() {
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, LOW);
}

// Function to move forward
void moveForward() {
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
}

// Function to turn left
void turnLeft() {
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, HIGH);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
}

// Function to turn right
void turnRight() {
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, HIGH);
}
```

---

## Results and Observations
- The car successfully avoided obstacles and navigated around them.
- Audible alerts worked as intended when all sides were blocked.
- The system demonstrated real-time responsiveness to obstacles.

---

## Conclusion
This project showcases an effective implementation of an obstacle-avoidance system for autonomous vehicles. It can be further enhanced with additional features such as GPS integration or advanced sensors for more precise navigation.

---

## Future Enhancements
- Integrate additional sensors for better obstacle detection.
- Implement a machine learning model for advanced decision-making.
- Add camera modules for object recognition.
- Enhance power management for extended use.

