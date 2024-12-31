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
