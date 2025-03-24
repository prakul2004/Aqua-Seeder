#define enA 10 // Enable1 L298 Pin enA 
#define in1 9  // Motor1 L298 Pin in1 
#define in2 8  // Motor1 L298 Pin in2 
#define in3 7  // Motor2 L298 Pin in3 
#define in4 6  // Motor2 L298 Pin in4 
#define enB 5  // Enable2 L298 Pin enB 

#define L_S A0 // IR sensor Left
#define R_S A1 // IR sensor Right
#define TRIG_PIN 4 // Ultrasonic sensor Trig pin
#define ECHO_PIN 3 // Ultrasonic sensor Echo pin

#define SIGNAL_PIN 2  // Pin to send signal to ESP32

unsigned long previousMillis = 0;  // Variable to store last time the robot moved
unsigned long stopInterval = 15000; // 5 seconds interval to stop
unsigned long waitInterval = 12000; // 10 seconds wait time
unsigned long lastActionTime = 0;  // Keeps track of last action time (stop or wait)

bool isStopped = false;  // Flag to track if the robot is stopped
bool obstacleDetected = false; // Flag to track if an obstacle is detected

// Function prototypes
void forward();
void back();
void turnRight();
void turnLeft();
void stop();
float getDistance();

void setup() {
  Serial.begin(9600); // Start serial communication with the PC
  
  pinMode(SIGNAL_PIN, OUTPUT);  // Set pin 2 as output to send signal to ESP32
  pinMode(R_S, INPUT); // Declare IR sensor as input
  pinMode(L_S, INPUT); // Declare IR sensor as input

  pinMode(enA, OUTPUT); // Declare as output for L298 Pin enA 
  pinMode(in1, OUTPUT); // Declare as output for L298 Pin in1 
  pinMode(in2, OUTPUT); // Declare as output for L298 Pin in2 
  pinMode(in3, OUTPUT); // Declare as output for L298 Pin in3   
  pinMode(in4, OUTPUT); // Declare as output for L298 Pin in4 
  pinMode(enB, OUTPUT); // Declare as output for L298 Pin enB 

  pinMode(TRIG_PIN, OUTPUT); // Ultrasonic sensor Trig
  pinMode(ECHO_PIN, INPUT);  // Ultrasonic sensor Echo

  analogWrite(enA, 200); // Set motor speed (0-255)
  analogWrite(enB, 200); // Set motor speed (0-255)
}

void loop() {  
  unsigned long currentMillis = millis();  // Get current time
  float distance = getDistance(); // Measure distance using ultrasonic sensor
  
  // Check for obstacle
  if (distance < 20.0 && distance > 0.0) { // Object detected within 20 cm
    stop();  // Stop the robot
    obstacleDetected = true;  // Set obstacle flag
    Serial.println("Obstacle detected! Robot stopped.");
    // No signal sent to ESP32 for obstacle stop
    return; // Skip the rest of the loop
  } else if (obstacleDetected && distance >= 20.0) {
    obstacleDetected = false; // Reset obstacle flag
    Serial.println("Obstacle cleared. Resuming operation.");
  }

  // Periodic stopping logic
  if (!obstacleDetected && currentMillis - lastActionTime >= stopInterval && !isStopped) {
    stop();  // Stop the robot
    isStopped = true;  // Mark the robot as stopped
    lastActionTime = currentMillis;  // Update the last action time
    // Send signal to ESP32 that the robot is stopped due to delay
    Serial.println("STOPPED due to delay");
    digitalWrite(SIGNAL_PIN, HIGH);  // Send signal HIGH to ESP32
  }

  // Wait and resume logic
  if (isStopped && currentMillis - lastActionTime >= waitInterval) {
    isStopped = false;  // Mark the robot as ready to run again
    lastActionTime = currentMillis;  // Reset the last action time
    Serial.println("Robot waiting time over, ready to run.");
    // Send signal to ESP32 that the robot is ready to run again
    Serial.println("RUNNING");
    digitalWrite(SIGNAL_PIN, LOW);  // Send signal LOW to ESP32
  }

  // Line following logic
  if (!isStopped && !obstacleDetected) {
    int leftSensor = digitalRead(L_S);
    int rightSensor = digitalRead(R_S);
    
    Serial.print("Left Sensor: ");
    Serial.print(leftSensor);
    Serial.print(" | Right Sensor: ");
    Serial.println(rightSensor);

    if (leftSensor == 0 && rightSensor == 0) {
      forward(); // Move forward
    }  
    else if (leftSensor == 0 && rightSensor == 1) {
      turnRight(); // Turn right
    }  
    else if (leftSensor == 1 && rightSensor == 0) {
      turnLeft(); // Turn left
    }
    else if (leftSensor == 1 && rightSensor == 1) {
      stop(); // Stop
    }
  }

  delay(10); // Small delay for stability
}

void forward() {
  digitalWrite(in1, LOW);  // Left motor forward
  digitalWrite(in2, HIGH); // Left motor backward
  digitalWrite(in3, HIGH); // Right motor forward
  digitalWrite(in4, LOW);  // Right motor backward
}

void back() {
  digitalWrite(in1, HIGH); // Left motor backward
  digitalWrite(in2, LOW);  // Left motor forward
  digitalWrite(in3, LOW);  // Right motor forward
  digitalWrite(in4, HIGH); // Right motor backward
}

void turnRight() {
  digitalWrite(in1, LOW);  // Left motor forward
  digitalWrite(in2, HIGH); // Left motor backward
  digitalWrite(in3, LOW);  // Right motor forward
  digitalWrite(in4, HIGH); // Right motor backward
}

void turnLeft() {
  digitalWrite(in1, HIGH); // Left motor forward
  digitalWrite(in2, LOW);  // Left motor backward
  digitalWrite(in3, HIGH); // Right motor forward
  digitalWrite(in4, LOW);  // Right motor backward
}

void stop() {
  digitalWrite(in1, LOW); // Left motor forward
  digitalWrite(in2, LOW); // Left motor backward
  digitalWrite(in3, LOW); // Right motor forward
  digitalWrite(in4, LOW); // Right motor backward
}

// Function to get distance from ultrasonic sensor
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH); // Measure time of echo
  float distance = duration * 0.034 / 2;  // Convert to distance in cm
  return distance;
}
