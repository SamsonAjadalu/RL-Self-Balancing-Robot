#include <Wire.h>          // I2C library for MPU6050
#include <MPU6050.h>       // MPU6050 library for sensor readings

// Motor pins
const int right_R1 = 8, right_R2 = 12, PWM_R = 10;
const int left_L1 = 7, left_L2 = 6, PWM_L = 9;

// MPU6050 object
MPU6050 mpu6050;

// PID parameters for balance control
double kp = 20.0, kd = 0.5;  // Adjusted Kp and Kd values
double angleSetpoint = 0.0;  // Target angle for balancing (0 means upright)

// Variables for PD control
double angle, previousError = 0;

// Experience replay parameters
const int maxExperience = 100;  // Maximum size of the experience replay buffer
int experienceIndex = 0;         // Current index in the experience buffer
double experienceBuffer[maxExperience][4]; // Buffer to store experiences

// Manual control variables
bool manualMode = true;  // Flag to indicate manual mode (true) or balance mode (false)

// Timer for balance control
unsigned long previousTime = 0;
const int balanceInterval = 5; // 5 ms

void setup() {
  Serial.begin(9600); // Start Serial communication at 9600 baud rate

  // Set motor control pins as OUTPUT
  pinMode(right_R1, OUTPUT);
  pinMode(right_R2, OUTPUT);
  pinMode(left_L1, OUTPUT);
  pinMode(left_L2, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  // Initialize the MPU6050 sensor
  Wire.begin();
  mpu6050.initialize();

  // Check if MPU6050 is connected
  if (mpu6050.testConnection()) {
    Serial.println("MPU6050 connected successfully.");
  } else {
    Serial.println("MPU6050 connection failed.");
  }

  Serial.println("Ready to receive commands."); // Feedback message
}

void loop() {
  // Read commands from the Bluetooth or Serial monitor
  readCommands();

  // Balance control loop if not in manual mode
  if (!manualMode) {
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= balanceInterval) {
      balanceControl();
      previousTime = currentTime;
    }
  }
}

// Function to read commands from Serial or Bluetooth
void readCommands() {
  if (Serial.available()) { // Check if data is available to read
    char command = Serial.read(); // Read the incoming Bluetooth command

    // Check the received command and control the LED or robot accordingly
    switch(command) {
      case 'M': // Toggle manual mode
        manualMode = !manualMode; // Toggle between manual and balance mode
        if (manualMode) {
          Serial.println("Switched to Manual Mode.");
        } else {
          Serial.println("Switched to Balance Mode.");
        }
        break;
      case 'S': // Stop the car
        moveMotor(0, 0);
        Serial.println("Stop");
        break;
      case 'F': // Move forward in manual mode
        if (manualMode) {
          moveMotor(50, 50); // Reduced speed for better control
          Serial.println("Moving forward");
        }
        break;
      case 'B': // Move backward in manual mode
        if (manualMode) {
          moveMotor(-50, -50); // Reduced speed for better control
          Serial.println("Moving backward");
        }
        break;
      case 'L': // Turn left in manual mode
        if (manualMode) {
          moveMotor(30, 50); // Adjusted speed for turning
          Serial.println("Turning left");
        }
        break;
      case 'R': // Turn right in manual mode
        if (manualMode) {
          moveMotor(50, 30); // Adjusted speed for turning
          Serial.println("Turning right");
        }
        break;
      default:
        Serial.print("Received unknown command: ");
        Serial.println(command);
        break;
    }
  }
}

// Function to balance the robot using PD control
void balanceControl() {
  // Read raw data from MPU6050
  int16_t ax, ay, az;  // Accelerometer data
  int16_t gx, gy, gz;  // Gyroscope data
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate angle using accelerometer data (simplified method)
  angle = atan2(ay, az) * 180 / PI;

  // PD control formula
  double error = angleSetpoint - angle;
  double derivative = error - previousError;
  double pwmOutput = kp * error + kd * derivative;

  // Move motors based on PWM output from PD controller
  moveMotor(pwmOutput, pwmOutput);

  // Store experience (state, action, reward, new state)
  storeExperience(angle, pwmOutput, calculateReward(), angle); // Adjust parameters accordingly

  // Update previous error
  previousError = error;
}

// Function to move motors with given PWM values
void moveMotor(int pwm1, int pwm2) {
  pwm1 = constrain(pwm1, -255, 255); // Constrain PWM values to be between -255 and 255
  pwm2 = constrain(pwm2, -255, 255);

  // Control left motor
  if (pwm1 >= 0) {
    digitalWrite(left_L1, HIGH);
    digitalWrite(left_L2, LOW);
    analogWrite(PWM_L, pwm1);
  } else {
    digitalWrite(left_L1, LOW);
    digitalWrite(left_L2, HIGH);
    analogWrite(PWM_L, -pwm1);
  }

  // Control right motor
  if (pwm2 >= 0) {
    digitalWrite(right_R1, HIGH);
    digitalWrite(right_R2, LOW);
    analogWrite(PWM_R, pwm2);
  } else {
    digitalWrite(right_R1, LOW);
    digitalWrite(right_R2, HIGH);
    analogWrite(PWM_R, -pwm2);
  }
}

// Function to store experiences
void storeExperience(double state, double action, double reward, double newState) {
  experienceBuffer[experienceIndex][0] = state;   // Current state
  experienceBuffer[experienceIndex][1] = action;  // Action taken
  experienceBuffer[experienceIndex][2] = reward;  // Reward received
  experienceBuffer[experienceIndex][3] = newState; // Next state

  experienceIndex = (experienceIndex + 1) % maxExperience; // Loop back if we reach max size
}

// Function to calculate reward (implement your reward logic)
double calculateReward() {
  // You can modify this function based on your reward strategy
  // For example, reward can be based on how close the angle is to the setpoint
  return 1.0 - abs(angle - angleSetpoint) / 90.0; // Simple reward example
}

// Call this function in the void loop() to print the Q-table
void printQTable() {
  Serial.println("Q-table:");
  for (int i = 0; i < maxExperience; i++) {
    Serial.print("Experience ");
    Serial.print(i);
    Serial.print(" - State: ");
    Serial.print(experienceBuffer[i][0]); // Current state
    Serial.print(" | Action: ");
    Serial.print(experienceBuffer[i][1]); // Action taken
    Serial.print(" | Reward: ");
    Serial.print(experienceBuffer[i][2]); // Reward received
    Serial.print(" | New State: ");
    Serial.println(experienceBuffer[i][3]); // New state
  }
}
