#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// Create a PCA9685 object (default I2C address 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo channels for the three joints (adjust as needed)
const int servo1 = 1;
const int servo2 = 2;
const int servo3 = 3;

// Servo pulse width limits (calibrate these for your servos)
const int SERVOMIN = 150;  // Minimum pulse length count
const int SERVOMAX = 600;  // Maximum pulse length count

// Global current angles for the three servos (start at neutral 90°)
int currentAngles[3] = {90, 90, 90};

// Maximum speed for smooth motion (degrees per step)
const float MAX_SPEED = 4.0;  // Lower values = smoother but slower motion

// Convert a servo angle (0–180°) to a PCA9685 pulse count
int angleToPulse(int angle) {
  return map(constrain(angle, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
}

// Move directly to target angles without interpolation
// This is used for commands from Python trajectory planning
void moveDirectly(int targetAngles[3]) {
  // Set servos to target angles immediately
  pwm.setPWM(servo1, 0, angleToPulse(targetAngles[0]));
  pwm.setPWM(servo2, 0, angleToPulse(targetAngles[1]));
  pwm.setPWM(servo3, 0, angleToPulse(targetAngles[2]));
  
  // Update currentAngles
  for (int j = 0; j < 3; j++) {
    currentAngles[j] = targetAngles[j];
  }
}

// Move from currentAngles to targetAngles with smooth interpolation
// This is used when we want the Arduino to handle motion planning
void moveSmoothly(int targetAngles[3]) {
  // Calculate the maximum number of steps needed based on the largest angle change
  float maxChange = 0;
  for (int i = 0; i < 3; i++) {
    float change = abs(targetAngles[i] - currentAngles[i]);
    if (change > maxChange) {
      maxChange = change;
    }
  }
  
  // Calculate number of steps needed (at least 1)
  // Fix: Cast the result of ceil() to int to match types for max()
  int calculatedSteps = (int)ceil(maxChange / MAX_SPEED);
  int steps = (calculatedSteps > 1) ? calculatedSteps : 1; // Ensure at least 1 step
  
  // Perform the interpolation
  int startAngles[3] = { currentAngles[0], currentAngles[1], currentAngles[2] };
  
  for (int i = 1; i <= steps; i++) {
    // Use sine-based acceleration profile for smooth start/stop
    float fraction = i / (float)steps;
    float smoothFraction = 0.5 - 0.5 * cos(PI * fraction); // Smoother S-curve acceleration
    
    int newAngles[3];
    for (int j = 0; j < 3; j++) {
      newAngles[j] = startAngles[j] + (targetAngles[j] - startAngles[j]) * smoothFraction;
    }
    
    // Update servos with interpolated angles
    pwm.setPWM(servo1, 0, angleToPulse(newAngles[0]));
    pwm.setPWM(servo2, 0, angleToPulse(newAngles[1]));
    pwm.setPWM(servo3, 0, angleToPulse(newAngles[2]));
    
    // Adaptive delay based on step count (more steps = shorter delays)
    // Fix: Ensure we're using consistent types for min/max
    int delayTime;
    if (steps <= 20) {
      delayTime = 25; // Slower for small movements
    } else if (steps >= 100) {
      delayTime = 5;  // Faster for large movements
    } else {
      delayTime = 30 - (steps / 4); // Linear scale between 5-25ms
    }
    delay(delayTime);
  }
  
  // Final correction: explicitly set servos to target angles
  pwm.setPWM(servo1, 0, angleToPulse(targetAngles[0]));
  pwm.setPWM(servo2, 0, angleToPulse(targetAngles[1]));
  pwm.setPWM(servo3, 0, angleToPulse(targetAngles[2]));
  
  // Update currentAngles
  for (int j = 0; j < 3; j++) {
    currentAngles[j] = targetAngles[j];
  }
}

// Process an incoming serial command
void processCommand(String command) {
  command.trim();
  
  if (command.startsWith("SETPOS")) {
    // Format: "SETPOS a1 a2 a3" - Direct movement (for Python interpolation)
    int firstSpace = command.indexOf(' ');
    if (firstSpace == -1) return;
    
    String params = command.substring(firstSpace + 1);
    params.trim();
    int space1 = params.indexOf(' ');
    int space2 = params.indexOf(' ', space1 + 1);
    if (space1 == -1 || space2 == -1) return;
    
    String s1 = params.substring(0, space1);
    String s2 = params.substring(space1 + 1, space2);
    String s3 = params.substring(space2 + 1);
    
    int a1 = s1.toInt();
    int a2 = s2.toInt();
    int a3 = s3.toInt();
    int target[3] = {a1, a2, a3};
    
    // Use direct movement for SETPOS (no interpolation)
    moveDirectly(target);
    
    // Print new angles for monitoring
    Serial.print("Angles: ");
    Serial.print(currentAngles[0]);
    Serial.print(", ");
    Serial.print(currentAngles[1]);
    Serial.print(", ");
    Serial.println(currentAngles[2]);
  }
  else if (command.startsWith("SMOOTH")) {
    // Format: "SMOOTH a1 a2 a3" - Smooth movement (Arduino interpolation)
    int firstSpace = command.indexOf(' ');
    if (firstSpace == -1) return;
    
    String params = command.substring(firstSpace + 1);
    params.trim();
    int space1 = params.indexOf(' ');
    int space2 = params.indexOf(' ', space1 + 1);
    if (space1 == -1 || space2 == -1) return;
    
    String s1 = params.substring(0, space1);
    String s2 = params.substring(space1 + 1, space2);
    String s3 = params.substring(space2 + 1);
    
    int a1 = s1.toInt();
    int a2 = s2.toInt();
    int a3 = s3.toInt();
    int target[3] = {a1, a2, a3};
    
    // Use smooth interpolated movement
    moveSmoothly(target);
    
    // Print new angles for monitoring
    Serial.print("Angles: ");
    Serial.print(currentAngles[0]);
    Serial.print(", ");
    Serial.print(currentAngles[1]);
    Serial.print(", ");
    Serial.println(currentAngles[2]);
  }
}

void setup() {
  Serial.begin(9600);  // Set baud rate to 9600 (matching Python)
  Serial.println("Arm Control Ready");
  
  pwm.begin();
  pwm.setPWMFreq(50);  // Typical servo frequency
  
  // Initialize servos to neutral (90°)
  pwm.setPWM(servo1, 0, angleToPulse(currentAngles[0]));
  pwm.setPWM(servo2, 0, angleToPulse(currentAngles[1]));
  pwm.setPWM(servo3, 0, angleToPulse(currentAngles[2]));
  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }
}