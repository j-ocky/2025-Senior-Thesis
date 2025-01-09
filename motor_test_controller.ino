// motor_test_controller.ino
//
// This arduino program represents a compilation of the 4 controllers designed, each with its own function:
//  
//    [Action]                    [Type of Controller]                          [Function Name]
//  ____________________________________________________________________________________________________
//  1) Straight Swimming;         PI + Feed-Forward Controller;                 "straight"
//  2) Asymmetric Flapping Turn;  PSO-Optimized PI Controller;                  "turnUsingPSO" and "turnUsingPSOcombined"
//  3) Asymmetric Flapping Turn;  Gain-Scheduled PI + Feed-Forward Controller;  "turnUsingFeedForward"
//  4) Glide Turn;                PID Controller;                               "setPosition"
//
//  The main() function creates a simple movement sequence by calling some of the above functions, and this can be easily changed in main().
//
//  A short description is given at the top of each function to explain its general purpose.
//
//
//  Below are the key global variables subject to change for quick testing (initialized in beginning):
//  1) setpoint: Initial setpoint flapping speed, max ~5 Hz attained in testing (can adjust alternation via ratio)
//  2) setpointRatio: Used to calculate the alternate (slower) setpoint for asymmetric flapping; typically between 1 (straight swimming) and 2.5 (high turning effort)
//  3) posSetpoint: Setpoint for position controller, used to set the position of caudal fin for gliding turn
//  
//  
//  



#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "BlueSwarm";
const char* password = "Tpossr236";

Servo myservo;

// fish assembly setup pins
#define ENCODER_A 27
#define ENCODER_B 26
#define ENCODER_Z 25
#define MOTOR_IN1 33
#define MOTOR_IN2 32
#define POT 34

// motor stand setup pins
// #define ENCODER_A 22
// #define ENCODER_B 23
// #define ENCODER_Z 21
// #define MOTOR_IN1 18
// #define MOTOR_IN2 19
// #define POT 13

volatile long encoderCount = 0;  // Variable to store encoder pulse count

volatile long encoderAbsPosition = 0;
unsigned long lastTime = 0;      // Time tracking for Hz calculation
unsigned long lastPulseTime = 0; // Time tracking for Hz calculation
unsigned long lastSpeedCalcTime = 0; // Time tracking for speed calculation
unsigned long lastPosTime = 0; // Time tracking for position control loop
unsigned long pulseInterval = 0;
double controlLoopDuration = 0.0;

unsigned long lastControlTime = 0;

const int pulsesPerRevolution = 4096;  // Number of encoder pulses per revolution
volatile bool lastAState = LOW;  // Store the last state of A

 // store potentiometer absolute location

volatile bool indexInterrupt = false; // boolean flag to indicate 0 deg index interrupt triggered

// Direction flag (1 for clockwise, -1 for counterclockwise)
volatile int direction = 1; 

double setpoint = 4;  // Initial setpoint speed, max 5 Hz 
double setpointRatio = 1.3; // Ratio between primary and alternate speed setpoint for asymmetric flapping
double altSetpoint = setpoint / setpointRatio; // alternate setpoint when turning, scaled down from original setpoint for asymmetric flapping
double prevSetpoint = setpoint;
double Kp = 60;  // **SUBJECT TO TUNING**
double Ki = 110;  // **SUBJECT TO TUNING**
double Kd = 0.0;  // **SUBJECT TO TUNING**
double feedForward = 0.0; // feed-forward control signal
double kff = 1; // relative strength of feed-forward term (0 to 1) // **SUBJECT TO CHANGE**
double derivative = 0.0;
double adjustedKff = kff;
double adjustedKp = Kp;
double measuredSpeed = 0.0;  // Measured motor speed in Hz
int pwmValue = 0;  
double lastError = 0.0;
double lastPosition = 0.0;
double integralError = 0.0;  // Accumulated integral error
double integralWindupLimit = 25.0; // so it doesn't wind up too much relative to PWM limit (255)
bool useLowAdjustedGains;
bool useHighAdjustedGains;
bool turnsStarted = false;

// Variables for speed measurement noise reduction (moving average)
const int speedWindowSize = 5;  // Window size for moving average on speed
double speedWindow[speedWindowSize] = {0, 0, 0, 0, 0};  // Array to store last 3 speeds
int speedWindowIndex = 0; 

// **Position-controller-specific variables**
double posSetpoint = 60; 
double posKp = 0.3; // **SUBJECT TO TUNING**
double posKi = 0.05; // **SUBJECT TO TUNING**
double posKd = 0.08; // **SUBJECT TO TUNING**
double posIntegralError = 0;
double posIntegralWindupLimit = 50;
double posError = 0.0;

unsigned long startTime;
double interval = 5;    
unsigned long speedCalcInterval = 5;  // Speed calculation every 5 ms   

// This function uses one of the encoder's channels as an interrupt pin to constantly update the motor's position
void IRAM_ATTR updateEncoder() {
  encoderCount++;
}

// This function resets the motor's absolute position reading by using the encoder's Z-channel as an interrupt
void resetPosition() {
  indexInterrupt = true; // Reset position when index channel is triggered
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS

          type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  // ... (other OTA callbacks)

  ArduinoOTA.begin();


  // set PWM frequencies for motor pins from ~490 Hz (default) to 1 kHz
  analogWriteFrequency(18, 1000);
  analogWriteFrequency(19, 1000);


  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(POT, INPUT);

  myservo.attach(22); // servo is on pin 22 

  digitalWrite(MOTOR_IN1, LOW);
  analogWrite(MOTOR_IN2, 0);

  delay(1000);

  // Attach interrupt to the A and Z channels
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Z), resetPosition, RISING);

    // Titles for outputs
  Serial.println("Time Speed Setpoint PWM Integral AdjustedKff Position Ref kpAdjustment");

  lastPulseTime = millis();   // Initialize pulse time tracking
  lastSpeedCalcTime = millis(); // Initialize speed calculation time
  lastControlTime = millis(); // Initialize control loop time
  startTime = millis(); // Record start time

}

// This function creates a simple movement sequence by calling some of the controller functions.
//
// In this example, if the supplied setpoint ratio is not 1 (i.e., setpoint != altSetpoint), the turnUsingPSO() function is used to execute the turn.
// Otherwise, the straight() function is used to make the fish swim forward.
//
// After 8 seconds pass, the program will execute a final glide turn using the position controller. 
//
// Unlike the other controllers, the setPosition() function doesn't use the loop() function to for its control loop. Instead, it uses its own 
// its own while loop. When this while loop is broken, the stop() function is used to completely halt movement. 

void loop() {

  ArduinoOTA.handle();
  myservo.write(90); 

  unsigned long currentTime = millis();
  double potDegrees = (4095 - analogRead(POT)) * 330.0 / 4095.0;  // Calculate absolute position via potentiometer

  // *These movement controllers use the entire loop() function to repeat, and they each have their own if-statements to regulate the time interval*
  if (setpoint != altSetpoint) {
    turnUsingPSO(currentTime, potDegrees);
    // turnUsingFeedForward(currentTime, potDegrees); // Switch to this control strategy when desired
  } else {
    straight(currentTime, potDegrees);
  }

  // If 8 seconds pass after actively swimming (turning or straight), set the position to some angle to do a passive glide turn
  if ((startTime - millis()) > 8000) {
    bool setPositionBreak = true; // breaks position controller if eventually set to false
    setPosition(currentTime, setPositionBreak);
    stop(); // End the program using stop() function after the position is set
  }

}

// This function uses a combined PI and feed-forward controller to move the fish forward at a constant tail beat frequency setpoint
void straight(unsigned long currentTime, double potDegrees) {
  setpoint = primSetpoint;

  // Fixed control loop interval for motor control (PI)
  if (currentTime - lastControlTime >= interval) {
    lastControlTime = currentTime;

    // PI control calculations (unchanged)
    double error = setpoint - measuredSpeed;
    integralError += error * (interval / 1000.0); 
    integralError = constrain(integralError, -integralWindupLimit, integralWindupLimit);
    double derivative = (error - lastError) / (interval / 1000.0);

    feedForward = 37.48 + 3.67 * setpoint + 5.22 * pow(setpoint, 2);

    pwmValue = constrain(Kp * error + Ki * integralError + Kd * derivative + kff * feedForward, -255, 255);

    setMotorVoltage(pwmValue);

    lastError = error;
  }

  // Separate speed calculation based on pulse count over time
  if (currentTime - lastSpeedCalcTime >= speedCalcInterval) {
    // Calculate speed based on pulses counted in the last 100 ms (or chosen interval)
    double pulsesInInterval = encoderCount;  // Get the number of pulses since the last speed calculation
    encoderCount = 0;  // Reset pulse count for the next interval

    // Calculate the speed in Hz (revolutions per second)
    measuredSpeed = (pulsesInInterval / pulsesPerRevolution) / (speedCalcInterval / 1000.0);

    // Moving average for speed smoothing
    speedWindow[speedWindowIndex] = measuredSpeed;
    speedWindowIndex = (speedWindowIndex + 1) % speedWindowSize;
    double speedSum = 0.0;
    for (int i = 0; i < speedWindowSize; i++) {
      speedSum += speedWindow[i];
    }
    double smoothedSpeed = speedSum / speedWindowSize;

    // Print out the smoothed speed for debugging
    Serial.print(currentTime / 1000.0, 3);  // Time in seconds
    Serial.print(",");
    Serial.print(smoothedSpeed);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(pwmValue);
    Serial.print(",");
    Serial.print(setpoint - smoothedSpeed);
    Serial.print(",");
    Serial.print(Ki * integralError);
    Serial.print(",");
    Serial.print((4095 - analogRead(POT)) * 330.0 / 4095.0);
    Serial.print(",");
    Serial.println(0);
    // Reset time for the next speed calculation
    lastSpeedCalcTime = currentTime;
  }
}

// This function uses a combined gain-scheduled PI and feed-forward controller to turn the fish via asymmetric flapping with two setpoint tail beat frequencies
void turnUsingFeedForward(unsigned long currentTime, double potDegrees) {

  if (potDegrees > 0 && potDegrees <= 180 || indexInterrupt) {
    setpoint = altSetpoint;
  } else {
    setpoint = primSetpoint;
  }

  if (setpoint != prevSetpoint) {
    integralError = 0;  // Reset the integral term
    useLowAdjustedGains = false;   // No gain scheduling for initial start-up
    useHighAdjustedGains = false;
    
    if (setpoint < prevSetpoint) {  // Enable gain scheduling for when when setpoint decreases
      useLowAdjustedGains = true;
      useHighAdjustedGains = false;
    } else if (setpoint > prevSetpoint) {     // Enable gain scheduling for when when setpoint increases
      useLowAdjustedGains = false;
      useHighAdjustedGains = true;
    } 
    prevSetpoint = setpoint;  // Update the previous setpoint
  }


  // Continuous position-based gain calculations
  if (potDegrees > 0 && potDegrees < 180) {         // adjusted gains for setpoint **DECREASE**
    // Calculate adjusted gains based on position
    double positionAdjustedKp = Kp * ((pow((1 - 0.2), (1 / 4.0) * (potDegrees + 4))) + 1);   
    double positionAdjustedKff;

    // Calculate adjustedKff with zero constraint near 0 degrees
    if (potDegrees > 0 && potDegrees <= 2.01) {
        positionAdjustedKff = 0;
    } else {
        positionAdjustedKff = kff * (-1 / pow((potDegrees - 1), 4.0 / 5.0) + 1.15); 
    }

    // Apply adjusted gains only if gain scheduling is enabled
    if (useLowAdjustedGains) {
      adjustedKp = positionAdjustedKp;
      adjustedKff = positionAdjustedKff;
    } else {
      // Use default gains if not in gain scheduling mode
      adjustedKp = Kp;
      adjustedKff = kff;
    }
  } else if (potDegrees > 180 && potDegrees < 330) { // adjusted gains for *SETPOINT INCREASE*

    // Calculate adjusted gains based on position
    double positionAdjustedKp = Kp * ((pow((1 - 0.08), (1 / 4.0) * (potDegrees - 145))) + 1); 
    double positionAdjustedKff = kff;  

    // Apply adjusted gains only if gain scheduling is enabled
    if (useHighAdjustedGains) {
      adjustedKp = positionAdjustedKp;
      adjustedKff = positionAdjustedKff;
    } else {
      // Use default gains if not in gain scheduling mode
      adjustedKp = Kp;
      adjustedKff = kff;
    }

  } else {
      // Default values when out of range or gain scheduling is not needed
      adjustedKp = Kp;
      adjustedKff = kff;
  }

  // Adjusted control interval to account for processing time
  double adjustedInterval = interval - (controlLoopDuration / 1000.0);  // Convert to ms

  // ** MAIN CONTROL LOOP **
  if (currentTime - lastControlTime >= adjustedInterval) {
    unsigned long loopStartTime = micros();  // Capture loop start time

    // Setpoint correction if potentiometer gives incorrect number in blind spot
    // (Corrects setpoint & position if potentiometer doesn't jump straight back to 0 after 330, only when the setpoint is alternating)
    if (lastPosition == 330 && potDegrees > 0 && potDegrees < 330) {
      potDegrees = 330;
      if (turnsStarted) {
          setpoint = 4;
      }
    }

    // Update lastControlTime to ensure consistent timing
    lastControlTime += interval;  // Maintain a consistent interval regardless of drift

    indexInterrupt = false;

    // PID control calculations
    double error = setpoint - measuredSpeed;
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralWindupLimit, integralWindupLimit);
    derivative = (error - lastError) / (interval / 1000.0);

    feedForward = 37.48 + 3.67 * setpoint + 5.22 * pow(setpoint, 2);

    pwmValue = constrain(adjustedKp * error + Ki * integralError + Kd * derivative + adjustedKff * feedForward, -255, 255);
    setMotorVoltage(pwmValue);

    lastError = error;

    // Calculate control loop duration in microseconds and convert to milliseconds
    controlLoopDuration = micros() - loopStartTime;
  }

  // Speed measurement and logging at fixed intervals
  if (currentTime - lastSpeedCalcTime >= speedCalcInterval) {
    double pulsesInInterval = encoderCount;
    encoderCount = 0;

    measuredSpeed = (pulsesInInterval / pulsesPerRevolution) / (speedCalcInterval / 1000.0);

    // Moving average for speed smoothing
    speedWindow[speedWindowIndex] = measuredSpeed;
    speedWindowIndex = (speedWindowIndex + 1) % speedWindowSize;
    double speedSum = 0.0;
    for (int i = 0; i < speedWindowSize; i++) {
      speedSum += speedWindow[i];
    }
    double smoothedSpeed = speedSum / speedWindowSize;

    // Log data
    Serial.print(currentTime / 1000.0, 3);  // Time in seconds
    Serial.print(",");
    Serial.print(smoothedSpeed);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(pwmValue);
    // Serial.print(",");
    // Serial.print(setpoint - smoothedSpeed);
    Serial.print(",");
    Serial.print(Ki * integralError);
    // Serial.print(",");
    // Serial.print(adjustedKff * feedForward);
    Serial.print(",");
    Serial.print(adjustedKff);
    // Serial.print(",");
    // Serial.print(feedForward);
    Serial.print(",");
    Serial.print(potDegrees);
    // Serial.print(",");
    // Serial.print(Kd * derivative);
    Serial.print(",");
    Serial.print(0);
    Serial.print(",");
    Serial.println(adjustedKp / Kp);

    lastSpeedCalcTime = currentTime;
    lastPosition = potDegrees;
  }
}

// This function uses a PI controller with particle-swarm-optimized gains to turn the fish via asymmetric flapping
void turnUsingPSO(unsigned long currentTime, double potDegrees) {
  // Change setpoint based on potentiometer position
  double psoKp = 47.51;
  double psoKi = 1429.61;
  
  if ((currentTime - startTime) >= 2000) {
    turnsStarted = true;

    if (potDegrees > 0 && potDegrees <= 180) {
      setpoint = altSetpoint;
    } else {
      setpoint = primSetpoint;
    }

    if (setpoint != prevSetpoint) {
      // integralError = 0;  // Reset the integral term
      prevSetpoint = setpoint;
    }
  }

  double adjustedInterval = interval - (controlLoopDuration / 1000.0);  // Convert to ms

  // Control loop with exact adjusted interval calculation
  if (currentTime - lastControlTime >= interval) {
    unsigned long loopStartTime = micros();

    // Setpoint correction if potentiometer gives incorrect number in blind spot
    // (Corrects setpoint & position if potentiometer doesn't jump straight back to 0 after 330, only when the setpoint is alternating)
    if (lastPosition == 330 && potDegrees > 0 && potDegrees < 330) {
      potDegrees = 330;
      if (turnsStarted) {
          setpoint = 4;
      }
    }

    // Update lastControlTime to ensure consistent timing
    lastControlTime += interval;  // Maintain a consistent interval regardless of drift

    double error = setpoint - measuredSpeed;
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralWindupLimit, integralWindupLimit);
    derivative = (error - lastError) / (interval / 1000.0);

    pwmValue = constrain(psoKp * error + psoKi * integralError, -255, 255);
    setMotorVoltage(pwmValue);

    lastError = error;

    // Calculate control loop duration
    controlLoopDuration = micros() - loopStartTime;

  }

  // Speed measurement and logging
  if (currentTime - lastSpeedCalcTime >= speedCalcInterval) {
    double pulsesInInterval = encoderCount;
    encoderCount = 0;

    measuredSpeed = (pulsesInInterval / pulsesPerRevolution) / (speedCalcInterval / 1000.0);

    // Moving average for speed smoothing
    speedWindow[speedWindowIndex] = measuredSpeed;
    speedWindowIndex = (speedWindowIndex + 1) % speedWindowSize;
    double speedSum = 0.0;
    for (int i = 0; i < speedWindowSize; i++) {
      speedSum += speedWindow[i];
    }
    double smoothedSpeed = speedSum / speedWindowSize;

    // Log data
    Serial.print(currentTime / 1000.0, 3);
    Serial.print(",");
    Serial.print(smoothedSpeed);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(pwmValue);
    Serial.print(",");
    Serial.print(Ki * integralError);
    Serial.print(",");
    Serial.print(kff);
    Serial.print(",");
    Serial.print(potDegrees);
    Serial.print(",");
    Serial.println(0);


    lastSpeedCalcTime = currentTime;
    lastPosition = potDegrees;
  }
}

// PSO V2, TURN USING COMBINED SPEED & CONTROL INTERVALS
void turnUsingPSOcombined(unsigned long currentTime, double potDegrees) {
    // Change setpoint based on potentiometer position
  if ((currentTime - startTime) >= 2000) {
    turnsStarted = true;

    if (potDegrees > 0 && potDegrees <= 180) {
      setpoint = altSetpoint;
    } else {
      setpoint = primSetpoint;
    }

    if (setpoint != prevSetpoint) {
      // integralError = 0;  // Reset the integral term
      prevSetpoint = setpoint;
    }
  }

  double adjustedInterval = interval - (controlLoopDuration / 1000.0);  // Convert to ms

  // Control loop with exact adjusted interval calculation
  if (currentTime - lastControlTime >= interval) {
    unsigned long loopStartTime = micros();

    // Setpoint correction if potentiometer gives incorrect number in blind spot
    // (Corrects setpoint & position if potentiometer doesn't jump straight back to 0 after 330, only when the setpoint is alternating)
    if (lastPosition == 330 && potDegrees > 0 && potDegrees < 330) {
      potDegrees = 330;
      if (turnsStarted) {
          setpoint = 4;
      }
    }

    // Update lastControlTime to ensure consistent timing
    lastControlTime += interval;  // Maintain a consistent interval regardless of drift

    double error = setpoint - measuredSpeed;
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralWindupLimit, integralWindupLimit);
    derivative = (error - lastError) / (interval / 1000.0);

    feedForward = 38.5 + 3.24 * setpoint + 5.47 * pow(setpoint, 2);

    pwmValue = constrain(Kp * error + Ki * integralError + Kd * derivative + kff * feedForward, -255, 255);
    setMotorVoltage(pwmValue);

    lastError = error;


    // beginning of speed measurement

    double pulsesInInterval = encoderCount;
    encoderCount = 0;

    measuredSpeed = (pulsesInInterval / pulsesPerRevolution) / (speedCalcInterval / 1000.0);

    // Moving average for speed smoothing
    speedWindow[speedWindowIndex] = measuredSpeed;
    speedWindowIndex = (speedWindowIndex + 1) % speedWindowSize;
    double speedSum = 0.0;
    for (int i = 0; i < speedWindowSize; i++) {
      speedSum += speedWindow[i];
    }
    double smoothedSpeed = speedSum / speedWindowSize;

    // Log data
    Serial.print(currentTime / 1000.0, 3);
    Serial.print(",");
    Serial.print(smoothedSpeed);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(pwmValue);
    Serial.print(",");
    Serial.print(Ki * integralError);
    Serial.print(",");
    Serial.print(kff);
    Serial.print(",");
    Serial.print(potDegrees);
    Serial.print(",");
    Serial.println(0);

    // Calculate control loop duration
    controlLoopDuration = micros() - loopStartTime;
    lastSpeedCalcTime = currentTime;
    lastPosition = potDegrees;
  }
}

// This function contains the position PID controller to be used for glide turns by setting the motor position to some angle.
void setPosition(unsigned long currentTime, bool setPositionBreak) {
  double lastPosError = 0.0;
  double posDerivative = 0.0;
  
  while (setPositionBreak = true) {
    double position = analogRead(POT)*330.0/4095.0;

    // Position control loop
    if (currentTime - lastPosTime >= interval) {
      
      // error preventing crossover of 0/360 deg mark, avoiding blindspot
      posError = posSetpoint - position;

      // If the error is greater than 180 degrees, wrap it to the equivalent negative angle
      if (posError > 180) {
        posError -= 360;
      } else if (posError < -180) {
      posError += 360;
      }

      // Update integral term
      posIntegralError = constrain(posIntegralError += posError, -posIntegralWindupLimit, posIntegralWindupLimit);

      posDerivative = (posError - lastPosError)/(interval/1000);

      pwmValue = constrain(posKp * posError + posKi * posIntegralError + posKd * posDerivative, -255, 255);

      setMotorVoltage(pwmValue);

      Serial.print("Time: ");
      Serial.print(currentTime / 1000.0, 3);  // Time in seconds
      Serial.print(" , Position: ");
      Serial.print(position);
      Serial.print(" , PWM: ");
      Serial.print(pwmValue);
      Serial.print(" , Error: ");
      Serial.println(posError); 


      lastPosError = posError;
      lastPosTime = currentTime;
    }
    
    // Breaks the position controller if the motor reaches the setpoint within +/- 15 degrees
    if (position>(posSetpoint-15) && position<(posSetpoint+15)) {
      setpointPositionBreak = false;
    }
  }
  return;
}

// This function uses an inputted PWM value (positive or negative) to rotate the DC motor at some strength/direction
void setMotorVoltage(int pwmValue) {
  if (pwmValue < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, abs(pwmValue));  // Reverse motor
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, pwmValue);  // Forward motor
  }
}

// This function is used to fully halt the program after using the position controller glide turn
void stop() {
  while(1) { /* stop the program */ }
}


