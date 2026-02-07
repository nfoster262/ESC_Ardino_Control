#include <Arduino.h>
#include <Servo.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
const int ESC_PIN = 9;        // Output pin for ESC signal
const int SENSOR_PIN = 2;     // Input pin for RPM sensor (Must be interrupt capable: 2 or 3 on Uno)

// Motor & Sensor Settings
// If using a reflective tape and optical sensor, PULSES_PER_REV = 1.
// If using a phase sensor on a 14-pole motor, PULSES_PER_REV = 7 (14/2).
const int PULSES_PER_REV = 1; 

// PID Control Constants (Tune these for your specific motor setup)
// Start with small values to prevent violent oscillations.
double Kp = 0.01;   // Proportional Gain
double Ki = 0.05;   // Integral Gain
double Kd = 0.00;   // Derivative Gain

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
Servo esc;
volatile unsigned long pulseCount = 0; // Volatile for interrupt access
unsigned long lastLoopTime = 0;
double integralError = 0;
double lastError = 0;
int targetRPM = 2000; // Initial target RPM

// ---------------------------------------------------------------------------
// Interrupt Service Routine
// ---------------------------------------------------------------------------
void onSensorPulse() {
  pulseCount++;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); // Higher baud rate for faster debug output
  Serial.println("System Initializing...");

  // Setup ESC
  esc.attach(ESC_PIN);
  // Send minimum throttle to arm the ESC
  esc.writeMicroseconds(1000);
  
  Serial.println("Arming ESC (waiting 3 seconds)...");
  delay(3000); // Wait for ESC to recognize the low signal
  Serial.println("ESC Armed.");

  // Setup Sensor
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onSensorPulse, RISING);
}

// ---------------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------------
void loop() {
  // Run the control loop at a fixed interval (e.g., every 100ms)
  if (millis() - lastLoopTime >= 100) {
    unsigned long currentTime = millis();
    double dt = (currentTime - lastLoopTime) / 1000.0; // Delta time in seconds
    lastLoopTime = currentTime;

    // 1. Read and Reset Pulse Counter safely
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // 2. Calculate Current RPM
    // RPM = (Pulses / PulsesPerRev) * (60 seconds / dt)
    double currentRPM = (double)pulses / PULSES_PER_REV * 60.0 / dt;

    // 3. PID Calculations
    double error = targetRPM - currentRPM;
    
    // Accumulate Integral (with simple anti-windup clamping)
    integralError += error * dt;
    integralError = constrain(integralError, -10000, 10000); // Prevent integral from growing too large

    // Calculate Derivative
    double derivative = (error - lastError) / dt;

    // Calculate PID Output
    // The output represents the throttle adjustment needed above the base.
    double output = (Kp * error) + (Ki * integralError) + (Kd * derivative);

    // 4. Map to ESC Signal
    // Base signal is 1000us. We add the PID output to it.
    int pwmSignal = 1000 + (int)output;
    
    // Safety limits for standard ESC (1000us to 2000us)
    pwmSignal = constrain(pwmSignal, 1000, 2000);

    // 5. Write to Motor
    // Safety check: if target is 0, force stop
    if (targetRPM == 0) {
        pwmSignal = 1000;
        integralError = 0; // Reset integral when stopped
    }
    esc.writeMicroseconds(pwmSignal);
    
    // Update last error
    lastError = error;

    // 6. Serial Plotter / Monitor Output
    Serial.print("Target:");
    Serial.print(targetRPM);
    Serial.print(" RPM:");
    Serial.print(currentRPM);
    Serial.print(" PWM:");
    Serial.println(pwmSignal);
  }

  // Check for Serial input to change RPM dynamically
  if (Serial.available() > 0) {
    int newTarget = Serial.parseInt();
    // Consume the rest of the buffer (newlines etc)
    while(Serial.available()) Serial.read();
    
    if (newTarget >= 0) {
      targetRPM = newTarget;
      Serial.print("New Target RPM set to: ");
      Serial.println(targetRPM);
    }
  }
}