

#include <Arduino.h>
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
// Motor and encoder pin definitions
#define BUZZER 13
#define LED 13
#define PWM_L 6
#define PWM_R 5
#define DIR_L1 8
#define DIR_L2 7
#define DIR_R1 3
#define DIR_R2 4
#define SPD_INT_R 11
#define SPD_PUL_R 12
#define SPD_INT_L 10
#define SPD_PUL_L 9

// MPU6050 interrupt pin
#define MPU_INT_PIN 2

// MPU6050 DMP Settings & Variables
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;

// PID Settings & Variables
double input, output;
double setpoint = 180;  // Will be auto-calibrated
// More conservative initial PID gains to reduce oscillations
double Kp = 8.0, Ki = 0.05, Kd = 1.5;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
double fallLimit = 60;  // Increased to allow larger tilts for recovery

// Motor balancing variables
int motorLeftSpeed = 0;
int motorRightSpeed = 0;
int motorSpeedFactor = 1;  // Adjust for motor differences
bool isCalibrated = false;
double calibrationSum = 0;
int calibrationCount = 0;

// Fast tilt recovery variables
double lastInput = 180;
double tiltRate = 0;
unsigned long lastTime = 0;

// Output smoothing and rate limiting
double lastOutput = 0;            // last PID output
const double maxDeltaPerLoop = 10; // slower rate limiting for smoother transitions
const int minStartSpeed = 30;      // even lower minimum for gentler start
const int maxNormalSpeed = 60;     // much lower cap for smoother operation
const double deadband = 0.5;       // degrees around setpoint with no actuation

// Function declarations
void moveForward(int power);
void moveBackward(int power);
void stopMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void controlMotors(int baseSpeed);

// Interrupt Function
void dmpDataReady() { mpuInterrupt = true; }

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(BUZZER, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(PWM_L, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(DIR_L1, OUTPUT);
    pinMode(DIR_L2, OUTPUT);
    pinMode(DIR_R1, OUTPUT);
    pinMode(DIR_R2, OUTPUT);
    pinMode(SPD_INT_R, INPUT);
    pinMode(SPD_PUL_R, INPUT);
    pinMode(SPD_INT_L, INPUT);
    pinMode(SPD_PUL_L, INPUT);
    digitalWrite(BUZZER, LOW);
    digitalWrite(LED, LOW);
    stopMotors();

    // MPU6050 setup
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    // Calibrate MPU6050
    mpu.setXGyroOffset(30);
    mpu.setYGyroOffset(-32);
    mpu.setZGyroOffset(47);
    mpu.setZAccelOffset(1170);
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(5);  // slightly slower to reduce noise sensitivity
        pid.SetOutputLimits(-255, 255);
        Serial.println("DMP ready. Balancing mode engaged.");
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Log the requested speed
    Serial.print(" | RAW_L: ");
    Serial.print(leftSpeed);
    Serial.print(" | RAW_R: ");
    Serial.print(rightSpeed);
    
    // Constrain speeds to valid range
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // REVERSED MOTOR CONTROL - Try this if robot moves wrong direction
    // Left motor control (REVERSED)
    if (leftSpeed > 0) {
        digitalWrite(DIR_L1, LOW);  // REVERSED
        digitalWrite(DIR_L2, HIGH); // REVERSED
        analogWrite(PWM_L, leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(DIR_L1, HIGH); // REVERSED
        digitalWrite(DIR_L2, LOW);  // REVERSED
        analogWrite(PWM_L, abs(leftSpeed));
    } else {
        digitalWrite(DIR_L1, LOW);
        digitalWrite(DIR_L2, LOW);
        analogWrite(PWM_L, 0);
    }
    
    // Right motor control (REVERSED)
    if (rightSpeed > 0) {
        digitalWrite(DIR_R1, LOW);  // REVERSED
        digitalWrite(DIR_R2, HIGH); // REVERSED
        analogWrite(PWM_R, rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(DIR_R1, HIGH); // REVERSED
        digitalWrite(DIR_R2, LOW);  // REVERSED
        analogWrite(PWM_R, abs(rightSpeed));
    } else {
        digitalWrite(DIR_R1, LOW);
        digitalWrite(DIR_R2, LOW);
        analogWrite(PWM_R, 0);
    }
}

void controlMotors(int baseSpeed) {
    // For balancing robot: motors need to spin in opposite directions
    // to make robot move forward/backward rather than turn
    int leftSpeed = baseSpeed;
    int rightSpeed = -baseSpeed * motorSpeedFactor;  // Opposite direction
    
    setMotorSpeeds(leftSpeed, rightSpeed);
}

void moveForward(int power) {
    setMotorSpeeds(power, power);
}

void moveBackward(int power) {
    setMotorSpeeds(-power, -power);
}

void stopMotors() {
    setMotorSpeeds(0, 0);
}

void loop() {
    if (!dmpReady) return;
    
    // Check for FIFO overflow
    fifoCount = mpu.getFIFOCount();
    if (!mpuInterrupt && fifoCount < packetSize) return;
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return;
    }
    
    if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // Convert to degrees and adjust - using roll axis instead of pitch
        input = ypr[2] * 180 / M_PI + 180;
        
        // Calculate tilt rate for fast disturbance detection
        unsigned long currentTime = millis();
        if (lastTime > 0) {
            double deltaTime = (currentTime - lastTime) / 1000.0;  // Convert to seconds
            if (deltaTime > 0) {
                tiltRate = abs(input - lastInput) / deltaTime;  // degrees per second
            }
        }
        lastInput = input;
        lastTime = currentTime;
        
        // Auto-calibrate setpoint for first 20 readings (much faster)
        if (!isCalibrated) {
            calibrationSum += input;
            calibrationCount++;
            
            if (calibrationCount >= 20) {  // Reduced from 100 to 20
                setpoint = calibrationSum / calibrationCount;
                isCalibrated = true;
                pid.SetTunings(Kp, Ki, Kd);  // Reset PID after calibration
                Serial.print("Calibrated setpoint: ");
                Serial.println(setpoint);
                delay(500);  // Reduced delay from 2000ms to 500ms
            } else {
                Serial.print("Calibrating... ");
                Serial.print(calibrationCount);
                Serial.print("/20 - Current: ");
                Serial.println(input);
                stopMotors();
                return;
            }
        }
        
        // Calculate PID output
        pid.Compute();

        // Apply deadband around setpoint to reduce micro-corrections
        double rawError = input - setpoint;
        if (abs(rawError) < deadband) {
            output = 0;
        }

        // Smooth and rate-limit output changes to avoid sudden flips
        double desired = output;
        double delta = desired - lastOutput;
        if (delta > maxDeltaPerLoop) delta = maxDeltaPerLoop;
        if (delta < -maxDeltaPerLoop) delta = -maxDeltaPerLoop;
        output = lastOutput + delta;
        lastOutput = output;
        
        // Debug output
        Serial.print("Pitch: ");
        Serial.print(input, 2);
        Serial.print(" | Setpoint: ");
        Serial.print(setpoint, 2);
        Serial.print(" | Error: ");
        Serial.print(input - setpoint, 2);
        Serial.print(" | Output: ");
        Serial.println(output, 2);
        
        // Check if robot has fallen over
        double error = abs(input - setpoint);
        if (error < fallLimit) {
            // Robot is upright, apply balancing force
            if (abs(output) > 1.0) {  // small deadband to reduce chatter
                int motorSpeed;
                
                // Use PID output with gentler limits during normal operation
                if (error > 15 || tiltRate > 30) {
                    // Large error or fast tilt - allow higher speeds for recovery
                    motorSpeed = constrain((int)abs(output), 0, 80);
                } else {
                    // Normal operation - cap to avoid violent swings
                    motorSpeed = constrain((int)abs(output), 0, maxNormalSpeed);
                }
                
                // Gradual minimum speed ramp to overcome stiction without jumps
                if (abs(output) > 5 && motorSpeed < minStartSpeed) {
                    motorSpeed = minStartSpeed;
                }
                
                // Apply motor control and debug
                if (output > 0) {
                    controlMotors(motorSpeed);  // Move forward
                    Serial.print(" | MOTOR_CMD: FORWARD ");
                    Serial.print(motorSpeed);
                } else {
                    controlMotors(-motorSpeed);  // Move backward
                    Serial.print(" | MOTOR_CMD: BACKWARD ");
                    Serial.print(motorSpeed);
                }
                
                Serial.print(" | Error: ");
                Serial.print(error, 1);
                Serial.print(" | TiltRate: ");
                Serial.print(tiltRate, 1);
                Serial.print(" | MotorSpeed: ");
                Serial.print((output > 0) ? motorSpeed : -motorSpeed);
            } else {
                stopMotors();
                Serial.print(" | Motors: STOPPED");
            }
        } else {
            // Robot has fallen, stop motors
            stopMotors();
            Serial.println("Robot fallen - stopping motors");
        }
    }
}
