

#include <Arduino.h>
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <SoftwareSerial.h>
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

// Bluetooth module pins (HC-05)
#define BT_RX_PIN A0  // Connect to HC-05 TX
#define BT_TX_PIN A1  // Connect to HC-05 RX
#define BT_DISABLE_PIN A2  // Ground this pin to disable Bluetooth (for uploads)

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
double setpoint = 180; // Will be auto-calibrated
// Conservative PID gains to prevent oscillation - gentler response
double Kp = 6.0, Ki = 0.01, Kd = 0.8;  // Much gentler gains for stability
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
double fallLimit = 60; // Increased to allow larger tilts for recovery

// Motor balancing variables
int motorLeftSpeed = 0;
int motorRightSpeed = 0;
int motorSpeedFactor = 1; // Adjust for motor differences
bool isCalibrated = false;
double calibrationSum = 0;
int calibrationCount = 0;

// Fast tilt recovery variables
double lastInput = 180;
double tiltRate = 0;
unsigned long lastTime = 0;

// Output smoothing and rate limiting
double lastOutput = 0;             // last PID output
const double maxDeltaPerLoop = 5;  // Much slower rate limiting for gentle operation
const int minStartSpeed = 20;      // Very low minimum for gentle start
const int maxNormalSpeed = 40;     // Much lower max speed to prevent violent swings
const double deadband = 0.8;       // Larger deadband to reduce sensitivity

// Bluetooth module setup
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

// Bluetooth control variables
double steeringCommand = 0;     // -100 to +100 (left to right)
double speedCommand = 0;        // -100 to +100 (backward to forward)
bool bluetoothEnabled = true;
String incomingCommand = "";
bool commandComplete = false;

// Telemetry variables
unsigned long lastTelemetryTime = 0;
const unsigned long telemetryInterval = 100; // Send telemetry every 100ms

// Function declarations
void moveForward(int power);
void moveBackward(int power);
void stopMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void controlMotors(int baseSpeed);
void controlMotorsAdvanced(double balanceOutput, double steeringCmd, double speedCmd);
void processBluetooth();
void sendTelemetry();

// Interrupt Function
void dmpDataReady() { mpuInterrupt = true; }

void setup()
{
    Serial.begin(115200);
    
    // Check if Bluetooth should be disabled (for firmware uploads)
    pinMode(BT_DISABLE_PIN, INPUT_PULLUP);
    bool bt_disabled = (digitalRead(BT_DISABLE_PIN) == LOW);
    
    if (bt_disabled) {
        bluetoothEnabled = false;
        Serial.println("Bluetooth disabled for upload mode");
    } else {
        bluetooth.begin(9600);  // Standard HC-05 baud rate
        bluetoothEnabled = true;
        Serial.println("Bluetooth enabled");
    }
    
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
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(5); // slightly slower to reduce noise sensitivity
        pid.SetOutputLimits(-255, 255);
        
        if (bluetoothEnabled) {
            Serial.println("DMP ready. Bluetooth balancing mode engaged.");
            bluetooth.println("Robot ready for Bluetooth control");
        } else {
            Serial.println("DMP ready. Upload mode - Bluetooth disabled.");
        }
        
        // Signal ready with LED
        for(int i = 0; i < 3; i++) {
            digitalWrite(LED, HIGH);
            delay(200);
            digitalWrite(LED, LOW);
            delay(200);
        }
    }
    else
    {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        if (bluetoothEnabled) {
            bluetooth.println("Robot initialization failed");
        }
    }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
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
    if (leftSpeed > 0)
    {
        digitalWrite(DIR_L1, LOW);  // REVERSED
        digitalWrite(DIR_L2, HIGH); // REVERSED
        analogWrite(PWM_L, leftSpeed);
    }
    else if (leftSpeed < 0)
    {
        digitalWrite(DIR_L1, HIGH); // REVERSED
        digitalWrite(DIR_L2, LOW);  // REVERSED
        analogWrite(PWM_L, abs(leftSpeed));
    }
    else
    {
        digitalWrite(DIR_L1, LOW);
        digitalWrite(DIR_L2, LOW);
        analogWrite(PWM_L, 0);
    }

    // Right motor control (REVERSED)
    if (rightSpeed > 0)
    {
        digitalWrite(DIR_R1, LOW);  // REVERSED
        digitalWrite(DIR_R2, HIGH); // REVERSED
        analogWrite(PWM_R, rightSpeed);
    }
    else if (rightSpeed < 0)
    {
        digitalWrite(DIR_R1, HIGH); // REVERSED
        digitalWrite(DIR_R2, LOW);  // REVERSED
        analogWrite(PWM_R, abs(rightSpeed));
    }
    else
    {
        digitalWrite(DIR_R1, LOW);
        digitalWrite(DIR_R2, LOW);
        analogWrite(PWM_R, 0);
    }
}

void controlMotors(int baseSpeed)
{
    // For balancing robot: motors need to spin in opposite directions
    // to make robot move forward/backward rather than turn
    int leftSpeed = baseSpeed;
    int rightSpeed = -baseSpeed * motorSpeedFactor; // Opposite direction
    
    setMotorSpeeds(leftSpeed, rightSpeed);
}

void moveForward(int power)
{
    setMotorSpeeds(power, power);
}

void moveBackward(int power)
{
    setMotorSpeeds(-power, -power);
}

void stopMotors()
{
    setMotorSpeeds(0, 0);
}

// ENHANCED: Motor control for balancing + steering (with opposite-direction logic)
void controlMotorsAdvanced(double balanceOutput, double steeringCmd, double speedCmd)
{
    // Base speed from PID balance controller
    double baseSpeed = balanceOutput;
    
    // Add forward/backward speed command
    baseSpeed += speedCmd;
    
    // Calculate differential speeds for turning with opposite-direction base logic
    // For your robot: motors spin opposite directions for forward/back
    double leftSpeed = baseSpeed - (steeringCmd * 0.6);     // Left wheel
    double rightSpeed = -baseSpeed + (steeringCmd * 0.6);   // Right wheel (opposite base + steering)
    
    setMotorSpeeds((int)leftSpeed, (int)rightSpeed);
    
    // Debug output
    Serial.print(" | Bal: "); Serial.print(balanceOutput, 1);
    Serial.print(" | Steer: "); Serial.print(steeringCmd, 1);
    Serial.print(" | Spd: "); Serial.print(speedCmd, 1);
}

// Non-blocking Bluetooth command processing
void processBluetooth()
{
    if (!bluetoothEnabled) return;
    
    // Read incoming data without blocking
    while (bluetooth.available() > 0)
    {
        char inChar = (char)bluetooth.read();
        if (inChar == '\n' || inChar == '\r')
        {
            if (incomingCommand.length() > 0)
            {
                commandComplete = true;
                break;
            }
        }
        else
        {
            incomingCommand += inChar;
        }
    }
    
    // Process complete commands
    if (commandComplete)
    {
        incomingCommand.trim();
        
        // Command processing
        if (incomingCommand.startsWith("SPEED:"))
        {
            speedCommand = incomingCommand.substring(6).toFloat();
            speedCommand = constrain(speedCommand, -50, 50);
            bluetooth.println("Speed set to " + String(speedCommand));
        }
        else if (incomingCommand.startsWith("STEER:"))
        {
            steeringCommand = incomingCommand.substring(6).toFloat();
            steeringCommand = constrain(steeringCommand, -50, 50);
            bluetooth.println("Steering set to " + String(steeringCommand));
        }
        else if (incomingCommand == "STOP")
        {
            speedCommand = 0;
            steeringCommand = 0;
            bluetooth.println("All commands stopped");
        }
        else if (incomingCommand.startsWith("PID:"))
        {
            // Format: "PID:Kp,Ki,Kd" e.g., "PID:12.0,0.015,2.5"
            int comma1 = incomingCommand.indexOf(',');
            int comma2 = incomingCommand.indexOf(',', comma1 + 1);
            
            if (comma1 > 0 && comma2 > 0)
            {
                double newKp = incomingCommand.substring(4, comma1).toFloat();
                double newKi = incomingCommand.substring(comma1 + 1, comma2).toFloat();
                double newKd = incomingCommand.substring(comma2 + 1).toFloat();
                
                if (newKp > 0 && newKi >= 0 && newKd >= 0)
                {
                    Kp = newKp;
                    Ki = newKi;
                    Kd = newKd;
                    pid.SetTunings(Kp, Ki, Kd);
                    
                    bluetooth.print("PID Updated: Kp="); bluetooth.print(Kp, 3);
                    bluetooth.print(", Ki="); bluetooth.print(Ki, 3);
                    bluetooth.print(", Kd="); bluetooth.println(Kd, 3);
                }
                else
                {
                    bluetooth.println("Invalid PID values");
                }
            }
            else
            {
                bluetooth.println("PID format: PID:Kp,Ki,Kd");
            }
        }
        else if (incomingCommand == "STATUS")
        {
            bluetooth.print("Pitch: "); bluetooth.print(input, 2);
            bluetooth.print(" | Setpoint: "); bluetooth.print(setpoint, 2);
            bluetooth.print(" | Error: "); bluetooth.print(input - setpoint, 2);
            bluetooth.print(" | PID: "); bluetooth.print(Kp); bluetooth.print(",");
            bluetooth.print(Ki, 3); bluetooth.print(","); bluetooth.println(Kd);
        }
        else if (incomingCommand == "HELP")
        {
            bluetooth.println("Commands:");
            bluetooth.println("SPEED:value (-50 to 50)");
            bluetooth.println("STEER:value (-50 to 50)");
            bluetooth.println("PID:Kp,Ki,Kd");
            bluetooth.println("STOP");
            bluetooth.println("STATUS");
        }
        else
        {
            bluetooth.println("Unknown command. Send HELP for commands.");
        }
        
        // Reset for next command
        incomingCommand = "";
        commandComplete = false;
    }
    
    // Gradual decay of commands to prevent runaway
    steeringCommand *= 0.97;  // 3% decay per loop
    speedCommand *= 0.98;     // 2% decay per loop
    
    if (abs(steeringCommand) < 1.0) steeringCommand = 0;
    if (abs(speedCommand) < 1.0) speedCommand = 0;
}

// Send telemetry data to Python client
void sendTelemetry()
{
    unsigned long currentTime = millis();
    if (currentTime - lastTelemetryTime >= telemetryInterval)
    {
        bluetooth.print("TEL:");
        bluetooth.print(input, 2); bluetooth.print(",");
        bluetooth.print(setpoint, 2); bluetooth.print(",");
        bluetooth.print(input - setpoint, 2); bluetooth.print(",");
        bluetooth.print(output, 1); bluetooth.print(",");
        bluetooth.print(tiltRate, 1); bluetooth.print(",");
        bluetooth.print(steeringCommand, 1); bluetooth.print(",");
        bluetooth.print(speedCommand, 1);
        bluetooth.println();
        
        lastTelemetryTime = currentTime;
    }
}

void loop()
{
    if (!dmpReady)
        return;

    // Process Bluetooth commands (non-blocking)
    processBluetooth();
    
    // Check for FIFO overflow
    fifoCount = mpu.getFIFOCount();
    if (!mpuInterrupt && fifoCount < packetSize)
    {
        // Send telemetry while waiting for sensor data
        sendTelemetry();
        return;
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return;
    }

    if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert to degrees and adjust - using roll axis instead of pitch
        input = ypr[2] * 180 / M_PI + 180;

        // Calculate tilt rate for fast disturbance detection
        unsigned long currentTime = millis();
        if (lastTime > 0)
        {
            double deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
            if (deltaTime > 0)
            {
                tiltRate = abs(input - lastInput) / deltaTime; // degrees per second
            }
        }
        lastInput = input;
        lastTime = currentTime;

        // Auto-calibrate setpoint for first 20 readings (much faster)
        if (!isCalibrated)
        {
            calibrationSum += input;
            calibrationCount++;

            if (calibrationCount >= 20)
            { // Reduced from 100 to 20
                setpoint = calibrationSum / calibrationCount;
                isCalibrated = true;
                pid.SetTunings(Kp, Ki, Kd); // Reset PID after calibration
                Serial.print("Calibrated setpoint: ");
                Serial.println(setpoint);
                if (bluetoothEnabled) {
                    bluetooth.print("Calibrated setpoint: ");
                    bluetooth.println(setpoint);
                }
                delay(500); // Reduced delay from 2000ms to 500ms
            }
            else
            {
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
        if (abs(rawError) < deadband)
        {
            output = 0;
        }

        // Smooth and rate-limit output changes to avoid sudden flips
        double desired = output;
        double delta = desired - lastOutput;
        if (delta > maxDeltaPerLoop)
            delta = maxDeltaPerLoop;
        if (delta < -maxDeltaPerLoop)
            delta = -maxDeltaPerLoop;
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
        if (error < fallLimit)
        {
            // Robot is upright, apply enhanced control
            if (abs(output) > 1.0 || abs(steeringCommand) > 1.0 || abs(speedCommand) > 1.0)
            {
                // Use enhanced motor control with Bluetooth commands
                controlMotorsAdvanced(output, steeringCommand, speedCommand);
                
                Serial.print(" | ACTIVE");
                Serial.print(" | Err: "); Serial.print(error, 1);
                Serial.print(" | TiltRate: "); Serial.print(tiltRate, 1);
            }
            else
            {
                stopMotors();
                Serial.print(" | STOPPED");
            }
        }
        else
        {
            // Robot has fallen, stop everything
            stopMotors();
            steeringCommand = 0;
            speedCommand = 0;
            Serial.print(" | FALLEN");
        }
        
        Serial.println();
    }
}
