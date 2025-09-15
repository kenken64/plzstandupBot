

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

// Bluetooth module pins (HC-05)
#define BT_RX_PIN A0  // Connect to HC-05 TX
#define BT_TX_PIN A1  // Connect to HC-05 RX
#define BT_DISABLE_PIN A2  // Ground this pin to disable Bluetooth (for uploads)

// MPU6050 interrupt pin
#define MPU_INT_PIN 2

// Bluetooth module setup
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

// Bluetooth command variables
String bluetoothCommand = "";
bool commandReceived = false;
char bluetoothBuffer[32];

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
// More conservative initial PID gains to reduce oscillations
double Kp = 7.0, Ki = 0.03, Kd = 1.2;  // Gentler response
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
void processBluetoothCommand();
void sendBluetoothStatus();
void readBluetoothCommand();

// Interrupt Function
void dmpDataReady() { mpuInterrupt = true; }

void setup()
{
    Serial.begin(115200);
    bluetooth.begin(9600);  // Initialize Bluetooth communication
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
    pinMode(BT_DISABLE_PIN, INPUT_PULLUP);  // Bluetooth disable pin
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
        Serial.println("DMP ready. Balancing mode engaged.");
        bluetooth.println("Robot initialized and ready for commands");
    }
    else
    {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
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

void loop()
{
    // Check for Bluetooth commands
    readBluetoothCommand();
    if (commandReceived) {
        processBluetoothCommand();
        commandReceived = false;
    }
    
    if (!dmpReady)
        return;

    // Check for FIFO overflow
    fifoCount = mpu.getFIFOCount();
    if (!mpuInterrupt && fifoCount < packetSize)
        return;

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
            // Robot is upright, apply balancing force
            if (abs(output) > 1.0)
            { // small deadband to reduce chatter
                int motorSpeed;

                // Use PID output with gentler limits during normal operation
                if (error > 15 || tiltRate > 30)
                {
                    // Large error or fast tilt - allow higher speeds for recovery
                    motorSpeed = constrain((int)abs(output), 0, 80);
                }
                else
                {
                    // Normal operation - cap to avoid violent swings
                    motorSpeed = constrain((int)abs(output), 0, maxNormalSpeed);
                }

                // Gradual minimum speed ramp to overcome stiction without jumps
                if (abs(output) > 5 && motorSpeed < minStartSpeed)
                {
                    motorSpeed = minStartSpeed;
                }

                // Apply motor control and debug
                if (output > 0)
                {
                    controlMotors(motorSpeed); // Move forward
                    Serial.print(" | MOTOR_CMD: FORWARD ");
                    Serial.print(motorSpeed);
                }
                else
                {
                    controlMotors(-motorSpeed); // Move backward
                    Serial.print(" | MOTOR_CMD: BACKWARD ");
                    Serial.print(motorSpeed);
                }

                Serial.print(" | Error: ");
                Serial.print(error, 1);
                Serial.print(" | TiltRate: ");
                Serial.print(tiltRate, 1);
                Serial.print(" | MotorSpeed: ");
                Serial.print((output > 0) ? motorSpeed : -motorSpeed);
            }
            else
            {
                stopMotors();
                Serial.print(" | Motors: STOPPED");
            }
        }
        else
        {
            // Robot has fallen, stop motors
            stopMotors();
            Serial.println("Robot fallen - stopping motors");
            bluetooth.println("STATUS: Robot fallen - motors stopped");
        }
    }
}

// Bluetooth communication functions
void readBluetoothCommand() {
    if (bluetooth.available()) {
        bluetoothCommand = bluetooth.readStringUntil('\n');
        bluetoothCommand.trim();
        if (bluetoothCommand.length() > 0) {
            commandReceived = true;
            Serial.print("BT Command received: ");
            Serial.println(bluetoothCommand);
        }
    }
}

void processBluetoothCommand() {
    bluetooth.print("ACK: ");
    bluetooth.println(bluetoothCommand);
    
    if (bluetoothCommand.equals("STATUS")) {
        sendBluetoothStatus();
    }
    else if (bluetoothCommand.equals("FORWARD")) {
        moveForward(50);
        bluetooth.println("RESPONSE: Moving forward");
        Serial.println("BT: Moving forward");
    }
    else if (bluetoothCommand.equals("BACKWARD")) {
        moveBackward(50);
        bluetooth.println("RESPONSE: Moving backward");
        Serial.println("BT: Moving backward");
    }
    else if (bluetoothCommand.equals("STOP")) {
        stopMotors();
        bluetooth.println("RESPONSE: Motors stopped");
        Serial.println("BT: Motors stopped");
    }
    else if (bluetoothCommand.startsWith("SPEED:")) {
        int speed = bluetoothCommand.substring(6).toInt();
        speed = constrain(speed, 0, 255);
        motorSpeedFactor = speed / 100.0;
        bluetooth.print("RESPONSE: Speed set to ");
        bluetooth.println(speed);
        Serial.print("BT: Speed set to ");
        Serial.println(speed);
    }
    else if (bluetoothCommand.startsWith("PID:")) {
        // Format: PID:Kp,Ki,Kd (e.g., PID:7.0,0.03,1.2)
        int firstComma = bluetoothCommand.indexOf(',');
        int secondComma = bluetoothCommand.indexOf(',', firstComma + 1);
        
        if (firstComma != -1 && secondComma != -1) {
            double newKp = bluetoothCommand.substring(4, firstComma).toDouble();
            double newKi = bluetoothCommand.substring(firstComma + 1, secondComma).toDouble();
            double newKd = bluetoothCommand.substring(secondComma + 1).toDouble();
            
            pid.SetTunings(newKp, newKi, newKd);
            Kp = newKp; Ki = newKi; Kd = newKd;
            
            bluetooth.print("RESPONSE: PID updated - Kp:");
            bluetooth.print(newKp);
            bluetooth.print(", Ki:");
            bluetooth.print(newKi);
            bluetooth.print(", Kd:");
            bluetooth.println(newKd);
            
            Serial.print("BT: PID updated - Kp:");
            Serial.print(newKp);
            Serial.print(", Ki:");
            Serial.print(newKi);
            Serial.print(", Kd:");
            Serial.println(newKd);
        }
    }
    else if (bluetoothCommand.equals("CALIBRATE")) {
        isCalibrated = false;
        calibrationSum = 0;
        calibrationCount = 0;
        bluetooth.println("RESPONSE: Recalibration started");
        Serial.println("BT: Recalibration started");
    }
    else {
        bluetooth.print("ERROR: Unknown command: ");
        bluetooth.println(bluetoothCommand);
        Serial.print("BT: Unknown command: ");
        Serial.println(bluetoothCommand);
    }
}

void sendBluetoothStatus() {
    bluetooth.println("=== ROBOT STATUS ===");
    bluetooth.print("Pitch: ");
    bluetooth.println(input, 2);
    bluetooth.print("Setpoint: ");
    bluetooth.println(setpoint, 2);
    bluetooth.print("Error: ");
    bluetooth.println(input - setpoint, 2);
    bluetooth.print("PID Output: ");
    bluetooth.println(output, 2);
    bluetooth.print("Motor Left: ");
    bluetooth.println(motorLeftSpeed);
    bluetooth.print("Motor Right: ");
    bluetooth.println(motorRightSpeed);
    bluetooth.print("Calibrated: ");
    bluetooth.println(isCalibrated ? "YES" : "NO");
    bluetooth.print("PID Values - Kp:");
    bluetooth.print(Kp);
    bluetooth.print(", Ki:");
    bluetooth.print(Ki);
    bluetooth.print(", Kd:");
    bluetooth.println(Kd);
    bluetooth.println("=== END STATUS ===");
}
