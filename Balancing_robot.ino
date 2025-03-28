#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.2.1

class PID
{
public:
    // Constants used in some of the functions below
    #define AUTOMATIC 1
    #define MANUAL 0
    #define DIRECT 0
    #define REVERSE 1
    #define P_ON_M 0
    #define P_ON_E 1

    PID(double*, double*, double*, double, double, double, int, int); // constructor
    PID(double*, double*, double*, double, double, double, int);

    void SetMode(int Mode);                      // sets PID to either Manual (0) or Auto (non-0)
    bool Compute();                              // performs the PID calculation
    void SetOutputLimits(double, double);        // clamps the output to a specific range
    void SetTunings(double, double, double);     // change tunings during runtime for Adaptive control
    void SetTunings(double, double, double, int);// overload for specifying proportional mode
    void SetControllerDirection(int);            // sets the Direction, or "Action" of the controller
    void SetSampleTime(int);                     // sets the frequency, in milliseconds

    double GetKp();                              // get proportional parameter
    double GetKi();                              // get integral parameter
    double GetKd();                              // get derivative parameter
    int GetMode();                               // get PID mode
    int GetDirection();                          // get controller direction

private:
    void Initialize();

    double dispKp;            // user-entered proportional parameter
    double dispKi;            // user-entered integral parameter
    double dispKd;            // user-entered derivative parameter

    double kp;                // proportional tuning parameter
    double ki;                // integral tuning parameter
    double kd;                // derivative tuning parameter

    int controllerDirection;
    int pOn;

    double *myInput;          // pointer to Input
    double *myOutput;         // pointer to Output
    double *mySetpoint;       // pointer to Setpoint

    unsigned long lastTime;
    double outputSum, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;

    bool inAuto, pOnE;
};

#endif // PID_v1_h

MPU6050 mpu;

int IN1 = 6;
int IN2 = 9;
int IN3 = 10;
int IN4 = 11;

// MPU control/status variables
bool ready = false;             // set true if DMP init was successful
uint8_t mpuIntStatus;           // holds actual interrupt status byte from MPU
uint8_t devStatus;              // return status after each device operation
uint16_t packetSize;            // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer

// orientation/motion variables
Quaternion q;
VectorFloat gravity;
float ypr[3];

double setpoint = 176;
double Kp = 18;
double Kd = 0.8;
double Ki = 140;
double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;
SemaphoreHandle_t pidMutex;

void dmpDataReady()
{
    mpuInterrupt = true;
}

void imuTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        if (mpuInterrupt || fifoCount < packetSize)
        {
            pid.Compute();
            xSemaphoreTake(pidMutex, portMAX_DELAY);

            // Print Input and Output on the serial monitor
            Serial.print(input);
            Serial.print(" => ");
            Serial.println(output);

            xSemaphoreGive(pidMutex);

            if (input > 150 && input < 200)
            {
                if (output > 0) Forward();
                else if (output < 0) Reverse();
            }
            else
            {
                Stop();
            }
        }

        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024)
        {
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
        }
        else if (mpuIntStatus & 0x02)
        {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            xSemaphoreTake(pidMutex, portMAX_DELAY);

            #if LOG_INPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180 / M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180 / M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180 / M_PI);
            #endif

            xSemaphoreGive(pidMutex);
        }
    }
}

void motorTask(void *pvParameters)
{
    (void)pvParameters;

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    for (;;)
    {
        xSemaphoreTake(pidMutex, portMAX_DELAY);

        if (output > 0) Forward();
        else if (output < 0) Reverse();
        else Stop();

        xSemaphoreGive(pidMutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println(F("Initializing I2C devices..."));

    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    ready = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pidMutex = xSemaphoreCreateMutex();

    // PID setup
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    xTaskCreate(imuTask, "IMU Task", 1024, NULL, 2, NULL);
    xTaskCreate(motorTask, "Motor Task", 128, NULL, 1, NULL);
}

void loop()
{
    // Empty loop as tasks are running in FreeRTOS
}

void Forward()
{
    analogWrite(6, output);
    analogWrite(9, 0);
    analogWrite(10, output);
    analogWrite(11, 0);
    Serial.print("F");
}

void Reverse()
{
    analogWrite(6, 0);
    analogWrite(9, output * -1);
    analogWrite(10, 0);
    analogWrite(11, output * -1);
    Serial.print("R");
}

void Stop()
{
    analogWrite(6, 0);
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, 0);
    Serial.print("S");
}
