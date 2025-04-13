// # File name   : encoders.py
// # Description : Wheel Encoder code & MPU6050 YPR Reader 
// # Product     : smilebot  
// # E-mail      : winicon@live.com
// # Author      : Semiu ADEBAYO
// # Date        : 2025/04/12
// # credit      : SciJoy @ https://www.youtube.com/watch?v=cLtMcqRetO0&t=101s
//               : Jeff Rowberg @  https://github.com/jrowberg/i2cdevlib
//               : Code assisted by Microsoft Copilot, an AI tool developed by Microsoft


// This code is designed for use on ESP32.
// It combines the custom code for reading MPU6050 accelerometer with the readings of Wheel Wheel Encoders
// Both components readings are combined into a list which is sent over serial to be read by Raspberry Pi for SQL 
// data logging and other computations.
// The full project can be found @  https://github.com/winicon001




// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working
//      2025-04-04 - Adapted for Smilebot sensors data

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// Ultrasonic Sensor Pins
int TRIG_PIN = 5; // The Arduino Nano ESP32 pin connected to the Ultrasonic Sensor's TRIG pin
int  ECHO_PIN = 18; // The Arduino Nano ESP32 pin connected to the Ultrasonic Sensor's ECHO pin

float duration_us, distance_cm;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===          WHEEL ENCODERS VARIABLES DECLARATION            ===
// ================================================================

#define MOTOR_R 25 // Motor 1 Interrupt Pin - INT 1
#define MOTOR_L 26 // Motor 2 Interrupt Pin - INT 2

// Integers for pulse counters
volatile int COUNTER_L = 0; // Variable to store pulse count
volatile int COUNTER_R = 0; // Variable to store pulse count
unsigned int ENC_TOTAL_COUNT_L = 0;
unsigned int ENC_TOTAL_COUNT_R = 0;

float rotation1, rotation2; // speed in rev/min
float speed_L; //Left Wheel Speed in cm/min
float speed_R; //Right Wheel Speed in cm/min
float dist_L;  //Instantaneous Distance travelled by Left wheel in cm
float dist_R;  //Instantaneous Distance travelled by Right wheel in cm

unsigned long currentTime1; // Current time Decalration
unsigned long currentTime2; // Current time Decalration
unsigned long lastTime1; // Last time Decalration
unsigned long lastTime2; // Last time Decalration

unsigned long timeLaps1; // Difference between current time and last time
unsigned long timeLaps2; // Difference between current time and last time

static float totalDist_L = 0; // Cummulative Diistance travelled by Left Wheel in cm
static float totalDist_R = 0; // Cummulative Diistance travelled by Right Wheel in cm

// ================================================================

// ###############################################
// INTERRUPT SERVICES ROUTINE FOR WHEEL ENCODERS
// ###############################################

// Left Motor Motor pulse count ISR
void IRAM_ATTR ISR_COUNT_L() {
  COUNTER_L++; // increment Motor 1 counter value
  ENC_TOTAL_COUNT_L++; // Increment Total Count  for Left wheel as a separate variable

}

// Right Motor Motor pulse count ISR
void IRAM_ATTR ISR_COUNT_R() {
  COUNTER_R++; // increment Motor 2 counter value
  ENC_TOTAL_COUNT_R++; // Increment Total Count  for Right wheel as a separate variable

}

//  RPM for Left Motor 
void RPM_L() 
{
  currentTime1 = millis(); // Get current time in milliseconds
  timeLaps1 = currentTime1 - lastTime1; // Calculate elapsed time since last update
  
  if (currentTime1 - lastTime1 >= 100) {
    // Calculate RPM every second
    rotation1 = (COUNTER_L * 60) / 20; // Assuming 20 pulses per revolution

    speed_L = rotation1 * 22;
    dist_L = speed_L/timeLaps1;

    totalDist_L += dist_L; // Accumulate distance

    COUNTER_L = 0; // Reset pulse count
    lastTime1 = currentTime1; // Update last time
    // Serial.print("RPM left: ");
    // Serial.print(rotation1); // Print RPM value
    // Serial.print("  Speed left: ");
    // Serial.print(speed_L); // Print Speed left value
    // Serial.print("  Distance By left: "); // Print Speed left value
    // Serial.print(dist_L); // Print distance Travelled by left Wheel
    // Serial.print("  Total Distance By left: "); // Print Speed left value
    // Serial.println(totalDist_L); // Print distance Travelled by left Wheel
  }
}

//  RPM for Right Motor 
void RPM_R() 
{
  currentTime2 = millis(); // Get current time in milliseconds
  timeLaps2 = currentTime2 - lastTime2; // Calculate elapsed time since last update
  
  if (currentTime2 - lastTime2 >= 100) {
    // Calculate RPM every second
    rotation2 = (COUNTER_R * 60) / 20; //  Assuming 20 pulses per revolution

    speed_R = rotation2 * 22;
    dist_R = speed_R/timeLaps2;

    totalDist_R += dist_R; // Accumulate distance

    COUNTER_R = 0; // Reset pulse count
    lastTime2 = currentTime2; // Update last time
    // Serial.print("RPM right: ");
    // Serial.print(rotation2); // Print RPM value
    // Serial.print("  Speed right: ");
    // Serial.print(speed_R); // Print Speed right value
    // Serial.print("  Distance By right: "); // Print Speed right value
    // Serial.print(dist_R); // Print distance Travelled by right Wheel
    // Serial.print("  Total Distance By right: "); // Print Speed right value
    // Serial.println(totalDist_R); // Print distance Travelled by right Wheel
  }
}

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {


    // ============================================================
    // ================= WHEEL ENCODER SETUP REQUIREMENTS =========

    pinMode(MOTOR_L, INPUT_PULLUP); // Set encoder pin as input with pull-up resistor
    attachInterrupt(digitalPinToInterrupt(MOTOR_L), ISR_COUNT_L, RISING); // Attach interrupt to encoder pin
    pinMode(MOTOR_R, INPUT_PULLUP); // Set encoder pin as input with pull-up resistor
    attachInterrupt(digitalPinToInterrupt(MOTOR_R), ISR_COUNT_R, RISING); // Attach interrupt to encoder pin

    Serial.begin(115200); // Initialize serial communication
    // ============================================================



    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (9600 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    // Serial.begin(9600); // Since alrerady initialised under Wheel Encoder Setup. Changed to 115200
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    // pinMode(LED_PIN, OUTPUT);

    // ###############################
    // Configure the trigger pin to output mode
    pinMode(TRIG_PIN, OUTPUT);

    // Configure the echo pin to input mode
    pinMode(ECHO_PIN, INPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // ===========================================
    // ===== Call Encoder Readings Functions =====
    // ===========================================
    RPM_L();
    RPM_R();

    // ===========================================


    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            //Commented out to send the data as an array instead
            // Semiu A. 14/09/2024
            
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);
            // Produce a 10-microsecond pulse to the TRIG pin

            // ############ULTRASONIC SENSOR ################
            digitalWrite(TRIG_PIN, HIGH);
            // delayMicroseconds(10);
            digitalWrite(TRIG_PIN, LOW);

            // Measure the pulse duration from the ECHO pin
            duration_us = pulseIn(ECHO_PIN, HIGH);

            // Calculate the distance
            distance_cm = 0.017 * duration_us;

            // Print the value to the Serial Monitor
            // Serial.println(distance_cm);
        

            // delay(10);

            // #################################################

            float sensorsdata[] = {
                distance_cm,

                (ypr[0] * 180/M_PI), (ypr[1] * 180/M_PI), (ypr[2] * 180/M_PI), 
                ENC_TOTAL_COUNT_L, ENC_TOTAL_COUNT_R, 
                COUNTER_L, COUNTER_R, 
                rotation1, rotation2, 
                speed_L, speed_R, 
                dist_L, dist_R, 
                totalDist_L, totalDist_R
                };
             // Send the entire array to the Raspberry Pi
              for (int i = 0; i < 16; i++) {
                  Serial.print(sensorsdata[i]);
                  if (i < 15) {
                      Serial.print(","); // Separate values with a comma
                  }
              }
              Serial.println(); // End the line
            //   delay(10); // Wait for
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        // digitalWrite(LED_PIN, blinkState);
    }
}
