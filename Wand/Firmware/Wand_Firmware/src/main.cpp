// rLab Hotstick Wand Firmware
//
// Jeremy Poulter

#include <Arduino.h>
#include <ArduinoJson.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define LED_RED         3
#define LED_GREEN       5
#define LED_BLUE        6

#define HEAT_DETECT_PIN A0
#define TEMP_PIN        A1
#define INTERRUPT_PIN   2  // use pin 2 on Arduino Uno & most boards

#define SEND_STATE_TIME (1000/25) // Update the base at 25 Hz
#define TIP_TEMP_READ_DELAY 5 // Time to wait after heat is turned off

#define TEMP_ERROR_HEATER_ON  -1.0
#define TEMP_ERROR_NO_TIP     -2.0

#define JSON_BUFFER_SIZE 200

// MPU control/status vars
MPU6050 mpu;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned long sendStateTimeout = 0;
unsigned long tipTempValidAfter = 0;
int lastHeatState = LOW;

double yaw = 0.0;
double pitch = 0.0;
double roll = 0.0;

// Temperature related functions
double readTempInt();
double readTempTip(double tempInt);

// MPU/DMP related stuff
void mpu_setup();
void mpu_read();
void dmpDataReady();

// LED control
void led_setup();
void led(int red, int green, int blue);

void setup()
{
  // Initialise the serial
  Serial.begin(38400);

  led_setup();
  mpu_setup();

  // Set the initial state of the LED to Yellow
  led(255, 255, 0);

  // We are using the Internal refernce voltage for the ADC
  analogReference(INTERNAL);

  lastHeatState = digitalRead(HEAT_DETECT_PIN);
  tipTempValidAfter = millis() + TIP_TEMP_READ_DELAY;
}

void loop()
{
  // Read the MPU
  mpu_read();

  // Has the state of the Heat detect pin changed
  int heatState = digitalRead(HEAT_DETECT_PIN);
  if(lastHeatState != heatState) {
    lastHeatState = heatState;
    tipTempValidAfter = millis() + TIP_TEMP_READ_DELAY;
  }

  // Read any message from the Wand
  if(Serial.available())
  {
    char start = Serial.peek();
    if('{' == start)
    {
      char data[JSON_BUFFER_SIZE];
      int read = Serial.readBytesUntil('}', data, JSON_BUFFER_SIZE);
      if(read + 2 < JSON_BUFFER_SIZE)
      {
        data[read++] = '}';
        data[read++] = '\0';
        Serial.print(F("Got "));
        Serial.print(data);
        Serial.println(F(" from Base"));

        StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;

        JsonObject& msg = jsonBuffer.parseObject(data);

        // Check to see if we got an LED message
        if(msg["led"].success()) {
          led(msg["led"][0], msg["led"][1], msg["led"][2]);
        }
      }
    } else {
      Serial.read();
    }
  }

  // Do we need to send a status updates
  if(millis() >= sendStateTimeout)
  {
    StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;

    // We want to send status updates exactly at the same interval so sent the next
    // timeout now
    sendStateTimeout = millis() + SEND_STATE_TIME;

    // Read the temp
    double tempint = readTempInt();
    double temptip = readTempTip(tempint);

    JsonObject& state = jsonBuffer.createObject();
    state["temptip"] = temptip;
    state["tempint"] = tempint;
    state["yaw"] = yaw;
    state["pitch"] = pitch;
    state["roll"] = roll;

    state.printTo(Serial);
    Serial.println("");
  }
}

void led(int red, int green, int blue)
{
  analogWrite(LED_RED, 255 - red);
  analogWrite(LED_GREEN, 255 - green);
  analogWrite(LED_BLUE, 255 - blue);
}

void led_setup()
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
}

double readTempTip(double intTemp)
{
  if(HIGH == digitalRead(HEAT_DETECT_PIN) || tipTempValidAfter > millis()) {
    return TEMP_ERROR_HEATER_ON;
  }

  double tempAdc = (double)analogRead(TEMP_PIN);
  if(tempAdc >= 1022.9) {
    return TEMP_ERROR_NO_TIP;
  }

//  return intTemp + 16.22 + tempAdc * 0.61;
  return 16.913 + (tempAdc * 0.61) + (-0.0002 * pow(tempAdc, 2));
}

double readTempInt()
{
  int16_t rawTemp = mpu.getTemperature();
  return (rawTemp/340.)+36.53;
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

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
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
}

void mpu_read()
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = (ypr[0] * 180/M_PI);
        pitch = (ypr[1] * 180/M_PI);
        roll = (ypr[2] * 180/M_PI);
    }
}
