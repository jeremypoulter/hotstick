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

unsigned long sendStateTimeout = 0;
unsigned long tipTempValidAfter = 0;
int lastHeatState = LOW;

// Temperature related functions
double readTempInt();
double readTempTip(double tempInt);

// MPU/DMP related stuff
void mpu_setup();
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
      Serial.readBytesUntil('}', data, JSON_BUFFER_SIZE);

      Serial.print(F("Got "));
      Serial.print(data);
      Serial.println(F(" from Base"));

      StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;

      JsonObject& msg = jsonBuffer.parseObject(data);

      // Check to see if we got an LED message
      if(msg["led"].success()) {
        led(msg["led"][0], msg["led"][1], msg["led"][2]);
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
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    JsonObject& state = jsonBuffer.createObject();
    state["temptip"] = temptip;
    state["tempint"] = tempint;
    state["x"] = x;
    state["y"] = y;
    state["z"] = z;

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
  return 22.0;
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
