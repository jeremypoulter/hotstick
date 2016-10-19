// rLab Hotstick Wand Firmware
//
// Jeremy Poulter

// Using example code from;
// MAX6675
// www.ladyada.net/learn/sensors/thermocouple

#include "max6675.h"

#define LED_RED   3
#define LED_GREEN 5
#define LED_BLUE  6

int thermoDO = MISO;
int thermoCS = SS;
int thermoCLK = SCK;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
int vccPin = 3;
int gndPin = 2;

void setup() {
  Serial.begin(9600);
  // use Arduino pins 
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp

  Serial.print("C = ");
  Serial.println(thermocouple.readCelsius());

  delay(1000);
}

void led(int red, int green, int blue)
{
    analogWrite(LED_RED, 255 - red);
    analogWrite(LED_GREEN, 255 - green);
    analogWrite(LED_BLUE, 255 - blue);
}
