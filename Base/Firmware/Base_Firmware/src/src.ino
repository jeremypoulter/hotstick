#include <Arduino.h>
#include <ArduinoJson.h>

#define NOT_ENABLE_PIN  2

// If we don't receive a message in an ammount of time go to a fail safe mode
#define COMMS_TIMEOUT   200

// The max tme to leave the heater on for
#define HEATER_ON_TIME  100

// Min time to leave the heater off for, to read the temp
#define HEATER_OFF_TIME 10

#define JSON_BUFFER_SIZE 200

// Heater state, HIGH is off, LOW is on
int heaterState = HIGH;

double temptip = 0.0;
double tempint = 0.0;
double yaw = 0.0;
double pitch = 0.0;
double roll = 0.0;

unsigned long messageTimeout = 0;
unsigned long heaterOnTimeout = 0;
unsigned long heaterOffTimeout = 0;

double tempTarget = 350.0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(38400);

  Serial.println("Hotstick base station");

  pinMode(NOT_ENABLE_PIN, OUTPUT);
  digitalWrite(NOT_ENABLE_PIN, heaterState);
}

void loop()
{
  // Read any message from the Wand
  if(Serial1.available())
  {
    char start = Serial1.peek();
    if('{' == start)
    {
      char data[JSON_BUFFER_SIZE];
      int read = Serial1.readBytesUntil('}', data, JSON_BUFFER_SIZE);
      if(read+2 < JSON_BUFFER_SIZE)
      {
        data[read++] = '}';
        data[read++] = '\0';

/*
        Serial.print("Got ");
        Serial.print(data);
        Serial.println(" from Wand");
*/

        StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;

        JsonObject& msg = jsonBuffer.parseObject(data);

        // tip temp will be < 0 if the heater is on
        if(msg["temptip"].success() &&
           msg["tempint"].success() &&
           msg["yaw"].success() &&
           msg["pitch"].success() &&
           msg["roll"].success())
        {
          double readtemptip = msg["temptip"];
          if(readtemptip > 0.0) {
            temptip = readtemptip;
          }
          tempint = msg["tempint"];
          yaw = msg["yaw"];
          pitch = msg["pitch"];
          roll = msg["roll"];
  
/*
          Serial.print("temptip = ");
          Serial.print(temptip);
          Serial.print(", ");
          Serial.println(readtemptip);
          Serial.print("tempint = ");
          Serial.println(tempint);
          Serial.print("yaw = ");
          Serial.println(yaw);
          Serial.print("pitch = ");
          Serial.println(pitch);
          Serial.print("roll = ");
          Serial.println(roll);
*/
          Serial.print(temptip);
          Serial.print(",");
          Serial.print(readtemptip);
          Serial.print(",");
          Serial.print(tempint);
          Serial.print(",");
          Serial.print(yaw);
          Serial.print(",");
          Serial.print(pitch);
          Serial.print(",");
          Serial.println(roll);

          messageTimeout = millis() + COMMS_TIMEOUT;
        }
      } else {
        Serial.println("Serial buffer full");
      }
    } else {
      Serial1.read();
    }
  }

  if(millis() < messageTimeout)
  {
    if(HIGH == heaterState)
    {
      // Heater is off

      // Turn on if the off timer has expired and the tip temp is lower than the
      // required temp
      if(temptip >= tempTarget)
      {
        Serial1.print("{\"led\":[0,255,0]}");
      }
      else if(millis() > heaterOffTimeout)
      {
        digitalWrite(NOT_ENABLE_PIN, LOW);
        heaterState = LOW;
        heaterOnTimeout = millis() + HEATER_ON_TIME;

        Serial1.print("{\"led\":[255,0,0]}");
      }
    }
    else
    {
      // Heater is on

      // Turn it off if the on timer has expired or the target temp has been reached
      if(millis() > heaterOnTimeout || temptip >= tempTarget)
      {
        digitalWrite(NOT_ENABLE_PIN, HIGH);
        heaterState = HIGH;
        heaterOffTimeout = millis() + HEATER_OFF_TIME;
        Serial1.print("{\"led\":[0,0,255]}");
      }
    }
  }
  else
  {
    // No data from Wand, make sure the heater is off
    digitalWrite(NOT_ENABLE_PIN, HIGH);
    heaterState = HIGH;
  }
}
