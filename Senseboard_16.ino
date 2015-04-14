#include <sensors.h>

#include <stdint.h>
#include <Servo.h>

#define INT16_MAX 0x7FFF

int led = 13; // Pin 13 has an LED connected on most Arduino boards.
Servo frontServo; // frontServo
char buffer[5]; // message byte buffer

int pos = 0; // position of the frontServo

const int BAUDRATE_SPEED = 115200; // Speed of the Baudrate
const int MAX_LOOP_COUNT = 100; // loop count for read and write

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(BAUDRATE_SPEED); // set baud rate speed
  pinMode(led, OUTPUT); // initialize the digital pin as an output.
  frontServo.attach(10); // attach frontServo to pin 10
}

bool readFull(char* buffer, int bufSize) {
    int result, loopCount = MAX_LOOP_COUNT;

    while(loopCount -- > 0) {
        result = Serial.readBytes(buffer, bufSize);
        if(result != -1) {
            // if there was no error while reading
            // then go further in the buffer
            buffer += result;
            bufSize -= result;
        } else {
            // we will skip errors here
        }

        // check if the full buffer was read
        if(0 == bufSize) return true;
    }

    // in case MAX_LOOP_COUNT was reached
    return false;
}

bool writeFull(const char* buffer, int bufSize) {
    int result, loopCount = MAX_LOOP_COUNT;

    while(loopCount -- > 0) {
        result = Serial.write((uint8_t*)buffer, bufSize);
        if(result != -1) {
            buffer += result;
            bufSize -= result;
        } else {
            // skip errors
        }

        if(0 == bufSize) return true;
    }

    return false;
}

// the loop routine runs over and over again forever:
void loop() {
  /*
    // Simple Serial connectivity test
    char c;
    if(readFull(buffer,1)){
      c = buffer[0];
      writeFull(&c,1);
    }else{
      digitalWrite(led, HIGH);
      c = 'B';
      writeFull(&c,1); 
    }
  */
  
  if (Serial.available() > 0) {
    if(readFull(buffer, 5)){
      sense_link::Message m;
      sense_link::decode(&m, buffer);
      //m.mType = sense_link::SENSOR_DATA;
      //m.sType = sense_link::SERVO;
      //m.id = 1;
      
      sense_link::Message out;
      
      if(m.mType == sense_link::SENSOR_DATA){
        /*
        if(m.sType == sense_link::LED){
          if(m.id == 1){
            //digitalWrite(led, HIGH);
            if(m.sensorData.Led.value == sense_link::ON){
              digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
            }else{
              digitalWrite(led, LOW);
            }
          }
        }
       */
       if(m.sType == sense_link::SERVO){
          if(m.id == 1){
            // set position from message angle
            pos = m.sensorData.Servo.angle;
            // write out servo position between 0 and 180 degree
            myservo.write(pos);
            // turn on led
            digitalWrite(led, HIGH);
          }
        }
      }
    
      // send message back
      out.mType = sense_link::SENSOR_DATA;
            out.sType = sense_link::SERVO; 
            out.id = 1;
            if(pos < INT16_MAX){
              out.sensorData.Servo.angle = pos;
            }else{
              out.sensorData.Proximity.distance = INT16_MAX;
            }
            int cntSend = sense_link::encode(&out, buffer);
           writeFull(buffer, cntSend);
    }
  }
}
