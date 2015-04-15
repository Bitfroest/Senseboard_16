#include <sensors.h>

#include <stdint.h>
#include <Servo.h>

#define INT16_MAX 0x7FFF

int led = 13; // Pin 13 has an LED connected on most Arduino boards.
Servo frontServo; // frontServo 
Servo motor; // motor of vehicel
char buffer[5]; // message byte buffer

int pos = 0; // position of the frontServo
float motorSpeed = 0;

//const int BAUDRATE_SPEED = 115200; // Speed of the Baudrate
const int MAX_LOOP_COUNT = 1000; // loop count for read and write

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200); // set baud rate speed !!! BAUDRATE MUST WITHOUT VARIABLE !!!
  pinMode(led, OUTPUT); // initialize the digital pin as an output.
  frontServo.attach(10); // attach frontServo to pin 10
  motor.attach(11); // attach motor to pin 11
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

bool readMessage(sense_link::Message *message) {
    uint8_t messageLength;
    char buffer[255];

    bool timeout = false;
    int bytesDecoded = 0;

    readFull((char*)&messageLength, 1);
    readFull(buffer, messageLength);

    bytesDecoded = sense_link::decodeMessage(message, buffer);

    // check for wrong checksum
    return bytesDecoded != -1;
}

bool writeMessage(const sense_link::Message *message) {
    char buffer[255];
    uint8_t messageLength = sense_link::encodeMessage(message, buffer + 1);
    buffer[0] = messageLength;

    writeFull(buffer, messageLength + 1);

    return true;
}

void setMotorVelocity(int16_t velocity){
    /*
    velocity is between -1.0 and 1.0
    1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
    */
    int microseconds = 1500 + (((int)velocity * 500)/10000);
    motor.writeMicroseconds(microseconds);
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
  //char c = 'A';
  //writeFull(&c,1);
  if (Serial.available() > 0) {
      digitalWrite(led, HIGH);
      
      sense_link::Message m;
      readMessage(&m);
      
      //m.mType = sense_link::SENSOR_DATA;
      //m.sType = sense_link::SERVO;
      //m.id = 1;
      
      // MessageTypes
      if(m.mType == sense_link::SENSOR_DATA){
        
        // SensorTypes
        if(m.sType == sense_link::LED){
          if(m.id == 1){
            //digitalWrite(led, HIGH);
            if(m.sensorData.Led.value == sense_link::ON){
              digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
            }else{
              digitalWrite(led, LOW);
            }
          }
        }else
        if(m.sType == sense_link::MOTOR_VELOCITY){
         if(m.id == 1){
          motorSpeed = m.sensorData.MotorVelocity.acceleration;
          setMotorVelocity(motorSpeed);
          
          // send message back
          sense_link::Message out;
          out.mType = sense_link::SENSOR_DATA;
          out.sType = sense_link::MOTOR_VELOCITY; 
          out.id = 1;
          
          out.sensorData.MotorVelocity.acceleration = motorSpeed;
          writeMessage(&out);
      
          
         }  
       }else
       if(m.sType == sense_link::SERVO){
          if(m.id == 1){
            // set position from message angle
            pos = m.sensorData.Servo.angle;
            // write out servo position between 0 and 180 degree
            frontServo.write(pos);
            // turn on led
            digitalWrite(led, HIGH);
            
            // send message back
            sense_link::Message out;
            out.mType = sense_link::SENSOR_DATA;
            out.sType = sense_link::SERVO; 
            out.id = 1;
            
            out.sensorData.Servo.angle = pos;
            writeMessage(&out);
      
            
          }
        }
      }else
      if(m.mType == sense_link::SENSOR_GET){
        if(m.sType == sense_link::SERVO){
          
        } // end if SERVO
      } // end if SENSOR_GET
      else{
        // send message back
        sense_link::Message out;
        out.mType = sense_link::ERROR;
        out.Error.code = 1;
        
        writeMessage(&out);
      }
      
  } // end if Serial available
} // end loop
