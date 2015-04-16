#include <sensors.h>

#include <stdint.h>
#include <Servo.h>

#define INT16_MAX 0x7FFF

int led = 13; // Pin 13 has an LED connected on most Arduino boards.
Servo frontServo; // frontServo 
Servo motor; // motor of vehicel
char buffer[5]; // message byte buffer

int frontServoPosition = 0; // position of the frontServo
int motorSpeed = 0; // speed of the motor

const int MAX_LOOP_COUNT = 1000; // loop count for read and write

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200); // set baud rate speed !!! DONT USE BAUDRATE VARIABLE !!!
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

/**
PWM values recieved are between -10000 (fully break/counter-clockwise) and 10000 (fully throttle/clockwise)
PWM values transmitted have to be 1000 to 2000 (1500 is the middle position)
*/
void setMotorVelocity(int16_t velocity){
    int microseconds = microsecondsPWM(velocity);
    motor.writeMicroseconds(microseconds);
}

int microsecondsPWM(int16_t value){
    /*
    value is between -10000 and 10000
    this is mapped to
    1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
    */
    return (1500 - (((int)value * 500)/10000));
}

bool sendMessage(sense_link::MessageType mType, sense_link::SensorType sType, uint8_t id, sense_link::SensorData data){
    
    // new Message out
    sense_link::Message out;
    // set MessageType
    out.mType = mType;
    // set SensorType
    out.sType = sType;
    // set ID 
    out.id = id;
    
    // set SensorData
    out.sensorData = data;
    //write Message
    writeMessage(&out);
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
      
      // read Message
      sense_link::Message m;
      readMessage(&m);
      
      //m.mType = sense_link::SENSOR_DATA;
      //m.sType = sense_link::SERVO;
      //m.id = 1;
      
      // check Message ---------
      
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
            
            sense_link::SensorData data;
            data.Led.value = m.sensorData.Led.value;
            
            //send message back
            sendMessage(sense_link::SENSOR_DATA, sense_link::LED, 1, data);
            
          }
        }else /* end LED*/
        if(m.sType == sense_link::MOTOR_VELOCITY){
          // Motor ID 1
          if(m.id == 1){
            // set global motorSpeed
            motorSpeed = m.sensorData.MotorVelocity.acceleration;
            setMotorVelocity(motorSpeed);
            
            sense_link::SensorData data;
            data.MotorVelocity.acceleration = motorSpeed;
            
            // send message back
            sendMessage(sense_link::SENSOR_DATA, sense_link::MOTOR_VELOCITY, 1, data);      
            
          }  
       }else /* end Motor*/
       if(m.sType == sense_link::SERVO){
          // front Servo ID 1
          if(m.id == 1){
            // set global position from message angle
            frontServoPosition = m.sensorData.Servo.angle;
            // write out servo position between 0 and 180 degree as Microseconds (1500 us = 90 degree)
            frontServo.writeMicroseconds(microsecondsPWM(frontServoPosition));
            
            // turn on led
            digitalWrite(led, HIGH);
            
            sense_link::SensorData data;
            data.Servo.angle = frontServoPosition;
            
            // send message back
            sendMessage(sense_link::SENSOR_DATA, sense_link::SERVO, 1, data);      
            
          } /* end Servo ID 1 */
        } /* end Servo */
      }else /* end SENSOR_DATA `*/
      if(m.mType == sense_link::SENSOR_GET){
        
        // get SERVO
        if(m.sType == sense_link::SERVO){
          // get Servo with ID 1
          if(m.id == 1){
            
            sense_link::SensorData data;
            data.Servo.angle = frontServoPosition;
            
            sendMessage(sense_link::SENSOR_DATA, sense_link::SERVO, 1, data);
          
          } /* end frontServo */
        }else /* end if SERVO */
        if(m.sType == sense_link::MOTOR_VELOCITY){
            sense_link::SensorData data;
            data.MotorVelocity.acceleration = motorSpeed;
            
            sendMessage(sense_link::SENSOR_DATA, sense_link::MOTOR_VELOCITY, 1, data);
        } /* end Motor */
        
      } /* end if SENSOR_GET */
      else{
        /* ErrorMessage */
        // send message back
        sense_link::Message out;
        out.mType = sense_link::ERROR;
        out.Error.code = 1;
        
        writeMessage(&out);
      }
      
  } /* end if Serial available */
} /* end loop */
