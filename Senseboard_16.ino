#include <TinyGPS.h>


#include <message.h>

#include <stdint.h>
#include <Servo.h>

#define INT16_MAX 0x7FFF

int led = 13; // Pin 13 has an LED connected on most Arduino boards.
Servo frontServo; // frontServo 
Servo motor; // motor of vehicel7
Servo rearServo; // rearServo

// Radio Control
int ch1 = 3;
int ch2 = 5;
int ch3 = 6;

char buffer[5]; // message byte buffer

int frontServoPosition = 0; // position of the frontServo
int rearServoPosition = 0; // position of the rearServo
int motorSpeed = 0; // speed of the motor
bool radioControl = true;

const int MAX_LOOP_COUNT = 1000; // loop count for read and write

// variables for timing
unsigned long previousMillis = 0;
unsigned long previousServoRefresh = 0;
const long interval = 250; // failsafe interval 250 ms
const long servoRefreshRate = 10; // set to 10 ms / 100hz


// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200); // set baud rate speed !!! DONT USE BAUDRATE VARIABLE !!!
  pinMode(led, OUTPUT); // initialize the digital pin as an output.
  
  // default pulse width limits range of 544-2400 us
  frontServo.attach(11, 1000, 2000); // attach frontServo to pin 10 (SET min and max PWM!!)
  motor.attach(9, 1000, 2000); // attach motor to pin 9
  rearServo.attach(10, 1000, 2000); 
  
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  
  
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
    // check PWM value is in range
    if(microseconds <= 2000 && microseconds >= 1000){
      motor.writeMicroseconds(microseconds);
    }else{
      // mid position
      motor.writeMicroseconds(1500);
    }
}

int microsecondsPWM(int16_t value){
    /*
    value is between -10000 and 10000
    this is mapped to
    1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
    */
    int microseconds (1500 - (((int)value * 500)/10000));
    if(microseconds > 2000 && microseconds < 1000){
      microseconds = 1500;
    }
    return microseconds;
}

bool sendMessage(sense_link::MessageType mType, sense_link::SensorType sType, uint8_t id, sense_link::SensorData data){
    
    // new Message out
    sense_link::Message out;
    // set MessageType
    out.message = mType;
    // set SensorType
    out.sensor = sType;
    // set ID 
    out.id = id;
    
    // set SensorData
    out.sensorData = data;
    //write Message
    writeMessage(&out);
}

// the loop routine runs over and over again forever:
void loop() {
  //stop time
  unsigned long currentMillis = millis();
  
  int trigger = pulseIn(ch3, HIGH);
  
  if(trigger > 1850){
      radioControl = false;
    }else{
      radioControl = true;
    }
    
  
  // Radio Control
  if(radioControl){
      frontServoPosition = pulseIn(ch1 , HIGH);
      motorSpeed = pulseIn(ch2, HIGH);
      
      Serial.println(trigger);
      frontServo.writeMicroseconds(frontServoPosition);
      rearServo.writeMicroseconds(frontServoPosition);
      motor.writeMicroseconds(motorSpeed);
  }
  
  // Servo and Motor Refresh
  if(currentMillis - previousServoRefresh >= servoRefreshRate) {
      // set previousServoRefresh
      previousServoRefresh = currentMillis;
    
      // rc frame rates are usually 50Hz, so we have to refresh at least every 20ms
      //frontServo.refresh();
      //motor.refresh();
  }
  
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
  if (Serial.available() > 0 && !radioControl) {
      //digitalWrite(led, HIGH);
      
      // read Message
      sense_link::Message m;
      readMessage(&m);
      
      //m.actuator = sense_link::ActuatorType;
      //m.actuatorData = sense_link::ActuatorType::SERVO;
      //m.id = 1;
      
      // check Message ---------
      
      // MessageTypes
      if(m.message == sense_link::MessageType::ACTUATOR){
        
        // SensorTypes
        if(m.actuator == sense_link::ActuatorType::LED){
          if(m.id == 1){
            //digitalWrite(led, HIGH);
            if(m.actuatorData.Led.value == sense_link::ON){
              digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
            }else{
              digitalWrite(led, LOW);
            }
            
            sense_link::ActuatorData data;
            data.Led.value = m.actuatorData.Led.value;
            
            //send message back
            //sendMessage(sense_link::SENSOR_DATA, sense_link::LED, 1, data);
            
          }
        }else /* end LED*/
        if(m.actuator == sense_link::ActuatorType::MOTOR){
          // Motor ID 1
          if(m.id == 1){
            //Failsafe
            previousMillis = currentMillis;
            
            // set global motorSpeed
            motorSpeed = m.actuatorData.Motor.speed;
            setMotorVelocity(motorSpeed);
            
            sense_link::ActuatorData data;
            data.Motor.speed = motorSpeed;
            
            // send message back
            //sendMessage(sense_link::SENSOR_DATA, sense_link::MOTOR_VELOCITY, 1, data);      
            
          }  
       }else /* end Motor*/
       if(m.actuator == sense_link::ActuatorType::SERVO){
          // front Servo ID 1
          if(m.id == 1){
            // set global position from message angle
            frontServoPosition = m.actuatorData.Servo.angle;
            // write out servo position between 0 and 180 degree as Microseconds (1500 us = 90 degree)
            frontServo.writeMicroseconds(microsecondsPWM(frontServoPosition));

            // turn on led
            // digitalWrite(led, HIGH);
            
            sense_link::ActuatorData data;
            data.Servo.angle = frontServoPosition;
            
            // send message back
            //sendMessage(sense_link::SENSOR_DATA, sense_link::SERVO, 1, data);      
            
          } /* end Servo ID 1 */
          
          // rear Servo ID 2
          if(m.id == 2){
            // set global position from message angle
            rearServoPosition = m.actuatorData.Servo.angle;
            // write out servo position between 0 and 180 degree as Microseconds (1500 us = 90 degree)
            rearServo.writeMicroseconds(microsecondsPWM(rearServoPosition));

            // turn on led
            // digitalWrite(led, HIGH);
            
            sense_link::ActuatorData data;
            data.Servo.angle = rearServoPosition;
            
            // send message back
            //sendMessage(sense_link::SENSOR_DATA, sense_link::SERVO, 1, data);      
            
          } /* end Servo ID 1 */
          
          
        } /* end Servo */
      }else /* end SENSOR_DATA `*/
      {
        /* ErrorMessage */
        // send message back
        sense_link::Message out;
        out.message = sense_link::MessageType::ERROR;
        out.error.code = sense_link::ErrorCode::INVALID_SENSOR_TYPE;
        
        writeMessage(&out);
      }
      
  } /* end if Serial available */
} /* end loop */
