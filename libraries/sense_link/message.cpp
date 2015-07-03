#include "message.h"

namespace sense_link {

/**
 * @brief writeInt16 writes 2 bytes in the buffer.
 * @param number int16_t|uint32_t number which is written into the buffer
 * @param buffer write target
 */
template<typename T>
void writeInt16(T number, char* buffer){
#if __cplusplus >= 201103L // C++11 support
    static_assert(sizeof(T) == 2, "writeInt16 type must be of size 2");
#endif
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    *buffer++ = (number >> 8) & 0xFF;
    *buffer++ = (number) & 0xFF;
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *( reinterpret_cast<T*>(buffer) ) = number;
#else
    #error unknown byte order
#endif
}

/**
 * @brief writeInt32 writes 4 bytes in the buffer.
 * @param number int32_t|uint32_t number which is written into the buffer
 * @param buffer write target
 */
template<typename T>
void writeInt32(T number, char* buffer){
#if __cplusplus >= 201103L // C++11 support
    static_assert(sizeof(T) == 4, "writeInt32 type must be of size 4");
#endif
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    *buffer++ = (number >> 24) & 0xFF;
    *buffer++ = (number >> 16) & 0xFF;
    *buffer++ = (number >> 8) & 0xFF;
    *buffer++ = (number) & 0xFF;
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *( reinterpret_cast<T*>(buffer) ) = number;
#else
    #error unknown byte order
#endif
}

/**
 * @brief writeInt64 writes 8 bytes in the buffer.
 * @param number int64_t|uint64_t number which is written into the buffer
 * @param buffer write target
 */
template<typename T>
void writeInt64(T number, char* buffer){
#if __cplusplus >= 201103L // C++11 support
    static_assert(sizeof(T) == 8, "writeInt64 type must be of size 8");
#endif
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    *buffer++ = (number >> 56) & 0xFF;
    *buffer++ = (number >> 48) & 0xFF;
    *buffer++ = (number >> 40) & 0xFF;
    *buffer++ = (number >> 32) & 0xFF;
    *buffer++ = (number >> 24) & 0xFF;
    *buffer++ = (number >> 16) & 0xFF;
    *buffer++ = (number >> 8) & 0xFF;
    *buffer++ = (number) & 0xFF;
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *( reinterpret_cast<T*>(buffer) ) = number;
#else
    #error unknown byte order
#endif
}

/**
 * @brief readInt16 reades 2 bytes from the buffer.
 * @param number int16_t|uint16_t number which will be read
 * @param buffer reading source
 */
template<typename T>
void readInt16(T* number, const void* voidBuf) {
#if __cplusplus >= 201103L // C++11 support
    static_assert(sizeof(T) == 2, "readInt16 type must be of size 2");
#endif
    const uint8_t *buffer = (const uint8_t*) voidBuf;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    *number = (((uint16_t) buffer[0]) << 8) | ((uint16_t )buffer[1]);
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *number = *( reinterpret_cast<const T*>(buffer) );
#else
    #error unknown byte order
#endif
}

/**
 * @brief readInt32 reades 4 bytes from the buffer.
 * @param number int32_t|uint32_t number which will be read
 * @param buffer reading source
 */
template<typename T>
void readInt32(T* number, const void* voidBuf) {
#if __cplusplus >= 201103L // C++11 support
    static_assert(sizeof(T) == 4, "readInt32 type must be of size 4");
#endif
    const uint8_t *buffer = (const uint8_t*) voidBuf;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    *number = ((uint32_t) buffer[0] << 24) | ((uint32_t) buffer[1] << 16)
            | ((uint32_t) buffer[2] << 8) | (uint32_t) buffer[3];
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *number = *( reinterpret_cast<const T*>(buffer) );
#else
    #error unknown byte order
#endif
}

/**
 * @brief readInt64 reades 8 bytes from the buffer.
 * @param number int64_t|uint64_t number which will be read
 * @param buffer reading source
 */
template<typename T>
void readInt64(T* number, const void* voidBuf) {
#if __cplusplus >= 201103L // C++11 support
    static_assert(sizeof(T) == 8, "readInt64 type must be of size 8");
#endif
    const uint8_t *buffer = (const uint8_t*) voidBuf;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    *number = ((uint64_t) buffer[0] << 56) | ((uint64_t) buffer[1] << 48)
            | ((uint64_t) buffer[2] << 40) | ((uint64_t) buffer[3] << 32)
            | ((uint64_t) buffer[4] << 24) | ((uint64_t) buffer[5] << 16)
            | ((uint64_t) buffer[6] << 8) | (uint64_t) buffer[7];
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *number = *( reinterpret_cast<const T*>(buffer) );
#else
    #error unknown byte order
#endif
}


/**
 * @brief encode encodes a Message to a byte buffer
 * @param message Message to encode (send)
 * @param buffer encoded Message in bytes
 * @return size of message (bytes)
 */
int encodeMessage (const Message* message, char* buffer) {
    //copy start adress of the message
    //to count bytes written
    const char* start = buffer;

    //writing first byte
    *buffer++ = static_cast<char>(message->message);

    //writing second byte
    if(message->message == MessageType::ACTUATOR){
        *buffer++ = static_cast<char> (message->actuator);
    }else{
        *buffer++ = static_cast<char> (message->sensor);
    }

    // Sensor/Actor id
    *buffer++ = message->id;

    // Sequence number
    *buffer++ = message->sequence;

    //writing rest bytes
    if(message->message == MessageType::ERROR){
        *buffer++ = static_cast<char>(message->error.code);
    }else if(message->message == MessageType::TIME){
        writeInt64(message->time.micros, buffer);
        buffer += 8;
    }else if(message->message == MessageType::ACTUATOR){
        switch (message->actuator) {
            case ActuatorType::LED:
                *buffer++ = message->sensorData.Led.value;
                break;
            case ActuatorType::SERVO:
                writeInt16(message->sensorData.Servo.angle, buffer);
                buffer+= 2;
                break;
            case ActuatorType::MOTOR:
                writeInt16(message->sensorData.Motor.speed, buffer);
                buffer+= 2;
        }
    }else if(message->message == MessageType::SENSOR_DATA){
        switch (message->sensor) {
            case SensorType::PROXIMITY:
                writeInt16(message->sensorData.Proximity.distance, buffer);
                buffer += 2;
                break;
            case SensorType::GYROSCOPE:
                writeInt32(message->sensorData.Gyroscope.x, buffer);
                buffer += 4;
                writeInt32(message->sensorData.Gyroscope.y, buffer);
                buffer += 4;
                writeInt32(message->sensorData.Gyroscope.z, buffer);
                buffer += 4;
                break;
            case SensorType::ACCELEROMETER:
                writeInt32(message->sensorData.Accelerometer.x, buffer);
                buffer += 4;
                writeInt32(message->sensorData.Accelerometer.y, buffer);
                buffer += 4;
                writeInt32(message->sensorData.Accelerometer.z, buffer);
                buffer += 4;
                break;
            case SensorType::MAGNETOMETER:
                writeInt32(message->sensorData.Magnetometer.x, buffer);
                buffer+= 4;
                writeInt32(message->sensorData.Magnetometer.y, buffer);
                buffer+= 4;
                writeInt32(message->sensorData.Magnetometer.z, buffer);
                buffer+= 4;
                break;
            case SensorType::ORIENTATION:
                writeInt32(message->sensorData.Orientation.x, buffer);
                buffer+= 4;
                writeInt32(message->sensorData.Orientation.y, buffer);
                buffer+= 4;
                writeInt32(message->sensorData.Orientation.z, buffer);
                buffer+= 4;
                break;
            case SensorType::SERVO:
                writeInt16(message->sensorData.Servo.angle, buffer);
                buffer+= 2;
                break;
            case SensorType::MOTOR:
                writeInt16(message->sensorData.Motor.speed, buffer);
                buffer+= 2;
                break;
            case SensorType::UNKNOWN:
            case SensorType::__END__:
                break;
            case SensorType::LED:
                *buffer++ = message->sensorData.Led.value;
                break;
            case SensorType::MOUSE:
                writeInt32(message->sensorData.Mouse.x, buffer);
                buffer += 4;
                writeInt32(message->sensorData.Mouse.y, buffer);
                buffer += 4;
                writeInt32(message->sensorData.Mouse.duration, buffer);
                buffer += 4;
                *buffer++ = message->sensorData.Mouse.surfaceQuality;
                break;
            case SensorType::SBUS:
                for(uint8_t i = 0; i < 8; i++)
                {
                    writeInt16(message->sensorData.SBus.channels[i], buffer);
                    buffer += 2;
                }
                *buffer++ = message->sensorData.SBus.flags;
                break;
        } // end switch
    } // end SENSOR_DATA

    //returns actual adress - start adress = bytes written
    unsigned int messageLength = buffer - start;

    // compute checksum
    *buffer = crc8(start, messageLength);

    return messageLength + 1; // +1 is for crc checksum
}

/**
 * @brief decode decodes a Message from a byte buffer
 * @param message decoded message (result)
 * @param buffer reading source
 * @return size of message (bytes)
 */
int decodeMessage (Message* message, const char* buffer) {
    const char* start = buffer;

    message->message = (MessageType) *buffer++;

    if(message->message == MessageType::ACTUATOR){
        message->actuator = (ActuatorType) *buffer++;
    }else{
        message->sensor = (SensorType) *buffer++;
    }

    message->id = *buffer++;
    message->sequence = *buffer++;

    if(message->message == MessageType::ERROR){
        message->error.code = static_cast<ErrorCode>(*buffer++);
    }else if(message->message == MessageType::TIME){
        readInt64( &(message->time.micros), buffer);
        buffer += 8;
    }else if(message->message == MessageType::ACTUATOR){
        switch (message->actuator) {
            case ActuatorType::LED:
                message->sensorData.Led.value = *buffer++;
                break;
            case ActuatorType::SERVO:
                readInt16(&(message->sensorData.Servo.angle), buffer);
                buffer+= 2;
                break;
            case ActuatorType::MOTOR:
                readInt16(&(message->sensorData.Motor.speed), buffer);
                buffer+= 2;
                break;
        }
    }else if(message->message == MessageType::SENSOR_DATA){
        switch (message->sensor){
        case SensorType::PROXIMITY:
            readInt16( &(message->sensorData.Proximity.distance), buffer);
            buffer += 2;
            break;
        case SensorType::GYROSCOPE:
            readInt32(&(message->sensorData.Gyroscope.x), buffer);
            buffer += 4;
            readInt32(&(message->sensorData.Gyroscope.y), buffer);
            buffer += 4;
            readInt32(&(message->sensorData.Gyroscope.z), buffer);
            buffer += 4;
            break;
        case SensorType::ACCELEROMETER:
            readInt32(&(message->sensorData.Accelerometer.x), buffer);
            buffer += 4;
            readInt32(&(message->sensorData.Accelerometer.y), buffer);
            buffer += 4;
            readInt32(&(message->sensorData.Accelerometer.z), buffer);
            buffer += 4;
            break;
        case SensorType::MAGNETOMETER:
            readInt32(&(message->sensorData.Magnetometer.x), buffer);
            buffer+= 4;
            readInt32(&(message->sensorData.Magnetometer.y), buffer);
            buffer+= 4;
            readInt32(&(message->sensorData.Magnetometer.z), buffer);
            buffer+= 4;
            break;
        case SensorType::ORIENTATION:
            readInt32(&(message->sensorData.Orientation.x), buffer);
            buffer+= 4;
            readInt32(&(message->sensorData.Orientation.y), buffer);
            buffer+= 4;
            readInt32(&(message->sensorData.Orientation.z), buffer);
            buffer+= 4;
            break;
        case SensorType::SERVO:
            readInt16(&(message->sensorData.Servo.angle), buffer);
            buffer+= 2;
            break;
        case SensorType::MOTOR:
            readInt16(&(message->sensorData.Motor.speed), buffer);
            buffer+= 2;
            break;
        case SensorType::UNKNOWN:
        case SensorType::__END__:
            break;
        case SensorType::LED:
            message->sensorData.Led.value = *buffer++;
            break;
        case SensorType::MOUSE:
            readInt32(&(message->sensorData.Mouse.x), buffer);
            buffer += 4;
            readInt32(&(message->sensorData.Mouse.y), buffer);
            buffer += 4;
            readInt32(&(message->sensorData.Mouse.duration), buffer);
            buffer += 4;
            message->sensorData.Mouse.surfaceQuality = *buffer++;
            break;
        case SensorType::SBUS:
            for(uint8_t i = 0; i < 8; i++)
            {
                readInt16(&(message->sensorData.SBus.channels[i]), buffer);
                buffer += 2;
            }
            message->sensorData.SBus.flags = *buffer++;
            break;
        }
    }

    unsigned int messageLength = buffer - start;

    uint8_t computedChecksum = crc8(start, messageLength);
    uint8_t sentChecksum = *buffer;

    if(computedChecksum != sentChecksum) {
        return -1;
    } else {
        return messageLength + 1; // +1 for checksum
    }
}
/**
  * CRC-8 lookup table
  */
static const uint8_t table[] =
{0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
/**
  Return CRC-8 of the data (cyclic redundancy check) - message validity
 * @brief crc8
 * @param vptr begin of data
 * @param len length of data
 * @return CRC-8 value
 */
uint8_t crc8(const char *vptr, unsigned int len) {
    const uint8_t *data = (uint8_t*)vptr;
    uint8_t crc = 0xde;
    int j;
    for (j = len; j; j--, data++) {
        crc = table[crc ^ *data];
    }

    return crc;
}

}  // namespace sense_link

