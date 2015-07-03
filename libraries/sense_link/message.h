#ifndef SENSE_LINK_MESSAGE_H
#define SENSE_LINK_MESSAGE_H


#include <stdint.h>

namespace sense_link {

enum class SensorType : uint8_t {
    UNKNOWN = 0,
    PROXIMITY = 1,
    GYROSCOPE = 2,
    ACCELEROMETER = 3,
    MAGNETOMETER = 4,
    ORIENTATION = 5,
    SERVO = 6,
    MOTOR = 7,
    LED = 8,
    MOUSE = 9,
    SBUS = 10,
    __END__ // SensorType counter, must be LAST
};

enum class ActuatorType : uint8_t {
    SERVO = 0,
    MOTOR = 1,
    LED = 2
};

enum class MessageType : uint8_t {
    ERROR = 0,
    SENSOR_ENABLE = 1,
    SENSOR_DISABLE = 2,
    SENSOR_DATA = 3,
    SENSOR_GET = 4,
    TIME = 5,
    ACTUATOR = 6,
    ACTUATOR_ACK = 7
};

enum class ErrorCode : uint8_t {
    // Invalid header
    INVALID_MESSAGE_TYPE,
    INVALID_SENSOR_TYPE,
    INVALID_SENSOR_ID,

    // Message type dependant
    SENSOR_ENABLE_FAILED,
    SENSOR_DISABLE_FAILED,
    SENSOR_DATA_FAILED,

    // Specific
    SENSOR_TIMEOUT
};

enum LedIllumination : uint8_t {
    ON = 255,
    OFF = 0
};

const int16_t MAXIMUM_MOTOR_VALUE = 10000;
const int16_t MINIMUM_MOTOR_VALUE = -10000;

/**
 * @brief The SensorData struct
 *
 * Sensor Data MOTOR
 * - int16_t speed: Motor revolutions per minute (in RPM)
 *
 * Sensor Data SERVO
 * - int16_t angle: Servo deflection angle [deg]
 *
 * Sensor Data PROXIMITY
 * - int16_t distance: Distance to object in [mm]
 *
 * Sensor Data ACCELEROMETER
 * - int32_t x: Acceleration along x-axis (including gravity) in micro-G [uG]
 * - int32_t y: Acceleration along y-axis (including gravity) in micro-G [uG]
 * - int32_t z: Acceleration along z-axis (including gravity) in micro-G [uG]
 *
 * Sensor Data GYROSCOPE
 * - int32_t x: Angular rate around x-axis in micro-degrees per second [udeg/s]
 * - int32_t y: Angular rate around y-axis in micro-degrees per second [udeg/s]
 * - int32_t z: Angular rate around z-axis in micro-degrees per second [udeg/s]
 *
 * Sensor Data MAGNETOMETER
 * - int32_t x: Magnetic field strengh along x-axis in nano teslas [nT]
 * - int32_t y: Magnetic field strengh along y-axis in nano teslas [nT]
 * - int32_t z: Magnetic field strengh along z-axis in nano teslas [nT]
 *
 * Sensor Data ORIENTATION
 * - int32_t x: Azimuth around the x axis [mdeg]
 * - int32_t y: Azimuth around the y axis [mdeg]
 * - int32_t z: Azimuth around the z axis [mdeg]
 *
 * Sensor Data LED
 * - uint8_t: 0 - 255 Digital brightness value
 *
 * Sensor Data MOUSE
 * - int32_t x: Movement in x direction in micro-inch
 * - int32_t y: Movement in y direction in micro-inch
 * - uint32_t duration: Duration in which the movement occured in microseconds
 * - uint8_t surfaceQuality: Value denoting the quality of the surface measurement (higher = better)
 * 
 * Sensor Data SBUS
 * - uint16_t channels[8]: Raw channel values of first 8 SBus channels
 * - uint8_t flags: SBus flags bitmask
 */
typedef union {
    struct {
        int16_t speed; //!< Revolutions per minute [rpm]
    } Motor;

    struct {
        int16_t angle; //!< Servo deflection angle [deg]
    } Servo;

    struct {
        int16_t distance; //!< Distance to object in [mm]
    } Proximity;

    struct {
        int32_t x; //!< Acceleration along x-axis (including gravity) in micro-G [uG]
        int32_t y; //!< Acceleration along y-axis (including gravity) in micro-G [uG]
        int32_t z; //!< Acceleration along z-axis (including gravity) in micro-G [uG]
    } Accelerometer;

    struct {
        int32_t x; //!< Angular rate around x-axis in micro-degrees per second [udeg/s]
        int32_t y; //!< Angular rate around y-axis in micro-degrees per second [udeg/s]
        int32_t z; //!< Angular rate around z-axis in micro-degrees per second [udeg/s]
    } Gyroscope;

    struct {
        int32_t x; //!< Magnetic field strengh along x-axis in nano teslas [nT]
        int32_t y; //!< Magnetic field strengh along y-axis in nano teslas [nT]
        int32_t z; //!< Magnetic field strengh along z-axis in nano teslas [nT]
    } Magnetometer;

    struct {
        int32_t x; //!< Azimuth around the x axis [mdeg]
        int32_t y; //!< Azimuth around the y axis [mdeg]
        int32_t z; //!< Azimuth around the z axis [mdeg]
    } Orientation;

    struct {
        uint8_t value; //!< Digital brightness value (8bit)
    } Led;

    struct {
        int32_t x; //!< Movement in x direction in micro-inch
        int32_t y; //!< Movement in y direction in micro-inch
        uint32_t duration; //!< Duration in which the movement occured in microseconds
        uint8_t surfaceQuality; //!< Value denoting the quality of the surface measurement (higher = better)
    } Mouse;
    
    struct {
        uint16_t channels[8]; //!< Channel data of first 8 SBus channels
        uint8_t flags; //!< SBus flags
    } SBus;
} SensorData;

/**
  *
  * @brief The Actuator Union
  *
  * Actuator Data LED
  * - uint8_t: 0 - 255 Digital brightness value
  *
  * Actuator Data MOTOR
  * - uint16_t: Revolutions per minute [rpm]
  *
  * Actuator Data SERVO
  * - uint16_t: Servo deflection angle [deg]
  *
  */
typedef union {
    struct {
        int16_t speed; //!< Revolutions per minute [rpm]
    } Motor;

    struct {
        int16_t angle; //!< Servo deflection angle [deg]
    } Servo;

    struct {
        uint8_t value; //!< Digital brightness value (8bit)
    } Led;

} ActuatorData;

/**
 * @brief The Error struct
 */
typedef struct {
    ErrorCode code; //!< Error code
} Error;

/**
 * @brief The Time struct
 */
typedef struct {
    uint64_t micros; //!< 8 byte microseconds timestamp
} Time;

/**
 * @brief The Message struct
 *
 * Header
 * - 1 byte MessageType
 * - 1 byte SensorType or ActuatorType
 * - 1 byte Sensor ID
 * 
 * Payload, one of
 * - Error struct
 * - Time struct
 * - SensorData struct (depending on SensorType)
 */
struct Message {
    MessageType message;
    union {
        SensorType sensor;
        ActuatorType actuator;
    };
    uint8_t id;
    uint8_t sequence;
    union {
        Error error;
        Time time;
        SensorData sensorData;
        ActuatorData actuatorData;
    };

};

int encodeMessage (const Message*, char*);
int decodeMessage (Message*, const char*);

uint8_t crc8(const char *vptr, unsigned int len);

}  // namespace sense_link

#endif /* SENSE_LINK_MESSAGE_H */
