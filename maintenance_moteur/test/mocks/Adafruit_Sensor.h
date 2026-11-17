#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>

// =============================================================================
// Adafruit Unified Sensor Driver Compatibility Header
// =============================================================================

#define SENSOR_TYPE_ACCELEROMETER         (1)
#define SENSOR_TYPE_MAGNETIC_FIELD        (2)
#define SENSOR_TYPE_ORIENTATION           (3)
#define SENSOR_TYPE_GYROSCOPE             (4)
#define SENSOR_TYPE_LIGHT                 (5)
#define SENSOR_TYPE_PRESSURE              (6)
#define SENSOR_TYPE_PROXIMITY             (8)
#define SENSOR_TYPE_GRAVITY               (9)
#define SENSOR_TYPE_LINEAR_ACCELERATION   (10)
#define SENSOR_TYPE_ROTATION_VECTOR       (11)
#define SENSOR_TYPE_RELATIVE_HUMIDITY     (12)
#define SENSOR_TYPE_AMBIENT_TEMPERATURE   (13)
#define SENSOR_TYPE_VOLTAGE               (15)
#define SENSOR_TYPE_CURRENT               (16)
#define SENSOR_TYPE_COLOR                 (17)

typedef struct {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        struct {
            float roll;
            float pitch;
            float heading;
        };
    };
    int8_t status;
    uint8_t reserved[3];
} sensors_vec_t;

typedef struct {
    union {
        float c[3];
        struct {
            float r;
            float g;
            float b;
        };
        uint32_t rgba;
    };
} sensors_color_t;

typedef struct {
    int32_t version;
    int32_t sensor_id;
    int32_t type;
    int32_t reserved0;
    int32_t timestamp;
    union {
        float data[4];
        sensors_vec_t acceleration;
        sensors_vec_t magnetic;
        sensors_vec_t orientation;
        sensors_vec_t gyro;
        float temperature;
        float distance;
        float light;
        float pressure;
        float relative_humidity;
        float current;
        float voltage;
        sensors_color_t color;
    };
} sensors_event_t;

typedef struct {
    char name[12];
    int32_t version;
    int32_t sensor_id;
    int32_t type;
    float max_value;
    float min_value;
    float resolution;
    int32_t min_delay;
} sensor_t;

class Adafruit_Sensor {
public:
    virtual ~Adafruit_Sensor() = default;
    virtual bool enableAutoRange(bool enabled) {
        (void)enabled;
        return false;
    }
    virtual bool getEvent(sensors_event_t *event) = 0;
    virtual void getSensor(sensor_t *sensor) = 0;
};
