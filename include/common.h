#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Math constants
#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / PI)

// Physical constants
#define GRAVITY 9.81           // m/s^2
#define EARTH_RADIUS 6371000.0 // meters
#define KNOTS_TO_MS 0.51444   // Convert knots to meters/second

// Component IDs must be less than MAX_COMPONENTS (5)
typedef enum {
    COMPONENT_FLIGHT_CONTROLLER = 0,
    COMPONENT_AUTOPILOT = 1,
    COMPONENT_GPS = 2,
    COMPONENT_INS = 3,
    COMPONENT_LANDING_RADIO = 4,
    COMPONENT_SAT_COM = 5
} ComponentId;

// Basic position structure
typedef struct {
    double latitude;
    double longitude;
    double altitude;
} Position;

// Basic flight state structure
typedef struct {
    Position position;
    double heading;        // degrees
    double speed;         // knots
    double vertical_speed; // feet per minute
    uint32_t timestamp;   // unix timestamp
} FlightState;

// Error handling
typedef enum {
    SUCCESS = 0,
    ERROR_GENERAL,
    ERROR_COMMUNICATION,
    ERROR_INVALID_DATA
} ErrorCode;

// Configuration constants
#define MAX_COMPONENTS 5

// Validation macros
#define VALIDATE_COMPONENT_ID(id) ((id) >= 0 && (id) < MAX_COMPONENTS)

#endif // COMMON_H
