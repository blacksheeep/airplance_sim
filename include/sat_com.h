#ifndef SAT_COM_H
#define SAT_COM_H

#include "common.h"
#include "bus.h"

#define SATCOM_PORT 5557
#define SATCOM_HOST "localhost"

// Types of satellite messages
typedef enum {
    SAT_MSG_WAYPOINT,     // New waypoint/target
    SAT_MSG_WEATHER,      // Weather update
    SAT_MSG_EMERGENCY,    // Emergency command
    SAT_MSG_STATUS_REQ,   // Request aircraft status
    SAT_MSG_ACK          // Acknowledge receipt
} SatMessageType;

// Weather information
typedef struct {
    double wind_speed;     // knots
    double wind_direction; // degrees
    double turbulence;     // severity 0-10
    double temperature;    // celsius
} WeatherInfo;

// Waypoint information
typedef struct {
    Position position;     // target position
    double speed;         // target speed in knots
    double heading;       // target heading in degrees
    uint32_t eta;        // estimated time of arrival (unix timestamp)
    bool is_final;       // true if this is the final destination
} Waypoint;

// Emergency commands
typedef enum {
    EMERGENCY_NONE = 0,
    EMERGENCY_RETURN_TO_BASE,
    EMERGENCY_DIVERT,
    EMERGENCY_LAND_IMMEDIATELY,
    EMERGENCY_CLIMB_TO_SAFE_ALTITUDE
} EmergencyCommand;

// Complete satellite message
typedef struct {
    SatMessageType type;
    union {
        Waypoint waypoint;
        WeatherInfo weather;
        EmergencyCommand emergency;
    } data;
} SatelliteMessage;

typedef struct SatCom SatCom;

// Initialize satellite communication
SatCom* sat_com_init(Bus* bus);

// Clean up satellite communication
void sat_com_cleanup(SatCom* sat);

// Process one iteration of satellite communication
void sat_com_process(SatCom* sat);

// Main entry point for satellite communication process
void sat_com_main(Bus* bus);

#endif // SAT_COM_H
