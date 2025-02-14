#ifndef MESSAGES_H
#define MESSAGES_H

#include "common.h"

#define MAX_MESSAGE_SIZE 1024

// Message types must match the protocol being used
typedef enum {
    MSG_POSITION_UPDATE = 0,    // Position updates from navigation components
    MSG_STATE_REQUEST = 1,      // Request for current state
    MSG_STATE_RESPONSE = 2,     // Response with current state
    MSG_AUTOPILOT_COMMAND = 3,  // Commands from autopilot
    MSG_SYSTEM_STATUS = 4       // System status updates
} MessageType;

// Message header structure
typedef struct {
    MessageType type;
    ComponentId sender;
    ComponentId receiver;
    uint32_t timestamp;
    uint32_t message_size;
} MessageHeader;

// Message payload structures
typedef struct {
    Position position;
} PositionUpdateMsg;

typedef struct {
    FlightState state;
} StateResponseMsg;

typedef struct {
    double target_heading;
    double target_speed;
    double target_altitude;
} AutopilotCommandMsg;

typedef struct {
    bool component_active;
} SystemStatusMsg;

// Complete message structure
typedef struct {
    MessageHeader header;
    union {
        PositionUpdateMsg position_update;
        StateResponseMsg state_response;
        AutopilotCommandMsg autopilot_command;
        SystemStatusMsg system_status;
    } payload;
} Message;

// Validation macros
#define VALIDATE_MESSAGE_TYPE(type) ((type) >= MSG_POSITION_UPDATE && (type) <= MSG_SYSTEM_STATUS)

#endif // MESSAGES_H
