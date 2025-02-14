#include "autopilot.h"
#include "log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <json-c/json.h>
#include <time.h>

#define UPDATE_INTERVAL_MS 100  // 10Hz update rate
#define STATE_REQUEST_INTERVAL_S 1
#define CONFIG_FILE "config/autopilot_config.json"

// Default PID controller constants
#define DEFAULT_HEADING_P 1.0
#define DEFAULT_HEADING_I 0.1
#define DEFAULT_HEADING_D 0.2

#define DEFAULT_ALTITUDE_P 0.5
#define DEFAULT_ALTITUDE_I 0.05
#define DEFAULT_ALTITUDE_D 0.1

#define DEFAULT_SPEED_P 0.3
#define DEFAULT_SPEED_I 0.02
#define DEFAULT_SPEED_D 0.05

// Default performance limits
#define DEFAULT_MAX_CLIMB_RATE 2000.0    // feet per minute
#define DEFAULT_MAX_DESCENT_RATE 1500.0  // feet per minute
#define DEFAULT_MAX_BANK_ANGLE 25.0      // degrees
#define DEFAULT_MAX_PITCH_ANGLE 15.0     // degrees
#define DEFAULT_MAX_SPEED 350.0          // knots
#define DEFAULT_MIN_SPEED 120.0          // knots
#define DEFAULT_MAX_HEADING_RATE 3.0     // degrees per second

struct Autopilot {
    Bus* bus;
    AutopilotConfig config;
    FlightState current_state;
    bool state_valid;
    time_t last_state_request;
    
    // PID control state
    struct {
        double heading_error;
        double heading_integral;
        double heading_last_error;
        double altitude_error;
        double altitude_integral;
        double altitude_last_error;
        double speed_error;
        double speed_integral;
        double speed_last_error;
    } pid_state;
};

static void request_state(Autopilot* ap) {
    Message msg = {0};
    msg.header.type = MSG_STATE_REQUEST;
    msg.header.sender = COMPONENT_AUTOPILOT;
    msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
    msg.header.timestamp = time(NULL);
    
    if (bus_publish(ap->bus, &msg) != SUCCESS) {
        LOG_ERROR(LOG_AUTOPILOT, "Failed to publish state request");
    } else {
        LOG_DEBUG(LOG_AUTOPILOT, "Requested state update");
    }
    
    ap->last_state_request = msg.header.timestamp;
}

static void send_control_command(Autopilot* ap, double target_heading, 
                               double target_speed, double target_altitude) {
    Message msg = {0};
    msg.header.type = MSG_AUTOPILOT_COMMAND;
    msg.header.sender = COMPONENT_AUTOPILOT;
    msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
    msg.header.timestamp = time(NULL);
    msg.header.message_size = sizeof(AutopilotCommandMsg);

    msg.payload.autopilot_command.target_heading = target_heading;
    msg.payload.autopilot_command.target_speed = target_speed;
    msg.payload.autopilot_command.target_altitude = target_altitude;

    if (bus_publish(ap->bus, &msg) != SUCCESS) {
        LOG_ERROR(LOG_AUTOPILOT, "Failed to publish control command");
    } else {
        LOG_DEBUG(LOG_AUTOPILOT, "Published control command - Hdg: %.1f°, Spd: %.1f kts, Alt: %.0f ft",
                 target_heading, target_speed, target_altitude);
    }
}

static void update_pid_controls(Autopilot* ap) {
    double dt = UPDATE_INTERVAL_MS / 1000.0;

    // Heading control
    double heading_error = ap->config.target_heading - ap->current_state.heading;
    // Normalize heading error to [-180, 180]
    if (heading_error > 180) heading_error -= 360;
    if (heading_error < -180) heading_error += 360;

    ap->pid_state.heading_integral += heading_error * dt;
    double heading_derivative = (heading_error - ap->pid_state.heading_last_error) / dt;

    double heading_output = ap->config.heading_pid[0] * heading_error +
                          ap->config.heading_pid[1] * ap->pid_state.heading_integral +
                          ap->config.heading_pid[2] * heading_derivative;

    // Limit heading rate
    heading_output = fmax(-ap->config.max_heading_rate,
                         fmin(ap->config.max_heading_rate, heading_output));

    ap->pid_state.heading_last_error = heading_error;

    // Altitude control
    double altitude_error = ap->config.target_altitude - ap->current_state.position.altitude;
    ap->pid_state.altitude_integral += altitude_error * dt;
    double altitude_derivative = (altitude_error - ap->pid_state.altitude_last_error) / dt;

    double altitude_output = ap->config.altitude_pid[0] * altitude_error +
                           ap->config.altitude_pid[1] * ap->pid_state.altitude_integral +
                           ap->config.altitude_pid[2] * altitude_derivative;

    // Limit vertical speed
    if (altitude_output > 0) {
        altitude_output = fmin(altitude_output, ap->config.max_climb_rate);
    } else {
        altitude_output = fmax(altitude_output, -ap->config.max_descent_rate);
    }

    ap->pid_state.altitude_last_error = altitude_error;

    // Speed control
    double speed_error = ap->config.target_speed - ap->current_state.speed;
    ap->pid_state.speed_integral += speed_error * dt;
    double speed_derivative = (speed_error - ap->pid_state.speed_last_error) / dt;

    double speed_output = ap->config.speed_pid[0] * speed_error +
                        ap->config.speed_pid[1] * ap->pid_state.speed_integral +
                        ap->config.speed_pid[2] * speed_derivative;

    // Limit speed
    speed_output = fmax(ap->config.min_speed,
                       fmin(ap->config.max_speed, ap->current_state.speed + speed_output)) -
                  ap->current_state.speed;

    ap->pid_state.speed_last_error = speed_error;

    LOG_TRACE(LOG_AUTOPILOT, "PID outputs - Hdg: %.2f, Alt: %.2f, Spd: %.2f",
              heading_output, altitude_output, speed_output);

    LOG_DEBUG(LOG_AUTOPILOT, "Errors - Hdg: %.1f°, Alt: %.0f ft, Spd: %.1f kts",
              heading_error, altitude_error, speed_error);

    // Apply control outputs with limits
    double new_heading = ap->current_state.heading + heading_output;
    // Normalize heading to [0, 360]
    while (new_heading >= 360.0) new_heading -= 360.0;
    while (new_heading < 0.0) new_heading += 360.0;

    double new_altitude = ap->current_state.position.altitude + altitude_output;
    double new_speed = ap->current_state.speed + speed_output;

    // Send command
    send_control_command(ap, new_heading, new_speed, new_altitude);
}

Autopilot* autopilot_init(Bus* bus) {
    LOG_INFO(LOG_AUTOPILOT, "Starting initialization");
    
    if (!bus) {
        LOG_ERROR(LOG_AUTOPILOT, "NULL bus in init");
        return NULL;
    }

    Autopilot* ap = malloc(sizeof(Autopilot));
    if (!ap) {
        LOG_ERROR(LOG_AUTOPILOT, "Failed to allocate memory");
        return NULL;
    }

    ap->bus = bus;
    ap->state_valid = false;
    ap->last_state_request = 0;
    memset(&ap->current_state, 0, sizeof(FlightState));
    memset(&ap->pid_state, 0, sizeof(ap->pid_state));

    // Load configuration
    LOG_INFO(LOG_AUTOPILOT, "Loading config from %s", CONFIG_FILE);
    ap->config = autopilot_load_config(CONFIG_FILE);

    // Subscribe to state responses
    ErrorCode err = bus_subscribe(bus, COMPONENT_AUTOPILOT, MSG_STATE_RESPONSE);
    if (err != SUCCESS) {
        LOG_ERROR(LOG_AUTOPILOT, "Failed to subscribe to state responses: %d", err);
        free(ap);
        return NULL;
    }

    LOG_INFO(LOG_AUTOPILOT, "Initialization complete");
    return ap;
}

void autopilot_cleanup(Autopilot* ap) {
    if (!ap) return;
    LOG_INFO(LOG_AUTOPILOT, "Cleaning up");
    free(ap);
}

AutopilotConfig autopilot_load_config(const char* filename) {
    AutopilotConfig config = {
        // Initial waypoint
        .target_latitude = 37.7749,      // San Francisco by default
        .target_longitude = -122.4194,
        .target_altitude = 10000.0,
        .target_speed = 250.0,
        .target_heading = 0.0,           // Will be calculated from lat/lon

        // Performance limits
        .max_climb_rate = DEFAULT_MAX_CLIMB_RATE,
        .max_descent_rate = DEFAULT_MAX_DESCENT_RATE,
        .max_bank_angle = DEFAULT_MAX_BANK_ANGLE,
        .max_pitch_angle = DEFAULT_MAX_PITCH_ANGLE,
        .max_speed = DEFAULT_MAX_SPEED,
        .min_speed = DEFAULT_MIN_SPEED,
        .max_heading_rate = DEFAULT_MAX_HEADING_RATE,

        // PID gains
        .heading_pid = {DEFAULT_HEADING_P, DEFAULT_HEADING_I, DEFAULT_HEADING_D},
        .altitude_pid = {DEFAULT_ALTITUDE_P, DEFAULT_ALTITUDE_I, DEFAULT_ALTITUDE_D},
        .speed_pid = {DEFAULT_SPEED_P, DEFAULT_SPEED_I, DEFAULT_SPEED_D}
    };

    LOG_INFO(LOG_AUTOPILOT, "Loading config from %s", filename);

    json_object *root = json_object_from_file(filename);
    if (!root) {
        LOG_WARN(LOG_AUTOPILOT, "Failed to load config file, using defaults");
        return config;
    }

    // Load waypoint configuration
    json_object_object_foreach(root, key, val) {
        if (strcmp(key, "target_latitude") == 0)
            config.target_latitude = json_object_get_double(val);
        else if (strcmp(key, "target_longitude") == 0)
            config.target_longitude = json_object_get_double(val);
        else if (strcmp(key, "target_altitude") == 0)
            config.target_altitude = json_object_get_double(val);
        else if (strcmp(key, "target_speed") == 0)
            config.target_speed = json_object_get_double(val);
        else if (strcmp(key, "target_heading") == 0)
            config.target_heading = json_object_get_double(val);
        
        // Load performance limits
        else if (strcmp(key, "max_climb_rate") == 0)
            config.max_climb_rate = json_object_get_double(val);
        else if (strcmp(key, "max_descent_rate") == 0)
            config.max_descent_rate = json_object_get_double(val);
        else if (strcmp(key, "max_bank_angle") == 0)
            config.max_bank_angle = json_object_get_double(val);
        else if (strcmp(key, "max_pitch_angle") == 0)
            config.max_pitch_angle = json_object_get_double(val);
        else if (strcmp(key, "max_speed") == 0)
            config.max_speed = json_object_get_double(val);
        else if (strcmp(key, "min_speed") == 0)
            config.min_speed = json_object_get_double(val);
        else if (strcmp(key, "max_heading_rate") == 0)
            config.max_heading_rate = json_object_get_double(val);
        
        // Load PID gains if present
        else if (strcmp(key, "heading_pid") == 0) {
            json_object* pid_array = val;
            if (json_object_get_type(pid_array) == json_type_array &&
                json_object_array_length(pid_array) == 3) {
                for (int i = 0; i < 3; i++) {
                    config.heading_pid[i] = json_object_get_double(
                        json_object_array_get_idx(pid_array, i));
                }
            }
        }
        else if (strcmp(key, "altitude_pid") == 0) {
            json_object* pid_array = val;
            if (json_object_get_type(pid_array) == json_type_array &&
                json_object_array_length(pid_array) == 3) {
                for (int i = 0; i < 3; i++) {
                    config.altitude_pid[i] = json_object_get_double(
                        json_object_array_get_idx(pid_array, i));
                }
            }
        }
        else if (strcmp(key, "speed_pid") == 0) {
            json_object* pid_array = val;
            if (json_object_get_type(pid_array) == json_type_array &&
                json_object_array_length(pid_array) == 3) {
                for (int i = 0; i < 3; i++) {
                    config.speed_pid[i] = json_object_get_double(
                        json_object_array_get_idx(pid_array, i));
                }
            }
        }
    }

    json_object_put(root);

    // Calculate initial heading if not specified
    if (config.target_heading == 0.0) {
        // Calculate heading to target waypoint
        double lat1 = DEG_TO_RAD(config.target_latitude);
        double lon1 = DEG_TO_RAD(config.target_longitude);
        double lat2 = DEG_TO_RAD(37.7749);  // Current position (San Francisco)
        double lon2 = DEG_TO_RAD(-122.4194);

        double y = sin(lon2 - lon1) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - 
                  sin(lat1) * cos(lat2) * cos(lon2 - lon1);
        config.target_heading = RAD_TO_DEG(atan2(y, x));
        if (config.target_heading < 0) {
            config.target_heading += 360.0;
        }
    }

    LOG_INFO(LOG_AUTOPILOT, "Loaded config - Target: %.6f,%.6f @ %.0f ft, Hdg: %.1f°, Spd: %.0f kts",
             config.target_latitude, config.target_longitude,
             config.target_altitude, config.target_heading, config.target_speed);

    return config;
}

void autopilot_process(Autopilot* ap) {
    if (!ap) {
        LOG_ERROR(LOG_AUTOPILOT, "NULL autopilot in process");
        return;
    }

    // Request current state if needed
    time_t now = time(NULL);
    if (now - ap->last_state_request >= STATE_REQUEST_INTERVAL_S) {
        LOG_DEBUG(LOG_AUTOPILOT, "Requesting state update");
        request_state(ap);
    }

    // Process any incoming messages
    Message msg;
    while (bus_read_message(ap->bus, COMPONENT_AUTOPILOT, &msg)) {
        LOG_TRACE(LOG_AUTOPILOT, "Received message type %d from component %d",
                 msg.header.type, msg.header.sender);

        if (msg.header.type == MSG_STATE_RESPONSE) {
            memcpy(&ap->current_state, &msg.payload.state_response.state, 
                   sizeof(FlightState));
            ap->state_valid = true;
            LOG_DEBUG(LOG_AUTOPILOT, "State updated - Pos: %.6f,%.6f @ %.0f ft, Hdg: %.1f°, Spd: %.1f kts",
                     ap->current_state.position.latitude,
                     ap->current_state.position.longitude,
                     ap->current_state.position.altitude,
                     ap->current_state.heading,
                     ap->current_state.speed);
        }
    }

    // Update controls if we have valid state
    if (ap->state_valid) {
        update_pid_controls(ap);
    } else {
        LOG_WARN(LOG_AUTOPILOT, "Skipping control update - no valid state");
    }
}

void autopilot_main(Bus* bus) {
    LOG_INFO(LOG_AUTOPILOT, "Starting main function");
    
    Autopilot* ap = autopilot_init(bus);
    if (!ap) {
        LOG_ERROR(LOG_AUTOPILOT, "Failed to initialize");
        return;
    }

    LOG_INFO(LOG_AUTOPILOT, "Entering main loop");
    
    while (1) {
        autopilot_process(ap);
        usleep(UPDATE_INTERVAL_MS * 1000);
    }

    autopilot_cleanup(ap);
}
