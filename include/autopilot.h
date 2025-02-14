#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "common.h"
#include "bus.h"

// Autopilot configuration structure
typedef struct {
    // Target waypoint
    double target_latitude;
    double target_longitude;
    double target_altitude;
    double target_speed;
    double target_heading;  // calculated from lat/lon but can be overridden

    // Performance limits
    double max_climb_rate;    // feet per minute
    double max_descent_rate;  // feet per minute
    double max_bank_angle;    // degrees
    double max_pitch_angle;   // degrees
    double max_speed;         // knots
    double min_speed;         // knots
    double max_heading_rate;  // degrees per second

    // Control gains
    double heading_pid[3];    // P, I, D gains for heading control
    double altitude_pid[3];   // P, I, D gains for altitude control
    double speed_pid[3];      // P, I, D gains for speed control
} AutopilotConfig;

typedef struct Autopilot Autopilot;

// Initialize autopilot
Autopilot* autopilot_init(Bus* bus);

// Clean up autopilot
void autopilot_cleanup(Autopilot* ap);

// Process one iteration of autopilot control
void autopilot_process(Autopilot* ap);

// Load configuration from JSON file
AutopilotConfig autopilot_load_config(const char* filename);

// Main entry point for autopilot process
void autopilot_main(Bus* bus);

#endif // AUTOPILOT_H
