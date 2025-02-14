#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include "common.h"
#include <stdbool.h>
#include <time.h>

// Extended flight state information
typedef struct {
    // Basic position and movement (from common.h)
    FlightState basic;
    
    // Navigation data
    struct {
        bool gps_valid;
        bool ins_valid;
        bool radio_valid;
        Position gps_position;
        Position ins_position;
        Position radio_position;
    } nav_data;

    // Aircraft parameters
    struct {
        double pitch;        // degrees (-90 to +90)
        double roll;         // degrees (-180 to +180)
        double yaw;         // degrees (0 to 360)
        double thrust;      // percentage (0 to 100)
    } parameters;

    // Autopilot state
    struct {
        bool enabled;
        double target_altitude;
        double target_heading;
        double target_speed;
    } autopilot;

    // System status
    struct {
        bool gps_connected;
        bool ins_operational;
        bool landing_radio_connected;
        bool sat_com_connected;
        time_t last_update_time;
    } system_status;

} ExtendedFlightState;

// Initialize flight state with default values
void flight_state_init(ExtendedFlightState* state);

// Update functions for different components
void flight_state_update_position(ExtendedFlightState* state, const Position* pos, ComponentId source);
void flight_state_update_parameters(ExtendedFlightState* state, double pitch, double roll, double yaw, double thrust);
void flight_state_update_autopilot(ExtendedFlightState* state, double target_altitude, double target_heading, double target_speed);
void flight_state_update_system_status(ExtendedFlightState* state, ComponentId component, bool connected);

// Get the best available position based on priority (INS > GPS > Radio)
Position flight_state_get_best_position(const ExtendedFlightState* state);

// Utility functions
const char* flight_state_to_string(const ExtendedFlightState* state, char* buffer, size_t buffer_size);
bool flight_state_is_valid(const ExtendedFlightState* state);

#endif // FLIGHT_STATE_H
