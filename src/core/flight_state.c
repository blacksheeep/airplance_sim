#include "flight_state.h"
#include "log.h"
#include <stdio.h>
#include <string.h>

void flight_state_init(ExtendedFlightState* state) {
    if (!state) return;
    
    memset(state, 0, sizeof(ExtendedFlightState));
    
    // Set some reasonable defaults
    state->basic.heading = 0.0;
    state->basic.speed = 0.0;
    state->basic.vertical_speed = 0.0;
    state->basic.timestamp = time(NULL);
    
    state->parameters.pitch = 0.0;
    state->parameters.roll = 0.0;
    state->parameters.yaw = 0.0;
    state->parameters.thrust = 0.0;
    
    state->autopilot.enabled = false;
    state->autopilot.target_altitude = 0.0;
    state->autopilot.target_heading = 0.0;
    state->autopilot.target_speed = 0.0;
    
    state->system_status.last_update_time = time(NULL);
}

void flight_state_update_position(ExtendedFlightState* state, const Position* pos, ComponentId source) {
    if (!state || !pos) return;

    // Update timestamp
    state->basic.timestamp = time(NULL);
    state->system_status.last_update_time = state->basic.timestamp;

    // Update position based on source
    switch (source) {
        case COMPONENT_GPS:
            state->nav_data.gps_valid = true;
            state->nav_data.gps_position = *pos;
            break;

        case COMPONENT_INS:
            state->nav_data.ins_valid = true;
            state->nav_data.ins_position = *pos;
            break;

        case COMPONENT_LANDING_RADIO:
            state->nav_data.radio_valid = true;
            state->nav_data.radio_position = *pos;
            break;

        default:
            return;
    }

    // Update basic position with best available position
    state->basic.position = flight_state_get_best_position(state);
}

void flight_state_update_parameters(ExtendedFlightState* state, double pitch, double roll, double yaw, double thrust) {
    if (!state) return;
    
    state->parameters.pitch = pitch;
    state->parameters.roll = roll;
    state->parameters.yaw = yaw;
    state->parameters.thrust = thrust;
    
    state->basic.timestamp = time(NULL);
    state->system_status.last_update_time = state->basic.timestamp;
}

void flight_state_update_autopilot(ExtendedFlightState* state, double target_altitude, double target_heading, double target_speed) {
    if (!state) return;
    
    state->autopilot.target_altitude = target_altitude;
    state->autopilot.target_heading = target_heading;
    state->autopilot.target_speed = target_speed;
    
    state->basic.timestamp = time(NULL);
    state->system_status.last_update_time = state->basic.timestamp;
}

void flight_state_update_system_status(ExtendedFlightState* state, ComponentId component, bool connected) {
    if (!state) return;
    
    switch (component) {
        case COMPONENT_GPS:
            state->system_status.gps_connected = connected;
            if (!connected) {
                state->nav_data.gps_valid = false;  // Invalidate GPS data when disconnected
                LOG_INFO(LOG_CORE, "GPS disconnected, invalidating position data");
            }
            break;
            
        case COMPONENT_INS:
            state->system_status.ins_operational = connected;
            if (!connected) {
                state->nav_data.ins_valid = false;  // Invalidate INS data when not operational
                LOG_INFO(LOG_CORE, "INS offline, invalidating position data");
            }
            break;
            
        case COMPONENT_LANDING_RADIO:
            state->system_status.landing_radio_connected = connected;
            if (!connected) {
                state->nav_data.radio_valid = false;  // Invalidate radio data when disconnected
                LOG_INFO(LOG_CORE, "Landing radio disconnected, invalidating position data");
            }
            break;
            
        case COMPONENT_SAT_COM:
            state->system_status.sat_com_connected = connected;
            break;
            
        default:
            return;
    }
    
    state->basic.timestamp = time(NULL);
    state->system_status.last_update_time = state->basic.timestamp;
    
    // Update basic position after status change
    state->basic.position = flight_state_get_best_position(state);
}

Position flight_state_get_best_position(const ExtendedFlightState* state) {
    if (!state) {
        Position invalid = {0};
        return invalid;
    }
    
    // Priority: GPS > INS > Radio
    if (state->nav_data.gps_valid) {
        return state->nav_data.gps_position;
    }
    
    if (state->nav_data.ins_valid) {
        return state->nav_data.ins_position;
    }
    
    if (state->nav_data.radio_valid) {
        return state->nav_data.radio_position;
    }
    
    // If no valid position available, return current basic position
    return state->basic.position;
}

const char* flight_state_to_string(const ExtendedFlightState* state, char* buffer, size_t buffer_size) {
    if (!state || !buffer || buffer_size == 0) return NULL;
    
    Position pos = state->basic.position;
    snprintf(buffer, buffer_size,
        "Flight State:\n"
        "Position: %.6f, %.6f, %.1f\n"
        "Heading: %.1f°, Speed: %.1f kts, VS: %.1f fpm\n"
        "Parameters - Pitch: %.1f°, Roll: %.1f°, Yaw: %.1f°, Thrust: %.1f%%\n"
        "Autopilot - %s, Target Alt: %.1f, Hdg: %.1f°, Spd: %.1f\n"
        "Systems - GPS: %s, INS: %s, Radio: %s, SatCom: %s",
        pos.latitude, pos.longitude, pos.altitude,
        state->basic.heading, state->basic.speed, state->basic.vertical_speed,
        state->parameters.pitch, state->parameters.roll, state->parameters.yaw, state->parameters.thrust,
        state->autopilot.enabled ? "ON" : "OFF",
        state->autopilot.target_altitude, state->autopilot.target_heading, state->autopilot.target_speed,
        state->system_status.gps_connected ? "OK" : "DISC",
        state->system_status.ins_operational ? "OK" : "FAIL",
        state->system_status.landing_radio_connected ? "OK" : "DISC",
        state->system_status.sat_com_connected ? "OK" : "DISC"
    );
    
    return buffer;
}

bool flight_state_is_valid(const ExtendedFlightState* state) {
    if (!state) return false;
    
    // Check if we have at least one valid position source
    if (!state->nav_data.ins_valid && 
        !state->nav_data.gps_valid && 
        !state->nav_data.radio_valid) {
        return false;
    }
    
    // Check if the state is too old (more than 10 seconds)
    time_t now = time(NULL);
    if (now - state->system_status.last_update_time > 10) {
        return false;
    }
    
    return true;
}
