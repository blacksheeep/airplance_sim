#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "common.h"
#include "bus.h"
#include "flight_state.h"

typedef struct FlightController FlightController;

// Initialize the flight controller
FlightController* flight_controller_init(Bus* bus);

// Start the flight controller and spawn component processes
ErrorCode flight_controller_start(FlightController* fc);

// Clean up and shutdown
void flight_controller_cleanup(FlightController* fc);

// Process incoming messages (called in main loop)
void flight_controller_process_messages(FlightController* fc);

// Get current flight state
const ExtendedFlightState* flight_controller_get_state(const FlightController* fc);

// Utility functions
ErrorCode flight_controller_spawn_component(FlightController* fc, ComponentId component);
void flight_controller_handle_component_exit(FlightController* fc, ComponentId component);

#endif // FLIGHT_CONTROLLER_H
