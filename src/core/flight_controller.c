#include "flight_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <string.h>
#include <time.h>

#define MAX_COMPONENTS 5

struct FlightController {
    Bus* bus;
    ExtendedFlightState state;
    pid_t component_pids[MAX_COMPONENTS];
    int shm_id;
    bool running;
};

// Forward declarations for component entry points
extern void gps_receiver_main(Bus* bus);
extern void ins_main(Bus* bus);
extern void landing_radio_main(Bus* bus);
extern void sat_com_main(Bus* bus);
extern void autopilot_main(Bus* bus);

FlightController* flight_controller_init(Bus* bus) {
    if (!bus) {
        fprintf(stderr, "Flight controller init: NULL bus\n");
        return NULL;
    }
    
    FlightController* fc = malloc(sizeof(FlightController));
    if (!fc) {
        fprintf(stderr, "Flight controller init: malloc failed\n");
        return NULL;
    }
    
    fc->bus = bus;
    fc->shm_id = bus_get_shm_id(bus);
    if (fc->shm_id == -1) {
        fprintf(stderr, "Flight controller init: failed to get shared memory ID\n");
        free(fc);
        return NULL;
    }

    fc->running = false;
    memset(fc->component_pids, 0, sizeof(fc->component_pids));
    
    // Initialize flight state
    flight_state_init(&fc->state);
    
    // Subscribe to relevant message types
    ErrorCode err;
    if ((err = bus_subscribe(bus, COMPONENT_FLIGHT_CONTROLLER, MSG_POSITION_UPDATE)) != SUCCESS) {
        fprintf(stderr, "Failed to subscribe to position updates: %d\n", err);
        free(fc);
        return NULL;
    }
    if ((err = bus_subscribe(bus, COMPONENT_FLIGHT_CONTROLLER, MSG_STATE_REQUEST)) != SUCCESS) {
        fprintf(stderr, "Failed to subscribe to state requests: %d\n", err);
        free(fc);
        return NULL;
    }
    if ((err = bus_subscribe(bus, COMPONENT_FLIGHT_CONTROLLER, MSG_AUTOPILOT_COMMAND)) != SUCCESS) {
        fprintf(stderr, "Failed to subscribe to autopilot commands: %d\n", err);
        free(fc);
        return NULL;
    }
    if ((err = bus_subscribe(bus, COMPONENT_FLIGHT_CONTROLLER, MSG_SYSTEM_STATUS)) != SUCCESS) {
        fprintf(stderr, "Failed to subscribe to system status: %d\n", err);
        free(fc);
        return NULL;
    }
    
    fprintf(stderr, "Flight controller initialized successfully\n");
    return fc;
}

ErrorCode flight_controller_spawn_component(FlightController* fc, ComponentId component) {
    if (!fc || component >= MAX_COMPONENTS) {
        fprintf(stderr, "Invalid spawn parameters\n");
        return ERROR_GENERAL;
    }
    
    fprintf(stderr, "Spawning component %d...\n", component);
    pid_t pid = fork();
    
    if (pid == -1) {
        perror("fork failed");
        return ERROR_GENERAL;
    }
    
    if (pid == 0) {
        // Child process
        fprintf(stderr, "Child process for component %d started\n", component);
        Bus* child_bus = bus_attach(fc->shm_id);
        if (!child_bus) {
            fprintf(stderr, "Child failed to attach to bus\n");
            exit(EXIT_FAILURE);
        }
        
        // Call appropriate component main function
        switch (component) {
            case COMPONENT_GPS:
                fprintf(stderr, "Starting GPS receiver\n");
                gps_receiver_main(child_bus);
                break;
            case COMPONENT_INS:
                fprintf(stderr, "Starting INS\n");
                ins_main(child_bus);
                break;
            case COMPONENT_LANDING_RADIO:
                fprintf(stderr, "Starting Landing Radio\n");
                landing_radio_main(child_bus);
                break;
            case COMPONENT_SAT_COM:
                fprintf(stderr, "Starting SATCOM\n");
                sat_com_main(child_bus);
                break;
            case COMPONENT_AUTOPILOT:
                fprintf(stderr, "Starting Autopilot\n");
                autopilot_main(child_bus);
                break;
            default:
                fprintf(stderr, "Unknown component type\n");
                exit(EXIT_FAILURE);
        }
        
        bus_detach(child_bus);
        exit(EXIT_SUCCESS);
    }
    
    // Parent process
    fprintf(stderr, "Parent: Component %d spawned with PID %d\n", component, pid);
    fc->component_pids[component] = pid;
    return SUCCESS;
}

void handle_position_update(FlightController* fc, Message* msg) {
    if (!fc || !msg) return;
    
    PositionUpdateMsg* update = &msg->payload.position_update;
    flight_state_update_position(&fc->state, &update->position, msg->header.sender);
    
    // Send state update to autopilot
    Message response = {0};
    response.header.type = MSG_STATE_RESPONSE;
    response.header.sender = COMPONENT_FLIGHT_CONTROLLER;
    response.header.receiver = COMPONENT_AUTOPILOT;
    response.header.timestamp = time(NULL);
    response.header.message_size = sizeof(StateResponseMsg);
    
    memcpy(&response.payload.state_response.state, &fc->state.basic, sizeof(FlightState));
    
    bus_publish(fc->bus, &response);
}

void handle_autopilot_command(FlightController* fc, Message* msg) {
    if (!fc || !msg) return;
    
    AutopilotCommandMsg* cmd = &msg->payload.autopilot_command;
    flight_state_update_autopilot(&fc->state, 
                                cmd->target_altitude,
                                cmd->target_heading,
                                cmd->target_speed);
}

void flight_controller_process_messages(FlightController* fc) {
    if (!fc || !fc->running) return;
    
    Message msg;
    while (bus_read_message(fc->bus, COMPONENT_FLIGHT_CONTROLLER, &msg)) {
        switch (msg.header.type) {
            case MSG_POSITION_UPDATE:
                handle_position_update(fc, &msg);
                break;
                
            case MSG_STATE_REQUEST: {
                Message response = {0};
                response.header.type = MSG_STATE_RESPONSE;
                response.header.sender = COMPONENT_FLIGHT_CONTROLLER;
                response.header.receiver = msg.header.sender;
                response.header.timestamp = time(NULL);
                response.header.message_size = sizeof(StateResponseMsg);
                
                memcpy(&response.payload.state_response.state, &fc->state.basic, sizeof(FlightState));
                bus_publish(fc->bus, &response);
                break;
            }
                
            case MSG_AUTOPILOT_COMMAND:
                handle_autopilot_command(fc, &msg);
                break;
                
            case MSG_SYSTEM_STATUS:
                flight_state_update_system_status(&fc->state, 
                                                msg.header.sender,
                                                true);
                break;

            case MSG_STATE_RESPONSE:
                // Flight controller doesn't handle state responses
                break;
        }
    }
    
    // Check for any terminated child processes
    int status;
    pid_t pid;
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        fprintf(stderr, "Child process %d terminated\n", pid);
        for (int i = 0; i < MAX_COMPONENTS; i++) {
            if (fc->component_pids[i] == pid) {
                flight_state_update_system_status(&fc->state, i, false);
                fc->component_pids[i] = 0;
                
                // Optionally restart the component
                flight_controller_spawn_component(fc, i);
                break;
            }
        }
    }
}

ErrorCode flight_controller_start(FlightController* fc) {
    if (!fc) {
        fprintf(stderr, "NULL flight controller in start\n");
        return ERROR_GENERAL;
    }
    
    // Spawn all components in correct order
    ComponentId components[] = {
        COMPONENT_AUTOPILOT,    // Must be 1
        COMPONENT_GPS,          // Must be 2
        COMPONENT_INS,          // Must be 3
        COMPONENT_LANDING_RADIO // Must be 4
    };
    
    size_t num_components = sizeof(components) / sizeof(components[0]);
    fprintf(stderr, "Starting %zu components...\n", num_components);
    
    for (size_t i = 0; i < num_components; i++) {
        ComponentId component = components[i];
        fprintf(stderr, "Starting component %zu (ID: %d)...\n", i, component);
        
        if (component >= MAX_COMPONENTS) {
            fprintf(stderr, "Invalid component ID %d (MAX_COMPONENTS is %d)\n", 
                    component, MAX_COMPONENTS);
            return ERROR_GENERAL;
        }
        
        ErrorCode err = flight_controller_spawn_component(fc, component);
        if (err != SUCCESS) {
            fprintf(stderr, "Failed to spawn component %d\n", component);
            flight_controller_cleanup(fc);
            return err;
        }
        
        // Small delay between spawns to ensure orderly startup
        usleep(100000);  // 100ms
    }
    
    fc->running = true;
    fprintf(stderr, "All components started successfully\n");
    return SUCCESS;
}

void flight_controller_cleanup(FlightController* fc) {
    if (!fc) return;
    
    fprintf(stderr, "Flight controller: Starting cleanup...\n");
    fc->running = false;
    
    // Terminate all child processes
    for (int i = 0; i < MAX_COMPONENTS; i++) {
        if (fc->component_pids[i] > 0) {
            fprintf(stderr, "Flight controller: Terminating component %d (PID %d)...\n", 
                    i, fc->component_pids[i]);
            
            // Send SIGTERM first
            kill(fc->component_pids[i], SIGTERM);
            
            // Wait a bit for graceful shutdown
            int status;
            pid_t result = waitpid(fc->component_pids[i], &status, WNOHANG);
            
            if (result == 0) {  // Process still running
                usleep(100000);  // Wait 100ms
                
                // Check again
                result = waitpid(fc->component_pids[i], &status, WNOHANG);
                if (result == 0) {
                    // Force kill if still running
                    fprintf(stderr, "Flight controller: Force killing component %d...\n", i);
                    kill(fc->component_pids[i], SIGKILL);
                    waitpid(fc->component_pids[i], NULL, 0);
                }
            }
            
            fc->component_pids[i] = 0;
        }
    }
    
    if (fc->bus) {
        bus_cleanup(fc->bus);
        fc->bus = NULL;
    }
    
    free(fc);
    fprintf(stderr, "Flight controller: Cleanup complete\n");
}

const ExtendedFlightState* flight_controller_get_state(const FlightController* fc) {
    return fc ? &fc->state : NULL;
}
