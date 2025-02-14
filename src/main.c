#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <stdbool.h>
#include "bus.h"
#include "flight_controller.h"
#include "common.h"

static volatile bool running = true;
static FlightController* controller = NULL;
static Bus* bus = NULL;

// Forward declaration
static void cleanup(void);

// Signal handler for graceful shutdown
static void handle_signal(int sig) {
    fprintf(stderr, "\nReceived signal %d, initiating shutdown...\n", sig);
    running = false;
}

// Cleanup handler that will be called on exit
static void cleanup(void) {
    fprintf(stderr, "Performing cleanup...\n");
    
    if (controller) {
        flight_controller_cleanup(controller);
        controller = NULL;
    }
    
    if (bus) {
        bus_cleanup(bus);
        bus = NULL;
    }
    
    fprintf(stderr, "Cleanup complete\n");
}

// Print simulation status
static void print_status(const FlightController* fc) {
    static time_t last_print = 0;
    time_t now = time(NULL);

    if (now - last_print >= 1) {  // Update every second
        const ExtendedFlightState* state = flight_controller_get_state(fc);
        if (state) {
            char buffer[1024];
            flight_state_to_string(state, buffer, sizeof(buffer));
            printf("\033[2J\033[H");  // Clear screen and move cursor to top
            printf("%s\n", buffer);
        }
        last_print = now;
    }
}

int main(void) {
    // Setup signal handlers
    struct sigaction sa = {0};
    sa.sa_handler = handle_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    
    // Register cleanup handler
    atexit(cleanup);

    fprintf(stderr, "Starting aircraft simulation...\n");

    // Initialize message bus
    bus = bus_init();
    if (!bus) {
        fprintf(stderr, "Failed to initialize message bus\n");
        return 1;
    }

    // Initialize flight controller
    controller = flight_controller_init(bus);
    if (!controller) {
        fprintf(stderr, "Failed to initialize flight controller\n");
        return 1;
    }

    // Start flight controller (this will fork component processes)
    if (flight_controller_start(controller) != SUCCESS) {
        fprintf(stderr, "Failed to start flight controller\n");
        return 1;
    }

    fprintf(stderr, "All systems initialized. Running simulation...\n");

    // Main loop
    while (running) {
        flight_controller_process_messages(controller);
        print_status(controller);
        usleep(10000);  // 10ms sleep
    }

    fprintf(stderr, "Simulation shutdown complete\n");
    return 0;
}
