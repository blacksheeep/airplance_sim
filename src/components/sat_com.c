#include "sat_com.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#define BUFFER_SIZE 1024
#define SATCOM_UPDATE_INTERVAL_MS 1000  // 1 Hz update rate

struct SatCom {
    Bus* bus;
    int socket_fd;
    struct sockaddr_in server_addr;
    bool connected;
    SatelliteMessage last_message;
    FlightState current_state;
};

// Initialize socket connection
static bool init_socket(SatCom* sat) {
    sat->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sat->socket_fd < 0) {
        perror("Failed to create socket");
        return false;
    }

    // Set socket to non-blocking
    int flags = fcntl(sat->socket_fd, F_GETFL, 0);
    fcntl(sat->socket_fd, F_SETFL, flags | O_NONBLOCK);

    // Setup server address
    memset(&sat->server_addr, 0, sizeof(sat->server_addr));
    sat->server_addr.sin_family = AF_INET;
    sat->server_addr.sin_port = htons(SATCOM_PORT);

    struct hostent* server = gethostbyname(SATCOM_HOST);
    if (server == NULL) {
        fprintf(stderr, "Failed to resolve hostname\n");
        return false;
    }
    memcpy(&sat->server_addr.sin_addr.s_addr, server->h_addr, server->h_length);

    return true;
}

SatCom* sat_com_init(Bus* bus) {
    if (!bus) return NULL;

    SatCom* sat = malloc(sizeof(SatCom));
    if (!sat) return NULL;

    sat->bus = bus;
    sat->connected = false;
    memset(&sat->last_message, 0, sizeof(SatelliteMessage));
    memset(&sat->current_state, 0, sizeof(FlightState));

    if (!init_socket(sat)) {
        free(sat);
        return NULL;
    }

    // Subscribe to state updates from flight controller
    bus_subscribe(bus, COMPONENT_SAT_COM, MSG_STATE_RESPONSE);

    return sat;
}

void sat_com_cleanup(SatCom* sat) {
    if (!sat) return;
    
    if (sat->socket_fd >= 0) {
        close(sat->socket_fd);
    }
    free(sat);
}

static void try_connect(SatCom* sat) {
    if (sat->connected) return;

    int result = connect(sat->socket_fd, 
                        (struct sockaddr*)&sat->server_addr, 
                        sizeof(sat->server_addr));

    if (result == 0 || (result < 0 && errno == EISCONN)) {
        sat->connected = true;
        fprintf(stderr, "SATCOM: Connected to ground station\n");
    }
}

static void handle_waypoint(SatCom* sat, const Waypoint* waypoint) {
    // Send autopilot command with new target
    Message msg = {0};
    msg.header.type = MSG_AUTOPILOT_COMMAND;
    msg.header.sender = COMPONENT_SAT_COM;
    msg.header.receiver = COMPONENT_AUTOPILOT;
    msg.header.timestamp = time(NULL);
    msg.header.message_size = sizeof(AutopilotCommandMsg);

    msg.payload.autopilot_command.target_altitude = waypoint->position.altitude;
    msg.payload.autopilot_command.target_heading = waypoint->heading;
    msg.payload.autopilot_command.target_speed = waypoint->speed;

    bus_publish(sat->bus, &msg);
}

static void handle_emergency(SatCom* sat, EmergencyCommand cmd) {
    Message msg = {0};
    msg.header.type = MSG_AUTOPILOT_COMMAND;
    msg.header.sender = COMPONENT_SAT_COM;
    msg.header.receiver = COMPONENT_AUTOPILOT;
    msg.header.timestamp = time(NULL);

    switch (cmd) {
        case EMERGENCY_RETURN_TO_BASE:
            // Set course to home base
            msg.payload.autopilot_command.target_heading = 280.0;  // SFO runway heading
            msg.payload.autopilot_command.target_altitude = 3000.0;
            msg.payload.autopilot_command.target_speed = 200.0;
            break;

        case EMERGENCY_CLIMB_TO_SAFE_ALTITUDE:
            msg.payload.autopilot_command.target_altitude = 
                sat->current_state.position.altitude + 5000.0;  // Climb 5000 feet
            break;

        case EMERGENCY_LAND_IMMEDIATELY:
            // Find nearest airport logic would go here
            // For now, just descend
            msg.payload.autopilot_command.target_altitude = 
                sat->current_state.position.altitude - 1000.0;
            msg.payload.autopilot_command.target_speed = 150.0;
            break;

        default:
            return;
    }

    bus_publish(sat->bus, &msg);
}

static void handle_weather(SatCom* sat, const WeatherInfo* weather) {
    // Adjust flight parameters based on weather
    // For example, reduce speed in turbulence
    if (weather->turbulence > 5.0) {
        Message msg = {0};
        msg.header.type = MSG_AUTOPILOT_COMMAND;
        msg.header.sender = COMPONENT_SAT_COM;
        msg.header.receiver = COMPONENT_AUTOPILOT;
        msg.header.timestamp = time(NULL);

        // Reduce speed by 20% in turbulence
        msg.payload.autopilot_command.target_speed = 
            sat->current_state.speed * 0.8;

        bus_publish(sat->bus, &msg);
    }
}

// Parse satellite message from sender
static bool parse_sat_message(const char* buffer, SatelliteMessage* msg) {
    // Expected format: "TYPE,DATA1,DATA2,..."
    char type_str[32];
    if (sscanf(buffer, "%31[^,]", type_str) != 1) {
        return false;
    }

    const char* data = strchr(buffer, ',');
    if (!data) return false;
    data++; // Skip comma

    if (strcmp(type_str, "WAYPOINT") == 0) {
        msg->type = SAT_MSG_WAYPOINT;
        Waypoint* wp = &msg->data.waypoint;
        return sscanf(data, "%lf,%lf,%lf,%lf,%lf,%u,%d",
                     &wp->position.latitude,
                     &wp->position.longitude,
                     &wp->position.altitude,
                     &wp->speed,
                     &wp->heading,
                     &wp->eta,
                     &wp->is_final) == 7;
    }
    else if (strcmp(type_str, "WEATHER") == 0) {
        msg->type = SAT_MSG_WEATHER;
        WeatherInfo* w = &msg->data.weather;
        return sscanf(data, "%lf,%lf,%lf,%lf",
                     &w->wind_speed,
                     &w->wind_direction,
                     &w->turbulence,
                     &w->temperature) == 4;
    }
    else if (strcmp(type_str, "EMERGENCY") == 0) {
        msg->type = SAT_MSG_EMERGENCY;
        int cmd;
        if (sscanf(data, "%d", &cmd) == 1) {
            msg->data.emergency = (EmergencyCommand)cmd;
            return true;
        }
    }

    return false;
}

void sat_com_process(SatCom* sat) {
    if (!sat) return;

    // Try to connect if not connected
    if (!sat->connected) {
        try_connect(sat);
        return;
    }

    // Process messages from ground station
    char buffer[BUFFER_SIZE];
    ssize_t bytes_read = recv(sat->socket_fd, buffer, BUFFER_SIZE - 1, 0);

    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        
        SatelliteMessage msg;
        if (parse_sat_message(buffer, &msg)) {
            sat->last_message = msg;

            // Handle different message types
            switch (msg.type) {
                case SAT_MSG_WAYPOINT:
                    handle_waypoint(sat, &msg.data.waypoint);
                    break;

                case SAT_MSG_WEATHER:
                    handle_weather(sat, &msg.data.weather);
                    break;

                case SAT_MSG_EMERGENCY:
                    handle_emergency(sat, msg.data.emergency);
                    break;

                case SAT_MSG_STATUS_REQ:
                    // Send current state back to ground station
                    // Implementation would go here
                    break;

                default:
                    break;
            }
        }
    } else if (bytes_read == 0 || (bytes_read < 0 && errno != EWOULDBLOCK)) {
        // Connection closed or error
        fprintf(stderr, "SATCOM: Connection lost, attempting to reconnect...\n");
        close(sat->socket_fd);
        init_socket(sat);
        sat->connected = false;
    }

    // Process messages from flight controller
    Message msg;
    while (bus_read_message(sat->bus, COMPONENT_SAT_COM, &msg)) {
        if (msg.header.type == MSG_STATE_RESPONSE) {
            memcpy(&sat->current_state, &msg.payload, sizeof(FlightState));
        }
    }
}

void sat_com_main(Bus* bus) {
    SatCom* sat = sat_com_init(bus);
    if (!sat) {
        fprintf(stderr, "Failed to initialize SATCOM\n");
        return;
    }

    fprintf(stderr, "SATCOM started\n");

    while (1) {
        sat_com_process(sat);
        usleep(SATCOM_UPDATE_INTERVAL_MS * 1000);
    }

    sat_com_cleanup(sat);
}
