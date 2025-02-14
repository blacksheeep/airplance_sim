#include "sat_com.h"
#include "log.h"
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
#define SATCOM_UPDATE_INTERVAL_MS 1000
#define STATUS_UPDATE_INTERVAL_S 1


struct SatCom {
    Bus* bus;
    int socket_fd;
    struct sockaddr_in server_addr;
    bool connected;
    SatelliteMessage last_message;
    FlightState current_state;
};

// Forward declarations of static functions
static bool init_socket(SatCom* sat);
static void send_status_update(SatCom* sat, bool connected);
static bool try_connect(SatCom* sat);
static bool parse_sat_message(const char* buffer, SatelliteMessage* msg);

// Initialize socket connection
static bool init_socket(SatCom* sat) {
    LOG_INFO(LOG_SATCOM, "Initializing socket");
    
    sat->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sat->socket_fd < 0) {
        LOG_ERROR(LOG_SATCOM, "Failed to create socket: %s", strerror(errno));
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
        LOG_ERROR(LOG_SATCOM, "Failed to resolve hostname");
        return false;
    }
    
    memcpy(&sat->server_addr.sin_addr.s_addr, server->h_addr, server->h_length);
    return true;
}

static void send_status_update(SatCom* sat, bool connected) {
    Message msg = {0};
    msg.header.type = MSG_SYSTEM_STATUS;
    msg.header.sender = COMPONENT_SAT_COM;
    msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
    msg.header.timestamp = time(NULL);
    msg.header.message_size = sizeof(SystemStatusMsg);
    msg.payload.system_status.component_active = connected;
    
    if (bus_publish(sat->bus, &msg) != SUCCESS) {
        LOG_ERROR(LOG_SATCOM, "Failed to publish status update");
    } else {
        LOG_INFO(LOG_SATCOM, "Published status: %s", connected ? "CONNECTED" : "DISCONNECTED");
    }
}

static bool try_connect(SatCom* sat) {
    if (sat->connected) return true;

    LOG_INFO(LOG_SATCOM, "Attempting to connect to %s:%d", SATCOM_HOST, SATCOM_PORT);
    
    int result = connect(sat->socket_fd, 
                        (struct sockaddr*)&sat->server_addr, 
                        sizeof(sat->server_addr));

    if (result == 0 || (result < 0 && errno == EISCONN)) {
        sat->connected = true;
        LOG_INFO(LOG_SATCOM, "Connected to ground station");
        send_status_update(sat, true);
        return true;
    }
    
    if (result < 0 && (errno != EINPROGRESS && errno != EALREADY && 
                       errno != EWOULDBLOCK)) {
        LOG_ERROR(LOG_SATCOM, "Connect error: %s", strerror(errno));
        close(sat->socket_fd);
        init_socket(sat);
    }

    return false;
}

// Parse satellite message from sender
static bool parse_sat_message(const char* buffer, SatelliteMessage* msg) {
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
        int is_final_int = 0;  // Use int for scanf
        int result = sscanf(data, "%lf,%lf,%lf,%lf,%lf,%u,%d",
                     &wp->position.latitude,
                     &wp->position.longitude,
                     &wp->position.altitude,
                     &wp->speed,
                     &wp->heading,
                     &wp->eta,
                     &is_final_int);
        wp->is_final = (bool)is_final_int;  // Convert to bool after
        return result == 7;
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

SatCom* sat_com_init(Bus* bus) {
    LOG_INFO(LOG_SATCOM, "Starting initialization");
    
    if (!bus) {
        LOG_ERROR(LOG_SATCOM, "NULL bus in init");
        return NULL;
    }

    SatCom* sat = malloc(sizeof(SatCom));
    if (!sat) {
        LOG_ERROR(LOG_SATCOM, "Failed to allocate memory");
        return NULL;
    }

    sat->bus = bus;
    sat->connected = false;
    memset(&sat->last_message, 0, sizeof(SatelliteMessage));
    memset(&sat->current_state, 0, sizeof(FlightState));

    if (!init_socket(sat)) {
        LOG_ERROR(LOG_SATCOM, "Socket initialization failed");
        free(sat);
        return NULL;
    }

    // Subscribe to state updates from flight controller
    ErrorCode err = bus_subscribe(bus, COMPONENT_SAT_COM, MSG_STATE_RESPONSE);
    if (err != SUCCESS) {
        LOG_ERROR(LOG_SATCOM, "Failed to subscribe to state responses: %d", err);
        free(sat);
        return NULL;
    }

    // Send initial status update
    send_status_update(sat, false);

    LOG_INFO(LOG_SATCOM, "Initialization complete");
    return sat;
}

void sat_com_process(SatCom* sat) {
    if (!sat) return;

    static time_t last_status_update = 0;
    time_t now = time(NULL);

    // Send periodic status updates
    if (now - last_status_update >= STATUS_UPDATE_INTERVAL_S) {
        send_status_update(sat, sat->connected);
        last_status_update = now;
    }

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
            // Process the message based on type...
        }
    } else if (bytes_read == 0 || (bytes_read < 0 && errno != EWOULDBLOCK)) {
        // Connection closed or error
        LOG_ERROR(LOG_SATCOM, "Connection lost: %s", 
                bytes_read == 0 ? "Closed by peer" : strerror(errno));
        close(sat->socket_fd);
        if (sat->connected) {
            sat->connected = false;
            send_status_update(sat, false);
        }
        init_socket(sat);
    }
}

void sat_com_main(Bus* bus) {
    LOG_INFO(LOG_SATCOM, "Starting main function");

    SatCom* sat = sat_com_init(bus);
    if (!sat) {
        LOG_ERROR(LOG_SATCOM, "Failed to initialize");
        return;
    }

    LOG_INFO(LOG_SATCOM, "Entering main loop");

    while (1) {
        sat_com_process(sat);
        usleep(SATCOM_UPDATE_INTERVAL_MS * 1000);
    }

    sat_com_cleanup(sat);
}
