#include "gps_receiver.h"
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
#define CONNECT_RETRY_INTERVAL_MS 1000
#define STATUS_UPDATE_INTERVAL_S 1

struct GpsReceiver {
    Bus* bus;
    int socket_fd;
    struct sockaddr_in server_addr;
    bool connected;
    Position last_position;
    time_t last_status_update;
};

// Initialize socket connection
static bool init_socket(GpsReceiver* gps) {
    LOG_INFO(LOG_GPS, "Initializing socket");
    
    gps->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (gps->socket_fd < 0) {
        LOG_ERROR(LOG_GPS, "Failed to create socket: %s", strerror(errno));
        return false;
    }

    // Set socket to non-blocking
    int flags = fcntl(gps->socket_fd, F_GETFL, 0);
    fcntl(gps->socket_fd, F_SETFL, flags | O_NONBLOCK);

    // Setup server address
    memset(&gps->server_addr, 0, sizeof(gps->server_addr));
    gps->server_addr.sin_family = AF_INET;
    gps->server_addr.sin_port = htons(GPS_PORT);

    struct hostent* server = gethostbyname(GPS_HOST);
    if (server == NULL) {
        LOG_ERROR(LOG_GPS, "Failed to resolve hostname %s", GPS_HOST);
        close(gps->socket_fd);
        return false;
    }

    memcpy(&gps->server_addr.sin_addr.s_addr, server->h_addr, server->h_length);
    LOG_INFO(LOG_GPS, "Socket initialized successfully");
    return true;
}

GpsReceiver* gps_receiver_init(Bus* bus) {
    LOG_INFO(LOG_GPS, "Starting initialization");
    
    if (!bus) {
        LOG_ERROR(LOG_GPS, "NULL bus in init");
        return NULL;
    }

    GpsReceiver* gps = malloc(sizeof(GpsReceiver));
    if (!gps) {
        LOG_ERROR(LOG_GPS, "Failed to allocate memory");
        return NULL;
    }

    gps->bus = bus;
    gps->connected = false;
    gps->last_status_update = 0;
    memset(&gps->last_position, 0, sizeof(Position));

    if (!init_socket(gps)) {
        LOG_ERROR(LOG_GPS, "Socket initialization failed");
        free(gps);
        return NULL;
    }

    LOG_INFO(LOG_GPS, "Initialization complete");
    return gps;
}

void gps_receiver_cleanup(GpsReceiver* gps) {
    if (!gps) return;
    
    LOG_INFO(LOG_GPS, "Cleaning up");
    if (gps->socket_fd >= 0) {
        close(gps->socket_fd);
    }
    free(gps);
}

static void send_status_update(GpsReceiver* gps, bool connected) {
    Message msg = {0};
    msg.header.type = MSG_SYSTEM_STATUS;
    msg.header.sender = COMPONENT_GPS;
    msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
    msg.header.timestamp = time(NULL);
    msg.header.message_size = sizeof(SystemStatusMsg);
    msg.payload.system_status.component_active = connected;
    
    if (bus_publish(gps->bus, &msg) != SUCCESS) {
        LOG_ERROR(LOG_GPS, "Failed to publish status update");
    } else {
        LOG_INFO(LOG_GPS, "Published status: %s", 
                connected ? "CONNECTED" : "DISCONNECTED");
    }
}

static void publish_position(GpsReceiver* gps, const Position* pos) {
    Message msg = {0};
    msg.header.type = MSG_POSITION_UPDATE;
    msg.header.sender = COMPONENT_GPS;
    msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
    msg.header.timestamp = time(NULL);
    msg.header.message_size = sizeof(PositionUpdateMsg);
    msg.payload.position_update.position = *pos;
    
    if (bus_publish(gps->bus, &msg) == SUCCESS) {
        gps->last_position = *pos;
        LOG_DEBUG(LOG_GPS, "Published position: %.6f, %.6f, %.1f",
                pos->latitude, pos->longitude, pos->altitude);
    } else {
        LOG_ERROR(LOG_GPS, "Failed to publish position");
    }
}

static bool try_connect(GpsReceiver* gps) {
    if (gps->connected) return true;

    LOG_INFO(LOG_GPS, "Attempting to connect to %s:%d", GPS_HOST, GPS_PORT);
    
    int result = connect(gps->socket_fd, 
                        (struct sockaddr*)&gps->server_addr, 
                        sizeof(gps->server_addr));

    if (result == 0 || (result < 0 && errno == EISCONN)) {
        gps->connected = true;
        LOG_INFO(LOG_GPS, "Connected to GPS sender");
        send_status_update(gps, true);
        return true;
    }
    
    if (result < 0 && (errno != EINPROGRESS && errno != EALREADY && 
                       errno != EWOULDBLOCK)) {
        LOG_ERROR(LOG_GPS, "Connect error: %s", strerror(errno));
        close(gps->socket_fd);
        init_socket(gps);
    }

    return false;
}

static bool validate_gps_data(const Position* current, const Position* new_pos) {
    static int frozen_count = 0;

    // Basic range validation
    if (new_pos->latitude < -90.0 || new_pos->latitude > 90.0 ||
        new_pos->longitude < -180.0 || new_pos->longitude > 180.0 ||
        new_pos->altitude < -1000.0) {
        LOG_WARN(LOG_GPS, "Invalid GPS coordinates: %.6f, %.6f, %.1f",
                new_pos->latitude, new_pos->longitude, new_pos->altitude);
        return false;
    }

    // Check for unreasonable position jumps if we have a previous position
    if (current && current->latitude != 0.0 && current->longitude != 0.0) {
        // Calculate maximum possible position change based on time and max aircraft speed
        // Allow for up to 600 knots = ~1100 km/h = ~0.3 km/s
        // At equator, 1 degree is approximately 111 km
        const double MAX_DEGREE_CHANGE_PER_SEC = 0.6 / 111.0;  // More permissive limit

        double lat_change = fabs(new_pos->latitude - current->latitude);
        double lon_change = fabs(new_pos->longitude - current->longitude);
        double alt_change = fabs(new_pos->altitude - current->altitude);

        if (lat_change > MAX_DEGREE_CHANGE_PER_SEC ||
            lon_change > MAX_DEGREE_CHANGE_PER_SEC ||
            alt_change > 2000.0) {  // More permissive altitude change limit
            LOG_WARN(LOG_GPS, "Large position change detected - delta lat: %.6f, delta lon: %.6f, delta alt: %.1f",
                    lat_change, lon_change, alt_change);
            // Don't reject large changes immediately, just log them
        }
    }

    // Check for frozen position
    if (current &&
        new_pos->latitude == current->latitude &&
        new_pos->longitude == current->longitude) {
        frozen_count++;

        if (frozen_count > 10) {  // More permissive frozen position threshold
            LOG_WARN(LOG_GPS, "Position appears frozen at %.6f, %.6f",
                    new_pos->latitude, new_pos->longitude);
            // Don't reject frozen positions immediately, just log them
        }
    } else {
        frozen_count = 0;
    }

    return true;  // Accept all positions that pass basic range validation
}

static bool parse_gps_data(GpsReceiver* gps, const char* buffer, Position* pos) {
    int result = sscanf(buffer, "%lf,%lf,%lf",
                       &pos->latitude,
                       &pos->longitude,
                       &pos->altitude);

    if (result != 3) {
        LOG_ERROR(LOG_GPS, "Failed to parse GPS data: %s", buffer);
        return false;
    }

    LOG_DEBUG(LOG_GPS, "Parsed position: %.6f, %.6f, %.1f",
            pos->latitude, pos->longitude, pos->altitude);

    // Validate the new position data
    if (!validate_gps_data(&gps->last_position, pos)) {
        return false;
    }

    return true;
}

void gps_receiver_process(GpsReceiver* gps) {
    if (!gps) {
        LOG_ERROR(LOG_GPS, "NULL GPS in process");
        return;
    }

    time_t now = time(NULL);

    // Send periodic status updates
    if (now - gps->last_status_update >= STATUS_UPDATE_INTERVAL_S) {
        send_status_update(gps, gps->connected);
        gps->last_status_update = now;
    }

    // Try to connect if not connected
    if (!gps->connected) {
        if (!try_connect(gps)) {
            usleep(CONNECT_RETRY_INTERVAL_MS * 1000);
            return;
        }
    }

    // Read data from socket
    char buffer[BUFFER_SIZE];
    ssize_t bytes_read = recv(gps->socket_fd, buffer, sizeof(buffer) - 1, 0);

    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        Position new_pos;
        if (parse_gps_data(gps, buffer, &new_pos)) {
            LOG_TRACE(LOG_GPS, "Position update - delta lat: %.6f, delta lon: %.6f, delta alt: %.1f",
                    new_pos.latitude - gps->last_position.latitude,
                    new_pos.longitude - gps->last_position.longitude,
                    new_pos.altitude - gps->last_position.altitude);
            publish_position(gps, &new_pos);
        } else {
            // If we get invalid data multiple times, consider reconnecting
            static int invalid_count = 0;
            invalid_count++;
            if (invalid_count > 10) {
                LOG_ERROR(LOG_GPS, "Too many invalid GPS readings, reconnecting...");
                close(gps->socket_fd);
                gps->connected = false;
                init_socket(gps);
                invalid_count = 0;
            }
        }
    } else if (bytes_read == 0 || (bytes_read < 0 && errno != EWOULDBLOCK)) {
        LOG_ERROR(LOG_GPS, "Connection lost: %s",
                bytes_read == 0 ? "Closed by peer" : strerror(errno));
        close(gps->socket_fd);
        if (gps->connected) {
            gps->connected = false;
            send_status_update(gps, false);
        }
        init_socket(gps);
    }
}

void gps_receiver_main(Bus* bus) {
    LOG_INFO(LOG_GPS, "Starting main function");
    
    GpsReceiver* gps = gps_receiver_init(bus);
    if (!gps) {
        LOG_ERROR(LOG_GPS, "Failed to initialize");
        return;
    }

    LOG_INFO(LOG_GPS, "Entering main loop");
    
    while (1) {
        gps_receiver_process(gps);
        usleep(10000);  // 10ms sleep to prevent busy waiting
    }

    gps_receiver_cleanup(gps);
}
