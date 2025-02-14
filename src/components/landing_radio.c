#include "landing_radio.h"
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
#include <math.h>

#define BUFFER_SIZE 1024
#define UPDATE_INTERVAL_MS 100  // 10 Hz update rate
#define CONNECT_RETRY_INTERVAL_MS 1000
#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) ((x) * PI / 180.0)

struct LandingRadio {
    Bus* bus;
    int socket_fd;
    struct sockaddr_in server_addr;
    bool connected;
    ILSData last_ils_data;
    time_t last_status_update;
};

// Initialize socket connection
static bool init_socket(LandingRadio* radio) {
    LOG_INFO(LOG_LANDING, "Initializing socket");
    
    radio->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (radio->socket_fd < 0) {
        LOG_ERROR(LOG_LANDING, "Failed to create socket: %s", strerror(errno));
        return false;
    }

    // Set socket to non-blocking
    int flags = fcntl(radio->socket_fd, F_GETFL, 0);
    fcntl(radio->socket_fd, F_SETFL, flags | O_NONBLOCK);

    // Setup server address
    memset(&radio->server_addr, 0, sizeof(radio->server_addr));
    radio->server_addr.sin_family = AF_INET;
    radio->server_addr.sin_port = htons(LANDING_RADIO_PORT);

    struct hostent* server = gethostbyname(LANDING_RADIO_HOST);
    if (server == NULL) {
        LOG_ERROR(LOG_LANDING, "Failed to resolve hostname");
        close(radio->socket_fd);
        return false;
    }
    memcpy(&radio->server_addr.sin_addr.s_addr, server->h_addr, server->h_length);

    LOG_INFO(LOG_LANDING, "Socket initialized");
    return true;
}

LandingRadio* landing_radio_init(Bus* bus) {
    LOG_INFO(LOG_LANDING, "Starting initialization");
    
    if (!bus) {
        LOG_ERROR(LOG_LANDING, "NULL bus in init");
        return NULL;
    }

    LandingRadio* radio = malloc(sizeof(LandingRadio));
    if (!radio) {
        LOG_ERROR(LOG_LANDING, "Failed to allocate memory");
        return NULL;
    }

    radio->bus = bus;
    radio->connected = false;
    radio->last_status_update = 0;
    memset(&radio->last_ils_data, 0, sizeof(ILSData));

    if (!init_socket(radio)) {
        LOG_ERROR(LOG_LANDING, "Socket initialization failed");
        free(radio);
        return NULL;
    }

    LOG_INFO(LOG_LANDING, "Initialization complete");
    return radio;
}

void landing_radio_cleanup(LandingRadio* radio) {
    if (!radio) return;
    
    LOG_INFO(LOG_LANDING, "Cleaning up");
    if (radio->socket_fd >= 0) {
        close(radio->socket_fd);
    }
    free(radio);
}

static void send_status_update(LandingRadio* radio, bool connected) {
    Message msg = {0};
    msg.header.type = MSG_SYSTEM_STATUS;
    msg.header.sender = COMPONENT_LANDING_RADIO;
    msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
    msg.header.timestamp = time(NULL);
    msg.header.message_size = sizeof(SystemStatusMsg);
    msg.payload.system_status.component_active = connected;
    
    if (bus_publish(radio->bus, &msg) != SUCCESS) {
        LOG_ERROR(LOG_LANDING, "Failed to publish status update");
    } else {
        LOG_INFO(LOG_LANDING, "Published status: %s", 
                connected ? "CONNECTED" : "DISCONNECTED");
    }
}

static void publish_position(LandingRadio* radio, const Position* pos) {
    Message msg = {0};
    msg.header.type = MSG_POSITION_UPDATE;
    msg.header.sender = COMPONENT_LANDING_RADIO;
    msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
    msg.header.timestamp = time(NULL);
    msg.header.message_size = sizeof(PositionUpdateMsg);
    msg.payload.position_update.position = *pos;

    if (bus_publish(radio->bus, &msg) == SUCCESS) {
        LOG_DEBUG(LOG_LANDING, "Published position: %.6f, %.6f, %.1f",
                pos->latitude, pos->longitude, pos->altitude);
    } else {
        LOG_ERROR(LOG_LANDING, "Failed to publish position");
    }
}

static bool try_connect(LandingRadio* radio) {
    if (radio->connected) return true;

    LOG_INFO(LOG_LANDING, "Attempting to connect to %s:%d", 
            LANDING_RADIO_HOST, LANDING_RADIO_PORT);
            
    int result = connect(radio->socket_fd, 
                        (struct sockaddr*)&radio->server_addr, 
                        sizeof(radio->server_addr));

    if (result == 0 || (result < 0 && errno == EISCONN)) {
        radio->connected = true;
        LOG_INFO(LOG_LANDING, "Connected to sender");
        send_status_update(radio, true);
        return true;
    }

    if (result < 0 && (errno != EINPROGRESS && errno != EALREADY && 
                       errno != EWOULDBLOCK)) {
        LOG_ERROR(LOG_LANDING, "Connect error: %s", strerror(errno));
        close(radio->socket_fd);
        init_socket(radio);
    }

    return false;
}

static bool parse_ils_data(const char* buffer, ILSData* ils) {
    // Expected format: "LOC,GS,DIST,LOC_VALID,GS_VALID,MARKER\n"
    int loc_valid, gs_valid, marker;
    int result = sscanf(buffer, "%lf,%lf,%lf,%d,%d,%d",
                       &ils->localizer,
                       &ils->glideslope,
                       &ils->distance,
                       &loc_valid,
                       &gs_valid,
                       &marker);
    
    if (result == 6) {
        ils->localizer_valid = loc_valid;
        ils->glideslope_valid = gs_valid;
        ils->marker_beacon = marker;
        
        LOG_DEBUG(LOG_LANDING, "Parsed ILS data - LOC: %.2f, GS: %.2f, DIST: %.1f",
                ils->localizer, ils->glideslope, ils->distance);
        return true;
    }
    
    LOG_ERROR(LOG_LANDING, "Failed to parse ILS data: %s", buffer);
    return false;
}

Position ils_deviations_to_position(const ILSData* ils, const Position* runway_threshold) {
    Position pos = *runway_threshold;

    if (!ils->localizer_valid || !ils->glideslope_valid) {
        LOG_WARN(LOG_LANDING, "Invalid ILS data - LOC valid: %d, GS valid: %d",
                ils->localizer_valid, ils->glideslope_valid);
        return pos;
    }

    // Convert nautical miles to meters for calculation
    double distance_m = ils->distance * 1852.0;

    // Calculate position based on localizer deviation
    double runway_heading_rad = DEG_TO_RAD(RUNWAY_HEADING);
    double localizer_rad = DEG_TO_RAD(ils->localizer);
    double total_angle_rad = runway_heading_rad + localizer_rad;

    // Calculate lateral and longitudinal offsets
    double x = distance_m * cos(total_angle_rad);
    double y = distance_m * sin(total_angle_rad);

    // Convert to latitude/longitude changes
    double lat_change = y / 111111.0;  // 1 degree latitude = ~111111 meters
    double lon_change = x / (111111.0 * cos(DEG_TO_RAD(runway_threshold->latitude)));

    pos.latitude += lat_change;
    pos.longitude += lon_change;

    // Calculate altitude based on glideslope
    double glideslope_rad = DEG_TO_RAD(GLIDE_SLOPE_ANGLE);
    double nominal_altitude = runway_threshold->altitude +
                            distance_m * tan(glideslope_rad);
    double glideslope_deviation_rad = DEG_TO_RAD(ils->glideslope);
    pos.altitude = nominal_altitude +
                  distance_m * tan(glideslope_deviation_rad);

    LOG_TRACE(LOG_LANDING, "Calculated position from ILS - Lat: %.6f, Lon: %.6f, Alt: %.1f",
             pos.latitude, pos.longitude, pos.altitude);

    return pos;
}

void landing_radio_process(LandingRadio* radio) {
    if (!radio) {
        LOG_ERROR(LOG_LANDING, "NULL radio in process");
        return;
    }

    // Send periodic status updates
    time_t now = time(NULL);
    if (now - radio->last_status_update >= 1) {
        send_status_update(radio, radio->connected);
        radio->last_status_update = now;
    }

    // Try to connect if not connected
    if (!radio->connected) {
        if (!try_connect(radio)) {
            usleep(CONNECT_RETRY_INTERVAL_MS * 1000);
            return;
        }
    }

    // Read data from socket
    char buffer[BUFFER_SIZE];
    ssize_t bytes_read = recv(radio->socket_fd, buffer, sizeof(buffer) - 1, 0);

    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        
        if (parse_ils_data(buffer, &radio->last_ils_data)) {
            // Convert ILS data to position update
            Position pos = ils_deviations_to_position(&radio->last_ils_data, 
                                                    &RUNWAY_THRESHOLD);
            publish_position(radio, &pos);
        }
    } else if (bytes_read == 0 || (bytes_read < 0 && errno != EWOULDBLOCK)) {
        // Connection closed or error
        LOG_ERROR(LOG_LANDING, "Connection lost: %s", 
                bytes_read == 0 ? "Closed by peer" : strerror(errno));
        close(radio->socket_fd);
        if (radio->connected) {
            radio->connected = false;
            send_status_update(radio, false);
        }
        init_socket(radio);
    }
}

void landing_radio_main(Bus* bus) {
    LOG_INFO(LOG_LANDING, "Starting main function");
    
    LandingRadio* radio = landing_radio_init(bus);
    if (!radio) {
        LOG_ERROR(LOG_LANDING, "Failed to initialize");
        return;
    }

    LOG_INFO(LOG_LANDING, "Entering main loop");
    
    while (1) {
        landing_radio_process(radio);
        usleep(UPDATE_INTERVAL_MS * 1000);
    }

    landing_radio_cleanup(radio);
}
