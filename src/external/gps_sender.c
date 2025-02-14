#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <stdbool.h>
#include <math.h>

#define GPS_PORT 5555
#define UPDATE_INTERVAL_MS 1000  // 1 Hz update rate
#define MAX_CLIENTS 5

static volatile bool running = true;

// Flight path definition
typedef struct {
    double current_lat;    // Current latitude
    double current_lon;    // Current longitude
    double current_alt;    // Current altitude in feet
    double heading;        // Current heading in degrees
    double ground_speed;   // Speed in knots
    double climb_rate;     // Vertical speed in feet per minute
    double target_alt;     // Target altitude in feet
} FlightPath;

static FlightPath flight_path = {
    .current_lat = 37.6188,    // SFO airport
    .current_lon = -122.3750,
    .current_alt = 0.0,
    .heading = 45.0,          // Northeast heading
    .ground_speed = 250.0,    // 250 knots
    .climb_rate = 1500.0,     // 1500 feet per minute climb
    .target_alt = 10000.0     // Target altitude 10,000 feet
};

void handle_signal(int sig) {
    running = false;
}

// Update simulated position based on heading, speed, and time
void update_position(FlightPath* path, double dt) {
    // Convert speed from knots to degrees per second
    // At the equator, 1 degree is approximately 60 nautical miles
    double speed_deg = path->ground_speed / (60.0 * 60.0);  // Convert from knots to degrees/second
    speed_deg /= cos(path->current_lat * M_PI / 180.0);     // Adjust for latitude

    // Calculate position changes
    double heading_rad = path->heading * M_PI / 180.0;
    path->current_lat += speed_deg * dt * cos(heading_rad);
    path->current_lon += speed_deg * dt * sin(heading_rad);

    // Update altitude
    if (path->current_alt < path->target_alt) {
        path->current_alt += (path->climb_rate / 60.0) * dt;  // Convert from feet/min to feet/sec
        if (path->current_alt > path->target_alt) {
            path->current_alt = path->target_alt;
        }
    }

    // Add some random variation to make it more realistic
    path->current_lat += (rand() % 100 - 50) * 0.000001;  // Small random variations
    path->current_lon += (rand() % 100 - 50) * 0.000001;
    path->current_alt += (rand() % 10 - 5);               // +/- 5 feet variation
}

int main(void) {
    int server_fd;
    struct sockaddr_in address;
    int client_sockets[MAX_CLIENTS] = {0};
    
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    srand(time(NULL));  // Initialize random number generator

    // Create socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        perror("Socket creation failed");
        return 1;
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt failed");
        return 1;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(GPS_PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        return 1;
    }

    if (listen(server_fd, MAX_CLIENTS) < 0) {
        perror("Listen failed");
        return 1;
    }

    printf("GPS sender started on port %d\n", GPS_PORT);

    struct timespec last_update;
    clock_gettime(CLOCK_MONOTONIC, &last_update);

    while (running) {
        fd_set read_fds;
        struct timeval tv = {0, 1000};  // 1ms timeout
        int max_fd = server_fd;

        FD_ZERO(&read_fds);
        FD_SET(server_fd, &read_fds);

        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (client_sockets[i] > 0) {
                FD_SET(client_sockets[i], &read_fds);
                if (client_sockets[i] > max_fd) {
                    max_fd = client_sockets[i];
                }
            }
        }

        int activity = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
        if (activity < 0 && errno != EINTR) {
            perror("Select error");
            continue;
        }

        // Handle new connections
        if (FD_ISSET(server_fd, &read_fds)) {
            int new_socket = accept(server_fd, NULL, NULL);
            if (new_socket >= 0) {
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (client_sockets[i] == 0) {
                        client_sockets[i] = new_socket;
                        printf("New client connected\n");
                        break;
                    }
                }
            }
        }

        // Update and send position
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double dt = (now.tv_sec - last_update.tv_sec) + 
                   (now.tv_nsec - last_update.tv_nsec) / 1e9;

        if (dt >= UPDATE_INTERVAL_MS / 1000.0) {
            // Update simulated position
            update_position(&flight_path, dt);

            // Prepare GPS data
            char buffer[256];
            snprintf(buffer, sizeof(buffer), "%.6f,%.6f,%.1f\n",
                    flight_path.current_lat,
                    flight_path.current_lon,
                    flight_path.current_alt);

            // Send to all clients
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (client_sockets[i] > 0) {
                    if (send(client_sockets[i], buffer, strlen(buffer), 0) <= 0) {
                        printf("Client disconnected\n");
                        close(client_sockets[i]);
                        client_sockets[i] = 0;
                    }
                }
            }

            last_update = now;
        }

        usleep(1000);  // 1ms sleep to prevent busy waiting
    }

    // Cleanup
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] > 0) {
            close(client_sockets[i]);
        }
    }
    close(server_fd);

    printf("GPS sender stopped\n");
    return 0;
}
