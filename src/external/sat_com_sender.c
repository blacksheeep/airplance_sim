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

#define SATCOM_PORT 5557
#define MAX_CLIENTS 5
#define BUFFER_SIZE 1024
#define UPDATE_INTERVAL_MS 1000  // 1 Hz update rate

static volatile bool running = true;

// Flight plan waypoint
typedef struct {
    double latitude;
    double longitude;
    double altitude;
    double speed;
    double heading;
    bool is_final;
} FlightPlanWaypoint;

// Predefined flight plan
static FlightPlanWaypoint flight_plan[] = {
    {37.7749, -122.4194, 5000.0, 250.0, 90.0, false},  // San Francisco
    {37.3688, -121.9314, 4000.0, 200.0, 120.0, false}, // San Jose
    {37.5483, -121.9886, 3000.0, 180.0, 150.0, true}   // Fremont
};
static int current_waypoint = 0;
static const int num_waypoints = sizeof(flight_plan) / sizeof(flight_plan[0]);

// Weather simulation parameters
typedef struct {
    double wind_speed;
    double wind_direction;
    double turbulence;
    double temperature;
    time_t last_update;
} WeatherState;

static WeatherState weather = {
    .wind_speed = 10.0,
    .wind_direction = 270.0,
    .turbulence = 2.0,
    .temperature = 15.0,
    .last_update = 0
};

void handle_signal(int sig) {
    running = false;
}

// Update simulated weather conditions
void update_weather(void) {
    time_t now = time(NULL);
    if (now - weather.last_update < 300) {  // Update every 5 minutes
        return;
    }

    // Add some random variations
    weather.wind_speed += (rand() % 100 - 50) / 10.0;
    if (weather.wind_speed < 0) weather.wind_speed = 0;
    if (weather.wind_speed > 50) weather.wind_speed = 50;

    weather.wind_direction += (rand() % 40 - 20);
    if (weather.wind_direction < 0) weather.wind_direction += 360;
    if (weather.wind_direction >= 360) weather.wind_direction -= 360;

    weather.turbulence += (rand() % 100 - 50) / 50.0;
    if (weather.turbulence < 0) weather.turbulence = 0;
    if (weather.turbulence > 10) weather.turbulence = 10;

    weather.temperature += (rand() % 100 - 50) / 50.0;
    weather.last_update = now;
}

// Send weather update to client
void send_weather_update(int client_socket) {
    char buffer[BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "WEATHER,%.1f,%.1f,%.1f,%.1f\n",
             weather.wind_speed,
             weather.wind_direction,
             weather.turbulence,
             weather.temperature);

    send(client_socket, buffer, strlen(buffer), 0);
}

// Send next waypoint to client
void send_waypoint(int client_socket) {
    if (current_waypoint >= num_waypoints) return;

    FlightPlanWaypoint* wp = &flight_plan[current_waypoint];
    time_t now = time(NULL);
    
    char buffer[BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "WAYPOINT,%.6f,%.6f,%.1f,%.1f,%.1f,%lu,%d\n",
             wp->latitude,
             wp->longitude,
             wp->altitude,
             wp->speed,
             wp->heading,
             (unsigned long)(now + 1800),  // ETA in 30 minutes
             wp->is_final);

    send(client_socket, buffer, strlen(buffer), 0);
}

// Simulate emergency conditions
void check_emergency_conditions(int client_socket) {
    // Simulate random emergency situations
    if (rand() % 1000 == 0) {  // 0.1% chance of emergency
        char buffer[BUFFER_SIZE];
        int emergency_type = rand() % 4 + 1;  // 1-4 emergency types
        
        snprintf(buffer, sizeof(buffer), "EMERGENCY,%d\n", emergency_type);
        send(client_socket, buffer, strlen(buffer), 0);
        
        fprintf(stderr, "Emergency condition %d sent\n", emergency_type);
    }
}

int main(int argc, char* argv[]) {
    int server_fd;
    struct sockaddr_in address;
    int client_sockets[MAX_CLIENTS] = {0};
    
    // Setup signal handler
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Initialize random number generator
    srand(time(NULL));

    // Create socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        perror("Socket creation failed");
        return 1;
    }

    // Set socket options
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt failed");
        return 1;
    }

    // Setup address structure
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(SATCOM_PORT);

    // Bind socket
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        return 1;
    }

    // Listen for connections
    if (listen(server_fd, MAX_CLIENTS) < 0) {
        perror("Listen failed");
        return 1;
    }

    printf("Ground station started on port %d\n", SATCOM_PORT);

    struct timespec last_update;
    clock_gettime(CLOCK_MONOTONIC, &last_update);

    // Main loop
    while (running) {
        fd_set read_fds;
        struct timeval tv = {0, 1000};  // 1ms timeout
        int max_fd = server_fd;

        FD_ZERO(&read_fds);
        FD_SET(server_fd, &read_fds);

        // Add client sockets to set
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (client_sockets[i] > 0) {
                FD_SET(client_sockets[i], &read_fds);
                if (client_sockets[i] > max_fd) {
                    max_fd = client_sockets[i];
                }
            }
        }

        // Wait for activity on sockets
        int activity = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
        if (activity < 0 && errno != EINTR) {
            perror("Select error");
            continue;
        }

        // Handle new connections
        if (FD_ISSET(server_fd, &read_fds)) {
            int new_socket = accept(server_fd, NULL, NULL);
            if (new_socket >= 0) {
                // Add to client sockets array
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (client_sockets[i] == 0) {
                        client_sockets[i] = new_socket;
                        printf("New aircraft connected\n");
                        
                        // Send initial waypoint
                        send_waypoint(new_socket);
                        break;
                    }
                }
            }
        }

        // Update and send data to clients
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double dt = (now.tv_sec - last_update.tv_sec) + 
                   (now.tv_nsec - last_update.tv_nsec) / 1e9;

        if (dt >= UPDATE_INTERVAL_MS / 1000.0) {
            // Update weather
            update_weather();

            // Process each client
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (client_sockets[i] > 0) {
                    // Send weather updates
                    send_weather_update(client_sockets[i]);

                    // Check for emergency conditions
                    check_emergency_conditions(client_sockets[i]);

                    // Check for client messages (like waypoint reached)
                    char buffer[BUFFER_SIZE];
                    ssize_t bytes_read = recv(client_sockets[i], buffer, 
                                            BUFFER_SIZE - 1, MSG_DONTWAIT);
                    
                    if (bytes_read > 0) {
                        buffer[bytes_read] = '\0';
                        if (strstr(buffer, "WAYPOINT_REACHED") != NULL) {
                            current_waypoint++;
                            if (current_waypoint < num_waypoints) {
                                send_waypoint(client_sockets[i]);
                            }
                        }
                    } else if (bytes_read == 0 || 
                              (bytes_read < 0 && errno != EWOULDBLOCK)) {
                        // Client disconnected
                        printf("Aircraft disconnected\n");
                        close(client_sockets[i]);
                        client_sockets[i] = 0;
                    }
                }
            }

            last_update = now;
        }

        usleep(1000);  // Small sleep to prevent busy waiting
    }

    // Cleanup
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] > 0) {
            close(client_sockets[i]);
        }
    }
    close(server_fd);

    printf("Ground station stopped\n");
    return 0;
}
