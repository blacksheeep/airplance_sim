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

#define LANDING_RADIO_PORT 5556
#define MAX_CLIENTS 5
#define BIND_RETRY_ATTEMPTS 5
#define BIND_RETRY_DELAY_MS 1000

static volatile bool running = true;

void handle_signal(int sig) {
    (void)sig;  // Suppress unused parameter warning
    running = false;
}

// Initialize server socket with retry logic
int initialize_server(void) {
    int server_fd;
    struct sockaddr_in address;
    int attempt = 0;

    while (attempt < BIND_RETRY_ATTEMPTS) {
        // Create socket
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == 0) {
            perror("Socket creation failed");
            return -1;
        }

        // Set socket options
        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                      &opt, sizeof(opt))) {
            perror("setsockopt failed");
            close(server_fd);
            return -1;
        }

        // Setup address structure
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(LANDING_RADIO_PORT);

        // Try to bind
        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            fprintf(stderr, "Bind attempt %d failed: %s\n", 
                    attempt + 1, strerror(errno));
            close(server_fd);
            
            if (attempt < BIND_RETRY_ATTEMPTS - 1) {
                usleep(BIND_RETRY_DELAY_MS * 1000);
                attempt++;
                continue;
            }
            return -1;
        }

        // Successfully bound
        return server_fd;
    }

    return -1;
}

int main(void) {
    int server_fd;
    int client_sockets[MAX_CLIENTS] = {0};
    
    // Setup signal handler
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Initialize server with retry logic
    server_fd = initialize_server();
    if (server_fd < 0) {
        fprintf(stderr, "Failed to initialize server after %d attempts\n", 
                BIND_RETRY_ATTEMPTS);
        return 1;
    }

    // Listen for connections
    if (listen(server_fd, MAX_CLIENTS) < 0) {
        perror("Listen failed");
        return 1;
    }

    printf("Landing radio sender started on port %d\n", LANDING_RADIO_PORT);

    // Rest of the landing radio sender code remains the same...
    // (ILS simulation logic)

    return 0;
}
