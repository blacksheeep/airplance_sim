#include "bus.h"
#include "log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

// Message timeout in seconds
#define MESSAGE_TIMEOUT_S 5

// Subscription entry
typedef struct {
    ComponentId subscriber;
    MessageType msg_type;
    bool active;
} Subscription;

// Circular message queue
typedef struct {
    Message messages[MAX_BUS_MESSAGES];
    time_t timestamps[MAX_BUS_MESSAGES];
    int read_idx;
    int write_idx;
    int count;
} MessageQueue;

// Bus structure (will be in shared memory)
struct Bus {
    MessageQueue queue;
    Subscription subscriptions[MAX_SUBSCRIBERS];
    sem_t* mutex;
    int ref_count;
    int shm_id;
};

// Create or get named semaphore
static sem_t* create_mutex(void) {
    sem_t* mutex = sem_open("/airplane_sim_bus", O_CREAT, 0644, 1);
    if (mutex == SEM_FAILED) {
        fprintf(stderr, "Bus: sem_open failed: %s\n", strerror(errno));
        return NULL;
    }
    fprintf(stderr, "Bus: Created/opened semaphore\n");
    return mutex;
}

Bus* bus_init(void) {
    fprintf(stderr, "Bus: Initializing...\n");
    
    // Create shared memory
    int shm_id = shmget(IPC_PRIVATE, sizeof(Bus), IPC_CREAT | 0666);
    if (shm_id == -1) {
        fprintf(stderr, "Bus: shmget failed: %s\n", strerror(errno));
        return NULL;
    }
    fprintf(stderr, "Bus: Created shared memory with ID: %d\n", shm_id);

    // Attach to shared memory
    Bus* bus = (Bus*)shmat(shm_id, NULL, 0);
    if (bus == (void*)-1) {
        fprintf(stderr, "Bus: shmat failed: %s\n", strerror(errno));
        return NULL;
    }

    // Initialize bus structure
    memset(bus, 0, sizeof(Bus));
    bus->mutex = create_mutex();
    if (!bus->mutex) {
        fprintf(stderr, "Bus: Failed to create mutex\n");
        shmdt(bus);
        return NULL;
    }
    
    bus->ref_count = 1;
    bus->shm_id = shm_id;
    
    fprintf(stderr, "Bus: Initialization complete\n");
    return bus;
}

void bus_cleanup(Bus* bus) {
    if (!bus) {
        fprintf(stderr, "Bus: NULL bus in cleanup\n");
        return;
    }

    fprintf(stderr, "Bus: Starting cleanup (ref_count: %d)\n", bus->ref_count);
    
    sem_wait(bus->mutex);
    bus->ref_count--;
    fprintf(stderr, "Bus: Decreased ref_count to %d\n", bus->ref_count);
    
    if (bus->ref_count == 0) {
        fprintf(stderr, "Bus: Last reference, cleaning up resources\n");
        sem_post(bus->mutex);
        sem_close(bus->mutex);
        sem_unlink("/airplane_sim_bus");
        
        int shm_id = bus->shm_id;
        shmdt(bus);
        if (shmctl(shm_id, IPC_RMID, NULL) == -1) {
            fprintf(stderr, "Bus: Failed to remove shared memory: %s\n", strerror(errno));
        }
    } else {
        sem_post(bus->mutex);
        shmdt(bus);
    }
}

static void prune_old_messages(MessageQueue* queue) {
    if (queue->count == 0) return;
    
    time_t now = time(NULL);
    int pruned = 0;
    int read_idx = queue->read_idx;
    
    while (queue->count > 0) {
        if (now - queue->timestamps[read_idx] > MESSAGE_TIMEOUT_S) {
            read_idx = (read_idx + 1) % MAX_BUS_MESSAGES;
            queue->count--;
            pruned++;
        } else {
            break;
        }
    }
    
    if (pruned > 0) {
        queue->read_idx = read_idx;
        fprintf(stderr, "Bus: Pruned %d old messages\n", pruned);
    }
}

ErrorCode bus_subscribe(Bus* bus, ComponentId subscriber, MessageType msg_type) {
    if (!bus) {
        fprintf(stderr, "Bus: NULL bus in subscribe\n");
        return ERROR_GENERAL;
    }

    fprintf(stderr, "Bus: Component %d subscribing to message type %d\n", 
            subscriber, msg_type);

    sem_wait(bus->mutex);

    // Find free subscription slot
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (!bus->subscriptions[i].active) {
            bus->subscriptions[i].subscriber = subscriber;
            bus->subscriptions[i].msg_type = msg_type;
            bus->subscriptions[i].active = true;
            sem_post(bus->mutex);
            fprintf(stderr, "Bus: Subscription added at slot %d\n", i);
            return SUCCESS;
        }
    }

    sem_post(bus->mutex);
    fprintf(stderr, "Bus: No free subscription slots\n");
    return ERROR_GENERAL;
}

ErrorCode bus_publish(Bus* bus, Message* message) {
    if (!bus || !message) {
        LOG_ERROR(LOG_BUS, "NULL parameter in publish");
        return ERROR_GENERAL;
    }

    sem_wait(bus->mutex);

    if (bus->queue.count >= MAX_BUS_MESSAGES) {
        LOG_WARN(LOG_BUS, "Message queue full (count: %d)", bus->queue.count);
        sem_post(bus->mutex);
        return ERROR_COMMUNICATION;
    }

    // Add message to queue
    memcpy(&bus->queue.messages[bus->queue.write_idx], message, sizeof(Message));
    bus->queue.write_idx = (bus->queue.write_idx + 1) % MAX_BUS_MESSAGES;
    bus->queue.count++;

    LOG_DEBUG(LOG_BUS, "Published message type %d from %d to %d (count: %d)",
            message->header.type, message->header.sender, 
            message->header.receiver, bus->queue.count);

    sem_post(bus->mutex);
    return SUCCESS;
}

bool bus_read_message(Bus* bus, ComponentId subscriber, Message* message) {
    if (!bus || !message) {
        fprintf(stderr, "Bus: NULL parameter in read_message\n");
        return false;
    }

    sem_wait(bus->mutex);

    // Try to prune old messages first
    if (bus->queue.count > MAX_BUS_MESSAGES / 2) {
        prune_old_messages(&bus->queue);
    }

    if (bus->queue.count == 0) {
        sem_post(bus->mutex);
        return false;
    }

    // Check if message is for this subscriber
    int current = bus->queue.read_idx;
    int checked = 0;
    bool found = false;

    while (checked < bus->queue.count && !found) {
        Message* current_msg = &bus->queue.messages[current];
        
        // Check subscriptions
        for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
            if (bus->subscriptions[i].active &&
                bus->subscriptions[i].subscriber == subscriber &&
                bus->subscriptions[i].msg_type == current_msg->header.type) {
                memcpy(message, current_msg, sizeof(Message));
                found = true;
                break;
            }
        }

        if (!found) {
            current = (current + 1) % MAX_BUS_MESSAGES;
            checked++;
        }
    }

    if (found) {
        // Remove message from queue
        bus->queue.read_idx = (current + 1) % MAX_BUS_MESSAGES;
        bus->queue.count--;
    }

    sem_post(bus->mutex);
    return found;
}

int bus_get_shm_id(Bus* bus) {
    if (!bus) {
        fprintf(stderr, "Bus: NULL bus in get_shm_id\n");
        return -1;
    }
    return bus->shm_id;
}

Bus* bus_attach(int shm_id) {
    fprintf(stderr, "Bus: Attaching to shared memory ID: %d\n", shm_id);
    
    Bus* bus = (Bus*)shmat(shm_id, NULL, 0);
    if (bus == (void*)-1) {
        fprintf(stderr, "Bus: Failed to attach: %s\n", strerror(errno));
        return NULL;
    }
    
    sem_wait(bus->mutex);
    bus->ref_count++;
    fprintf(stderr, "Bus: Attached successfully, ref_count now %d\n", bus->ref_count);
    sem_post(bus->mutex);
    
    return bus;
}

void bus_detach(Bus* bus) {
    if (!bus) return;
    fprintf(stderr, "Bus: Detaching\n");
    bus_cleanup(bus);
}
