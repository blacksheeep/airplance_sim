#ifndef BUS_H
#define BUS_H

#include "common.h"
#include "messages.h"
#include <stdbool.h>

// Maximum number of messages in the bus queue
#define MAX_BUS_MESSAGES 100
// Maximum number of subscribers
#define MAX_SUBSCRIBERS 10

typedef struct Bus Bus;

// Initialize the message bus
Bus* bus_init(void);

// Clean up the message bus
void bus_cleanup(Bus* bus);

// Subscribe to messages
ErrorCode bus_subscribe(Bus* bus, ComponentId subscriber, MessageType msg_type);

// Publish a message to the bus
ErrorCode bus_publish(Bus* bus, Message* message);

// Read next message for a component (non-blocking)
// Returns true if message was read, false if no message available
bool bus_read_message(Bus* bus, ComponentId subscriber, Message* message);

// Get shared memory ID for attaching in forked processes
int bus_get_shm_id(Bus* bus);

// Attach to existing bus in forked process
Bus* bus_attach(int shm_id);

// Detach from bus (for forked processes)
void bus_detach(Bus* bus);

#endif // BUS_H
