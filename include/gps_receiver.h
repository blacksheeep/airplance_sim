#ifndef GPS_RECEIVER_H
#define GPS_RECEIVER_H

#include "common.h"
#include "bus.h"

// GPS connection settings
#define GPS_PORT 5555
#define GPS_HOST "localhost"

typedef struct GpsReceiver GpsReceiver;

// Initialize GPS receiver
GpsReceiver* gps_receiver_init(Bus* bus);

// Clean up GPS receiver
void gps_receiver_cleanup(GpsReceiver* gps);

// Process one iteration of GPS data
void gps_receiver_process(GpsReceiver* gps);

// Main entry point for GPS receiver process
void gps_receiver_main(Bus* bus);

#endif // GPS_RECEIVER_H
