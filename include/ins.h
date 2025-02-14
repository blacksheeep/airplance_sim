#ifndef INS_H
#define INS_H

#include "common.h"
#include "bus.h"
#include <math.h>

// Math constants
#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / PI)

// Physical constants
#define GRAVITY 9.81
#define EARTH_RADIUS 6371000.0  // meters

// INS sensor data
typedef struct {
    // Accelerometer data (m/s^2)
    double accel_x;
    double accel_y;
    double accel_z;
    
    // Gyroscope data (radians/s)
    double gyro_x;  // Roll rate
    double gyro_y;  // Pitch rate
    double gyro_z;  // Yaw rate
    
    // Magnetometer data (normalized)
    double mag_x;
    double mag_y;
    double mag_z;
} INSSensorData;

// INS state data
typedef struct {
    // Position
    Position position;
    
    // Orientation (radians)
    double roll;
    double pitch;
    double yaw;
    
    // Velocities (m/s)
    double velocity_n;  // North velocity
    double velocity_e;  // East velocity
    double velocity_d;  // Down velocity
    
    // Bias and drift estimates
    double gyro_bias[3];
    double accel_bias[3];
    
    // Error estimates
    double position_error;  // meters
    double attitude_error;  // radians
} INSState;

typedef struct INS INS;

// Initialize INS
INS* ins_init(Bus* bus);

// Clean up INS
void ins_cleanup(INS* ins);

// Process one iteration of INS calculations
void ins_process(INS* ins);

// Main entry point for INS process
void ins_main(Bus* bus);

// Get the current INS state
const INSState* ins_get_state(const INS* ins);

// Simulate sensor readings based on current flight state
INSSensorData ins_simulate_sensors(const FlightState* flight_state);

#endif // INS_H
