#include "ins.h"
#include "log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#define INS_UPDATE_INTERVAL_MS 10  // 100Hz update rate
#define STATUS_UPDATE_INTERVAL_S 1
#define INIT_TIMEOUT_S 10         // Time to wait for GPS before failing

// Sensor noise parameters
#define ACCEL_NOISE 0.05         // m/s^2
#define GYRO_NOISE 0.001         // rad/s
#define MAG_NOISE 0.01          // normalized
#define ACCEL_BIAS_DRIFT 0.0001  // m/s^2 per second
#define GYRO_BIAS_DRIFT 0.0001   // rad/s per second

struct INS {
    Bus* bus;
    INSState state;
    INSSensorData sensors;
    FlightState current_state;
    Position gps_position;
    bool gps_valid;
    struct timespec last_update;
    time_t last_status_update;
    time_t start_time;
    bool initialized;
};

// Generate random sensor noise using Box-Muller transform
static double generate_noise(double magnitude) {
    double u1 = (double)rand() / RAND_MAX;
    double u2 = (double)rand() / RAND_MAX;
    return magnitude * sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}

static void send_status_update(INS* ins, bool operational) {
    Message msg = {0};
    msg.header.type = MSG_SYSTEM_STATUS;
    msg.header.sender = COMPONENT_INS;
    msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
    msg.header.timestamp = time(NULL);
    msg.header.message_size = sizeof(SystemStatusMsg);
    msg.payload.system_status.component_active = operational;
    
    if (bus_publish(ins->bus, &msg) != SUCCESS) {
        LOG_ERROR(LOG_INS, "Failed to publish status update");
    } else {
        LOG_INFO(LOG_INS, "Published status: %s", operational ? "OPERATIONAL" : "FAILED");
    }
}

INS* ins_init(Bus* bus) {
    LOG_INFO(LOG_INS, "Starting initialization");
    
    if (!bus) {
        LOG_ERROR(LOG_INS, "NULL bus in init");
        return NULL;
    }

    INS* ins = malloc(sizeof(INS));
    if (!ins) {
        LOG_ERROR(LOG_INS, "Failed to allocate memory");
        return NULL;
    }

    ins->bus = bus;
    memset(&ins->state, 0, sizeof(INSState));
    memset(&ins->sensors, 0, sizeof(INSSensorData));
    memset(&ins->current_state, 0, sizeof(FlightState));
    memset(&ins->gps_position, 0, sizeof(Position));
    ins->gps_valid = false;
    clock_gettime(CLOCK_MONOTONIC, &ins->last_update);
    ins->last_status_update = 0;
    ins->start_time = time(NULL);
    ins->initialized = false;

    // Subscribe to messages
    ErrorCode err;
    if ((err = bus_subscribe(bus, COMPONENT_INS, MSG_STATE_RESPONSE)) != SUCCESS ||
        (err = bus_subscribe(bus, COMPONENT_INS, MSG_POSITION_UPDATE)) != SUCCESS) {
        LOG_ERROR(LOG_INS, "Failed to subscribe to messages: %d", err);
        free(ins);
        return NULL;
    }

    // Initialize random number generator for noise
    srand(time(NULL));

    LOG_INFO(LOG_INS, "Initialization complete, waiting for GPS fix");
    return ins;
}

void ins_cleanup(INS* ins) {
    if (!ins) return;
    LOG_INFO(LOG_INS, "Cleaning up");
    free(ins);
}

INSSensorData ins_simulate_sensors(const FlightState* flight_state) {
    INSSensorData sensors = {0};
    
    if (!flight_state) {
        LOG_WARN(LOG_INS, "NULL flight state in sensor simulation");
        return sensors;
    }

    // Convert flight parameters to sensor readings
    double pitch_rad = 0.0;  // Assume level flight for now
    double roll_rad = 0.0;   // Assume no bank
    double yaw_rad = DEG_TO_RAD(flight_state->heading);
    
    // Convert speed from knots to m/s
    double speed_ms = flight_state->speed * KNOTS_TO_MS;
    
    // Calculate accelerations
    double forward_accel = 0.0;  // Assume constant speed
    double centripetal_accel = speed_ms * speed_ms / EARTH_RADIUS;  // Earth curvature
    
    // Convert vertical speed from feet/min to m/s
    double vertical_speed_ms = flight_state->vertical_speed * 0.00508;
    
    // Transform accelerations to body frame
    double cos_pitch = cos(pitch_rad);
    double sin_pitch = sin(pitch_rad);
    double cos_roll = cos(roll_rad);
    double sin_roll = sin(roll_rad);
    
    // Simulate accelerometer readings with noise
    sensors.accel_x = forward_accel * cos_pitch + 
                     centripetal_accel * sin_pitch +
                     generate_noise(ACCEL_NOISE);
                     
    sensors.accel_y = forward_accel * sin_roll * sin_pitch +
                     centripetal_accel * sin_roll * cos_pitch +
                     generate_noise(ACCEL_NOISE);
                     
    sensors.accel_z = -forward_accel * cos_roll * sin_pitch +
                     centripetal_accel * cos_roll * cos_pitch +
                     GRAVITY +
                     generate_noise(ACCEL_NOISE);
    
    // Simulate gyroscope readings
    sensors.gyro_x = generate_noise(GYRO_NOISE);  // Roll rate
    sensors.gyro_y = generate_noise(GYRO_NOISE);  // Pitch rate
    sensors.gyro_z = DEG_TO_RAD(flight_state->heading) +  // Yaw rate
                    generate_noise(GYRO_NOISE);
    
    // Simulate magnetometer readings
    sensors.mag_x = cos(yaw_rad) + generate_noise(MAG_NOISE);
    sensors.mag_y = sin(yaw_rad) + generate_noise(MAG_NOISE);
    sensors.mag_z = generate_noise(MAG_NOISE);

    // Add vertical acceleration component
    if (fabs(vertical_speed_ms) > 0.1) {
        sensors.accel_z += vertical_speed_ms * 0.1 + generate_noise(ACCEL_NOISE);
    }

    LOG_TRACE(LOG_INS, "Simulated sensors - Acc(x,y,z): %.2f,%.2f,%.2f Gyro(x,y,z): %.3f,%.3f,%.3f",
              sensors.accel_x, sensors.accel_y, sensors.accel_z,
              sensors.gyro_x, sensors.gyro_y, sensors.gyro_z);

    return sensors;
}

static void process_sensors(INS* ins, double dt) {
    LOG_TRACE(LOG_INS, "Processing sensor data with dt=%f", dt);

    // Update velocities based on accelerometer readings
    ins->state.velocity_n += ins->sensors.accel_x * dt;
    ins->state.velocity_e += ins->sensors.accel_y * dt;
    ins->state.velocity_d += ins->sensors.accel_z * dt;

    // Apply damping to prevent unbounded growth
    const double damping = 0.99;
    ins->state.velocity_n *= damping;
    ins->state.velocity_e *= damping;
    ins->state.velocity_d *= damping;

    LOG_TRACE(LOG_INS, "Velocities (m/s) - N: %.2f, E: %.2f, D: %.2f",
              ins->state.velocity_n, ins->state.velocity_e, ins->state.velocity_d);

    // Update position
    const double meters_per_degree_lat = 111111.0;
    double meters_per_degree_lon = meters_per_degree_lat * 
                                 cos(DEG_TO_RAD(ins->state.position.latitude));

    double lat_change = (ins->state.velocity_n * dt) / meters_per_degree_lat;
    double lon_change = (ins->state.velocity_e * dt) / meters_per_degree_lon;
    double alt_change = -ins->state.velocity_d * dt;

    ins->state.position.latitude += lat_change;
    ins->state.position.longitude += lon_change;
    ins->state.position.altitude += alt_change;

    LOG_TRACE(LOG_INS, "Position updates - Lat: %.6f, Lon: %.6f, Alt: %.1f",
              lat_change, lon_change, alt_change);

    // Update attitude
    ins->state.roll += ins->sensors.gyro_x * dt;
    ins->state.pitch += ins->sensors.gyro_y * dt;
    ins->state.yaw += ins->sensors.gyro_z * dt;

    // Normalize angles
    ins->state.roll = fmod(ins->state.roll + PI, 2*PI) - PI;
    ins->state.pitch = fmod(ins->state.pitch + PI, 2*PI) - PI;
    ins->state.yaw = fmod(ins->state.yaw, 2*PI);

    // Update sensor biases with random walk
    for (int i = 0; i < 3; i++) {
        ins->state.gyro_bias[i] += generate_noise(GYRO_BIAS_DRIFT * dt);
        ins->state.accel_bias[i] += generate_noise(ACCEL_BIAS_DRIFT * dt);
    }

    // Update error estimates
    ins->state.position_error += 0.1 * dt;  // 0.1 meters per second drift
    ins->state.attitude_error += 0.001 * dt;  // 0.001 radians per second drift

    LOG_TRACE(LOG_INS, "Error estimates - Pos: %.2f m, Att: %.3f rad",
              ins->state.position_error, ins->state.attitude_error);
}

void ins_process(INS* ins) {
    if (!ins) {
        LOG_ERROR(LOG_INS, "NULL INS in process");
        return;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = (now.tv_sec - ins->last_update.tv_sec) +
                (now.tv_nsec - ins->last_update.tv_nsec) / 1e9;

    // Send periodic status updates
    time_t current_time = time(NULL);
    if (current_time - ins->last_status_update >= STATUS_UPDATE_INTERVAL_S) {
        send_status_update(ins, ins->initialized);
        ins->last_status_update = current_time;
    }

    // Process incoming messages
    Message msg;
    while (bus_read_message(ins->bus, COMPONENT_INS, &msg)) {
        LOG_DEBUG(LOG_INS, "Received message type %d from component %d",
                 msg.header.type, msg.header.sender);

        if (msg.header.type == MSG_POSITION_UPDATE && 
            msg.header.sender == COMPONENT_GPS) {
            // Got GPS position
            ins->gps_position = msg.payload.position_update.position;
            ins->gps_valid = true;
            
            if (!ins->initialized) {
                // Initialize INS with GPS position
                ins->state.position = ins->gps_position;
                memset(&ins->state.gyro_bias, 0, sizeof(ins->state.gyro_bias));
                memset(&ins->state.accel_bias, 0, sizeof(ins->state.accel_bias));
                ins->state.position_error = 0;
                ins->state.attitude_error = 0;
                ins->initialized = true;
                LOG_INFO(LOG_INS, "Initialized with GPS position: %.6f, %.6f, %.1f",
                        ins->gps_position.latitude,
                        ins->gps_position.longitude,
                        ins->gps_position.altitude);
                send_status_update(ins, true);
            }
        } else if (msg.header.type == MSG_STATE_RESPONSE) {
            // Got flight state update
            memcpy(&ins->current_state, &msg.payload.state_response.state, 
                   sizeof(FlightState));
            
            // Update sensor readings
            ins->sensors = ins_simulate_sensors(&ins->current_state);
            LOG_DEBUG(LOG_INS, "Updated flight state and sensors");
        }
    }

    // Check timeout for initialization
    if (!ins->initialized && 
        (current_time - ins->start_time) > INIT_TIMEOUT_S) {
        LOG_ERROR(LOG_INS, "Failed to get GPS fix within timeout");
        send_status_update(ins, false);
        return;
    }

    // If initialized, update position based on sensors
    if (ins->initialized) {
        process_sensors(ins, dt);
        
        // Publish position update
        Message pos_msg = {0};
        pos_msg.header.type = MSG_POSITION_UPDATE;
        pos_msg.header.sender = COMPONENT_INS;
        pos_msg.header.receiver = COMPONENT_FLIGHT_CONTROLLER;
        pos_msg.header.timestamp = current_time;
        pos_msg.header.message_size = sizeof(PositionUpdateMsg);
        pos_msg.payload.position_update.position = ins->state.position;
        
        if (bus_publish(ins->bus, &pos_msg) == SUCCESS) {
            LOG_DEBUG(LOG_INS, "Published position: %.6f, %.6f, %.1f",
                     ins->state.position.latitude,
                     ins->state.position.longitude,
                     ins->state.position.altitude);
        } else {
            LOG_ERROR(LOG_INS, "Failed to publish position");
        }
    }

    ins->last_update = now;
}

void ins_main(Bus* bus) {
    LOG_INFO(LOG_INS, "Starting main function");
    
    INS* ins = ins_init(bus);
    if (!ins) {
        LOG_ERROR(LOG_INS, "Failed to initialize");
        return;
    }

    LOG_INFO(LOG_INS, "Entering main loop");
    
    while (1) {
        ins_process(ins);
        usleep(INS_UPDATE_INTERVAL_MS * 1000);
    }

    ins_cleanup(ins);
}
