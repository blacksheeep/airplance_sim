#!/bin/bash

# First run cleanup
./cleanup.sh

# Ensure config directory exists
mkdir -p config

# Ensure autopilot config exists
if [ ! -f config/autopilot_config.json ]; then
    echo "Creating default autopilot config..."
    cat > config/autopilot_config.json << EOL
{
    "target_latitude": 37.7749,
    "target_longitude": -122.4194,
    "target_altitude": 10000.0,
    "target_speed": 250.0,
    "max_climb_rate": 2000.0,
    "max_descent_rate": 1500.0,
    "max_bank_angle": 25.0,
    "max_pitch_angle": 15.0
}
EOL
fi

# Start external components
echo "Starting external components..."
./build/external/gps_sender &
sleep 0.5
./build/external/landing_radio_sender &
sleep 0.5
./build/external/sat_com_sender &
sleep 0.5

# Wait for components to initialize
echo "Waiting for external components to initialize..."
sleep 1

# Start main simulation
echo "Starting main simulation..."
./build/airplane_sim

# When main simulation exits, kill external components
echo "Stopping external components..."
./cleanup.sh
