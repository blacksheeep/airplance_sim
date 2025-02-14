#!/bin/bash

# Function to check if a port is in use
check_port() {
    local port=$1
    netstat -an | grep ":$port " | grep -q LISTEN
    return $?
}

# Function to kill process using a specific port
kill_port_process() {
    local port=$1
    local pid=$(lsof -ti tcp:$port)
    if [ ! -z "$pid" ]; then
        echo "Killing process using port $port (PID: $pid)"
        kill -9 $pid 2>/dev/null
    fi
}

echo "Starting thorough cleanup..."

# Kill specific processes
echo "Killing simulation processes..."
pkill -f gps_sender
pkill -f landing_radio_sender
pkill -f sat_com_sender
pkill -f airplane_sim

# Check and clean up ports
for port in 5555 5556 5557; do
    if check_port $port; then
        echo "Cleaning up port $port..."
        kill_port_process $port
    fi
done

# Wait for processes to terminate
sleep 1

# Double-check ports
for port in 5555 5556 5557; do
    if check_port $port; then
        echo "Warning: Port $port still in use"
        kill_port_process $port
        sleep 1
    fi
done

# Clean up socket files
echo "Cleaning up socket files..."
rm -f /tmp/airplane_sim_*

# Clean up semaphores
echo "Cleaning up semaphores..."
for sem in $(ipcs -s | grep airplane_sim | awk '{print $2}'); do
    ipcrm -s $sem 2>/dev/null
done

# Clean up shared memory
echo "Cleaning up shared memory..."
for shm in $(ipcs -m | grep $USER | awk '{print $2}'); do
    ipcrm -m $shm 2>/dev/null
done

echo "Cleanup complete"
sleep 1
