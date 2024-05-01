#!/bin/bash

# Function to perform cleanup
cleanup() {
    echo "Cleaning up..."
    # Add any cleanup actions here if needed
    exit 1
}

# Trap SIGINT signal (Ctrl+C)
trap cleanup SIGINT

for instance in ../instances/nlsp_*.inp; do
    echo "Running instance $instance"
    gtimeout 60s ./nlsp < "$instance" | ./checker
    echo "................"
done