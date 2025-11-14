#!/bin/bash

# Usage: ./run.sh [acc|jerk] [config_file.yaml]
# Default: acc with constraints_case0.yaml

TYPE=${1:-acc}
CONFIG=${2:-}

if [ "$TYPE" = "acc" ]; then
    echo "Running constant acceleration example..."
    if [ -n "$CONFIG" ]; then
        ./build/acc_example "$CONFIG"
    else
        ./build/acc_example constraints_case0.yaml
    fi
elif [ "$TYPE" = "jerk" ]; then
    echo "Running constant jerk example..."
    if [ -n "$CONFIG" ]; then
        ./build/jerk_example "$CONFIG"
    else
        ./build/jerk_example constraints_jerk.yaml
    fi
else
    echo "Usage: $0 [acc|jerk] [config_file.yaml]"
    echo "  acc  - Run constant acceleration example (default)"
    echo "  jerk - Run constant jerk example"
    exit 1
fi