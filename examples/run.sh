#!/bin/bash

# Usage: ./run.sh [acc|jerk]
# Default: acc

TYPE=${1:-acc}

if [ "$TYPE" = "acc" ]; then
    echo "Running constant acceleration example..."
    ./build/acc_example constraints.yaml
elif [ "$TYPE" = "jerk" ]; then
    echo "Running constant jerk example..."
    ./build/jerk_example constraints_jerk.yaml
else
    echo "Usage: $0 [acc|jerk]"
    echo "  acc  - Run constant acceleration example (default)"
    echo "  jerk - Run constant jerk example"
    exit 1
fi