#!/bin/bash

# Usage: ./build_and_run.sh [acc|jerk]
# Default: acc

TYPE=${1:-acc}

echo "=== Building Two Points Interpolation Examples ==="
./build.sh

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    echo "=== Running $TYPE interpolation example ==="
    ./run.sh "$TYPE"
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "📊 Generated files:"
        if [ "$TYPE" = "acc" ]; then
            ls -la data.txt plot.gnu graph.png 2>/dev/null || echo "Some output files may not have been generated"
        elif [ "$TYPE" = "jerk" ]; then
            ls -la data_jerk.txt plot_jerk.gnu graph_jerk.png 2>/dev/null || echo "Some output files may not have been generated"
        fi
        echo ""
        echo "✅ Example completed successfully!"
    else
        echo "❌ Example execution failed!"
        exit 1
    fi
else
    echo "❌ Build failed!"
    echo "Make sure dependencies are installed:"
    echo "  sudo apt install libyaml-cpp-dev gnuplot"
    exit 1
fi