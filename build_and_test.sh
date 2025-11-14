#!/bin/bash

# Simple build script for the test suite
echo "Building C++ interpolation test suite..."

# Compile the test
g++ -std=c++11 -Wall -Wextra -O2 -Iinclude -o test_interpolation tests/test_interpolation.cpp

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo "Running tests..."
    ./test_interpolation
else
    echo "❌ Build failed!"
    exit 1
fi
