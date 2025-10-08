# Two Points Interpolation C++

C++ implementation of trajectory planning between two points with kinematic constraints.

## Overview

This library provides two trajectory planning algorithms in C++:
1. **Constant Acceleration**: Generates smooth trajectories with acceleration limits
2. **Constant Jerk**: Generates even smoother trajectories with jerk (acceleration derivative) limits

## Features

- ✅ Position, velocity, and acceleration constraints
- ✅ Time-optimal trajectory generation  
- ✅ Robust error handling and input validation
- ✅ Header-only implementation for easy integration
- ✅ Comprehensive test suite
- ✅ Support for both single-point and two-point initialization APIs

## Quick Start

### Constant Acceleration Example

```cpp
#include "two_points_interpolation_constant_acc.hpp"

// Create interpolator
TwoPointInterpolation interp;

// Set parameters: start_pos, end_pos, max_acc, max_vel, start_time, start_vel, end_vel
interp.init(0.0, 100.0, 2.0, 10.0, 0.0, 0.0, 0.0);

// Calculate trajectory
double total_time = interp.calcTrajectory();

// Get position, velocity, acceleration at any time t
auto result = interp.getPoint(5.0);
double pos = result[0];
double vel = result[1]; 
double acc = result[2];
```

### Constant Jerk Example

```cpp
#include "two_points_interpolation_constant_jerk.hpp"

// Create interpolator
TwoPointInterpolationJerk interp;

// Method 1: New API (compatible with constant_acc)
interp.init(0.0, 100.0, 2.0, 10.0, 1.0, 0.0, 0.0, 0.0);

// Method 2: Original API
// std::vector<double> max_values = {vmax, amax, jmax};
// interp.set(0.0, 100.0, max_values);
// interp.setInitialTime(0.0);

// Calculate trajectory
double total_time = interp.calcTrajectory();

// Get position, velocity, acceleration, jerk at any time t
auto result = interp.getPoint(5.0);
double pos = result[0];
double vel = result[1]; 
double acc = result[2];
double jerk = result[3];
```

## Files

- `two_points_interpolation_constant_acc.hpp`: Constant acceleration trajectory planner (header-only)
- `two_points_interpolation_constant_jerk.hpp`: Constant jerk trajectory planner (header-only) 
- `test_interpolation.cpp`: Comprehensive test suite
- `build_and_test.sh`: Build and run test script
- `examples/`: Example applications with YAML configuration and plotting

## Running Tests

```bash
# Build and run unit tests
./build_and_test.sh
```

## Examples with Plotting

### Dependencies
- yaml-cpp: `sudo apt install libyaml-cpp-dev`
- gnuplot: `sudo apt-get install gnuplot`

### Running Examples

The examples use a unified build system with argument-based selection:

```bash
# Install dependencies
sudo apt install libyaml-cpp-dev gnuplot

# Navigate to examples directory
cd examples

# Run constant acceleration example (default)
./build_and_run.sh
./build_and_run.sh acc

# Run constant jerk example
./build_and_run.sh jerk

# Run without rebuilding (if already built)
./run.sh acc
./run.sh jerk
```

### Configuration Files
- `constraints.yaml`: Configuration for constant acceleration interpolation
- `constraints_jerk.yaml`: Configuration for constant jerk interpolation

### Generated Output Files

**Acceleration Example:**
- `data.txt`: Trajectory data points
- `plot.gnu`: gnuplot script  
- `graph.png`: Visualization plots

**Jerk Example:**
- `data_jerk.txt`: Trajectory data points  
- `plot_jerk.gnu`: gnuplot script
- `graph_jerk.png`: Visualization plots (jerk, acceleration, velocity, position)

### Example Features
- YAML-based parameter configuration
- Automatic plot generation with gnuplot
- Data export to text files
- Verbose trajectory information
- Unified build system for both algorithms

## Error Handling

The library includes robust error handling for:
- Invalid constraint values (negative or zero)
- Uninitialized trajectory calculations
- Physical impossibilities (same position, different velocities)
- Missing required parameters

### Example result
#### case 0: not reach velocity limit
constraints: 
- p0 = 10, pe = 170.0
- v0 = -5, ve = 1
- amax = 1, vmax = 100
- t0 = 10

![alt text](examples/images/acc_constant_0.png)

#### case 1: reach velocity limit
constraints: 
- p0 = 50, pe = -90.0
- v0 = -5, ve = 1
- amax = 1, vmax = 10
- t0 = 0

![alt text](examples/images/acc_constant_1.png)