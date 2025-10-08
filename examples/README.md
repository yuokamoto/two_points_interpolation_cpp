# Examples

This directory contains example applications demonstrating the use of the two-point interpolation library.

## Files

### Example Applications
- `two_points_interpolation_constant_acc_example.cpp` - Constant acceleration interpolation example
- `two_points_interpolation_constant_jerk_example.cpp` - Constant jerk interpolation example

### Configuration Files  
- `constraints.yaml` - Parameters for acceleration example
- `constraints_jerk.yaml` - Parameters for jerk example

### Build Scripts
- `build_and_run.sh [acc|jerk]` - Build and run examples (unified script)
- `run.sh [acc|jerk]` - Run examples (assumes already built)
- `build.sh` - Build both examples
- `CMakeLists.txt` - CMake build configuration

## Quick Start

### Install Dependencies
```bash
sudo apt install libyaml-cpp-dev gnuplot
```

### Run Examples
```bash
# Make scripts executable (first time only)
chmod +x build_and_run.sh run.sh build.sh

# Run constant acceleration example (default)
./build_and_run.sh
./build_and_run.sh acc

# Run constant jerk example  
./build_and_run.sh jerk

# Run without rebuilding (if already built)
./run.sh acc
./run.sh jerk
```

## Output Files

Each example generates:
- **Data file**: `data.txt` or `data_jerk.txt` containing trajectory points
- **Plot script**: `plot.gnu` or `plot_jerk.gnu` for gnuplot
- **Graph image**: `graph.png` or `graph_jerk.png` showing trajectory plots

## Configuration Parameters

### Common Parameters
- `ps`/`p0`: Start position
- `pe`: End position  
- `v0`: Start velocity
- `ve`: End velocity
- `amax`: Maximum acceleration
- `vmax`: Maximum velocity
- `t0`: Start time
- `dt`: Sampling interval
- `verbose`: Enable detailed output

### Jerk-Specific Parameters
- `jmax`: Maximum jerk (rate of change of acceleration)

## Customization

Edit the YAML configuration files to test different trajectory scenarios:
- Change position and velocity constraints
- Adjust maximum acceleration, velocity, and jerk limits
- Modify sampling rate and timing parameters
