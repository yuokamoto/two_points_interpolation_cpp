# Examples

This directory contains example applications demonstrating the use of the two-point interpolation library.

## Files

### Example Applications
- `constant_acc_example.cpp` - Constant acceleration interpolation example
- `constant_jerk_example.cpp` - Constant jerk interpolation example

### Configuration Files  
- `constraints_case0.yaml` - Case 0: vmax not reached (matches Python example)
- `constraints_case1.yaml` - Case 1: vmax reached (matches Python example)
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

# Run with Python-matching parameters
./build_and_run.sh acc constraints_case0.yaml  # Case 0: vmax not reached
./build_and_run.sh acc constraints_case1.yaml  # Case 1: vmax reached

# Run constant acceleration example (default)
./build_and_run.sh
./build_and_run.sh acc

# Run constant jerk example  
./build_and_run.sh jerk

# Run without rebuilding (if already built)
./run.sh acc constraints_case0.yaml
./run.sh jerk
```

## Output Files

Each example generates:
- **Data file**: `data.txt` or `data_jerk.txt` containing trajectory points
- **Plot script**: `plot.gnu` or `plot_jerk.gnu` for gnuplot
- **Graph image**: `graph.png` or `graph_jerk.png` showing trajectory plots

## Configuration Parameters

### Common Parameters
- `p0`: Start position
- `pe`: End position  
- `v0`: Start velocity
- `ve`: End velocity
- `amax`: Maximum acceleration
- `dec_max`: Maximum deceleration (optional, defaults to amax)
- `vmax`: Maximum velocity
- `t0`: Start time
- `dt`: Sampling interval
- `verbose`: Enable detailed output
- `normalize_angle`: Normalize angle to [-π, π]

### Example Configurations

**Case 0** (`constraints_case0.yaml`): vmax not reached
- `p0=5.0, pe=15.0, v0=0.0, ve=0.0`
- `amax=2.0, dec_max=3.0, vmax=10.0`
- Two phases: acceleration → deceleration

**Case 1** (`constraints_case1.yaml`): vmax reached  
- `p0=0.0, pe=50.0, v0=2.0, ve=1.0`
- `amax=2.0, dec_max=4.0, vmax=8.0`
- Three phases: acceleration → constant velocity → deceleration

### Jerk-Specific Parameters
- `jmax`: Maximum jerk (rate of change of acceleration)

