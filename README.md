# Two Points Interpolation C++

C++ implementation of trajectory planning between two points with kinematic constraints.

## Overview

This library provides two trajectory planning algorithms in C++:
1. **Constant Acceleration**: Generates smooth trajectories with acceleration/deceleration limits
2. **Constant Jerk**: Generates even smoother trajectories with jerk (acceleration derivative) limits

**Key Features:**
- ✅ Independent acceleration and deceleration limits (`acc_max` != `dec_max`)
- ✅ Non-zero initial and final velocities
- ✅ Time-optimal trajectory generation
- ✅ Header-only implementation for easy integration
- ✅ Comprehensive test suite (40+ test scenarios)
- ✅ Apache 2.0 license

**For detailed mathematical derivations and theory**, see the [Python version documentation](https://github.com/yuokamoto/two_points_interpolation_py).

## Quick Start

### Constant Acceleration Example

```cpp
#include "two_point_interpolation/constant_acc.hpp"

TwoPointInterpolation interp;

// Parameters: p0, pe, acc_max, vmax, t0, v0, ve, dec_max
interp.init(0.0, 100.0, 2.0, 10.0, 0.0, 0.0, 0.0, 4.0);  // dec_max=4.0 (faster deceleration)

double total_time = interp.calcTrajectory();

// Get position, velocity, acceleration at time t
auto result = interp.getPoint(5.0);
double pos = result[0];
double vel = result[1]; 
double acc = result[2];
```

**Note**: If `dec_max` is not specified (or set to -1), it defaults to `acc_max`.

### Constant Jerk Example

```cpp
#include "two_point_interpolation/constant_jerk.hpp"

TwoPointInterpolationJerk interp;

// Parameters: p0, pe, amax, vmax, jmax, t0, v0, ve
interp.init(0.0, 100.0, 2.0, 10.0, 1.0, 0.0, 0.0, 0.0);

double total_time = interp.calcTrajectory();

// Get position, velocity, acceleration, jerk at time t
auto result = interp.getPoint(5.0);
double pos = result[0];
double vel = result[1]; 
double acc = result[2];
double jerk = result[3];
```

## Project Structure

```
two_points_interpolation_cpp/
├── include/
│   └── two_point_interpolation/
│       ├── constant_acc.hpp     # Constant acceleration trajectory planner
│       └── constant_jerk.hpp    # Constant jerk trajectory planner
├── tests/
│   └── test_interpolation.cpp   # Comprehensive test suite
├── examples/
│   ├── constant_acc_example.cpp
│   ├── constant_jerk_example.cpp
│   ├── constraints.yaml
│   └── ...
├── docs/
│   └── CHANGELOG.md
├── build_and_test.sh
├── LICENSE
└── README.md
```

## Building and Testing

```bash
# Build and run unit tests
./build_and_test.sh
```

**Test Coverage:**
- Basic functionality tests
- Error handling validation
- Parameterized tests for Case 0 (vmax not reached) and Case 1 (vmax reached)
- Non-zero initial/final velocity scenarios
- Independent acceleration/deceleration tests

## Examples with Plotting

### Dependencies
```bash
sudo apt install libyaml-cpp-dev gnuplot
```

### Running Examples

```bash
cd examples

# Run constant acceleration example
./build_and_run.sh acc

# Run constant jerk example
./build_and_run.sh jerk
```

Configuration files: `constraints.yaml`, `constraints_jerk.yaml`

### Example Results

#### Case 0: vmax not reached
![Case 0](examples/images/acc_constant_0.png)

Constraints: p0=10, pe=170, v0=-5, ve=1, acc_max=1, vmax=100, t0=10

#### Case 1: vmax reached  
![Case 1](examples/images/acc_constant_1.png)

Constraints: p0=50, pe=-90, v0=-5, ve=1, acc_max=1, vmax=10, t0=0

## API Reference

### TwoPointInterpolation (Constant Acceleration)

```cpp
void init(double p0, double pe, double acc_max, double vmax, 
          double t0 = 0, double v0 = 0, double ve = 0, double dec_max = -1.0);
double calcTrajectory();
std::vector<double> getPoint(double t) const;  // Returns [pos, vel, acc]
```

### TwoPointInterpolationJerk (Constant Jerk)

```cpp
void init(double p0, double pe, double amax, double vmax, double jmax,
          double t0 = 0, double v0 = 0, double ve = 0);
double calcTrajectory();
std::vector<double> getPoint(double t) const;  // Returns [pos, vel, acc, jerk]
```

## Error Handling

The library validates:
- ❌ Negative or zero constraints
- ❌ Uninitialized trajectory calculations
- ❌ Same position with different velocities (physically impossible)
- ❌ Invalid trajectories (insufficient distance for given vmax)

## License

Apache License 2.0 - see [LICENSE](LICENSE) file for details.

This software is provided "AS IS" without warranty of any kind.

## Related Projects

- **Python Version**: [two_points_interpolation_py](https://github.com/yuokamoto/two_points_interpolation_py) - Detailed documentation, mathematical derivations, and pip-installable package

## Contributing

Issues and pull requests are welcome on [GitHub](https://github.com/yuokamoto/two_points_interpolation_cpp).
