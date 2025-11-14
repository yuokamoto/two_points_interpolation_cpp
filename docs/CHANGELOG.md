# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Added
- Independent acceleration and deceleration limits (`dec_max` parameter)
- Apache 2.0 license headers to all source files
- Comprehensive test suite with 40+ parameterized test scenarios
  - 12 Case 0 tests (vmax not reached)
  - 13 Case 1 tests (vmax reached)
  - Non-zero initial/final velocity tests
  - Asymmetric acceleration/deceleration validation
- Error handling for invalid trajectories (`dt12 < 0` check)
- Improved error messages with constraint values

### Changed
- Fixed quadratic equation solver to properly handle forward/backward motion
- Reorganized directory structure to match Python version:
  - Headers moved to `include/two_point_interpolation/`
  - Tests moved to `tests/`
  - Simplified example filenames
- Simplified README with link to Python repo for mathematical derivations
- Updated examples to demonstrate `dec_max` parameter

### Fixed
- Discriminant calculation for asymmetric acceleration/deceleration
- Solution selection logic for quadratic equation (both roots considered)

## Related

For mathematical derivations and detailed documentation, see the [Python version](https://github.com/yuokamoto/two_points_interpolation_py).
