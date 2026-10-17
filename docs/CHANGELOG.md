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
- README example: an initial velocity away from the target
  (`examples/constraints_case2.yaml`, `examples/images/acc_constant_2.png`)
- Tests for an initial velocity away from the target, acceleration-only arrivals,
  infeasible requests, a grid check that every returned trajectory ends at (`pe`, `ve`)
  within its limits, and an angle turning away from its target

### Changed
- Fixed quadratic equation solver to properly handle forward/backward motion
- Reorganized directory structure to match Python version:
  - Headers moved to `include/two_point_interpolation/`
  - Tests moved to `tests/`
  - Simplified example filenames
- Simplified README with link to Python repo for mathematical derivations
- Updated examples to demonstrate `dec_max` parameter
- **Behaviour change**: inputs that used to return a wrong trajectory now throw
  `std::runtime_error`. These are requests whose end velocity `ve` is faster, toward
  `pe`, than any speed reachable within the distance; the trajectory returned for them
  never ended at `pe`. Inputs that produced a correct trajectory are unaffected.
- Error messages name the actual cause: "Insufficient distance to decelerate ..." when
  the body moves toward `pe` too fast to slow down to `ve`, and "End velocity
  unreachable ..." with the highest reachable speed when `ve` cannot be reached. The
  messages referring to a discriminant or to "no positive time solution" are gone,
  since the quadratic is no longer solved.

### Fixed
- Discriminant calculation for asymmetric acceleration/deceleration
- Solution selection logic for quadratic equation (both roots considered)
- **Initial velocity away from the target**: `calcTrajectory()` could return a
  trajectory that never reaches `pe` when `v0` points away from it.
  - The acceleration time was taken as the smaller positive root of a quadratic whose
    two roots correspond to a peak velocity `v1` of `+|v1|` and `-|v1|`. When `v0`
    points away from `pe` over a short enough distance, both roots are positive and
    the smaller one can be the `-|v1|` root: the acceleration phase ends still moving
    away, and the deceleration phase drives further away. `getPoint()` returns `pe`
    from the end time on, so the result only showed as a jump at the end.
  - `v1` is now computed directly from the distance equation and takes its sign from
    the direction of travel. Results are unchanged wherever the old root was correct.
  - An acceleration-only arrival (`ve` exactly the speed reached by accelerating over
    the whole distance) is accepted despite square-root rounding: a peak within a
    relative `1e-9` of `ve` is snapped onto it.
- **Initial velocity above vmax toward the target**: the vmax case accelerated from
  `v0` to `vmax` even when `v0` was already faster, giving a negative phase duration.
  It now decelerates to `vmax` first, as the Python version does.

## Related

For mathematical derivations and detailed documentation, see the [Python version](https://github.com/yuokamoto/two_points_interpolation_py).
