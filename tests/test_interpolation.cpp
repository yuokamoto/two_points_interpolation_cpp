// Copyright 2025 Yu Okamoto
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>

#include "two_point_interpolation/constant_acc.hpp"
#include "two_point_interpolation/constant_jerk.hpp"

// Test helper functions
bool almostEqual(double a, double b, double tolerance = 1e-5) {
    return std::abs(a - b) < tolerance;
}

void assertFinalState(const TwoPointInterpolation& tpi, double total_time, 
                     double expected_pe, double expected_ve) {
    auto result = tpi.getPoint(total_time);
    double p_final = result[0];
    double v_final = result[1];
    double a_final = result[2];
    
    if (!almostEqual(expected_pe, p_final)) {
        throw std::runtime_error("Final position mismatch: expected " + 
                               std::to_string(expected_pe) + ", got " + std::to_string(p_final));
    }
    if (!almostEqual(expected_ve, v_final)) {
        throw std::runtime_error("Final velocity mismatch: expected " + 
                               std::to_string(expected_ve) + ", got " + std::to_string(v_final));
    }
    if (!almostEqual(0.0, a_final)) {
        throw std::runtime_error("Final acceleration should be zero, got " + std::to_string(a_final));
    }
}

void assertBoundaryContinuity(const TwoPointInterpolation& tpi, double eps = 1e-6) {
    // Physics-based tolerances
    double p_tolerance = 1.1 * eps * tpi.getVmax();
    double v_tolerance = 1.1 * eps * std::max(tpi.getAmaxAccel(), tpi.getAmaxDecel());
    
    // Check continuity at each phase boundary
    double cumulative_time = 0.0;
    const auto& dt = tpi.getDt();
    for (size_t i = 0; i < dt.size() - 1; ++i) {  // Loop through all boundaries between phases
        cumulative_time += dt[i];
        double t_boundary = cumulative_time;
        
        // Get values before, at, and after the boundary
        auto result_before = tpi.getPoint(t_boundary - eps);
        auto result_at = tpi.getPoint(t_boundary);
        auto result_after = tpi.getPoint(t_boundary + eps);
        
        double p_before = result_before[0], v_before = result_before[1];
        double p_at = result_at[0], v_at = result_at[1];
        double p_after = result_after[0], v_after = result_after[1];
        
        // Check position continuity
        if (std::abs(p_before - p_at) > p_tolerance) {
            throw std::runtime_error("Position discontinuity before boundary " + std::to_string(i) +
                                   ": |" + std::to_string(p_before) + " - " + std::to_string(p_at) + 
                                   "| = " + std::to_string(std::abs(p_before - p_at)) + 
                                   " > " + std::to_string(p_tolerance));
        }
        if (std::abs(p_at - p_after) > p_tolerance) {
            throw std::runtime_error("Position discontinuity after boundary " + std::to_string(i) +
                                   ": |" + std::to_string(p_at) + " - " + std::to_string(p_after) + 
                                   "| = " + std::to_string(std::abs(p_at - p_after)) + 
                                   " > " + std::to_string(p_tolerance));
        }
        
        // Check velocity continuity
        if (std::abs(v_before - v_at) > v_tolerance) {
            throw std::runtime_error("Velocity discontinuity before boundary " + std::to_string(i) +
                                   ": |" + std::to_string(v_before) + " - " + std::to_string(v_at) + 
                                   "| = " + std::to_string(std::abs(v_before - v_at)) + 
                                   " > " + std::to_string(v_tolerance));
        }
        if (std::abs(v_at - v_after) > v_tolerance) {
            throw std::runtime_error("Velocity discontinuity after boundary " + std::to_string(i) +
                                   ": |" + std::to_string(v_at) + " - " + std::to_string(v_after) + 
                                   "| = " + std::to_string(std::abs(v_at - v_after)) + 
                                   " > " + std::to_string(v_tolerance));
        }
    }
}

// Test structure for parameterized tests
struct TestCase {
    double p0, pe, acc_max, dec_max, vmax, v0, ve;
    std::string description;
};

void testConstantAccBasic() {
    std::cout << "Testing TwoPointInterpolation (Constant Acc) basic functionality..." << std::endl;
    
    TwoPointInterpolation interp;
    interp.init(0.0, 10.0, 2.0, 5.0, 0.0, 0.0, 0.0);
    double te = interp.calcTrajectory();
    
    assert(te > 0);
    
    auto result = interp.getPoint(te);
    assert(almostEqual(result[0], 10.0, 0.01)); // position
    assert(almostEqual(result[1], 0.0, 0.01));  // velocity
    
    std::cout << "✅ Basic functionality test passed" << std::endl;
}

void testConstantAccErrorHandling() {
    std::cout << "Testing TwoPointInterpolation error handling..." << std::endl;
    
    TwoPointInterpolation interp;
    
    // Test invalid constraints
    try {
        interp.setConstraints(-1.0, 10.0);
        assert(false); // Should not reach here
    } catch (const std::invalid_argument& e) {
        std::cout << "✅ Caught expected error for negative amax" << std::endl;
    }
    
    try {
        interp.setConstraints(1.0, -10.0);
        assert(false); // Should not reach here
    } catch (const std::invalid_argument& e) {
        std::cout << "✅ Caught expected error for negative vmax" << std::endl;
    }
    
    try {
        interp.setConstraints(1.0, 10.0, 0.0);
        assert(false); // Should not reach here
    } catch (const std::invalid_argument& e) {
        std::cout << "✅ Caught expected error for zero dec_max" << std::endl;
    }
    
    // Test trajectory calculation without setup
    try {
        TwoPointInterpolation interp2;
        interp2.calcTrajectory();
        assert(false); // Should not reach here
    } catch (const std::runtime_error& e) {
        std::cout << "✅ Caught expected error for uninitialized trajectory" << std::endl;
    }
}

void testConstantAccZeroDisplacement() {
    std::cout << "Testing TwoPointInterpolation zero displacement..." << std::endl;
    
    // Test same position, same velocity (should work)
    TwoPointInterpolation interp1;
    interp1.init(10.0, 10.0, 2.0, 5.0, 0.0, 1.0, 1.0);
    double te1 = interp1.calcTrajectory();
    assert(almostEqual(te1, 0.0));
    
    auto result1 = interp1.getPoint(0.0);
    assert(almostEqual(result1[0], 10.0)); // position
    assert(almostEqual(result1[1], 1.0));  // velocity
    assert(almostEqual(result1[2], 0.0));  // acceleration
    
    std::cout << "✅ Zero displacement (same velocity) test passed" << std::endl;
    
    // Test same position, different velocity (should fail)
    try {
        TwoPointInterpolation interp2;
        interp2.init(10.0, 10.0, 2.0, 5.0, 0.0, 1.0, 2.0);
        interp2.calcTrajectory();
        assert(false); // Should not reach here
    } catch (const std::invalid_argument& e) {
        std::cout << "✅ Caught expected error for different velocities at same position" << std::endl;
    }
}

void testCase0() {
    std::cout << "\nTesting Case 0 (vmax not reached) with multiple parameter combinations..." << std::endl;
    
    std::vector<TestCase> test_cases = {
        // Small displacement with high vmax ensures Case 0
        {0, 10, 2.0, 3.0, 20.0, 0, 0, "forward, zero v0/ve, asymmetric acc/dec"},
        {10, 0, 2.0, 3.0, 20.0, 0, 0, "backward, zero v0/ve, asymmetric acc/dec"},
        {0, 5, 1.5, 2.5, 15.0, 0, 0, "forward, zero v0/ve, different acc/dec"},
        {0, 8, 2.0, 2.0, 25.0, 0, 0, "forward, zero v0/ve, symmetric acc/dec"},
        {5, 15, 3.0, 4.0, 30.0, 0, 0, "forward, zero v0/ve, non-zero start position"},
        {20, 8, 2.5, 3.5, 28.0, 0, 0, "backward, zero v0/ve, non-zero positions"},
        // Non-zero v0 cases
        {0, 8, 2.0, 3.0, 20.0, 1.0, 0, "forward, non-zero v0"},
        {10, 2, 2.0, 3.0, 20.0, 0.5, 0, "backward, non-zero v0"},
        // Non-zero ve cases
        {0, 6, 2.0, 3.0, 18.0, 0, 0.5, "forward, non-zero ve"},
        {12, 4, 2.0, 3.0, 20.0, 0, 0.3, "backward, non-zero ve"},
        // Non-zero v0 and ve cases
        {0, 5, 2.0, 3.0, 18.0, 0.8, 0.4, "forward, non-zero v0 and ve"},
        {10, 5, 2.5, 3.5, 22.0, 0.6, 0.3, "backward, non-zero v0 and ve"},
    };
    
    int passed = 0;
    for (const auto& tc : test_cases) {
        try {
            TwoPointInterpolation tpi;
            tpi.init(tc.p0, tc.pe, tc.acc_max, tc.vmax, 0.0, tc.v0, tc.ve, tc.dec_max);
            double total_time = tpi.calcTrajectory();
            
            // Should be Case 0
            assert(total_time > 0);
            
            // Physical validity: total_time should be at least dp/vmax
            double dp = std::abs(tc.pe - tc.p0);
            assert(total_time >= dp / tc.vmax - 1e-6);
            
            // Check final state
            assertFinalState(tpi, total_time, tc.pe, tc.ve);
            
            // Check boundary continuity at all phase transitions
            assertBoundaryContinuity(tpi);
            
            passed++;
        } catch (const std::exception& e) {
            std::cout << "❌ Test failed for: " << tc.description << std::endl;
            std::cout << "   Error: " << e.what() << std::endl;
            throw;
        }
    }
    
    std::cout << "✅ All " << passed << " Case 0 tests passed" << std::endl;
}

void testCase1() {
    std::cout << "\nTesting Case 1 (vmax reached) with multiple parameter combinations..." << std::endl;
    
    std::vector<TestCase> test_cases = {
        // Large displacement with low vmax ensures Case 1
        {0, 50, 2.0, 4.0, 8.0, 0, 0, "forward, zero v0/ve, asymmetric acc/dec"},
        {50, 0, 2.0, 4.0, 8.0, 0, 0, "backward, zero v0/ve, asymmetric acc/dec"},
        {0, 60, 3.0, 3.0, 10.0, 0, 0, "forward, zero v0/ve, symmetric acc/dec"},
        {0, 80, 2.5, 5.0, 12.0, 0, 0, "forward, zero v0/ve, faster deceleration"},
        {0, 100, 4.0, 2.5, 12.0, 0, 0, "forward, zero v0/ve, faster acceleration"},
        {10, 90, 3.0, 4.5, 10.0, 0, 0, "forward, zero v0/ve, non-zero start position"},
        {100, 20, 2.8, 3.8, 9.0, 0, 0, "backward, zero v0/ve, non-zero positions"},
        // Non-zero v0 cases
        {0, 55, 2.0, 4.0, 8.0, 1.5, 0, "forward, non-zero v0"},
        {60, 0, 2.5, 4.0, 9.0, 1.0, 0, "backward, non-zero v0"},
        // Non-zero ve cases
        {0, 52, 2.0, 4.0, 8.5, 0, 1.2, "forward, non-zero ve"},
        {55, 0, 2.5, 4.0, 9.0, 0, 0.8, "backward, non-zero ve"},
        // Non-zero v0 and ve cases
        {0, 58, 2.0, 4.0, 9.0, 1.8, 1.0, "forward, non-zero v0 and ve"},
        {70, 10, 2.5, 3.5, 10.0, 1.5, 0.8, "backward, non-zero v0 and ve"},
    };
    
    int passed = 0;
    for (const auto& tc : test_cases) {
        try {
            TwoPointInterpolation tpi;
            tpi.init(tc.p0, tc.pe, tc.acc_max, tc.vmax, 0.0, tc.v0, tc.ve, tc.dec_max);
            double total_time = tpi.calcTrajectory();
            
            // Should be Case 1
            assert(total_time > 0);
            
            // Physical validity: total_time should be at least dp/vmax
            double dp = std::abs(tc.pe - tc.p0);
            assert(total_time >= dp / tc.vmax - 1e-6);
            
            // Check final state
            assertFinalState(tpi, total_time, tc.pe, tc.ve);
            
            // Check boundary continuity at all phase transitions
            assertBoundaryContinuity(tpi);
            
            passed++;
        } catch (const std::exception& e) {
            std::cout << "❌ Test failed for: " << tc.description << std::endl;
            std::cout << "   Error: " << e.what() << std::endl;
            throw;
        }
    }
    
    std::cout << "✅ All " << passed << " Case 1 tests passed" << std::endl;
}

void testFasterDeceleration() {
    std::cout << "\nTesting that faster deceleration reduces total time..." << std::endl;
    
    // Same acceleration, different deceleration
    TwoPointInterpolation tpi1;
    tpi1.init(0, 30, 2.0, 10.0, 0.0, 0.0, 0.0, 2.0);
    double time1 = tpi1.calcTrajectory();
    
    TwoPointInterpolation tpi2;
    tpi2.init(0, 30, 2.0, 10.0, 0.0, 0.0, 0.0, 4.0);
    double time2 = tpi2.calcTrajectory();
    
    // Faster deceleration should result in shorter time
    if (time2 >= time1) {
        throw std::runtime_error("Faster deceleration should reduce total time");
    }
    
    std::cout << "✅ Faster deceleration test passed (time1=" << time1 
              << ", time2=" << time2 << ")" << std::endl;
}

void testDefaultDecMax() {
    std::cout << "\nTesting that dec_max defaults to acc_max when not specified..." << std::endl;
    
    // Without dec_max (should default to acc_max)
    TwoPointInterpolation tpi1;
    tpi1.init(0, 20, 2.0, 10.0, 0.0, 0.0, 0.0);
    double time1 = tpi1.calcTrajectory();
    
    // With explicit dec_max equal to acc_max
    TwoPointInterpolation tpi2;
    tpi2.init(0, 20, 2.0, 10.0, 0.0, 0.0, 0.0, 2.0);
    double time2 = tpi2.calcTrajectory();
    
    // Times should be approximately equal
    if (!almostEqual(time1, time2, 1e-4)) {
        throw std::runtime_error("Default dec_max should equal acc_max");
    }
    
    // Sample points should match at key times
    for (double t : {0.0, time1 * 0.25, time1 * 0.5, time1 * 0.75, time1}) {
        auto p1 = tpi1.getPoint(t);
        auto p2 = tpi2.getPoint(t);
        
        if (!almostEqual(p1[0], p2[0], 0.001)) {
            throw std::runtime_error("Position mismatch at t=" + std::to_string(t));
        }
        if (!almostEqual(p1[1], p2[1], 0.001)) {
            throw std::runtime_error("Velocity mismatch at t=" + std::to_string(t));
        }
    }
    
    std::cout << "✅ Default dec_max test passed" << std::endl;
}

void testConstantJerkBasic() {
    std::cout << "\nTesting TwoPointInterpolationJerk basic functionality..." << std::endl;
    
    TwoPointInterpolationJerk interp;
    interp.init(0.0, 100.0, 2.0, 5.0, 1.0, 0.0, 0.0, 0.0);
    double te = interp.calcTrajectory();
    
    assert(te > 0);
    
    auto result = interp.getPoint(te);
    std::cout << "  End position: " << result[0] << " (target: 100.0)" << std::endl;
    
    std::cout << "✅ Jerk basic functionality test passed" << std::endl;
}

void testConstantJerkErrorHandling() {
    std::cout << "Testing TwoPointInterpolationJerk error handling..." << std::endl;
    
    TwoPointInterpolationJerk interp;
    
    // Test invalid constraints
    try {
        interp.setConstraints(-1.0, 5.0, 1.0);
        assert(false); // Should not reach here
    } catch (const std::invalid_argument& e) {
        std::cout << "✅ Caught expected error for negative constraint" << std::endl;
    }
}

// ---------------------------------------------------------------------------
// Initial velocity away from the target, end-velocity feasibility, rounding
// ---------------------------------------------------------------------------

void check(bool condition, const std::string& what) {
    if (!condition) {
        throw std::runtime_error(what);
    }
}

// Position and velocity just before the end time: getPoint() returns (pe, ve)
// from the end time on, which would hide a trajectory that misses the target.
std::vector<double> stateJustBeforeEnd(const TwoPointInterpolation& tpi, double t0, double total_time) {
    return tpi.getPoint(t0 + total_time - 1e-9);
}

std::string describe(double p0, double pe, double v0, double acc, double dec, double vmax, double ve) {
    return "p0=" + std::to_string(p0) + " pe=" + std::to_string(pe) + " v0=" + std::to_string(v0) +
           " acc=" + std::to_string(acc) + " dec=" + std::to_string(dec) + " vmax=" + std::to_string(vmax) +
           " ve=" + std::to_string(ve);
}

void checkReachesTarget(double p0, double pe, double v0, double acc, double dec, double vmax, double ve) {
    TwoPointInterpolation tpi;
    tpi.init(p0, pe, acc, vmax, 0.0, v0, ve, dec);
    double total = tpi.calcTrajectory();
    auto end = stateJustBeforeEnd(tpi, 0.0, total);
    check(almostEqual(end[0], pe, 1e-6) && almostEqual(end[1], ve, 1e-6),
          "does not end at (pe, ve): " + describe(p0, pe, v0, acc, dec, vmax, ve) + " -> p=" +
              std::to_string(end[0]) + " v=" + std::to_string(end[1]));
}

void testMovingAwayStillReachesTheTarget() {
    std::cout << "Testing an initial velocity away from the target..." << std::endl;
    const double accs[] = {1.0, 2.0};
    const double decs[] = {1.0, 3.0};
    const double vmaxs[] = {2.0, 10.0};  // reached and not reached
    const double ves[] = {0.0, 0.5};
    const double v0s[] = {-0.5, -2.0, -4.0};
    int n = 0;
    // A short distance is what makes both roots of the old quadratic positive.
    for (double dp : {1.0, 10.0}) {
        for (double direction : {1.0, -1.0}) {
            for (double acc : accs) {
                for (double dec : decs) {
                    for (double vmax : vmaxs) {
                        for (double ve : ves) {
                            for (double v0 : v0s) {
                                checkReachesTarget(0.0, direction * dp, direction * v0, acc, dec, vmax, direction * ve);
                                ++n;
                            }
                        }
                    }
                }
            }
        }
    }
    std::cout << "✅ " << n << " trajectories starting away from the target reach it" << std::endl;
}

void testTheReportedCase() {
    std::cout << "Testing the reported case (p0=0, pe=1, v0=-2, a=d=1)..." << std::endl;
    TwoPointInterpolation tpi;
    tpi.init(0.0, 1.0, 1.0, 10.0, 0.0, -2.0, 0.0, 1.0);
    double total = tpi.calcTrajectory();
    // It slows from -2 to 0 over 2 m before turning back.
    auto turn = tpi.getPoint(2.0);
    check(almostEqual(turn[0], -2.0, 1e-9) && almostEqual(turn[1], 0.0, 1e-9), "does not turn back at -2");
    auto end = stateJustBeforeEnd(tpi, 0.0, total);
    check(almostEqual(end[0], 1.0, 1e-6) && almostEqual(end[1], 0.0, 1e-6), "does not end at the target");
    std::cout << "✅ Turns back at -2 and ends at the target" << std::endl;
}

void testAccelerationOnlyArrival() {
    std::cout << "Testing acceleration-only arrivals (ve at the peak)..." << std::endl;
    int n = 0;
    for (double direction : {1.0, -1.0}) {
        for (double v0_toward : {0.0, -1.0, 0.5}) {
            const double cases[][2] = {{15.0, 0.3}, {1.0, 2.0}, {7.3, 0.7}};
            for (const auto& c : cases) {
                const double dp = c[0], acc = c[1];
                const double v0 = v0_toward * direction;
                const double ve = direction * std::sqrt(v0_toward * v0_toward + 2 * acc * dp);
                checkReachesTarget(0.0, direction * dp, v0, acc, 0.5, 100.0, ve);
                ++n;
            }
        }
    }
    // The case from review: v1 comes out as 2.9999999999999996.
    checkReachesTarget(0.0, 15.0, 0.0, 0.3, 0.5, 5.0, 3.0);
    std::cout << "✅ " << n + 1 << " acceleration-only arrivals are accepted" << std::endl;
}

void expectError(double p0, double pe, double v0, double acc, double dec, double vmax, double ve,
                 const std::string& expected) {
    TwoPointInterpolation tpi;
    tpi.init(p0, pe, acc, vmax, 0.0, v0, ve, dec);
    try {
        tpi.calcTrajectory();
    } catch (const std::runtime_error& e) {
        check(std::string(e.what()).find(expected) != std::string::npos,
              "unexpected message for " + describe(p0, pe, v0, acc, dec, vmax, ve) + ": " + e.what());
        return;
    }
    throw std::runtime_error("expected an error for " + describe(p0, pe, v0, acc, dec, vmax, ve));
}

void testInfeasibleRequestsThrow() {
    std::cout << "Testing infeasible requests..." << std::endl;
    // Toward the target too fast to stop within 1 m: needs 2 m.
    expectError(0.0, 1.0, 2.0, 1.0, 1.0, 10.0, 0.0, "Insufficient distance to decelerate");
    // ve beyond the highest reachable speed sqrt(0 + 2 * 1 * 1) = 1.4142.
    expectError(0.0, 1.0, 0.0, 1.0, 1.0, 10.0, 2.0, "End velocity unreachable");
    expectError(0.0, 1.0, 0.0, 1.0, 1.0, 10.0, 2.0, "is 1.4142, but ve 2.0000 was requested");
    std::cout << "✅ Infeasible requests throw with their cause" << std::endl;
}

void testEveryReturnedTrajectoryIsValid() {
    std::cout << "Testing a grid of requests (every returned trajectory)..." << std::endl;
    int built = 0, refused = 0;
    for (double v0 : {-3.0, -1.0, 0.0, 0.5, 2.0}) {
        for (double dp : {-6.0, -0.7, 0.4, 3.0, 12.0}) {
            for (double acc : {0.5, 2.0}) {
                for (double dec : {0.5, 3.0}) {
                    for (double vmax : {1.5, 4.0}) {
                        for (double ve_mag : {0.0, 0.8}) {
                            for (double ve_sign : {1.0, -1.0}) {
                                // ve_sign < 0: ve points away from pe (pass it and come back)
                                const double ve = ve_sign * ve_mag * (dp > 0 ? 1.0 : -1.0);
                                TwoPointInterpolation tpi;
                                tpi.init(0.0, dp, acc, vmax, 0.0, v0, ve, dec);
                                double total;
                                try {
                                    total = tpi.calcTrajectory();
                                } catch (const std::runtime_error&) {
                                    ++refused;
                                    continue;
                                }
                                ++built;
                                const std::string where = describe(0.0, dp, v0, acc, dec, vmax, ve);
                                auto end = stateJustBeforeEnd(tpi, 0.0, total);
                                check(almostEqual(end[0], dp, 1e-6) && almostEqual(end[1], ve, 1e-6),
                                      "does not end at (pe, ve): " + where);
                                for (double dt : tpi.getDt()) {
                                    check(dt >= -1e-12, "negative phase duration: " + where);
                                }
                                const double vcap = std::max(vmax, std::fabs(v0)) + 1e-9;
                                for (int i = 0; i <= 200; ++i) {
                                    auto s = tpi.getPoint(total * i / 200.0);
                                    check(std::fabs(s[1]) <= vcap, "speed above max(vmax, |v0|): " + where);
                                    check(std::fabs(s[2]) <= std::max(acc, dec) + 1e-9,
                                          "acceleration above its limits: " + where);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    std::cout << "✅ " << built << " returned trajectories are valid (" << refused << " refused)" << std::endl;
}

void testAngleTurningAwayFromItsTarget() {
    std::cout << "Testing an angle starting to turn away from its target..." << std::endl;
    // 350 deg to 10 deg is +20 deg the short way; start turning at -1 rad/s.
    TwoAngleInterpolation tai;
    const double pe = 10.0 * M_PI / 180.0;
    tai.init(350.0 * M_PI / 180.0, pe, 1.0, 3.0, 0.0, -1.0, 0.0, 1.0);
    double total = tai.calcTrajectory(350.0 * M_PI / 180.0, pe, 1.0, 3.0, 0.0, -1.0, 0.0, 1.0);
    auto end = tai.getPoint(total - 1e-9);
    check(almostEqual(end[0], pe, 1e-6) && almostEqual(end[1], 0.0, 1e-6), "angle does not end at its target");
    std::cout << "✅ The angle reaches its target" << std::endl;
}

int main() {
    std::cout << "=== C++ Two Points Interpolation Test Suite ===" << std::endl;
    std::cout << "Testing constant acceleration interpolation with dec_max support\n" << std::endl;
    
    try {
        // Basic tests
        testConstantAccBasic();
        testConstantAccErrorHandling();
        testConstantAccZeroDisplacement();
        
        // Parameterized tests
        testCase0();
        testCase1();
        
        // Feature tests
        testFasterDeceleration();
        testDefaultDecMax();

        // Initial velocity away from the target and end-velocity feasibility
        testMovingAwayStillReachesTheTarget();
        testTheReportedCase();
        testAccelerationOnlyArrival();
        testInfeasibleRequestsThrow();
        testEveryReturnedTrajectoryIsValid();
        testAngleTurningAwayFromItsTarget();
        
        // Jerk tests (basic)
        testConstantJerkBasic();
        testConstantJerkErrorHandling();
        
        std::cout << "\n🎉 All tests passed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "\n❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
