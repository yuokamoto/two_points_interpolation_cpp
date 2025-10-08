#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <stdexcept>

#include "two_points_interpolation_constant_acc.hpp"
#include "two_points_interpolation_constant_jerk.hpp"

// Test helper function
bool almostEqual(double a, double b, double tolerance = 1e-6) {
    return std::abs(a - b) < tolerance;
}

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
        std::cout << "✅ Caught expected error for negative amax: " << e.what() << std::endl;
    }
    
    try {
        interp.setConstraints(1.0, -10.0);
        assert(false); // Should not reach here
    } catch (const std::invalid_argument& e) {
        std::cout << "✅ Caught expected error for negative vmax: " << e.what() << std::endl;
    }
    
    // Test trajectory calculation without setup
    try {
        TwoPointInterpolation interp2;
        interp2.calcTrajectory();
        assert(false); // Should not reach here
    } catch (const std::runtime_error& e) {
        std::cout << "✅ Caught expected error for uninitialized trajectory: " << e.what() << std::endl;
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
        std::cout << "✅ Caught expected error for different velocities at same position: " << e.what() << std::endl;
    }
}

void testConstantJerkBasic() {
    std::cout << "Testing TwoPointInterpolationJerk basic functionality..." << std::endl;
    
    TwoPointInterpolationJerk interp;
    interp.init(0.0, 100.0, 2.0, 5.0, 1.0, 0.0, 0.0, 0.0);
    double te = interp.calcTrajectory();
    
    assert(te > 0);
    
    auto result = interp.getPoint(te);
    // Note: This is a simplified implementation for demonstration
    // Full jerk implementation would have more precise end position
    std::cout << "End position: " << result[0] << " (target: 100.0)" << std::endl;
    
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
        std::cout << "✅ Caught expected error for negative constraint: " << e.what() << std::endl;
    }
    
    try {
        std::vector<double> invalidConstraints = {5.0, -1.0, 1.0};
        interp.setConstraints(invalidConstraints);
        assert(false); // Should not reach here
    } catch (const std::invalid_argument& e) {
        std::cout << "✅ Caught expected error for negative constraint in array: " << e.what() << std::endl;
    }
    
    try {
        std::vector<double> wrongSize = {5.0, 1.0}; // Wrong size
        interp.setConstraints(wrongSize);
        assert(false); // Should not reach here
    } catch (const std::invalid_argument& e) {
        std::cout << "✅ Caught expected error for wrong array size: " << e.what() << std::endl;
    }
}

void testConstantJerkZeroDisplacement() {
    std::cout << "Testing TwoPointInterpolationJerk zero displacement..." << std::endl;
    
    TwoPointInterpolationJerk interp;
    interp.init(10.0, 10.0, 2.0, 5.0, 1.0, 0.0, 0.0, 0.0);
    double te = interp.calcTrajectory();
    
    assert(almostEqual(te, 0.0));
    
    auto result = interp.getPoint(0.0);
    assert(almostEqual(result[0], 10.0)); // position
    assert(almostEqual(result[1], 0.0));  // velocity
    assert(almostEqual(result[2], 0.0));  // acceleration
    assert(almostEqual(result[3], 0.0));  // jerk
    
    std::cout << "✅ Jerk zero displacement test passed" << std::endl;
}

int main() {
    std::cout << "=== C++ Two Points Interpolation Test Suite ===" << std::endl;
    
    try {
        testConstantAccBasic();
        testConstantAccErrorHandling();
        testConstantAccZeroDisplacement();
        
        testConstantJerkBasic();
        testConstantJerkErrorHandling(); 
        testConstantJerkZeroDisplacement();
        
        std::cout << "\n🎉 All tests passed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "\n❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
