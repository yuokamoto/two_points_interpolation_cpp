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
#include <cmath>
#include <vector>
#include <stdexcept>

/**
 * Two-point interpolation with constant jerk constraints.
 * 
 * d^3x/dt^3
 * 
 *     ^
 *     |
 *     |
 *  max|--------            ----------
 *     |       |            |
 *     |       |            |
 *  --------------------------------->
 *             | t1         |3t1
 *             |            | 
 *  min        --------------
 * 
 * 
 * * assumption v0 = 0
 * * jerk is constant
 */

class TwoPointInterpolationJerk {
private:
    bool _pointSetted;
    bool _constraintsSetted; 
    bool _initialStateSetted;
    bool _trajectoryCalced;
    bool _verbose;

    double _t0;
    double _p0;
    double _v0;
    double _ps; // start position (alternative to p0)
    double _pe;
    double _ve;
    double _amax;
    double _vmax;
    double _jmax;
    double _t1;
    double _t2;
    double _t3;
    double _te;
    int _caseNum;

public:
    TwoPointInterpolationJerk(const bool verbose = false) {
        _pointSetted = false;
        _constraintsSetted = false;
        _initialStateSetted = false;
        _trajectoryCalced = false;
        _verbose = verbose;
    }

    void setInitialTime(const double time) {
        _t0 = time;
    }

    void setInitial(const double t0, const double p0, const double v0 = 0) {
        _t0 = t0;
        _p0 = p0;
        _v0 = v0;
        _initialStateSetted = true;
    }

    void setPoint(const double ps, const double pe) {
        _ps = ps;
        _pe = pe;
        _pointSetted = true;
    }

    void setPoint(const double pe) {
        // Compatible with constant_acc API where only end point is set
        _pe = pe;
        _ve = 0;  // Default end velocity
        _pointSetted = true;
    }

    void setConstraints(const double amax, const double vmax, const double jmax) {
        if (amax <= 0 || vmax <= 0 || jmax <= 0) {
            throw std::invalid_argument("All constraint values must be positive");
        }
        _amax = amax;
        _vmax = vmax;
        _jmax = jmax;
        _constraintsSetted = true;
    }

    void setConstraints(const std::vector<double>& maxConstraints) {
        if (maxConstraints.size() != 3) {
            throw std::invalid_argument("maxConstraints must contain [vmax, amax, jmax]");
        }
        if (maxConstraints[0] <= 0 || maxConstraints[1] <= 0 || maxConstraints[2] <= 0) {
            throw std::invalid_argument("All constraint values must be positive");
        }
        _vmax = maxConstraints[0];
        _amax = maxConstraints[1];
        _jmax = maxConstraints[2];
        _constraintsSetted = true;
    }

    void set(const double ps, const double pe, const std::vector<double>& maxConstraints) {
        setPoint(ps, pe);
        setConstraints(maxConstraints);
    }

    void init(const double p0, const double pe, const double amax, const double vmax, 
              const double jmax, const double t0 = 0, const double v0 = 0, const double ve = 0) {
        setInitial(t0, p0, v0);
        setPoint(pe);
        _ve = ve;
        setConstraints(amax, vmax, jmax);
    }

    bool isInitialized() const {
        return _pointSetted && _constraintsSetted && _trajectoryCalced;
    }

    double calcTrajectory() {
        if (!_pointSetted) {
            throw std::runtime_error("End point not set. Call setPoint() first.");
        }
        if (!_constraintsSetted) {
            throw std::runtime_error("Constraints not set. Call setConstraints() first.");
        }

        // Handle case where initial state is set vs using ps
        double ps;
        if (_initialStateSetted) {
            ps = _p0;
        } else {
            ps = _ps;
        }

        // Check if start and end positions are the same
        double dp = _pe - ps;
        if (dp == 0) {
            // Handle case where no movement is needed
            if (_initialStateSetted && _ve != _v0) {
                throw std::invalid_argument("Cannot have different velocities at the same position (dp=0, but dv!=0)");
            }
            _caseNum = -1;  // Special case for no movement
            _te = 0.0;
            _t1 = 0.0;  // Initialize t1 for consistency
            _trajectoryCalced = true;
            return 0.0;
        }

        _t1 = std::pow(std::abs(dp) / 2.0 / _jmax, 1.0/3.0);
        _te = 0.0;

        if (_t1 * _jmax < _amax) { // not hit acc limit
            if (_t1 * _jmax * _t1 < _vmax) { // not hit v limit
                _caseNum = 0;
                _te = 4 * _t1;
            } else { // hit v limit
                _caseNum = 1;
                _t1 = std::sqrt(_vmax / _jmax);
                _t2 = std::abs(dp) / _vmax - 2.0 * std::sqrt(_vmax / _jmax);
                _te = 4 * _t1 + _t2;
            }
        } else { // hit acc limit
            _t1 = _amax / _jmax;
            _t2 = -1.5 * _t1 + std::sqrt(4 * std::abs(dp) / _amax + 1.0/3.0 * _t1 * _t1) / 2.0;
            if ((_t1 + _t2) * _amax < _vmax) { // not hit v limit
                _caseNum = 2;
                _te = 4 * _t1 + 2 * _t2;
            } else { // hit v limit
                _caseNum = 3;
                _t1 = _amax / _jmax;
                _t2 = _vmax / _amax - _t1;
                _t3 = std::abs(dp) / _vmax - 2 * _t1 - _t2;
                _te = 4 * _t1 + 2 * _t2 + _t3;
            }
        }

        if (_verbose) {
            std::cout << "case " << _caseNum << " " << _te << std::endl;
        }

        _trajectoryCalced = true;
        return _te;
    }

    std::vector<double> getPoint(const double t) const {
        double j = 0;
        double a = 0;
        double v = 0;
        double p = 0;

        // Handle case where initial state is set vs using ps
        double ps;
        if (_initialStateSetted) {
            ps = _p0;
        } else {
            ps = _ps;
        }

        // Handle special case where no movement is needed (dp=0)
        if (_caseNum == -1) {
            j = 0.0;
            a = 0.0;
            if (_initialStateSetted) {
                v = _v0;
            } else {
                v = 0.0;
            }
            p = ps;
            return {p, v, a, j};
        }

        // Ensure trajectory has been calculated
        if (!_trajectoryCalced) {
            throw std::runtime_error("Trajectory not calculated. Call calcTrajectory() first.");
        }

        double tau = t - _t0;
        double jmax = _jmax;
        // double amax = _amax;  // Unused
        // double vmax = _vmax;  // Unused
        double t1 = _t1;

        if (_caseNum == 0) {
            if (tau < 0) {
                j = jmax;
                if (_pe < ps) {
                    j = -j;
                }
                a = 0.0;
                v = 0.0;
                p = ps;
            } else if (tau < 4 * t1) {
                if (tau < t1) {
                    j = jmax;
                    a = jmax * tau;
                    v = 0.5 * jmax * tau * tau;
                    p = 1.0/6.0 * jmax * tau * tau * tau;
                } else if (tau < 3 * t1) {
                    double tau1 = tau - t1;
                    j = -jmax;
                    a = -jmax * tau1 + jmax * t1;
                    v = (-0.5 * jmax * tau1 * tau1 + jmax * t1 * tau1 + 
                         0.5 * jmax * t1 * t1);
                    p = (-1.0/6.0 * jmax * tau1 * tau1 * tau1 + 
                         0.5 * jmax * t1 * tau1 * tau1 + 
                         0.5 * jmax * t1 * t1 * tau1 + 
                         1.0/6.0 * jmax * t1 * t1 * t1);
                } else { // tau < 4 * t1
                    double tau3 = tau - 3 * t1;
                    j = jmax;
                    a = jmax * tau3 - jmax * t1;
                    v = (0.5 * jmax * tau3 * tau3 - jmax * t1 * tau3 + 
                         0.5 * jmax * t1 * t1);
                    p = (1.0/6.0 * jmax * tau3 * tau3 * tau3 - 
                         0.5 * jmax * t1 * tau3 * tau3 + 
                         0.5 * jmax * t1 * t1 * tau3 + 
                         11.0/6.0 * jmax * t1 * t1 * t1);
                }

                if (_pe < ps) {
                    j = -j;
                    a = -a;
                    v = -v;
                    p = -p;
                }
                p += ps;
            } else { // tau > 4 * t1
                j = jmax;
                if (_pe < ps) {
                    j = -j;
                }
                a = 0.0;
                v = 0.0;
                p = _pe;
            }
        } else {
            // For other cases (1, 2, 3) or out of range, return appropriate values
            if (tau < 0) {
                j = jmax;
                if (_pe < ps) j = -j;
                a = 0.0;
                v = (_initialStateSetted) ? _v0 : 0.0;
                p = ps;
            } else {
                // After trajectory completion
                j = 0.0;
                a = 0.0;
                v = (_initialStateSetted) ? _ve : 0.0;
                p = _pe;
            }
        }

        return {p, v, a, j};
    }
};
