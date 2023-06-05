#include <iostream>
#include <cmath>
#include <vector>

double vInteg(double v0, double a, double dt) {
    return v0 + a * dt;
}

double pInteg(double p0, double v0, double a, double dt) {
    return p0 + v0 * dt + 0.5 * a * dt * dt;
}

class TwoPointInterpolation {
private:
    bool _pointSetted;
    bool _constraintsSetted;
    bool _initialStateSetted;
    bool _trajectoryCalced;
    bool _verbose;

    double _t0;
    double _p0;
    double _v0;
    double _pe;
    double _ve;
    double _amax;
    double _vmax;
    std::vector<double> _dt;
    std::vector<double> _a;
    std::vector<double> _v;
    std::vector<double> _p;
    double _aSigned;
    int _caseNum;

public:
    TwoPointInterpolation(bool verbose = false) {
        _pointSetted = false;
        _constraintsSetted = false;
        _initialStateSetted = false;
        _trajectoryCalced = false;
        _verbose = verbose;
    }

    void setInitial(double t0, double p0, double v0 = 0) {
        _t0 = t0;
        _p0 = p0;
        _v0 = v0;
        _initialStateSetted = true;
    }

    void setPoint(double pe, double ve = 0) {
        _pe = pe;
        _ve = ve;
        _pointSetted = true;
    }

    void setConstraints(double amax, double vmax) {
        _amax = amax;
        _vmax = vmax;
        _constraintsSetted = true;
    }

    void init(double p0, double pe, double amax, double vmax, double t0 = 0, double v0 = 0, double ve = 0) {
        setInitial(t0, p0, v0);
        setPoint(pe, ve);
        setConstraints(amax, vmax);
    }

    double calcTrajectory() {
        double dp = _pe - _p0;
        double dv = _ve - _v0;

        _dt.clear();
        _a.clear();
        _v.clear();
        _p.clear();

        _v.push_back(_v0);
        _p.push_back(_p0);

        _aSigned = _amax * dp / std::fabs(dp);
        double b = 2 * _v0 / _aSigned;
        double c = (-dv * (_ve + _v0) * 0.5 - dp) / _aSigned;
        if (b * b - 4 * c > 0) { // not reach the vmax
            double dt01 = 0.5 * (-b + std::sqrt(b * b - 4 * c));
            double v1 = vInteg(_v0, _aSigned, dt01);

            if (std::fabs(v1) < _vmax) {
                _caseNum = 0;
                double p1 = pInteg(_p0, _v0, _aSigned, dt01);
                double dt1e = dt01 - dv / _aSigned;
                _dt.push_back(dt01);
                _dt.push_back(dt1e);
                _a.push_back(_aSigned);
                _a.push_back(-_aSigned);
                _v.push_back(v1);
                _p.push_back(p1);
            } else {
                _caseNum = 1;
                double v1 = _vmax * dp / std::fabs(dp);
                double dt01 = std::fabs((v1 - _v0) / _aSigned);
                double p1 = pInteg(_p0, _v0, _aSigned, dt01);
                _dt.push_back(dt01);
                _a.push_back(_aSigned);
                _v.push_back(v1);
                _p.push_back(p1);
                double v2 = v1;
                double dt2e = -(_ve - v2) / _aSigned;
                double dp2e = pInteg(0, v2, -_aSigned, dt2e);
                double dt12 = (_pe - p1 - dp2e) / v1;
                double p2 = _pe - dp2e;
                _dt.push_back(dt12);
                _dt.push_back(dt2e);
                _a.push_back(0.0);
                _a.push_back(-_aSigned);
                _v.push_back(v2);
                _p.push_back(p2);
            }
        } else {
            std::cout << "error" << std::endl;
            return -1;
        }

        if (_verbose) {
            std::cout << "case " << _caseNum << std::endl;
            std::cout << "dt ";
            for (double t : _dt) {
                std::cout << t << " ";
            }
            std::cout << std::endl;
            std::cout << "a ";
            for (double acc : _a) {
                std::cout << acc << " ";
            }
            std::cout << std::endl;
            std::cout << "v ";
            for (double vel : _v) {
                std::cout << vel << " ";
            }
            std::cout << std::endl;
            std::cout << "p ";
            for (double pos : _p) {
                std::cout << pos << " ";
            }
            std::cout << std::endl;
        }

        _trajectoryCalced = true;

        double totalDt = 0;
        for (double t : _dt) {
            totalDt += t;
        }

        return totalDt;
    }

    std::vector<double> getPoint(double t) {
        double a = 0;
        double v = 0;
        double pos = 0;

        double tau = t - _t0;

        if (tau < 0) {
            a = 0.0;
            v = _v0;
            pos = _p0;
        } else if (tau >= sum(_dt, _dt.size() + 1)) {
            a = 0.0;
            v = _ve;
            pos = _pe;
        } else {
            double a_in = 0.0;
            double v_in = 0.0;
            double p_in = 0.0;
            double t_in = tau;
            for (int i = 0; i < _dt.size(); i++) {
                double dt_i = sum(_dt, i + 1);
                if (tau <= dt_i) {
                    t_in = tau - sum(_dt, i);
                    a_in = _a[i];
                    v_in = _v[i];
                    p_in = _p[i];
                    break;
                }
            }

            a = a_in;
            v = vInteg(v_in, a_in, t_in);
            pos = pInteg(p_in, v_in, a_in, t_in);
        }

        std::vector<double> result = {pos, v, a};
        return result;
    }

private:
    double sum(const std::vector<double>& values, int count) {
        double total = 0.0;
        for (int i = 0; i < count && i < values.size(); ++i) {
            total += values[i];
        }
        return total;
    }
};
