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
    bool pointSetted;
    bool constraintsSetted;
    bool initialStateSetted;
    bool trajectoryCalced;

    double t0;
    double p0;
    double v0;
    double pe;
    double ve;
    double amax;
    double vmax;
    std::vector<double> dt;
    std::vector<double> a;
    std::vector<double> v;
    std::vector<double> p;
    double aSigned;
    int caseNum;

public:
    TwoPointInterpolation() {
        pointSetted = false;
        constraintsSetted = false;
        initialStateSetted = false;
        trajectoryCalced = false;
    }

    void setInitial(double t0, double p0, double v0 = 0) {
        this->t0 = t0;
        this->p0 = p0;
        this->v0 = v0;
        initialStateSetted = true;
    }

    void setPoint(double pe, double ve = 0) {
        this->pe = pe;
        this->ve = ve;
        pointSetted = true;
    }

    void setConstraints(double amax, double vmax) {
        this->amax = amax;
        this->vmax = vmax;
        constraintsSetted = true;
    }

    void init(double p0, double pe, double amax, double vmax, double t0 = 0, double v0 = 0, double ve = 0) {
        setInitial(t0, p0, v0);
        setPoint(pe, ve);
        setConstraints(amax, vmax);
    }

    double calcTrajectory() {
        if (!initialStateSetted || !pointSetted || !constraintsSetted)
        {
            std::cout << "Necessary parameters are not given." << std::endl;
            return -1;
        }

        double vmax = this->vmax;
        double v0 = this->v0;
        double p0 = this->p0;
        double t0 = this->t0;
        double ve = this->ve;
        double pe = this->pe;
        double dp = pe - p0;
        double dv = ve - v0;

        dt.clear();
        a.clear();
        v.clear();
        p.clear();

        aSigned = amax * dp / std::fabs(dp);
        double b = 2 * v0 / aSigned;
        double c = (-dv * (ve + v0) * 0.5 - dp) / aSigned;
        if (b * b - 4 * c > 0) { // not reach the vmax
            double dt01 = 0.5 * (-b + std::sqrt(b * b - 4 * c));
            double v1 = vInteg(v0, aSigned, dt01);

            if (std::fabs(v1) < vmax) {
                caseNum = 0;
                double p1 = pInteg(p0, v0, aSigned, dt01);
                double dt1e = dt01 - dv / aSigned;
                dt.push_back(dt01);
                dt.push_back(dt1e);
                a.push_back(aSigned);
                a.push_back(-aSigned);
                v.push_back(v1);
                p.push_back(p1);
            } else {
                caseNum = 1;
                double v1 = vmax * dp / std::fabs(dp);
                double dt01 = std::fabs((v1 - v0) / aSigned);
                double p1 = pInteg(p0, v0, aSigned, dt01);
                dt.push_back(dt01);
                a.push_back(aSigned);
                v.push_back(v1);
                p.push_back(p1);
                double v2 = v1;
                double dt2e = -(ve - v2) / aSigned;
                double dp2e = pInteg(0, v2, -aSigned, dt2e);
                double dt12 = (pe - p1 - dp2e) / v1;
                double p2 = pe - dp2e;
                dt.push_back(dt12);
                dt.push_back(dt2e);
                a.push_back(0.0);
                a.push_back(-aSigned);
                v.push_back(v2);
                p.push_back(p2);
            }
        } else {
            std::cout << "Failed to solve equation. Please check parameters." << std::endl;
            return -1;
        }

        std::cout << "case " << caseNum << std::endl;
        std::cout << "dt ";
        for (double t : dt) {
            std::cout << t << " ";
        }
        std::cout << std::endl;
        std::cout << "a ";
        for (double acc : a) {
            std::cout << acc << " ";
        }
        std::cout << std::endl;
        std::cout << "v ";
        for (double vel : v) {
            std::cout << vel << " ";
        }
        std::cout << std::endl;
        std::cout << "p ";
        for (double pos : p) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;

        trajectoryCalced = true;

        double total_dt = 0;
        for (double t : dt) {
            total_dt += t;
        }

        return total_dt;
    }

    std::vector<double> getPoint(double t) {
        double a = 0;
        double v = 0;
        double pos = 0;

        double tau = t - t0;

        if (tau < 0) {
            a = 0.0;
            v = v0;
            pos = p0;
        } else if (tau >= sum(dt)) {
            a = 0.0;
            v = ve;
            pos = pe;
        } else {
            double a_in = 0.0;
            double v_in = 0.0;
            double p_in = 0.0;
            double t_in = tau;
            for (int i = 0; i < dt.size(); i++) {
                double dt_i = sum(dt, i + 1);
                if (tau <= dt_i) {
                    t_in = tau - sum(dt, i);
                    a_in = a[i];
                    v_in = v[i];
                    p_in = p[i];
                    break;
                }
            }

            a = a_in;
            v = vInteg(v_in, a_in, t_in);
            pos = pInteg(p_in, v_in, a_in, t_in);
        }

        std::vector<double> result;
        result.push_back(pos);
        result.push_back(v);
        result.push_back(a);

        return result;
    }

private:
    double sum(const std::vector<double>& values, int count = -1) {
        if (count < 0 || count >= values.size()) {
            return std::accumulate(values.begin(), values.end(), 0.0);
        } else {
            return std::accumulate(values.begin(), values.begin() + count, 0.0);
        }
    }
};