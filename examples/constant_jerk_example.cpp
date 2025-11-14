#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <algorithm> 

#include <yaml-cpp/yaml.h>

#include "../include/two_point_interpolation/constant_jerk.hpp"

// Function to generate a Gnuplot script for jerk interpolation
void generateGnuplotScript(const std::vector<double>& v1, const std::vector<double>& v2,
                          const std::vector<double>& v3, const std::vector<double>& v4,
                          const std::string& dataFilePath, const std::string& scriptFilePath) {
    std::ofstream scriptFile(scriptFilePath);
    scriptFile << "set terminal png\n";
    scriptFile << "set output 'graph_jerk.png'\n";
    scriptFile << "set grid\n";
    scriptFile << "set multiplot layout 4,1\n";
    
    // Helper function to get safe plot range
    auto getSafeRange = [](const std::vector<double>& data) -> std::pair<double, double> {
        double minVal = *std::min_element(data.begin(), data.end());
        double maxVal = *std::max_element(data.begin(), data.end());
        double range = maxVal - minVal;
        if (range < 1e-10) {
            return {minVal - 1, maxVal + 1};
        }
        return {minVal - 0.1 * range, maxVal + 0.1 * range};
    };

    // Jerk plot
    auto jerkRange = getSafeRange(v1);
    scriptFile << "set yrange [" << jerkRange.first << ":" << jerkRange.second << "]\n";
    scriptFile << "plot '" << dataFilePath << "' using 1:2 with lines title 'jerk[m/s^3]'\n";
    
    // Acceleration plot
    auto accRange = getSafeRange(v2);
    scriptFile << "set yrange [" << accRange.first << ":" << accRange.second << "]\n";
    scriptFile << "plot '" << dataFilePath << "' using 1:3 with lines title 'acc[m/s^2]'\n";
    
    // Velocity plot
    auto velRange = getSafeRange(v3);
    scriptFile << "set yrange [" << velRange.first << ":" << velRange.second << "]\n";
    scriptFile << "plot '" << dataFilePath << "' using 1:4 with lines title 'vel[m/s]'\n";
    
    // Position plot
    auto posRange = getSafeRange(v4);
    scriptFile << "set yrange [" << posRange.first << ":" << posRange.second << "]\n";
    scriptFile << "plot '" << dataFilePath << "' using 1:5 with lines title 'pos[m]'\n";
    
    scriptFile << "unset multiplot\n";
    scriptFile.close();
}

// Function to save vector data to a file (5 columns for jerk)
void saveVectorDataToFile(const std::vector<double>& v1, const std::vector<double>& v2,
                          const std::vector<double>& v3, const std::vector<double>& v4, 
                          const std::vector<double>& v5,
                          const std::string& filePath) {
    std::ofstream outFile(filePath);
    outFile << std::fixed << std::setprecision(6);

    for (std::size_t i = 0; i < v1.size(); ++i) {
        outFile << v1[i] << " " << v2[i] << " " << v3[i] << " " << v4[i] << " " << v5[i] << "\n";
    }

    outFile.close();
}

// Function to load constraints from a YAML file for jerk interpolation
bool loadConstraintsFromYaml(const std::string& filePath, double& ps, double& pe,
                             double& v0, double& ve, double& amax, double& vmax, double& jmax,
                             double& t0, double& dt, bool& verbose) {
    try {
        YAML::Node config = YAML::LoadFile(filePath);

        ps = config["ps"] ? config["ps"].as<double>() : config["p0"].as<double>();
        pe = config["pe"].as<double>();
        v0 = config["v0"].as<double>();
        ve = config["ve"].as<double>();
        amax = config["amax"].as<double>();
        vmax = config["vmax"].as<double>();
        jmax = config["jmax"] ? config["jmax"].as<double>() : 1.0; // Default jmax
        t0 = config["t0"].as<double>();
        dt = config["dt"].as<double>();
        verbose = config["verbose"].as<bool>();

        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        return false;
    }
}

int main() {
    std::cout << "=== Two Points Interpolation with Constant Jerk Example ===" << std::endl;

    // Load constraints from YAML file
    double ps, pe, v0, ve, amax, vmax, jmax, t0, dt;
    bool verbose;

    if (!loadConstraintsFromYaml("constraints_jerk.yaml", ps, pe, v0, ve, amax, vmax, jmax, t0, dt, verbose)) {
        std::cerr << "Failed to load constraints from YAML file. Using default values." << std::endl;
        
        // Default values
        ps = 5.5;
        pe = 100.0;
        v0 = 0.0;
        ve = 0.0;
        amax = 1.0;
        vmax = 5.0;
        jmax = 0.98;
        t0 = 0.5;
        dt = 0.001;
        verbose = true;
    }

    std::cout << "Parameters:" << std::endl;
    std::cout << "  ps = " << ps << ", pe = " << pe << std::endl;
    std::cout << "  v0 = " << v0 << ", ve = " << ve << std::endl;
    std::cout << "  amax = " << amax << ", vmax = " << vmax << ", jmax = " << jmax << std::endl;
    std::cout << "  t0 = " << t0 << ", dt = " << dt << std::endl;

    try {
        // Create interpolator
        TwoPointInterpolationJerk interp(verbose);

        // Method 1: Using new init API (compatible with constant_acc)
        interp.init(ps, pe, amax, vmax, jmax, t0, v0, ve);

        // Alternative method 2: Using original API
        // std::vector<double> maxConstraints = {vmax, amax, jmax};
        // interp.set(ps, pe, maxConstraints);
        // interp.setInitialTime(t0);

        // Calculate trajectory
        double te = interp.calcTrajectory();
        std::cout << "Total trajectory time: " << te << " seconds" << std::endl;

        if (te <= 0) {
            std::cerr << "Error: Invalid trajectory time" << std::endl;
            return 1;
        }

        // Generate trajectory data
        std::vector<double> tref;
        std::vector<double> jerk_data;
        std::vector<double> acc_data;
        std::vector<double> vel_data;
        std::vector<double> pos_data;

        for (double t = t0; t <= t0 + te; t += dt) {
            auto result = interp.getPoint(t);
            
            tref.push_back(t);
            pos_data.push_back(result[0]);  // position
            vel_data.push_back(result[1]);  // velocity
            acc_data.push_back(result[2]);  // acceleration
            jerk_data.push_back(result[3]); // jerk
        }

        std::cout << "Generated " << tref.size() << " data points" << std::endl;

        // Save data to file
        std::string dataFilePath = "data_jerk.txt";
        saveVectorDataToFile(tref, jerk_data, acc_data, vel_data, pos_data, dataFilePath);
        std::cout << "Data saved to " << dataFilePath << std::endl;

        // Generate gnuplot script
        std::string scriptFilePath = "plot_jerk.gnu";
        generateGnuplotScript(jerk_data, acc_data, vel_data, pos_data, dataFilePath, scriptFilePath);
        std::cout << "Gnuplot script saved to " << scriptFilePath << std::endl;

        // Execute gnuplot if available
        int result = std::system("which gnuplot > /dev/null 2>&1");
        if (result == 0) {
            std::cout << "Generating plot..." << std::endl;
            std::string command = "gnuplot " + scriptFilePath;
            int plotResult = std::system(command.c_str());
            if (plotResult == 0) {
                std::cout << "Plot saved to graph_jerk.png" << std::endl;
            } else {
                std::cerr << "Warning: Failed to generate plot" << std::endl;
            }
        } else {
            std::cout << "gnuplot not found. Install gnuplot to generate plots automatically." << std::endl;
            std::cout << "You can run: gnuplot " << scriptFilePath << std::endl;
        }

        // Print trajectory summary
        std::cout << "\n=== Trajectory Summary ===" << std::endl;
        std::cout << "Start: pos=" << pos_data.front() << ", vel=" << vel_data.front() << std::endl;
        std::cout << "End:   pos=" << pos_data.back() << ", vel=" << vel_data.back() << std::endl;
        std::cout << "Max jerk: " << *std::max_element(jerk_data.begin(), jerk_data.end()) << std::endl;
        std::cout << "Max acc:  " << *std::max_element(acc_data.begin(), acc_data.end()) << std::endl;
        std::cout << "Max vel:  " << *std::max_element(vel_data.begin(), vel_data.end()) << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n✅ Jerk interpolation example completed successfully!" << std::endl;
    return 0;
}
