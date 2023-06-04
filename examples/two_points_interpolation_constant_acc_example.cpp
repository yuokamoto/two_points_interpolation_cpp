#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cstdlib>

#include <yaml-cpp/yaml.h>

#include "../two_points_interpolation_constant_acc.hpp"

// Function to generate a Gnuplot script
void generateGnuplotScript(const std::string& dataFilePath, const std::string& scriptFilePath) {
    std::ofstream scriptFile(scriptFilePath);
    scriptFile << "set terminal png\n";
    scriptFile << "set output 'graph.png'\n";
    scriptFile << "set multiplot layout 4,1\n";
    scriptFile << "plot '" << dataFilePath << "' using 1:2 with lines title 'acc[m/s^2]'\n";
    scriptFile << "plot '" << dataFilePath << "' using 1:3 with lines title 'vel[m/s]'\n";
    scriptFile << "plot '" << dataFilePath << "' using 1:4 with lines title 'pos[m]'\n";
    scriptFile << "unset multiplot\n";
    scriptFile.close();
}

// Function to save vector data to a file
void saveVectorDataToFile(const std::vector<double>& vector1, const std::vector<double>& vector2,
                          const std::vector<double>& vector3, const std::string& filePath) {
    std::ofstream outFile(filePath);
    outFile << std::fixed << std::setprecision(6);

    for (std::size_t i = 0; i < vector1.size(); ++i) {
        outFile << i << " " << vector1[i] << " " << vector2[i] << " " << vector3[i] << "\n";
    }

    outFile.close();
}

// Function to load constraints from a YAML file
bool loadConstraintsFromYaml(const std::string& filePath, double& p0, double& pe,
                             double& v0, double& ve, double& amax, double& vmax,
                             double& t0, double& dt, double& te) {
    try {
        YAML::Node config = YAML::LoadFile(filePath);

        p0 = config["p0"].as<double>();
        pe = config["pe"].as<double>();
        v0 = config["v0"].as<double>();
        ve = config["ve"].as<double>();
        amax = config["amax"].as<double>();
        vmax = config["vmax"].as<double>();
        t0 = config["t0"].as<double>();
        dt = config["dt"].as<double>();
        te = config["te"].as<double>();

        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load constraints from YAML: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: ./program <filename.yaml>" << std::endl;
        return 1;
    }

    const std::string filename = argv[1];
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Failed to open file: " << filename << std::endl;
        return 1;
    }

    // Load constraints from YAML
    double p0, pe, v0, ve, amax, vmax, t0, dt, te;
    if (!loadConstraintsFromYaml(filename, p0, pe, v0, ve, amax, vmax, t0, dt, te)) {
        return 1;
    }

    // Generate trajectory


    // Simulation condition
    std::vector<double> tref;
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> acc;

    for (double t = t0; t < t0 + te; t += dt) {
        std::cout << t << std::endl;
        tref.push_back(t);
        pos.push_back(0.0);
        vel.push_back(0.0);
        acc.push_back(0.0);
    }

    // Calculate pos, vel, and acc
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < tref.size(); ++j) {
            // todo update
            // pos[j][i], vel[j][i], acc[j][i] = interp.get_point(tref[j]);
        }
    }

    // Save data to a file
    std::string dataFilePath = "data.txt";
    saveVectorDataToFile(acc, vel, pos, dataFilePath);

    // Generate Gnuplot script
    std::string scriptFilePath = "script.gnu";
    generateGnuplotScript(dataFilePath, scriptFilePath);

    // Execute Gnuplot script
    std::string command = "gnuplot " + scriptFilePath;
    std::system(command.c_str());

    return 0;
}
