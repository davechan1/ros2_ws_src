#include "waypoint_trajectory_generator/framing/frenet_serret.hpp"
#include "waypoint_trajectory_generator/framing/gravity_normal.hpp"
#include "waypoint_trajectory_generator/backward.hpp"

#include <iostream>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <random>
// stack tracer​
namespace backward
{
    backward::SignalHandling sh;
}

int main()
{
    // Define control points for the B-spline
    std::vector<Framing::Vec3> controlPoints = {
        {0.0, 0.0, 0.0},
        {10.0, 10.0, 10.0},
        {20.0, 0.0, 0.0},
        {40.0, 40.0, 10.0}
    };

    try 
    {
        FrenetFraming framing(controlPoints);

        // Test the accuracy and timing of I2F and F2I
        constexpr int numTests = 1000; // Number of iterations
        double totalF2ITime = 0.0;
        double totalI2FTime = 0.0;
        double totalError = 0.0;

        // Random number generator for s, w1, and w2
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distS(0.0, 1.0); // s is within [0, 1]
        std::uniform_real_distribution<> distW(-10.0, 10.0); // Arbitrary range for w1 and w2

        // Eigen::Vector3d F_a(0.770475, -6.0042, -1.31396);
        // auto I_a = framing.F2I(F_a);
        // std::cout << "I_a: (" << I_a[0] << ", " << I_a[1] << ", " << I_a[2] << ")" << std::endl;
        // auto F_a_recovered = framing.I2F(I_a);

        int error_count = 0;
        for (int i = 0; i < numTests; ++i) 
        {
            // Generate random input (F_a)
            double s = distS(gen);
            double w1 = distW(gen);
            double w2 = distW(gen);

            Eigen::Vector3d F_a(s, w1, w2);

            // std::cout << "\n\nFa : (" << s << ", " << w1 << ", " << w2 << ")" << std::endl;

            // Measure F2I
            auto startF2I = std::chrono::high_resolution_clock::now();
            auto I_a = framing.F2I(F_a);
            auto endF2I = std::chrono::high_resolution_clock::now();
            totalF2ITime += std::chrono::duration<double, std::micro>(endF2I - startF2I).count();

            // Measure I2F
            auto startI2F = std::chrono::high_resolution_clock::now();
            auto F_a_recovered = framing.I2F(I_a);
            auto endI2F = std::chrono::high_resolution_clock::now();
            totalI2FTime += std::chrono::duration<double, std::micro>(endI2F - startI2F).count();
                
            // std::cout << "Far: (" << F_a_recovered[0] << ", " << F_a_recovered[1] << ", " << F_a_recovered[2] << ")" << std::endl;
            std::cout << "s error: " << std::abs(F_a[0] - F_a_recovered[0]) << std::endl;
            if (std::abs(F_a[0] - F_a_recovered[0]) > 1e-2) 
            {
                error_count++;
            }
            
            // Compute error
            double error = (F_a - F_a_recovered).norm(); // Euclidean distance
            totalError += error;
        }

        // Compute averages
        double avgF2ITime   = totalF2ITime / numTests;
        double avgI2FTime   = totalI2FTime / numTests;
        double avgError     = totalError / numTests;

        // Output results
        std::cout << "\n\nTests completed: " << numTests << "\n";
        std::cout << "Average F2I time: " << avgF2ITime << " µs\n";
        std::cout << "Average I2F time: " << avgI2FTime << " µs\n";
        std::cout << "Average error (F_a vs. F_a_recovered): " << avgError << "\n";

        std::cout << "Number of cases with s error > 1e-2: " << error_count << " out of " << numTests << "\n";
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
