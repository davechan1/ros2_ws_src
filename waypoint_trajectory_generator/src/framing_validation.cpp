#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>

#include "waypoint_trajectory_generator/backward.hpp"
#include "waypoint_trajectory_generator/framing/frenet_serret.hpp"
#include "waypoint_trajectory_generator/framing/gravity_normal.hpp"

// ANSI color codes for pretty output
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

// stack tracer​
namespace backward
{
    backward::SignalHandling sh;
}

class FramingValidator 
{
public:
    static void test_orthonormality(Framing& framing, const std::string& name) 
    {
        std::cout << BLUE << "\n=== Testing Orthonormality: " << name << " ===" << RESET << "\n";
        
        const int samples = 50;
        double max_error = 0.0;
        
        for (int i = 0; i <= samples; ++i) 
        {
            double s = static_cast<double>(i) / samples;
            Eigen::Vector3d e1, e2, e3;
            framing.get_basis(s, e1, e2, e3);
            
            // Check unit length
            double len1 = e1.norm();
            double len2 = e2.norm();
            double len3 = e3.norm();
            
            // Check orthogonality
            double dot12 = e1.dot(e2);
            double dot13 = e1.dot(e3);
            double dot23 = e2.dot(e3);
            
            // Check right-handedness: e3 = e1 × e2
            Eigen::Vector3d cross_check = e1.cross(e2);
            double cross_error = (cross_check - e3).norm();
            
            double error = std::max({
                std::abs(len1 - 1.0), std::abs(len2 - 1.0), std::abs(len3 - 1.0),   //length should be 1
                std::abs(dot12), std::abs(dot13), std::abs(dot23),                  //dot product should be 0
                cross_error                                                         //cross product should be e3
            });
            
            max_error = std::max(max_error, error);
            
            if (error > 1e-6) 
            {
                std::cout << RED << "  FAIL at s=" << s << ": max_error=" << error << RESET << "\n";
                std::cout << "    |e1|=" << len1 << ", |e2|=" << len2 << ", |e3|=" << len3 << "\n";
                std::cout << "    e1·e2=" << dot12 << ", e1·e3=" << dot13 << ", e2·e3=" << dot23 << "\n";
            }
        }
        
        if (max_error < 1e-6)
            std::cout << GREEN << "  ✓ PASS: max error = " << max_error << RESET << "\n";
        else 
            std::cout << YELLOW << "  ⚠ WARNING: max error = " << max_error << RESET << "\n";
    }
    
    static void test_cartan_consistency(Framing& framing, const std::string& name)
    {
        std::cout << BLUE << "\n=== Testing Cartan against numerical derivatives: " << name << " ===" << RESET << "\n";
        
        const int samples = 50;
        const double ds = 1e-5; // Small step for finite difference
        double max_error = 0.0;
        
        for (int i = 1; i < samples; ++i)  // Skip endpoints
        {
            double s = static_cast<double>(i) / samples;
            
            // Get basis at s
            Eigen::Vector3d e1_s, e2_s, e3_s;
            framing.get_basis(s, e1_s, e2_s, e3_s);
            
            // Get basis at s + ds
            Eigen::Vector3d e1_p, e2_p, e3_p;
            framing.get_basis(s + ds, e1_p, e2_p, e3_p);
            
            // Numerical derivatives
            Eigen::Vector3d de1_numerical = (e1_p - e1_s) / ds;
            Eigen::Vector3d de2_numerical = (e2_p - e2_s) / ds;
            Eigen::Vector3d de3_numerical = (e3_p - e3_s) / ds;
            
            // Analytical derivatives from Cartan
            double l1, l2, l3;
            Eigen::Vector3d de1_cartan, de2_cartan, de3_cartan;
            framing.cartan(s, l1, l2, l3, de1_cartan, de2_cartan, de3_cartan);
            
            // Compare
            double error1 = (de1_numerical - de1_cartan).norm();
            double error2 = (de2_numerical - de2_cartan).norm();
            double error3 = (de3_numerical - de3_cartan).norm();
            
            double error    = std::max({error1, error2, error3});
            max_error       = std::max(max_error, error);
            
            if (error > 1e-3)  // Looser tolerance for finite difference
            {
                std::cout << RED << "  FAIL at s=" << s << ": error=" << error << RESET << "\n";
                std::cout << "    de1 error: " << error1 << "\n";
                std::cout << "    de2 error: " << error2 << "\n";
                std::cout << "    de3 error: " << error3 << "\n";
                std::cout << "    l1=" << l1 << ", l2=" << l2 << ", l3=" << l3 << "\n";
            }
        }
        
        if (max_error < 1e-3) 
            std::cout << GREEN << "  ✓ PASS: max error = " << max_error << RESET << "\n";
        else 
            std::cout << YELLOW << "  ⚠ WARNING: max error = " << max_error << RESET << "\n";
    }
    
    static void test_F2I_I2F_roundtrip(Framing& framing, const std::string& name, int num_tests = 1000) 
    {
        std::cout << BLUE << "\n=== Testing F2I ↔ I2F Round-trip: " << name << " ===" << RESET << "\n";
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distS(0.0, 1.0);
        std::uniform_real_distribution<> distW(-5.0, 5.0);
        
        int fail_count      = 0;
        double max_error    = 0.0;
        double avg_error    = 0.0;
        
        for (int i = 0; i < num_tests; ++i) 
        {
            // Add progress tracking
            if (i % 100 == 0) {
                std::cout << "  Progress: " << i << "/" << num_tests << std::flush << "\r";
            }
            
            try {
                Eigen::Vector3d F_a(distS(gen), distW(gen), distW(gen));
                
                auto I_a            = framing.F2I(F_a);
                auto F_a_recovered  = framing.I2F(I_a);
                
                double error = (F_a - F_a_recovered).norm();
                avg_error += error;
                max_error = std::max(max_error, error);
                
                if (error > 1e-2) 
                {
                    fail_count++;
                    if (fail_count <= 5)
                    {
                        std::cout << RED << "  FAIL: error=" << error << RESET << "\n";
                        std::cout << "    F_a  : " << F_a.transpose() << "\n";
                        std::cout << "    F_a' : " << F_a_recovered.transpose() << "\n";
                    }
                }
            } catch (const std::exception& e) {
                std::cout << RED << "\n  Exception at iteration " << i << ": " << e.what() << RESET << "\n";
                throw;
            }
        }
        
        avg_error /= num_tests;
        
        std::cout << "\n  Tests: " << num_tests << "\n";
        std::cout << "  Failures (error > 1e-2): " << fail_count << "\n";
        std::cout << "  Average error: " << avg_error << "\n";
        std::cout << "  Max error: " << max_error << "\n";
        
        if (fail_count == 0) 
            std::cout << GREEN << "  ✓ PASS" << RESET << "\n";
        else 
            std::cout << RED << "  ✗ FAIL: " << fail_count << "/" << num_tests << RESET << "\n";
    }

    
    static void test_frenet_specific(FrenetFraming& framing) 
    {
        std::cout << BLUE << "\n=== Testing Frenet-Specific Properties ===" << RESET << "\n";
        
        const int samples = 50;
        double max_l2_error = 0.0;
        
        for (int i = 0; i <= samples; ++i) 
        {
            double s = static_cast<double>(i) / samples;
            
            double l1, l2, l3;
            Eigen::Vector3d de1, de2, de3;
            framing.cartan(s, l1, l2, l3, de1, de2, de3);
            
            // For Frenet frame, l2 should be exactly 0
            max_l2_error = std::max(max_l2_error, std::abs(l2));
        }
        
        std::cout << "  l2 (should be 0): max = " << max_l2_error << "\n";
        
        if (max_l2_error < 1e-10) 
            std::cout << GREEN << "  ✓ PASS: l2 ≈ 0" << RESET << "\n";
        else 
            std::cout << RED << "  ✗ FAIL: l2 ≠ 0" << RESET << "\n";
    }
    
    static void test_gravity_specific(GravityNormal& framing) 
    {
        std::cout << BLUE << "\n=== Testing Gravity-Normal Specific Properties ===" << RESET << "\n";
        
        Eigen::Vector3d k(0, 0, 1);  // Gravity direction
        const int samples   = 50;
        double max_error    = 0.0;
        
        for (int i = 0; i <= samples; ++i) 
        {
            double s = static_cast<double>(i) / samples;
            
            Eigen::Vector3d e1, e2, e3;
            framing.get_basis(s, e1, e2, e3);
            
            // Check: e2 should be perpendicular to both k and e1
            double k_dot_e2 = std::abs(k.dot(e2));
            
            // Check: e2 should be in direction of k × e1
            Eigen::Vector3d expected_e2 = k.cross(e1).normalized();
            double e2_error = (e2 - expected_e2).norm();
            
            double error = std::max(k_dot_e2, e2_error);
            max_error = std::max(max_error, error);
            
            if (error > 1e-6) 
            {
                std::cout << RED << "  FAIL at s=" << s << ": error=" << error << RESET << "\n";
                std::cout << "    k·e2 = " << k_dot_e2 << " (should be ~0)\n";
                std::cout << "    |e2 - k×e1| = " << e2_error << "\n";
            }
        }
        
        if (max_error < 1e-6) 
            std::cout << GREEN << "  ✓ PASS: max error = " << max_error << RESET << "\n";
        else 
            std::cout << RED << "  ✗ FAIL: max error = " << max_error << RESET << "\n";
    }
};

int main() 
{
    std::vector<Framing::Vec3> controlPoints = {
        {0.0, 0.0, 0.0},
        {10.0, 10.0, 10.0},
        {20.0, 20.0, 0.0},
        {40.0, 40.0, 10.0} };
    
    try 
    {
        std::cout << std::fixed << std::setprecision(6);
        
        // Test Gravity-Normal
        GravityNormal gravity(controlPoints);
        FramingValidator::test_orthonormality(gravity, "Gravity-Normal");
        FramingValidator::test_cartan_consistency(gravity, "Gravity-Normal");
        FramingValidator::test_F2I_I2F_roundtrip(gravity, "Gravity-Normal");
        FramingValidator::test_gravity_specific(gravity);

        std::cout << BLUE << "\n============================================================================================" << RESET << "\n";


        // Test Frenet-Serret
        FrenetFraming frenet(controlPoints);
        FramingValidator::test_orthonormality(frenet, "Frenet-Serret");
        FramingValidator::test_cartan_consistency(frenet, "Frenet-Serret");
        FramingValidator::test_F2I_I2F_roundtrip(frenet, "Frenet-Serret");
        FramingValidator::test_frenet_specific(frenet);
        
        
        
        std::cout << "\n" << GREEN << "=== All tests completed ===" << RESET << "\n";
        
    } 
    catch (const std::exception& e) 
    {
        std::cerr << RED << "Exception: " << e.what() << RESET << "\n";
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}
