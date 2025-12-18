#ifndef FRENET_FRAMING_HPP
#define FRENET_FRAMING_HPP

#include "waypoint_trajectory_generator/framing/framing.hpp"

class FrenetFraming : public Framing
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    FrenetFraming(const std::vector<Vec3>& ctrlPts);
    ~FrenetFraming() override = default;

    // Delete copy semantics (inherit from base class)
    FrenetFraming(const FrenetFraming&) = delete;
    FrenetFraming& operator=(const FrenetFraming&) = delete;

    // Allow move semantics (inherit from base class)
    FrenetFraming(FrenetFraming&&) = default;
    FrenetFraming& operator=(FrenetFraming&&) = default;

    void get_basis(double s, 
                   Vec3& e1, Vec3& e2, Vec3& e3)  const override;
    void cartan(double s, 
                double& l1, double& l2, double& l3,
                Vec3& de1, Vec3& de2, Vec3& de3)   const override;
};

#endif
