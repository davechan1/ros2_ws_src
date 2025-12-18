#ifndef GRAVITY_NORMAL_FRAMING_HPP
#define GRAVITY_NORMAL_FRAMING_HPP

#include "waypoint_trajectory_generator/framing/framing.hpp"

class GravityNormal : public Framing
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GravityNormal(const std::vector<Vec3>& ctrlPts);
    ~GravityNormal() override = default;

    // Delete copy semantics
    GravityNormal(const GravityNormal&) = delete;
    GravityNormal& operator=(const GravityNormal&) = delete;

    // Allow move semantics
    GravityNormal(GravityNormal&&) = default;
    GravityNormal& operator=(GravityNormal&&) = default;

    void get_basis(double s, 
                   Vec3& e1, Vec3& e2, Vec3& e3)  const override;
    void cartan(double s, 
                double& l1, double& l2, double& l3,
                Vec3& de1, Vec3& de2, Vec3& de3)   const override;
};

#endif
