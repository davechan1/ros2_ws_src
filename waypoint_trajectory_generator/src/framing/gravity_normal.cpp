#include "waypoint_trajectory_generator/framing/gravity_normal.hpp"

GravityNormal::GravityNormal(const std::vector<Vec3>& ctrlPts)
    : Framing(ctrlPts) {}

void GravityNormal::get_basis(double s, 
                              Vec3& e1, Vec3& e2, Vec3& e3) const
{
    Vec3 dp = dpos(s);
    Vec3 k(0, 0, 1);

    e1   = dpos(s).normalized(); 
    e2   = k.cross(dp).normalized(); 
    e3   = e1.cross(e2).normalized();
}

void GravityNormal::cartan(double s, 
                           double& l1, double& l2, double& l3,
                           Vec3& de1, Vec3& de2, Vec3& de3) const
{
    /*  [| | | ]   [| | |] [  0 -l3  l2]
        [T'H'P'] = [T H P] [ l3   0 -l1] , this time no v = |a_prime| as from first principle
        [| | | ]   [| | |] [-l2  l1   0]
    */

    Vec3 k(0, 0, 1);
    Vec3 dp     = dpos(s);
    Vec3 ddp    = ddpos(s);
    
    Vec3 k_cross_dp     = k.cross(dp);
    Vec3 dp_cross_ddp   = dp.cross(ddp);
    
    double norm_dp              = dp.norm();
    double norm_k_cross_dp      = k_cross_dp.norm();
    double norm_k_cross_dp_sq   = k_cross_dp.squaredNorm();
    double norm_dp_sq           = dp.squaredNorm();
    
    // Cartan connection coefficients
    l1 = (k.dot(dp) * k.dot(dp_cross_ddp)) / (norm_dp * norm_k_cross_dp_sq + 1e-8);
    l2 = k_cross_dp.dot(dp_cross_ddp) / (norm_dp_sq * norm_k_cross_dp + 1e-8);
    l3 = k.dot(dp_cross_ddp) / (norm_dp * norm_k_cross_dp + 1e-8);
    
    Vec3 t, h, p;
    get_basis(s, t, h, p);
    de1 =  l3*h - l2*p;  // t_prime
    de2 = -l3*t + l1*p;  // h_prime
    de3 =  l2*t - l1*h;  // p_prime
}