#include "waypoint_trajectory_generator/framing/frenet_serret.hpp"
#include <iostream>

FrenetFraming::FrenetFraming(const std::vector<Vec3>& ctrlPts)
    : Framing(ctrlPts) {}

void FrenetFraming::get_basis(double s, Vec3& e1, Vec3& e2, Vec3& e3) const
{
    Vec3 dp     = dpos(s);
    Vec3 ddp    = ddpos(s);
    
    double dp_norm = dp.norm();
    if (dp_norm < 1e-10)
    {
        e1 << 1.0, 0.0, 0.0;
        e2 << 0.0, 1.0, 0.0;
        e3 << 0.0, 0.0, 1.0;
        return;
    }
    
    // Tangent
    e1 = dp / dp_norm;
    
    // Normal (perpendicular component of ddp)
    double ddp_dot_e1 = ddp.dot(e1);
    Vec3 ddp_perp;
    ddp_perp = ddp - ddp_dot_e1 * e1;
    
    double ddp_perp_norm = ddp_perp.norm();
    
    if (ddp_perp_norm < 1e-10) 
    {
        // Degenerate: create arbitrary perpendicular vector
        if (std::abs(e1.x()) < 0.9) {
            Vec3 temp;
            temp << 1.0, 0.0, 0.0;
            e2 = temp.cross(e1);
        } else {
            Vec3 temp;
            temp << 0.0, 1.0, 0.0;
            e2 = temp.cross(e1);
        }
        e2.normalize();
    } 
    else 
    {
        e2 = ddp_perp / ddp_perp_norm;
    }
    
    // Binormal
    e3 = e1.cross(e2);
}


// void FrenetFraming::cartan(double s, 
//                            double& l1, double& l2, double& l3,
//                            Vec3& de1, Vec3& de2, Vec3& de3) const
// {
//     /*  [| | | ]   [| | |] [0 -k  0]   |  | [| | | ]   [| | |] [  0 -l3  l2]
//         [T'N'B'] = [T N B] [k  0 -t]v  |or| [T'N'B'] = [T N B] [ l3   0 -l1] v, v = |dp|
//         [| | | ]   [| | |] [0  t  0]   |  | [| | | ]   [| | |] [-l2  l1   0]
//     */
//     Vec3 dp         = dpos(s);
//     Vec3 ddp        = ddpos(s);
//     Vec3 dddp       = dddpos(s);
//     double dp_norm  = dp.norm();
//     // tau * |dp|
//     l1 = dp_norm * (dp.cross(ddp)).dot(dddp)  / (std::pow(dp.cross(ddp).norm(), 2) + 1e-8); 
//     l2 = 0;
//     // kappa * |dp|
//     l3 = dp_norm * dp.cross(ddp).norm() / (std::pow(dp.norm(), 3) + 1e-8); 
    
//     Vec3 t, n, b;
//     get_basis(s, t, n, b);
//     de1 =  l3*n - l2*b;
//     de2 = -l3*t + l1*b;
//     de3 =  l2*t - l1*n;
// }

void FrenetFraming::cartan(double s, 
                           double& l1, double& l2, double& l3,
                           Vec3& de1, Vec3& de2, Vec3& de3) const
{
    /*  [| | | ]   [| | |] [0 -k  0]   |  | [| | | ]   [| | |] [  0 -l3  l2]
        [T'N'B'] = [T N B] [k  0 -t]v  |or| [T'N'B'] = [T N B] [ l3   0 -l1] v, v = |dp|
        [| | | ]   [| | |] [0  t  0]   |  | [| | | ]   [| | |] [-l2  l1   0]
    */

    Vec3 dp = dpos(s);
    Vec3 ddp = ddpos(s);
    Vec3 dddp = dddpos(s);
    double dp_norm = dp.norm();
    
    if (dp_norm < 1e-10) {
        std::cerr << "  Degenerate case: zero velocity" << std::endl;
        l1 = 0.0; l2 = 0.0; l3 = 0.0;
        de1.setZero(); de2.setZero(); de3.setZero();
        return;
    }
    
    Vec3 dp_cross_ddp = dp.cross(ddp);
    double cross_norm_sq = dp_cross_ddp.squaredNorm();
    
    if (cross_norm_sq < 1e-12) {
        std::cerr << "  Degenerate case: straight line" << std::endl;
        l1 = 0.0; l2 = 0.0; l3 = 0.0;
        de1.setZero(); de2.setZero(); de3.setZero();
        return;
    }
    
    double cross_norm = std::sqrt(cross_norm_sq);
    
    l1 = dp_norm * dp_cross_ddp.dot(dddp) / (cross_norm_sq + 1e-12);
    l2 = 0.0;
    l3 = dp_norm * cross_norm / (dp_norm * dp_norm * dp_norm + 1e-12);
    
    Vec3 t, n, b;
    get_basis(s, t, n, b);  // <--- THIS IS WHERE IT PROBABLY CRASHES
    
    de1 =  l3*n - l2*b;
    de2 = -l3*t + l1*b;
    de3 =  l2*t - l1*n;
}
