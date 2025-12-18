#include "waypoint_trajectory_generator/framing/framing.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

Framing::Framing(const std::vector<Vec3>& ctrlPts)
    : controlPoints_()
{
    // copy into aligned vector to avoid allocator/copy-constructor mismatch
    controlPoints_.assign(ctrlPts.begin(), ctrlPts.end());

    if (controlPoints_.size() < 4)
        throw std::invalid_argument("Need at least 4 control points.");

    std::vector<double> flattenedControlPoints;
    flattenedControlPoints.reserve(controlPoints_.size()*3);
    for (const auto& pt : controlPoints_)
    {
        flattenedControlPoints.push_back(pt.x());
        flattenedControlPoints.push_back(pt.y());
        flattenedControlPoints.push_back(pt.z());
    }

    int dim             = 3;
    int numCtrlPoints   = static_cast<int>(controlPoints_.size());
    
    // Prefer a cubic B-spline; ensure degree < numCtrlPoints
    int deg = (3 >= numCtrlPoints) ? numCtrlPoints-1 : 3;

    std::cerr << "[Framing] numCtrlPoints=" << numCtrlPoints << " degree=" << deg << std::endl;

    sp_     = std::make_unique<tinyspline::BSpline>(numCtrlPoints, dim, deg);
    sp_->setControlPoints(flattenedControlPoints);
    d_sp_   = std::make_unique<tinyspline::BSpline>(sp_->derive());
    dd_sp_  = std::make_unique<tinyspline::BSpline>(d_sp_->derive());
    
    // NOTE: We do NOT initialize ddd_sp_ here.
    // A degree 3 spline derived 3 times becomes degree 0.
    // Evaluating degree 0 splines in some versions of tinyspline causes heap corruption.
    // We will calculate dddpos using finite differences of ddpos instead.
    // ddd_sp_ = std::make_unique<tinyspline::BSpline>(dd_sp_->derive());

    std::cout << "Arc Length: " << arclength() << std::endl;
}

double Framing::arclength() const 
{
    const int samples   = 200;
    double length       = 0.0;
    Vec3 prev           = pos(0.0);
    for (int i = 1; i <= samples; ++i) 
    {
        double u    = static_cast<double>(i) / samples;
        Vec3 cur    = pos(u);
        length      += (cur - prev).norm();
        prev        = cur;
    }
    return length;
}

Framing::Vec3 Framing::pos(double s) const 
{
    if (!sp_) return Vec3::Zero();
    auto eval_result = sp_->eval(s);
    std::vector<tinyspline::real> result = eval_result.result();
    
    if (result.size() < 3) return Vec3::Zero();

    return Vec3(result[0], result[1], result[2]);
}

Framing::Vec3 Framing::dpos(double s) const 
{
    if (!d_sp_) return Vec3::Zero();
    auto eval_result = d_sp_->eval(s);
    std::vector<tinyspline::real> result = eval_result.result();
    
    if (result.size() < 3) return Vec3::Zero();

    return Vec3(result[0], result[1], result[2]);
}

Framing::Vec3 Framing::ddpos(double s) const 
{
    if (!dd_sp_) return Vec3::Zero();
    auto eval_result = dd_sp_->eval(s);
    std::vector<tinyspline::real> result = eval_result.result();
    
    if (result.size() < 3) return Vec3::Zero();

    return Vec3(result[0], result[1], result[2]);
}

Framing::Vec3 Framing::dddpos(double s) const 
{
    // Replaced direct spline evaluation with Finite Difference of ddpos
    // to prevent malloc/heap corruption issues with degree-0 splines.
    double h = 1e-5; 
    
    double s_next = s + h;
    double s_prev = s - h;
    
    // Boundary handling
    if (s_next > 1.0) {
        s_next = 1.0;
        s_prev = 1.0 - h;
    } else if (s_prev < 0.0) {
        s_prev = 0.0;
        s_next = h;
    }
    
    Vec3 v_next = ddpos(s_next);
    Vec3 v_prev = ddpos(s_prev);
    
    // Central difference (or forward/backward at boundaries)
    return (v_next - v_prev) / (s_next - s_prev);
}

double Framing::dist(const Vec3& a, const Vec3& b) const 
{
    return (a - b).norm();
}

double Framing::check_vec_angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
    double cos_angle = a.normalized().dot(b.normalized());
    cos_angle   = std::clamp(cos_angle, -1.0, 1.0);
    
    return std::acos(cos_angle);
}

double Framing::guess_closest_pt(const Vec3& target, int num_samples) const 
{
    double best_s           = 0.5;
    double best_dist_sq     = std::numeric_limits<double>::max();

    for (int i=0; i<=num_samples; ++i) 
    {
        double s        = static_cast<double>(i) / num_samples;
        double dist_sq  = (pos(s) - target).squaredNorm();
        
        if (dist_sq < best_dist_sq) 
        {
            best_dist_sq = dist_sq;
            best_s = s;
        }
    }

    return best_s;
}

Framing::Vec3 Framing::F2I(const Eigen::Vector3d& F_a) const 
{
    double s    = std::clamp(F_a[0], 0.0, 1.0);
    double w1   = F_a[1],   w2 = F_a[2];

    Vec3 gamma_s    = pos(s);
    Vec3 e1, e2, e3;
    get_basis(s, e1, e2, e3);

    Eigen::Matrix3d I_R_F;
    I_R_F << e1, e2, e3;

    Vec3 offset(0.0, w1, w2);
    return gamma_s + I_R_F * offset;
}

Framing::Vec3 Framing::I2F(const Vec3& I_a) const 
{
    double s_star   = get_s_star_point(I_a);
    s_star          = std::clamp(s_star, 0.0, 1.0);
    Vec3 gamma_s    = pos(s_star);

    Vec3 e1, e2, e3;
    get_basis(s_star, e1, e2, e3);
    Eigen::Matrix3d I_R_F;
    I_R_F << e1, e2, e3;

    Vec3 displacement = I_R_F.transpose() * (I_a - gamma_s);
    return Vec3(s_star, displacement[1], displacement[2]);
}

double Framing::get_s_star_point(const Vec3& a, 
                                 int max_iter, 
                                 double tol) const
{
    double s = std::clamp(guess_closest_pt(a, 50), 0.0, 1.0); 

    int iter = 0;
    double damping_factor = 1e-2; 

    while (iter++ < max_iter)
    {
        Vec3 Gamma_s    = pos(s);
        Vec3 dGamma_s   = dpos(s);
        Vec3 ddGamma_s  = ddpos(s);

        Vec3 diff       = Gamma_s - a;
        double f        = diff.dot(dGamma_s);
        double df       = dGamma_s.dot(dGamma_s) + diff.dot(ddGamma_s);

        if (std::fabs(df) < 1e-6) break;

        double delta_s = damping_factor * -f / df;
        if (std::fabs(delta_s) > 1e-2) delta_s *= damping_factor;

        s += delta_s;
        s = std::clamp(s, 0.0, 1.0);

        if (std::fabs(delta_s) < tol) break;
        if (std::fabs(f) > 1e5) break;
    }

    return std::clamp(s, 0.0, 1.0);
}
