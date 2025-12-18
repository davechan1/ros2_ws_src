#ifndef FRAMING_HPP
#define FRAMING_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>

#include <tinyspline_ros/tinysplinecpp.hpp>

class Framing 
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Vec3  = Eigen::Vector3d;

    Framing(const std::vector<Vec3>& ctrlPts);
    virtual ~Framing() = default;

    // Delete copy constructor and copy assignment (prevent double-free)
    Framing(const Framing&) = delete;
    Framing& operator=(const Framing&) = delete;

    // Allow move semantics (safe transfer of ownership)
    Framing(Framing&&) = default;
    Framing& operator=(Framing&&) = default;

    Vec3 pos(double s)      const;
    Vec3 dpos(double s)     const;
    Vec3 ddpos(double s)    const;
    Vec3 dddpos(double s)   const;

    // Simple helpers
    double arclength()                                          const;
    double guess_closest_pt(const Vec3& a, int n_samples=15)    const;  
    double dist(const Vec3& a, const Vec3& b)                   const;
    double check_vec_angle(const Vec3& a, const Vec3& b)        const;

    // Transforms
    double get_s_star_point(const Vec3& a,
                            int max_iter=10,
                            double tol=1e-5)  const;
    Vec3 F2I(const Vec3& F_a)   const;
    Vec3 I2F(const Vec3& I_a)   const;

    // To be implemented by child classes
    virtual void get_basis( double s,
                            Vec3& e1, Vec3& e2, Vec3& e3) const = 0;
    virtual void cartan(  double s, 
                          double& l1, double& l2, double& l3,
                          Vec3& de1, Vec3& de2, Vec3& de3) const = 0;
    
  protected:
    std::vector<Vec3, Eigen::aligned_allocator<Vec3>> controlPoints_;

    // tinyspline::BSpline sp_, d_sp_, dd_sp_, ddd_sp_;
    std::unique_ptr<tinyspline::BSpline> sp_;
    std::unique_ptr<tinyspline::BSpline> d_sp_;
    std::unique_ptr<tinyspline::BSpline> dd_sp_;
    std::unique_ptr<tinyspline::BSpline> ddd_sp_;
};

#endif // FRAMING_HPP
