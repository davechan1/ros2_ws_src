#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <eigen3/Eigen/Dense>
#include <vector>

class TrajectoryGeneratorWaypoint 
{
    private:
  		  double          qp_cost_;
	  	  Eigen::MatrixXd Q_;
		    Eigen::VectorXd Px_, Py_, Pz_;
    public:
        TrajectoryGeneratorWaypoint();
        ~TrajectoryGeneratorWaypoint();

        int Factorial(int x);

        Eigen::VectorXd OSPQPolyQPGeneration( const Eigen::VectorXd &waypoints,
                                              const Eigen::VectorXd &ts,
                                              const int &n_seg,
                                              const int &n_order,
                                              const int &n_poly_perseg);

        Eigen::VectorXd                             getDerivative(double T, int n_order, int k);
        Eigen::MatrixXd                             getQ(const int &n_seg, 
                                                         const int &n_order, 
                                                         const Eigen::VectorXd& ts);
        std::pair<Eigen::MatrixXd, Eigen::VectorXd> getAbeq(const int &n_seg, const int &n_order,
                                                            const Eigen::VectorXd& waypoints,
                                                            const Eigen::VectorXd& ts,
                                                            const Eigen::VectorXd& start_cond,
                                                            const Eigen::VectorXd& end_cond);

        double polyval(const Eigen::VectorXd& coeffs, double t);
};
        

#endif
