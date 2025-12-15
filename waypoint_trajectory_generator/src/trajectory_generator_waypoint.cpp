#include "waypoint_trajectory_generator/trajectory_generator_waypoint.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "OsqpEigen/OsqpEigen.h"

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i=x; i>0; i--)
        fac = fac*i;
    return fac;
}

// Compute the k-th derivative of a polynomial of given order.
VectorXd TrajectoryGeneratorWaypoint::getDerivative(double T, int n_order, int k)
{
    VectorXd derivative = VectorXd::Zero(n_order + 1);
    for (int i=k; i<=n_order; ++i) 
    {
        double coeff = 1.0;
        for (int j = 0; j < k; ++j) 
            coeff *= i-j;
        
        derivative(i) = coeff * std::pow(T, i - k);
    }
    return derivative;
}

// Function to compute the Q matrix
MatrixXd TrajectoryGeneratorWaypoint::getQ(const int &n_seg, 
                                           const int &n_order, 
                                           const VectorXd& ts)
{
    MatrixXd Q = MatrixXd::Zero(0, 0);
    for (int k=0; k<n_seg; ++k) 
    {
        MatrixXd Q_k = MatrixXd::Zero(n_order+1, n_order+1);    // zero-index
        for (int i=4; i<=n_order; ++i) 
        {
            for (int j=4; j<=n_order; ++j) 
            {
                Q_k(i, j) = (i*(i - 1)*(i - 2)*(i - 3) * j*(j - 1)*(j - 2)*(j - 3) / double(i+j-7)) * 
                             std::pow(ts(k), i+j-7);
            }
        }

        if (Q.size() == 0)
        {        
            Q = Q_k;
        } else 
        {
            MatrixXd Q_new                                  = MatrixXd::Zero(Q.rows() + Q_k.rows(), Q.cols() + Q_k.cols());
            Q_new.topLeftCorner(Q.rows(), Q.cols())         = Q;
            Q_new.bottomRightCorner(Q_k.rows(), Q_k.cols()) = Q_k;
            Q                                               = Q_new;    //  Q = blkdiag(Q, Q_k);
        }
    }
    return Q;
}

std::pair<MatrixXd, VectorXd> TrajectoryGeneratorWaypoint::getAbeq( const int &n_seg, const int &n_order,
                                                                    const VectorXd& waypoints,
                                                                    const VectorXd& ts,
                                                                    const VectorXd& start_cond,
                                                                    const VectorXd& end_cond)
{
    int n_coef          = n_order + 1;      // Number of coefficients per polynomial
    int n_all_coef      = n_seg * n_coef;   // Total number of coefficients across all segments
    
    // Start constraints (p, v, a, j) 
    // I print std::cout << "Aeq_start:\n" << Aeq_start << "\n\n"; for check, same as other continuity
    MatrixXd Aeq_start  = MatrixXd::Zero(4, n_all_coef);
    VectorXd beq_start  = start_cond;
    for (int i=0; i<4; ++i)
        Aeq_start.row(i).segment(0, n_coef) = getDerivative(0, n_order, i).transpose(); // vector.segment(start_index, size)
    
    // End constraints (p, v, a, j) 
    MatrixXd Aeq_end = MatrixXd::Zero(4, n_all_coef);
    VectorXd beq_end = end_cond;
    for (int i = 0; i < 4; ++i) 
        Aeq_end.row(i).segment(n_coef*(n_seg - 1), n_coef)  = getDerivative(ts(ts.size() - 1), n_order, i).transpose();
    
    // Waypoints constraints
    MatrixXd Aeq_wp = MatrixXd::Zero(n_seg-1, n_all_coef);
    VectorXd beq_wp = VectorXd::Zero(n_seg-1);
    for (int i=0; i<n_seg-1; ++i)
    {
        Aeq_wp.row(i).segment(n_coef*(i+1), n_coef) = getDerivative(0, n_order, 0).transpose();
        beq_wp(i)                                   = waypoints(i + 1);
    }
    
    // Position continuity constraints
    MatrixXd Aeq_con_p = MatrixXd::Zero(n_seg-1, n_all_coef);
    VectorXd beq_con_p = VectorXd::Zero(n_seg-1);
    for (int i=0; i<n_seg-1; ++i) 
    {
        auto p1 = getDerivative(ts(i+1), n_order, 0);
        auto p2 = getDerivative(0, n_order, 0);
        Aeq_con_p.row(i).segment(n_coef*i, n_coef)      = p1.transpose();
        Aeq_con_p.row(i).segment(n_coef*(i+1), n_coef)  = -p2.transpose();
    }

    // Velocity continuity constraints
    MatrixXd Aeq_con_v = MatrixXd::Zero(n_seg-1, n_all_coef);
    VectorXd beq_con_v = VectorXd::Zero(n_seg-1);
    for (int i=0; i<n_seg-1; ++i) 
    {
        auto v1 = getDerivative(ts(i+1), n_order, 1);
        auto v2 = getDerivative(0, n_order, 1);
        Aeq_con_v.row(i).segment(n_coef*i, n_coef)      = v1.transpose();
        Aeq_con_v.row(i).segment(n_coef*(i+1), n_coef)  = -v2.transpose();
    }
    
    // Acceleration continuity constraints
    MatrixXd Aeq_con_a = MatrixXd::Zero(n_seg-1, n_all_coef);
    VectorXd beq_con_a = VectorXd::Zero(n_seg-1);
    for (int i=0; i<n_seg-1; ++i) 
    {
        auto a1 = getDerivative(ts(i+1), n_order, 2);
        auto a2 = getDerivative(0, n_order, 2);
        Aeq_con_a.row(i).segment(n_coef*i, n_coef)      = a1.transpose();
        Aeq_con_a.row(i).segment(n_coef*(i+1), n_coef)  = -a2.transpose();
    }
    
    // Jerk continuity constraints
    MatrixXd Aeq_con_j = MatrixXd::Zero(n_seg-1, n_all_coef);
    VectorXd beq_con_j = VectorXd::Zero(n_seg-1);
    for (int i=0; i<n_seg-1; ++i) 
    {
        auto j1 = getDerivative(ts(i+1), n_order, 3);
        auto j2 = getDerivative(0, n_order, 3);
        Aeq_con_j.row(i).segment(n_coef*i, n_coef)      = j1.transpose();
        Aeq_con_j.row(i).segment(n_coef*(i+1), n_coef)  = -j2.transpose();
    }
    
    // Combine all submatrices into Aeq and beq
    MatrixXd Aeq_con = MatrixXd::Zero(4 * (n_seg - 1), n_all_coef);
    Aeq_con << Aeq_con_p, Aeq_con_v, Aeq_con_a, Aeq_con_j;

    VectorXd beq_con(4 * (n_seg - 1));
    beq_con << beq_con_p, beq_con_v, beq_con_a, beq_con_j;

    MatrixXd Aeq(Aeq_start.rows() + Aeq_end.rows() + Aeq_wp.rows() + Aeq_con.rows(), n_all_coef);
    Aeq << Aeq_start, Aeq_end, Aeq_wp, Aeq_con;

    VectorXd beq(beq_start.size() + beq_end.size() + beq_wp.size() + beq_con.size());
    beq << beq_start, beq_end, beq_wp, beq_con;

    return {Aeq, beq};    
}

Eigen::VectorXd TrajectoryGeneratorWaypoint::OSPQPolyQPGeneration(  const Eigen::VectorXd &waypoints,
                                                                    const Eigen::VectorXd &ts,
                                                                    const int &n_seg,
                                                                    const int &n_order,
                                                                    const int &n_poly_perseg)
{
    // cout << "n_seg: " << n_seg 
    //      << " n_order: " << n_order 
    //      << " n_poly_perseg: " << n_poly_perseg 
    //      << endl;
    // cout << "ts" << ts << endl;   
    int n_variables = n_seg * n_poly_perseg;  // 3 * 8 = 24

    VectorXd start_cond(4);
    start_cond << waypoints(0), 0, 0, 0;
    VectorXd end_cond(4);
    end_cond << waypoints(waypoints.size() - 1), 0, 0, 0;
    
    // STEP 1: compute Q of p'Qp
    MatrixXd Q = getQ(n_seg, n_order, ts);
    // Extract the upper triangular part of Q
    MatrixXd Q_upper = Q.triangularView<Eigen::Upper>();
    // Convert the upper triangular part into a sparse matrix
    SparseMatrix<double> sparse_Q = Q_upper.sparseView();
    // cout << "Q: " << Q << std::fixed << std::setprecision(1) << endl << endl;

    // STEP 2: compute Aeq and beq 
    auto [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    SparseMatrix<double> sparse_Aeq = Aeq.sparseView();
    // cout << "sparse_Aeq: " << sparse_Aeq << endl << endl;
    
    // STEP 3: gradient vector (zeros for this QP)
    VectorXd gradient = VectorXd::Zero(n_variables);
    
    // STEP 4: set bounds for equality constraints (Aeq * x = beq means lowerBound = upperBound = beq)
    VectorXd lowerBound = beq;
    VectorXd upperBound = beq;
    // cout << "lowerBound: " << lowerBound << endl << endl;
    // cout << "upperBound: " << upperBound << endl << endl;

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(n_variables);
    solver.data()->setNumberOfConstraints(Aeq.rows());
    
    if (!solver.data()->setHessianMatrix(sparse_Q)) 
    {
        std::cerr << "Failed to set Hessian matrix" << std::endl;
        return VectorXd::Zero(n_variables);
    }
    
    if (!solver.data()->setGradient(gradient)) 
    {
        std::cerr << "Failed to set gradient" << std::endl;
        return VectorXd::Zero(n_variables);
    }
    
    if (!solver.data()->setLinearConstraintsMatrix(sparse_Aeq)) 
    {
        std::cerr << "Failed to set constraint matrix" << std::endl;
        return VectorXd::Zero(n_variables);
    }
    
    if (!solver.data()->setLowerBound(lowerBound)) 
    {
        std::cerr << "Failed to set lower bound" << std::endl;
        return VectorXd::Zero(n_variables);
    }
    
    if (!solver.data()->setUpperBound(upperBound)) 
    {
        std::cerr << "Failed to set upper bound" << std::endl;
        return VectorXd::Zero(n_variables);
    }

    // instantiate the solver
    if (!solver.initSolver()) 
    {
        std::cerr << "Failed to initialize solver" << std::endl;
        return VectorXd::Zero(n_variables);
    }

    // solve the QP problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        std::cerr << "Failed to solve QP problem" << std::endl;
        return VectorXd::Zero(n_variables);
    }
    
    // Get QPSolution vector
    Eigen::VectorXd QPSolution = solver.getSolution();

    return QPSolution;
}