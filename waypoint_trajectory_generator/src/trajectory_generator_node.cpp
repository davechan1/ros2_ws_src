#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>

// Useful customized headers
#include "waypoint_trajectory_generator/trajectory_generator_waypoint.h"
#include "waypoint_trajectory_generator/backward.hpp"

#include "OsqpEigen/OsqpEigen.h"

using namespace std;
using namespace Eigen;

// stack tracer​
namespace backward
{
    backward::SignalHandling sh;
}

class TrajNode : public rclcpp::Node
{
public:
    TrajNode() : Node("trajectory_generator_node")
    {
        this->declare_parameter("planning/vel",         0.0);
        this->declare_parameter("planning/acc",         0.0);
        this->declare_parameter("planning/dev_order",   4);
        this->declare_parameter("planning/min_order",   3);
        this->declare_parameter("vis/vis_traj_width",   0.15);
        
        this->get_parameter("planning/vel",             Vel_);
        this->get_parameter("planning/acc",             Acc_);
        this->get_parameter("planning/dev_order",       dev_order_);
        this->get_parameter("planning/min_order",       min_order_);
        this->get_parameter("vis/vis_traj_width",       vis_traj_width_);

        //_poly_numID is the maximum order of polynomial
        poly_num1D_     = 2*dev_order_;

        //state of start point
        startPos_(0)    = 0;
        startPos_(1)    = 0;
        startPos_(2)    = 0;

        startVel_(0)    = 0;
        startVel_(1)    = 0;
        startVel_(2)    = 0;
        
        // Subscriber
        way_pts_sub_    = this->create_subscription<nav_msgs::msg::Path>(
            "waypoints", 1, std::bind(&TrajNode::rcvWaypointsCallBack, this, std::placeholders::_1));

        // Publisher
        wp_traj_path_pub_   = this->create_publisher<nav_msgs::msg::Path>(
            "waypoint_trajectory_path", 10);
        wp_traj_vis_pub_    = this->create_publisher<visualization_msgs::msg::Marker>(
            "vis_trajectory",   1);
        wp_path_vis_pub_    = this->create_publisher<visualization_msgs::msg::Marker>(
            "vis_waypoint_path",   1);
    }
    ~TrajNode(){}

private:
    // Param from launch file
    double vis_traj_width_;
    double Vel_, Acc_;
    int    dev_order_, min_order_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr            way_pts_sub_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   wp_traj_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   wp_path_vis_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               wp_traj_path_pub_;

    // for planning
    int         poly_num1D_;
    MatrixXd    polyCoeff_;
    VectorXd    polyTime_;
    Vector3d    startPos_ = Vector3d::Zero();
    Vector3d    startVel_ = Vector3d::Zero();

    //Get the path points 
    void rcvWaypointsCallBack(const nav_msgs::msg::Path::SharedPtr wp)
    {   
        vector<Vector3d> wp_list;
        wp_list.clear();

        for (int k = 0; k < (int)wp->poses.size(); k++)
        {
            Vector3d pt(wp->poses[k].pose.position.x, 
                        wp->poses[k].pose.position.y, 
                        wp->poses[k].pose.position.z);
            wp_list.push_back(pt);

            // if(wp->poses[k].pose.position.z < 0.0)
            //     break;
        }

        MatrixXd waypoints(wp_list.size(), 3);
        // waypoints.row(0)    = startPos_;
        
        for(int k = 0; k < (int)wp_list.size(); k++)
            waypoints.row(k) = wp_list[k];

        //Trajectory generation: use minimum snap trajectory generation method
        //waypoints is the result of path planning (Manual in this homework)
        // MatrixXd waypoints_test(4, 3);
        // waypoints_test << 0, 0, 0,
        //                   1, 1, 1,
        //                   2, 0, 0,
        //                   3, 1, 1;

        trajGeneration(waypoints);
    }

    void trajGeneration(Eigen::MatrixXd path)
    {
        TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;

        // give an arbitraty time allocation, all set all durations as 1 in the commented function.
        polyTime_   = timeAllocation(path);

        /*
        osqp
        */
        int n_order         = 2*dev_order_ - 1; // the order of polynomial for satisfying start, end all order derivative constraints
        int n_seg           = polyTime_.size(); // the number of segments
        int n_poly_perseg   = n_order + 1;      // the number of variables in each segment


        VectorXd path_x         = path.col(0);  // Extract the first column (x)
        VectorXd path_y         = path.col(1);  // Extract the second column (y)
        VectorXd path_z         = path.col(2);  // Extract the third column (z)
        VectorXd poly_coef_x    = trajectoryGeneratorWaypoint.OSPQPolyQPGeneration(path_x, 
                                                                                   polyTime_, 
                                                                                   n_seg, 
                                                                                   n_order,
                                                                                   n_poly_perseg);
        VectorXd poly_coef_y    = trajectoryGeneratorWaypoint.OSPQPolyQPGeneration(path_y, 
                                                                                   polyTime_,
                                                                                   n_seg, 
                                                                                   n_order,
                                                                                   n_poly_perseg);
        VectorXd poly_coef_z    = trajectoryGeneratorWaypoint.OSPQPolyQPGeneration(path_z, 
                                                                                   polyTime_,
                                                                                   n_seg, 
                                                                                   n_order,
                                                                                   n_poly_perseg);
        // cout << "poly_coef_x:" << poly_coef_x << endl << endl;
        // cout << "poly_coef_y:" << poly_coef_y << endl << endl;
        // cout << "poly_coef_z:" << poly_coef_z << endl << endl;

        //After you finish your homework, you can use the function visWayPointTraj below to visulize your trajectory
        visWayPointPath(path);
        visWayPointTraj(poly_coef_x,
                        poly_coef_y,
                        poly_coef_z, 
                        polyTime_,
                        n_poly_perseg);
    }

    // Helper function to evaluate polynomial at time t
    // coeffs should be in order [c0, c1, c2, ..., c7] for c0 + c1*t + c2*t^2 + ...
    double polyval(const VectorXd& coeffs, double t) 
    {
        double result   = 0.0;
        double t_power  = 1.0;
        for (int i=0; i<coeffs.size(); i++) 
        {
            result += coeffs(i) * t_power;
            t_power *= t;
        }
        return result;
    }


    void visWayPointTraj(VectorXd poly_coef_x,
                        VectorXd poly_coef_y,
                        VectorXd poly_coef_z, 
                        VectorXd time,
                        int n_poly_perseg)
    {        
        visualization_msgs::msg::Marker traj_vis;

        traj_vis.header.stamp       = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        traj_vis.header.frame_id    = "world";

        traj_vis.ns                 = "traj_node/trajectory_waypoints";
        traj_vis.id                 = 0;
        traj_vis.type               = visualization_msgs::msg::Marker::SPHERE_LIST;
        traj_vis.action             = visualization_msgs::msg::Marker::ADD;
        traj_vis.scale.x            = vis_traj_width_;
        traj_vis.scale.y            = vis_traj_width_;
        traj_vis.scale.z            = vis_traj_width_;
        traj_vis.pose.orientation.x = 0.0;
        traj_vis.pose.orientation.y = 0.0;
        traj_vis.pose.orientation.z = 0.0;
        traj_vis.pose.orientation.w = 1.0;

        traj_vis.color.a            = 0.4;
        traj_vis.color.r            = 1.0;
        traj_vis.color.g            = 0.0;
        traj_vis.color.b            = 0.0;

        traj_vis.points.clear();

        // Create Path message
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "world";
        path_msg.header.stamp = traj_vis.header.stamp;

        geometry_msgs::msg::Point pt;

        for(int i=0; i<time.size(); i++)
        {
            for (double t=0.0; t<time(i); t+=0.01)
            {
                // Extract coefficients for i-th segment
                VectorXd seg_coef_x = poly_coef_x.segment(i*n_poly_perseg, n_poly_perseg);
                VectorXd seg_coef_y = poly_coef_y.segment(i*n_poly_perseg, n_poly_perseg);
                VectorXd seg_coef_z = poly_coef_z.segment(i*n_poly_perseg, n_poly_perseg);       

                pt.x = polyval(seg_coef_x, t);
                pt.y = polyval(seg_coef_y, t);
                pt.z = polyval(seg_coef_z, t);

                // Add to marker
                traj_vis.points.push_back(pt);

                // Add to path
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id    = "world";
                pose.header.stamp       = path_msg.header.stamp;
                pose.pose.position.x    = pt.x;
                pose.pose.position.y    = pt.y;
                pose.pose.position.z    = pt.z;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                
                path_msg.poses.push_back(pose);
            }
        }

        // Publish both
        wp_traj_vis_pub_->publish(traj_vis);
        wp_traj_path_pub_->publish(path_msg);
    }

    void visWayPointPath(MatrixXd path)
    {
        visualization_msgs::msg::Marker points, line_list;
        int id                      = 0;
        points.type                 = visualization_msgs::msg::Marker::SPHERE_LIST;
        points.id                   = id;
        points.header.frame_id      = line_list.header.frame_id    = "/world";
        points.header.stamp         = line_list.header.stamp       = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        points.ns                   = line_list.ns                 = "wp_path";
        points.action               = line_list.action             = visualization_msgs::msg::Marker::ADD;
        points.pose.orientation.w   = line_list.pose.orientation.w = 1.0;
        points.pose.orientation.x   = line_list.pose.orientation.x = 0.0;
        points.pose.orientation.y   = line_list.pose.orientation.y = 0.0;
        points.pose.orientation.z   = line_list.pose.orientation.z = 0.0;
        points.scale.x              = 0.3;
        points.scale.y              = 0.3;
        points.scale.z              = 0.3;
        points.color.a              = 1.0;
        points.color.r              = 0.0;
        points.color.g              = 0.0;
        points.color.b              = 0.0;
        
        line_list.id                = id;
        line_list.type              = visualization_msgs::msg::Marker::LINE_STRIP;
        line_list.scale.x           = 0.15;
        line_list.scale.y           = 0.15;
        line_list.scale.z           = 0.15;
        line_list.color.a           = 1.0;        
        line_list.color.r           = 0.0;
        line_list.color.g           = 1.0;
        line_list.color.b           = 0.0;
        
        line_list.points.clear();

        for(int i=0; i<path.rows(); i++)
        {
            geometry_msgs::msg::Point p;
            p.x = path(i, 0);
            p.y = path(i, 1); 
            p.z = path(i, 2); 

            points.points.push_back(p);

            if( i < (path.rows() - 1) )
            {
                geometry_msgs::msg::Point p_line;
                p_line    = p;
                line_list.points.push_back(p_line);
                p_line.x  = path(i+1, 0);
                p_line.y  = path(i+1, 1); 
                p_line.z  = path(i+1, 2);
                line_list.points.push_back(p_line);
            }
        }

        wp_path_vis_pub_->publish(points);
        wp_path_vis_pub_->publish(line_list);
    }

    Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
    {
        Vector3d ret;

        for ( int dim=0; dim<3; dim++)
        {
            VectorXd coeff = (polyCoeff.row(k)).segment(dim*poly_num1D_, poly_num1D_ );
            VectorXd time  = VectorXd::Zero(poly_num1D_ );
            
            for(int j=0; j<poly_num1D_; j++)
            if(j==0)
                time(j) = 1.0;
            else
                time(j) = pow(t, j);

            ret(dim) = coeff.dot(time);
            //cout << "dim:" << dim << " coeff:" << coeff << endl;
        }

        return ret;
    }


    

    VectorXd timeAllocation( MatrixXd Path)
    { 
        VectorXd time(Path.rows() - 1);
    /*
    STEP 1: Learn the "trapezoidal velocity" of "TIme Allocation" in L5, then finish this timeAllocation function
    variable declaration: _Vel, _Acc: _Vel = 1.0, _Acc = 1.0 in this homework, you can change these in the test.launch
    You need to return a variable "time" contains time allocation, which's type is VectorXd
    The time allocation is many relative timeline but not one common timeline
    */
        time.setOnes();
        return time;
    }

};

/**
 * @brief Main function of the node.
 *
 * Initializes the ROS node and starts the main loop of the node.
 *
 * @param argc The number of command line arguments.
 * @param argv The command line arguments.
 *
 * @return 0 if the node runs successfully, otherwise -1.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajNode>());
    rclcpp::shutdown();
    return 0;
}
