#include <iostream>
#include <fstream>
#include <math.h>

#include <ompl-1.7/ompl/config.h>
#include <ompl-1.7/ompl/base/StateSpace.h>
#include <ompl-1.7/ompl/base/Path.h>
#include <ompl-1.7/ompl/base/spaces/RealVectorBounds.h>
#include <ompl-1.7/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl-1.7/ompl/base/StateValidityChecker.h>
#include <ompl-1.7/ompl/base/OptimizationObjective.h>
#include <ompl-1.7/ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl-1.7/ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl-1.7/ompl/geometric/SimpleSetup.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "grid_path_searcher/rrt/rrt_graph_searcher.h"
#include "grid_path_searcher/backward.hpp"

using namespace std;
using namespace Eigen;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace backward 
{
    backward::SignalHandling sh;
}

class DemoRRTNode : public rclcpp::Node
{
public:
    DemoRRTNode() : Node("demo_rrt_node")
    {
        // Declare and get parameters
        this->declare_parameter("map/cloud_margin", 0.0);
        this->declare_parameter("map/resolution",   0.2);
        this->declare_parameter("map/x_size",       50.0);
        this->declare_parameter("map/y_size",       50.0);
        this->declare_parameter("map/z_size",       5.0 );
        this->declare_parameter("planning/start_x", 0.0);
        this->declare_parameter("planning/start_y", 0.0);
        this->declare_parameter("planning/start_z", 0.0);

        this->get_parameter("map/cloud_margin",  _cloud_margin);
        this->get_parameter("map/resolution",    _resolution);
        this->get_parameter("map/x_size",        _x_size);
        this->get_parameter("map/y_size",        _y_size);
        this->get_parameter("map/z_size",        _z_size);
        this->get_parameter("planning/start_x",  _start_pt(0));
        this->get_parameter("planning/start_y",  _start_pt(1));
        this->get_parameter("planning/start_z",  _start_pt(2));

        _inv_resolution = 1.0 / _resolution;       

        // Initialize map boundaries
        _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
        _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;

        _max_x_id = (int)(_x_size * _inv_resolution);
        _max_y_id = (int)(_y_size * _inv_resolution);
        _max_z_id = (int)(_z_size * _inv_resolution);
    
        // Initialize path finder
        _RRTstar_preparatory  = new RRTstarPreparatory();
        _RRTstar_preparatory  -> initGridMap(_resolution, 
                                             _map_lower, _map_upper, 
                                             _max_x_id, _max_y_id, _max_z_id);

        // Subscribers
        map_sub_    = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "map", 1, std::bind(&DemoRRTNode::rcvPointCloudCallBack, this, std::placeholders::_1));
        pts_sub_    = this->create_subscription<nav_msgs::msg::Path>(
            "waypoints", 1, std::bind(&DemoRRTNode::rcvWaypointsCallback, this, std::placeholders::_1));

        // Publishers
        grid_map_vis_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis",         1);
        RRTstar_path_vis_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("RRTstar_path_vis",   1);
    }
    ~DemoRRTNode()
    { 
        delete _RRTstar_preparatory;
    }

private:
    RRTstarPreparatory *_RRTstar_preparatory;
    
    // Global variables
    double _resolution, _inv_resolution, _cloud_margin;
    double _x_size, _y_size, _z_size;    

    // useful global variables
    bool _has_map   = false;

    Vector3d    _start_pt;
    Vector3d    _map_lower, _map_upper;
    int         _max_x_id, _max_y_id, _max_z_id;

    // ros related
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                                .reliable()
                                .durability_volatile();
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr            pts_sub_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   RRTstar_path_vis_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     grid_map_vis_pub_;

    void rcvWaypointsCallback(const nav_msgs::msg::Path::SharedPtr wp)
    {
        if (wp->poses[0].pose.position.z < 0.0 || !_has_map)
            return;

        Vector3d    target_pt;
        target_pt   << wp->poses[0].pose.position.x,
                       wp->poses[0].pose.position.y,
                       wp->poses[0].pose.position.z;

                     
        RCLCPP_INFO(this->get_logger(), "[node] receive the planning target");
        pathFinding(_start_pt, target_pt); 
    }

    void rcvPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
    {
        if(_has_map ) return;

        pcl::PointCloud<pcl::PointXYZ>  cloud;
        pcl::PointCloud<pcl::PointXYZ>  cloud_vis;
        sensor_msgs::msg::PointCloud2   map_vis;

        pcl::fromROSMsg(*pointcloud_map, cloud);
        
        if (cloud.points.empty())   return;

        pcl::PointXYZ   pt;
        for (const auto &point : cloud.points)
        {    
            pt = point;

            // set obstalces into grid map for path planning
            _RRTstar_preparatory->setObs(pt.x, pt.y, pt.z);

            // for visualize only
            Vector3d cor_round  = _RRTstar_preparatory->coordRounding(Vector3d(pt.x, pt.y, pt.z));
            pt.x                = cor_round(0);
            pt.y                = cor_round(1);
            pt.z                = cor_round(2);
            cloud_vis.points.push_back(pt);
        }

        cloud_vis.width    = cloud_vis.points.size();
        cloud_vis.height   = 1;
        cloud_vis.is_dense = true;

        pcl::toROSMsg(cloud_vis, map_vis);

        map_vis.header.frame_id = "/world";
        grid_map_vis_pub_->publish(map_vis);

        _has_map = true;    
    }

    void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
    {
        // Construct the robot state space in which we're planning. 
        ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));

        // Set the bounds of space to be in [0,1].
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, -_x_size*0.5);
        bounds.setLow(1, -_y_size*0.5);
        bounds.setLow(2, 0.0);

        bounds.setHigh(0, +_x_size*0.5);
        bounds.setHigh(1, +_y_size*0.5);
        bounds.setHigh(2, _z_size);

        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        // Construct a space information instance for this state space
        ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
        // Set the object used to check which states in the space are valid

        si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si, _RRTstar_preparatory)));
        si->setup();

        // Set our robot's starting state
        ob::ScopedState<> start(space);
        /**
        STEP 2: Finish the initialization of start state
        */
        start[0] = start_pt(0);
        start[1] = start_pt(1);
        start[2] = start_pt(2);

        // Set our robot's goal state
        ob::ScopedState<> goal(space);
        /**
        STEP 3: Finish the initialization of goal state
        */
        goal[0] = target_pt(0);
        goal[1] = target_pt(1);
        goal[2] = target_pt(2);

        // Create a problem instance

        /**
        STEP 4: Create a problem instance, 
        please define variable as pdef
        */
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // Set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // Set the optimization objective
        /**
        STEP 5: Set the optimization objective, the options you can choose are defined earlier:
        getPathLengthObjective() and getThresholdPathLengthObj()
        */  
        pdef->setOptimizationObjective(  std::make_shared<ob::PathLengthOptimizationObjective>(si) );

        // Construct our optimizing planner using the RRTstar algorithm.
        /**
        STEP 6: Construct our optimizing planner using the RRTstar algorithm, 
        please define varible as optimizingPlanner
        */ 
        auto optimizingPlanner( std::make_shared<og::RRTstar>(si) );

        // Set the problem instance for our planner to solve
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // print the settings for this space
        si->printSettings(std::cout);

        // attempt to solve the planning problem within one second of
        // planning time
        ob::PlannerStatus solved = optimizingPlanner->ob::Planner::solve(1.0);

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
            
            vector<Vector3d> path_points;

            for (size_t path_idx=0; path_idx<path->getStateCount(); path_idx++)
            {
                const ob::RealVectorStateSpace::StateType *state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>(); 
                /**
                STEP 7: Trandform the found path from path to path_points for rviz display
                */
                Vector3d pt((*state)[0], (*state)[1], (*state)[2]);
                // pt.x    = (*state)[0];
                // pt.y    = (*state)[1];
                // pt.z    = (*state)[2];
                path_points.push_back(pt);

            }
            visRRTstarPath(path_points);       
        }
    }

    void visRRTstarPath(vector<Vector3d> nodes)
    {
        visualization_msgs::msg::Marker Points, Line; 
        Points.header.frame_id      = Line.header.frame_id      = "world";
        Points.header.stamp         = Line.header.stamp         = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        Points.ns                   = Line.ns                   = "demo_rrt_node/RRTstarPath";
        Points.action               = Line.action               = visualization_msgs::msg::Marker::ADD;
        Points.pose.orientation.w   = Line.pose.orientation.w   = 1.0;
        Points.id                   = 0;
        Line.id                     = 1;
        Points.type                 = visualization_msgs::msg::Marker::POINTS;
        Line.type                   = visualization_msgs::msg::Marker::LINE_STRIP;

        Points.scale.x = _resolution/2; 
        Points.scale.y = _resolution/2;
        Line.scale.x   = _resolution/2;

        //points are green and Line Strip is blue
        Points.color.g = 1.0f;
        Points.color.a = 1.0;
        Line.color.b   = 1.0;
        Line.color.a   = 1.0;

        geometry_msgs::msg::Point pt;
        for(int i = 0; i < int(nodes.size()); i++)
        {
            Vector3d coord  = nodes[i];
            pt.x            = coord(0);
            pt.y            = coord(1);
            pt.z            = coord(2);

            Points.points.push_back(pt);
            Line.points.push_back(pt);
        }
        RRTstar_path_vis_pub_->publish(Points);
        RRTstar_path_vis_pub_->publish(Line); 
    }

    // Our collision checker. For this demo, our robot's state space
    class ValidityChecker : public ob::StateValidityChecker
    {
    public:
        ValidityChecker(const ob::SpaceInformationPtr& si, RRTstarPreparatory* rrt_preparatory) : 
            ob::StateValidityChecker(si), rrt_preparatory_(rrt_preparatory) {}

        // Returns whether the given state's position overlaps the
        // circular obstacle
        bool isValid(const ob::State* state) const
        {   
            // We know we're working with a RealVectorStateSpace in this
            // example, so we downcast state into the specific type.
            const auto *state3D = state->as<ob::RealVectorStateSpace::StateType>();
            /**
            STEP 1: Extract the robot's (x,y,z) position from its state
            */
            double x = (*state3D)[0];
            double y = (*state3D)[1];
            double z = (*state3D)[2];
            return rrt_preparatory_->isObsFree(x, y, z);
        }
    private:
        RRTstarPreparatory* rrt_preparatory_;
    };
};

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoRRTNode>());
    rclcpp::shutdown();
    return 0;
}
