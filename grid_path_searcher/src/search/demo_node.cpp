#include <iostream>
#include <fstream>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// #include "grid_path_searcher/graph_searcher.h"
#include "grid_path_searcher/search/Astar_searcher.h"
#include "grid_path_searcher/search/JPS_searcher.h"
#include "grid_path_searcher/backward.hpp"

using namespace std;
using namespace Eigen;

// stack tracer​
namespace backward
{
    backward::SignalHandling sh;
}

class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("demo_node")
    {
        // Declare and get parameters
        this->declare_parameter("map/cloud_margin", 0.0);
        this->declare_parameter("map/resolution",   0.2);
        this->declare_parameter("map/x_size",       50.0);
        this->declare_parameter("map/y_size",       50.0);
        this->declare_parameter("map/z_size",       5.0);
        this->declare_parameter("planning/start_x", 0.0);
        this->declare_parameter("planning/start_y", 0.0);
        this->declare_parameter("planning/start_z", 0.0);
        this->declare_parameter("planning/use_jps", true);
        
        this->get_parameter("map/cloud_margin",     cloud_margin_);
        this->get_parameter("map/resolution",       resolution_);
        this->get_parameter("map/x_size",           x_size_);
        this->get_parameter("map/y_size",           y_size_);
        this->get_parameter("map/z_size",           z_size_);
        this->get_parameter("planning/start_x",     start_pt_(0));
        this->get_parameter("planning/start_y",     start_pt_(1));
        this->get_parameter("planning/start_z",     start_pt_(2));
        this->get_parameter("planning/use_jps",     is_use_jps_);

        // Initialize map boundaries
        map_lower_ << -x_size_ / 2.0, -y_size_ / 2.0, 0.0;
        map_upper_ <<  x_size_ / 2.0,  y_size_ / 2.0, z_size_;

        max_x_id_ = static_cast<int>(x_size_ / resolution_);
        max_y_id_ = static_cast<int>(y_size_ / resolution_);
        max_z_id_ = static_cast<int>(z_size_ / resolution_);

        // Initialize path finder
        astar_path_finder_  = new AstarPathFinder();
        astar_path_finder_->initGridMap(resolution_, map_lower_, map_upper_, 
                                        max_x_id_, max_y_id_, max_z_id_);
        jps_path_finder_    = new JPSPathFinder();
        jps_path_finder_->initGridMap(  resolution_, map_lower_, map_upper_, 
                                        max_x_id_, max_y_id_, max_z_id_);
        
        // Subscribers
        map_sub_    = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "map", 1, std::bind(&DemoNode::rcvPointCloudCallBack, this, std::placeholders::_1));
        pts_sub_    = this->create_subscription<nav_msgs::msg::Path>(
            "waypoints", 1, std::bind(&DemoNode::rcvWaypointsCallback, this, std::placeholders::_1));

        // Publishers
        grid_map_vis_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis",             1);
        grid_path_vis_pub_      = this->create_publisher<visualization_msgs::msg::Marker>("grid_path_vis",          1);
        visited_nodes_vis_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("_visited_nodes_vis_pub", 1);
    }

    ~DemoNode()
    {
        delete astar_path_finder_;
        delete jps_path_finder_;
    }

private:
    // Path finder
    AstarPathFinder *astar_path_finder_;
    JPSPathFinder   *jps_path_finder_;
    
    // Global variables
    double resolution_, cloud_margin_;
    double x_size_, y_size_, z_size_;

    bool        has_map_ = false;
    bool        is_use_jps_;

    Vector3d    start_pt_;
    Vector3d    map_lower_, map_upper_;
    int         max_x_id_, max_y_id_, max_z_id_;

    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                                .reliable()
                                .durability_volatile();

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr            pts_sub_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   grid_path_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   visited_nodes_vis_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     grid_map_vis_pub_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   debug_nodes_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   closed_nodes_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   open_nodes_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   close_nodes_sequence_vis_pub_;

    void rcvWaypointsCallback(const nav_msgs::msg::Path::SharedPtr wp)
    {
        if (wp->poses[0].pose.position.z < 0.0 || !has_map_)
            return;

        Vector3d target_pt;
        target_pt << wp->poses[0].pose.position.x,
                     wp->poses[0].pose.position.y,
                     wp->poses[0].pose.position.z;

                     
        RCLCPP_INFO(this->get_logger(), "[node] Received waypoints.");
        pathFinding(start_pt_, target_pt); 
    }

    void rcvPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
    {
        if (has_map_)   return;

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
            astar_path_finder_->setObs(pt.x, pt.y, pt.z);
            jps_path_finder_->setObs(pt.x, pt.y, pt.z);

            // for visualize only
            Vector3d cor_round  = astar_path_finder_->coordRounding(Vector3d(pt.x, pt.y, pt.z));
            pt.x                = cor_round(0);
            pt.y                = cor_round(1);
            pt.z                = cor_round(2);
            cloud_vis.points.push_back(pt);
        }        

        cloud_vis.width     = cloud_vis.points.size();
        cloud_vis.height    = 1;
        cloud_vis.is_dense  = true;

        pcl::toROSMsg(cloud_vis, map_vis);

        map_vis.header.frame_id = "world";
        grid_map_vis_pub_->publish(map_vis);

        has_map_ = true;
    }

    void pathFinding(const Vector3d start_pt, 
                     const Vector3d target_pt)
    {
        if(is_use_jps_)
        {
            //Call JPS to search for a path
            jps_path_finder_ -> JPSGraphSearch(start_pt, target_pt);

            //Retrieve the path
            auto grid_path     = jps_path_finder_->getPath();
            auto visited_nodes = jps_path_finder_->getVisitedNodes();

            //Visualize the result
            visGridPath(grid_path);
            visVisitedNode(visited_nodes);

            //Reset map for next call
            jps_path_finder_->resetUsedGrids();
        }
        else
        {
            //Call A* to search for a path
            astar_path_finder_->AstarGraphSearch(start_pt, target_pt);

            //Retrieve the path
            auto grid_path     = astar_path_finder_->getPath();
            auto visited_nodes = astar_path_finder_->getVisitedNodes();

            //Visualize the result
            visGridPath(grid_path);
            visVisitedNode(visited_nodes);

            //Reset map for next call
            astar_path_finder_->resetUsedGrids();
        }
    }

    void visGridPath( vector<Vector3d> nodes )
    {   
        visualization_msgs::msg::Marker     node_vis; 
        node_vis.header.frame_id            = "world";
        node_vis.header.stamp               = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        if(is_use_jps_)
            node_vis.ns = "demo_node/jps_path";
        else
            node_vis.ns = "demo_node/astar_path";

        node_vis.type   = visualization_msgs::msg::Marker::CUBE_LIST;
        node_vis.action = visualization_msgs::msg::Marker::ADD;
        node_vis.id     = 0;

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;

        if(is_use_jps_)
        {
            node_vis.color.a = 1.0;
            node_vis.color.r = 1.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 0.0;
        }
        else
        {
            node_vis.color.a = 1.0;
            node_vis.color.r = 0.0;
            node_vis.color.g = 1.0;
            node_vis.color.b = 0.0;
        }

        node_vis.scale.x = resolution_;
        node_vis.scale.y = resolution_;
        node_vis.scale.z = resolution_;

        geometry_msgs::msg::Point pt;
        for(int i=0; i<int(nodes.size()); i++)
        {
            Vector3d coord  = nodes[i];
            pt.x            = coord(0);
            pt.y            = coord(1);
            pt.z            = coord(2);

            node_vis.points.push_back(pt);
        }

        grid_path_vis_pub_->publish(node_vis);
    }

    void visVisitedNode( vector<Vector3d> nodes )
    {
        visualization_msgs::msg::Marker node_vis; 
        node_vis.header.frame_id    = "world";
        node_vis.header.stamp       = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        node_vis.ns                 = "demo_node/expanded_nodes";
        node_vis.type               = visualization_msgs::msg::Marker::CUBE_LIST;
        node_vis.action             = visualization_msgs::msg::Marker::ADD;
        node_vis.id                 = 0;

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;
        node_vis.color.a            = 0.2;
        node_vis.color.r            = 0.0;
        node_vis.color.g            = 0.0;
        node_vis.color.b            = 1.0;

        node_vis.scale.x            = resolution_;
        node_vis.scale.y            = resolution_;
        node_vis.scale.z            = resolution_;

        geometry_msgs::msg::Point pt;
        for(int i=0; i<int(nodes.size()); i++)
        {
            Vector3d coord  = nodes[i];
            pt.x            = coord(0);
            pt.y            = coord(1);
            pt.z            = coord(2);

            node_vis.points.push_back(pt);
        }

        visited_nodes_vis_pub_->publish(node_vis);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoNode>());
    rclcpp::shutdown();
    return 0;
}