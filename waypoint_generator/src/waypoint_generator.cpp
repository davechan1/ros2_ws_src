#include "waypoint_generator/sample_waypoints.h"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;
using bfmt = boost::format;

class WaypointGenerator : public rclcpp::Node 
{
  public:
    WaypointGenerator() : Node("waypoint_generator") 
    {
        // Initialize parameters
        this->declare_parameter<std::string>("waypoint_type", "manual");

        // Publisher and Subscriber Initialization
        pub1_ = this->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
        pub2_ = this->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);
        
        sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&WaypointGenerator::odomCallback, this, std::placeholders::_1));
        sub2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", 10, std::bind(&WaypointGenerator::goalCallback, this, std::placeholders::_1));
        sub3_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "traj_start_trigger", 10, std::bind(&WaypointGenerator::trajStartTriggerCallback, this, std::placeholders::_1));

        trigged_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

        RCLCPP_INFO(this->get_logger(), "Waypoint Generator Node Initialized");
    }

  private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                   pub1_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr         pub2_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            sub1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    sub2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    sub3_;

    std::string                     waypoint_type_;
    bool                            is_odom_ready_ = false;
    nav_msgs::msg::Odometry         odom_;
    nav_msgs::msg::Path             waypoints_;
    std::deque<nav_msgs::msg::Path> waypoint_segments_;
    rclcpp::Time                    trigged_time_;

    void loadSegment(int segid, const rclcpp::Time& time_base) 
    {
        std::string seg_str = (bfmt("seg%d/") % segid).str();
        double      yaw;
        double      time_from_start;

        RCLCPP_INFO(this->get_logger(), "Getting segment %d", segid);

        this->get_parameter_or(seg_str + "yaw", yaw, 0.0);
        this->get_parameter_or(seg_str + "time_from_start", time_from_start, 0.0);

        std::vector<double> ptx, pty, ptz;
        this->get_parameter_or(seg_str + "x", ptx, std::vector<double>());
        this->get_parameter_or(seg_str + "y", pty, std::vector<double>());
        this->get_parameter_or(seg_str + "z", ptz, std::vector<double>());

        if (ptx.size() != pty.size() || ptx.size() != ptz.size()) 
        {
            RCLCPP_ERROR(this->get_logger(), "Size mismatch in waypoint coordinates");
            return;
        }

        nav_msgs::msg::Path         path_msg;
        path_msg.header.stamp       = time_base + rclcpp::Duration::from_seconds(time_from_start);
        path_msg.header.frame_id    = "world";

        // Get the yaw from the tf2::Quaternion
        tf2::Quaternion tf2_quat;
        tf2::fromMsg(odom_.pose.pose.orientation, tf2_quat);
        double baseyaw = tf2::getYaw(tf2_quat);

        for (size_t k = 0; k < ptx.size(); ++k) 
        {
            geometry_msgs::msg::PoseStamped pt;
            tf2::Quaternion q;
            q.setRPY(0, 0, baseyaw + yaw);
            pt.pose.orientation = tf2::toMsg(q);

            Eigen::Vector2d dp(ptx[k], pty[k]);
            Eigen::Vector2d rdp;
            rdp.x() =  std::cos(-baseyaw - yaw) * dp.x() + std::sin(-baseyaw - yaw) * dp.y();
            rdp.y() = -std::sin(-baseyaw - yaw) * dp.x() + std::cos(-baseyaw - yaw) * dp.y();

            pt.pose.position.x = rdp.x() + odom_.pose.pose.position.x;
            pt.pose.position.y = rdp.y() + odom_.pose.pose.position.y;
            pt.pose.position.z = ptz[k] + odom_.pose.pose.position.z;

            path_msg.poses.push_back(pt);
        }

        waypoint_segments_.push_back(path_msg);
    }

    void loadWaypoints(const rclcpp::Time& time_base) 
    {
        int seg_cnt = 0;
        this->get_parameter_or("segment_cnt", seg_cnt, 0);

        waypoint_segments_.clear();
        for (int i = 0; i < seg_cnt; ++i) 
        {
            loadSegment(i, time_base);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu segments", waypoint_segments_.size());
    }

    void publishWaypoints() 
    {
        waypoints_.header.frame_id = "world";
        waypoints_.header.stamp = this->get_clock()->now();
        pub1_->publish(waypoints_);
    }

    void publishWaypointsVis() 
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = "world";
        pose_array.header.stamp = this->get_clock()->now();

        for (const auto& pose_stamped : waypoints_.poses) 
        {
            pose_array.poses.push_back(pose_stamped.pose);
        }

        pub2_->publish(pose_array);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        is_odom_ready_ = true;
        odom_ = *msg;

        if (!waypoint_segments_.empty()) 
        {
            auto expected_time = waypoint_segments_.front().header.stamp;

            rclcpp::Time odom_time(odom_.header.stamp);
            rclcpp::Time expected_time_rclcpp(expected_time);
            if (odom_time >= expected_time_rclcpp)
            {
                waypoints_ = waypoint_segments_.front();
                waypoint_segments_.pop_front();

                publishWaypointsVis();
                publishWaypoints();
            }
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        trigged_time_ = this->get_clock()->now();
        this->get_parameter("waypoint_type", waypoint_type_);

        // if (waypoint_type_ == "manual-lonely-waypoint" && msg->pose.position.z >= 0) 
        if (msg->pose.position.z >= 0)
        {
            waypoints_.poses.clear();
            waypoints_.poses.push_back(*msg);
            publishWaypointsVis();
            publishWaypoints();
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "Unsupported waypoint type or invalid goal.");
        }
    }

    void trajStartTriggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        (void)msg; // Suppress unused parameter warning
        if (!is_odom_ready_) 
        {
            RCLCPP_ERROR(this->get_logger(), "No odom data received!");
            return;
        }

        trigged_time_ = odom_.header.stamp;
        this->get_parameter("waypoint_type", waypoint_type_);

        if (waypoint_type_ == "series") 
        {
            loadWaypoints(trigged_time_);
        }
    }
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointGenerator>());
    rclcpp::shutdown();
    return 0;
}
