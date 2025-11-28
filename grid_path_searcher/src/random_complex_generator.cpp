#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <random>

using namespace std;
using namespace Eigen;

class RandomComplexScene : public rclcpp::Node
{
  public:
    RandomComplexScene() : Node("random_complex_scene")
    {
        this->declare_parameter("init_state_x",                 0.0);
        this->declare_parameter("init_state_y",                 0.0);
        this->declare_parameter("map/x_size",                   50.0);
        this->declare_parameter("map/y_size",                   50.0);
        this->declare_parameter("map/z_size",                   5.0);
        this->declare_parameter("map/obs_num",                  30);
        this->declare_parameter("map/circle_num",               30);
        this->declare_parameter("map/resolution",               0.2);
        this->declare_parameter("ObstacleShape/lower_rad",      0.3);
        this->declare_parameter("ObstacleShape/upper_rad",      0.8);
        this->declare_parameter("ObstacleShape/lower_hei",      3.0);
        this->declare_parameter("ObstacleShape/upper_hei",      7.0);
        this->declare_parameter("CircleShape/lower_circle_rad", 0.3);
        this->declare_parameter("CircleShape/upper_circle_rad", 0.8);
        this->declare_parameter("sensing/rate",                 1.0);

        this->get_parameter("init_state_x",                     _init_x);
        this->get_parameter("init_state_y",                     _init_y);
        this->get_parameter("map/x_size",                       _x_size);
        this->get_parameter("map/y_size",                       _y_size);
        this->get_parameter("map/z_size",                       _z_size);
        this->get_parameter("map/obs_num",                      _obs_num);
        this->get_parameter("map/circle_num",                   _cir_num);
        this->get_parameter("map/resolution",                   _resolution);
        this->get_parameter("ObstacleShape/lower_rad",          _w_l);
        this->get_parameter("ObstacleShape/upper_rad",          _w_h);
        this->get_parameter("ObstacleShape/lower_hei",          _h_l);
        this->get_parameter("ObstacleShape/upper_hei",          _h_h);
        this->get_parameter("CircleShape/lower_circle_rad",     _w_c_l);
        this->get_parameter("CircleShape/upper_circle_rad",     _w_c_h);
        this->get_parameter("sensing/rate",                     _sense_rate);

        // Initialize map boundaries
        _x_l = -_x_size / 2.0;
        _x_h =  _x_size / 2.0;
        _y_l = -_y_size / 2.0;
        _y_h =  _y_size / 2.0;

        // Publisher
        _all_map_pub    = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 1);

        // Generate the random map
        RandomMapGenerate();

        // Timer for publishing sensed points
        double sensing_period = 1.0 / _sense_rate;
        _timer = this->create_wall_timer(
            std::chrono::duration<double>(sensing_period),
            std::bind(&RandomComplexScene::pubSensedPoints, this));
    }

  private:
    // Member variables
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_pub;
    rclcpp::TimerBase::SharedPtr                                _timer;

    int _obs_num, _cir_num;
    double _x_size, _y_size, _z_size, _init_x, _init_y, _resolution, _sense_rate;
    double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _w_c_l, _w_c_h;

    bool _has_map = false;

    sensor_msgs::msg::PointCloud2   _globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ>  _cloudMap;

    pcl::search::KdTree<pcl::PointXYZ>  _kdtreeMap;
    vector<int>                         _pointIdxSearch;
    vector<float>                       _pointSquaredDistance;

    void RandomMapGenerate()
    {
        random_device rd;
        default_random_engine eng(rd());

        uniform_real_distribution<double> rand_x(_x_l, _x_h);
        uniform_real_distribution<double> rand_y(_y_l, _y_h);
        uniform_real_distribution<double> rand_w(_w_l, _w_h);
        uniform_real_distribution<double> rand_h(_h_l, _h_h);

        uniform_real_distribution<double> rand_x_circle(_x_l + 1.0, _x_h - 1.0);
        uniform_real_distribution<double> rand_y_circle(_y_l + 1.0, _y_h - 1.0);
        uniform_real_distribution<double> rand_r_circle(_w_c_l, _w_c_h);

        uniform_real_distribution<double> rand_roll(-M_PI, M_PI);
        uniform_real_distribution<double> rand_pitch(M_PI / 4.0, M_PI / 2.0);
        uniform_real_distribution<double> rand_yaw(M_PI / 4.0, M_PI / 2.0);
        uniform_real_distribution<double> rand_ellipse_c(0.5, 2.0);
        uniform_real_distribution<double> rand_num(0.0, 1.0);

        pcl::PointXYZ pt_random;

        // Generate circles
        for (int i = 0; i < _cir_num; i++)
        {
            double x0, y0, z0, R;
            vector<Vector3d> circle_set;

            x0  = rand_x_circle(eng);
            y0  = rand_y_circle(eng);
            z0  = rand_h(eng) / 2.0;
            R   = rand_r_circle(eng);

            if (sqrt(pow(x0 - _init_x, 2) + pow(y0 - _init_y, 2)) < 2.0)
                continue;

            double a    = rand_ellipse_c(eng);
            double b    = rand_ellipse_c(eng);

            for (double theta = -M_PI; theta < M_PI; theta += 0.025)
            {
                double x = a*cos(theta)*R;
                double y = b*sin(theta)*R;
                double z = 0;
                Vector3d pt3(x, y, z);
                circle_set.push_back(pt3);
            }

            Matrix3d Rot;
            double roll     = rand_roll(eng);
            double pitch    = rand_pitch(eng);
            double yaw      = rand_yaw(eng);
            Rot <<  cos(roll)*cos(yaw) - cos(pitch)*sin(roll)*sin(yaw), -cos(pitch)*cos(yaw)*sin(roll) - cos(roll)*sin(yaw),  sin(roll)*sin(pitch),
                    cos(yaw)*sin(roll) + cos(roll)*cos(pitch)*sin(yaw),  cos(roll)*cos(pitch)*cos(yaw) - sin(roll)*sin(yaw), -cos(roll)*sin(pitch),
                    sin(pitch)*sin(yaw), cos(yaw)*sin(pitch), cos(pitch);

            for (auto pt : circle_set)
            {
                Vector3d pt3_rot = Rot*pt;
                pt_random.x = pt3_rot(0) + x0 + 0.001;
                pt_random.y = pt3_rot(1) + y0 + 0.001;
                pt_random.z = pt3_rot(2) + z0 + 0.001;

                if (pt_random.z >= 0.0)
                    _cloudMap.points.push_back(pt_random);
            }
        }

        // Check kdtree
        bool is_kdtree_empty = false;
        if (_cloudMap.points.size() > 0)
            _kdtreeMap.setInputCloud(_cloudMap.makeShared());
        else
            is_kdtree_empty = true;

        // Generate pillars
        for (int i = 0; i < _obs_num; i++)
        {
            double x, y, w, h;
            x   = rand_x(eng);
            y   = rand_y(eng);
            w   = rand_w(eng);

            if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 0.8)
                continue;

            pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h) / 2.0);
            _pointIdxSearch.clear();
            _pointSquaredDistance.clear();

            if (!is_kdtree_empty)
            {
                if (_kdtreeMap.nearestKSearch(searchPoint, 1, _pointIdxSearch, _pointSquaredDistance) > 0)
                {
                    if (sqrt(_pointSquaredDistance[0]) < 1.0)
                        continue;
                }
            }

            x = floor(x / _resolution) * _resolution + _resolution / 2.0;
            y = floor(y / _resolution) * _resolution + _resolution / 2.0;

            int widNum = ceil(w / _resolution);
            for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
            {
                for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
                {
                    h = rand_h(eng);
                    int heiNum = 2.0 * ceil(h / _resolution);
                    for (int t = 0; t < heiNum; t++)
                    {
                        pt_random.x = x + (r + 0.0) * _resolution + 0.001;
                        pt_random.y = y + (s + 0.0) * _resolution + 0.001;
                        pt_random.z = (t + 0.0) * _resolution * 0.5 + 0.001;
                        _cloudMap.points.push_back(pt_random);
                    }
                }
            }
        }

        _cloudMap.width      = _cloudMap.points.size();
        _cloudMap.height     = 1;
        _cloudMap.is_dense   = true;

        _has_map = true;

        pcl::toROSMsg(_cloudMap, _globalMap_pcd);
        _globalMap_pcd.header.frame_id = "world";
    }

    void pubSensedPoints()
    {
        if (!_has_map)
            return;

        _all_map_pub->publish(_globalMap_pcd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomComplexScene>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}