#ifndef _RRT_NODE_H_
#define _RRT_NODE_H_

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include "grid_path_searcher/backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(){};
    ~GridNode(){};
};


#endif
