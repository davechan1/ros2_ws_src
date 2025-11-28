#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "grid_path_searcher/backward.hpp"

#include "rclcpp/rclcpp.hpp"

#define inf 1>>20
struct  GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    int             id;     // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord;  
    Eigen::Vector3i dir;    // direction of expanding, for JPS
    Eigen::Vector3i index;
	
    double          gScore, fScore;
    GridNodePtr     cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt; 

    // Constructor
    GridNode(Eigen::Vector3i _index,
             Eigen::Vector3d _coord)
    {
      id        = 0;
      index     = _index;
      coord     = _coord;
      dir       = Eigen::Vector3i::Zero();

      gScore    = inf;
      fScore    = inf;
      cameFrom  = NULL;
    }
    GridNode(){};
    ~GridNode(){};
};


#endif
