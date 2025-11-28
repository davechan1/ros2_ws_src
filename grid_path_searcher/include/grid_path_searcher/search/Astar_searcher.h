#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "grid_path_searcher/backward.hpp"

#include "rclcpp/rclcpp.hpp"
#include "grid_path_searcher/search/node.h"

class AstarPathFinder
{
    private:

    protected:
        rclcpp::Logger            logger_ = rclcpp::get_logger("logger");
        rclcpp::Clock::SharedPtr  clock_  = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

        uint8_t         *data_;
        GridNodePtr     ***GridNodeMap_;
        Eigen::Vector3i goalIdx_;
        int             GLX_SIZE,   GLY_SIZE,   GLZ_SIZE;
        int             GLXYZ_SIZE, GLYZ_SIZE;

        double          resolution_,    inv_resolution_;
        double          gl_xl_, gl_yl_, gl_zl_;
        double          gl_xu_, gl_yu_, gl_zu_;

        GridNodePtr     terminatePtr_;

        std::multimap<double, GridNodePtr>  openSet_;

        double getHeu(GridNodePtr node1, GridNodePtr node2);
        void AstarGetSucc(GridNodePtr               currentPtr,
                          std::vector<GridNodePtr>  &neighborPtrSets,
                          std::vector<double>       &edgeCostSets);

        bool isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;
        bool isOccupied(const Eigen::Vector3i &index) const;
        bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
        bool isFree(const Eigen::Vector3i &index) const;
        
        Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
        Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);

    public:
        AstarPathFinder(){};
        ~AstarPathFinder(){};
        void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
        void resetGrid(GridNodePtr ptr);
        void resetUsedGrids();

        void initGridMap(double resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
        void setObs(const double coord_x, const double coord_y, const double coord_z);

        double getDiagHeu(GridNodePtr node1, GridNodePtr node2);

        Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);
        std::vector<Eigen::Vector3d> getPath();
        std::vector<Eigen::Vector3d> getVisitedNodes();
};

#endif