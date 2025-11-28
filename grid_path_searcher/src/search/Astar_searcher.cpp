#include "grid_path_searcher/search/Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(  double resolution, 
                                    Vector3d global_xyz_l, 
                                    Vector3d global_xyz_u, 
                                    int max_x_id, 
                                    int max_y_id, 
                                    int max_z_id)
{
    gl_xl_      = global_xyz_l(0);                  //gl: global
    gl_yl_      = global_xyz_l(1);
    gl_zl_      = global_xyz_l(2);

    gl_xu_      = global_xyz_u(0);
    gl_yu_      = global_xyz_u(1);
    gl_zu_      = global_xyz_u(2);
    
    GLX_SIZE    = max_x_id;
    GLY_SIZE    = max_y_id;
    GLZ_SIZE    = max_z_id;
    GLYZ_SIZE   = GLY_SIZE*GLZ_SIZE;
    GLXYZ_SIZE  = GLX_SIZE*GLYZ_SIZE;

    resolution_     = resolution;
    inv_resolution_ = 1.0/resolution;               // for bypassing division

    data_           = new uint8_t[GLXYZ_SIZE];      // obstacle map
    memset(data_, 0, GLXYZ_SIZE*sizeof(uint8_t));

    // https://www.geeksforgeeks.org/cpp/how-to-dynamically-allocate-a-3d-array-in-c/
    GridNodeMap_ = new GridNodePtr **[GLX_SIZE];
    for(int i=0; i<GLX_SIZE; i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[GLY_SIZE];
        for(int j=0; j<GLY_SIZE; j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr [GLZ_SIZE];
            for(int k=0; k<GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);

                GridNodeMap_[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id         = 0;
    ptr->cameFrom   = NULL;
    ptr->gScore     = inf;
    ptr->fScore     = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap_[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl_  || coord_y < gl_yl_  || coord_z <  gl_zl_ || 
        coord_x >= gl_xu_ || coord_y >= gl_yu_ || coord_z >= gl_zu_ )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl_)*inv_resolution_ );
    int idx_y = static_cast<int>( (coord_y - gl_yl_)*inv_resolution_ );
    int idx_z = static_cast<int>( (coord_z - gl_zl_)*inv_resolution_ );      

    data_[idx_x*GLYZ_SIZE + idx_y*GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for(int i=0; i<GLX_SIZE; i++)
        for(int j=0; j<GLY_SIZE; j++)
            for(int k=0; k<GLZ_SIZE; k++)
            {   
                // visualize nodes in close list only
                if(GridNodeMap_[i][j][k]->id == -1)
                    visited_nodes.push_back(GridNodeMap_[i][j][k]->coord);
            }

    RCLCPP_WARN(rclcpp::get_logger("logger"), "visited_nodes size: %zu", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index)
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5)*resolution_ + gl_xl_;
    pt(1) = ((double)index(1) + 0.5)*resolution_ + gl_yl_;
    pt(2) = ((double)index(2) + 0.5)*resolution_ + gl_zl_;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt)
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl_)*inv_resolution_ ), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl_)*inv_resolution_ ), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl_)*inv_resolution_ ), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, 
                                        const int &idx_y, 
                                        const int &idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data_[idx_x*GLYZ_SIZE + idx_y*GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const int & idx_x, 
                                    const int & idx_y, 
                                    const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data_[idx_x*GLYZ_SIZE + idx_y*GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(  GridNodePtr currentPtr, 
                                            vector<GridNodePtr> &neighborPtrSets, 
                                            vector<double> &edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();

    /*
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    */
    Vector3i currentIdx = currentPtr->index;
    for(int i=-1; i<2; i++)
        for(int j=-1; j<2; j++)
            for(int k=-1; k<2; k++)
                {
                    if (i==0 && j==0 and k==0 ) continue;

                    Vector3i idx = currentIdx + Vector3i(i,j,k);
                    
                    if (idx(0) < 0 || idx(0) >= GLX_SIZE || 
                        idx(1) < 0 || idx(1) >= GLY_SIZE ||
                        idx(2) < 0 || idx(2) >= GLZ_SIZE)   continue;

                    neighborPtrSets.push_back(GridNodeMap_[idx(0)][idx(1)][idx(2)]);
                    edgeCostSets.push_back(sqrt(i*i + j*j + k*k));
                }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    */
    // double h = (node2->index.cast<double>() - node1->index.cast<double>()).norm();
    // return (1 + 1/1000) * h;

    double tie_breaker = 1 + 1 / 1000;
    return tie_breaker * getDiagHeu(node1, node2);
}

double AstarPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
  double dx = abs(node1->index(0) - node2->index(0));
  double dy = abs(node1->index(1) - node2->index(1));
  double dz = abs(node1->index(2) - node2->index(2));

  double h = 0.0;
  int diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx == 0)
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  if (dy == 0)
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  if (dz == 0)
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);

  return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    rclcpp::Time time_1 = clock_->now();

    //index of start_point and end_point
    Vector3i start_idx  = coord2gridIndex(start_pt);
    Vector3i end_idx    = coord2gridIndex(end_pt);
    goalIdx_            = end_idx;

    //position of start_point and end_point
    start_pt            = gridIndex2coord(start_idx);
    end_pt              = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr    = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr      = new GridNode(end_idx,   end_pt);

    //openSet_ is the open_list implemented through multimap in STL library
    openSet_.clear();
    
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //STEP 1: finish the AstarPathFinder::getHeu, heuristic function
    startPtr->gScore    = 0;
    startPtr->fScore    = getHeu(startPtr, endPtr);
    startPtr->id        = 1;            // 1: open
    startPtr->coord     = start_pt;
    //put start node in open set
    openSet_.insert( make_pair(startPtr -> fScore, startPtr) );
    
    /*
    STEP 2 :  someelse preparatory works which should be done before while loop
    please write your code below
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double>      edgeCostSets;
    double              tentative_gScore;

    // this is the main loop
    while ( !openSet_.empty() )
    {
        /*
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        */
        auto it     = openSet_.begin(); // always the smallest key
        currentPtr  = it->second;
        // put to close and erase it
        openSet_.erase(it);
        currentPtr->id = -1;
        
        // if the current node is the goal 
        if( currentPtr->index == goalIdx_ )
        {
            rclcpp::Time time_2 = clock_->now();
            terminatePtr_       = currentPtr;
            RCLCPP_WARN(rclcpp::get_logger("logger"), 
                        "[A*]{success} Time in A* is %f ms, path cost is %f m", (time_2 - time_1).seconds()*1000.0, 
                        currentPtr->gScore*resolution_);
            return;
        }

        // STEP 4: finish AstarPathFinder::AstarGetSucc yourself, get the succetion 
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        /*
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        */
        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            /*
            Judge if the neigbors have been expanded
            neighborPtrSets[i]->id = -1 : expanded,     in close set
                                      1 : unexpanded,   in open set
            */
            neighborPtr = neighborPtrSets[i];
            if (isOccupied(neighborPtr->index) || neighborPtr->id == -1) 
                continue;   // if in obstacle / in close set

            tentative_gScore = currentPtr->gScore + edgeCostSets[i];

            if(neighborPtr->id != 1)
            {   /*
                STEP 6:  Discover a new node, do what you need do, 
                and then put neighbor in open set and record it
                */
                neighborPtr->id         = 1;
                neighborPtr->cameFrom   = currentPtr;
                neighborPtr->gScore     = tentative_gScore;
                neighborPtr->fScore     = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                
                neighborPtr->nodeMapIt  = openSet_.insert(make_pair(neighborPtr->fScore, neighborPtr));
            }
            else if(tentative_gScore <= neighborPtr-> gScore)
            {   /*
                In open set, judge if it needs to update
                STEP 7:  As for a node in open set, update it , maintain the openset,
                and then put neighbor in open set and record it
                */
                neighborPtr->cameFrom   = currentPtr;
                neighborPtr->gScore     = tentative_gScore;
                neighborPtr->fScore     = tentative_gScore + getHeu(neighborPtr, endPtr);
                openSet_.erase(neighborPtr->nodeMapIt);
                neighborPtr->nodeMapIt  = openSet_.insert(make_pair(neighborPtr->fScore, neighborPtr));
            }
        }
    }
    //if search fails
    rclcpp::Time time_2 = clock_->now();
    if((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("logger"),
                    "Time consumed in A* path finding is %f ms",
                    (time_2 - time_1).seconds()*1000.0);
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below     
    */
    gridPath.push_back(terminatePtr_);

    while (terminatePtr_->cameFrom != NULL)
    {
        terminatePtr_ = terminatePtr_->cameFrom;
        gridPath.push_back(terminatePtr_);
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}