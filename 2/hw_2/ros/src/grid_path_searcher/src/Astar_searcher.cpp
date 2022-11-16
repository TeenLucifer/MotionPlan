#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index)//栅格地图索引转换为世界坐标系坐标值
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt)//世界地图坐标系坐标值转换为栅格地图索引
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
   Vector3i currentIdx = currentPtr->index;
   int x = currentIdx[0];
   int y = currentIdx[1];
   int z = currentIdx[2];

   for(int i = -1; i <= 1; ++i)
   {
        for(int j = -1; j <= 1; ++j)
        {
            for(int k = -1; k <= 1; ++k)
            {
                if(i == 0 && j == 0 && k == 0)//不搜索当前节点
                {
                    continue;
                }
                int newX = x + i;
                int newY = y + j;
                int newZ = z + k;
                GridNodePtr node = GridNodeMap[newX][newY][newZ];
                if(newX >= 0 && newX < GLX_SIZE && newY >= 0 && newY < GLY_SIZE && newZ >= 0 && newZ < GLZ_SIZE)//确保新的索引不超过边界值
                {
                    if(isOccupied({newX, newY, newZ}))//确保该邻居节点不是障碍物
                    {
                        continue;
                    }
                    //if(node->id == -1)//确保该邻居节点不在close中
                    //{
                    //    continue;
                    //}//判断邻居节点是否在close中的程序在后面的步骤中有，这里就不多写了
                    neighborPtrSets.push_back(node);//记录未在close中且非障碍物的邻居节点
                    edgeCostSets.push_back((currentPtr->coord - node->coord).norm());//计算当前节点到该邻居节点的距离
                }
            }
        }
   }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2, GridNodePtr startNode, GridNodePtr goalNode)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    double dx = abs(node1->coord(0) - node2->coord(0));
    double dy = abs(node1->coord(1) - node2->coord(1));
    double dz = abs(node1->coord(2) - node2->coord(2));
    double dx_ = abs(startNode->coord(0) - goalNode->coord(0));
    double dy_ = abs(startNode->coord(1) - goalNode->coord(1));
    double dz_ = abs(startNode->coord(2) - goalNode->coord(2));
    double tieBreakerCoeff = 1;
    if(useTieBreaker == NO)
    {
        tieBreakerCoeff = 0;
    }
    else if(useTieBreaker == YES)
    {
        double cross = abs((dx * dy_ - dx_ * dy) + (dx * dz_ - dx_ * dz) + (dy * dz_ - dy_ * dz));
        tieBreakerCoeff = cross * 0.01;
    }

    switch (heuristicsType)
    {
    case ManhattanHeu:
        return (dx + dy + dz) + tieBreakerCoeff;
        break;
    
    case EuclideanHeu:
        return sqrt(dx * dx + dy * dy + dz * dz) + tieBreakerCoeff;
        break;

    case DiagonalHeu:
        return (dx + dy + dz + (sqrt(3) - 2) * min({dx, dy, dz})) + tieBreakerCoeff;
        break;

    default:
        break;
    }

    return 0;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);//起点在栅格地图下的坐标
    Vector3i end_idx   = coord2gridIndex(end_pt);//终点在栅格地图下的坐标
    goalIdx = end_idx;//目标点即终点

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);//
    end_pt   = gridIndex2coord(end_idx);//

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr, startPtr, endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1;
    startPtr -> coord = start_pt;
    startPtr->nodeMapIt = openSet.insert(make_pair(startPtr -> fScore, startPtr));
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while (!openSet.empty())
    {
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
       currentPtr = openSet.begin()->second;//openSet是<double fScore, GridNodePtr node>的集合，second指node
       currentPtr->id = -1;//将node从open中移除，标记为close
       openSet.erase(currentPtr->nodeMapIt);

        // if the current node is the goal 
        if( currentPtr->index == goalIdx )
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            if(heuristicsType == ManhattanHeu)
            {
                ROS_WARN("[ManhattanHeu A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            }
            else if(heuristicsType == EuclideanHeu)
            {
                if(useTieBreaker == NO)
                {
                    ROS_WARN("[EuclideanHeu A* without tieBreaker]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
                }
                else if(useTieBreaker == YES)
                {
                    ROS_WARN("[EuclideanHeu A* with tieBreaker]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
                }
            }
            else if(heuristicsType == DiagonalHeu)
            {
                ROS_WARN("DiagnalHeu [A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            }
            //ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     
        //扩展，得到当前节点的邻居节点

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */
        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            double gn = currentPtr->gScore + edgeCostSets[i];//当前邻居节点的gn，是当前节点的gn加当前节点与邻居节点的距离
            double hn = getHeu(neighborPtr, endPtr, startPtr, endPtr);
            double fn = gn + hn;//当前邻居节点的fn，是当前邻居节点的gn加当前邻居节点与目标点之间的启发式函数值;

            if(neighborPtr->id == 0)//discover a new node, which is not in the closed set and open set
            { 
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                //对于未探索过的节点，即f(n) = inf，设置f(n) = g(n) + h(n)
                neighborPtr->gScore = gn;
                neighborPtr->fScore = fn;
                neighborPtr->cameFrom = currentPtr;//父节点
                neighborPtr->id = 1;//标记为open

                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));//记录下当前邻居节点在open中的位置
                //if(neighborPtr->index == goalIdx)//到达目标点
                //{
                //    ros::Time time_2 = ros::Time::now();
                //    terminatePtr = neighborPtr;
                //    ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
                //    return;
                //}
                continue;
            }
            else if(neighborPtr->id == 1)//this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
            { 
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                //对于open中的节点，即g(n) = m，若果新的g(n) = n < m，更新g(n)的值
                if(neighborPtr->gScore >= gn)
                {
                    neighborPtr->gScore = gn;
                    neighborPtr->fScore = fn;
                    neighborPtr->cameFrom = currentPtr;
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                }

                continue;
            }
            else//this node is in closed set
            {
                /*
                *
                please write your code below
                *        
                */
                //对于close中的节点，不需要其它处理
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
    {
        if(heuristicsType == ManhattanHeu)
        {
            ROS_WARN("Time consume in ManhatanHeu Astar path finding is %f", (time_2 - time_1).toSec() );
        }
        else if(heuristicsType == EuclideanHeu)
        {
            ROS_WARN("Time consume in EuclideanHeu Astar path finding is %f", (time_2 - time_1).toSec() );
        }
        else if(heuristicsType == DiagonalHeu)
        {
            ROS_WARN("Time consume in DiagnalHeu Astar path finding is %f", (time_2 - time_1).toSec() );
        }
        //ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
    }
}

vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    //回溯获得最短路径
    GridNodePtr node = terminatePtr;
    while(node->cameFrom != nullptr)
    {
        gridPath.push_back(node);
        node = node->cameFrom;
    }
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}