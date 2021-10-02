/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <stdio.h>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <vector>
#include <chrono>

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

// Point in 2-D space

//**Will have to change this to make effecient
struct point2d_int
{
    int x, y;

    // std::vector<std::pair<int, int>>;

};

struct point_vector
{
    std::vector<std::pair<int, int>> point;
};

struct cell
{
    point2d_int location;
    int g_value{INT_MAX};
    int f_value{INT_MAX};
    bool in_open_list{false};
    // int cost = getMapValue(location.x, location.y);
};

struct f_value_compare
{
    bool operator()(const cell &c1, const cell &c2)
    {
        return c1.f_value > c2.f_value;
    }
};

class Search {



    //Make constructor class to initialize variables


    public:
    double* map;
    double* target_traj;
    int row;
    int col;
    int robotposeX;
    int robotposeY;
    int goalX;
    int goalY;
    int x_size;
    int y_size;
    int collision_thresh;
    int target_steps;
    int curr_time;
    point2d_int start_point;
    point2d_int goal_point;

    int cost_so_far;
    int total_cost;
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    int directions = 8;


    std::priority_queue<cell, std::vector<cell>, f_value_compare> OPEN;
    std::unordered_map<int, bool> CLOSED;
    std::unordered_map<int, int> PARENT;


    void astar();
    point2d_int astarExecute();
    point2d_int getLocationFromIndex(int);
    int getIndexFromLocation(point2d_int);

    int getHeuristic(int, int);
    bool checkCellValid();
    int getPathCost();
    void getPathSteps();
    void getGoal();
    int getMapValue(int, int);
    // cell getCell(int, int);
    std::vector<std::pair<int, int>> getPath();

    std::vector<std::pair<int, int>> getNeighbors(point2d_int);


};

std::vector<std::pair<int, int>> getPath()
{
    std::vector<std::pair<int, int>> path;
    point2d_int goal = {goalX, goalY};
    point2d_int robot = {robotposeX, robotposeY};
    int current_index = getIndexFromLocation(goal); //Initially index of goal
    int robot_index = getIndexFromLocation(robot);


    while(robot_index != current_index)
    {
        path.push_back(std::make_pair(getLocationFromIndex(current_index).x, getLocationFromIndex(current_index).y));
        current_index = PARENT[current_index];

    }
    // path.push_back(std::make_pair(robotposeX, robotposeY));
    // std::reverse(path.begin(), path.end());

    return path;
}

int getHeuristic(int x, int y, int goalX, int goalY)
{
    return (int)sqrt(((x-goalX)*(x-goalX) + (y-goalY)*(y-goalY)));
}

point2d_int getGoal()
{
    point2d_int goal;
    goalX = (int) target_traj[target_steps-1];
    goalY = (int) target_traj[target_steps-1+target_steps];
    goal.x = goalX;
    goal.y = goalY;
    return goal;
}


point2d_int getLocationFromIndex(int index)
{
    int x, y;
    x = index % x_size;
    y = index / x_size;
    point2d_int p;
    p.x = x - 1;
    p.y = y - 1;

    return p; 
}

int getIndexFromLocation(point2d_int location)
{
    int index;

    // (location.y -1 ??)
    index = location.x -1  + ((location.y - 1) * x_size);
    return index;
}

int getMapValue(int newx, int newy)
{
    return (int)map[GETMAPINDEX(newx, newy, x_size, y_size)];
}


std::vector<std::pair<int, int>> getNeighbors(point2d_int cell_location)
{
    std::vector<std::pair<int, int>> neighbors;
    for (int i = 0; i < directions; ++i )
    {
        int newx = cell_location.x + dX[i];
        int newy = cell_location.y + dY[i];

        int map_val = getMapValue(newx, newy);

        if ((map_val >= 0) && (map_val < collision_thresh))  //if free
            {
                neighbors.push_back({newx, newy});
                // neighbors.push_back(std::make_pair(newx, newy));
            }
    }
    return neighbors;
}


void astar(){

    // priority_queue OPEN;
    // closed_map CLOSED;
    // parent_map PARENT;
    // std::vector<std::pair<int, int>> neighbors;
    std::priority_queue<cell, std::vector<cell>, f_value_compare> OPEN;
    std::unordered_map<int, bool> CLOSED;
    std::unordered_map<int, int> PARENT;


    int start_index = 0;
    point2d_int start_point;
    start_point = getLocationFromIndex(start_index);
    cell start, neighbor_cell;
    start.location = start_point;
    start.g_value = 0;
    start.f_value = 0;

    

    OPEN.push(start);
    PARENT[start_index] = start_index;

    while(!OPEN.empty()) {
        cell best = OPEN.top();
        OPEN.pop();

        robotposeX = best.location.x;
        robotposeY = best.location.y;

        if (getIndexFromLocation(best.location) == getIndexFromLocation(goal_point)){
            break;
        }

        CLOSED[getIndexFromLocation(best.location)] = true;

        for (auto &neighbor : getNeighbors(best.location))
        {
            int n_row = neighbor.first;
            int n_col = neighbor.second;
            neighbor_cell.location = {n_row, n_col};
            // cell neighbor_cell = getCell(n_row, n_col);
            //Check if cell has been visited or not
            //Col vs row order??
            if (CLOSED[getIndexFromLocation({n_row, n_col})])
            {
                continue;
            }
            int new_cost = best.g_value + getMapValue(n_row, n_col);
            if (new_cost < neighbor_cell.g_value)
            {
                neighbor_cell.g_value = new_cost;
                neighbor_cell.f_value = neighbor_cell.g_value + getHeuristic(n_row, n_col);
                neighbor_cell.in_open_list = true;
                OPEN.push(neighbor_cell);
                PARENT[getIndexFromLocation(neighbor_cell.location)] = getIndexFromLocation(best.location);
            }
            
        }


    }

}

point2d_int astarExecute(
        double* map,
        int collision_thresh,
        int robotposeX,
        int robotposeY,
        int goalposeX,
        int goalposeY,
        int x_size,
        int y_size
        )
{

}


inline point2d_int greedy(
        double*	map,
        int collision_thresh,
        int robotposeX,
        int robotposeY,
        int goalposeX,
        int goalposeY,
        int x_size,
        int y_size
        )
{
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    int bestX = 0, bestY = 0;
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double disttotarget;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;

    struct point2d_int p;
    p.x = robotposeX;
    p.y = robotposeY;

    return p;

}
static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{

    Search search;
    // 8-connected grid
    // int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    // int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    // int goalposeX = (int) target_traj[target_steps-1];
    // int goalposeY = (int) target_traj[target_steps-1+target_steps];

    int goalposeX = (int) target_traj[20 - 1];
    int goalposeY = (int) target_traj[20 - 1 + 20];
    std::vector<std::pair<int, int>> path;

    search.goalX = goalposeX;
    search.goalY = goalposeY;
    search.robotposeX = robotposeX;
    search.robotposeY = robotposeY;
    search.x_size = x_size;
    search.y_size = y_size;
    search.collision_thresh = collision_thresh;
    search.map = map;
    search.target_traj = target_traj;

    search.astar();
    path = search.getPath();

    std::cout << path << std::endl;

    int x = path.front().first;
    int y = path.front().second;

    std::cout << x << std::endl;
    std::cout << y << std::endl;

    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    // struct point2d_int r;
    // r = greedy(map, collision_thresh, robotposeX, robotposeY, goalposeX, goalposeY, x_size, y_size);

    action_ptr[0] = x;
    action_ptr[1] = y;
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}