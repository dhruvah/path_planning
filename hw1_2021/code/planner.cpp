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



class Cell
{
    public:
        int x;
        int y;
        int g_value;
        int f_value;
        // int parent;
    Cell()
    {};

    Cell(int x, int y)
    {
        this->x = x;
        this->y = y;
        this->g_value = INT_MAX;
        this->f_value = INT_MAX;
        // this->parent = -1;
    }

};

struct f_value_compare
{
    bool operator()(const Cell &c1, const Cell &c2)
    {
        return c1.f_value > c2.f_value;
    }
};


priority_queue<Cell, vector<Cell>, f_value_compare> OPEN;
unordered_map<int, bool> CLOSED;
unordered_map<int, int> PARENT;

pair<int,int> getLocationFromIndex(int index, int x_size, int y_size)
{
    pair<int, int> p;
    p.first = (int)(index % x_size) + 1;
    p.second = (int)(index / x_size) + 1;
    return p;
}

int getHeuristic(int x, int y, int goalposeX, int goalposeY)
{
    return (int)sqrt(((x-goalposeX)*(x-goalposeX) + (y-goalposeY)*(y-goalposeY)));

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
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    Cell start_cell = Cell(robotposeX, robotposeY);

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // int goalposeX = targetposeX;
    // int goalposeY = targetposeY;
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    OPEN.push(start_cell);
    int start_idx = GETMAPINDEX(start_cell.x, start_cell.y, x_size, y_size);
    // node_info[start_idx] = start_cell;
    PARENT[start_idx] = -1;


    while(!OPEN.empty())
    {
        Cell best = OPEN.top();
        OPEN.pop();

        if (CLOSED[GETMAPINDEX(best.x, best.y, x_size, y_size)] == true)
        {
            continue;
        }
        
        CLOSED[GETMAPINDEX(best.x, best.y, x_size, y_size)] = true;

        if (best.x == goalposeX && best.y == goalposeY)
        {
            cout << "Reached Goal" << endl;
            break;
        }

        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = best.x + dX[dir];
            int newy = best.y + dY[dir];
            Cell neighbor_cell = Cell(newx, newy);
            int neighbor_idx = GETMAPINDEX(newx,newy,x_size,y_size);
            if(CLOSED[neighbor_idx] ==true) continue;
                    
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
            {
                if (((int)map[neighbor_idx] >= 0) && ((int)map[neighbor_idx] < collision_thresh))  //if free
                {
                    
                    int map_value = (int)map[neighbor_idx];
                    int new_cost = best.g_value + map_value;

                    if (neighbor_cell.g_value > new_cost)
                    {
                        neighbor_cell.g_value = new_cost;
                        neighbor_cell.f_value = neighbor_cell.g_value + 100*getHeuristic(neighbor_cell.x, neighbor_cell.y, goalposeX, goalposeY);
                        OPEN.push(neighbor_cell);
                        PARENT[neighbor_idx] = GETMAPINDEX(best.x,best.y,x_size,y_size);
                    }
                }
            }
        }

    } 

    int current_index = GETMAPINDEX(goalposeX,goalposeY,x_size,y_size);

    vector <int> path;
    
    while(current_index != start_idx)
    {
        path.push_back(current_index);
        // current_index = node_info[current_index].parent;
        current_index = PARENT[current_index];
        // cout << "idx ..." << current_index << endl;
    }
    
    pair<int,int> nextAction = getLocationFromIndex(path.back(), x_size, y_size);
    
    // int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    // double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    // double disttotarget;
    // for(int dir = 0; dir < NUMOFDIRS; dir++)
    // {
    //     int newx = robotposeX + dX[dir];
    //     int newy = robotposeY + dY[dir];

    //     if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
    //     {
    //         if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
    //         {
    //             disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
    //             if(disttotarget < olddisttotarget)
    //             {
    //                 olddisttotarget = disttotarget;
    //                 bestX = dX[dir];
    //                 bestY = dY[dir];
    //             }
    //         }
    //     }
    // }
    // robotposeX = robotposeX;
    // robotposeY = robotposeY;
    action_ptr[0] = nextAction.first;
    action_ptr[1] = nextAction.second;
    
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