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
#include <algorithm>
#include <time.h>


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

typedef pair<pair<int,int>,int> pi;

class Cell
{
    public:
        int x;
        int y;
        int t;
        int t_fdijkstra;
        int g_value;
        int g_value_fdijkstra;
        int f_value;
        int h_value;
        int robot_count;
    Cell()
    {};

    Cell(int x, int y)
    {
        this->x = x;
        this->y = y;
        this->t = 0;
        this->t_fdijkstra = 0;
        this->g_value = INT_MAX;
        this->g_value_fdijkstra = INT_MAX;
        this->f_value = INT_MAX;
        this->h_value = INT_MAX;

    }

    Cell(int x, int y, int t)
    {
        this->x = x;
        this->y = y;
        this->t = t;
        this->t_fdijkstra = 0;
        this->g_value = INT_MAX;
        this->g_value_fdijkstra = INT_MAX;
        this->f_value = INT_MAX;
        this->h_value = INT_MAX;
    }

};

struct f_value_compare // astar
{
    bool operator()(const Cell &c1, const Cell &c2)
    {
        return c1.f_value > c2.f_value;
    }
};

struct h_value_compare 
{// heuristic dijkstra
    bool operator()(const Cell &c1, const Cell &c2)
    {
        return c1.h_value > c2.h_value;
    }
};

struct g_value_compare_fdijkstra
{
    bool operator()(const pi &c1, const pi &c2)
    {
        return c1.second > c2.second;
    }
};


// 8-connected grid
int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

// 9-connected grid
int dX_3D[NUMOFDIRS+1] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
int dY_3D[NUMOFDIRS+1] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

//A-star
priority_queue<Cell, vector<Cell>, f_value_compare> OPEN;
priority_queue<Cell, vector<Cell>, f_value_compare> OPEN3D;
unordered_map<int, bool> CLOSED; //Index of Cell vs If it is in the List
unordered_map<int, bool> CLOSED3D;
unordered_map<int, int> PARENT; //Index of the child vs Index of the parent
unordered_map<int, int> PARENT3D;

pair<int, int> goal;
vector<int> astarPath; //Index of Cells that form the path
vector<int> astarPath3D;


//Forward Dijkstra
priority_queue<Cell, vector<Cell>, h_value_compare> OPEN_DF;
unordered_map<int, int> HEURISTIC_DF; //Index of Cell vs Cost to get to the Cell
unordered_map<int, bool> EXPANDED_DF; //Closed list for Forward Dijkstra
unordered_map<int, int> ROBOT_STEPS; //Index of Cell vs Steps robot needs to get to the Cell


//Backward Dijkstra
priority_queue<Cell, vector<Cell>, h_value_compare> OPEN_DB;
unordered_map<int, int> HEURISTIC_DB; //Index of Cell vs Heuristic value used in A-star
unordered_map<int, bool> EXPANDED_DB;

pair<int,int> getLocationFromIndex2D(int index, int x_size, int y_size)
{
    pair<int, int> p;
    p.first = (int)(index % x_size) + 1;
    p.second = (int)(index / x_size) + 1;
    return p;
}

int getIndexFromLocation3D(int x, int y, int t, int x_size, int y_size)
{
    return (t * x_size * y_size) + ((y-1) * x_size) + x-1;
}
pi getLocationFromIndex3D(int index, int x_size, int y_size)
{
    pi point3D;
    point3D.second = index / (x_size * y_size);
    index -= (point3D.second * x_size * y_size);
    point3D.first.second = index / x_size + 1;
    point3D.first.first = index % x_size + 1;
    return point3D;
}

int getEuclidHeuristic(int x, int y, int goalposeX, int goalposeY)
{
    return (int)sqrt(((x-goalposeX)*(x-goalposeX) + (y-goalposeY)*(y-goalposeY)));
}



void DijkstraBackwardHeuristic(double *map, int collision_thresh, int x_size, int y_size, int goalposeX, int goalposeY)
{

    Cell target = Cell(goalposeX, goalposeY);
    target.h_value = 0;
    HEURISTIC_DB[GETMAPINDEX(target.x, target.y, x_size, y_size)] = 0;

    OPEN_DB.push(target);

    while(!OPEN_DB.empty())
    {
        Cell s = OPEN_DB.top();
        OPEN_DB.pop();

        int s_idx = GETMAPINDEX(s.x, s.y, x_size, y_size);

        if (EXPANDED_DB[s_idx] == true) continue;

        EXPANDED_DB[s_idx] = true;
        HEURISTIC_DB[s_idx] = s.h_value;

        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = s.x + dX[dir];
            int newy = s.y + dY[dir];
            Cell neighbor_cell = Cell(newx, newy);
            int neighbor_idx = GETMAPINDEX(newx,newy,x_size,y_size);
            if(EXPANDED_DB[neighbor_idx] ==true) continue;
                    
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
            {
                if (((int)map[neighbor_idx] >= 0) && ((int)map[neighbor_idx] < collision_thresh))  //if free
                {
                    
                    int map_value = (int)map[neighbor_idx];
                    int new_cost = s.h_value + map_value;

                    if (neighbor_cell.h_value > new_cost)
                    {
                        HEURISTIC_DB[neighbor_idx] = new_cost;
                        neighbor_cell.h_value = new_cost;
                        OPEN_DB.push(neighbor_cell);
                    }
                }
            }            
        }
    }

}

void DijkstraForward(double *map, int collision_thresh, int x_size, int y_size, int robotposeX, int robotposeY)
{

    Cell target = Cell(robotposeX, robotposeY);
    target.h_value = 0;
    
    HEURISTIC_DF[GETMAPINDEX(target.x, target.y, x_size, y_size)] = 0; //G-VALUES
    ROBOT_STEPS[GETMAPINDEX(target.x, target.y, x_size, y_size)] = target.t_fdijkstra; //T-VALUE
    OPEN_DF.push(target);

    while(!OPEN_DF.empty())
    {
        Cell s = OPEN_DF.top();
        OPEN_DF.pop();

        int s_idx = GETMAPINDEX(s.x, s.y, x_size, y_size);

        if (EXPANDED_DF[s_idx] == true) continue;

        EXPANDED_DF[s_idx] = true;
        HEURISTIC_DF[s_idx] = s.h_value;
        ROBOT_STEPS[s_idx] = s.t_fdijkstra;

        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = s.x + dX[dir];
            int newy = s.y + dY[dir];
            int t = s.t_fdijkstra + 1;
            Cell neighbor_cell = Cell(newx, newy);
            int neighbor_idx = GETMAPINDEX(newx,newy,x_size,y_size);
            if(EXPANDED_DF[neighbor_idx] ==true) continue;
                    
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
            {
                if (((int)map[neighbor_idx] >= 0) && ((int)map[neighbor_idx] < collision_thresh))  //if free
                {
                    
                    int map_value = (int)map[neighbor_idx];
                    int new_cost = s.h_value + map_value;

                    if (neighbor_cell.h_value > new_cost)
                    {
                        HEURISTIC_DF[neighbor_idx] = new_cost;
                        neighbor_cell.t_fdijkstra = t;
                        ROBOT_STEPS[neighbor_idx] = neighbor_cell.t_fdijkstra;
                        neighbor_cell.h_value = new_cost;
                        OPEN_DF.push(neighbor_cell);
                    }
                }
            }            
        }
    }

}



pair<int,int> getOptimalGoal(double *map, int collision_thresh, int x_size, int y_size, int robotposeX, int robotposeY, int target_steps, double *target_traj)
{
    //1. Run Forward Dijkstra
    //2. Get the H_VALUES for all the target trajectory points (cells)
    //3. Loop through all the target cells
    //3.1 Get the H Value
    //3.2 Get the time taken or distance (1 cell/sec) to get to the trajectory points (cells)
    //3.3 Sort the valid time cells in order of minimum cost (H Value + cost of waiting)
    //3.4 Get the top cell and set it as the optimal goal
     
    priority_queue <pi, vector<pi>, g_value_compare_fdijkstra > TARGET_G_VAL;
    pair<int, int> goalPos;
    goalPos.first = (int) target_traj[target_steps-1];
    goalPos.second = (int) target_traj[target_steps -1 + target_steps];
    int time_buffer = 0;
    
    DijkstraForward(map, collision_thresh, x_size, y_size, robotposeX, robotposeY);
    for (int i = 0; i < target_steps; ++i)
    {
        int xtarget = target_traj[i];
        int ytarget = target_traj[i + target_steps];
        int target_idx = GETMAPINDEX(xtarget, ytarget, x_size, y_size);
        int h_val = HEURISTIC_DF[target_idx]; 
        
        Cell target_cell = Cell(xtarget, ytarget);
        int time = ROBOT_STEPS[target_idx];
        target_cell.t_fdijkstra = time;
        
        if (time + time_buffer<= i)
        {
            target_cell.g_value_fdijkstra = h_val + (i - time);
            // cout << "X Target = " << xtarget << " Y Target = " << ytarget << " HVAL = " << target_cell.g_value_fdijkstra << endl;
            pi cell_we_want = make_pair(make_pair(xtarget, ytarget),target_cell.g_value_fdijkstra);
            TARGET_G_VAL.push(cell_we_want);
        }

    }

    pi goalPair = TARGET_G_VAL.top();

    
    goalPos.first = goalPair.first.first;
    goalPos.second = goalPair.first.second;
    
    return goalPos;

}

pi aStar3D(double* map,
                    int collision_thresh,
                    int x_size,
                    int y_size,
                    int robotposeX, 
                    int robotposeY,
                    int goalposeX,
                    int goalposeY,
                    int target_steps,
                    int curr_time,
                    int epsilon)
{

    Cell start_cell = Cell(robotposeX, robotposeY, curr_time);
    int start_2d_idx = GETMAPINDEX(start_cell.x, start_cell.y, x_size, y_size);
    int start_3d_idx = getIndexFromLocation3D(start_cell.x, start_cell.y, start_cell.t, x_size, y_size);

    start_cell.g_value = 0;
    start_cell.f_value = epsilon*HEURISTIC_DB[start_2d_idx];

    OPEN3D.push(start_cell);
    PARENT3D[start_3d_idx] = -1;


    while(!OPEN.empty())
    {
        Cell best = OPEN3D.top();
        OPEN3D.pop();

        int best_3d_idx = getIndexFromLocation3D(best.x, best.y, best.t, x_size, y_size);
        if (CLOSED3D[best_3d_idx] == true) continue;
        
        CLOSED3D[best_3d_idx] = true;

        if (best.x == goalposeX && best.y == goalposeY)
        {
            break;
        }

        for(int dir = 0; dir < NUMOFDIRS + 1; dir++)
        {
            int newx = best.x + dX_3D[dir];
            int newy = best.y + dY_3D[dir];
            int time = best.t + 1;
            Cell neighbor_cell = Cell(newx, newy, time);
            int neighbor_2d_idx = GETMAPINDEX(newx,newy,x_size,y_size);
            int neighbor_3d_idx = getIndexFromLocation3D(newx, newy, time, x_size, y_size);
            if(CLOSED3D[neighbor_3d_idx] ==true) continue;
                    
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && time <= target_steps) //check (curr_time <= target_steps)
            {
                int map_value = (int)map[neighbor_2d_idx];
                if ((map_value >= 0) && (map_value < collision_thresh))  //if free
                {
                    
                    int new_cost = best.g_value + map_value;

                    if (neighbor_cell.g_value > new_cost)
                    {
                        neighbor_cell.g_value = new_cost;
                        // neighbor_cell.f_value = neighbor_cell.g_value + 100*getEuclidHeuristic(neighbor_cell.x, neighbor_cell.y, goalposeX, goalposeY);
                        neighbor_cell.f_value = neighbor_cell.g_value + epsilon*(HEURISTIC_DB[neighbor_2d_idx]);
                        neighbor_cell.t = best.t + 1;

                        OPEN.push(neighbor_cell);
                        PARENT3D[neighbor_3d_idx] = best_3d_idx;
                    }
                }
            }
        }

    } 

    int current_2d_index = GETMAPINDEX(goalposeX,goalposeY,x_size,y_size);
    int current_3d_index = getIndexFromLocation3D(goalposeX, goalposeY, HEURISTIC_DF[current_2d_index], x_size, y_size);

    vector <int> path3D;
    
    while(current_3d_index != start_3d_idx)
    {
        path3D.push_back(current_3d_index);
        astarPath3D.push_back(current_3d_index);
        current_3d_index = PARENT[current_3d_index];
    }
    
    return getLocationFromIndex3D(path3D.back(), x_size, y_size);

}

void aStar(double* map,
                    int collision_thresh,
                    int x_size,
                    int y_size,
                    int robotposeX, 
                    int robotposeY,
                    int goalposeX,
                    int goalposeY,
                    int epsilon)
{

    Cell start_cell = Cell(robotposeX, robotposeY);
    int start_idx = GETMAPINDEX(start_cell.x, start_cell.y, x_size, y_size);
    start_cell.g_value = 0;
    start_cell.f_value = epsilon*HEURISTIC_DB[start_idx];

    OPEN.push(start_cell);
    PARENT[start_idx] = -1;


    while(!OPEN.empty())
    {
        Cell best = OPEN.top();
        OPEN.pop();

        int best_idx = GETMAPINDEX(best.x, best.y, x_size, y_size);
        if (CLOSED[best_idx] == true) continue;
        
        CLOSED[best_idx] = true;

        if (best.x == goalposeX && best.y == goalposeY)
        {
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
                        // //****Uncomment to Use Euclidean Heuristic (Comment out the Dijkstra in the next line)
                        // neighbor_cell.f_value = neighbor_cell.g_value + epsilon*getEuclidHeuristic(neighbor_cell.x, neighbor_cell.y, goalposeX, goalposeY);
                        neighbor_cell.f_value = neighbor_cell.g_value + epsilon*(HEURISTIC_DB[neighbor_idx]);

                        OPEN.push(neighbor_cell);
                        PARENT[neighbor_idx] = best_idx;
                    }
                }
            }
        }

    } 

    int current_index = GETMAPINDEX(goalposeX,goalposeY,x_size,y_size);
    
    while(current_index != start_idx)
    {
        astarPath.push_back(current_index);
        current_index = PARENT[current_index];
    }
    
    return;
}

pair<int,int> greedy(
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

    pair<int,int> p;
    p.first = robotposeX;
    p.second = robotposeY;

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

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];


    bool target_moved;
    if (goalposeX == (int) target_traj[0] && goalposeY == (int) target_traj[target_steps])
    {
        target_moved = false;
    }
    else{
        target_moved = true;
    }

    
    if(!target_moved)
    {
    goal.first = goalposeX;
    goal.second = goalposeY;
    }

    if (curr_time == 0) 
    {

        clock_t time = clock();
        
        if (target_moved) goal = getOptimalGoal(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj);
        
        cout << "GOALX = " << goal.first << " GOALY = " << goal.second << endl;

        // //****Comment if want to use Euclidean Heuristic (Map 6)
        DijkstraBackwardHeuristic(map, collision_thresh, x_size, y_size, goal.first, goal.second);
        
        // // 3D A-star
        // pi nextAction = aStar3D(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, goal.first, goal.second, target_steps, curr_time, 100);
        
        aStar(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, goal.first, goal.second, 100);
        time = clock() - time;
        cout << "Time Taken to Precompute Goal and run A-Star = " << ((float)time)/CLOCKS_PER_SEC << " secs" << endl;

    }

    // //****Uncomment for Greedy
    // pair<int,int> nextAction = greedy(map, collision_thresh, robotposeX, robotposeY, goalposeX, goalposeY, x_size, y_size);

    if (robotposeX == goal.first && robotposeY == goal.second)
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;

        return;   
    }
    

    // //***Uncomment for Greedy (Map5)
    // //***Comment back if not using Greedy
    // action_ptr[0] = nextAction.first;
    // action_ptr[1] = nextAction.second;

    int pos = astarPath.back();
    pair<int,int> action = getLocationFromIndex2D(pos, x_size, y_size);
    astarPath.pop_back();

    // // 3D A-star
    // int pos3D = astarPath3D.back();
    // pi action3D = getLocationFromIndex3D(pos3D, x_size, y_size);
    // astarPath3D.pop_back();
    
    //***2d A star. Can comment out if want to run Greedy
    action_ptr[0] = action.first;
    action_ptr[1] = action.second;

    // // 3D A-star
    // action_ptr[0] = action3D.first.first;
    // action_ptr[1] = action3D.first.second;
    
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