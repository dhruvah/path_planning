/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <vector>
#include <chrono>
#include <algorithm>
#include <time.h>
#include <list>

using namespace std;

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}




class State
{
  // Represents information about a particular robot configuration
    public:

    double *angles;
    int configID;
    int componentID = -1;
    // priority_queue<State*, vector<State*>, dist_compare> neighbors;

    vector <State*> neighborhood; //knn
    vector <State*> edges; //edges of graph
    double f_value;
    double g_value;
    State* parent;

    double q_cost = 0;
    double distance;

    State(){};

    State(double *angles)
    {
      this->angles = angles;
    };


};
// struct dist_compare
// {
//   bool operator()(const State* s1, const State* s2)
//   {
//     return s1->distance > s2->distance;
//   }
// };
struct f_value_compare
{
  bool operator()(const State* s1, const State* s2)
  {
    return s1->f_value > s2->f_value;
  }
};

class ProbabilisticRoadmap
{
    public:

    int N = 1000; //number of samples
    double* map;
    int x_size;
    int y_size;
    double* armstart_anglesV_rad;
    double* armgoal_anglesV_rad;
    int numofDOFs;
    double ***plan;
    int *planlength;

    vector <State*> graph;
    vector <State*> path;

    ProbabilisticRoadmap(){}
    ProbabilisticRoadmap(double* map, int x_size, int y_size, int numofDOFs)
    {
      this->map = map;
      this->x_size = x_size;
      this->y_size = y_size;
      this->numofDOFs = numofDOFs;

    }


    //generates a random valid configuration state
    State* RandomConfig()
    {
      double *angles = new double[numofDOFs];
      
      for (int i = 0; i < numofDOFs; i++)
      {
        angles[i] = ((double) rand() / (RAND_MAX))*2*PI;
      }

      State* q = new State(angles);
      return q;
    }

    //Computes Distance between two states
    double Distance(State* &p, State* &q)
    {
      double distance = 0;
      for(int i = 0; i < numofDOFs; ++i)
      {
        cout << "Angles p = " << p->angles[i] << endl;
        distance += (p->angles[i] - q->angles[i])*(p->angles[i] - q->angles[i]);
      }
      return sqrt(distance);
    }

    //Adds Statest in neighborhood to the State
    void Neighborhood(State* &q)
    {
      //priority queue
      //top k
      //neighborhood can just be the priority queue

      double radius = PI/2;
      double dist;
      //i < K-Neighbors
      for(int i = 0; i < graph.size(); i++)
      {
        dist = Distance(q, graph[i]);
        if (dist < radius)
        {
          // radius = dist;
          q->neighborhood.push_back(graph[i]);
        }
      }  
    }

    bool Connect(State *p, State *q)
    {
      double distance = Distance(p, q);
      int numofsamples = (int)(distance/(PI/20));
      if(numofsamples < 2){
        return true;
      }

    for (int i = 0; i < numofsamples; i++){

      double *angles = new double[numofDOFs]; 
      for(int j = 0; j < numofDOFs; j++){
          angles[j] = p->angles[j] + ((double)(i)/(numofsamples-1))*(q->angles[j] - p->angles[j]);
      }
      if(!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size))
      {
        return false;
      }
    }
    return true;    
    
    }

    //Makes the Roadmap
    void RoadMap()
    {
      int i = 0; //0th sample
      int componentID = 0;
      State *q;


      while (i < N)
      {
        q = RandomConfig();
        i = i+1;
        if (!IsValidArmConfiguration(q->angles, numofDOFs, map, x_size, y_size))
        {
          continue;
        }
        q->configID = graph.size();
        graph.push_back(q);
        // i = i + 1;
        Neighborhood(q);
        
        if(q->neighborhood.empty())
        {
          q->componentID = componentID;
          componentID++;
          continue;
        }
        for (int j = 0; j<=q->neighborhood.size(); j++)
        {
          State* n = q->neighborhood[j];
          if(!(q->componentID == n->componentID))
          {
            if(Connect(q, n))
            {
              q->componentID = n->componentID;
              q->edges.push_back(n);
              for(int k; k <= q->edges.size(); k++)
              {
                q->edges[k]->componentID = q->componentID;
              }
            }
          }
        } 
      }
    }

    //returns closest state to input state
    State* ClosestState(State* state)
    {
      State* closest = new State();
      double dist = 10000;
      for(int i = 0; i < graph.size(); i++)
      {
        double distance = Distance(state, graph[i]);
        if(distance < dist)
        {
          dist = distance;
          closest = graph[i];
        }
      }
      return closest;
    }
    //Connect Start and Goal Position to Roadmap
    void ConnectStartandGoal(State *start, State *goal)
    {
      graph.push_back(start);
      State *closest_to_start = ClosestState(start);
      start->componentID = closest_to_start->componentID;
      start->configID = graph.size();
      start->edges.push_back(closest_to_start);
      
      graph.push_back(goal);
      State *closest_to_goal = ClosestState(goal);
      goal->componentID = closest_to_goal->componentID;
      goal->configID = graph.size();
      goal->edges.push_back(closest_to_goal);

    }

    //Query graph using A-star
    void Query(State *start, State *goal)
    {
      double weight = 1;
      priority_queue<State*, vector<State*>, f_value_compare> OPEN;
      unordered_map<int, bool> CLOSED;
      start->g_value = 0;
      start->f_value = -1;

      OPEN.push(start);
      
      while (!OPEN.empty())
      {
        State *best = OPEN.top();
        OPEN.pop();

        if (CLOSED[best->configID] == true) continue;
        CLOSED[best->configID] = true;

        if(best->configID == goal->configID)
        {
          break;
        }
        
        for(int i = 0; i < best->edges.size(); i++)
        {
          State *neighbor = best->edges[i];
          if(CLOSED[neighbor->configID] == true) continue;
          
          double distance = Distance(best, neighbor);
          double new_cost = best->g_value + distance;

          if(neighbor->g_value > new_cost)
          {
            neighbor->g_value = new_cost;
            neighbor->f_value = new_cost + weight*distance;
            OPEN.push(neighbor);
            neighbor->parent = best;
          }

        }

      }

      State *current = goal;
      while(current->configID != start->configID)
      {
        path.push_back(current);
        current = current->parent;
      }
      
      return;
    }

};

class RRTs
{
  public:
  double epsilon = PI/30.0;
  int N = 100000;

  double* map;
  int x_size;
  int y_size;
  double* armstart_anglesV_rad;
  double* armgoal_anglesV_rad;
  int numofDOFs;
  double ***plan;
  int *planlength;  
  
  vector <State*> tree;
  vector <State*> RRTpath;
  vector <State*> Ts, Tg;

  RRTs(){}

  RRTs(double* map, 
       int x_size, 
       int y_size,
       double *armstart_anglesV_rad,
       double *armgoal_anglesV_rad,
       int numofDOFs)
  {
    this->map = map;
    this->x_size = x_size;
    this->y_size = y_size;
    this->armstart_anglesV_rad = armstart_anglesV_rad;
    this->armgoal_anglesV_rad = armgoal_anglesV_rad;
    this->numofDOFs = numofDOFs;
  }
  
  //Computes Distance between two states
  double RRTDistance(State* &p, State* &q)
  {
    double distance = 0;
    for(int i = 0; i < numofDOFs; ++i)
    {
      // cout << "Angles p = " << p->angles[i] << endl;
      distance += (p->angles[i] - q->angles[i])*(p->angles[i] - q->angles[i]);
    }
    return sqrt(distance);
  }

  //returns closest state to input state
  State* RRTClosestState(State* state, vector<State*> &T)
  {
    // cout<< "I am here help 1" << endl;
    State* closest = new State();
    double dist = 10000;
    // cout<< "Tree Size = " << T.size() << endl;
    for(int i = 0; i < T.size(); i++)
    {
      // cout<< "I am here help 2" << endl;

      double distance = RRTDistance(state, T[i]);
      // cout << "Distance = " << distance << endl;
      if(distance < dist)
      {
        dist = distance;
        closest = T[i];
      }
    }
    // cout <<"I am returning closest" << endl;
    return closest;
  }

  State* RRTRandomConfig()
  {
    // cout << "I am in Random Config" << endl;
    double prob = ((double) rand() / (RAND_MAX));
    
    if(prob < 0.90)
    {
      double *angles = new double[numofDOFs];
    
      for (int i = 0; i < numofDOFs; i++)
      {
        angles[i] = ((double) rand() / (RAND_MAX))*2*PI;
      }

      State* q = new State(angles);
      return q;
    }
    else
    {
      State *qgoal = new State(armgoal_anglesV_rad);
      return qgoal;
    }

  }  

  State* NewConfig(State* &qnear, State* &q)
  {
    // cout<<"I am in newconfig" <<endl;
    State* qnew = new State();

    double distance = RRTDistance(qnear, q);
    // cout << "DUSTANCE2 = " << distance << endl;
    // State* qnew = new State();
    int steps = (int)(distance/epsilon);
    // cout << "Steps = " << steps<< endl;
    if(steps < 2)
    {
      // cout << "I return Null" << endl;
      qnew = q;
      return qnew;
    }
    
    

    double *direction = new double[numofDOFs]; 
    for(int j = 0; j < numofDOFs; j++)
      {
          direction[j] = (double)((q->angles[j] - qnear->angles[j])/distance);
          // cout << "Direction = " << direction[j] << endl;
          direction[j] = qnear->angles[j] + epsilon*direction[j]; 
      }
    
    qnew->angles = direction;
    // cout << "DISTTTTTTT = " << RRTDistance(qnew, qnear) << endl;
    // qnew = qnear;
    // cout <<"I am out of New Config" << endl;
    return qnew;
  }



  void BuildRRT(State* &start, State* &end)
  {
     int i = 0;
     start->configID = tree.size();
     tree.push_back(start);
    //  State *goal;

     State *goal;
     
     while(i<N)
     {
      State *qrand = RRTRandomConfig();
      // cout << "ANGLES = " << qrand->angles[0] << endl;
      i = i + 1;
      // cout <<"iter = " << i << endl;
      // if (!IsValidArmConfiguration(qrand->angles, numofDOFs, map, x_size, y_size))
      // {
      //   continue;
      // }

      // qrand->configID = tree.size();

      //Extend
      State *qnear = RRTClosestState(qrand, tree);

      if(qnear->configID == start->configID)
      {
        cout << "LETS GO" << endl;
      }
      // tree.push_back(qnear);
      // qnear->configID = tree.size();
      State *qnew = NewConfig(qnear, qrand);

      if (!IsValidArmConfiguration(qnew->angles, numofDOFs, map, x_size, y_size))
      {
        // cout << "Not Valid Config" << endl;
        continue;
      }

      qnew->configID = tree.size();
      // qnear->edges.push_back(qnew);
      qnew->parent = qnear;
      tree.push_back(qnew);

      if(RRTDistance(qnew, end) <= 0.5)
      {
        // goal = qnew;
        cout << "REACHED" << endl;
        // goal = qnew;
        // end->parent = qnew;
        break;
      }
     }

     //Get as close as possible to qgoal 
     goal = RRTClosestState(end, tree);

     cout << "I am out of WHILE" << endl;
     State *current;
     current = goal;
    //  cout << current->configID << endl;
    //  cout << start->configID << endl;
     while(current->configID != start->configID)
     {
      RRTpath.push_back(current);
      // cout << "In the path WHILE" << endl;

      current = current->parent;
     }

     RRTpath.push_back(start); 
    return;

  }

  //RRT Connect
  bool CONNECT(State* qnew, vector<State*> &T)
  {
    State *qnearG = RRTClosestState(qnew, T);
    bool reached = false;

    while(!reached)
    {
      State *qnewG = NewConfig(qnearG, qnew);
      if (!IsValidArmConfiguration(qnewG->angles, numofDOFs, map, x_size, y_size))
      {
        return false;
        break;
      }
      qnewG->configID = T.size();
      qnewG->parent = qnearG;
      T.push_back(qnewG);

      if(RRTDistance(qnewG, qnew) <= 0.5)
      {
        reached = true;
        break;
      }
      qnearG = qnewG;

    }
    return reached;

  }  
  
  //Build Trees and Path for RRT-Connect
  void BuildRRTConnect(State *start, State *end)
  {
    vector<State*> TsPath;
    vector<State*> TgPath;

    bool start_tree = true;
    int i = 0;
    start->configID = Ts.size();
    end->configID = Tg.size();

    Ts.push_back(start);
    Tg.push_back(end);

    while(i < N)
    {
      State *qrand = RRTRandomConfig();
      i = i + 1;

      //Extend
      State *qnear = RRTClosestState(qrand, Ts);
      State *qnew = NewConfig(qnear, qrand);
      if (!IsValidArmConfiguration(qnew->angles, numofDOFs, map, x_size, y_size))
      {
        continue;
      }
      qnew->configID = Ts.size();
      qnew->parent = qnear;
      Ts.push_back(qnew);
      if(CONNECT(qnew, Tg))
      {
        break;
      }
      swap(Ts, Tg);
      start_tree = !start_tree;
    }
  
    if(!start_tree)
    {
      swap(Ts,Tg);
    }

    //path using Ts
    State *currentS = Ts[Ts.size() - 1];
    while(currentS->configID != start->configID)
    {
      TsPath.push_back(currentS);
      currentS = currentS->parent;
    }
    TsPath.push_back(start);

    //path using Tg
    State *currentG = Tg[Tg.size() - 1];
    while(currentG->configID != end->configID)
    {
      TgPath.push_back(currentG);
      currentG = currentG->parent;
    }
    TgPath.push_back(end);
    reverse(TgPath.begin(), TgPath.end());

    RRTpath.insert(RRTpath.begin(), TgPath.begin(), TgPath.end());
    RRTpath.insert(RRTpath.end(), TsPath.begin(), TsPath.end());
  }

  void NearNeighbors(State* &q, vector<State*> &tree)
  {
    //priority queue
    //top k
    //neighborhood can just be the priority queue
    // vector<State*> neighbors;
    double radius = PI/10;
    double dist;
    //i < K-Neighbors
    for(int i = 0; i < tree.size(); i++)
    {

      if(q->configID == tree[i]->configID)
      {
        continue;
      }
      dist = RRTDistance(q, tree[i]);
      // tree[i]->distance = dist;
      // cout << "DISTANCE NNNNNNNN = " << tree[i]->distance << endl;
      
      // tree[i]->q_cost = tree[i]->q_cost + dist;
      if (dist < radius)
      {
        // radius = dist;
        // q->neighbors.push(tree[i]);
        q->neighborhood.push_back(tree[i]);
        // neighbors.push_back(tree[i]);
      }
    } 
    // return neighbors; 
  }

  bool Connect(State* &p, State* &q)
  {
    double distance = RRTDistance(p, q);
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
      return true;
    }

    for (int i = 0; i < numofsamples; i++){

      double *angles = new double[numofDOFs]; 
      for(int j = 0; j < numofDOFs; j++){
          angles[j] = p->angles[j] + ((double)(i)/(numofsamples-1))*(q->angles[j] - p->angles[j]);
      }
      if(!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size))
      {
        return false;
      }
    }
    return true;    

  }

  void BuildRRTStar(State *start, State *end)
  {
    int i = 0;
    start->configID = tree.size();
    tree.push_back(start);

    State *goal;

    while(i<N)
    {
      State *qrand = RRTRandomConfig();
      i = i + 1;

      //Extend

      State *qnearest = RRTClosestState(qrand, tree);
      State *qnew = NewConfig(qnearest, qrand);

      // qnew->q_cost = qnearest->q_cost + RRTDistance(qnearest, qnew);
      if (!IsValidArmConfiguration(qnew->angles, numofDOFs, map, x_size, y_size))
      {
        continue;
      }

      qnew->configID = tree.size();
      qnew->parent = qnearest;
      tree.push_back(qnew);

      cout << "Working Till HERE" << endl;

      State *qmin = qnearest;
      NearNeighbors(qnew, tree);
      cout << "NEighborhood Size = " << qnew->neighborhood.size() << endl;
      // cout << "NEIGHBORS SIZE = " << neighbors.size() << endl;
      //rewire
      for(int j = 0; j < qnew->neighborhood.size();j++)
      {
        State *qnear = qnew->neighborhood[j];
        // cout << "BABUA DISTANCE = " << qnear->distance << endl;
        // cout << "COSTT = " << qnear->q_cost << endl;
        if(Connect(qnear, qnew))
        {
          double dist = RRTDistance(qnear, qnew);
          cout << "Can Connect" << endl;
          double cost = qnear->q_cost + dist;
          cout << "Cost = " << cost << endl;
          cout << "qnew Cost = " << qnew->q_cost << endl;
          cout << "DISTTT = " << RRTDistance(qnear, qnew) << endl;
          if(cost < qnew->q_cost)
          {
            qnew->q_cost = cost;
            qmin = qnear;
            cout << "New qmin has been Set" << endl;
          }
        }
      }
      cout << "HERE IS WHERE I FAIL 1" << endl;

      qnew->parent = qmin;
      // qnew->q_cost = qmin->q_cost + RRTDistance(qmin, qnew);
      for(int k = 0; k < qnew->neighborhood.size(); k++)
      {
        State *qnear1 = qnew->neighborhood[k];
        cout << "HERE IS WHERE I FAIL" << endl;
        if(qnear1 == qmin) continue;
        // if(qnear1->configID != qmin->configID)
        // {

          if(Connect(qnear1, qnew))
          {
            if(qnear1->q_cost > qnew->q_cost + RRTDistance(qnew, qnear1))
            {
              qnear1->parent = qnew;

            }
          }
        // }
      }
      if(RRTDistance(qnew, end) <= 0.5)
      {
        // goal = qnew;
        cout << "REACHED" << endl;
        // goal = qnew;
        // end->parent = qnew;
        break;
      }

    }

    goal = RRTClosestState(end, tree);

    State *current;
    current = goal;
    while(current->configID != start->configID)
    {
      RRTpath.push_back(current);
      current = current->parent;
    }
    RRTpath.push_back(start);

    return;
  }

};


static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
	     int numofDOFs,
	     double*** plan,
	    int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    
    
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
        {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

static void plannerRRT(
		   double*	map,
		   int x_size,
 		   int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
	     int numofDOFs,
	     double*** plan,
	     int* planlength)
{

  *plan = NULL;
	*planlength = 0;

  RRTs Object = RRTs(map, x_size, y_size, armstart_anglesV_rad ,armgoal_anglesV_rad, numofDOFs);

  State *start = new State(armstart_anglesV_rad);
  State *goal = new State(armgoal_anglesV_rad);

  Object.BuildRRTConnect(start, goal);
  // cout << Object.tree.size() << endl;
  cout << Object.RRTpath.size() << endl;

  int numofsamples = Object.RRTpath.size();

  *plan = (double**) malloc(numofsamples*sizeof(double*));
  for (int i = 0; i < numofsamples; i++){
      (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
      for(int j = 0; j < numofDOFs; j++){
          // (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
          (*plan)[i][j] = Object.RRTpath[numofsamples - i -1]->angles[j];
      }
  }    
  *planlength = numofsamples;

  // *planlength = Objects.RRTpath.size();

}



static void plannerRRTStar(
		   double*	map,
		   int x_size,
 		   int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
	     int numofDOFs,
	     double*** plan,
	     int* planlength)
{

  *plan = NULL;
	*planlength = 0;

  RRTs Object = RRTs(map, x_size, y_size, armstart_anglesV_rad ,armgoal_anglesV_rad, numofDOFs);

  State *start = new State(armstart_anglesV_rad);
  State *goal = new State(armgoal_anglesV_rad);

  Object.BuildRRTStar(start, goal);
  cout << Object.tree.size() << endl;
  cout << Object.RRTpath.size() << endl;

  int numofsamples = Object.RRTpath.size();

  *plan = (double**) malloc(numofsamples*sizeof(double*));
  for (int i = 0; i < numofsamples; i++){
      (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
      for(int j = 0; j < numofDOFs; j++){
          // (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
          (*plan)[i][j] = Object.RRTpath[numofsamples - i -1]->angles[j];
      }
  }    
  *planlength = numofsamples;

  // *planlength = Objects.RRTpath.size();

}


//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    //you can may be call the corresponding planner function here
    if (planner_id == RRT)
    {
       plannerRRT(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }

    if (planner_id == RRTSTAR)
    {
      plannerRRTStar(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    
    //dummy planner which only computes interpolated path
    // planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    
    printf("planner returned plan of length=%d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}





