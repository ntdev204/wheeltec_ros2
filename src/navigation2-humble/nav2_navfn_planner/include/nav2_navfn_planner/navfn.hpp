





































#ifndef NAV2_NAVFN_PLANNER__NAVFN_HPP_
#define NAV2_NAVFN_PLANNER__NAVFN_HPP_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

namespace nav2_navfn_planner
{


#define COST_UNKNOWN_ROS 255
#define COST_OBS 254
#define COST_OBS_ROS 253











#define COST_NEUTRAL 50
#define COST_FACTOR 0.8




#ifndef COSTTYPE
#define COSTTYPE unsigned char
#endif


#define POT_HIGH 1.0e10


#define PRIORITYBUFSIZE 10000



int create_nav_plan_astar(
  const COSTTYPE * costmap, int nx, int ny,
  int * goal, int * start,
  float * plan, int nplan);



class NavFn
{
public:
  

  NavFn(int nx, int ny);

  ~NavFn();

  

  void setNavArr(int nx, int ny);
  int nx, ny, ns;  

  

  void setCostmap(const COSTTYPE * cmap, bool isROS = true, bool allow_unknown = true);

  

  bool calcNavFnAstar();

  

  bool calcNavFnDijkstra(bool atStart = false);

  

  float * getPathX();

  

  float * getPathY();

  

  int getPathLen();

  

  float getLastPathCost();

  
  COSTTYPE * costarr;  
  float * potarr;  
  bool * pending;  
  int nobs;  

  
  int * pb1, * pb2, * pb3;  
  int * curP, * nextP, * overP;  
  int curPe, nextPe, overPe;  

  
  float curT;  
  float priInc;  

  
  

  void setGoal(int * goal);

  

  void setStart(int * start);

  int goal[2];
  int start[2];
  

  void initCost(int k, float v);

  

  

  void updateCell(int n);

  

  void updateCellAstar(int n);

  

  void setupNavFn(bool keepit = false);

  

  bool propNavFnDijkstra(int cycles, bool atStart = false);

  

  bool propNavFnAstar(int cycles);  

  
  float * gradx, * grady;  
  float * pathx, * pathy;  
  int npath;  
  int npathbuf;  

  float last_path_cost_;  

  

  int calcPath(int n, int * st = NULL);

  

  float gradCell(int n);  

  float pathStep;  

  
  




  
  

};

}

#endif
