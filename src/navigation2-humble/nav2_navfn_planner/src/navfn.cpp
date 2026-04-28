










































#include "nav2_navfn_planner/navfn.hpp"

#include <algorithm>
#include "rclcpp/rclcpp.hpp"

namespace nav2_navfn_planner
{















NavFn::NavFn(int xs, int ys)
{

  costarr = NULL;
  potarr = NULL;
  pending = NULL;
  gradx = grady = NULL;
  setNavArr(xs, ys);


  pb1 = new int[PRIORITYBUFSIZE];
  pb2 = new int[PRIORITYBUFSIZE];
  pb3 = new int[PRIORITYBUFSIZE];



  priInc = 2 * COST_NEUTRAL;


  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;






  npathbuf = npath = 0;
  pathx = pathy = NULL;
  pathStep = 0.5;
}


NavFn::~NavFn()
{
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }
  if (pathx) {
    delete[] pathx;
  }
  if (pathy) {
    delete[] pathy;
  }
  if (pb1) {
    delete[] pb1;
  }
  if (pb2) {
    delete[] pb2;
  }
  if (pb3) {
    delete[] pb3;
  }
}






void
NavFn::setGoal(int * g)
{
  goal[0] = g[0];
  goal[1] = g[1];
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void
NavFn::setStart(int * g)
{
  start[0] = g[0];
  start[1] = g[1];
  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"), "[NavFn] Setting start to %d,%d\n", start[0],
    start[1]);
}





void
NavFn::setNavArr(int xs, int ys)
{
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Array is %d x %d\n", xs, ys);

  nx = xs;
  ny = ys;
  ns = nx * ny;

  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }

  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }

  costarr = new COSTTYPE[ns];
  memset(costarr, 0, ns * sizeof(COSTTYPE));
  potarr = new float[ns];
  pending = new bool[ns];
  memset(pending, 0, ns * sizeof(bool));
  gradx = new float[ns];
  grady = new float[ns];
}






void
NavFn::setCostmap(const COSTTYPE * cmap, bool isROS, bool allow_unknown)
{
  COSTTYPE * cm = costarr;
  if (isROS) {
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {




        *cm = COST_OBS;
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  } else {
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        *cm = COST_OBS;
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
          continue;
        }
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  }
}

bool
NavFn::calcNavFnDijkstra(bool atStart)
{
  setupNavFn(true);


  return propNavFnDijkstra(std::max(nx * ny / 20, nx + ny), atStart);
}






bool
NavFn::calcNavFnAstar()
{
  setupNavFn(true);


  return propNavFnAstar(std::max(nx * ny / 20, nx + ny));
}





float * NavFn::getPathX() {return pathx;}
float * NavFn::getPathY() {return pathy;}
int NavFn::getPathLen() {return npath;}


#define push_cur(n)  {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && curPe < PRIORITYBUFSIZE) \
    {curP[curPe++] = n; pending[n] = true;}}
#define push_next(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && nextPe < PRIORITYBUFSIZE) \
    {nextP[nextPe++] = n; pending[n] = true;}}
#define push_over(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && overPe < PRIORITYBUFSIZE) \
    {overP[overPe++] = n; pending[n] = true;}}




void
NavFn::setupNavFn(bool keepit)
{

  for (int i = 0; i < ns; i++) {
    potarr[i] = POT_HIGH;
    if (!keepit) {
      costarr[i] = COST_NEUTRAL;
    }
    gradx[i] = grady[i] = 0.0;
  }


  COSTTYPE * pc;
  pc = costarr;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }


  curT = COST_OBS;
  curP = pb1;
  curPe = 0;
  nextP = pb2;
  nextPe = 0;
  overP = pb3;
  overPe = 0;
  memset(pending, 0, ns * sizeof(bool));


  int k = goal[0] + goal[1] * nx;
  initCost(k, 0);


  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++) {
    if (*pc >= COST_OBS) {
      ntot++;
    }
  }
  nobs = ntot;
}




void
NavFn::initCost(int k, float v)
{
  potarr[k] = v;
  push_cur(k + 1);
  push_cur(k - 1);
  push_cur(k - nx);
  push_cur(k + nx);
}










#define INVSQRT2 0.707106781

inline void
NavFn::updateCell(int n)
{

  float u, d, l, r;
  l = potarr[n - 1];
  r = potarr[n + 1];
  u = potarr[n - nx];
  d = potarr[n + nx];





  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}
  if (u < d) {ta = u;} else {ta = d;}


  if (costarr[n] < COST_OBS) {
    float hf = static_cast<float>(costarr[n]);
    float dc = tc - ta;
    if (dc < 0) {
      dc = -dc;
      ta = tc;
    }


    float pot;
    if (dc >= hf) {
      pot = ta + hf;
    } else {



      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }




    if (pot < potarr[n]) {
      float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
      float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
      float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
      float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);
      potarr[n] = pot;
      if (pot < curT) {
        if (l > pot + le) {push_next(n - 1);}
        if (r > pot + re) {push_next(n + 1);}
        if (u > pot + ue) {push_next(n - nx);}
        if (d > pot + de) {push_next(n + nx);}
      } else {
        if (l > pot + le) {push_over(n - 1);}
        if (r > pot + re) {push_over(n + 1);}
        if (u > pot + ue) {push_over(n - nx);}
        if (d > pot + de) {push_over(n + nx);}
      }
    }
  }
}










#define INVSQRT2 0.707106781

inline void
NavFn::updateCellAstar(int n)
{

  float u, d, l, r;
  l = potarr[n - 1];
  r = potarr[n + 1];
  u = potarr[n - nx];
  d = potarr[n + nx];





  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}
  if (u < d) {ta = u;} else {ta = d;}


  if (costarr[n] < COST_OBS) {
    float hf = static_cast<float>(costarr[n]);
    float dc = tc - ta;
    if (dc < 0) {
      dc = -dc;
      ta = tc;
    }


    float pot;
    if (dc >= hf) {
      pot = ta + hf;
    } else {



      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }




    if (pot < potarr[n]) {
      float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
      float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
      float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
      float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);


      int x = n % nx;
      int y = n / nx;
      float dist = hypot(x - start[0], y - start[1]) * static_cast<float>(COST_NEUTRAL);

      potarr[n] = pot;
      pot += dist;
      if (pot < curT) {
        if (l > pot + le) {push_next(n - 1);}
        if (r > pot + re) {push_next(n + 1);}
        if (u > pot + ue) {push_next(n - nx);}
        if (d > pot + de) {push_next(n + nx);}
      } else {
        if (l > pot + le) {push_over(n - 1);}
        if (r > pot + re) {push_over(n + 1);}
        if (u > pot + ue) {push_over(n - nx);}
        if (d > pot + de) {push_over(n + nx);}
      }
    }
  }
}










bool
NavFn::propNavFnDijkstra(int cycles, bool atStart)
{
  int nwv = 0;
  int nc = 0;
  int cycle = 0;


  int startCell = start[1] * nx + start[0];

  for (; cycle < cycles; cycle++) {
    if (curPe == 0 && nextPe == 0) {
      break;
    }


    nc += curPe;
    if (curPe > nwv) {
      nwv = curPe;
    }


    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      pending[*(pb++)] = false;
    }


    pb = curP;
    i = curPe;
    while (i-- > 0) {
      updateCell(*pb++);
    }






    curPe = nextPe;
    nextPe = 0;
    pb = curP;
    curP = nextP;
    nextP = pb;


    if (curPe == 0) {
      curT += priInc;
      curPe = overPe;
      overPe = 0;
      pb = curP;
      curP = overP;
      overP = pb;
    }


    if (atStart) {
      if (potarr[startCell] < POT_HIGH) {
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  return (cycle < cycles) ? true : false;
}










bool
NavFn::propNavFnAstar(int cycles)
{
  int nwv = 0;
  int nc = 0;
  int cycle = 0;


  float dist = hypot(goal[0] - start[0], goal[1] - start[1]) * static_cast<float>(COST_NEUTRAL);
  curT = dist + curT;


  int startCell = start[1] * nx + start[0];


  for (; cycle < cycles; cycle++) {
    if (curPe == 0 && nextPe == 0) {
      break;
    }


    nc += curPe;
    if (curPe > nwv) {
      nwv = curPe;
    }


    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      pending[*(pb++)] = false;
    }


    pb = curP;
    i = curPe;
    while (i-- > 0) {
      updateCellAstar(*pb++);
    }






    curPe = nextPe;
    nextPe = 0;
    pb = curP;
    curP = nextP;
    nextP = pb;


    if (curPe == 0) {
      curT += priInc;
      curPe = overPe;
      overPe = 0;
      pb = curP;
      curP = overP;
      overP = pb;
    }


    if (potarr[startCell] < POT_HIGH) {
      break;
    }
  }

  last_path_cost_ = potarr[startCell];

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  if (potarr[startCell] < POT_HIGH) {
    return true;
  } else {
    return false;
  }
}


float NavFn::getLastPathCost()
{
  return last_path_cost_;
}













int
NavFn::calcPath(int n, int * st)
{




  if (npathbuf < n) {
    if (pathx) {delete[] pathx;}
    if (pathy) {delete[] pathy;}
    pathx = new float[n];
    pathy = new float[n];
    npathbuf = n;
  }



  if (st == NULL) {st = start;}
  int stc = st[1] * nx + st[0];


  float dx = 0;
  float dy = 0;
  npath = 0;


  for (int i = 0; i < n; i++) {

    int nearest_point = std::max(
      0,
      std::min(
        nx * ny - 1, stc + static_cast<int>(round(dx)) +
        static_cast<int>(nx * round(dy))));
    if (potarr[nearest_point] < COST_NEUTRAL) {
      pathx[npath] = static_cast<float>(goal[0]);
      pathy[npath] = static_cast<float>(goal[1]);
      return ++npath;
    }

    if (stc < nx || stc > ns - nx) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Out of bounds");
      return 0;
    }


    pathx[npath] = stc % nx + dx;
    pathy[npath] = stc / nx + dy;
    npath++;

    bool oscillation_detected = false;
    if (npath > 2 &&
      pathx[npath - 1] == pathx[npath - 3] &&
      pathy[npath - 1] == pathy[npath - 3])
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[PathCalc] oscillation detected, attempting fix.");
      oscillation_detected = true;
    }

    int stcnx = stc + nx;
    int stcpx = stc - nx;


    if (potarr[stc] >= POT_HIGH ||
      potarr[stc + 1] >= POT_HIGH ||
      potarr[stc - 1] >= POT_HIGH ||
      potarr[stcnx] >= POT_HIGH ||
      potarr[stcnx + 1] >= POT_HIGH ||
      potarr[stcnx - 1] >= POT_HIGH ||
      potarr[stcpx] >= POT_HIGH ||
      potarr[stcpx + 1] >= POT_HIGH ||
      potarr[stcpx - 1] >= POT_HIGH ||
      oscillation_detected)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);


      int minc = stc;
      int minp = potarr[stc];
      int st = stcpx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc + 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stcnx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      stc = minc;
      dx = 0;
      dy = 0;

      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
        potarr[stc], pathx[npath - 1], pathy[npath - 1]);

      if (potarr[stc] >= POT_HIGH) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, high potential");

        return 0;
      }
    } else {

      gradCell(stc);
      gradCell(stc + 1);
      gradCell(stcnx);
      gradCell(stcnx + 1);



      float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
      float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
      float x = (1.0 - dy) * x1 + dy * x2;
      float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
      float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
      float y = (1.0 - dy) * y1 + dy * y2;

#if 0

      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
        gradx[stc], grady[stc], gradx[stc + 1], grady[stc + 1],
        gradx[stcnx], grady[stcnx], gradx[stcnx + 1], grady[stcnx + 1],
        x, y);
#endif


      if (x == 0.0 && y == 0.0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
        return 0;
      }


      float ss = pathStep / hypot(x, y);
      dx += x * ss;
      dy += y * ss;


      if (dx > 1.0) {stc++; dx -= 1.0;}
      if (dx < -1.0) {stc--; dx += 1.0;}
      if (dy > 1.0) {stc += nx; dy -= 1.0;}
      if (dy < -1.0) {stc -= nx; dy += 1.0;}
    }



  }


  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");

  return 0;
}








float
NavFn::gradCell(int n)
{
  if (gradx[n] + grady[n] > 0.0) {
    return 1.0;
  }

  if (n < nx || n > ns - nx) {
    return 0.0;
  }

  float cv = potarr[n];
  float dx = 0.0;
  float dy = 0.0;


  if (cv >= POT_HIGH) {
    if (potarr[n - 1] < POT_HIGH) {
      dx = -COST_OBS;
    } else if (potarr[n + 1] < POT_HIGH) {
      dx = COST_OBS;
    }
    if (potarr[n - nx] < POT_HIGH) {
      dy = -COST_OBS;
    } else if (potarr[n + nx] < POT_HIGH) {
      dy = COST_OBS;
    }
  } else {

    if (potarr[n - 1] < POT_HIGH) {
      dx += potarr[n - 1] - cv;
    }
    if (potarr[n + 1] < POT_HIGH) {
      dx += cv - potarr[n + 1];
    }


    if (potarr[n - nx] < POT_HIGH) {
      dy += potarr[n - nx] - cv;
    }
    if (potarr[n + nx] < POT_HIGH) {
      dy += cv - potarr[n + nx];
    }
  }


  float norm = hypot(dx, dy);
  if (norm > 0) {
    norm = 1.0 / norm;
    gradx[n] = norm * dx;
    grady[n] = norm * dy;
  }
  return norm;
}



















































}
