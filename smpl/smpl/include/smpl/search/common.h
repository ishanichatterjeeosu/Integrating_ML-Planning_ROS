#ifndef COMMON_H
#define COMMON_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/GridCells.h>
#include <chrono>
#include <ctime>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>



#include <fstream>
#include <sstream>

struct parameters
{
    double vnom;
    bool follow_leader;
    double IDleader;
    double d_f;
    bool follow_wall;
    double x_wall, y_wall;
    double dtime;
    double d_wall;
    std::string dir;
    std::string act_info;
    bool maintain_theta;
    double m_origin_x;
    double m_origin_y;
    double m_res;
    int numcells_x;
    int numcells_y;
    double goal_x;
    double goal_y;
    int wallstp;
    int perstp;
    int gap;
    int tmax;
    int lmax;
};

struct xy_planner_state
{
    int x;
    int y;
    int f;
    int g_itr = -100;
    float g_Q = -100;
    int g;
    int h;
    int besta = -1;
    int un_id = -1;

    xy_planner_state *fp;
    // int min_g_itr = -100; int gQ_of_min_g_itr = -100;
    // int min_g_Q = -100; int gitr_of_min_g_Q = -100;
    //std::unordered_map<int, int> dom;
};
struct xy_planner_state_ppcp
{
    int x;
    int y;
    int f;
    float g_Q = -100;
    int g;
    int h;
    int besta = -1;
    int un_id = -1;

    xy_planner_state_ppcp *fp;
    // int min_g_itr = -100; int gQ_of_min_g_itr = -100;
    // int min_g_Q = -100; int gitr_of_min_g_Q = -100;
    //std::unordered_map<int, int> dom;
};

struct dom
{
    int g_Q;
    int g_iter;
    dom *bp;
};

struct xy_planner_state_dom
{
    // int x;
    // int y;

    int min_g_itr = -100;
    int gQ_of_min_g_itr = -100;
    int min_g_Q = -100;
    int gitr_of_min_g_Q = -100;
    //std::std::vector<std::pair<int, int>> dom;
    dom *end_p;
};

struct bel_state
{
    xy_planner_state sx;
    std::string hx;
    int besta = -1;
    int besta_prev = -1;
    int un_id = -1;
    float v;
    bool was_pivot = false;
    bel_state *bp = NULL;
};
struct bel_state_ppcp
{
    xy_planner_state_ppcp sx;
    std::string hx;
    int besta = -1;
    int besta_prev = -1;
    int un_id = -1;
    float v;
    bool was_pivot = false;
    bel_state_ppcp *bp = NULL;
};

////////////////////////////////////// Some variables local to this node //////////////////////////////
class CommonConstants
{

public:
    // int elapsed_dummy_t = 0;
    // float elapsed_dummy_1_t = 0;

    int T_alloted = 150000;

    ros::Publisher pub_points_path_opt;

    double inf = std::numeric_limits<int>::max();

    int Cfail = 2;
    //int w_inpl = 100; int w_mp = 100; int sc = 10; int T_alloted = 5000; /// just make sure atleast one path can be computed from this

    int w_mp = 1;
    int sc = 10; // for large envs with euclidean dist, sc = 1 for smallenv with manhattan dist

    int numcells_x, numcells_y;
    //int l;

    int mod_E = 1000;

    bool debug_computePath = false; // separate sicnce ts output is very long

    float cmax = 1.4;

    bool visualize = true;

    int obs_cell = 0;

    //time variables

    int t1;

    float getheur2D(int x, int y, int goal_x, int goal_y);
    int getHashKey(int numcells_x, int gx, int gy);
    int getPredMotPrims(xy_planner_state *s, int act, std::vector<int> &dx_, std::vector<int> &dy_, xy_planner_state *s_pred);
    //void visualizeMarkerOpt(double x, double y, int id, int shape, visualization_msgs::MarkerArray &rob_cyl);
};

class CompareSt
{
public:
    bool operator()(xy_planner_state *n1, xy_planner_state *n2)
    {
        return n1->f > n2->f;
    }
};

#endif