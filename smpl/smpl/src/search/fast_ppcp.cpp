#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/GridCells.h>

#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <cmath>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <functional>
#include <limits>
#include <vector>

#include <unordered_map>
#include <queue>
#include <algorithm>
#include <ctime>
#include <chrono>
#include <iomanip>

#include <visualization_msgs/MarkerArray.h>

#include <string>

#include <pedsim_msgs/controller_output.h>
#include <fstream>
#include <sstream>
#include <pedsim_simulator/twoD_ppcp.h>
#include <pedsim_simulator/common.h>

//// ///////////////////////////////////////////Simulates 3 behaviours of robot: Wall follow, person follow and maintain a fixed orientation./////////////////////////////////
double g_updateRate, g_simulationFactor;
std::string g_worldFrame, g_robotFrame;
geometry_msgs::Twist g_currentTwist;
tf::Transform g_currentPose;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
std::vector<geometry_msgs::Point> grid_cells;
spencer_tracking_msgs::TrackedPerson p_leader;
spencer_tracking_msgs::TrackedPerson p_gl;
std::vector<spencer_tracking_msgs::TrackedPerson> peds;
pedsim_msgs::controller_output ctr_msg;
ros::Publisher pub_points_policy;
ros::Publisher pub_points_rob;

ros::Publisher pub_points_path;

ros::Publisher pub_points_hidden;
ros::Publisher pub_points_path_del;
ros::Publisher pub_points_st;
ros::Publisher pub_points_gl;
ros::Publisher goal;
ros::Publisher start;
ros::Publisher pub_obs_movAI;
ros::Publisher pub_hidden_movAI;

// Sub opt ppcp parameters
int env_num;
double l;
bool cons_costs_flag = true;
bool debug = false;
bool with_timeout_flag = true;
float tol = 0; // to account for floating point errors
bool exp_small_envs = true;
bool debug_large_env = false;
bool debug_single_hidden_var = false;
bool only_giter = false;
int free_cell = 1;
int unknown_cell = 2;
int num_np_sense = 90000; // |E|
int num_np_move = 1;
bool small_ex_for_paper = true;

int start_x, start_y, goal_x, goal_y;
float p_door_closed;
std::chrono::time_point<std::chrono::system_clock> start_t1, end_t1;
int elapsed_termsubopt_t = 0;
int elapsed_upmdprev_t = 0;
int elapsed_upmdp_t = 0;
int elapsed_comppivnosol_t = 0;
int elapsed_comppiv_t = 0;
int elapsed_compsuboptpath_t = 0;
float elapsed_dominated_t = 0;
int elapsed_expansions_t = 0;
int elapsed_comp_subopt_while_t = 0;
float elapsed_predecessor_comp_t = 0;

// class object to access common constants and params, found in common.h
CommonConstants constants;

boost::mutex mutex;
using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// Structure of params //////////////////////////

struct xy_state
{
    int x;
    int y;
};

struct tmp
{

    unsigned long long int key;
    /*   int x;
    int y;
    int t;*/
    float nr_pt_x;
    float nr_pt_y; //// nearest point on wall
    std::vector<float> interm_x;
    std::vector<float> interm_y;
    int cost;
};

class Compare
{
public:
    bool operator()(xy_planner_state *n1, xy_planner_state *n2)
    {
        return n1->f > n2->f;
    }
};

/////////////////// Comnparator functions //////////////////////////////////////////////

bool comp(const pair<double, int> p1, pair<double, int> p2)
{
    return (p1.first < p2.first);
}
bool same(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    return (((p1.x == p2.x) && (p1.y == p2.y) && (p1.z == p2.z)));
}

bool compmax(const pair<double, int> p1, pair<double, int> p2)
{
    return ((p1.second) > (p2.second));
}

bool unique_fn(tmp p1, tmp p2)
{
    return (p1.key == p2.key);
}
/// Return the point in world coordinates marking the center of the cell at the
/// given effective grid coordinates.
void worldToGrid(
    double world_x, double world_y,
    int &x, int &y, double m_origin_x, double m_origin_y, double m_res)
{
    //std::cout << "m origin x, y" << m_origin_x << "," << m_origin_y << std::endl;
    //std::cout << "m reso, m inv reso" << m_res << "," << m_inv_res << std::endl;
    x = (int)(1 / m_res * (world_x - m_origin_x));
    y = (int)(1 / m_res * (world_y - m_origin_y));

    //std::cout << "x,y, grid coords: " << x << "," << y << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

uint64_t getHashKeyGq(int gx, int gy, int g_Q)
{

    return ((uint64_t)gx | ((uint64_t)gy) << 16 | ((uint64_t)(g_Q)) << 32);
}

void visualizeText(double x, double y, int id, string txt, ros::Publisher pb)
{

    visualization_msgs::Marker st;
    st.header.frame_id = "odom";
    st.header.stamp = ros::Time();

    st.id = id;
    st.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    st.action = visualization_msgs::Marker::ADD;
    st.pose.position.x = x;
    st.pose.position.y = y;
    st.pose.position.z = 0;
    st.pose.orientation.x = 0.0;
    st.pose.orientation.y = 0.0;
    st.pose.orientation.z = 0.0;
    st.pose.orientation.w = 1.0;
    st.scale.x = 2.2;
    st.scale.y = 2.2;
    st.scale.z = 1.2;
    st.color.a = 1; // Don't forget to set the alpha!

    st.text = txt;
    pb.publish(st);
}

void visualizeMarker(double x, double y, int id, int shape, visualization_msgs::MarkerArray &rob_cyl)
{

    visualization_msgs::Marker rob1;
    rob1.header.frame_id = "odom";
    rob1.header.stamp = ros::Time();
    rob1.id = id;
    //id = id+1;
    rob1.type = shape; // 1 for cube
    rob1.action = visualization_msgs::Marker::ADD;
    rob1.pose.position.x = x;
    rob1.pose.position.y = y;
    rob1.pose.position.z = 0;
    rob1.pose.orientation.x = 0;
    rob1.pose.orientation.y = 0;
    rob1.pose.orientation.z = 0;
    rob1.pose.orientation.w = 1;
    rob1.scale.x = 1.2;
    rob1.scale.y = 1.2;
    rob1.scale.z = 1.2;
    rob1.color.a = 1.0; // Don't forget to set the alpha!
    rob1.color.r = 0.0;
    rob1.color.g = 0.0;
    rob1.color.b = 1.0;

    rob_cyl.markers.push_back(rob1);
}

void visualizeMarkerHidden(double x, double y, int id, int shape, visualization_msgs::MarkerArray &rob_cyl)
{

    visualization_msgs::Marker rob1;
    rob1.header.frame_id = "odom";
    rob1.header.stamp = ros::Time();
    rob1.id = id;
    //id = id+1;
    rob1.type = shape; // 1 for cube
    rob1.action = visualization_msgs::Marker::ADD;
    rob1.pose.position.x = x;
    rob1.pose.position.y = y;
    rob1.pose.position.z = 0;
    rob1.pose.orientation.x = 0;
    rob1.pose.orientation.y = 0;
    rob1.pose.orientation.z = 0;
    rob1.pose.orientation.w = 1;
    rob1.scale.x = 1.2;
    rob1.scale.y = 1.2;
    rob1.scale.z = 1.2;
    rob1.color.a = 0.5; // Don't forget to set the alpha!
    rob1.color.r = 0.0;
    rob1.color.g = 1.0;
    rob1.color.b = 0.0;

    rob_cyl.markers.push_back(rob1);
}

int contToDiscTime(float t, double dt)
{

    return floor(t / dt);
}

void bel_succ(const bel_state *X, vector<int> &dx_, vector<int> &dy_, int act, bel_state &X_b, bel_state &X_f, int u_ctr /*,bel_state& X_succ*/)
{

    xy_planner_state S_x;
    xy_planner_state S_xf = X->sx;

    string hx_b;
    hx_b.assign(X->hx);
    string hx_f;
    hx_f.assign(X->hx);

    if (debug)
        cout << "X->sx x,y, un id: " << X->sx.x << "," << X->sx.y << "," << X->sx.un_id << endl;

    if ((X->sx).fp != NULL)
    {
        S_x = *((X->sx).fp);

        if (act == dx_.size() - 1) // if sense action
        {
            // int u_ctr = X->sx.un_id;
            // cout << "g " << X->sx.g << endl;
            // if (X->sx.x == 14 && X->sx.y == 5)
            //     cout << "u_ctr: " << u_ctr << endl;
            //getchar();
            hx_b.replace(u_ctr, 1, "s");
            X_b.sx = S_x;
            X_b.hx.assign(hx_b);
            ////// For failure, remain in same position, no movement. only time increases
            X_f.sx = S_xf;
            hx_f.replace(u_ctr, 1, "f");
            X_f.hx.assign(hx_f);
            if (debug)
            {
                cout << "sense successors:" << endl;
                cout << "Xs, Xf: " << X_b.sx.x << X_b.sx.y << X_b.hx << "," << X_f.sx.x << X_f.sx.y << X_f.hx << endl;
            }
        }
        else
        {
            if (debug)
            {
                cout << "mot prim successor" << endl;
                cout << "S_x, y" << (*((X->sx).fp)).x << "," << (*((X->sx).fp)).y << endl;
            }

            X_b.sx = S_x;
            X_b.hx.assign(X->hx);
        }
    }
}

bool updateMDP(bel_state *X_piv, std::unordered_map<string, bel_state *> &map_bel, std::unordered_map<unsigned long long int, xy_planner_state *> &map_st, vector<int> &dx_, vector<int> &dy_, xy_planner_state S_goal, parameters &param1)
{

    /*  
    if(b == 0) // no path found 
    {   
        unsigned long long int key = getHashKeyTime(X_piv->sx.x, X_piv->sx.y, X_piv->sx.t, constants.numcells_x, param1.numcells_y, 100);
        map_bel[key] = 
    }*/
    unsigned long long int kpiv = constants.getHashKey(constants.numcells_x, X_piv->sx.x, X_piv->sx.y);
    string ks_piv = std::to_string(kpiv) + X_piv->hx;

    if (X_piv->sx.g == constants.inf)
    { /// no path found

        cout << "no path from pivot" << endl;

        map_bel[ks_piv] = X_piv;
        map_bel[ks_piv]->v = constants.inf;

        return 0;
    }

    bel_state *X = X_piv;

    //////////////////////// Compute X^u first //////////////////////////////
    ////// replace all s by u, if f then keep.
    string hu_x;
    hu_x.assign(X_piv->hx);

    for (int i = 0; i < hu_x.length(); i++)
    {
        if (hu_x.find("s") != std::string::npos)
        {
            hu_x.replace(hu_x.find("s"), 1, "u");
        }
    }

    bel_state X_b;
    bel_state X_f;
    int i = 0;

    while (1)
    {

        if ((X->sx.x == S_goal.x) && (X->sx.y == S_goal.y))
        {

            ////// create the goal then break
            bel_state *X_g = new bel_state;
            X_g->sx.x = S_goal.x;
            X_g->sx.y = S_goal.y;

            X_g->hx = X->hx;
            X_g->v = 0;
            unsigned long long int kg = constants.getHashKey(constants.numcells_x, S_goal.x, S_goal.y);
            string ks_g = std::to_string(kg) + X_g->hx;
            map_bel[ks_g] = X_g;

            break;
        }

        i++;
        // if (debug)
        // {
        // cout << "X " << X->sx.x << X->sx.y << X->hx << endl;
        //}

        xy_planner_state S_x = X->sx;
        //cout << S_x.x << S_x.y << S_x.t << endl;
        unsigned long long int key = constants.getHashKey(constants.numcells_x, S_x.x, S_x.y);

        if (debug)
        {
            cout << "un id:" << map_st[key]->un_id << endl;
        }

        string keys = std::to_string(key) + X->hx;

        //// if bel state does not exist, create new and add

        if (map_bel.find(keys) == map_bel.end())
        {
            if (debug)
                cout << "bel state does not exist, create new and add: " << endl;
            bel_state *X_new = new bel_state;
            X_new->sx = S_x;
            // X_new->sx.y = S_x.y;
            // X_new->sx.t = S_x.t;
            X_new->hx = X->hx;
            xy_planner_state S_xtemp = *((X_new->sx).fp);
            if (debug)
                cout << "Xnew fp: " << S_xtemp.x << "," << S_xtemp.y << endl;
            map_bel[keys] = X_new;
        }
        map_bel[keys]->sx.x = map_st[key]->x;
        map_bel[keys]->sx.y = map_st[key]->y;
        map_bel[keys]->sx.fp = map_st[key]->fp; // update front pointer of state sx
        map_bel[keys]->v = map_st[key]->g_Q;    // update only using g_Q values
        map_bel[keys]->besta = map_st[key]->besta;
        map_bel[keys]->un_id = map_st[key]->un_id;
        if (keys == ks_piv)
        {
            map_bel[keys]->was_pivot = true;
        }

        //cout << " g val, besta, unknown var id: " << map_bel[keys]->v << "," << map_bel[keys]->besta << "," << map_bel[keys]->un_id << endl;

        //getchar();

        //update X^u thingy!!! //////
        // string keys_Xu = std::to_string(key) + hu_x;
        // if (map_bel.find(keys_Xu) == map_bel.end())
        // {

        //     //cout << "new xu created" << endl;

        //     bel_state *X_u_new = new bel_state;
        //     X_u_new->sx = S_x;

        //     X_u_new->hx = hu_x;
        //     map_bel[keys_Xu] = X_u_new;
        // }

        // map_bel[keys_Xu]->v = map_st[key]->g_Q; ///// V(X^u) also updated, update only using g_Q values
        // map_bel[keys_Xu]->un_id = map_st[key]->un_id;

        bel_succ(X, dx_, dy_, map_st[key]->besta, X_b, X_f, map_bel[keys]->un_id); // got the best successor

        X = &X_b; //move to best succ
    }

    return 1;
}

bel_state *computePivot(bel_state *X_st, std::unordered_map<string, bel_state *> &map_bel, std::unordered_map<unsigned long long int, xy_planner_state *> map_st, vector<int> &dx_, vector<int> &dy_, xy_planner_state Sg, float &ps, parameters &param1)
{

    ///////////////////// define fifo queue

    std::queue<bel_state *> q_fifo;
    bel_state X_b, X_f;

    if (!q_fifo.empty())
    {
        cout << "ERROR: compute pivot queue isn't empty!!!!!!! debug" << endl;
        exit(0);
    }

    unsigned long long int k_piv = constants.getHashKey(constants.numcells_x, X_st->sx.x, X_st->sx.y);
    string ks_piv = std::to_string(k_piv) + X_st->hx;

    q_fifo.push(map_bel[ks_piv]);
    map_bel[ks_piv]->bp = NULL;

    // cout << "X_st: " << map_bel[ks_piv]->sx.x << "," << map_bel[ks_piv]->sx.y << "," << map_bel[ks_piv]->hx << "," << map_bel[ks_piv]->bp << endl;
    // getchar();

    bel_state *exp = NULL;

    //float dt = 1;

    while (!q_fifo.empty())
    {

        /// pop fifo
        //cout << "fifo queue sze before:" << q_fifo.size() << endl;
        exp = q_fifo.front();
        q_fifo.pop();

        // cout << "exp: " << exp->sx.x << "," << exp->sx.y << "," << exp->hx << "," << exp->bp << endl;
        // getchar();

        if (exp->v == constants.inf) //// expanded state which served as pivot and path not found, just skip to next
        {
            if (debug)
            {
                cout << "expanded state which served as pivot and path not found, just skip to next " << endl;
            }

            continue;
        }

        if (debug)
        {
            cout << "exp " << exp->sx.x << exp->sx.y << exp->hx << endl;
        }

        // if s(goal) exp then done
        if ((exp->sx.x == Sg.x) && (exp->sx.y == Sg.y))
        {
            if (debug)
            {
                cout << "goal expanded before pivot found!!! So end search" << endl;
            }

            //X_pnew = NULL;

            continue;
            //break;
        }

        unsigned long long int k_exp = constants.getHashKey(constants.numcells_x, exp->sx.x, exp->sx.y);
        string ks_exp = std::to_string(k_exp) + exp->hx;

        if (map_bel.find(ks_exp) == map_bel.end() || ((exp->bp != NULL) && (exp->bp->un_id != -1) && (exp->hx[exp->bp->un_id] == 'f') && map_bel[ks_exp]->was_pivot == false) || map_bel[ks_exp]->besta == -1)
        {
            if (constants.debug_computePath)
                cout << "exp bel state doesn't exist...surely a pivot, exp key: " << ks_exp << endl;
            if (exp->bp != NULL)
            {
                //cout << "exp bp best a: " << exp->bp->besta << endl;
                //getchar();
            }
            if (constants.debug_computePath)
                getchar();
            //map_bel[ks_exp]->was_pivot = true;
            return exp;
        }

        //cout << "29,38: " << exp->sx.x << "," << exp->sx.y << "," << map_bel[ks_exp]->hx << "," << map_bel[ks_exp]->besta << endl;

        /// get successors
        if (debug)
        {
            cout << "best a " << exp->besta << endl;
            cout << "un id in computePivot: " << map_bel[ks_exp]->un_id << endl;
        }

        bel_succ(map_bel[ks_exp], dx_, dy_, exp->besta, X_b, X_f, map_bel[ks_exp]->un_id);

        float Ev, vs, vf;

        // compute cost of best successor
        float c_s = constants.sc * sqrt((X_b.sx.x - exp->sx.x) * (X_b.sx.x - exp->sx.x) + (X_b.sx.y - exp->sx.y) * (X_b.sx.y - exp->sx.y));

        // get v value of success bel state
        unsigned long long int ks = constants.getHashKey(constants.numcells_x, X_b.sx.x, X_b.sx.y);
        string ks_succ = std::to_string(ks) + X_b.hx;

        if (map_bel.find(ks_succ) == map_bel.end())
        {
            bel_state *X_sn = new bel_state;
            X_sn->sx = X_b.sx;
            X_sn->hx = X_b.hx;
            X_sn->v = constants.sc * constants.getheur2D(X_sn->sx.x, X_sn->sx.y, Sg.x, Sg.y); //// heur value to goal from exp state, admissible only!
            vs = X_sn->v;
            // update bp of x_b
            X_sn->bp = exp;
            cout << "X_sn bp: " << X_sn->bp << endl;
            //getchar();

            q_fifo.push(X_sn);
        }
        else
        {
            vs = map_bel[ks_succ]->v;
            if (debug)
            {
                cout << "Vs updated: " << vs << endl;
            }
            // update bp of x_b
            map_bel[ks_succ]->bp = exp;
            q_fifo.push(map_bel[ks_succ]);
        }

        if (exp->besta == (dx_.size() - 1)) // if sense action then get V value for failure bel state too
        {

            int c_f = constants.sc * constants.Cfail;
            float vf;

            // Get V value of failure bel state
            unsigned long long int kf = constants.getHashKey(constants.numcells_x, X_f.sx.x, X_f.sx.y);
            string ks_f = std::to_string(kf) + X_f.hx;

            if (map_bel.find(ks_f) == map_bel.end())
            {
                bel_state *X_fn = new bel_state;
                X_fn->sx = X_f.sx;
                X_fn->hx = X_f.hx;
                X_fn->v = constants.sc * constants.getheur2D(X_fn->sx.x, X_fn->sx.y, Sg.x, Sg.y); //// heur value to goal from exp state, admissible only!
                if (debug)
                {
                    cout << "Vf underestimate: " << X_fn->v << endl;
                }

                vf = X_fn->v;
                // update bp of x_f
                X_fn->bp = exp;
                //cout << "X_fn bp: " << X_fn->bp << endl;

                if (debug)
                    cout << "X_fn bp best a: " << X_fn->bp->besta << endl;
                q_fifo.push(X_fn);
            }
            else
            {
                vf = map_bel[ks_f]->v;
                // update bp of x_f
                map_bel[ks_f]->bp = exp;
                q_fifo.push(map_bel[ks_f]);
            }

            // Compute V and check condition
            Ev = ps * (c_s + vs) + (1 - ps) * (c_f + vf);
            if (debug)
            {
                cout << "ps, cs, vs, cf, vf: " << ps << "," << c_s << "," << vs << "," << c_f << "," << vf << endl;
            }
        }
        else // if mprims
        {
            Ev = constants.w_mp * c_s + vs;
        }
        if (debug)
        {
            cout << "exp-> v and ev " << exp->v << "," << Ev << endl;
        }

        //getchar();
        // cout << "fifo queue sze after:" << q_fifo.size() << endl;

        // if (exp->v < Ev)
        // {
        //     if (debug)
        //     {
        //         cout << "exp->v less than Ev" << endl;
        //         cout << "map_bel[ks_exp]->bp->besta, exp->besta: " << map_bel[ks_exp]->bp->besta << "," << exp->besta << endl;
        //     }
        //     // if (exp->besta == dx_.size() - 1)
        //     // {
        //     //     if (debug)
        //     //         cout << "pivot produces stochastic outcome: " << endl;
        //     //     return exp;
        //     // }
        //     // else //if its a mot prim outcome, move on to the next
        //     // {
        //     for (bel_state *s = exp; s; s = s->bp)
        //     {
        //         if (s->bp == NULL)
        //         {
        //             // start state
        //             //cout << "X piv = X st in computePivot: " << s->sx.x << "," << s->sx.y << "," << s->hx << endl;
        //             return s;
        //         }
        //         if (s->bp->besta == dx_.size() - 1)
        //         {
        //             //cout << "s->besta, " << endl;
        //             //cout << "X piv in computePivot: " << s->sx.x << "," << s->sx.y << "," << s->hx << endl;
        //             return s;
        //         }
        //         //cout << "Intermediate Xs: " << s->sx.x << "," << s->sx.y << "," << s->hx << endl;
        //         //cout << "bp " << s->bp << endl;
        //         //getchar();
        //     }
        // }
    }

    return NULL;
}

int getCost(int sx, int sy, int sp_x, int sp_y)
{
    return floor(constants.sc * sqrt((sx - sp_x) * (sx - sp_x) + (sy - sp_y) * (sy - sp_y)));
}
void updateGQStart(xy_planner_state *s, bel_state *X_piv, bel_state *X_st, float &v_hat_Xst,
                   std::unordered_map<string, bel_state *> &map_bel, parameters &param1, vector<int> &dx_, vector<int> &dy_, float &prob_s,
                   float &prob_f, xy_planner_state &S_goal)
{

    if (X_piv->bp != NULL) // X_piv has a parent assigned from computePivot and is not start
    {
        if (constants.debug_computePath)
            cout << "bp x piv" << X_piv->bp->sx.x << "," << X_piv->bp->sx.y << endl;

        bel_state X_pred = *X_piv->bp;
        bel_state *X_pred_p = &X_pred;

        bel_state X = *X_piv;
        bel_state *X_p = &X;

        bel_state X_b, X_f;
        float g_Q_st = s->g_Q;
        ;
        float v_f_i, v_f_i1;

        if (constants.debug_computePath)
            cout << "s->gQ: " << s->g_Q << endl;

        float prob_cum_X_p = 1;

        while (X_pred_p != NULL)
        {

            unsigned long long int key_p = constants.getHashKey(constants.numcells_x, X_pred_p->sx.x, X_pred_p->sx.y);
            string keys_p = std::to_string(key_p) + X_pred_p->hx;

            unsigned long long int key = constants.getHashKey(constants.numcells_x, X_p->sx.x, X_p->sx.y);
            string keys = std::to_string(key) + X_p->hx;

            bel_succ(X_pred_p, dx_, dy_, X_pred_p->besta, X_b, X_f, map_bel[keys_p]->un_id);

            //
            //cout << "xp, x_b, x pred " << X_p->sx.x << "," << X_p->sx.y << X_p->hx << "," << X_b.sx.x << "," << X_b.sx.y << X_b.hx << "," << X_pred_p->sx.x << "," << X_pred_p->sx.y << X_pred_p->hx << endl;

            float c_s = getCost(X_pred_p->sx.x, X_pred_p->sx.y, X_b.sx.x, X_b.sx.y);
            float c_f = constants.sc * constants.Cfail;

            unsigned long long int key_b = constants.getHashKey(constants.numcells_x, X_b.sx.x, X_b.sx.y);
            string keys_b = std::to_string(key_b) + X_b.hx;

            //cout << "keys, key_b" << keys << "," << keys_b << endl;

            // get value

            // replace as V(xf) in prev run with V(xf) in new run
            // if key in map_bel, use that, else use underestimate. Note that current Xf has not been updated with new value n update MDP
            // if (map_bel.find(keys_b) == map_bel.end())
            // {

            //     v_f = constants.sc * constants.getheur2D(X->sx.x, X->sx.y, S_goal.x, S_goal.y);
            // }
            // else
            //     v_f = map_bel[keys]->v;

            // If besta = sto, then use prob_f, else use just the gQ
            if (X_pred_p->besta == (dx_.size() - 1)) // sto action
            {
                unsigned long long int key_f = constants.getHashKey(constants.numcells_x, X_f.sx.x, X_f.sx.y);
                string keys_f = std::to_string(key_f) + X_f.hx;

                //cout << "Xf: " << X_f.sx.x << "," << X_f.sx.y << endl;

                // whichever outcome on branch gets updated g_q_st, other gets value from map_bel if exists, else underestimate

                if (keys == keys_b) // success is on the branch from X_piv
                {

                    prob_cum_X_p = prob_cum_X_p * prob_s;

                    if (map_bel.find(keys_f) == map_bel.end()) // current pivot isn't in map_bel, so only this. Assuming everything above it in the branch exists in map_bel
                    {
                        v_f_i = constants.sc * constants.getheur2D(X_f.sx.x, X_f.sx.y, S_goal.x, S_goal.y);
                    }
                    else
                    {
                        if (abs(map_bel[keys_f]->v - constants.inf) < 10)
                        {
                            //v_hat_Xst = constants.inf;
                            //cout << "v_hat_Xst set to constants.infinity because (map_bel[keys_f]->v = constants.inf" << endl;
                            map_bel[keys_f]->v = constants.sc * constants.getheur2D(X_f.sx.x, X_f.sx.y, S_goal.x, S_goal.y);
                            //return;
                        }
                        else
                        {

                            v_f_i = map_bel[keys_f]->v;

                            //g_Q_st = prob_s * (c_s + g_Q_st) + prob_f * max((c_s + g_Q_st),(c_f + v_f_i));
                            g_Q_st = prob_s * (c_s + g_Q_st) + prob_f * (c_f + v_f_i);
                            //if (constants.debug_computePath)
                            //cout << "cs,  g_Q_st, cf, vf" << c_s << "," << g_Q_st << "," << c_f << "," << v_f_i << endl;

                            //
                        }
                    }

                    // {
                    //     cout << "v xf used: " << v_f_i << endl;
                    // }
                }
                else // failure is on the branch from X_piv
                {
                    //cout << "failure is piv: prob s, cs, vs, cf, vf, " << prob_s << "," << c_s << "," << map_bel[keys_b]->v << "," << c_f << "," << g_Q_st << endl;
                    if (abs(map_bel[keys_b]->v - constants.inf) < 10)
                    {
                        //v_hat_Xst = constants.inf;
                        //cout << "v_hat_Xst set to constants.infinity because (map_bel[keys_f]->v = constants.inf" << endl;
                        //return;
                        map_bel[keys_b]->v = constants.sc * constants.getheur2D(X_f.sx.x, X_f.sx.y, S_goal.x, S_goal.y);
                    }
                    else
                    {

                        g_Q_st = prob_s * (c_s + map_bel[keys_b]->v) + prob_f * (c_f + g_Q_st);
                        prob_cum_X_p = prob_cum_X_p * prob_f;
                        //cout << "prob, prob f cs, vs, cf, g_Q_st" << c_s << "," << map_bel[keys_b]->v << "," << c_f << "," << g_Q_st << endl;
                    }
                }
            }
            else //deter actions
            {

                g_Q_st = c_s + g_Q_st;

                //if (constants.debug_computePath)
                //cout << "gq old, new for deter action: " << X_pred_p->v << "," << g_Q_st << endl;
            }
            X_p->sx = X_pred_p->sx;

            X_p->hx.assign(X_pred_p->hx);
            X_pred_p = X_pred_p->bp;

            if (constants.debug_computePath)
                cout << "prob_cum_X_p: " << prob_cum_X_p << endl;

            //cout << "gq new for deter action: " << g_Q_st << endl;

            // if (X_piv->sx.x == 47 && X_piv->sx.y == 14)
            //     getchar();
        }
        v_hat_Xst = g_Q_st;

        /////////////////////////////////////////////
        unsigned long long int key_piv = constants.getHashKey(constants.numcells_x, X_piv->sx.x, X_piv->sx.y);
        string keys_piv = std::to_string(key_piv) + X_piv->hx;

        // if (map_bel.find(keys_piv) == map_bel.end())
        // {
        //     v_f_i = constants.sc * constants.getheur2D(X_piv->sx.x, X_piv->sx.y, S_goal.x, S_goal.y);
        // }
        // else
        // {
        //     v_f_i = map_bel[keys_piv]->v;
        // }

        // cout << "key piv " << keys_piv << endl;

        // if (constants.debug_computePath)
        //     cout << "v_hat_Xst before: " << v_hat_Xst << endl;
        // v_hat_Xst = v_hat_Xst + prob_cum_X_p * (-v_f_i + s->g_Q);

        //if (constants.debug_computePath)
        // cout << "prob_cum_X_p, map_bel[keys_piv]->v, s->g_Q, new_v_hat_Xst : " << prob_cum_X_p << "," << v_f_i << "," << s->g_Q << "," << v_hat_Xst << endl;
        // getchar();
        // if (constants.debug_computePath)
        // {
        //     cout << "g_Q_st: start" << g_Q_st << endl;
        //     cout << "x piv is start with null pointer, gQ: " << X_piv->sx.x << "," << X_piv->sx.y << "," << s->g_Q << endl;
        //     //getchar();
        // }

        // return g_Q_st;
        //return new_v_hat_Xst;
    }
    else
    {
        if (constants.debug_computePath)
            cout << "x piv is start with null pointer, gQ: " << X_piv->sx.x << "," << X_piv->sx.y << "," << s->g_Q << endl;
        v_hat_Xst = s->g_Q;
        //getchar();
        //return s->g_Q;
    }
}
bool TerminateSubOptPath(bel_state *X_st, xy_planner_state *s, bel_state *X_piv, int opt_gQ, float &min_gQ_p, float &v_hat_Xst, double lambda, string keys_st, unsigned long long int key_piv,
                         std::unordered_map<unsigned long long int, xy_planner_state *> &map_st,
                         std::unordered_map<string, bel_state *> &map_bel, std::unordered_map<unsigned long long int, int> &map_un, parameters &param1, vector<int> &dx_, vector<int> &dy_, float &prob_s,
                         float &prob_f, xy_planner_state &S_goal)
{

    //cout << "S(Xp) pivot expanded yayyyyy" << endl;
    //cout << "path cost " << s->g <0< endl;
    //cout << "x piv, x, y: " << X_piv->sx.x << "," << X_piv->sx.y << endl;

    // check if GQ is within lambda.gQ* then terminate else continue
    //if (constants.debug_computePath)
    //cout << "g_itrs, gQ, lambda*gQ*" << s->g_itr << "," << s->g_Q << "," << lambda * opt_gQ << endl;
    //getchar();

    float g_Q_st = v_hat_Xst; // store the initial v hat Xst

    updateGQStart(s, X_piv, X_st, v_hat_Xst, map_bel, param1, dx_, dy_, prob_s,
                  prob_f, S_goal); // find updated

    //cout << "g_iters, gQ, V_hat_xst, lambda*gQ*, gQ*" << s->g_itr << "," << s->g_Q << "," << (int)v_hat_Xst << "," << lambda * opt_gQ << "," << opt_gQ << endl;

    std::unordered_map<unsigned long long int, bool> map_tmp;

    // for (xy_planner_state *s1 = s; s1; s1 = s1->fp)
    // {
    //    cout << s1->x << "," << s1->y << "f = " << s1->f << "g_itr, g_Q, g = " << s1->g_itr << "," << s1->g_Q << "," << s1->g << " h = " << s1->h << endl;

    // }
    //getchar();

    if ((int)v_hat_Xst > 0 && (int)v_hat_Xst <= (lambda * opt_gQ) + tol)
    {
        // bounded subopt solution found

        // check if solution doesnt have repeated hidden vars
        //PathHasRepeatedHiddenVars();

        bool repeated_hidden = false;
        for (xy_planner_state *s1 = s; s1; s1 = s1->fp)
        {
            unsigned long long int key_path = constants.getHashKey(constants.numcells_x, s1->x, s1->y);

            //cout << s1->x << "," << s1->y << "f = " << s1->f << "g_itr, g_Q, g = " << s1->g_itr << "," << s1->g_Q << "," << s1->g << " h = " << s1->h << endl;

            map_st[key_path] = s1;
            std::unordered_map<unsigned long long int, int>::const_iterator itr_hidden = map_un.find(key_path);
            if (itr_hidden != map_un.end()) //hidden state
            {
                if (map_tmp.find(key_path) != map_tmp.end())
                {
                    repeated_hidden = true;
                    break;
                }
                else
                {
                    map_tmp[key_path] = true;
                }
            }

            if (s1->fp == NULL)
            {

                break;
            }
        }
        map_tmp.clear();

        bool repeated_state = false;
        map_st.clear();
        for (xy_planner_state *s1 = s; s1; s1 = s1->fp)
        {
            unsigned long long int key_path = constants.getHashKey(constants.numcells_x, s1->x, s1->y);
            if (map_st.find(key_path) == map_st.end())
            {
                map_st[key_path] = s1;
            }
            else
            {
                repeated_state = true;
                break;
            }

            if (s1->fp == NULL)
            {

                break;
            }
        }

        if (repeated_state == false && repeated_hidden == false) // only then return path
        {
            //store the min g_Q for use if no solution found from thos pivot
            min_gQ_p = std::min(min_gQ_p, s->g_Q);
            //cout << "min g q: " << min_gQ_p << endl;
            X_piv->sx = *s;
            //closed[key_piv] = 1;
            xy_planner_state *S_piv = &(X_piv->sx);
            map_st[key_piv] = S_piv;
            //if (constants.debug_computePath)
            //cout << "computepath from pivot terminated, g_iter, g_Q" << s->g_itr << "," << s->g_Q << endl;
            //cout << "V_(Xst) using g_Q for this iteration: " << v_hat_Xst << endl;
            //getchar();
            // assign in bel map the gQ(X_st)
            if (map_bel.find(keys_st) != map_bel.end())
            {
                map_bel[keys_st]->v = v_hat_Xst;
            }

            return true;
        }
        else
        {
            //cout << "Repeated hidden detected: g_itrs, gQ_st, lambda*gQ*" << s->g_itr << "," << v_hat_Xst << "," << lambda * opt_gQ << endl;
            v_hat_Xst = g_Q_st;
            //getchar();
            return false;
        }
    }
    else
    {
        //store the min g_Q for use if no solution found from thos pivot
        min_gQ_p = std::min(min_gQ_p, s->g_Q);
        //cout << "min g q: " << min_gQ_p << endl;
        //if (constants.debug_computePath)
        // reset v hat Xst to initial value
        //cout << "g_itrs, gQ_st, lambda*gQ*" << s->g_itr << "," << v_hat_Xst << "," << lambda * opt_gQ << endl;
        v_hat_Xst = g_Q_st;

        return false;
    }
}
int computeSuboptPath(bel_state *X_st, bel_state *X_piv, int opt_gQ, float &min_gQ_p, float &v_hat_Xst, vector<int> &dx_, vector<int> &dy_,
                      std::unordered_map<string, bel_state *> &map_bel, std::unordered_map<unsigned long long int, xy_planner_state *> &map_st, xy_planner_state_dom **map_st_temp,
                      xy_planner_state S_goal, parameters &param1, float &prob_s,
                      float &prob_f, unsigned int **grid2D, std::unordered_map<unsigned long long int, int> &map_un, double lambda, int &tot_expansions)
{

    //Ppcp::hello();
    //getchar();
    // auto start_comp_subopt_while = std::chrono::system_clock::now();
    //clear map_st_temp
    for (int tx = 0; tx < constants.numcells_x; tx++)
    {
        for (int ty = 0; ty < constants.numcells_y; ty++)
        {
            map_st_temp[tx][ty].end_p = NULL;
        }
    }

    X_piv->sx.g = constants.inf;
    //X_piv->sx.x = 33; X_piv->sx.y = 19;
    //cout << "constants.inf: " << std::numeric_limits<double>::max() << endl;
    xy_planner_state *Sg = new xy_planner_state;
    xy_planner_state_dom *Sg_dom = new xy_planner_state_dom;
    xy_planner_state *exp = NULL;
    Sg->x = S_goal.x;

    Sg->y = S_goal.y;

    Sg->g = 0;
    Sg->g_itr = 0;
    Sg->g_Q = 0;
    Sg->besta = -1;
    Sg->h = constants.sc * constants.getheur2D(X_piv->sx.x, X_piv->sx.y, Sg->x, Sg->y);
    Sg->f = Sg->g + Sg->h;
    unsigned long long int keyg = constants.getHashKey(constants.numcells_x, Sg->x, Sg->y);

    Sg->fp = NULL;
    //cout << "key g" << keyg << endl;
    // map_st[keyg] = Sg;
    /*float prob_s = 0.8;
    float prob_f = 0.2;*/

    /////////////////////////////////////////// make priorty queue here ////////////////////////////////
    std::priority_queue<xy_planner_state *, vector<xy_planner_state *>, CompareSt> heap; ///// for s(X) search
    heap.push(Sg);

    //std::unordered_map<unsigned long long int, int> closed;
    //std::unordered_map<unsigned long long int, xy_planner_state_dom *> map_st_temp;
    // insert goal into dominance list

    map_st_temp[Sg->x][Sg->y].min_g_itr = 0;
    map_st_temp[Sg->x][Sg->y].gitr_of_min_g_Q = 0;
    map_st_temp[Sg->x][Sg->y].min_g_Q = 0;
    map_st_temp[Sg->x][Sg->y].gQ_of_min_g_itr = 0;
    //map_st_temp[Sg->x][Sg->y].dom.push_back(std::make_pair(0,0));

    //////////////////////// Compute X^u first //////////////////////////////
    ////// replace all s by u, if f then keep.
    string hu_x_s_holder;
    hu_x_s_holder.assign(X_piv->hx);

    // for (int i = 0; i < hu_x_s_replaced.length(); i++)
    // {
    //     if (hu_x_s_replaced.find("s") != std::string::npos)
    //     {
    //         hu_x_s_replaced.replace(hu_x_s_replaced.find("s"), 1, "u");
    //     }
    // }
    // string hu_x_s_replaced_cpy;
    // hu_x_s_replaced_cpy.assign(hu_x_s_replaced);

    bel_state Y_f;

    //cout << "size of map st " << map_st.size() << endl;
    int xt = 0;

    /*visualization_msgs::MarkerArray rob_cyl;*/

    //cout << "x pivot value " << X_piv->sx.g << endl;

    unsigned long long int key_piv = constants.getHashKey(constants.numcells_x, X_piv->sx.x, X_piv->sx.y);
    unsigned long long int key_st = constants.getHashKey(constants.numcells_x, X_st->sx.x, X_st->sx.y);
    string keys_st = std::to_string(key_st) + X_st->hx; /// v value of failure state

    min_gQ_p = constants.inf;
    std::unordered_map<int, int>::iterator it;

    while (1)
    {

        /*unsigned long long int key;
       do 
       {
        exp = heap.top();
        heap.pop();
        key = getHashKeyTime(exp->x, exp->y, exp->t, constants.numcells_x, param1.numcells_y, 100);
        cout << "stuck here" << endl;
       } while (map_st.find(key) != map_st.end());

      */

        if (heap.empty() == 1)
        {

            cout << "heap is empty. Path not found" << endl;

            // save min_gQ_p as the v value
        cout << "min g q: " << min_gQ_p << endl;
            X_piv->sx.g = min_gQ_p;
            // reset besta of pivot to -1
            X_piv->besta = -1;
            //X_piv->sx.g = constants.inf;

            return 200; // code for no solution found
            //break;
        }

        end_t1 = std::chrono::system_clock::now();
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_t1 - start_t1);

        constants.t1 = (int)elapsed.count();

        if (with_timeout_flag)
        {
            if (constants.t1 > constants.T_alloted)
            { /////////////////////// If time over, then return immediately
                return 100;
            }
        }
        //auto start_expansions = std::chrono::system_clock::now();
        exp = heap.top();
        //cout << "exp x, y, t, best a: " << exp->x << "," << exp->y <<  "," << exp->besta << endl << "f = " << exp->f << " g = " << exp->g << " h = " << exp->h << endl;

        heap.pop();
        // auto end_expansions = std::chrono::system_clock::now();
        // auto elapsed_expansions =
        //     std::chrono::duration_cast<std::chrono::microseconds>(end_expansions - start_expansions);
        // elapsed_expansions_t += (int)elapsed_expansions.count();

        //cout << "size of heap: " << heap.size() << endl;
        // if (heap.size() == 0)
        // {
        //     cout << "size of heap: " << heap.size() << endl;
        // }

        //exp->closed = true;

        xy_planner_state *s = new xy_planner_state;
        s = exp;

        if (constants.debug_computePath)
        {
            cout << "s,exp address: " << s << "," << exp << endl;
        }

        tot_expansions++;
        // if (tot_expansions == 197136)
        // {
        //     //cout << "           cumulated predecessor comp time (ms): " << elapsed_predecessor_comp_t / 1000 << endl;

        //     cout << " cumulated computesubopt while loop (ms): " << elapsed_comp_subopt_while_t / 1000 << endl;
        //     getchar();
        // }

        /*visualizeMarker(exp->x, exp->y, exp_no, 1,  rob_cyl);*/

        //uint64_t key_exp_gQ = constants.getHashKeyGq(s->x, s->y, s->g_Q);
        unsigned long long int key_exp = constants.getHashKey(constants.numcells_x, s->x, s->y);

        //map_st[key_exp] = s; // real key

        if (constants.debug_computePath)
        {
            cout << "exp x, y, g_itr, g_Q, g, h, f " << s->x << "," << s->y << "," << s->g_itr << "," << s->g_Q << "," << s->g << "," << s->h << "," << s->f << endl;
        }
        //getchar();

        if (constants.debug_computePath)
        {
            cout << "piv x, y " << X_piv->sx.x << "," << X_piv->sx.y << endl;
        }

        // Termination condition ////////////////////////////////////////
        if (s->x == X_piv->sx.x && s->y == X_piv->sx.y) // if exp state = pivot
        {
            //cout << "s->gitr" << s->g_itr << endl;
            //auto start_termsubopt = std::chrono::system_clock::now();
            bool term_flag = TerminateSubOptPath(X_st, s, X_piv, opt_gQ, min_gQ_p, v_hat_Xst, lambda, keys_st, key_piv, map_st, map_bel, map_un, param1, dx_, dy_, prob_s,
                                                 prob_f, S_goal);
            // auto end_termsubopt = std::chrono::system_clock::now();
            // auto elapsed_termsubopt =
            //     std::chrono::duration_cast<std::chrono::microseconds>(end_termsubopt - start_termsubopt);
            // elapsed_termsubopt_t = elapsed_termsubopt_t + (int)elapsed_termsubopt.count();

            //getchar();

            if (term_flag)
            {
                break;
            }
            else
            {

                continue; // expand again
            }
        }

        // auto start_dummy = std::chrono::system_clock::now();

        // int a = 0;
        // for (int i = 0; i < 1000; i++)
        // {
        //     auto start_dummy_1 = std::chrono::system_clock::now();
        //     a = a + 1;
        //     auto end_dummy_1 = std::chrono::system_clock::now();
        //     auto elapsed_dummy_1 =
        //         std::chrono::duration_cast<std::chrono::microseconds>(end_dummy_1 - start_dummy_1);
        //     elapsed_dummy_1_t = elapsed_dummy_1_t + (float)elapsed_dummy_1.count();
        // }

        // auto end_dummy = std::chrono::system_clock::now();
        // auto elapsed_dummy =
        //     std::chrono::duration_cast<std::chrono::microseconds>(end_dummy - start_dummy);
        // elapsed_dummy_t = (int)elapsed_dummy.count();

        // cout << "sum of inside for: " << elapsed_dummy_1_t << endl;
        // cout << "sum of outside for: " << elapsed_dummy_t << endl;
        // getchar();

        //

        // unsigned long long int key_tmp = constants.getHashKey(constants.numcells_x, 53, 19);
        // unsigned long long int key_tmp2 = constants.getHashKey(constants.numcells_x, 50, 19);
        // unsigned long long int key_tmp3 = constants.getHashKey(constants.numcells_x, 53,21);
        // unsigned long long int key_goal = constants.getHashKey(constants.numcells_x, 56,20);
        // cout << "keys: 53,19, 50,19, 53, 21" << key_tmp << "," << key_tmp2 << "," << key_tmp3 << endl;

        //std::unordered_map<unsigned long long int, int>::const_iterator cls_it_exp = closed.find(key_exp);
        //cout << "closed list 53, 19, 50,19, 53, 21 status " << closed[key_tmp] << "," << closed[key_tmp2] << "," << closed[key_tmp3];
        // cout << "key exp: " << key_exp << endl;
        // cout << "closed list goal 56,20 status: " << closed[key_goal] << endl;
        // cout << "closed list size:" << closed.size() << endl;
        // getchar();
        // not allowng re-expansions
        // if (cls_it_exp != closed.end())
        // {
        //     cout << "EXP state should not exist in the CLOSED LIST, ERROR!!!!!" << endl;
        //     exit(0);
        // }

        // closed[key_exp] = 1;
        // map_st[key_exp] = s;

        // if (cls_it_exp == closed.end()) //
        // {
        //     closed[key_exp] = 1;

        //     map_st[key_exp] = s; // real key
        // }
        // else
        // {
        // if (constants.debug_computePath)
        // {
        //     cout << "because of STL heap, duplicate copies inserted before expanded, so just move to next" << endl;
        // }
        // continue;
        //}

        //cout << "crossed this before generate succ" << endl;

        //map_st[key]->un_id = s->un_id;
        // if(exp->x == 14 && exp->y == 5)
        // {
        //     cout << "u id: while putting in closed: " << map_st[key]->un_id << endl;
        //     getchar();
        // }
        // }
        // else
        //     continue;

        float q;

        bool fl_u = false;
        int u_ctr = -1;

        std::unordered_map<unsigned long long int, int>::const_iterator got = map_un.find(key_exp);

        if (got != map_un.end() && (X_piv->hx[got->second] == 'u')) // If exp state is unknown, then only sense applies
        {
            u_ctr = got->second;
            // if (constants.debug_computePath)
            // {
            //     cout << "unknown variable to sense: " << u_ctr << endl;
            // }
            //auto start_predecessor_comp = std::chrono::system_clock::now();
            for (int a = 0; a < dx_.size() - 1; a++) // get all 8 valid s_preds
            {

                xy_planner_state *s_pred = new xy_planner_state;
                float c_s = constants.getPredMotPrims(s, a, dx_, dy_, s_pred);
                float c_f = constants.sc * constants.Cfail;
                //check valid state
                // if (!isValidState(s_pred, param1, closed, grid2D))
                //     continue;

                if (s_pred->x < 0 || s_pred->x >= constants.numcells_x || s_pred->y < 0 || s_pred->y >= param1.numcells_y)
                {
                    if (constants.debug_computePath)
                    {
                        cout << " s_pred out of bounds, move to next action " << endl;
                    }

                    continue;
                }

                ////////////// static collision check
                else if (grid2D[s_pred->x][s_pred->y] == constants.obs_cell)
                {
                    // if (s->x == 25 && s->y == 28)
                    //     cout << "wall identify" << endl;
                    if (constants.debug_computePath)
                    {
                        cout << " s_pred " << s_pred->x << "," << s_pred->y << "in collision, move to next action " << endl;
                        //getchar();
                    }

                    continue;
                }

                // key for closed list has gQ, for this subotpimal search correct termination
                unsigned long long int key_pred = constants.getHashKey(constants.numcells_x, s_pred->x, s_pred->y);
                // std::unordered_map<unsigned long long int, int>::const_iterator cls_it_pred = closed.find(key_pred);
                // if (cls_it_pred != closed.end())
                // {

                //     if (constants.debug_computePath)
                //     {
                //         cout << " s_pred in closed, move to next action " << endl;
                //     }

                //     continue;
                // }
                if (constants.debug_computePath)
                {
                    cout << "s pred sense: " << s_pred->x << "," << s_pred->y << endl;
                }

                //real key for use in updateMD, computepivot, hidden variable matchign

                std::unordered_map<unsigned long long int, int>::const_iterator gotp1 = map_un.find(key_pred);
                if (gotp1 != map_un.end()) //pred is hidden
                {
                    int u_ctr1 = gotp1->second;
                    //cout << "This case should never come if there are no adjacent hidden vars" << endl;
                    if (X_piv->hx[u_ctr1] == 'f') // if it is hidden and known as "f" in
                    {

                        continue;
                    }
                }

                // Get V value of failure bel state V^f
                // string h_u_yf;
                // //cout << "hu_x: " << hu_x << endl;
                // h_u_yf.assign(hu_x_s_replaced.replace(u_ctr, 1, "f")); ///// Ask max H^u(Xp)??
                // // // restore it back
                // hu_x_s_replaced.assign(hu_x_s_replaced_cpy);

                // // restore it back to pivot
                //
                string h_u_yf;
                h_u_yf.assign(hu_x_s_holder.replace(u_ctr, 1, "f"));

                // // restore it back to pivot
                hu_x_s_holder.assign(X_piv->hx);

                unsigned long long int key_f = constants.getHashKey(constants.numcells_x, s_pred->x, s_pred->y); // S(X)  upon failure remain at same state s_pred

                string keys = std::to_string(key_f) + h_u_yf; /// v value of failure state
                float v_f;

                if (constants.debug_computePath)
                {
                    cout << "v f string" << h_u_yf << endl;
                }

                //// if doesn't exist, v = admissible heuristic to GOAL!
                if (map_bel.find(keys) == map_bel.end())
                {
                    // if (constants.debug_computePath)
                    // {
                    //     cout << "keys not updated: " << keys << endl;
                    // }

                    v_f = constants.sc * constants.getheur2D(s_pred->x, s_pred->y, Sg->x, Sg->y); //// heur value to goal from exp state, admissible only!

                    // if (s_pred->x == 45 && s_pred->y == 52)
                    // {
                    //cout << "used v of Xf underestimate: state " << s_pred->x << "," << s_pred->y << h_u_yf << "," << keys << "," << v_f << "," << endl;
                    // }
                }
                else
                {
                    if (abs(map_bel[keys]->v - constants.inf) < 10) // invalid v path not found, just skip this predecessor
                    {
                        //     if (s_pred->x == 45 && s_pred->y == 52)
                        // {
                        //cout << "used Vf: state " << s_pred->x << "," << s_pred->y << h_u_yf << "," << keys << "," << map_bel[keys]->v << "," << endl;
                        //getchar();
                        //}
                        continue;
                    }
                    else
                    {

                        v_f = map_bel[keys]->v;
                        //cout << "used v of Xf from bel map: state " << s_pred->x << "," << s_pred->y << h_u_yf << "," << keys << "," << v_f << "," << endl;
                    
                    }

                    // if (debug)
                    //cout << "used v of Xf from bel map: state " << s_pred->x << "," << s_pred->y << h_u_yf << "," << keys << "," << v_f << "," << endl;
                    //getchar();
                }

                // Compute g_Q
                if ((X_piv->hx[u_ctr] == 'u') || (X_piv->hx[u_ctr] == 's'))
                {

                    //cout << "action c : " << a << "," << c << endl;

                    //q = prob_s * (c_s + s->g_Q) + (1 - prob_s) * max((c_s + s->g_Q), (c_f + v_f)); ///// cost for failure is still c, because time loss
                    q = prob_s * (c_s + s->g_Q) + (1 - prob_s) * (c_f + v_f);

                    if (constants.debug_computePath)
                    {
                        cout << "q, p s, c s, s->g, c f, v_f" << q << "," << prob_s << "," << c_s << "," << s->g_Q << "," << c_f << "," << v_f << endl;
                    }
                }
                else
                {
                    cout << "should never come here, exit!!!!!!!!!!!!!!!!!" << endl;
                    exit(0);
                }

                // Push predecessor
                // Also compute g_itr and update g value as sum of g_its and g_Q

                s_pred->g_Q = q; // g_Q

                s_pred->g_itr = exp->g_itr + constants.sc * constants.mod_E; // g_itr

                s_pred->besta = dx_.size() - 1; // sense action
                s_pred->un_id = u_ctr;

                s_pred->h = constants.sc * constants.getheur2D(s_pred->x, s_pred->y, X_piv->sx.x, X_piv->sx.y);

                s_pred->f = s_pred->g_itr + s_pred->h; // minimize g_iter, given cost setting of |E| and 1, euclidean heuristic is admissible

                s_pred->fp = s;

                if (constants.debug_computePath)
                {
                    cout << "s pred not hidden, g_itr, g_Q, g value, h value, f value: " << s_pred->x << "," << s_pred->y << "," << s_pred->g_itr << "," << s_pred->g_Q << "," << s_pred->g << "," << s_pred->h << "," << s_pred->f << endl;
                    cout << "s pred memory being pushed in heap: " << s_pred << endl;
                }

                //////////// check if dominated
                //
                //std::unordered_map<unsigned long long int, xy_planner_state_dom *>::const_iterator dom_it = map_st_temp.find(key_pred);
                bool dom_flag = false;
                //  cout << "min gQ,  curr gq and min gitr, curr gitr of keypred: " << key_pred << "," << map_st_temp[s_pred->x][s_pred->y].min_g_Q << "," << s_pred->g_Q << "," << map_st_temp[s_pred->x][s_pred->y].min_g_itr
                //  << "," << s_pred->g_itr << endl;
                if (map_st_temp[s_pred->x][s_pred->y].end_p == NULL) // // first entry
                {

                    map_st_temp[s_pred->x][s_pred->y].min_g_itr = s_pred->g_itr;
                    map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr = s_pred->g_Q;
                    map_st_temp[s_pred->x][s_pred->y].min_g_Q = s_pred->g_Q;
                    map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q = s_pred->g_itr;
                    //assign first elem to null
                }
                else
                {
                    if ((s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].min_g_Q) || (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].min_g_itr))
                    {

                        //update the min gQ and corr g_itr
                        if (s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].min_g_Q)
                        {
                            // if (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q)
                            // {
                            //     //remove the dominated state from dom list
                            //     map_st_temp[s_pred->x][s_pred->y].dom.erase(map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q);
                            // }
                            map_st_temp[s_pred->x][s_pred->y].min_g_Q = s_pred->g_Q;
                            map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q = s_pred->g_itr;
                            //cout << "g Q less than min g itr" << endl;
                            //         // if(map_st_temp[s_pred->x][s_pred->y].dom.size()>2)
                            //         // cout << "min gQ of keypred: " << key_pred << "," << map_st_temp[s_pred->x][s_pred->y].min_g_Q  <<endl;
                            //         auto end_dominated_1 = std::chrono::system_clock::now();
                            //  auto elapsed_dominated_1 =
                            //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated_1 - start_dominated_1);
                            // elapsed_dominated_t += (float)elapsed_dominated_1.count();
                        }
                        if (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].min_g_itr)
                        {
                            // if (s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr)
                            // {
                            //     //remove the dominated state from dom list
                            //     map_st_temp[s_pred->x][s_pred->y].dom.erase(map_st_temp[s_pred->x][s_pred->y].min_g_itr);
                            // }
                            //update the min gQ and corr g_itr
                            map_st_temp[s_pred->x][s_pred->y].min_g_itr = s_pred->g_itr;
                            map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr = s_pred->g_Q;
                            //cout << "g itr less than min g itr" << endl;
                            //  cout << "min gitr of keypred: " << key_pred << "," << map_st_temp[s_pred->x][s_pred->y].min_g_itr  <<endl;
                            //          auto end_dominated_1 = std::chrono::system_clock::now();
                            //  auto elapsed_dominated_1 =
                            //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated_1 - start_dominated_1);
                            // elapsed_dominated_t += (float)elapsed_dominated_1.count();
                        }
                    }
                    else
                    {
                        // if (map_st_temp[s_pred->x][s_pred->y].dom.size() > 20)
                        //cout << "iterate over full dom map of size" << map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;
                        //it = map_st_temp[s_pred->x][s_pred->y].dom.begin();
                        //auto start_dominated = std::chrono::system_clock::now();
                        if (s_pred->g_itr >= map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q || s_pred->g_Q >= map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr)
                        {
                            //cout << "came here 1" << endl;
                            continue;
                        }
                        else // no option but iterate over all states in dom list
                        {
                            for (dom *s1 = map_st_temp[s_pred->x][s_pred->y].end_p; s1; s1 = s1->bp)
                            {
                                //solution_path->push_back(state);

                                //cout << s1->x << "," << s1->y << "f = " << s1->f << "g_itr, g_Q, g = " << s1->g_itr << "," << s1->g_Q << "," << s1->g << " h = " << s1->h << endl;

                                if (s1->g_iter <= s_pred->g_itr && s1->g_Q <= s_pred->g_Q) // dominated by one
                                {

                                    dom_flag = true;
                                    break;
                                }

                                if (s1->bp == NULL)
                                {

                                    //int motdir =
                                    //cout << motdir << endl;

                                    break;
                                }
                            }
                        }
                    }
                }

                //auto start_dominated = std::chrono::system_clock::now();

                // auto end_dominated = std::chrono::system_clock::now();
                // auto elapsed_dominated =
                //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated - start_dominated);
                // elapsed_dominated_t += (float)elapsed_dominated.count();

                if (dom_flag)
                {
                    if (constants.debug_computePath)
                        cout << "s pred dominated, don't push and cont" << endl;
                    continue;
                }
                //
                // //auto start_dominated = std::chrono::system_clock::now();
                // it = map_st_temp[s_pred->x][s_pred->y].dom.begin();
                // while (it != map_st_temp[s_pred->x][s_pred->y].dom.end())
                // {
                //     cout << "dom map elem: " << it->first << "," << it->second << endl;
                //     if (it->first <= s_pred->g_itr && it->second <= s_pred->g_Q) // dominated by one
                //     {

                //         dom_flag = true;
                //         break;
                //     }
                //     it++;
                // }
                // if (dom_flag)
                // {
                //     cout << "pushing dominated state in map" << endl;
                //     getchar();
                // }

                // // If you are pushing, then undominated, so insert into your dom list
                dom *undom_g = new dom;
                undom_g->g_iter = s_pred->g_itr;
                undom_g->g_Q = s_pred->g_Q;
                undom_g->bp = map_st_temp[s_pred->x][s_pred->y].end_p;
                // if (undom_g->bp == NULL)
                // {
                //     cout << "map_st_temp[s_pred->x][s_pred->y].end_p is NULL first time" << endl;
                // }
                map_st_temp[s_pred->x][s_pred->y].end_p = undom_g;
                // cout << "map_st_temp[s_pred->x][s_pred->y].end_p: " << (map_st_temp[s_pred->x][s_pred->y].end_p)->g_iter << "," << (map_st_temp[s_pred->x][s_pred->y].end_p)->g_Q << endl;
                // if (undom_g->bp != NULL)
                // {
                //     cout << "bp, and end pointer" << undom_g->bp->g_iter << "," << undom_g->bp->g_Q << "," << map_st_temp[s_pred->x][s_pred->y].end_p << endl;
                // }
                // getchar();
                //map_st_temp[s_pred->x][s_pred->y].dom.push_back(std::make_pair(s_pred->g_itr,s_pred->g_Q));
                // cout << "pushng in dom map and heap" << endl;
                // cout << "dom map size" << map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;

                // if (constants.debug_computePath)
                // {

                //     cout << "dom map size: " << map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;
                // }

                heap.push(s_pred);
                // cout << "closed list size when exp = u" << closed.size() << endl;
                // getchar();
                // if (X_piv->sx.x == 13 && X_piv->sx.y == 5)
                //     getchar();
            }
            // auto end_predecessor_comp = std::chrono::system_clock::now();
            // auto elapsed_predecessor_comp =
            //     std::chrono::duration_cast<std::chrono::microseconds>(end_predecessor_comp - start_predecessor_comp);
            // elapsed_predecessor_comp_t += (float)elapsed_predecessor_comp.count();
        }
        else // exp s not unknown, mprims apply for "free", if "blocked" then mprims don't apply
        {
            if (constants.debug_computePath)
            {
                cout << "exp not unknown, check if preds hidden and f" << endl;
                cout << "dx_.size" << dx_.size() << endl;
            }

            //auto start_predecessor_comp_1 = std::chrono::system_clock::now();
            for (int a = 0; a < dx_.size() - 1; a++)
            {

                xy_planner_state *s_pred = new xy_planner_state;

                float c = constants.getPredMotPrims(s, a, dx_, dy_, s_pred);

                if (constants.debug_computePath)
                {
                    cout << "s pred h = " << s_pred->x << "," << s_pred->y << endl;
                }

                // if (!isValidState(s_pred, param1, closed, grid2D))
                // {
                //     continue;
                // }

                if (s_pred->x < 0 || s_pred->x >= constants.numcells_x || s_pred->y < 0 || s_pred->y >= param1.numcells_y)
                {
                    if (constants.debug_computePath)
                    {
                        cout << " s_pred out of bounds, move to next action " << endl;
                    }

                    continue;
                }

                ////////////// static collision check
                else if (grid2D[s_pred->x][s_pred->y] == constants.obs_cell)
                {
                    // if (s->x == 25 && s->y == 28)
                    //     cout << "wall identify" << endl;
                    if (constants.debug_computePath)
                    {
                        cout << " s_pred in collision, move to next action " << endl;
                    }

                    continue;
                }
                // gQ key for closed list
                //uint64_t key_pred_1_gQ = constants.getHashKeyGq(s_pred->x, s_pred->y, s_pred->g_Q); //if in closed list then move on
                unsigned long long int key_pred_1 = constants.getHashKey(constants.numcells_x, s_pred->x, s_pred->y);
                // std::unordered_map<unsigned long long int, int>::const_iterator cls_it_pred_1 = closed.find(key_pred_1);
                // if (cls_it_pred_1 != closed.end())
                // {
                //     if (constants.debug_computePath)
                //     {
                //         cout << " s_pred in closed, move to next action " << endl;
                //     }

                //     continue;
                // }

                // if s_pred is known to be failure, then treat as obstacle
                // gQ key for closed list

                std::unordered_map<unsigned long long int, int>::const_iterator gotp = map_un.find(key_pred_1);

                if (gotp != map_un.end()) //pred is hidden
                {
                    int u_ctr2 = gotp->second;
                    if (X_piv->hx[u_ctr2] == 'u' || X_piv->hx[u_ctr2] == 's')
                    {
                        if (constants.debug_computePath)
                        {
                            cout << "piv has u_ctr = u, so insert into OPEN" << endl;
                        }

                        // put in open list
                        q = constants.w_mp * c + s->g_Q;
                        // Push predecessor
                        s_pred->g_Q = q;

                        s_pred->g_itr = exp->g_itr + constants.sc * constants.cmax;

                        s_pred->besta = a;
                        s_pred->h = constants.sc * constants.getheur2D(s_pred->x, s_pred->y, X_piv->sx.x, X_piv->sx.y);

                        s_pred->f = s_pred->g_itr + s_pred->h;

                        s_pred->fp = s;
                        if (constants.debug_computePath)
                        {
                            cout << "s pred, g_itr, g_Q, g value, h value, f value: " << s_pred->x << "," << s_pred->y << "," << s_pred->g_itr << "," << s_pred->g_Q << "," << s_pred->g << "," << s_pred->h << "," << s_pred->f << endl;
                            cout << "s pred memory being pushed in heap: " << s_pred << endl;
                        }

                        /////////// check if dominated

                        //std::unordered_map<unsigned long long int, xy_planner_state_dom *>::const_iterator dom_it_1 = map_st_temp.find(key_pred_1);
                        bool dom_flag_mp = false;
                        // cout << "min gQ,  curr gq and min gitr, curr gitr of keypred: " << key_pred_1 << "," << map_st_temp[s_pred->x][s_pred->y].min_g_Q << "," << s_pred->g_Q << "," << map_st_temp[s_pred->x][s_pred->y].min_g_itr
                        //      << "," << s_pred->g_itr << endl;
                        if (map_st_temp[s_pred->x][s_pred->y].end_p == NULL) // // first entry
                        {

                            map_st_temp[s_pred->x][s_pred->y].min_g_itr = s_pred->g_itr;
                            map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr = s_pred->g_Q;
                            map_st_temp[s_pred->x][s_pred->y].min_g_Q = s_pred->g_Q;
                            map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q = s_pred->g_itr;
                        }
                        else
                        {
                            if ((s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].min_g_Q) || (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].min_g_itr))
                            {

                                //update the min gQ and corr g_itr
                                if (s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].min_g_Q)
                                {
                                    // if (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q)
                                    // {
                                    //     //remove the dominated state from dom list
                                    //     map_st_temp[s_pred->x][s_pred->y].dom.erase(map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q);
                                    // }
                                    map_st_temp[s_pred->x][s_pred->y].min_g_Q = s_pred->g_Q;
                                    map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q = s_pred->g_itr;
                                    //cout << "g Q less than min g itr" << endl;
                                    //         // if(map_st_temp[s_pred->x][s_pred->y].dom.size()>2)
                                    //         // cout << "min gQ of keypred: " << key_pred_1 << "," << map_st_temp[s_pred->x][s_pred->y].min_g_Q  <<endl;
                                    //         auto end_dominated_1 = std::chrono::system_clock::now();
                                    //  auto elapsed_dominated_1 =
                                    //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated_1 - start_dominated_1);
                                    // elapsed_dominated_t += (float)elapsed_dominated_1.count();
                                }
                                if (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].min_g_itr)
                                {
                                    // if (s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr)
                                    // {
                                    //     //remove the dominated state from dom list
                                    //     map_st_temp[s_pred->x][s_pred->y].dom.erase(map_st_temp[s_pred->x][s_pred->y].min_g_itr);
                                    // }
                                    //update the min gQ and corr g_itr
                                    map_st_temp[s_pred->x][s_pred->y].min_g_itr = s_pred->g_itr;
                                    map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr = s_pred->g_Q;
                                    //cout << "g itr less than min g itr" << endl;
                                    //  cout << "min gitr of keypred: " << key_pred_1 << "," << map_st_temp[s_pred->x][s_pred->y].min_g_itr  <<endl;
                                    //          auto end_dominated_1 = std::chrono::system_clock::now();
                                    //  auto elapsed_dominated_1 =
                                    //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated_1 - start_dominated_1);
                                    // elapsed_dominated_t += (float)elapsed_dominated_1.count();
                                }
                            }
                            else // no option but iterate over all states in dom list
                            {
                                //it = map_st_temp[s_pred->x][s_pred->y].dom.begin();
                                // auto start_dominated = std::chrono::system_clock::now();
                                //if (map_st_temp[s_pred->x][s_pred->y].dom.size() > 20)
                                //cout << "iterate over full dom map of size" << map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;
                                if (s_pred->g_itr >= map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q || s_pred->g_Q >= map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr)
                                {
                                    //cout << "came here 2" << endl;
                                    continue;
                                }
                                else // no option but iterate over all states in dom list
                                {
                                    for (dom *s1 = map_st_temp[s_pred->x][s_pred->y].end_p; s1; s1 = s1->bp)
                                    {
                                        //solution_path->push_back(state);

                                        //cout << s1->x << "," << s1->y << "f = " << s1->f << "g_itr, g_Q, g = " << s1->g_itr << "," << s1->g_Q << "," << s1->g << " h = " << s1->h << endl;

                                        if (s1->g_iter <= s_pred->g_itr && s1->g_Q <= s_pred->g_Q) // dominated by one
                                        {

                                            dom_flag_mp = true;
                                            break;
                                        }

                                        if (s1->bp == NULL)
                                        {

                                            //int motdir =
                                            //cout << motdir << endl;

                                            break;
                                        }
                                    }
                                }
                            }
                        }

                        //auto start_dominated_2 = std::chrono::system_clock::now();

                        // auto end_dominated_2 = std::chrono::system_clock::now();
                        // auto elapsed_dominated_2 =
                        //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated_2 - start_dominated_2);
                        // elapsed_dominated_t += (float)elapsed_dominated_2.count();

                        if (dom_flag_mp)
                        {
                            if (constants.debug_computePath)
                                cout << "s pred dominated, don't push and cont" << endl;
                            //cout << "dominated state, continue" << endl;
                            continue;
                        }

                        // If you are pushing, then undominated, so insert into your dom list
                        // it = map_st_temp[s_pred->x][s_pred->y].dom.begin();
                        // while (it != map_st_temp[s_pred->x][s_pred->y].dom.end())
                        // {
                        //     cout << "dom map elem: " << it->first << "," << it->second << endl;
                        //     if (it->first <= s_pred->g_itr && it->second <= s_pred->g_Q) // dominated by one
                        //     {

                        //         dom_flag_mp = true;
                        //         break;
                        //     }
                        //     it++;
                        // }
                        // if (dom_flag_mp)
                        // {
                        //     cout << "pushing dominated states in map" << endl;
                        //     getchar();
                        // }
                        dom *undom_g = new dom;
                        undom_g->g_iter = s_pred->g_itr;
                        undom_g->g_Q = s_pred->g_Q;
                        undom_g->bp = map_st_temp[s_pred->x][s_pred->y].end_p;
                        // if (undom_g->bp == NULL)
                        // {
                        //     cout << "map_st_temp[s_pred->x][s_pred->y].end_p is NULL first time" << endl;
                        // }
                        map_st_temp[s_pred->x][s_pred->y].end_p = undom_g;
                        // cout << "map_st_temp[s_pred->x][s_pred->y].end_p: " << (map_st_temp[s_pred->x][s_pred->y].end_p)->g_iter << "," << (map_st_temp[s_pred->x][s_pred->y].end_p)->g_Q << endl;
                        // if (undom_g->bp != NULL)
                        // {
                        //     cout << "bp, and end pointer" << undom_g->bp->g_iter << "," << undom_g->bp->g_Q << "," << map_st_temp[s_pred->x][s_pred->y].end_p << endl;
                        // }
                        // getchar();
                        //map_st_temp[s_pred->x][s_pred->y].dom.push_back(std::make_pair(s_pred->g_itr, s_pred->g_Q));
                        // cout << "pushng in dom map and heap" << endl;
                        // cout << "dom map size" << map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;

                        // if (constants.debug_computePath)
                        // {
                        //     if (map_st_temp[s_pred->x][s_pred->y].dom.size() > 20)
                        //         cout << "dom map size: " << map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;
                        // }

                        heap.push(s_pred);
                        // cout << "closed list size when s pred is hidden " << closed.size() << endl;
                        // getchar();
                        // if (u_ctr > 0)
                        // {
                        //     break;
                        // }
                        // if (X_piv->sx.x == 13 && X_piv->sx.y == 5)
                        //     getchar();
                    }
                    else // move to next mot prim
                    {
                        continue;
                    }
                }
                else // s pred not hidden
                {
                    // if (constants.debug_computePath)
                    // {
                    //     cout << "first loop of u_unknown, so insert into OPEN" << endl;
                    // }

                    // put in open list
                    q = constants.w_mp * c + s->g_Q;
                    // Push predecessor
                    s_pred->g_Q = q; // g_Q

                    s_pred->g_itr = exp->g_itr + constants.sc * constants.cmax; // g_itr

                    s_pred->besta = a;
                    s_pred->h = constants.sc * constants.getheur2D(s_pred->x, s_pred->y, X_piv->sx.x, X_piv->sx.y);

                    s_pred->f = s_pred->g_itr + s_pred->h;

                    s_pred->fp = s;
                    if (constants.debug_computePath)
                    {
                        cout << "s pred not hidden g_itr, g_Q, g value, h value, f value: " << s_pred->x << "," << s_pred->y << "," << s_pred->g_itr << "," << s_pred->g_Q << "," << s_pred->g << "," << s_pred->h << "," << s_pred->f << endl;
                        cout << "s pred memory being pushed in heap: " << s_pred << endl;
                    }
                    // cout << "closed list size when s pred is mot prim " << closed.size() << endl;
                    // getchar();

                    ///////////// check if dominated
                    //

                    //std::unordered_map<unsigned long long int, xy_planner_state_dom *>::const_iterator dom_it_1 = map_st_temp.find(key_pred_1);
                    bool dom_flag_mp_1 = false;
                    // cout << "min gQ,  curr gq and min gitr, curr gitr of keypred: " << key_pred_1 << "," <<map_st_temp[s_pred->x][s_pred->y].min_g_Q << "," << s_pred->g_Q << "," <<map_st_temp[s_pred->x][s_pred->y].min_g_itr
                    //  << "," << s_pred->g_itr << endl;
                    if (map_st_temp[s_pred->x][s_pred->y].end_p == NULL) // // pred exists in this temp map, use it
                    {

                        map_st_temp[s_pred->x][s_pred->y].min_g_itr = s_pred->g_itr;
                        map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr = s_pred->g_Q;
                        map_st_temp[s_pred->x][s_pred->y].min_g_Q = s_pred->g_Q;
                        map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q = s_pred->g_itr;
                    }
                    else
                    {
                        if ((s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].min_g_Q) || (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].min_g_itr))
                        {

                            //update the min gQ and corr g_itr
                            if (s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].min_g_Q)
                            {
                                // if (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q)
                                // {
                                //     //remove the dominated state from dom list
                                //     map_st_temp[s_pred->x][s_pred->y].dom.erase(map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q);
                                // }
                                map_st_temp[s_pred->x][s_pred->y].min_g_Q = s_pred->g_Q;
                                map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q = s_pred->g_itr;
                                //cout << "g Q less than min g itr" << endl;
                                //         // if(map_st_temp[key_pred_1]->dom.size()>2)
                                //         // cout << "min gQ of keypred: " << key_pred_1 << "," <<map_st_temp[s_pred->x][s_pred->y].min_g_Q  <<endl;
                                //         auto end_dominated_1 = std::chrono::system_clock::now();
                                //  auto elapsed_dominated_1 =
                                //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated_1 - start_dominated_1);
                                // elapsed_dominated_t += (float)elapsed_dominated_1.count();
                            }
                            if (s_pred->g_itr < map_st_temp[s_pred->x][s_pred->y].min_g_itr)
                            {
                                // if (s_pred->g_Q < map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr)
                                // {
                                //     //remove the dominated state from dom list
                                //     map_st_temp[s_pred->x][s_pred->y].dom.erase(map_st_temp[s_pred->x][s_pred->y].min_g_itr);
                                // }
                                //update the min gQ and corr g_itr
                                map_st_temp[s_pred->x][s_pred->y].min_g_itr = s_pred->g_itr;
                                map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr = s_pred->g_Q;
                                //cout << "g itr less than min g itr" << endl;
                                //  cout << "min gitr of keypred: " << key_pred_1 << "," <<map_st_temp[s_pred->x][s_pred->y].min_g_itr  <<endl;
                                //          auto end_dominated_1 = std::chrono::system_clock::now();
                                //  auto elapsed_dominated_1 =
                                //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated_1 - start_dominated_1);
                                // elapsed_dominated_t += (float)elapsed_dominated_1.count();
                            }
                        }
                        else // no option but iterate over all states in dom list
                        {
                            //it = map_st_temp[s_pred->x][s_pred->y].dom.begin();
                            //if(map_st_temp[key_pred_1]->dom.size() > 20)
                            //cout << "iterate over full dom map of size" <<map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;
                            if (s_pred->g_itr >= map_st_temp[s_pred->x][s_pred->y].gitr_of_min_g_Q || s_pred->g_Q >= map_st_temp[s_pred->x][s_pred->y].gQ_of_min_g_itr)
                            {
                                //cout << "came here 3" << endl;
                                continue;
                            }
                            else // no option but iterate over all states in dom list
                            {
                                for (dom *s1 = map_st_temp[s_pred->x][s_pred->y].end_p; s1; s1 = s1->bp)
                                {
                                    //solution_path->push_back(state);

                                    //cout << s1->x << "," << s1->y << "f = " << s1->f << "g_itr, g_Q, g = " << s1->g_itr << "," << s1->g_Q << "," << s1->g << " h = " << s1->h << endl;

                                    if (s1->g_iter <= s_pred->g_itr && s1->g_Q <= s_pred->g_Q) // dominated by one
                                    {

                                        dom_flag_mp_1 = true;
                                        break;
                                    }

                                    if (s1->bp == NULL)
                                    {

                                        //int motdir =
                                        //cout << motdir << endl;

                                        break;
                                    }
                                }
                            }
                        }
                    }

                    //auto start_dominated_1 = std::chrono::system_clock::now();
                    // bool check_min_gQ = ;
                    // bool check_min_gitr = ;

                    //  auto end_dominated_1 = std::chrono::system_clock::now();
                    //  auto elapsed_dominated_1 =
                    //     std::chrono::duration_cast<std::chrono::nanoseconds>(end_dominated_1 - start_dominated_1);
                    // elapsed_dominated_t += (float)elapsed_dominated_1.count();

                    if (dom_flag_mp_1)
                    {
                        // if (constants.debug_computePath)
                        //     cout << "s pred dominated, don't push and cont" << endl;
                        //cout << "dominated state, continue" << endl;
                        continue;
                    }

                    // it =map_st_temp[s_pred->x][s_pred->y].dom.begin();
                    // //if(map_st_temp[key_pred_1]->dom.size() > 20)

                    // while (it !=map_st_temp[s_pred->x][s_pred->y].dom.end())
                    // {
                    //     cout << "dom map elem: " << it->first << "," << it->second << endl;
                    //     if (it->first <= s_pred->g_itr && it->second <= s_pred->g_Q) // dominated by one
                    //     {

                    //         dom_flag_mp_1 = true;
                    //         break;
                    //     }
                    //     it++;
                    // }
                    // if (dom_flag_mp_1)
                    // {
                    //     cout << "dominated state being pushed, error" << endl;
                    //     getchar();
                    // }

                    //map_st_temp[s_pred->x][s_pred->y].dom.push_back(std::make_pair(s_pred->g_itr,s_pred->g_Q));
                    dom *undom_g = new dom;
                    undom_g->g_iter = s_pred->g_itr;
                    undom_g->g_Q = s_pred->g_Q;
                    undom_g->bp = map_st_temp[s_pred->x][s_pred->y].end_p;
                    // if (undom_g->bp == NULL)
                    // {
                    //     cout << "map_st_temp[s_pred->x][s_pred->y].end_p is NULL first time" << endl;
                    // }
                    map_st_temp[s_pred->x][s_pred->y].end_p = undom_g;
                    // cout << "map_st_temp[s_pred->x][s_pred->y].end_p: " << (map_st_temp[s_pred->x][s_pred->y].end_p)->g_iter << "," << (map_st_temp[s_pred->x][s_pred->y].end_p)->g_Q << endl;
                    // if (undom_g->bp != NULL)
                    // {
                    //     cout << "bp, and end pointer" << undom_g->bp->g_iter << "," << undom_g->bp->g_Q << "," << map_st_temp[s_pred->x][s_pred->y].end_p << endl;
                    // }
                    // getchar();
                    // cout << "pushng in dom map and heap" << endl;
                    // cout << "dom map size" <<map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;
                    // if (constants.debug_computePath)
                    // {
                    //     if (map_st_temp[s_pred->x][s_pred->y].dom.size() > 20)
                    //         cout << "dom map size: " << map_st_temp[s_pred->x][s_pred->y].dom.size() << endl;
                    // }

                    heap.push(s_pred);
                }
            }
            // auto end_predecessor_comp_1 = std::chrono::system_clock::now();
            // auto elapsed_predecessor_comp_1 =
            //     std::chrono::duration_cast<std::chrono::microseconds>(end_predecessor_comp_1 - start_predecessor_comp_1);
            // elapsed_predecessor_comp_t += (float)elapsed_predecessor_comp_1.count();
        }

        // auto end_comp_subopt_while = std::chrono::system_clock::now();
        // auto elapsed_comp_subopt_while =
        //     std::chrono::duration_cast<std::chrono::microseconds>(end_comp_subopt_while - start_comp_subopt_while);
        // elapsed_comp_subopt_while_t += (int)elapsed_comp_subopt_while.count();
    }

    // auto end_comp_subopt_while = std::chrono::system_clock::now();
    // auto elapsed_comp_subopt_while =
    //     std::chrono::duration_cast<std::chrono::microseconds>(end_comp_subopt_while - start_comp_subopt_while);
    // elapsed_comp_subopt_while_t += (int)elapsed_comp_subopt_while.count();

    //// path computation
    if (constants.debug_computePath)
    {
        cout << "path" << endl;
    }

    int i = 0;

    if (constants.visualize)
    {
        visualization_msgs::MarkerArray rob_cyl;
        for (xy_planner_state *s1 = exp; s1; s1 = s1->fp)
        {
            //solution_path->push_back(state);
            i++;
            //if (debug)
            //if(exp->x == 29 && exp->y == 36)

            //cout << s1->x << "," << s1->y << "f = " << s1->f << "g_itr, g_Q, g = " << s1->g_itr << "," << s1->g_Q << "," << s1->g << " h = " << s1->h << endl;
            if (constants.visualize)
                visualizeMarker((s1->x + 0.5), (s1->y + 0.5), i, 1, rob_cyl);

            // assign in map_st
            // unsigned long long int key_path = constants.getHashKey(constants.numcells_x, s1->x, s1->y);
            //map_st[key_path] = s1; // real key

            if (s1->fp == NULL)
            {

                //int motdir =
                //cout << motdir << endl;

                break;
            }
        }
        //cout << "path size" << i << endl;
        pub_points_path.publish(rob_cyl);
    }

    // if (constants.debug_computePath)
    //     getchar();

    heap = priority_queue<xy_planner_state *, vector<xy_planner_state *>, CompareSt>();
    //cout << "size after............." << heap.size() << endl;
    // empty unordered map
    //closed.clear();
    return 1;
}

void create_unknown_states(parameters param1, std::unordered_map<unsigned long long int, int> &map_un)
{

    std::default_random_engine generator;
    generator.seed(std::time(NULL));
    std::uniform_int_distribution<int> distributionH(5, 10);
    std::uniform_int_distribution<int> distribution1(constants.numcells_x / 2 - 1, constants.numcells_x / 2 + 1);
    std::uniform_int_distribution<int> distribution2(1.0, param1.numcells_y - 1);

    unsigned long long int key1 = constants.getHashKey(constants.numcells_x, 15, 5);
    unsigned long long int key2 = constants.getHashKey(constants.numcells_x, 15, 21);
    unsigned long long int key3 = constants.getHashKey(constants.numcells_x, 15, 23);
    unsigned long long int key4 = constants.getHashKey(constants.numcells_x, 15, 25);

    map_un[key1] = 0;
    map_un[key2] = 1;
    map_un[key3] = 2;
    map_un[key4] = 3;
}

void read_unknown_states(parameters param1, std::unordered_map<unsigned long long int, int> &map_un, unsigned int **grid2D)
{

    // check if Grid2D == 2 (un), then compute key and enter into map
    unsigned long long int key;
    int unknown_ctr = 0;
    for (int i = 0; i < constants.numcells_x; i++)
    {
        for (int j = 0; j < constants.numcells_y; j++)
        {
            if (grid2D[i][j] == unknown_cell) // hidden state
            {
                key = constants.getHashKey(constants.numcells_x, i, j);
                map_un[key] = unknown_ctr;
                unknown_ctr++;
                //cout << "hidden vars:" << i << "," << j << endl;
                //getchar();
            }
        }
    }
}
bel_state *computePivotNoSol(bel_state *X_piv, bel_state *X_st, vector<int> &dx_, vector<int> &dy_,
                             std::unordered_map<string, bel_state *> &map_bel,
                             std::unordered_map<unsigned long long int, xy_planner_state *> &map_st, xy_planner_state Sg)
{
    int sum_f = 0;
    int sum_f_no_besta = 0;

    bel_state X_b, X_f;
    bel_state *X_proceed = X_st;
    bel_state *X_p = X_st;

    while (1)
    {
        unsigned long long int key = constants.getHashKey(constants.numcells_x, X_proceed->sx.x, X_proceed->sx.y);
        string keys = std::to_string(key) + X_proceed->hx;

        //if(constants.debug_computePath)
        //cout << "In compute piv no solution: X_p: " << X_p->sx.x << "," << X_p->sx.y << "," << X_p->hx << endl;

        // if(constants.debug_computePath)
        //cout << "map_bel[keys]->besta: " << map_bel[keys]->besta << endl;
        bel_succ(X_proceed, dx_, dy_, map_bel[keys]->besta, X_b, X_f, map_bel[keys]->un_id);
        //get the success and failure state key
        unsigned long long int kb = constants.getHashKey(constants.numcells_x, X_b.sx.x, X_b.sx.y);
        string ks_b = std::to_string(kb) + X_b.hx;

        if (map_bel[keys]->besta == dx_.size() - 1) // stochastic action
        {
            sum_f++;
            unsigned long long int kf = constants.getHashKey(constants.numcells_x, X_f.sx.x, X_f.sx.y);
            string ks_f = std::to_string(kf) + X_f.hx;
            if ((map_bel.find(ks_f) == map_bel.end()) || (map_bel[ks_f]->besta == -1)) //never created in updateMDP, so Xf is unexplored and has Ve, or it has -1 as besta
            {
                sum_f_no_besta++;
                // if(constants.debug_computePath)
                //cout << "In compute piv no solution: moving to X_b: " << map_bel[ks_b]->sx.x << "," << map_bel[ks_b]->sx.y << "," << map_bel[ks_b]->hx << endl;
                X_proceed = map_bel[ks_b]; //move down towards goal on the primary path from the current X_p
            }
            else
            {
                // if(constants.debug_computePath)
                //cout << "In compute piv no solution: X_f: " << map_bel[ks_f]->sx.x << "," << map_bel[ks_f]->sx.y << "," << map_bel[ks_f]->hx << endl;

                X_p = map_bel[ks_f]; // set X_f to be the new X_p and start searching down the primary path from X_f
                //reset the sum counters and X_proceed
                X_proceed = map_bel[ks_f];
                sum_f = 0;
                sum_f_no_besta = 0;
            }
            //getchar();
        }
        else // deterministic action, move down on the primary path from current X_p
        {
            X_proceed = map_bel[ks_b];
        }

        if ((X_proceed->sx.x == Sg.x && X_proceed->sx.y == Sg.y)) //reached goal
        {
            // if(constants.debug_computePath)
            //cout << "sum_f, sum_f_no_besta: " << sum_f << "," << sum_f_no_besta << endl;
            X_p->was_pivot = true;
            return X_p;
        }
    }
}
void updateMDPRev(bel_state *X_piv, bel_state *X_st,
                  std::unordered_map<string, bel_state *> &map_bel, parameters &param1, vector<int> &dx_, vector<int> &dy_, float &prob_s,
                  float &prob_f, xy_planner_state &S_goal)

{
    cout << "In updateMDPRev........." << endl;
    unsigned long long int key_piv = constants.getHashKey(constants.numcells_x, X_piv->sx.x, X_piv->sx.y);
    string keys_piv = std::to_string(key_piv) + X_piv->hx;

    float g_Q_st = map_bel[keys_piv]->v; //initialize gQ of pivot

    ////cout << "g_Q piv" << g_Q_st << endl;

    if (X_piv->bp != NULL) // X_piv has a parent assigned from computePivot and is not start
    {
        bel_state X_pred = *X_piv->bp;
        bel_state *X_pred_p = &X_pred;

        bel_state X = *X_piv;
        bel_state *X_p = &X;

        bel_state X_b, X_f;
        float v_f_i;

        while (X_pred_p != NULL)
        {

            unsigned long long int key_p = constants.getHashKey(constants.numcells_x, X_pred_p->sx.x, X_pred_p->sx.y);
            string keys_p = std::to_string(key_p) + X_pred_p->hx;

            unsigned long long int key = constants.getHashKey(constants.numcells_x, X_p->sx.x, X_p->sx.y);
            string keys = std::to_string(key) + X_p->hx;

            bel_succ(X_pred_p, dx_, dy_, X_pred_p->besta, X_b, X_f, map_bel[keys_p]->un_id);
            //if (constants.debug_computePath)
            //cout << "xp, x_b, x pred " << X_p->sx.x << "," << X_p->sx.y << X_p->hx << "," << X_b.sx.x << "," << X_b.sx.y << X_b.hx << "," << X_pred_p->sx.x << "," << X_pred_p->sx.y << X_pred_p->hx << endl;

            float c_s = getCost(X_pred_p->sx.x, X_pred_p->sx.y, X_b.sx.x, X_b.sx.y);
            float c_f = constants.sc * constants.Cfail;

            unsigned long long int key_b = constants.getHashKey(constants.numcells_x, X_b.sx.x, X_b.sx.y);
            string keys_b = std::to_string(key_b) + X_b.hx;
            int blah = 1;

       
                if (X_pred_p->besta == (dx_.size() - 1))
                {
                    //cout << "stochastic trans " << endl;
                    unsigned long long int key_f = constants.getHashKey(constants.numcells_x, X_f.sx.x, X_f.sx.y);
                    string keys_f = std::to_string(key_f) + X_f.hx;

                    if (keys == keys_f) // failure is on the branch from X_piv
                    {
                        //cout << "Xf: " << X_f.sx.x << "," << X_f.sx.y << X_f.hx << endl;

                        // update v value of X_f in policy before  g_Q_st gets cumulated for in this loop
                        if (abs(map_bel[keys_piv]->v - constants.inf) < 10)
                        {
                            ////cout << "constants.inf v for piv" << endl;
                            //map_bel[keys_f]->v = constants.inf;
                            map_bel[keys_f]->v = g_Q_st;
                        }
                        else
                        {
                            map_bel[keys_f]->v = g_Q_st;
                            //cout << "updated v value: " << map_bel[keys_f]->v << endl; //getchar();
                        }

                        ////cout << "Vf values of pivots on policy backtrack, key : " << map_bel[keys_f]->v << "," << keys_f << endl;
                        ////cout << "map_bel[keys_b]->v: " << map_bel[keys_b]->v << endl;

                        //update g_Q_st
                        g_Q_st = prob_s * (c_s + map_bel[keys_b]->v) + prob_f * (c_f + g_Q_st);
                    }
                    else
                    {
                        // update v value of X_s in policy before  g_Q_st gets cumulated for in this loop
                        if (abs(map_bel[keys_piv]->v - constants.inf) < 10)
                        {
                            //map_bel[keys_b]->v = constants.inf;
                            map_bel[keys_b]->v = g_Q_st;
                        }
                        else
                        {
                            map_bel[keys_b]->v = g_Q_st;
                           //cout << "updated v value mot prim: " << map_bel[keys_b]->v << endl;
                        }

                        if (map_bel.find(keys_f) == map_bel.end()) // current pivot isn't in map_bel, so only this. Assuming everything above it in the branch exists in map_bel
                        {
                            v_f_i = constants.sc * constants.getheur2D(X_f.sx.x, X_f.sx.y, S_goal.x, S_goal.y);
                        }
                        else
                        {
                            //cout << "Xf which exists in belief state: " << X_f.sx.x << "," << X_f.sx.y << X_f.hx << endl;
                            v_f_i = map_bel[keys_f]->v;
                        }

                        //g_Q_st = prob_s * (c_s + g_Q_st) + prob_f * max((c_s + g_Q_st),c_f + v_f_i);
                        g_Q_st = prob_s * (c_s + g_Q_st) + prob_f * (c_f + v_f_i);
                    }
                }
                else
                {
                    if (abs(map_bel[keys_piv]->v - constants.inf) < 10)
                    {
                        //map_bel[keys_b]->v = constants.inf;
                        map_bel[keys_b]->v = g_Q_st;
                    }
                    else
                    {
                        map_bel[keys_b]->v = g_Q_st;
                        //cout << "updated v value mot prim: " << map_bel[keys_b]->v << endl;
                    }
                    g_Q_st = c_s + g_Q_st;
                }

            // set X_p to X_pred
            X_p->sx = X_pred_p->sx;

            X_p->hx.assign(X_pred_p->hx);

            // set X_pred to ts back pointer and move reverse in polciy
            X_pred_p = X_pred_p->bp;
            //cout << "cumulative g_Q_st n update mdp reverse: " << g_Q_st << endl;
        }
        unsigned long long int key_st = constants.getHashKey(constants.numcells_x, X_st->sx.x, X_st->sx.y);
        string keys_st = std::to_string(key_st) + X_st->hx;
        map_bel[keys_st]->v = g_Q_st;
    }
}

void fast_ppcp(parameters param1)
{

    //// do with 8 motion primitives and 1 overtake action

    ////// hashing check /////

    /* std::unordered_map<string, int> map;
    int key1 = 100*10+20;
    int key2 = 100*20 + 10;
    std::string str = "uuuu";
    map[str] = 10;
    std::string str1 = "s";
    std::string str2 = "u";
    std::string str3 = "f";

    std::string str9;
    str9.assign(str);

    str9.replace(2,1,str1);
    map[std::to_string(key1) + str9] = 20;

    std::string str10;
    str10.assign(str);
    str10.replace(2,1,str3);
    map[std::to_string(key2) + str10] = 30;

    cout << "1,2,3: ..............." << map[std::to_string(key1)+str9] << "," << map[std::to_string(key2)+str10] << endl;*/

    auto nh = ros::NodeHandle(); //nodeHandle
    //auto msg = ros::topic::waitForMessage<nav_msgs::GridCells>("/pedsim/static_obstacles", nh, ros::Duration(10.0));
    std::ofstream *file = new std::ofstream();
    std::stringstream stream, ld;
    stream << std::fixed << std::setprecision(2) << p_door_closed;
    std::string s = stream.str();
    ld << std::fixed << std::setprecision(3) << l;
    std::string s_ld = ld.str();
    std::string filename;
    if (exp_small_envs)
    {
        filename = "/home/ashwin/Research/crowd_nav/results/2d_subopt_ppcp/FINAL_fastppcp_SMALLENV_results/env_random_unknown_l" + s_ld + "p" + s + ".txt";
    }
    else
    {
        filename = "/home/ashwin/Research/crowd_nav/results/2d_subopt_ppcp/FINAL_fastppcp_LARGEENV_results/env_random_unknown_l" + s_ld + "p" + s + ".txt";
    }

    (*file).open(filename, ios::app);

    //ros::Rate rate(g_updateRate);

    //u_states.clear();

    // set fixed p success and p fail for all states
    float prob_s = 0.50;
    float prob_f = 0.50;

    /////////////////////// actions //////////////////////////////

    std::vector<int>
        dx_;
    std::vector<int> dy_;

    dx_.push_back({1}); // up
    dy_.push_back({0});

    dx_.push_back({0}); // down
    dy_.push_back({1});

    dx_.push_back({-1}); // left
    dy_.push_back({0});

    dx_.push_back({0}); // right
    dy_.push_back({-1});

    /////////////////////// Add 8 actions for large environments, and make constants.sc = 10

    dx_.push_back({1});
    dy_.push_back({1});

    dx_.push_back({-1});
    dy_.push_back({1});

    dx_.push_back({1});
    dy_.push_back({-1});

    dx_.push_back({-1});
    dy_.push_back({-1});

    dx_.push_back({0}); /////// /////// sense and move if free
    dy_.push_back({0});

    // Publish walls

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.id = 10000;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 2.0;
    marker.pose.position.z = marker.scale.z / 2.0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;

    // Publish hidden cells

    visualization_msgs::Marker marker_h;
    marker_h.header.frame_id = "odom";
    marker_h.header.stamp = ros::Time();
    marker_h.id = 10001;
    marker_h.color.a = 0.8;
    marker_h.color.r = 0.0;
    marker_h.color.g = 1.0;
    marker_h.color.b = 0.0;
    marker_h.scale.x = 1;
    marker_h.scale.y = 1;
    marker_h.scale.z = 2.0;
    marker_h.pose.position.z = marker_h.scale.z / 2.0;
    marker_h.type = visualization_msgs::Marker::CUBE_LIST;

    // read the grid size from environment file
    // for 4x4 env
    // ifstream f("/home/ashwin/Research/crowd_nav/test_envs_ppcp/room_64_grid_with_hidden_1.txt");
    // for big env

    std::string env_filename;

    if (exp_small_envs)
    {
        if (small_ex_for_paper)
        {
            env_filename = "/home/ashwin/Research/crowd_nav/test_envs_ppcp/nav_hidden_doors_small/room_big_doors_prob_2_hid0.50_expaper.txt";
        }
        else
        {
            env_filename = "/home/ashwin/Research/crowd_nav/test_envs_ppcp/nav_hidden_doors_small/room_big_doors_prob_" + std::to_string(env_num) + "_hid" + s + ".txt";
        }
    }
    else
    {
        env_filename = "/home/ashwin/Research/crowd_nav/test_envs_ppcp/room_big_doors_prob_" + std::to_string(env_num) + "_hid" + s + ".txt";
    }

    cout << env_filename << endl;
    ifstream f(env_filename);

    f >> constants.numcells_x >> constants.numcells_y;

    /////////////////////////////////////////
    // allocate the grid
    unsigned int **grid2D;
    grid2D = new unsigned int *[constants.numcells_y];
    for (int tx = 0; tx < constants.numcells_x; tx++)
    {
        grid2D[tx] = new unsigned int[constants.numcells_x];
    }

    if (debug_large_env)
    {
        for (int i = 0; i < constants.numcells_x; i++)
        {
            for (int j = 0; j < constants.numcells_y; j++)
            {
                grid2D[i][j] = free_cell;
            }
        }
        for (int i = 0; i < grid_cells.size(); i++)
        {
            int gx_s, gy_s;

            worldToGrid(grid_cells[i].x, grid_cells[i].y, gx_s, gy_s, param1.m_origin_x, param1.m_origin_y, param1.m_res);

            if (gx_s >= 0 && gx_s < constants.numcells_x && gy_s >= 0 && gy_s < param1.numcells_y)
            {
                if (debug)
                    cout << "obs: " << gx_s << "," << gy_s << endl;
                grid2D[gx_s][gy_s] = constants.obs_cell;
            }
        }
    }
    else
    {
        for (int i = 0; i < constants.numcells_x; i++)
        {
            for (int j = 0; j < constants.numcells_y; j++)
            {

                f >> grid2D[i][j];
                // if ((i == 33 && j == 41) || (i == 33 && j == 40))
                // {
                //     grid2D[i][j] = free_cell;
                // }
                ////////////// Specifically added for debugging multiple "h" cells
                if (debug_single_hidden_var)
                {
                    if ((i == 33 && j == 22) || (i == 33 && j == 21) || (i == 33 && j == 20) || (i == 49 && j == 15) || (i == 49 && j == 30)
                        //|| (i == 49 && j == 10) || (i == 40 && j == 33) || (i == 9 && j == 49) || (i == 33 && j == 60) || (i == 49 && j == 56)
                    )
                    {
                        grid2D[i][j] = constants.obs_cell;
                    }
                }

                if (grid2D[i][j] == constants.obs_cell) // obstacle
                {
                    geometry_msgs::Point p;
                    p.x = i + 0.5;
                    p.y = j + 0.5;
                    p.z = 0;
                    marker.points.push_back(p);
                }
                if (grid2D[i][j] == unknown_cell) // hidden cell
                {
                    geometry_msgs::Point p;
                    p.x = i + 0.5;
                    p.y = j + 0.5;
                    p.z = 0;
                    marker_h.points.push_back(p);
                }
            }
        }
    }

    // make the environment and publsih obstacles

    pub_obs_movAI.publish(marker);
    pub_hidden_movAI.publish(marker_h);
    cout << "In movAI: " << endl;
    //getchar();
    //cout << "Testing, some elems from Grid: " << grid2D[18][34] << "," << grid2D[12][12] << endl;

    xy_planner_state_dom **map_st_temp;
    map_st_temp = new xy_planner_state_dom *[constants.numcells_y];
    for (int tx = 0; tx < constants.numcells_x; tx++)
    {
        map_st_temp[tx] = new xy_planner_state_dom[constants.numcells_x];
    }

    //////////////////////////////////////////////////////////// unordered_map for bel state /////////////////////////////////////////////////////////////////////
    std::unordered_map<string, bel_state *> map_bel;
    std::unordered_map<unsigned long long int, xy_planner_state *> map_st; /////////////// for closed list in S(X) search

    // map for unknown states
    std::unordered_map<unsigned long long int, int> map_un;
    if (debug_large_env)
    {
        create_unknown_states(param1, map_un);
    }
    else
    {
        read_unknown_states(param1, map_un, grid2D);
    }

    cout << "num hidden vars: " << map_un.size() << endl;
    getchar();
    //double x = g_currentPose.getOrigin().x();
    //double y = g_currentPose.getOrigin().y();
    //double theta = tf::getYaw(g_currentPose.getRotation());

    //double x0 = x, y0 = y;

    bel_state *X_piv = new bel_state;

    //just for the start
    bel_state_ppcp *X_piv_ppcp = new bel_state_ppcp;

    // if (debug_large_env)
    // {
    //     X_piv->sx.x = x0;
    //     X_piv->sx.y = y0;
    // }
    // else
    // {
    X_piv->sx.x = start_x;
    X_piv->sx.y = start_y;
    //}

    //(X_piv->hx).assign(s1);
    X_piv->sx.besta = -1;
    X_piv->besta = -1;

    ///////////// make the ppcp start state
    X_piv_ppcp->sx.x = start_x;
    X_piv_ppcp->sx.y = start_y;
    //}

    //(X_piv_ppcp->hx).assign(s1);
    X_piv_ppcp->sx.besta = -1;
    X_piv_ppcp->besta = -1;

    ///// get successors
    bel_state X_b, X_f;
    bel_state *X_p;
    bel_state *X_pnew;
    bel_state_ppcp *X_pnew_ppcp;
    bel_state_ppcp *X_p_ppcp;

    //create the PPCP object and ppcp deterministc map
    Ppcp ppcp_obj(constants.numcells_x, constants.numcells_y);
    //getchar();

    // ppcp_obj.map_st_ppcp[20][30].g_Q = 4.5;

    //ppcp_obj.hello();
    // cout << ppcp_obj.map_bel_ppcp["a"]->hx << endl;
    // getchar();

    while (true)
    {
        ////////////////////// initialize ////////////////////////////

        // for (int l = 0; l < 20; l++)
        // {
        //cout << "dt" << dt << endl;
        visualizeText(start_x + 0.5, start_y + 0.5, 1, "START", pub_points_st);

        // x = g_currentPose.getOrigin().x();
        // y = g_currentPose.getOrigin().y();
        // theta = tf::getYaw(g_currentPose.getRotation());

        xy_planner_state Sg;
        // if (debug_large_env)
        // {
        //     Sg.x = param1.goal_x;
        //     Sg.y = param1.goal_y;
        // }
        // else
        // {
        Sg.x = goal_x;
        Sg.y = goal_y;
        //}

        visualizeText(Sg.x + 0.5, Sg.y + 0.5, 1, "GOAL", pub_points_gl);

        //visualization_msgs::MarkerArray rob_cyl;

        // for (int k = 0; k < map_un.size(); k++)
        // {
        //     k++;
        //     visualizeMarkerHidden(u.x, u.y, k, 0, rob_cyl);
        // }

        // pub_points_hidden.publish(rob_cyl);
        // getchar();

        //Sg.t = 80;

        //Hmax = Hmax - 1;

        //cout << "Hmax: " << Hmax << endl;

        std::string s1;

        for (int i = 0; i < map_un.size(); i++)
        {

            s1.append("u");
        }

        (X_piv->hx).assign(s1);
        (X_piv_ppcp->hx).assign(s1);

        X_p = X_piv;

        X_p_ppcp = X_piv_ppcp;

        int i = 0;

        //cout << "another run" << endl;

        //cout << "grid cell size " << grid_cells.size() << endl;

        std::clock_t c_start = std::clock();

        // count num iteratons of PPCP and total expansions
        int ppcp_iters = 0;
        int tot_expansions = 0;

        bool increase_bound = true;
        unsigned long long int k_p = constants.getHashKey(constants.numcells_x, X_piv->sx.x, X_piv->sx.y);
        string ks_p = std::to_string(k_p) + X_piv->hx;

        float v_hat_Xst;
        int opt_gQ;
        int iter_tmp = 0;

        while (increase_bound)
        {
            increase_bound = false;

            opt_gQ = ppcp_obj.computeOptQ(X_p_ppcp, dx_, dy_, Sg, prob_s, prob_f, grid2D, map_un, l, tot_expansions, constants);
            // cout << "opt_gQ: " << opt_gQ << endl;
            // cout << "restarted ppcp" << endl;
            ppcp_iters++;
            iter_tmp++;
            //getchar();

            // if (constants.debug_computePath)

                v_hat_Xst = -1000000;

            start_t1 = std::chrono::system_clock::now();

            while (X_p != NULL)
            {

                //cout << i << endl;
                i++;
                map_st.clear();

                if (constants.visualize)
                {
                    cout << "x pivot" << X_p->sx.x << "," << X_p->sx.y << "," << X_p->hx << endl;

                    //getchar();
                }

                //auto start_t1 = std::chrono::system_clock::now();
                //cout << "before" << map_st.size() << endl;
                // for (int k = 0; k < 10; k++)
                // {

                //}

                //get gQ opt

                if (debug)
                    cout << "opt gQ " << opt_gQ << endl;

                float min_gQ_p = constants.inf;

                //auto start_compsuboptpath = std::chrono::system_clock::now();
                int emer = computeSuboptPath(X_piv, X_p, opt_gQ, min_gQ_p, v_hat_Xst, dx_, dy_, map_bel, map_st, map_st_temp, Sg, param1, prob_s, prob_f, grid2D, map_un, l, tot_expansions);
                // auto end_compsuboptpath = std::chrono::system_clock::now();
                // auto elapsed_compsuboptpath =
                //     std::chrono::duration_cast<std::chrono::milliseconds>(end_compsuboptpath - start_compsuboptpath);
                // elapsed_compsuboptpath_t += (int)elapsed_compsuboptpath.count();

                //cout << "g_iters: " << X_p->sx.g_itr << endl;

                //cout << "............computed path.................." << endl;

                // cout << "total expansions: " << tot_expansions << endl;
                //getchar();
                //if (debug)

                //
                //update V value n belief table to min gQ
                unsigned long long int kpiv = constants.getHashKey(constants.numcells_x, X_p->sx.x, X_p->sx.y);
                string ks_piv = std::to_string(kpiv) + X_p->hx;

                string hu_x;
                hu_x.assign(X_p->hx);

                for (int i = 0; i < hu_x.length(); i++)
                {
                    if (hu_x.find("s") != std::string::npos)
                    {
                        hu_x.replace(hu_x.find("s"), 1, "u");
                    }
                }

                string keyp_Xu = std::to_string(kpiv) + hu_x;
                if (map_bel.find(keyp_Xu) == map_bel.end())
                {

                    //cout << "new xu created" << endl;

                    bel_state *X_u_new = new bel_state;
                    X_u_new->sx = X_p->sx;

                    X_u_new->hx = hu_x;
                    map_bel[keyp_Xu] = X_u_new;
                }

                if (emer == 100) //// have to just exit coz times up
                {
                    cout << "timeout, path not found, terminated" << endl;
                    if (debug)
                    {

                        *file << std::to_string(start_x) << " " << std::to_string(start_y) << " " << std::to_string(goal_x) << " " << std::to_string(goal_y) << " " << std::to_string(l) << " " << std::to_string(-100) << " " << std::to_string(-100) << " " << std::to_string(constants.t1) << endl;
                        (*file).close();
                        exit(0);
                    }

                    break;
                }
                if (emer == 200) // no solution found
                {

                    map_bel[ks_piv] = X_p;
                    map_bel[ks_piv]->v = min_gQ_p;
                    //reset besta to -1 when no solution found to indicate path doesn't exist
                    map_bel[ks_piv]->besta = -1;

                    // //update the V(X^u)
                    // string hu_x;
                    // hu_x.assign(X_p->hx);

                    // for (int i = 0; i < hu_x.length(); i++)
                    // {
                    //     if (hu_x.find("s") != std::string::npos)
                    //     {
                    //         hu_x.replace(hu_x.find("s"), 1, "u");
                    //     }
                    // }

                    // string keyp_Xu = std::to_string(kpiv) + hu_x;
                    // if (map_bel.find(keyp_Xu) == map_bel.end())
                    // {

                    //     //cout << "new xu created" << endl;

                    //     bel_state *X_u_new = new bel_state;
                    //     X_u_new->sx = X_p->sx;

                    //     X_u_new->hx = hu_x;
                    //     map_bel[keyp_Xu] = X_u_new;
                    // }

                    map_bel[keyp_Xu]->v = min_gQ_p; ///// V(X^u) also updated, update only using g_Q values
                    if (debug)
                        cout << "v value of X_p, X^u_p: " << map_bel[ks_piv]->v << "," << map_bel[keyp_Xu]->v << endl;

                    //if X p is X st, increase bound by 1 step and restart
                    if (ks_piv == ks_p && iter_tmp > 0)
                    {
                        // if (abs(map_bel[ks_p]->v - constants.inf) < 10) // path does not exist in graph
                        // {
                        //     map_bel.clear();
                        //     break;
                        // }
                        // reset and restart
                        map_bel.clear();
                        iter_tmp = 0;
                        //call the PPCP::updateMDP and PPCP::computePivot
                        // int a = 30;

                        while (1)
                        {
                            ppcp_iters++;
                            bool fl_u = ppcp_obj.updateMDP(X_p_ppcp, dx_, dy_, Sg, constants);
                            //cout << "map_st_ppcp[20][30].g_Q" << ppcp_obj.map_st_ppcp[20][30].g_Q << endl;
                            X_pnew_ppcp = ppcp_obj.computePivot(X_piv_ppcp, dx_, dy_, Sg, prob_s, constants);

                            X_p_ppcp = X_pnew_ppcp;

                            //cout << "Have to increase V*L(Xst)." << endl;
                            unsigned long long int k_ppcp = constants.getHashKey(constants.numcells_x, X_p_ppcp->sx.x, X_p_ppcp->sx.y);
                            string ks_ppcp = std::to_string(k_ppcp) + X_p_ppcp->hx;
                            opt_gQ = ppcp_obj.computeOptQ(X_p_ppcp, dx_, dy_, Sg, prob_s, prob_f, grid2D, map_un, l, tot_expansions, constants);
                            if (ks_ppcp == ks_p)
                            {
                                //cout << "ppcp Xst expanded" << endl;
                                increase_bound = true;
                                break;
                            }
                            //getchar();
                        }
                        if (increase_bound = true)
                        {
                            break;
                        }
                        end_t1 = std::chrono::system_clock::now();
                        auto elapsed =
                            std::chrono::duration_cast<std::chrono::milliseconds>(end_t1 - start_t1);

                        constants.t1 = (int)elapsed.count();

                        if (with_timeout_flag)
                        {
                            if (constants.t1 > constants.T_alloted)
                            { /////////////////////// If time over, then return immediately
                                increase_bound = false;
                                break;
                            }
                        }

                        //getchar();

                        //restart PPCP::computePath
                    }

                    // if (constants.debug_computePath)
                    //     getchar();

                    // find new pivot from set of prevously explored pivots
                    //auto start_comppivnosol = std::chrono::system_clock::now();
                    X_pnew = computePivotNoSol(X_p, X_piv, dx_, dy_, map_bel, map_st, Sg);
                    // auto end_comppivnosol = std::chrono::system_clock::now();
                    // auto elapsed_comppivnosol =
                    //     std::chrono::duration_cast<std::chrono::microseconds>(end_comppivnosol - start_comppivnosol);
                    // elapsed_comppivnosol_t += (int)elapsed_comppivnosol.count();

                    if (debug)
                        cout << "Since no sol found, computed pivot using ComputePivotNoSol" << endl;
                    // if (constants.debug_computePath)
                    //     getchar();

                    // unsigned long long int kpiv_prev = getHashKey(constants.numcells_x, 29, 37);
                    // string kpiv_prev_s = std::to_string(kpiv_prev) + "uuuufuuuuuuuuuu";

                    // min_gQ_p = constants.inf;
                    // int emer = computeSuboptPath(X_piv, map_bel[kpiv_prev_s], opt_gQ, min_gQ_p, dx_, dy_, map_bel, map_st, Sg, param1, prob_s, prob_f, grid2D, map_un, l, tot_expansions);

                    // X_p = map_bel[kpiv_prev_s];

                    // find previous pivot for re-starting computeusboptpath on
                }
                else
                {
                    //auto start_upmdp = std::chrono::system_clock::now();
                    bool b = updateMDP(X_p, map_bel, map_st, dx_, dy_, Sg, param1);
                    // auto end_upmdp = std::chrono::system_clock::now();
                    // auto elapsed_upmdp =
                    //     std::chrono::duration_cast<std::chrono::microseconds>(end_upmdp - start_upmdp);
                    // elapsed_upmdp_t += (int)elapsed_upmdp.count();

                    if (b == 0)
                    {
                        if (debug)
                            cout << "x pivot g value: " << X_p->sx.g << endl;
                    }

                    if (debug)
                        cout << "updated mdp " << endl;

                    map_bel[keyp_Xu]->v = map_bel[ks_piv]->v;
                   

                    //X_pnew = computePivot(X_p, map_bel, dx_, dy_, dt_, Sg, prob_s, param1);
                    //auto start_comppiv = std::chrono::system_clock::now();
                    X_pnew = computePivot(X_piv, map_bel, map_st, dx_, dy_, Sg, prob_s, param1);
                    // auto end_comppiv = std::chrono::system_clock::now();
                    // auto elapsed_comppiv =
                    //     std::chrono::duration_cast<std::chrono::microseconds>(end_comppiv - start_comppiv);
                    // elapsed_comppiv_t += (int)elapsed_comppiv.count();

                    if (debug)
                        cout << "computed pivot using usual ComputePivot" << endl;
                    //cout << "bel map size after computePivot " << map_bel.size() << endl;
                    //getchar();
                }

                //auto start_upmdprev = std::chrono::system_clock::now();
                updateMDPRev(X_p, X_piv, map_bel, param1, dx_, dy_, prob_s,
                             prob_f, Sg);
                
                 cout << "V value of pivot after update: " << map_bel[ks_piv]->v << endl;
                    cout << "V value of pivot X^u after update: " << map_bel[keyp_Xu]->v << endl;
                // auto end_upmdprev = std::chrono::system_clock::now();
                // auto elapsed_upmdprev =
                //     std::chrono::duration_cast<std::chrono::microseconds>(end_upmdprev - start_upmdprev);
                // elapsed_upmdprev_t += (int)elapsed_upmdprev.count();

                // getchar();

                //check V(Xpiv) and V(Xpiv wth Hu) both should have the same value
                //cout << "V(piv) and V(X_piv) with Hu: Both should be same, if not then error: " << map_bel[ks_piv]->v << "," << map_bel[keyp_Xu]->v << endl;

                // update V_xst as well using min gQ
                //cout << "V_xst before this iteration " << v_hat_Xst << endl;
                v_hat_Xst = map_bel[ks_p]->v;

                //cout << "Final V_xst after this iteration " << v_hat_Xst << endl;

                //getchar();

                ppcp_iters++;
                //cout << "bel map size after computepath " << map_bel.size() << endl;

                //cout << "x pivot" << X_p->sx.x << "," << X_p->sx.y << "," << X_p->hx << endl;
                // if (X_pnew != NULL)
                //     cout << "x pivot new" << X_pnew->sx.x << "," << X_pnew->sx.y << "," << X_pnew->hx << endl;
                // getchar();

                X_p = X_pnew;

                // unsigned long long int key_test = getHashKey(constants.numcells_x, 29, 45);
                // string keys_test = std::to_string(key_test) + "uuffuuuuuuu"; /// v value of failure state

                // if (map_bel.find(keys_test) != map_bel.end())
                // {
                //     cout << "29, 45 uuffuuuuuuu v value, constants.inf:" << abs(map_bel[keys_test]->v-constants.inf) << endl;

                //     if(abs(map_bel[keys_test]->v-constants.inf) < 10)
                //     {
                //         cout << "yayyyy " << keys_test << endl;
                //         getchar();
                //     }

                // }

                //cout << "done" << endl;

            } /// PPCP done
        }

        end_t1 = std::chrono::system_clock::now();
        std::clock_t c_end = std::clock();
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_t1 - start_t1);

        long double cpu_time_elapsed = (c_end - c_start);
        std::cout << "CPU time used: " << cpu_time_elapsed << "ticks";

        constants.t1 = (int)elapsed.count();

        // cout << "cumulated compsuboptpath time (ms): " << elapsed_compsuboptpath_t << endl;
        // cout << "breaking down compsuboptpath: cumulated domaince check time in compsuboptpath (ms): " << elapsed_dominated_t / 1000000 << endl;
        // cout << "           cumulated termsubopt time (ms): " << elapsed_termsubopt_t / 1000 << endl;
        // cout << "           cumulated expansion time (ms): " << elapsed_expansions_t / 1000 << endl;
        // cout << "           cumulated predecessor comp time (ms): " << elapsed_predecessor_comp_t / 1000 << endl;

        // cout << " cumulated computesubopt while loop (ms): " << elapsed_comp_subopt_while_t / 1000 << endl;
        // cout << "cumulated termsubopt + exp + predeccessor comp time (ms): " << (elapsed_termsubopt_t + elapsed_expansions_t + elapsed_predecessor_comp_t) / 1000 << endl;

        // cout << "cumulated updatemdprev time (ms): " << elapsed_upmdprev_t / 1000 << endl;
        // cout << "cumulated comppiv time (ms): " << elapsed_comppiv_t / 1000 << endl;
        // cout << "cumulated updatemdp time (ms): " << elapsed_upmdp_t / 1000 << endl;
        // cout << "cumulated comppivnosol time (micros): " << elapsed_comppivnosol_t << endl;
        // cout << "cumulated total all functions time: " << (elapsed_termsubopt_t + elapsed_expansions_t + elapsed_predecessor_comp_t + elapsed_upmdprev_t + elapsed_comppiv_t + elapsed_upmdp_t + elapsed_comppivnosol_t) / 1000 << endl;

        cout << "lambda, #ppcp iterations, V(X_p), total ppcp expansions, ppcp cpu time, ppcp termination time: " << l << "," << ppcp_iters << "," << v_hat_Xst << "," << tot_expansions << "," << cpu_time_elapsed / CLOCKS_PER_SEC << "," << constants.t1 << endl;
        //}
        *file << std::to_string(start_x) << " " << std::to_string(start_y) << " " << std::to_string(goal_x) << " " << std::to_string(goal_y) << " " << std::to_string(l)
              << " " << std::to_string(ppcp_iters) << " " << std::to_string(v_hat_Xst) << " " << std::to_string(tot_expansions) << " " << std::to_string(cpu_time_elapsed / CLOCKS_PER_SEC) << " " << std::to_string(constants.t1) << endl;
        (*file).close();

        // visualize primary path having all preferred outcomes
        // Publish walls

        visualization_msgs::Marker marker_policy;
        marker_policy.header.frame_id = "odom";
        marker_policy.header.stamp = ros::Time();
        marker_policy.id = 10002;
        marker_policy.color.a = 1.0;
        marker_policy.color.r = 1.0;
        marker_policy.color.g = 0.0;
        marker_policy.color.b = 1.0;
        marker_policy.scale.x = 1;
        marker_policy.scale.y = 1;
        marker_policy.scale.z = 2.0;
        marker_policy.pose.position.z = marker_policy.scale.z / 2.0;
        marker_policy.type = visualization_msgs::Marker::CUBE_LIST;
        for (xy_planner_state *s = &(map_bel[ks_p]->sx); s; s = s->fp)
        {
            geometry_msgs::Point p;
            p.x = s->x + 0.5;
            p.y = s->y + 0.5;
            p.z = 0;
            marker_policy.points.push_back(p);

            if ((s->x == goal_x) && (s->y == goal_y))
            {

                //int motdir =
                //cout << motdir << endl;

                break;
            }
        }
        //Visualize full policy
        std::queue<bel_state *> q_fifo;
        bel_state X_b, X_f;

        q_fifo.push(map_bel[ks_p]);

        // cout << "X_st: " << map_bel[ks_piv]->sx.x << "," << map_bel[ks_piv]->sx.y << "," << map_bel[ks_piv]->hx << "," << map_bel[ks_piv]->bp << endl;
        // getchar();

        bel_state *exp = NULL;

        //float dt = 1;

        while (!q_fifo.empty())
        {

            /// pop fifo
            //cout << "fifo queue sze before:" << q_fifo.size() << endl;
            exp = q_fifo.front();
            q_fifo.pop();

            // if s(goal) exp then done
            if ((exp->sx.x == Sg.x) && (exp->sx.y == Sg.y))
            {
                if (debug)
                {
                    cout << "goal expanded before pivot found!!! So end search" << endl;
                }

                //X_pnew = NULL;

                continue;
                //break;
            }

            unsigned long long int k_exp = constants.getHashKey(constants.numcells_x, exp->sx.x, exp->sx.y);
            string ks_exp = std::to_string(k_exp) + exp->hx;

            /// get successors

            // cout << "best a " << exp->besta << endl;
            // cout << "un id in computePivot: " << map_bel[ks_exp]->un_id << endl;

            bel_succ(map_bel[ks_exp], dx_, dy_, exp->besta, X_b, X_f, map_bel[ks_exp]->un_id);

            // get v value of success bel state
            unsigned long long int ks = constants.getHashKey(constants.numcells_x, X_b.sx.x, X_b.sx.y);
            string ks_succ = std::to_string(ks) + X_b.hx;

            q_fifo.push(map_bel[ks_succ]);

            if (exp->besta == (dx_.size() - 1)) // if sense action then get V value for failure bel state too
            {

                // Get V value of failure bel state
                unsigned long long int kf = constants.getHashKey(constants.numcells_x, X_f.sx.x, X_f.sx.y);
                string ks_f = std::to_string(kf) + X_f.hx;

                if (map_bel.find(ks_f) == map_bel.end())
                {
                    cout << "pivot got no path.ERROR: DEBUG THIS " << X_f.besta << endl;
                    //getchar();
                }
                //cout << "failure pivot: " << X_f.sx.x << "," << X_f.sx.y << X_f.hx << endl;

                q_fifo.push(map_bel[ks_f]);
                //visualize path from failire pivots
                // marker_policy.points.clear();

                for (xy_planner_state *s = &(map_bel[ks_f]->sx); s; s = s->fp)
                {
                    geometry_msgs::Point p;
                    p.x = s->x + 0.5;
                    p.y = s->y + 0.5;
                    p.z = 0;
                    marker_policy.points.push_back(p);

                    if ((s->x == goal_x) && (s->y == goal_y))
                    {

                        //int motdir =
                        //cout << motdir << endl;

                        break;
                    }
                }
                //
                //getchar();
            }
        }

        pub_points_policy.publish(marker_policy);
        map_bel.clear();
        // if (constants.visualize)
        getchar();
        exit(0);
    }
}

/*void socialForce(parameters.param1)
{
      

}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulate_omnirobot");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    // Process parameters
    privateHandle.param<std::string>("world_frame", g_worldFrame, "odom");
    privateHandle.param<std::string>("robot_frame", g_robotFrame,
                                     "base_footprint");

    privateHandle.param<double>("simulation_factor", g_simulationFactor,
                                1.0);                               // set to e.g. 2.0 for 2x speed
    privateHandle.param<double>("update_rate", g_updateRate, 25.0); // in Hz

    double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
    privateHandle.param<double>("pose_initial_x", initialX, 0.0);
    privateHandle.param<double>("pose_initial_y", initialY, 0.0);
    privateHandle.param<double>("pose_initial_theta", initialTheta, 0.0);
    // subopt ppcp parameters
    privateHandle.getParam("l", l);
    privateHandle.getParam("env_num", env_num);
    privateHandle.getParam("s_x", start_x);
    privateHandle.getParam("s_y", start_y);
    privateHandle.getParam("g_x", goal_x);
    privateHandle.getParam("g_y", goal_y);
    privateHandle.getParam("p_door_closed", p_door_closed);

    //////////////////// init params /////////////////////////////////
    parameters param1;
    param1.vnom = 0.0;
    param1.follow_leader = false;
    param1.IDleader = 100;
    param1.d_f = 1;
    param1.follow_wall = false;
    param1.x_wall = 0;
    param1.y_wall = 0;
    param1.dtime = 0;
    param1.d_wall = 0;
    param1.maintain_theta = false;
    param1.m_origin_x = 2;
    param1.m_origin_y = 2;
    param1.m_res = 0.05;
    param1.numcells_x = 200;
    param1.numcells_y = 200;

    ///////////////////////// Get all the parameters ///////////////////////////////////
    privateHandle.getParam("vnom", param1.vnom);

    ////////////////// follow leader params ////////////////////////////////////
    privateHandle.getParam("follow_leader", param1.follow_leader);
    privateHandle.getParam("leader_ID", param1.IDleader);
    privateHandle.getParam("dist_from_leader", param1.d_f);
    //privateHandle.getParam("distance_from_leader:", d_f);

    ////////////////// follow wall parameters ///////////////////////////////////////////////
    privateHandle.getParam("follow_wall", param1.follow_wall);
    privateHandle.getParam("start_x_on_a_wall", param1.x_wall);
    privateHandle.getParam("start_y_on_a_wall", param1.y_wall);
    privateHandle.getParam("dt_for_carrot", param1.dtime);
    privateHandle.getParam("dist_from_wall", param1.d_wall);
    privateHandle.getParam("direction", param1.dir);

    /////////////////////// maintain orientation parameter /////////////////////////////////////////////
    privateHandle.getParam("maintain_orientation", param1.maintain_theta);

    //////////////////// a_info value //////////////////////////
    privateHandle.getParam("a_info", param1.act_info);

    /////////////////////////////////////////////////////////////////
    privateHandle.getParam("m_origin_x", param1.m_origin_x);
    privateHandle.getParam("m_origin_y", param1.m_origin_y);
    privateHandle.getParam("m_res", param1.m_res);
    privateHandle.getParam("numcells_x", param1.numcells_x);
    privateHandle.getParam("numcells_y", param1.numcells_y);

    privateHandle.getParam("goal_x", param1.goal_x);
    privateHandle.getParam("goal_y", param1.goal_y);

    privateHandle.getParam("no_of_steps_followwall", param1.wallstp);
    privateHandle.getParam("no_of_steps_followperson", param1.perstp);
    privateHandle.getParam("gap", param1.gap);

    privateHandle.getParam("t_horizon", param1.tmax);

    privateHandle.getParam("peds_max_no", param1.lmax);

    g_currentPose.getOrigin().setX(initialX);
    g_currentPose.getOrigin().setY(initialY);
    g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));

    // Create ROS subscriber and TF broadcaster
    g_transformBroadcaster.reset(new tf::TransformBroadcaster());

    pub_points_rob = nodeHandle.advertise<visualization_msgs::Marker>(
        "/pedsim/text_points_rob", 10, true);
    pub_points_st = nodeHandle.advertise<visualization_msgs::Marker>(
        "/pedsim/text_points_st", 10, true);
    pub_points_gl = nodeHandle.advertise<visualization_msgs::Marker>(
        "/pedsim/text_points_gl", 10, true);
    goal = nodeHandle.advertise<visualization_msgs::Marker>(
        "/pedsim/text_goal", 10, true);
    start = nodeHandle.advertise<visualization_msgs::Marker>(
        "/pedsim/text_start", 10, true);
    pub_points_path = nodeHandle.advertise<visualization_msgs::MarkerArray>(
        "/pedsim/path", 100);
    constants.pub_points_path_opt = nodeHandle.advertise<visualization_msgs::MarkerArray>(
        "/pedsim/path_opt", 100);

    pub_points_path_del = nodeHandle.advertise<visualization_msgs::MarkerArray>(
        "/pedsim/path_del", 100);
    pub_obs_movAI = nodeHandle.advertise<visualization_msgs::Marker>(
        "/pedsim/obs_movAI", 100, true);
    pub_hidden_movAI = nodeHandle.advertise<visualization_msgs::Marker>(
        "/pedsim/hidden_movAI", 100, true);
    pub_points_policy = nodeHandle.advertise<visualization_msgs::Marker>(
        "/pedsim/policy", 100, true);

    ///////////////////// Right now static obstacles map dont get updated, but if want to uodate, use time synchronized subscriber callback

    // Run

    /////////// Update loop is for modiefed demo ////////////////////
    //boost::thread updateThread( updateLooptest, param1);

    //////////////////// example plan old /////////////////////
    //boost::thread updateThread( examplePlan, param1);
    // , , , maintain_theta

    ////////////////////// planner //////////////////////////////////////////

    // boost::thread updateThread( base_planner, param1);
    // boost::thread updateThread( value_itr, param1);
    // boost::thread updateThread( rtdp, param1);
    boost::thread updateThread(fast_ppcp, param1);
    // boost::thread updateThread(barge, param1);

    ros::spin();
}