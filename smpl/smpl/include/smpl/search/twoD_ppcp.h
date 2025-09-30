#ifndef twoD_PPCP_H
#define twoD_PPCP_H

#include <unordered_map>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>
#include <smpl/search/common.h>

class Ppcp
{

public:
    Ppcp(int numcells_x, int numcells_y)
    {
        map_st_ppcp = new xy_planner_state_ppcp *[numcells_y];
        for (int tx = 0; tx < numcells_x; tx++)
        {
            map_st_ppcp[tx] = new xy_planner_state_ppcp[numcells_x];
        }
        
    }
    void hello();
    // int getPredMotPrimsPpcp(xy_planner_state_ppcp *s, int act, std::vector<int> &dx_, std::vector<int> &dy_, xy_planner_state_ppcp *s_pred, int sc);
    // void bel_succ_ppcp(const bel_state_ppcp *X, std::vector<int> &dx_, std::vector<int> &dy_, int act, bel_state_ppcp &X_b, bel_state_ppcp &X_f, int u_ctr /*,bel_state& X_succ*/);
    // int computeOptQ(bel_state_ppcp *X_piv, std::vector<int> &dx_, std::vector<int> &dy_,
    //                 xy_planner_state S_goal, float &prob_s,
    //                 float &prob_f, unsigned int **grid2D, std::unordered_map<unsigned long long int, int> &map_un, double lambda, int &tot_expansions, CommonConstants &constants);
    // bool updateMDP(bel_state_ppcp *X_piv_ppcp, std::vector<int> &dx_, std::vector<int> &dy_, xy_planner_state S_goal, CommonConstants& constants);

    // bel_state_ppcp *computePivot(bel_state_ppcp *X_st, std::vector<int> &dx_, std::vector<int> &dy_, xy_planner_state& Sg, float &ps, CommonConstants& constants);
    xy_planner_state_ppcp **map_st_ppcp;
    std::unordered_map<std::string, bel_state_ppcp *> map_bel_ppcp;
    
};

class CompareStPpcp
{
public:
    bool operator()(xy_planner_state_ppcp *n1, xy_planner_state_ppcp *n2)
    {
        return n1->f > n2->f;
    }
};

#endif