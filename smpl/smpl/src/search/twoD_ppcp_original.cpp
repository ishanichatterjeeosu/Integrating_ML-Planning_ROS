#include <iostream>

#include <queue>
#include <smpl/search/common.h>
#include <smpl/search/twoD_ppcp.h>
#include <algorithm>

bool debug_ppcp = false;
using namespace std;

void Ppcp::hello()
{
    bel_state_ppcp *X = new bel_state_ppcp();
    X->sx.x = 2;
    X->sx.y = 3;
    X->hx = "uuuu";
    map_bel_ppcp["a"] = X;
    std::cout << "hello " << std::endl;
}

int Ppcp::getPredMotPrimsPpcp(xy_planner_state_ppcp *s, int act, std::vector<int> &dx_, std::vector<int> &dy_, xy_planner_state_ppcp *s_pred, int sc)
{

    /////////////////////////// Add checks for out of bounds, collision checker, dynamic collision checker!!!!!!!!!!!!!!!!!!! /////////////////////////////////
    // ##############################################################3!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    int dx = 0;
    int dy = 0;
    float c = 0;

    dx += dx_[act];
    dy += dy_[act];
    c = c + sc * sqrt(dx_[act] * dx_[act] + dy_[act] * dy_[act]);

    s_pred->x = s->x - dx;
    s_pred->y = s->y - dy;

    return floor(c);
}

void Ppcp::bel_succ_ppcp(const bel_state_ppcp *X, vector<int> &dx_, vector<int> &dy_, int act, bel_state_ppcp &X_b, bel_state_ppcp &X_f, int u_ctr /*,bel_state& X_succ*/)
{

    xy_planner_state_ppcp S_x;
    xy_planner_state_ppcp S_xf = X->sx;

    string hx_b;
    hx_b.assign(X->hx);
    string hx_f;
    hx_f.assign(X->hx);

    // if (debug_ppcp)
    //     cout << "X->sx x,y, un id: " << X->sx.x << "," << X->sx.y << "," << X->sx.un_id << endl;

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
            // if (debug_ppcp)
            // {
            //     cout << "sense successors:" << endl;
            //     cout << "Xs, Xf: " << X_b.sx.x << X_b.sx.y << X_b.hx << "," << X_f.sx.x << X_f.sx.y << X_f.hx << endl;
            // }
        }
        else
        {
            // if (debug_ppcp)
            // {
            //     cout << "mot prim successor" << endl;
            //     cout << "S_x, y" << (*((X->sx).fp)).x << "," << (*((X->sx).fp)).y << endl;
            // }

            X_b.sx = S_x;
            X_b.hx.assign(X->hx);
        }
    }
}

int Ppcp::computeOptQ(bel_state_ppcp *X_piv_ppcp, std::vector<int> &dx_, std::vector<int> &dy_,

                      xy_planner_state S_goal, float &prob_s,
                      float &prob_f, unsigned int **grid2D, std::unordered_map<unsigned long long int, int> &map_un, double lambda, int &tot_expansions, CommonConstants &constants)
{

    //clear map_st_ppcp
    // for(int tx = 0; tx < constants.numcells_x; tx++)
    // {
    //     for(int ty = 0; ty < constants.numcells_y; ty++)
    //     {
    //         map_st_ppcp[Sg->x][Sg->y]
    //     }
    // }

    X_piv_ppcp->sx.g = constants.inf;
    //X_piv_ppcp->sx.x = 33; X_piv_ppcp->sx.y = 19;
    //cout << "constants.inf: " << std::numeric_limits<double>::max() << endl;
    xy_planner_state_ppcp *Sg = NULL;
    xy_planner_state_ppcp *exp = NULL;
    map_st_ppcp[S_goal.x][S_goal.y].x = S_goal.x;
    map_st_ppcp[S_goal.x][S_goal.y].y = S_goal.y;

    map_st_ppcp[S_goal.x][S_goal.y].g = 0;
    // Sg->g_itr = 0;
    map_st_ppcp[S_goal.x][S_goal.y].g_Q = 0;
    map_st_ppcp[S_goal.x][S_goal.y].besta = -1;
    map_st_ppcp[S_goal.x][S_goal.y].h = constants.sc * constants.getheur2D(X_piv_ppcp->sx.x, X_piv_ppcp->sx.y, S_goal.x, S_goal.y);
    map_st_ppcp[S_goal.x][S_goal.y].f = map_st_ppcp[S_goal.x][S_goal.y].g + map_st_ppcp[S_goal.x][S_goal.y].h;
    unsigned long long int keyg = constants.getHashKey(constants.numcells_x, S_goal.x, S_goal.y);
    map_st_ppcp[S_goal.x][S_goal.y].fp = NULL;
    Sg = &map_st_ppcp[S_goal.x][S_goal.y];

    // cout << "map st ppcp, Sg" << map_st_ppcp[Sg->x][Sg->y].x << "," << map_st_ppcp[Sg->x][Sg->y].y << endl;
    // getchar();
    //cout << "key g" << keyg << endl;
    //add map st
    //map_st_ppcp[Sg->x][Sg->y] = Sg;
    /*float prob_s = 0.8;
    float prob_f = 0.2;*/

    /////////////////////////////////////////// make priorty queue here ////////////////////////////////
    std::priority_queue<xy_planner_state_ppcp *, std::vector<xy_planner_state_ppcp *>, CompareStPpcp> heap; ///// for s(X) search
    heap.push(Sg);

    std::unordered_map<unsigned long long int, int> closed;
    //////////////////////// Compute X^u first //////////////////////////////
    ////// replace all s by u, if f then keep.
    string hu_x_s_replaced;
    hu_x_s_replaced.assign(X_piv_ppcp->hx);

    for (int i = 0; i < hu_x_s_replaced.length(); i++)
    {
        if (hu_x_s_replaced.find("s") != std::string::npos)
        {
            hu_x_s_replaced.replace(hu_x_s_replaced.find("s"), 1, "u");
        }
    }
    string hu_x_s_replaced_cpy;
    hu_x_s_replaced_cpy.assign(hu_x_s_replaced);

    bel_state Y_f;

    //cout << "size of map st " << map_st.size() << endl;
    int xt = 0;

    /*visualization_msgs::MarkerArray rob_cyl;*/

    //cout << "x pivot value " << X_piv_ppcp->sx.g << endl;

    unsigned long long int key_piv = constants.getHashKey(constants.numcells_x, X_piv_ppcp->sx.x, X_piv_ppcp->sx.y);

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

            //cout << "heap is empty. Path not found" << endl;
            X_piv_ppcp->sx.g = constants.inf;

            return constants.inf;
            //break;
        }

        // end_t1 = std::chrono::system_clock::now();
        // auto elapsed =
        //     std::chrono::duration_cast<std::chrono::milliseconds>(end_t1 - start_t1);

        // t1 = (int)elapsed.count();

        // if (t1 > constants.T_alloted)
        // { /////////////////////// If time over, then return immediately
        //     return 500;
        // }

        exp = heap.top();
        //cout << "exp x, y, t, best a: " << exp->x << "," << exp->y <<  "," << exp->besta << endl << "f = " << exp->f << " g = " << exp->g << " h = " << exp->h << endl;

        heap.pop();

        //cout << "size of heap: " << heap.size() << endl;
        // if (heap.size() == 0)
        // {
        //     cout << "size of heap: " << heap.size() << endl;
        // }

        //exp->closed = true;

        xy_planner_state_ppcp *s = new xy_planner_state_ppcp;
        s = exp;
        if (constants.debug_computePath)
        {
            cout << "s,exp address: " << s << "," << exp << endl;
        }

        tot_expansions++;

        /*visualizeMarker(exp->x, exp->y, exp_no, 1,  rob_cyl);*/

        unsigned long long int key_exp = constants.getHashKey(constants.numcells_x, s->x, s->y);

        if (constants.debug_computePath)
        {
            cout << "s x, y, g_Q, g, h, f " << s->x << "," << s->y << "," << s->g_Q << "," << s->g << "," << s->h << "," << s->f << endl;
        }
        //getchar();

        if (constants.debug_computePath)
        {
            cout << "piv x, y " << X_piv_ppcp->sx.x << "," << X_piv_ppcp->sx.y << endl;
        }

        if (s->x == X_piv_ppcp->sx.x && s->y == X_piv_ppcp->sx.y)
        {
            //cout << "S(Xp) pivot expanded yayyyyy" << endl;
            if (constants.debug_computePath)
                cout << "g_Q* of optimal path"
                     << "," << s->g_Q << endl;
            X_piv_ppcp->sx = *s;
            map_st_ppcp[s->x][s->y] = *s;
            //cout << "piv x, y, g" << map_st_ppcp[s->x][s->y].x << "," << map_st_ppcp[s->x][s->y].y << "," << map_st_ppcp[s->x][s->y].g << endl;
            // cout << "x piv, x, y: " << X_piv_ppcp->sx.x << "," << X_piv_ppcp->sx.y << endl;
            // getchar();
            break;
        }

        // unsigned long long int key_tmp = constants.getHashKey(constants.numcells_x, 53, 19);
        // unsigned long long int key_tmp2 = constants.getHashKey(constants.numcells_x, 50, 19);
        // unsigned long long int key_tmp3 = constants.getHashKey(constants.numcells_x, 53,21);
        // unsigned long long int key_goal = constants.getHashKey(constants.numcells_x, 56,20);
        // cout << "keys: 53,19, 50,19, 53, 21" << key_tmp << "," << key_tmp2 << "," << key_tmp3 << endl;

        std::unordered_map<unsigned long long int, int>::const_iterator cls_it_exp = closed.find(key_exp);
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

        if (cls_it_exp == closed.end()) //
        {
            closed[key_exp] = 1;
        }
        else
        {
            if (constants.debug_computePath)
            {
                cout << "because of STL heap, duplicate copies inserted before expanded, so just move to next" << endl;
            }
            continue;
        }

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

        if (got != map_un.end() && (X_piv_ppcp->hx[got->second] == 'u')) // If exp state is unknown, then only sense applies
        {
            u_ctr = got->second;
            // if (constants.debug_computePath)
            // {
            //     cout << "unknown variable to sense: " << u_ctr << endl;
            // }

            for (int a = 0; a < dx_.size() - 1; a++) // get all 8 valid s_preds
            {
                xy_planner_state_ppcp *s_pred = new xy_planner_state_ppcp;
                float c_s = getPredMotPrimsPpcp(s, a, dx_, dy_, s_pred, constants.sc);
                float c_f = constants.sc * constants.Cfail;
                //check valid state
                // if (!isValidState(s_pred, param1, closed, grid2D))
                //     continue;

                if (s_pred->x < 0 || s_pred->x >= constants.numcells_x || s_pred->y < 0 || s_pred->y >= constants.numcells_y)
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

                // This key is for this bounded suboptimal search
                unsigned long long int key_pred = constants.getHashKey(constants.numcells_x, s_pred->x, s_pred->y); //if in closed list then move on

                std::unordered_map<unsigned long long int, int>::const_iterator cls_it_pred = closed.find(key_pred);
                if (cls_it_pred != closed.end())
                {

                    if (constants.debug_computePath)
                    {
                        cout << " s_pred in closed, move to next action " << endl;
                    }

                    continue;
                }
                if (constants.debug_computePath)
                {
                    cout << "s pred sense: " << s_pred->x << "," << s_pred->y << endl;
                }

                std::unordered_map<unsigned long long int, int>::const_iterator gotp1 = map_un.find(key_pred);
                if (gotp1 != map_un.end()) //pred is hidden
                {
                    int u_ctr1 = gotp1->second;
                    //cout << "This case should never come if there are no adjacent hidden vars" << endl;
                    if (X_piv_ppcp->hx[u_ctr1] == 'f') // if it is hidden and known as "f" in
                    {

                        continue;
                    }
                }

                // Get V value of failure bel state V^f
                string h_u_yf;
                //cout << "hu_x: " << hu_x << endl;
                h_u_yf.assign(hu_x_s_replaced.replace(u_ctr, 1, "f")); ///// Ask max H^u(Xp)??
                // // restore it back
                hu_x_s_replaced.assign(hu_x_s_replaced_cpy);

                // // restore it back to pivot
                //hu_x_s_replaced.assign(X_piv_ppcp->hx);

                unsigned long long int key_f = constants.getHashKey(constants.numcells_x, s_pred->x, s_pred->y); // S(X)  upon failure remain at same state s_pred

                string keys = std::to_string(key_f) + h_u_yf; /// v value of failure state
                float v_f;

                if (constants.debug_computePath)
                {
                    cout << "v f string" << h_u_yf << endl;
                }

                //// if doesn't exist, v = admissible heuristic to GOAL!
                if (map_bel_ppcp.find(keys) == map_bel_ppcp.end())
                {
                    // if (constants.debug_computePath)
                    // {
                    //     cout << "keys not updated: " << keys << endl;
                    // }

                    v_f = constants.sc * constants.getheur2D(s_pred->x, s_pred->y, Sg->x, Sg->y); //// heur value to goal from exp state, admissible only!
                }
                else
                    v_f = map_bel_ppcp[keys]->v;

                // Compute g_Q
                if ((X_piv_ppcp->hx[u_ctr] == 'u') || (X_piv_ppcp->hx[u_ctr] == 's'))
                {

                    //cout << "action c : " << a << "," << c << endl;
                    q = prob_s * (c_s + s->g_Q) + (1 - prob_s) * (max(c_f + v_f, c_s + s->g_Q)); ///// cost for failure is still c, because time loss
                    //q = prob_s * (c_s + s->g_Q) + (1 - prob_s) * (c_s + s->g_Q);

                    if (constants.debug_computePath)
                    {
                        cout << "q, p s, c s, s->g, c f, v_f" << q << "," << prob_s << "," << c_s << "," << s->g_Q << "," << c_f << "," << v_f << endl;
                    }
                }

                // Push predecessor
                // Also compute g_itr and update g value as sum of g_its and g_Q

                s_pred->g_Q = q; // g_Q
                //s_pred->g_itr = exp->g_itr + constants.mod_E * c_s;

                s_pred->besta = dx_.size() - 1; // sense action
                s_pred->un_id = u_ctr;

                s_pred->h = constants.getheur2D(s_pred->x, s_pred->y, X_piv_ppcp->sx.x, X_piv_ppcp->sx.y);

                s_pred->f = s_pred->g_Q + s_pred->h;

                s_pred->fp = s;
                if (constants.debug_computePath)
                {
                    cout << "s pred not hidden, g_Q, g value, h value, f value: " << s_pred->x << "," << s_pred->y << "," << s_pred->g_Q << "," << s_pred->g << "," << s_pred->h << "," << s_pred->f << endl;
                    cout << "s pred memory being pushed in heap: " << s_pred << endl;
                }

                heap.push(s_pred);
                // cout << "closed list size when exp = u" << closed.size() << endl;
                // getchar();
                // if (X_piv_ppcp->sx.x == 13 && X_piv_ppcp->sx.y == 5)
                //     getchar();
            }
        }
        else // exp s not unknown, mprims apply for "free", if "blocked" then mprims don't apply
        {
            if (constants.debug_computePath)
            {
                cout << "exp not unknown, check if preds hidden and f" << endl;
                cout << "dx_.size" << dx_.size() << endl;
            }

            for (int a = 0; a < dx_.size() - 1; a++)
            {
                xy_planner_state_ppcp *s_pred = new xy_planner_state_ppcp;

                int c = getPredMotPrimsPpcp(s, a, dx_, dy_, s_pred, constants.sc);

                if (constants.debug_computePath)
                {
                    cout << "s pred h = " << s_pred->x << "," << s_pred->y << endl;
                }

                // if (!isValidState(s_pred, param1, closed, grid2D))
                // {
                //     continue;
                // }

                if (s_pred->x < 0 || s_pred->x >= constants.numcells_x || s_pred->y < 0 || s_pred->y >= constants.numcells_y)
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

                unsigned long long int key_pred_1 = constants.getHashKey(constants.numcells_x, s_pred->x, s_pred->y); //if in closed list then move on

                std::unordered_map<unsigned long long int, int>::const_iterator cls_it_pred_1 = closed.find(key_pred_1);
                if (cls_it_pred_1 != closed.end())
                {
                    if (constants.debug_computePath)
                    {
                        cout << " s_pred in closed, move to next action " << endl;
                    }

                    continue;
                }

                // if s_pred is known to be failure, then treat as obstacle

                std::unordered_map<unsigned long long int, int>::const_iterator gotp = map_un.find(key_pred_1);

                if (gotp != map_un.end()) //pred is hidden
                {
                    int u_ctr2 = gotp->second;
                    if (X_piv_ppcp->hx[u_ctr2] == 'u' || X_piv_ppcp->hx[u_ctr2] == 's')
                    {
                        if (constants.debug_computePath)
                        {
                            cout << "piv has u_ctr = u, so insert into OPEN" << endl;
                        }

                        // put in open list
                        q = constants.w_mp * c + s->g_Q;
                        // Push predecessor
                        s_pred->g_Q = q;
                        //s_pred->g_itr = exp->g_itr + constants.cmax;

                        s_pred->besta = a;
                        s_pred->h = constants.sc * constants.getheur2D(s_pred->x, s_pred->y, X_piv_ppcp->sx.x, X_piv_ppcp->sx.y);

                        s_pred->f = s_pred->g_Q + s_pred->h;

                        s_pred->fp = s;
                        if (constants.debug_computePath)
                        {
                            cout << "s pred, g_Q, g value, h value, f value: " << s_pred->x << "," << s_pred->y << "," << s_pred->g_Q << "," << s_pred->g << "," << s_pred->h << "," << s_pred->f << endl;
                            cout << "s pred memory being pushed in heap: " << s_pred << endl;
                        }

                        heap.push(s_pred);
                        // cout << "closed list size when s pred is hidden " << closed.size() << endl;
                        // getchar();
                        // if (u_ctr > 0)
                        // {
                        //     break;
                        // }
                        // if (X_piv_ppcp->sx.x == 13 && X_piv_ppcp->sx.y == 5)
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
                    //s_pred->g_itr = exp->g_itr + constants.cmax;

                    s_pred->besta = a;
                    s_pred->h = constants.sc * constants.getheur2D(s_pred->x, s_pred->y, X_piv_ppcp->sx.x, X_piv_ppcp->sx.y);

                    s_pred->f = s_pred->g_Q + s_pred->h;

                    s_pred->fp = s;
                    if (constants.debug_computePath)
                    {
                        cout << "s pred not hidden g_Q, g value, h value, f value: " << s_pred->x << "," << s_pred->y << "," << s_pred->g_Q << "," << s_pred->g << "," << s_pred->h << "," << s_pred->f << endl;
                        cout << "s pred memory being pushed in heap: " << s_pred << endl;
                    }
                    // cout << "closed list size when s pred is mot prim " << closed.size() << endl;
                    // getchar();

                    heap.push(s_pred);
                }
            }
        }
    }

    //// path computation
    if (constants.debug_computePath)
    {
        cout << "path" << endl;
    }

    int i = 0;
    visualization_msgs::MarkerArray rob_cyl;
    // int g_Q = -100;
    if (constants.visualize)
    {
        for (xy_planner_state_ppcp *s1 = exp; s1; s1 = s1->fp)
        {
            //solution_path->push_back(state);
            // i++;
            // if (constants.debug_computePath)

            //     cout << s1->x << "," << s1->y << "f = " << s1->f << "g_Q, g = "
            //          << "," << s1->g_Q << "," << s1->g << " h = " << s1->h << endl;

            // constants.visualizeMarkerOpt((s1->x + 0.5), (s1->y + 0.5), i, 1, rob_cyl);
            map_st_ppcp[s1->x][s1->y] = *s1;

            if (s1->fp == NULL)
            {

                //int motdir =
                //cout << motdir << endl;

                break;
            }
        }
    }

    //cout << "path size" << i << endl;
    // if(constants.debug_computePath)
    // {
    // for (int i = 0; i < 2; i++)
    // {
    //     constants.pub_points_path_opt.publish(rob_cyl);
    //     getchar();
    // }

    

    /// empty the priority queue
    //cout << "size ............." << heap.size() << endl;
    heap = priority_queue<xy_planner_state_ppcp *, std::vector<xy_planner_state_ppcp *>, CompareStPpcp>();
    //cout << "size after............." << heap.size() << endl;
    // empty unordered map

    closed.clear();
    return exp->g_Q;
}

bool Ppcp::updateMDP(bel_state_ppcp *X_piv_ppcp, vector<int> &dx_, vector<int> &dy_, xy_planner_state S_goal, CommonConstants &constants)
{

    /*  
    if(b == 0) // no path found 
    {   
        unsigned long long int key = getHashKeyTime(X_piv->sx.x, X_piv->sx.y, X_piv->sx.t, constants.numcells_x, numcells_y, 100);
        map_bel[key] = 
    }*/

    if (X_piv_ppcp->sx.g == constants.inf)
    { /// no path found

        cout << "no path from pivot" << endl;
        unsigned long long int kpiv = constants.getHashKey(constants.numcells_x, X_piv_ppcp->sx.x, X_piv_ppcp->sx.y);
        string ks_piv = std::to_string(kpiv) + X_piv_ppcp->hx;
        map_bel_ppcp[ks_piv] = X_piv_ppcp;
        map_bel_ppcp[ks_piv]->v = constants.inf;

        return 0;
    }

    bel_state_ppcp *X = X_piv_ppcp;

    //////////////////////// Compute X^u first //////////////////////////////
    ////// replace all s by u, if f then keep.
    string hu_x;
    hu_x.assign(X_piv_ppcp->hx);

    for (int i = 0; i < hu_x.length(); i++)
    {
        if (hu_x.find("s") != std::string::npos)
        {
            hu_x.replace(hu_x.find("s"), 1, "u");
        }
    }

    bel_state_ppcp X_b;
    bel_state_ppcp X_f;
    int i = 0;

    while (1)
    {

        if ((X->sx.x == S_goal.x) && (X->sx.y == S_goal.y))
        {

            ////// create the goal then break
            bel_state_ppcp *X_g = new bel_state_ppcp;
            X_g->sx.x = S_goal.x;
            X_g->sx.y = S_goal.y;

            X_g->hx = X->hx;
            X_g->v = 0;
            unsigned long long int kg = constants.getHashKey(constants.numcells_x, S_goal.x, S_goal.y);
            string ks_g = std::to_string(kg) + X_g->hx;
            map_bel_ppcp[ks_g] = X_g;

            break;
        }

        i++;
        // if (debug_ppcp)
        // {
        // cout << "X " << X->sx.x << X->sx.y << X->hx << endl;
        //}

        xy_planner_state_ppcp S_x = X->sx;
        //cout << S_x.x << S_x.y << S_x.t << endl;
        unsigned long long int key = constants.getHashKey(constants.numcells_x, S_x.x, S_x.y);

        // if (debug_ppcp)
        // {
        //cout << "un id:" << map_st_ppcp[key]->un_id << endl;
        // }

        string keys = std::to_string(key) + X->hx;

        //// if bel state does not exist, create new and add

        if (map_bel_ppcp.find(keys) == map_bel_ppcp.end())
        {
            //if (debug_ppcp)
            //cout << "bel state does not exist, create new and add: " << endl;
            bel_state_ppcp *X_new = new bel_state_ppcp;
            X_new->sx = S_x;
            // X_new->sx.y = S_x.y;
            // X_new->sx.t = S_x.t;
            X_new->hx = X->hx;
            xy_planner_state_ppcp S_xtemp = *((X_new->sx).fp);
            //if (debug_ppcp)
            //cout << "Xnew fp: " << S_xtemp.x << "," << S_xtemp.y << endl;
            map_bel_ppcp[keys] = X_new;
        }
        map_bel_ppcp[keys]->sx.x = map_st_ppcp[S_x.x][S_x.y].x;
        map_bel_ppcp[keys]->sx.y = map_st_ppcp[S_x.x][S_x.y].y;
        map_bel_ppcp[keys]->sx.fp = map_st_ppcp[S_x.x][S_x.y].fp; // update front pointer of state sx
        map_bel_ppcp[keys]->v = map_st_ppcp[S_x.x][S_x.y].g_Q;    // update only using g_Q values
        map_bel_ppcp[keys]->besta = map_st_ppcp[S_x.x][S_x.y].besta;
        map_bel_ppcp[keys]->un_id = map_st_ppcp[S_x.x][S_x.y].un_id;

        // if (debug_ppcp)
        // {
        //     cout << " g val, besta, unknown var id: " << map_bel_ppcp[keys]->v << "," << map_bel_ppcp[keys]->besta << "," << map_bel_ppcp[keys]->un_id << endl;
        // }
        //getchar();

        //// update X^u thingy!!! //////
        string keys_Xu = std::to_string(key) + hu_x;
        if (map_bel_ppcp.find(keys_Xu) == map_bel_ppcp.end())
        {

            //cout << "new xu created" << endl;

            bel_state_ppcp *X_u_new = new bel_state_ppcp;
            X_u_new->sx = S_x;
            // X_u_new->sx.y = S_x.y;
            // X_u_new->sx.t = S_x.t;
            X_u_new->hx = hu_x;
            map_bel_ppcp[keys_Xu] = X_u_new;
        }

        map_bel_ppcp[keys_Xu]->v =map_st_ppcp[S_x.x][S_x.y].g_Q; ///// V(X^u) also updated, update only using g_Q values
        map_bel_ppcp[keys_Xu]->un_id = map_st_ppcp[S_x.x][S_x.y].un_id;

        bel_succ_ppcp(X, dx_, dy_, map_st_ppcp[S_x.x][S_x.y].besta, X_b, X_f, map_bel_ppcp[keys_Xu]->un_id); // got the best successor

        X = &X_b; //move to best succ
    }

    return 1;
}

bel_state_ppcp *Ppcp::computePivot(bel_state_ppcp *X_st, vector<int> &dx_, vector<int> &dy_, xy_planner_state &Sg, float &ps, CommonConstants &constants)
{

    ///////////////////// define fifo queue

    std::queue<bel_state_ppcp *> q_fifo;
    bel_state_ppcp X_b, X_f;

    unsigned long long int k_piv = constants.getHashKey(constants.numcells_x, X_st->sx.x, X_st->sx.y);
    string ks_piv = std::to_string(k_piv) + X_st->hx;

    q_fifo.push(map_bel_ppcp[ks_piv]);
    map_bel_ppcp[ks_piv]->bp = NULL;

    // cout << "X_st: " << map_bel_ppcp[ks_piv]->sx.x << "," << map_bel_ppcp[ks_piv]->sx.y << "," << map_bel_ppcp[ks_piv]->hx << "," << map_bel_ppcp[ks_piv]->bp << endl;
    // getchar();

    bel_state_ppcp *exp = NULL;

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
            if (debug_ppcp)
            {
                cout << "expanded state which served as pivot and path not found, just skip to next " << endl;
            }

            continue;
        }

        if (debug_ppcp)
        {
            cout << "exp " << exp->sx.x << exp->sx.y << exp->hx << endl;
        }

        // if s(goal) exp then done
        if ((exp->sx.x == Sg.x) && (exp->sx.y == Sg.y))
        {
            if (debug_ppcp)
            {
                cout << "goal expanded before pivot found!!! So end search" << endl;
            }

            //X_pnew = NULL;

            //continue;
            break;
        }

        unsigned long long int k_exp = constants.getHashKey(constants.numcells_x, exp->sx.x, exp->sx.y);
        string ks_exp = std::to_string(k_exp) + exp->hx;

        if (map_bel_ppcp.find(ks_exp) == map_bel_ppcp.end()) /// surely pivot
        {
            //cout << "exp bel state doesn't exist...surely a pivot, exp key: " << ks_exp << endl;
            if (exp->bp != NULL)
            {
                //cout << "exp bp best a: " << exp->bp->besta << endl;
                //getchar();
            }
            return exp;
        }

        /// get successors
        if (debug_ppcp)
        {
            cout << "best a " << exp->besta << endl;
            cout << "un id in computePivot: " << map_bel_ppcp[ks_exp]->un_id << endl;
        }

        bel_succ_ppcp(map_bel_ppcp[ks_exp], dx_, dy_, exp->besta, X_b, X_f, map_bel_ppcp[ks_exp]->un_id);

        int Ev, vs, vf;

        // compute cost of best successor
        int c_s = floor(constants.sc * sqrt((X_b.sx.x - exp->sx.x) * (X_b.sx.x - exp->sx.x) + (X_b.sx.y - exp->sx.y) * (X_b.sx.y - exp->sx.y)));

        // get v value of success bel state
        unsigned long long int ks = constants.getHashKey(constants.numcells_x, X_b.sx.x, X_b.sx.y);
        string ks_succ = std::to_string(ks) + X_b.hx;

        if (map_bel_ppcp.find(ks_succ) == map_bel_ppcp.end())
        {
            bel_state_ppcp *X_sn = new bel_state_ppcp;
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
            vs = map_bel_ppcp[ks_succ]->v;
            if (debug_ppcp)
            {
                cout << "Vs updated: " << vs << endl;
            }
            // update bp of x_b
            map_bel_ppcp[ks_succ]->bp = exp;
            q_fifo.push(map_bel_ppcp[ks_succ]);
        }

        if (exp->besta == (dx_.size() - 1)) // if sense action then get V value for failure bel state too
        {

            int c_f = constants.sc * constants.Cfail;
            int vf;

            // Get V value of failure bel state
            unsigned long long int kf = constants.getHashKey(constants.numcells_x, X_f.sx.x, X_f.sx.y);
            string ks_f = std::to_string(kf) + X_f.hx;

            if (map_bel_ppcp.find(ks_f) == map_bel_ppcp.end())
            {
                bel_state_ppcp *X_fn = new bel_state_ppcp;
                X_fn->sx = X_f.sx;
                X_fn->hx = X_f.hx;
                X_fn->v = constants.sc * constants.getheur2D(X_fn->sx.x, X_fn->sx.y, Sg.x, Sg.y); //// heur value to goal from exp state, admissible only!
                if (debug_ppcp)
                {
                    cout << "Vf underestimate: " << X_fn->v << endl;
                }

                vf = X_fn->v;
                // update bp of x_f
                X_fn->bp = exp;
                //cout << "X_fn bp: " << X_fn->bp << endl;

                if (debug_ppcp)
                    cout << "X_fn bp best a: " << X_fn->bp->besta << endl;
                q_fifo.push(X_fn);
            }
            else
            {
                vf = map_bel_ppcp[ks_f]->v;
                // update bp of x_f
                map_bel_ppcp[ks_f]->bp = exp;
                q_fifo.push(map_bel_ppcp[ks_f]);
            }

            // Compute V and check condition
            Ev = ps * (c_s + vs) + (1 - ps) * (c_f + vf);
            if (debug_ppcp)
            {
                cout << "ps, cs, vs, cf, vf: " << ps << "," << c_s << "," << vs << "," << c_f << "," << vf << endl;
            }
        }
        else // if mprims
        {
            Ev = constants.w_mp * c_s + vs;
        }
        if (debug_ppcp)
        {
            cout << "exp-> v and ev " << exp->v << "," << Ev << endl;
        }

        //getchar();
        // cout << "fifo queue sze after:" << q_fifo.size() << endl;

        if (exp->v < Ev)
        {
            if (debug_ppcp)
            {
                cout << "exp->v less than Ev" << endl;
                cout << "map_bel_ppcp[ks_exp]->bp->besta, exp->besta: " << map_bel_ppcp[ks_exp]->bp->besta << "," << exp->besta << endl;
            }
            // if (exp->besta == dx_.size() - 1)
            // {
            //     if (debug_ppcp)
            //         cout << "pivot produces stochastic outcome: " << endl;
            //     return exp;
            // }
            // else //if its a mot prim outcome, move on to the next
            // {
            for (bel_state_ppcp *s = exp; s; s = s->bp)
            {
                if (s->bp == NULL)
                {
                    // start state
                    //cout << "X piv = X st in computePivot: " << s->sx.x << "," << s->sx.y << "," << s->hx << endl;
                    return s;
                }
                if (s->bp->besta == dx_.size() - 1)
                {
                    //cout << "s->besta, " << endl;
                    //cout << "X piv in computePivot: " << s->sx.x << "," << s->sx.y << "," << s->hx << endl;
                    return s;
                }
                //cout << "Intermediate Xs: " << s->sx.x << "," << s->sx.y << "," << s->hx << endl;
                //cout << "bp " << s->bp << endl;
                //getchar();
            }
        }
    }

    return NULL;
}

