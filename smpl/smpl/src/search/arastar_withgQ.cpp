////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <smpl/search/arastar.h>
#include <smpl/search/twoD_ppcp.h>
#include <smpl/search/common.h>

#include <algorithm>
#include <unordered_map>

// system includes
#include <sbpl/utils/key.h>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>
#include <smpl/types.h>
#include <smpl/graph/manip_lattice.h>

namespace smpl
{

    static const char *SLOG = "search";
    static const char *SELOG = "search.expansions";

    ARAStar::ARAStar(
        DiscreteSpaceInformation *space,
        Heuristic *heur)
        : SBPLPlanner(),
          m_space(space),
          m_heur(heur),
          m_time_params(),
          m_initial_eps(1.0),
          m_final_eps(1.0),
          m_delta_eps(1.0),
          m_allow_partial_solutions(false),
          m_states(),
          m_start_state_id(-1),
          m_goal_state_id(-1),
          m_open(),
          m_incons(),
          m_curr_eps(1.0),
          m_iteration(1),
          m_call_number(0),
          m_last_start_state_id(-1),
          m_last_goal_state_id(-1),
          m_last_eps(1.0),
          m_expand_count_init(0),
          m_expand_count(0),
          m_search_time_init(clock::duration::zero()),
          m_search_time(clock::duration::zero()),
          m_satisfied_eps(std::numeric_limits<double>::infinity())
    {
        environment_ = space;

        m_time_params.bounded = true;
        m_time_params.improve = false;
        m_time_params.type = TimeParameters::TIME;
        m_time_params.max_expansions_init = 0;
        m_time_params.max_expansions = 0;
        m_time_params.max_allowed_time_init = clock::duration::zero();
        m_time_params.max_allowed_time = clock::duration::zero();
    }

    ARAStar::~ARAStar()
    {
        for (SearchState *s : m_states)
        {
            if (s != NULL)
            {
                delete s;
            }
        }
    }

    enum ReplanResultCode
    {
        SUCCESS = 0,
        PARTIAL_SUCCESS,
        START_NOT_SET,
        GOAL_NOT_SET,
        TIMED_OUT,
        EXHAUSTED_OPEN_LIST
    };

    int ARAStar::replan(
        const TimeParameters &params,
        std::vector<int> *solution,
        int *cost)
    {
        // Ppcp obj(2,3);
        // obj.hello();
        // SMPL_INFO_NAMED(SLOG, );
        SMPL_INFO_NAMED(SLOG, "Find path to goal");

        if (m_start_state_id < 0)
        {
            SMPL_ERROR_NAMED(SLOG, "Start state not set");
            return !START_NOT_SET;
        }
        if (m_goal_state_id < 0)
        {
            SMPL_ERROR_NAMED(SLOG, "Goal state not set");
            return !GOAL_NOT_SET;
        }

        m_time_params = params;

        // Fast-PPCP outer loop should be here

        SMPL_INFO_NAMED(SLOG, "start, goal state id: %d, %d", m_start_state_id, m_goal_state_id);

        SearchState *start_state = getSearchState(m_start_state_id); // id 1. Start state has goal coordinates
        SearchState *goal_state = getSearchState(m_goal_state_id);   // id 0. Goal state has start coordinates

        if (m_start_state_id != m_last_start_state_id)
        {
            SMPL_INFO_NAMED(SLOG, "Reinitialize search");
            m_open.clear();
            m_incons.clear();
            ++m_call_number; // trigger state reinitializations

            reinitSearchState(start_state); // goal state
            reinitSearchState(goal_state);  // start state

            start_state->g = 0;
            start_state->gQ = 0;
            start_state->f = computeKey(start_state);
            m_open.push(start_state);

            m_iteration = 1; // 0 reserved for "not closed on any iteration"

            m_expand_count_init = 0;
            m_search_time_init = clock::duration::zero();

            m_expand_count = 0;
            m_search_time = clock::duration::zero();

            m_curr_eps = m_initial_eps;

            m_satisfied_eps = std::numeric_limits<double>::infinity();

            m_last_start_state_id = m_start_state_id;
        }

        if (m_goal_state_id != m_last_goal_state_id)
        {
            SMPL_INFO_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
            recomputeHeuristics();
            reorderOpen();

            m_last_goal_state_id = m_goal_state_id;
        }

        auto start_time = clock::now();
        int num_expansions = 0;
        clock::duration elapsed_time = clock::duration::zero();

        int err;
        while (m_satisfied_eps > m_final_eps)
        {
            if (m_curr_eps == m_satisfied_eps)
            {
                if (!m_time_params.improve)
                {
                    break;
                }
                // begin a new search iteration
                ++m_iteration;
                m_curr_eps -= m_delta_eps;
                m_curr_eps = std::max(m_curr_eps, m_final_eps);
                for (SearchState *s : m_incons)
                {
                    s->incons = false;
                    m_open.push(s);
                }
                reorderOpen();
                m_incons.clear();
                SMPL_INFO_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps); // getchar();
            }
            // initialize the Xpiv belief state

            bel_state *X_piv = new bel_state;
            X_piv->sx.state_id = goal_state->state_id;

            // initialize map_un
            // Initialize state ids for h1 and h2
            RobotState state_h1, state_h2; // do h2 for later, first finish the search
            auto state_coord_h1 = RobotCoord(6);
            state_h1.push_back(2.14847);
            state_h1.push_back(1.05692);
            state_h1.push_back(1.73111);
            state_h1.push_back(5.63979);
            state_h1.push_back(0.911667);
            state_h1.push_back(2.84393);

            m_space->stateToCoord(state_h1, state_coord_h1);
            // //SMPL_INFO_STREAM_NAMED(G_LOG, "  coord: " << start_coord); getchar();

            int m_state_h1_id = m_space->getOrCreateState(state_coord_h1, state_h1);

            std::cout << "state id" << m_state_h1_id << std::endl;

            map_un[0].push_back(m_state_h1_id); // once you get the failure pose, do for iter 2 as well

            // assign string
            std::string s1;

            for (int i = 0; i < map_un.size(); i++)
            {

                s1.append("u");
            }

            (X_piv->hx).assign(s1);

            err = improvePath(start_time, goal_state, num_expansions, elapsed_time, X_piv);
            if (m_curr_eps == m_initial_eps)
            {
                m_expand_count_init += num_expansions;
                m_search_time_init += elapsed_time;
            }
            if (!err)
            {
                break;
            }
            SMPL_INFO_NAMED(SLOG, "Improved solution");
            m_satisfied_eps = m_curr_eps;
        }

        m_search_time += elapsed_time;
        m_expand_count += num_expansions;

        // if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
        //     if (m_allow_partial_solutions && !m_open.empty()) {
        //         SearchState* next_state = m_open.min();
        //         extractPath(next_state, *solution, *cost);
        //         return !SUCCESS;
        //     }
        //     return !err;
        // }

        extractPath(goal_state, *solution, *cost);
        return !SUCCESS;
    }

    int ARAStar::replan(
        double allowed_time,
        std::vector<int> *solution)
    {
        int cost;
        return replan(allowed_time, solution, &cost);
    }

    // decide whether to start the search from scratch
    //
    // if start changed
    //     reset the search to its initial state
    // if goal changed
    //     reevaluate heuristics
    //     reorder the open list
    //
    // case scenario_hasnt_changed (start and goal the same)
    //   case have solution for previous epsilon
    //       case epsilon lowered
    //           reevaluate heuristics and reorder the open list
    //       case epsilon raised
    //           pass
    //   case dont have solution
    //       case epsilon lowered
    //           reevaluate heuristics and reorder the open list
    //       case epsilon raised
    //           reevaluate heuristics and reorder the open list
    // case scenario_changed
    int ARAStar::replan(
        double allowed_time,
        std::vector<int> *solution,
        int *cost)
    {
        TimeParameters tparams = m_time_params;
        if (tparams.max_allowed_time_init == tparams.max_allowed_time)
        {
            // NOTE/TODO: this may lead to awkward behavior, if the caller sets the
            // allowed time to the current repair time, the repair time will begin
            // to track the allowed time for further calls to replan. perhaps set
            // an explicit flag for using repair time or an indicator value as is
            // done with ReplanParams
            tparams.max_allowed_time_init = to_duration(allowed_time);
            tparams.max_allowed_time = to_duration(allowed_time);
        }
        else
        {
            tparams.max_allowed_time_init = to_duration(allowed_time);
            // note: retain original allowed improvement time
        }
        return replan(tparams, solution, cost);
    }

    int ARAStar::replan(
        std::vector<int> *solution,
        ReplanParams params)
    {
        int cost;
        return replan(solution, params, &cost);
    }

    int ARAStar::replan(
        std::vector<int> *solution,
        ReplanParams params,
        int *cost)
    {
        // note: if replan fails before internal time parameters are updated (this
        // happens if the start or goal has not been set), then the internal
        // epsilons may be affected by this set of ReplanParams for future calls to
        // replan where ReplanParams is not used and epsilon parameters haven't been
        // set back to their desired values.
        TimeParameters tparams;
        convertReplanParamsToTimeParams(params, tparams);
        return replan(tparams, solution, cost);
    }

    /// Force the planner to forget previous search efforts, begin from scratch,
    /// and free all memory allocated by the planner during previous searches.
    int ARAStar::force_planning_from_scratch_and_free_memory()
    {
        force_planning_from_scratch();
        m_open.clear();
        for (SearchState *s : m_states)
        {
            if (s != NULL)
            {
                delete s;
            }
        }
        m_states.clear();
        m_states.shrink_to_fit();
        return 0;
    }

    /// Return the suboptimality bound of the current solution for the current search.
    double ARAStar::get_solution_eps() const
    {
        return m_satisfied_eps;
    }

    /// Return the number of expansions made in progress to the final solution.
    int ARAStar::get_n_expands() const
    {
        return m_expand_count;
    }

    /// Return the initial suboptimality bound
    double ARAStar::get_initial_eps()
    {
        return m_initial_eps;
    }

    /// Return the time consumed by the search in progress to the initial solution.
    double ARAStar::get_initial_eps_planning_time()
    {
        return to_seconds(m_search_time_init);
    }

    /// Return the time consumed by the search in progress to the final solution.
    double ARAStar::get_final_eps_planning_time()
    {
        return to_seconds(m_search_time);
    }

    /// Return the number of expansions made in progress to the initial solution.
    int ARAStar::get_n_expands_init_solution()
    {
        return m_expand_count_init;
    }

    /// Return the final suboptimality bound.
    double ARAStar::get_final_epsilon()
    {
        return m_final_eps;
    }

    /// Return statistics for each completed search iteration.
    void ARAStar::get_search_stats(std::vector<PlannerStats> *s)
    {
        PlannerStats stats;
        stats.eps = m_curr_eps;
        //    stats.cost; // TODO: implement
        stats.expands = m_expand_count;
        stats.time = to_seconds(m_search_time);
        s->push_back(stats);
    }

    /// Set the desired suboptimality bound for the initial solution.
    void ARAStar::set_initialsolution_eps(double eps)
    {
        m_initial_eps = eps;
    }

    /// Set the goal state.
    int ARAStar::set_goal(int goal_state_id)
    {
        m_goal_state_id = goal_state_id;
        return 1;
    }

    /// Set the start state.
    int ARAStar::set_start(int start_state_id)
    {
        m_start_state_id = start_state_id;
        return 1;
    }

    /// Force the search to forget previous search efforts and start from scratch.
    int ARAStar::force_planning_from_scratch()
    {
        m_last_start_state_id = -1;
        m_last_goal_state_id = -1;
        return 0;
    }

    /// Set whether the number of expansions is bounded by time or total expansions
    /// per call to replan().
    int ARAStar::set_search_mode(bool first_solution_unbounded)
    {
        m_time_params.bounded = !first_solution_unbounded;
        return 0;
    }

    /// Notify the search of changes to edge costs in the graph.
    void ARAStar::costs_changed(const StateChangeQuery &changes)
    {
        force_planning_from_scratch();
    }

    // Recompute heuristics for all states.
    void ARAStar::recomputeHeuristics()
    {
        for (SearchState *s : m_states)
        {
            if (s != NULL)
            {
                s->h = m_heur->GetGoalHeuristic(s->state_id);
            }
        }
    }

    // Convert TimeParameters to ReplanParams. Uses the current epsilon values
    // to fill in the epsilon fields.
    void ARAStar::convertTimeParamsToReplanParams(
        const TimeParameters &t,
        ReplanParams &r) const
    {
        r.max_time = to_seconds(t.max_allowed_time_init);
        r.return_first_solution = !t.bounded && !t.improve;
        if (t.max_allowed_time_init == t.max_allowed_time)
        {
            r.repair_time = -1.0;
        }
        else
        {
            r.repair_time = to_seconds(t.max_allowed_time);
        }

        r.initial_eps = m_initial_eps;
        r.final_eps = m_final_eps;
        r.dec_eps = m_delta_eps;
    }

    // Convert ReplanParams to TimeParameters. Sets the current initial, final, and
    // delta eps from ReplanParams.
    void ARAStar::convertReplanParamsToTimeParams(
        const ReplanParams &r,
        TimeParameters &t)
    {
        t.type = TimeParameters::TIME;

        t.bounded = !r.return_first_solution;
        t.improve = !r.return_first_solution;

        t.max_allowed_time_init = to_duration(r.max_time);
        if (r.repair_time > 0.0)
        {
            t.max_allowed_time = to_duration(r.repair_time);
        }
        else
        {
            t.max_allowed_time = t.max_allowed_time_init;
        }

        m_initial_eps = r.initial_eps;
        m_final_eps = r.final_eps;
        m_delta_eps = r.dec_eps;
    }

    // Test whether the search has run out of time.
    bool ARAStar::timedOut(
        int elapsed_expansions,
        const clock::duration &elapsed_time) const
    {
        if (!m_time_params.bounded)
        {
            return false;
        }

        switch (m_time_params.type)
        {
        case TimeParameters::EXPANSIONS:
            if (m_satisfied_eps == std::numeric_limits<double>::infinity())
            {
                return elapsed_expansions >= m_time_params.max_expansions_init;
            }
            else
            {
                return elapsed_expansions >= m_time_params.max_expansions;
            }
        case TimeParameters::TIME:
            if (m_satisfied_eps == std::numeric_limits<double>::infinity())
            {
                return elapsed_time >= m_time_params.max_allowed_time_init;
            }
            else
            {

                return elapsed_time >= m_time_params.max_allowed_time;
            }
        case TimeParameters::USER:
            return m_time_params.timed_out_fun();
        default:
            SMPL_ERROR_NAMED(SLOG, "Invalid timer type");
            return true;
        }

        return true;
    }

    // Expand states to improve the current solution until a solution within the
    // current suboptimality bound is found, time runs out, or no solution exists.
    int ARAStar::improvePath(
        const clock::time_point &start_time,
        SearchState *goal_state,
        int &elapsed_expansions,
        clock::duration &elapsed_time, bel_state *X_piv)
    {
        int term_iter = 0;
        X_piv->sx.g = INFINITECOST;
        std::string hu_x_s_holder;
        hu_x_s_holder.assign(X_piv->hx);

        while (!m_open.empty())
        {
            SearchState *min_state = m_open.min();

            auto now = clock::now();
            elapsed_time = now - start_time;

            // path to goal found
            // if (min_state->f >= goal_state->f || min_state == goal_state) {
            //     SMPL_INFO_NAMED(SLOG, "Found path to goal");
            //     return SUCCESS;
            // }

            // modfy termination
            // count the search number in termination
            if (min_state->f >= goal_state->f || min_state->state_id == m_goal_state_id)
            {
                term_iter++;
                SMPL_INFO_NAMED(SLOG, "Found path to goal");
                return SUCCESS;
            }

            // if (timedOut(elapsed_expansions, elapsed_time)) {
            //     SMPL_INFO_NAMED(SLOG, "Ran out of time");
            //     return TIMED_OUT;
            // }

            SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

            m_open.pop();

            assert(min_state->iteration_closed != m_iteration);
            assert(min_state->g != INFINITECOST);

            min_state->iteration_closed = m_iteration;
            min_state->eg = min_state->g;

            expand(min_state, X_piv, hu_x_s_holder);

            ++elapsed_expansions;
        }

        return EXHAUSTED_OPEN_LIST;
    }

    // Expand a state, updating its successors and placing them into OPEN, CLOSED,
    // and INCONS list appropriately.
    void ARAStar::expand(SearchState *s, bel_state *X_piv, std::string& hu_x_s_holder)
    {
        SMPL_DEBUG_NAMED(SELOG, "  %zu successors", m_succs.size());
        float q;
        // Before the successors check
        // if expanded state is unknown in Pivot
        std::vector<int>::iterator got = std::find(map_un[term_iter].begin(), map_un[term_iter].end(), s->state_id);
        int u_ctr = (term_iter)*num_unk + (got - map_un[term_iter].begin());
        if (got != map_un[term_iter].end() & X_piv->hx[u_ctr] == 'u')
        {
            m_succs.clear();
            m_costs.clear();
            m_space->GetSuccs(s->state_id, &m_succs, &m_costs); // get valid preds
            for (size_t sidx = 0; sidx < m_succs.size(); ++sidx)
            {
                int succ_state_id = m_succs[sidx];
                
                //get new predecessor state
                SearchState *succ_state = getSearchState(succ_state_id);
                reinitSearchState(succ_state);

                //get cosst of succ and failure
                float cost_s = m_costs[sidx];
                float cost_f = Cfail
                 // find the corresponding failure belief state and its V value
                std::string h_u_yf;
                h_u_yf.assign(hu_x_s_holder.replace(u_ctr, 1, "f"))
                 // // restore it back to pivot
                hu_x_s_holder.assign(X_piv->hx)
                int key_f = succ_state_id;
                std::string keys = std::to_string(key_f) + h_u_yf; /// v value of failure state
                float v_f;
                //if doesnt exist in map_bel then use underestimate heur admissible to goal coordinates(meaning START_STATE in this search) else grab from map_bel
                 if (map_bel.find(keys) == map_bel.end())
                {
                   
                    v_f = m_heur->GetStartHeuristic(succ_state_id); // Verified: make Sure heuristic from goal, which is "start_state" for this planner
                    
                }
                else
                {
                    v_f = map_bel[keys]->v;
                }
                if ((X_piv->hx[u_ctr] == 'u') || (X_piv->hx[u_ctr] == 's'))
                {
                    //compute gQ
                    q = prob_s * (cost_s + s->g_Q) + (1 - prob_s) * (cost_f + v_f);

                }
                 else
                {
                    std::cout << "should never come here, exit!!!!!!!!!!!!!!!!!" << std::endl;
                    exit(0);
                }
                // push predecessor

                succ_state->g_Q = q; // g_Q

                succ_state->g = s->g + mod_E; // g_itr

                succ_state->besta = actions_size - 1; // sense action
                succ_state->un_id = u_ctr;

                //succ_state->h = constants.sc * constants.getheur2D(succ_state->x, succ_state->y, X_piv->sx.x, X_piv->sx.y);

                succ_state->f = succ_state->g + (unsigned int) succ_state->h; // minimize g_iter, given cost setting of |E| and 1, euclidean heuristic is admissible

                succ_state->bp = s;
                
                m_open.push(succ_state);
                
            }   
        }
        else
        {
        // find predecessors
        // do validty
        // if preds unknown in piv and fail, then continue
        }
        // else
        // add normal costs

        //}

        //

        //}

        // for (size_t sidx = 0; sidx < m_succs.size(); ++sidx)
        // {
        //     int succ_state_id = m_succs[sidx];
        //     int cost = m_costs[sidx];

        //     SearchState *succ_state = getSearchState(succ_state_id);
        //     reinitSearchState(succ_state);

        //     int new_cost = s->eg + cost;
        //     SMPL_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
        //     if (new_cost < succ_state->g)
        //     {
        //         succ_state->g = new_cost;
        //         succ_state->bp = s;
        //         if (succ_state->iteration_closed != m_iteration)
        //         {
        //             succ_state->f = computeKey(succ_state);
        //             if (m_open.contains(succ_state))
        //             {
        //                 m_open.decrease(succ_state);
        //             }
        //             else
        //             {
        //                 m_open.push(succ_state);
        //             }
        //         }
        //         else if (!succ_state->incons)
        //         {
        //             m_incons.push_back(succ_state);
        //         }
        //     }
        // }
    }

    // Recompute the f-values of all states in OPEN and reorder OPEN.
    void ARAStar::reorderOpen()
    {
        for (auto it = m_open.begin(); it != m_open.end(); ++it)
        {
            (*it)->f = computeKey(*it);
        }
        m_open.make();
    }

    int ARAStar::computeKey(SearchState *s) const
    {
        // return s->g + (unsigned int)(m_curr_eps * s->h);
        return s->g + (unsigned int)(s->h);
    }

    // Get the search state corresponding to a graph state, creating a new state if
    // one has not been created yet.
    ARAStar::SearchState *ARAStar::getSearchState(int state_id)
    {
        if (m_states.size() <= state_id)
        {
            m_states.resize(state_id + 1, nullptr);
        }

        auto &state = m_states[state_id];
        if (state == NULL)
        {
            state = createState(state_id);
        }

        return state;
    }

    // Create a new search state for a graph state.
    ARAStar::SearchState *ARAStar::createState(int state_id)
    {
        assert(state_id < m_states.size());

        SearchState *ss = new SearchState;
        ss->state_id = state_id;
        ss->call_number = 0;

        return ss;
    }

    // Lazily (re)initialize a search state.
    void ARAStar::reinitSearchState(SearchState *state)
    {
        if (state->call_number != m_call_number)
        {
            SMPL_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
            state->g = INFINITECOST;
            state->gQ = -100;
            state->h = m_heur->GetGoalHeuristic(state->state_id);
            state->f = INFINITECOST;
            state->eg = INFINITECOST;
            state->iteration_closed = 0;
            state->call_number = m_call_number;
            state->bp = nullptr;
            state->incons = false;
        }
    }

    // Extract the path from the start state up to a new state.
    void ARAStar::extractPath(
        SearchState *to_state,
        std::vector<int> &solution,
        int &cost) const
    {
        for (SearchState *s = to_state; s; s = s->bp)
        {
            solution.push_back(s->state_id);
            std::cout << "State id of path" << s->state_id << std::endl;
        }
        // Do it for forward search
        std::reverse(solution.begin(), solution.end());
        SMPL_INFO_NAMED(SLOG, "Came hrer");
        cost = to_state->g;
    }

} // namespace smpl
