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

#ifndef SMPL_META_MHASTAR_DTS_H
#define SMPL_META_MHASTAR_DTS_H

// standard includes
#include <iomanip>
#include <ostream>
#include <random>

// system includes
#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/heap/intrusive_heap.h>
#include <smpl/search/mhastar_base.h> // for MHASearchState declaration
#include <smpl/time.h>

namespace smpl {

class MetaMHAstarDTS : public SBPLPlanner
{
public:

    MetaMHAstarDTS(
        DiscreteSpaceInformation* environment,
        Heuristic* hanchor,
        Heuristic** heurs,
        int hcount);

    ~MetaMHAstarDTS();

    /// \name Required Functions from SBPLPlanner
    ///@{
    int set_start(int start_state_id) override;
    int set_goal(int goal_state_id) override;

    int replan(
        double allocated_time_sec,
        std::vector<int>* solution) override;

    int replan(
        double allocated_time_sec,
        std::vector<int>* solution,
        int* solcost) override;

    int force_planning_from_scratch() override;

    void costs_changed(const StateChangeQuery& stateChange) override;

    int set_search_mode(bool first_solution_unbounded) override;
    ///@}

    /// \name Reimplemented Functions from SBPLPlanner
    ///@{
    int replan(
        std::vector<int>* solution,
        ReplanParams params) override;

    int replan(
        std::vector<int>* solution,
        ReplanParams params,
        int* solcost) override;

    int force_planning_from_scratch_and_free_memory() override;

    void    set_initialsolution_eps(double eps) override;
    double  get_initial_eps() override;

    double  get_solution_eps() const override;
    double  get_final_epsilon() override;
    double  get_final_eps_planning_time() override;
    double  get_initial_eps_planning_time() override;
    int     get_n_expands() const override;
    int     get_n_expands_init_solution() override;
    void    get_search_stats(std::vector<PlannerStats>* s) override;
    ///@}

    /// \name Homogeneous accessor methods for search mode and timing parameters
    // @{

    void    set_initial_eps(double eps) { return set_initialsolution_eps(eps); }
    void    set_final_eps(double eps);
    void    set_dec_eps(double eps);
    void    set_max_expansions(int expansion_count);
    void    set_max_time(double max_time);

    // double get_initial_eps();
    double  get_final_eps() const;
    double  get_dec_eps() const;
    int     get_max_expansions() const;
    double  get_max_time() const;

    ///@}

private:

    // Related objects
    Heuristic* m_hanchor;
    Heuristic** m_heurs;
    int m_hcount;           // number of additional heuristics used

    ReplanParams m_params;
    int m_max_expansions;

    double m_eps;           // current w_1

    /// suboptimality bound satisfied by the last search
    double m_eps_satisfied;

    int m_num_expansions;   // current number of expansion
    double m_elapsed;       // current amount of seconds

    int m_call_number;

    MHASearchState* m_start_state;
    MHASearchState* m_goal_state;

    std::vector<MHASearchState*> m_search_states;
    std::vector<int> m_graph_to_search_state;

    std::minstd_rand m_rng;
    std::uniform_real_distribution<double> m_uniform;
    std::vector<int> m_best_hvals;
    std::vector<double> m_alphas;
    std::vector<double> m_betas;

    struct HeapCompare
    {
        bool operator()(
            const MHASearchState::HeapData& s,
            const MHASearchState::HeapData& t) const
        {
            return s.f < t.f;
        }
    };

    typedef intrusive_heap<MHASearchState::HeapData, HeapCompare> rank_pq;

    // m_open[0] contain the actual OPEN list sorted by g(s) + h(s)
    // m_open[i], i > 0, maintains a copy of the PSET for each additional
    // heuristic, sorted by rank(s, i). The PSET maintains, at all times, those
    // states which are in the OPEN list, have not been closed inadmissably,
    // and satisfy the P-CRITERION
    rank_pq* m_open;

    bool check_params(const ReplanParams& params);

    bool time_limit_reached() const;

    int num_heuristics() const { return m_hcount + 1; }
    MHASearchState* get_state(int state_id);
    void init_state(MHASearchState* state, int state_id);
    void reinit_state(MHASearchState* state);
    void reinit_search();
    void clear_open_lists();
    void clear();
    int compute_key(MHASearchState* state, int hidx);
    int choose_search();
    void update_meta_method(int hidx);
    void expand(MHASearchState* state, int hidx);
    MHASearchState* state_from_open_state(MHASearchState::HeapData* open_state);
    int compute_heuristic(int state_id, int hidx);
    int get_minf(rank_pq& pq) const;
    void insert_or_update(MHASearchState* state, int hidx);
    MHASearchState* select_state(int hidx);

    void extract_path(std::vector<int>* solution_path, int* solcost);

    bool closed_in_anc_search(MHASearchState* state) const;
    bool closed_in_add_search(MHASearchState* state) const;
    bool closed_in_any_search(MHASearchState* state) const;
};

} // namespace smpl

#endif
