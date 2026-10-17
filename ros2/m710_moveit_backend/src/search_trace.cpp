/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

// Adapted growTree/solve only, upstream OMPL 1.7.0. See SEARCH_TRACE_SOURCES.md.

#include "search_trace.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/String.h>
using namespace ompl;
using namespace ompl::geometric;
namespace m710 {
std::shared_ptr<SearchTrace> active_search_trace;
thread_local TraceJson validity_witness=nullptr;
thread_local bool witness_requested=false;
thread_local SampleRecorder* extension_recorder=nullptr;
void traceFailure(const char* checker,const TraceJson& original) {
  if(witness_requested) validity_witness={{"checker",checker},{"original",original}};
}
std::vector<double> TraceRRTConnect::values(const base::State* s) const {
  std::vector<double> q;si_->getStateSpace()->copyToReals(q,s);return q;
}
void TraceRRTConnect::extension(const base::State* from,const base::State* to,bool start,bool accepted,const char* outcome) {
  const auto sequence=event_count_++;
  if(events_.size()<trace_->event_limit)
    events_.push_back({sequence,start,accepted,values(from),values(to),event_sample_begin_,samples_.size(),outcome});
}
base::PlannerStatus TraceRRTConnect::solve(const base::PlannerTerminationCondition& ptc) {
  // MoveIt assumes its concrete StateValidityChecker; never replace it.
  // A cached OMPL context must bind this solve's request, not its creation request.
  if(active_search_trace)trace_=active_search_trace;
  if(!trace_)throw std::runtime_error("SEARCH_TRACE_REQUEST_MISSING");
  events_.clear();samples_.clear();event_count_=sample_count_=0;
  recorder_=[this](const std::vector<double>& q,bool valid,const char* checker,const TraceJson& original) {
    ++sample_count_;
    if(samples_.size()<trace_->sample_limit) samples_.push_back({q,valid,valid?TraceJson(nullptr):TraceJson{{"checker",checker},{"original",original}}});
  };
  try {
    auto status=solveRecorded(ptc);extension_recorder=nullptr;snapshot(status);return status;
  } catch(...) {extension_recorder=nullptr;throw;}
}
void TraceRRTConnect::snapshot(const base::PlannerStatus& status) {
  base::PlannerData data(si_);getPlannerData(data);
  TraceJson j={{"planner_instance",trace_->next_instance++},{"planner","RRTConnect/OMPL-1.7.0"},
    {"status",status.asString()},{"sequence_semantics","ACTUAL_EXTENSION_ORDER_NOT_WALL_TIME"},
    {"nodes",TraceJson::array()},{"edges",TraceJson::array()},{"events",TraceJson::array()},
    {"checked_samples",TraceJson::array()},{"raw_solution",nullptr},
    {"simplified_path",nullptr},{"simplified_path_status","NOT_EXPOSED_AT_PLANNER_SOLVE_BOUNDARY"},
    {"planner_data_vertices",data.numVertices()},{"planner_data_edges",data.numEdges()},
    {"total_extensions",event_count_},{"total_extension_custom_checks",sample_count_},
    {"truncated",event_count_>events_.size()||sample_count_>samples_.size()},
    {"event_limit",trace_->event_limit},{"sample_limit",trace_->sample_limit}};
  for(unsigned i=0;i<data.numVertices();++i) {
    const auto& v=data.getVertex(i);
    j["nodes"].push_back({{"id",i},{"q",values(v.getState())},{"tree",v.getTag()==1?"START":"GOAL"},
      {"root",data.isStartVertex(i)||data.isGoalVertex(i)}});
    std::vector<unsigned> edges;data.getEdges(i,edges);
    for(auto k:edges)j["edges"].push_back({i,k});
  }
  if(connectionPoint_.first && connectionPoint_.second) {
    j["join"]={data.vertexIndex(base::PlannerDataVertex(connectionPoint_.first)),data.vertexIndex(base::PlannerDataVertex(connectionPoint_.second))};
  } else j["join"]=nullptr;
  for(const auto& e:events_)j["events"].push_back({e.sequence,e.start?0:1,e.accepted,e.from,e.to,e.sample_begin,e.sample_end,e.outcome});
  for(const auto& s:samples_)j["checked_samples"].push_back({s.q,s.valid,s.witness});
  if(pdef_->hasSolution()) {
    auto path=std::dynamic_pointer_cast<PathGeometric>(pdef_->getSolutionPath());
    if(path) {j["raw_solution"]=TraceJson::array();for(auto s:path->getStates())j["raw_solution"].push_back(values(s));}
    j["raw_solution_approximate"]=pdef_->hasApproximateSolution();
  }
  j["witness_semantics"]="Only original same-call Clearance/ProcessPolicy failures. Null means UNKNOWN (MoveIt validity/cache/constraint/bounds); no extra geometry query. Samples are actual custom feasibility checks; ordinary MoveIt rejections and cached checks can have no sample. Samples are not all validity calls, not tree nodes or ordered swept proof.";
  j["truncation"]={{"events_missing_from",event_count_>events_.size()?TraceJson(events_.size()):TraceJson(nullptr)},
    {"samples_missing_from",sample_count_>samples_.size()?TraceJson(samples_.size()):TraceJson(nullptr)}};
  std::lock_guard<std::mutex> lock(trace_->mutex);trace_->planners.push_back(std::move(j));
}
}

m710::TraceRRTConnect::GrowState m710::TraceRRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                             Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate)) {
            event_sample_begin_=samples_.size();
            extension(nmotion->state,tgi.xstate,tgi.start,false,"NO_PROGRESS");
            return TRAPPED;
        }

        dstate = tgi.xstate;
        reach = false;
    }

    event_sample_begin_=samples_.size();extension_recorder=&recorder_;
    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                                   si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    extension_recorder=nullptr;
    extension(nmotion->state,dstate,tgi.start,validMotion,validMotion?"ACCEPTED":"REJECTED");
    if (!validMotion)
        return TRAPPED;

    if (addIntermediateStates_)
    {
        const base::State *astate = tgi.start ? nmotion->state : dstate;
        const base::State *bstate = tgi.start ? dstate : nmotion->state;

        std::vector<base::State *> states;
        const unsigned int count = si_->getStateSpace()->validSegmentCount(astate, bstate);

        if (si_->getMotionStates(astate, bstate, states, count, true, true))
        {
            // if coming from start, don't add the start state (start->goal)
            if (tgi.start)
                si_->freeState(states[0]);
            // if coming from the goal, don't add the start state (goal->start)
            else
                si_->freeState(states[states.size() - 1]);
        }

        // Add states forwards if from start, backwards if from goal
        const auto &add_state = [&](const auto &state)
        {
            auto *motion = new Motion;
            motion->state = state;
            motion->parent = nmotion;
            motion->root = nmotion->root;
            tree->add(motion);
            nmotion = motion;
        };

        if (tgi.start)
            std::for_each(++std::begin(states), std::end(states), add_state);
        else
            std::for_each(++std::rbegin(states), std::rend(states), add_state);

        tgi.xmotion = nmotion;
    }
    else
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tree->add(motion);

        tgi.xmotion = motion;
    }

    return reach ? REACHED : ADVANCED;
}

ompl::base::PlannerStatus m710::TraceRRTConnect::solveRecorded(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool solved = false;
    base::PlannerStatus::StatusType status = base::PlannerStatus::TIMEOUT;

    while (!ptc)
    {
        TreeData &tree = startTree_ ? tStart_ : tGoal_;
        tgi.start = startTree_;
        startTree_ = !startTree_;
        TreeData &otherTree = startTree_ ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                status = base::PlannerStatus::INVALID_GOAL;
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            tgi.start = startTree_;

            /* if initial progress cannot be done from the otherTree, restore tgi.start */
            GrowState gsc = growTree(otherTree, tgi, rmotion);
            if (gsc == TRAPPED)
                tgi.start = !tgi.start;

            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);

                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
            else
            {
                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (tgi.start)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : status;
}
