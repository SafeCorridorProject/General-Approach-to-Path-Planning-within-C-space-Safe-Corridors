

#ifndef CH_PLANNER_OMPL_H
#define CH_PLANNER_OMPL_H
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/PathSimplifier.h"
#include <ompl/base/State.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/informedRRTstar.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/geometric/SimpleSetup.h>
#include "ompl/base/terminationconditions/IterationTerminationCondition.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/Planner.h"
#include "ChPlanner.h"
#include "ChPlanModel.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/informed/RejectionInfSampler.h>

using namespace ompl;
using namespace ompl::base;

namespace chrono {


    ///////////////// OMPL DECLARATIONS ///////////////////////////////
    // the declaration of the OMPL classes are put here but can be put in header
    // could have been done to reduce the dependencies on OMPL, but might be moved in headers later on

    class OMPLStateSampler : public StateSampler
    {
    public:
        OMPLStateSampler(const StateSpace* space) : ompl::base::StateSampler(space)
        {
        }

        virtual void sampleUniform(State* state);
        virtual void sampleUniformNear(State* state, const ompl::base::State* near, const double distance);
        virtual void sampleGaussian(State* state, const State* mean, const double stdDev);
    };




    class OMPLStateSpace : public /*StateSpace*/ompl::base::RealVectorStateSpace
    {
    public:

        class StateType : public State
        {
        public:
            StateType(std::size_t ndofs) : system_state(ndofs)
            {
            }

            ChVectorDynamic<> system_state;
        };

        OMPLStateSpace(ChPlanModel* model) : /*StateSpace*/ompl::base::RealVectorStateSpace(), model(model), epsilon(1e-3)
        {
            setName("OMPLStateSpace" + getName());
        }

        virtual unsigned int getDimension() const
        {
            return model->getDof();
        }

        // virtual int getType() const {
        //     return ompl::base::STATE_SPACE_REAL_VECTOR;
        //}

        virtual double getMaximumExtent() const
        {

            std::size_t ndofs = this->model->getDof();
            ChVectorDynamic<> maximum(ndofs), minimum(ndofs);
            maximum = model->getMaximum();                                     
            minimum = model->getMinimum();

            return model->distance(minimum, maximum);
        }

        virtual double getMeasure() const
        {
            std::size_t ndofs = this->model->getDof();
            ChVectorDynamic<> maximum(ndofs), minimum(ndofs);
            maximum = model->getMaximum();
            minimum = model->getMinimum();                                     
            double measure = 1.;
            for (::std::size_t i = 0; i < ndofs; ++i)
                measure*= maximum[i] - minimum[i];
            return measure;
        }

        virtual void enforceBounds(State* state) const
        {
            std::size_t ndofs = this->model->getDof();
            ChVectorDynamic<> maximum(ndofs), minimum(ndofs);
            maximum = model->getMaximum();
            minimum = model->getMinimum();
            ChVectorDynamic<>& v = state->as<StateType>()->system_state;
            for (::std::size_t i = 0; i < ndofs; ++i)
            {                                                                         
                if (v[i] < minimum[i])
                    v[i] = minimum[i];
                else if (v[i] > maximum[i])
                    v[i] = maximum[i];
            }
        }

        virtual bool satisfiesBounds(const State* state) const
        {
            std::size_t ndofs = this->model->getDof();
            ChVectorDynamic<> maximum(ndofs), minimum(ndofs);
            maximum = model->getMaximum();
            minimum = model->getMinimum();                                                       
            const ChVectorDynamic<>& v = state->as<StateType>()->system_state;
            for (std::size_t i = 0; i < ndofs; ++i)
                if (v[i] < minimum[i] || v[i] > maximum[i])
                    return false;
            return true;
        }

        virtual void copyState(State* destination, const State* source) const
        {
            destination->as<StateType>()->system_state = source->as<StateType>()->system_state;
        }





        virtual double distance(const State* state1, const State* state2) const
        {
             double dist = model->distance(                                                        
                state1->as<StateType>()->system_state,
                state2->as<StateType>()->system_state);

             return dist;  //
        }

        virtual bool equalStates(const State* state1, const State* state2) const
        {
            return distance(state1, state2) < epsilon;
        }

        virtual unsigned int getSerializationLength() const {
            return model->getDof() * sizeof(double);
        }

        virtual void interpolate(const State *from, const State *to, const double t, State *state) const
        {
                state->as<StateType>()->system_state = model->interpolate(
                from->as<StateType>()->system_state,
                to->as<StateType>()->system_state,
                t);
        }

        virtual StateSamplerPtr allocDefaultStateSampler() const                          // verify if it is upgradable
        {
            return StateSamplerPtr(new OMPLStateSampler(this));
        }

        virtual State* allocState() const
        {
            return new StateType(model->getDof());
        }

        virtual void freeState(State* state) const
        {
            delete state->as<StateType>();
        }

        virtual void registerProjections();

        void registerDefaultProjection(const ProjectionEvaluatorPtr& projection)
        {
            registerProjection(DEFAULT_PROJECTION_NAME, projection);
        }


        ChPlanModel* model;
        double epsilon;
        State* state;

    };


    ///////////////////////////////////////////////////////




    class PathLengthRejectionObjective : public ompl::base::PathLengthOptimizationObjective {
    public:
        using ompl::base::PathLengthOptimizationObjective::PathLengthOptimizationObjective;

        ompl::base::InformedSamplerPtr allocInformedStateSampler(
            const ompl::base::ProblemDefinitionPtr& probDefn,
            unsigned int maxNumberCalls) const override
        {
            return std::make_shared<ompl::base::RejectionInfSampler>(probDefn, maxNumberCalls);
        }
    };



    /////////////////////////////////////////////////////////////




    class OMPLStateValidityChecker : public ompl::base::StateValidityChecker
    {
    public:
        OMPLStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
            ChPlanModel* model)
            : ompl::base::StateValidityChecker(si), model(model)
        {
        }

        virtual ~OMPLStateValidityChecker()
        {
        }

        virtual bool isValid(const State* state) const
        {
            // ORIGINALLY:
            model->setPosition(state->as<OMPLStateSpace::StateType>()->system_state);
 
            //return !model->isColliding();
            model->isColliding(state->as<OMPLStateSpace::StateType>()->system_state);
            return !model->isColliding();
        }

        ChPlanModel* model;
        
    };





    class OMPLProjectionEvaluator : public ompl::base::ProjectionEvaluator
    {
    public:
        OMPLProjectionEvaluator(const StateSpace *space, unsigned int ndim)
            : ProjectionEvaluator(space), ndim(ndim)
        {
        }

        OMPLProjectionEvaluator(const StateSpacePtr &space, unsigned int ndim)
            : ProjectionEvaluator(space), ndim(ndim)
        {
        }

        virtual ~OMPLProjectionEvaluator()
        {
        }

        virtual unsigned int getDimension() const override
        {
            return ndim;
        }

        virtual void project(const State* state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            // TODO: compute a better projection based on, e.g., end effector position
            // instead of first couple joint angles.
            for (unsigned int i = 0; i < ndim; ++i)
                projection(i) = state->as<OMPLStateSpace::StateType>()->system_state[i];
        }
        ///////////////////////////////
        
        //virtual void project(const State* state, Eigen::Ref<Eigen::VectorXd> projection) const override
        //{
        //    // Compute the end effector position based on the system state
        //    ChVectorDynamic<> endEffectorPos = model->computeEndEffectorPosition(state->as<OMPLStateSpace::StateType>()->system_state);

        //    // Project the end effector position into the projection space
        //    for (unsigned int i = 0; i < ndim; ++i)
        //        projection(i) = endEffectorPos(i);
        //}


        





        /////////////////////////////////

        virtual void setCellSizes(const std::vector<double>& cellSizes) {

            cellSizes_ = cellSizes;
        }

        void computeCoordinates(const State* state, Eigen::Ref<Eigen::VectorXi> coord) const
        {
            Eigen::VectorXd projection(getDimension());
            project(state, projection);
            computeCoordinates(projection, coord);
        }


        void computeCoordinates(const Eigen::Ref<Eigen::VectorXd>& projection,
            Eigen::Ref<Eigen::VectorXi> coord) const;

        //////////////////////////////////////

        virtual void defaultCellSizes() override
        {
            std::size_t ndofs = space_->as<OMPLStateSpace>()->model->getDof();
            ChVectorDynamic<> maximum(ndofs), minimum(ndofs);
            maximum = space_->as<OMPLStateSpace>()->model->getMaximum();                          
            minimum = space_->as<OMPLStateSpace>()->model->getMinimum();
            bounds_.resize(ndim);
            cellSizes_.resize(ndim);
            for (unsigned int i = 0; i < cellSizes_.size(); ++i)
            {
                bounds_.low[i] = minimum[i];
                bounds_.high[i] = maximum[i];
                cellSizes_[i] = (bounds_.high[i] - bounds_.low[i]) / ::ompl::magic::PROJECTION_DIMENSION_SPLITS;
            }
        }
        ChPlanModel* model;
    protected:
        unsigned int ndim;
    };



    



    class ChPlannerOMPL : public ChPlanner
    {
    public:
        enum PlannerType {
            AUTOCONFIG, // let OMPL choose one
            PRM, PRMSTAR, LAZYPRM, LAZYPRMSTAR, SPARS, SPARS2, // roadmap-based planners
            RRT, RRTCONNECT, RRTSTAR, TRRT, LBTRRT, LAZYRRT, InformedRRTstar, // RRT variants
            EST, SBL, STRIDE, // EST-like planners
            KPIECE, BKPIECE, LBKPIECE, // KPIECE variants
            PDST, CFOREST, FMT, ANYTIMEPATHSHORTENING,  // other planners
        };
        ChPlannerOMPL(ChPlanModel* mdl, PlannerType plannerType /*= AUTOCONFIG*/);
        virtual ~ChPlannerOMPL();
        virtual ::std::string getName() const override;
        virtual std::vector<ChVectorDynamic<>> getPath() override;
        virtual std::vector<ChVectorDynamic<>> getPath2() override;
        //virtual void getPath(ChVectorDynamic<>& path) override;
        //virtual std::vector<ChVectorDynamic<>> getRawPath() override;
        virtual void reset() override;
        //virtual bool solve() override;
        bool solve(double duration);
        bool solve(const ompl::base::PlannerTerminationCondition& ptc);
        double getLastPlanComputationTime();
        void addMultipleGoals();
        unsigned int ChPlannerOMPL::getIterationCount() const;
        ompl::base::PlannerTerminationCondition firstSolutionPTC() const;
     protected:
        PlannerType plannerType;
        ompl::base::StateSpacePtr space;
        ompl::geometric::SimpleSetup setup;
        void solveCommon();
        
    };

}
#endif  //CH_PLANNER_OMPL_H
