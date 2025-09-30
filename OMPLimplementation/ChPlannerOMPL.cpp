

//#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ChPlannerOMPL.h"
#include "ChPlanModel.h"
#include "ChPlanner.h"
#include "ChJoint.h"
#include "ompl/base/goals/GoalStates.h"
#include <ompl/base/samplers/informed/RejectionInfSampler.h>

namespace chrono
{



    ///////////////// OMPL DEFINITIONS ///////////////////////////////
    // the definition of the OMPL classes are spread both in class header and here

    void OMPLStateSpace::registerProjections()
    {
        std::size_t ndofs = model->getDof();
        if (ndofs > 0)
        {
            if (ndofs > 2)
            {
                int p = std::max(2, (int)ceil(log((double)ndofs)));
                registerDefaultProjection(ProjectionEvaluatorPtr(new OMPLProjectionEvaluator(this, p)));
            }
            else
                registerDefaultProjection(ProjectionEvaluatorPtr(new OMPLProjectionEvaluator(this, ndofs)));
        }
    }

    void OMPLStateSampler::sampleUniform(State* state)
    {
        const ChPlanModel* model = space_->as<OMPLStateSpace>()->model;
        std::size_t ndofs = model->getDof();
        ChVectorDynamic<> maximum(ndofs), minimum(ndofs);
        maximum = model->getMaximum();
        minimum = model->getMinimum();

        ChVectorDynamic<>& v = state->as<OMPLStateSpace::StateType>()->system_state;
        for (std::size_t i = 0; i < ndofs; ++i)
            v[i] = rng_.uniformReal(minimum[i], maximum[i]);
    }

    void OMPLStateSampler::sampleUniformNear(State* state, const State* near, const double distance)
    {
        const ChPlanModel* model = space_->as<OMPLStateSpace>()->model;
        std::size_t ndofs = model->getDof();
        ChVectorDynamic<> maximum(ndofs), minimum(ndofs);
        maximum = model->getMaximum();
        minimum = model->getMinimum();

        ChVectorDynamic<>& v = state->as<OMPLStateSpace::StateType>()->system_state;
        const ChVectorDynamic<>& vNear = near->as<OMPLStateSpace::StateType>()->system_state;
        for (::std::size_t i = 0; i < ndofs; ++i)
            v[i] = rng_.uniformReal(std::max(minimum[i], vNear[i] - distance),
                std::min(maximum[i], vNear[i] + distance));
    }

    void OMPLStateSampler::sampleGaussian(State* state, const State* mean, const double stdDev)
    {
        const ChPlanModel* model = space_->as<OMPLStateSpace>()->model;
        std::size_t ndofs = model->getDof();
        ChVectorDynamic<> maximum(ndofs), minimum(ndofs);
        maximum = model->getMaximum();
        minimum = model->getMinimum();

        ChVectorDynamic<>& v = state->as<OMPLStateSpace::StateType>()->system_state;
        const ChVectorDynamic<>& vMean = mean->as<OMPLStateSpace::StateType>()->system_state;
        for (std::size_t i = 0; i < ndofs; ++i)
        {
            double r = rng_.gaussian(vMean[i], stdDev);
            if (r < minimum[i])
                r = minimum[i];
            else if (r > maximum[i])
                r = maximum[i];
            v[i] = r;
        }
    }

    ///////////////////////////// ChPlannerOMPL ///////////////////////////


    ChPlannerOMPL::ChPlannerOMPL(ChPlanModel* mdl, PlannerType plannerType) :
        ChPlanner(mdl),
        plannerType(plannerType),
        space(ompl::base::StateSpacePtr(new chrono::OMPLStateSpace(this->model))),            
        setup(space)
    {
        setup.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
            new chrono::OMPLStateValidityChecker(
                setup.getSpaceInformation(), model)));

    }

    ChPlannerOMPL::~ChPlannerOMPL()
    {
    }

    std::string ChPlannerOMPL::getName() const
    {
        return setup.getPlanner()->getName();
    }

    std::vector<ChVectorDynamic<>> ChPlannerOMPL::getPath()
    {
        std::vector<ChVectorDynamic<>> path;
        if (setup.haveSolutionPath())
        {
            //ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(2.0);
            //setup.simplifySolution();
            ompl::geometric::PathGeometric& p(setup.getSolutionPath());
            ompl::geometric::PathSimplifier simplifier(setup.getSpaceInformation());
            simplifier.ropeShortcutPath(p);
            simplifier.partialShortcutPath(p, 500, 0);
            simplifier.smoothBSpline(p, 3);
            path.clear();
            for (unsigned int i = 0; i < p.getStateCount(); ++i)
                path.push_back(
                    static_cast<chrono::OMPLStateSpace::StateType*>(p.getState(i))->system_state);
        }
        return path;
    }


    std::vector<ChVectorDynamic<>> ChPlannerOMPL::getPath2()
    {
        std::vector<ChVectorDynamic<>> path;
        if (setup.haveSolutionPath())
        {
            //ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(2.0);
            setup.simplifySolution();
            //ompl::geometric::PathSimplifier::smoothBSpline();
            ompl::geometric::PathGeometric& p(setup.getSolutionPath());
            ompl::geometric::PathSimplifier simplifier(setup.getSpaceInformation());
            simplifier.partialShortcutPath(p, 200, 0);
            simplifier.smoothBSpline(p, 5);
            path.clear();
            for (unsigned int i = 0; i < p.getStateCount(); ++i)
                path.push_back(
                    static_cast<chrono::OMPLStateSpace::StateType*>(p.getState(i))->system_state);
        }
        return path;
    }


    //std::vector<ChVectorDynamic<>> ChPlannerOMPL::getRawPath()
    //{
    //    std::vector<ChVectorDynamic<>> path1;
    //    if (setup.haveSolutionPath())
    //    {
    //        ompl::geometric::PathGeometric& p(setup.getSolutionPath());
    //        path1.clear();
    //        for (unsigned int i = 0; i < p.getStateCount(); ++i)
    //            path1.push_back(
    //                static_cast<chrono::OMPLStateSpace::StateType*>(p.getState(i))->system_state);
    //    }

    //    return path1;
    //}


    PlannerTerminationCondition chrono::ChPlannerOMPL::firstSolutionPTC() const {
        auto exact = exactSolnPlannerTerminationCondition(setup.getProblemDefinition());
        
            
        return exact;
        
    }

    
    void ChPlannerOMPL::addMultipleGoals() {
        auto goalStates = std::make_shared<ompl::base::GoalStates>(setup.getSpaceInformation());

        for (const auto& goalVec : this->goals) {
            auto state = space->allocState();
            for (size_t i = 0; i < goalVec.size(); ++i) {
                state->as<OMPLStateSpace::StateType>()->system_state[i] = goalVec[i];
            }

            goalStates->addState(state);
        }

        setup.setGoal(goalStates);
    }

    void ChPlannerOMPL::reset()
    {
        setup.clear();
    }


    bool ChPlannerOMPL::solve(double duration)
    {
        
        solveCommon();

        return setup.solve(this->duration);

    }
    bool ChPlannerOMPL::solve(const ompl::base::PlannerTerminationCondition& ptc) {
      
        solveCommon();
        return setup.solve(ptc);

    }
    double ChPlannerOMPL::getLastPlanComputationTime() {
       return setup.getLastPlanComputationTime();
        
    }



    unsigned int ChPlannerOMPL::getIterationCount() const {
        ompl::base::PlannerData data(setup.getSpaceInformation());
        setup.getPlannerData(data);      
        return data.numVertices();       
    }




    void ChPlannerOMPL::solveCommon()
    {
        ompl::base::ScopedState<chrono::OMPLStateSpace> start(space), goal(space);
        start->system_state = *this->start;
        goal->system_state = *this->goal;
        setup.setStartAndGoalStates(start, goal);
        //setup.addStartState(start);

        //addMultipleGoals();
        if (!setup.getPlanner())
        {
            const ompl::base::SpaceInformationPtr &si = setup.getSpaceInformation();
            const ompl::base::ProblemDefinitionPtr &pdef = setup.getProblemDefinition();
            //auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
            //setup.setOptimizationObjective(objective);

            auto objective = std::make_shared<PathLengthRejectionObjective>(si);
            setup.setOptimizationObjective(objective);

            //auto exactSolution = exactSolnPlannerTerminationCondition(setup.getProblemDefinition());

            switch (plannerType)
            {
            case PRM:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::PRM(si)));
                setup.getPlanner()->as<ompl::geometric::PRM>()->setMaxNearestNeighbors(50);
                break;
            case PRMSTAR:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::PRMstar(si)));
                setup.getPlanner()->as<ompl::geometric::PRMstar>()->setMaxNearestNeighbors(20);
                break;
            case LAZYPRM:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LazyPRM(si)));
                break;
            case LAZYPRMSTAR:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LazyPRMstar(si)));
                break;
            case SPARS:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::SPARS(si)));
                break;
            case SPARS2:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::SPARStwo(si)));
                break;
            case RRT:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRT(si)));
                setup.getPlanner()->as<ompl::geometric::RRT>()->setGoalBias(0.1);
                setup.getPlanner()->as < ompl::geometric::RRT>()->getGoalBias();
                std::cout << "RRT Goal Bias: " << setup.getPlanner()->as<ompl::geometric::RRT>()->getGoalBias() << std::endl;
                //setup.getPlanner()->as<ompl::geometric::RRT>()->setIntermediateStates(true);
                setup.getPlanner()->as<ompl::geometric::RRT>()->setRange((0.3 * space->getMaximumExtent()));
                //setup.getPlanner()->as<ompl::geometric::RRT>()->printSettings(std::cout);
                //setup.getPlanner()->as<ompl::geometric::RRT>()->printProperties(std::cout);

                break;
            case RRTCONNECT:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si)));
                setup.getPlanner()->as<ompl::geometric::RRTConnect>()->setIntermediateStates(true);
                setup.getPlanner()->as<ompl::geometric::RRTConnect>()->setRange(0.1);
                //setup.getPlanner()->as<ompl::geometric::RRTConnect>()->setIntermediateStates(true);

                break;
            case RRTSTAR:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si)));
                setup.getPlanner()->as<ompl::geometric::RRTstar>()->setGoalBias(0.1);
                setup.getPlanner()->as<ompl::geometric::RRTstar>()->setRange((0.3 * space->getMaximumExtent()));
                setup.getPlanner()->as<ompl::geometric::RRTstar>()->getSpecs();

                break;
            case TRRT:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::TRRT(si)));
                break;
            case LBTRRT:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LBTRRT(si)));
                break;
            case LAZYRRT:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LazyRRT(si)));
                break;
            case EST:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::EST(si)));
                break;
            case SBL:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::SBL(si)));
                break;
            case STRIDE:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::STRIDE(si)));
                break;
            case KPIECE:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si)));
                setup.getPlanner()->as<ompl::geometric::KPIECE1>()->setGoalBias(0.1);
                break;
            case BKPIECE:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::BKPIECE1(si)));
                break;
            case LBKPIECE:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::LBKPIECE1(si)));
                break;
            case PDST:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::PDST(si)));
                break;
            case CFOREST:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::CForest(si)));
                break;
            case FMT:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::FMT(si)));
                break;
            case InformedRRTstar:
				setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::InformedRRTstar(si)));
                setup.getPlanner()->as<ompl::geometric::InformedRRTstar>()->setGoalBias(0.1);
                //setup.getPlanner()->as<ompl::geometric::InformedRRTstar>()->setRewireFactor(1.1);
                setup.getPlanner()->as<ompl::geometric::InformedRRTstar>()->setRange((0.3 * space->getMaximumExtent()));
				break;
            case ANYTIMEPATHSHORTENING:
                setup.setPlanner(ompl::base::PlannerPtr(new ompl::geometric::AnytimePathShortening(si)));
                break;
            default:
                ; // nothing: autoconfigure
            }

        }

    }
}







       

// ompl::base::ParamSet& OMPLPlanner::params()
// {
//     return setup.getPlanner();
// }
