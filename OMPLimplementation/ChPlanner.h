

#ifndef CH_PLANNER_H
#define CH_PLANNER_H

#include <chrono>
#include <string>

#include "chrono/core/ChMatrix.h"
#include "chrono/physics/ChSystem.h"
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include "ompl/base/goals/GoalStates.h"
#include "ChPlanModel.h"
#include "chrono/core/ChVector3.h"
namespace chrono 
{


	class ChPlanner
	{
	public: 
		ChPlanner(ChPlanModel* mdl);

		virtual ~ChPlanner();

		double getDuration() const;

		ChVectorDynamic<>* getGoal() const;

		void addGoal(const ChVectorDynamic<>& goal);

		ChPlanModel* getModel() const;

		virtual std::string getName() const = 0;

		//*
		//	* Get solution path.
		//	*
		//	* @pre solve()
		//	
		virtual std::vector<ChVectorDynamic<>> getPath() = 0;
		virtual std::vector<ChVectorDynamic<>> getPath2() = 0;
		//virtual std::vector<ChVectorDynamic<>> getRawPath() = 0;

		ChVectorDynamic<>* getStart() const;

		//Viewer* getViewer() const;

		/**
			* Reset planner.
			*/
		virtual void reset() = 0;

		void setDuration(double duration);

		void setGoal(ChVectorDynamic<>* goal);

		void setGoals(const std::vector<ChVectorDynamic<>>& goals);

		void setModel(ChPlanModel* model);

		void setStart(ChVectorDynamic<>* start);

		/*void setViewer(Viewer* viewer);*/

		/**
			* Find collision free path.
			*/
		//bool solve();

		virtual void solveCommon() = 0;
		

		/**
			* Vertify that start and goal configuration are within joint limits and collision free.
			*/
		bool verify();

		/** Upper bound for search. */
		double duration;

		/** Goal configuration. */
		ChVectorDynamic<>* goal;

		ChPlanModel* model;

		/** Start configuration. */
		ChVectorDynamic<>* start;
		
		std::vector<ChVectorDynamic<>> goals;
		
	protected:
		double time;

	private:

	};
}

#endif // CH_PLANNER_H
