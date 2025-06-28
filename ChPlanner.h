

#ifndef CH_PLANNER_H
#define CH_PLANNER_H

#include <chrono>
#include <string>
#include "chrono/core/ChMatrix.h"
#include "chrono/physics/ChSystem.h"
#include "ChPlanModel.h"
#include "chrono/core/ChVector3.h"
namespace chrono 
{


	class ChPlanner
	{
	public: 
		ChPlanner(ChPlanModel* mdl);

		 ~ChPlanner();

		double getDuration() const;

		ChVectorDynamic<>* getGoal() const;

		void addGoal(const ChVectorDynamic<>& goal);

		ChPlanModel* getModel() const;

		 std::string getName() const;


		 std::vector<ChVectorDynamic<>> getPath(std::vector<ChVectorDynamic<>>& path);

		//void ChPlanner::setGoalNode(Node* node);
		//virtual std::vector<ChVectorDynamic<>> getRawPath() = 0;

		ChVectorDynamic<>* getStart() const;


		void reset();

		void setDuration(double duration);

		void setGoal(ChVectorDynamic<>* goal);

		void setGoals(const std::vector<ChVectorDynamic<>>& goals);

		void setModel(ChPlanModel* model);

		void setStart(ChVectorDynamic<>* start);


		void solveCommon();
		
		bool verify();

	
		double duration;

		/** Goal configuration. */
		ChVectorDynamic<>* goal;
		
		ChPlanModel* model;
		ChPlanModel* planner;                   
		std::vector<ChVectorDynamic<>> path;
		/** Start configuration. */
		ChVectorDynamic<>* start;
		
		std::vector<ChVectorDynamic<>> goals;
		
	protected:
		double time;

	private:

	};
}

#endif // CH_PLANNER_H
