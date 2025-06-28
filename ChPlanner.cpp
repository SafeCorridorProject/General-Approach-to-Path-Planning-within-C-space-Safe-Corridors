

#include "ChPlanner.h"                                                    // (chrono) ChPlanner class is designed to represent a generic planner for some kind of motion planning. It encapsulates start and goal configurations, a model for the robot or system, and a viewer for displaying information.
//#include "ChPlannerOMPL.h"

namespace chrono {
	ChPlanner::ChPlanner(ChPlanModel* mdl) :
		planner(mdl),                              
		model(mdl),
		duration(0.0),
		goal(nullptr),
		//model(nullptr),
		start(nullptr),
		time()
	{}

	ChPlanner::~ChPlanner()
	{}

	double ChPlanner::getDuration() const
	{
		return this->duration;
	}

	ChVectorDynamic<>* ChPlanner::getGoal() const
	{
		return this->goal;
	}

	void ChPlanner::addGoal(const ChVectorDynamic<>& goal) {
		this->goals.push_back(goal);
	}

	ChPlanModel* ChPlanner::getModel() const
		{
			return this->model;
		}
		
	ChVectorDynamic<>* ChPlanner::getStart() const
		{
			return this->start;
		}
	


	
	std::vector<ChVectorDynamic<>> getPath(std::vector<ChVectorDynamic<>>& path) {
		return path;
	}



	void ChPlanner::setDuration(double duration)
	{
		this->duration = duration;
	}
		
	void ChPlanner::setGoal(ChVectorDynamic<>* goal)
	{
		this->goal = goal;
	}

	void ChPlanner::setGoals(const std::vector<ChVectorDynamic<>>& goals) {
		this->goals = goals;
	}
		
	 void ChPlanner::setModel(ChPlanModel* model)
	{
		this->model = model;
	}
		
	void ChPlanner::setStart(ChVectorDynamic<>* start)
	{
		this->start = start;
	}


	//bool ChPlanner::solveWithInterpolation(std::vector<ChVectorDynamic<>> goals) {
	//	for (size_t i = 0; i < goals.size() - 1; ++i) {
	//		// numbers of interpolations
	//		int numInterpolationSteps = 10; 

	//		// interpolation step
	//		ChVectorDynamic<> step = (goals[i + 1] - goals[i]) / numInterpolationSteps;

	//		// interpolation
	//		for (int j = 0; j < numInterpolationSteps; ++j) {
	//			ChVectorDynamic<> interpolatedGoal = goals[i] + step * j;
	//			setGoal(&interpolatedGoal);
	//			if (!solve()) {
	//				std::cerr << "Failed to reach interpolated goal." << std::endl;
	//				return false;
	//			}
	//		}
	//	}
	//	// final goal
	//	setGoal(&goals.back());
	//	if (!solve()) {
	//		std::cerr << "Failed to reach final goal." << std::endl;
	//		return false;
	//	}

	//	return true;
	//}





		
	bool ChPlanner::verify()                                                                 //chrono  (verify function checks the validity of the start and goal configurations and whether they lead to collisions)
	{
		
		if (!this->model->isValid(*this->start))
		{
			std::cerr << "Invalid start configuration." << std::endl;
				
			return false;
		}
			
		if (!this->model->isValid(*this->goal))
		{
			std::cerr << "Invalid goal configuration." << std::endl;

			return false;
				
		}
			
		if (this->model->isColliding(*this->start))
		{
            std::cerr << "Colliding start configuration in body " << this->model->getCollidingBodyA()->GetName()<< " and "
				<< this->model->getCollidingBodyB()->GetName()<< "." << std::endl;
			
				
			return false;
		}
			
		if (this->model->isColliding(*this->goal))
		{
		    std::cerr << "Colliding goal configuration in body " << this->model->getCollidingBodyA()->GetName() << " and"
				<< this->model->getCollidingBodyB()->GetName()<< "." << std::endl;
			
				
			return false;
		}
			
		return true;
	}
}

