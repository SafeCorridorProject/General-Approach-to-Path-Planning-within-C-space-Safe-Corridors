#include "ChJoint.h"
#include "ChPlanModel.h"
#include "ChPlanner.h"  
#include "ChPlannerOMPL.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorAll.h"
#include "chrono/physics/ChLinkMotorLinear.h"
//#include "chrono/physics/ChLinkMotorLinearPosition.h"
//#include "chrono/physics/ChLinkMotorLinearSpeed.h"
//#include "chrono/physics/ChLinkMotorLinearForce.h"
//#include "chrono/physics/ChLinkMotorLinearDriveline.h"
//#include "chrono/physics/ChLinkMotorRotationAngle.h"
//#include "chrono/physics/ChLinkMotorRotationSpeed.h"
//#include "chrono/physics/ChLinkMotorRotationTorque.h"
//#include "chrono/physics/ChLinkMotorRotationDriveline.h"
//#include "chrono/physics/ChShaftsMotorSpeed.h"
//#include "chrono/physics/ChShaftsMotorPosition.h"
//#include "chrono/physics/ChShaftsPlanetary.h"
//#include "chrono/physics/ChShaftsGear.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/core/ChVector3.h"
#include <cmath>
#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include <fstream>


#include "CAD_export/comau_racer_export_2.h"
#include "ChRobot_6dof_CAD.h"
#include "ChRobot_6dof_CAD_free.h"


// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;




constexpr bool write_mode = true;

ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;


class CartesianRobot {
public:
    CartesianRobot();
    void initialize(ChSystemNSC& sys);
    void createGround(ChSystemNSC& sys);
    void createBodies(ChSystemNSC& sys);
    void createObstacles(ChSystemNSC& sys);
    void createMotors(ChSystemNSC& sys);
    void createJoints(ChSystemNSC& sys);

    std::shared_ptr<ChBody> groundBody;
    std::shared_ptr<ChBody> bodyX;
    std::shared_ptr<ChBody> bodyY;
    std::shared_ptr<ChBody> bodyZ;
    std::shared_ptr<ChBody> obstacle;
    std::shared_ptr<ChBody> obstacle2;
    std::shared_ptr<ChBody> obstacle3;
    std::shared_ptr<chrono::ChLinkMotorLinearPosition> motorX;
    std::shared_ptr<chrono::ChLinkMotorLinearPosition> motorY;
    std::shared_ptr<chrono::ChLinkMotorLinearPosition> motorZ;
    std::shared_ptr<chrono::ChContactMaterialNSC> material;
    std::shared_ptr<chrono::ChJointPrismatic> jointX;
    std::shared_ptr<chrono::ChJointPrismatic> jointY;
    std::shared_ptr<chrono::ChJointPrismatic> jointZ;

    double length = 3.0;
    double width = 0.3;
    double height0 = 3.0;
};

CartesianRobot::CartesianRobot() : material(std::make_shared<chrono::ChContactMaterialNSC>()) {
}
void CartesianRobot::initialize(ChSystemNSC& sys) {
    createGround(sys);
    createBodies(sys);
    createObstacles(sys);
    createMotors(sys);
    createJoints(sys);
}

void CartesianRobot::createGround(ChSystemNSC& sys) {
    groundBody = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 1, 3000, material);
    groundBody->SetPos(ChVector3d(0.0, 0.0, -0.5));
    groundBody->SetFixed(true);
    groundBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(groundBody);
 }

void CartesianRobot::createBodies(ChSystemNSC& sys) {
    //BodyX
    bodyX = chrono_types::make_shared<chrono::ChBodyEasyBox>(width, width, width, 1000, material);
    bodyX->SetPos(ChVector3d(-2.0, 3.0, height0));
    bodyX->SetName("bodyX");
    bodyX->GetVisualShape(0)->SetColor(chrono::ChColor(1.f, 0.0f, 0.0f));
    sys.Add(bodyX);
    //BodyY
    bodyY = chrono_types::make_shared<chrono::ChBodyEasyBox>(/*length*/0.3, /*width*/0.3, /*width*/0.3, 1000, material);
    bodyY->SetPos(ChVector3d(-2.0, 3.0, height0 - width * 1.4));
    bodyY->SetName("bodyY");
    bodyY->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 1.0f, 0.0f));
    sys.Add(bodyY);
    //BodyZ
    bodyZ = chrono_types::make_shared<chrono::ChBodyEasyBox>(width, width, width, 1000, material);
    bodyZ->SetPos(ChVector3d(-2.0, 3.0, height0 - 2 * width * 1.4));
    bodyZ->SetName("bodyZ");
    bodyZ->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    sys.Add(bodyZ);
}

void CartesianRobot::createObstacles(ChSystemNSC& sys) {
    obstacle = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 6.0, 1.0, 1000, material);
    obstacle->SetPos(ChVector3d(0.0, 2.0, 1.6));
    obstacle->SetFixed(true);
    obstacle->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    sys.Add(obstacle);

    obstacle2 = chrono_types::make_shared<chrono::ChBodyEasyBox>(4.2, 0.5, 3.0, 1000, material);
    obstacle2->SetPos(ChVector3d(2.5, 5.0, 1.6));
    obstacle2->SetFixed(true);
    obstacle2->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    sys.Add(obstacle2);

    obstacle3 = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 6.0, 3.0, 1000, material);
    obstacle3->SetPos(ChVector3d(5.0, 2.0, 1.6));
    obstacle3->SetFixed(true);
    obstacle3->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    sys.Add(obstacle3);
}

void CartesianRobot::createMotors(ChSystemNSC& sys) {
    motorX = chrono_types::make_shared<chrono::ChLinkMotorLinearPosition>();
    motorY = chrono_types::make_shared<chrono::ChLinkMotorLinearPosition>();
    motorZ = chrono_types::make_shared<chrono::ChLinkMotorLinearPosition>();

    motorX->Initialize(bodyX,
                       groundBody,
                       ChFrame<>(bodyX->GetPos(), Q_ROTATE_Z_TO_X));
    motorY->Initialize(bodyY,
                       bodyX,
                       ChFrame<>(bodyY->GetPos(), Q_ROTATE_Z_TO_Y));
    motorZ->Initialize(bodyZ,
                       bodyY,
                       ChFrame<>(bodyZ->GetPos()));

    sys.Add(motorX);
    sys.Add(motorY);
    sys.Add(motorZ);
}

void CartesianRobot::createJoints(ChSystemNSC& sys) {
    jointX = std::make_shared<ChJointPrismatic>(-5, 5, motorX);
    jointY = std::make_shared<ChJointPrismatic>(-5, 5, motorY);
    jointZ = std::make_shared<ChJointPrismatic>(-1, 4, motorZ);
}


class scaraRobot {
public:
    scaraRobot();
    void initialize(ChSystemNSC& sys);
    void createGround(ChSystemNSC& sys);
    void createBodies(ChSystemNSC& sys);
    void createObstacles(ChSystemNSC& sys);
    void createMotors(ChSystemNSC& sys);
    void createJoints(ChSystemNSC& sys);

    std::shared_ptr<ChBody> groundBody;
    std::shared_ptr<ChBody> body1;
    std::shared_ptr<ChBody> body2;
    std::shared_ptr<ChBody> bodyZ;

    std::shared_ptr<ChBody> obstacle;
    std::shared_ptr<ChBody> obstacle2;
    std::shared_ptr<ChBody> obstacle3;
    std::shared_ptr<ChBody> obstacle4;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> motor1;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> motor2;
    std::shared_ptr<chrono::ChLinkMotorLinearPosition> motorZ;
    std::shared_ptr<chrono::ChContactMaterialNSC> material;
    std::shared_ptr<chrono::ChJointRevolute> joint1;
    std::shared_ptr<chrono::ChJointRevolute> joint2;
    std::shared_ptr<chrono::ChJointPrismatic> jointZ;
    double length = 3.0;
    double width = 0.3;
    double height0 = 3.0;
};

scaraRobot::scaraRobot() : material(std::make_shared<chrono::ChContactMaterialNSC>()) {
}
void scaraRobot::initialize(ChSystemNSC& sys) {
    createGround(sys);
    createBodies(sys);
    createObstacles(sys);
    createMotors(sys);
    createJoints(sys);
}

void scaraRobot::createGround(ChSystemNSC& sys) {
	groundBody = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 1, 3000, material);
	groundBody->SetPos(ChVector3d(0.0, 0.0, -0.5));
	groundBody->SetFixed(true);
	groundBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    groundBody->GetVisualShape(0)->SetOpacity(0.3);
	sys.Add(groundBody);
}

void scaraRobot::createBodies(ChSystemNSC& sys) {
    //Body1
    body1 = chrono_types::make_shared<ChBodyEasyBox>(0.5, 2.0, 0.5, 1000, material);
    body1->SetPos(ChVector3d(0.0, 0.0, height0));
    body1->SetName("bodyX");
    body1->GetVisualShape(0)->SetColor(chrono::ChColor(1.f, 0.0f, 0.0f));
    sys.Add(body1);

    body2 = chrono_types::make_shared<ChBodyEasyBox>(0.5, 2.0, 0.5, 1000, material);
    body2->SetPos(ChVector3d(0.0, -1.5, height0 + 0.8));
    body2->SetName("bodyX");
    body2->GetVisualShape(0)->SetColor(chrono::ChColor(1.f, 0.0f, 0.0f));
    sys.Add(body2);

    bodyZ = chrono_types::make_shared<ChBodyEasyBox>(0.3, 0.3, 0.3, 1000, material);
    bodyZ->SetPos(ChVector3d(0.0, -3.0, height0 + 0.8));
    bodyZ->SetName("bodyX");
    bodyZ->GetVisualShape(0)->SetColor(chrono::ChColor(1.f, 0.0f, 0.0f));
    sys.Add(bodyZ);

}

void scaraRobot::createObstacles(ChSystemNSC& sys) {
    obstacle = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 2.5, 0.5, 1000, material);
    //obstacle = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.8, 2.5, 1.0, 1000, material);
    obstacle->SetPos(ChVector3d(1.0, 1.0, 1.6));
    obstacle->SetFixed(true);
    obstacle->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle->GetVisualShape(0)->SetOpacity(0.1);
    sys.Add(obstacle);

    obstacle2 = chrono_types::make_shared<chrono::ChBodyEasyBox>(1.2, 0.5, 1.0, 1000, material);
    obstacle2->SetPos(ChVector3d(2.5, 2.0, 1.6));
    obstacle2->SetFixed(true);
    obstacle2->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle2->GetVisualShape(0)->SetOpacity(0.1);
    sys.Add(obstacle2);

    obstacle3 = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 2.5, 1.0, 1000, material);
    obstacle3->SetPos(ChVector3d(3.5, 1.0, 1.6));
    obstacle3->SetFixed(true);
    obstacle3->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle3->GetVisualShape(0)->SetOpacity(0.1);
    sys.Add(obstacle3);

    obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(1.5, 0.5, 1.0, 1000, material);    //(x = 1.2)
    //obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(3, 0.5, 3.0, 1000, material);
    //obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(2.5, 0.5, 3.0, 1000, material);
    obstacle4->SetPos(ChVector3d(2.5, -0.8, 1.6));
    obstacle4->SetFixed(true);
    obstacle4->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle4->GetVisualShape(0)->SetOpacity(0.1);
    sys.Add(obstacle4);
    
}

void scaraRobot::createMotors(ChSystemNSC& sys) {
	motor1 = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
	motor2 = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
	motorZ = chrono_types::make_shared<chrono::ChLinkMotorLinearPosition>();

	motor1->Initialize(body1,
        			   groundBody,
        			   ChFrame<>(ChVector3d(0.0, 1.0, height0)/*, Q_ROTATE_Z_TO_X)*/));
	motor2->Initialize(body2,
        			   body1,
        			   ChFrame<>(ChVector3d(0.0, -1.0, height0 + 0.8)/*, Q_ROTATE_Z_TO_Y*/));
	motorZ->Initialize(bodyZ,
        			   body2,
        			   ChFrame<>(bodyZ->GetPos()));

	sys.Add(motor1);
	sys.Add(motor2);
	sys.Add(motorZ);
}

void scaraRobot::createJoints(ChSystemNSC& sys) {
	joint1 = std::make_shared<ChJointRevolute>(-10, 10, true, motor1);
	joint2 = std::make_shared<ChJointRevolute>(-10, 10, true, motor2);
    jointZ = std::make_shared<ChJointPrismatic>(-5, 4, motorZ);
}


class scaraRobot2 {
public:
    scaraRobot2();
	void initialize(ChSystemNSC& sys);
	void createGround(ChSystemNSC& sys);
	void createBodies(ChSystemNSC& sys);
	void createObstacles(ChSystemNSC& sys);
	void createMotors(ChSystemNSC& sys);
	//void createJoints(ChSystemNSC& sys);


	std::shared_ptr<ChBody> groundBody;
	std::shared_ptr<ChBody> body1;
	std::shared_ptr<ChBody> body2;
	std::shared_ptr<ChBody> bodyZ;
	std::shared_ptr<ChBody> obstacle;
	std::shared_ptr<ChBody> obstacle2;
	std::shared_ptr<ChBody> obstacle3;
	std::shared_ptr<ChBody> obstacle4;
	std::shared_ptr<chrono::ChLinkMateRevolute> motor1;
	std::shared_ptr<chrono::ChLinkMateRevolute> motor2;
	std::shared_ptr<chrono::ChLinkMatePrismatic> motorZ;
	std::shared_ptr<chrono::ChContactMaterialNSC> material;

	double length = 3.0;
	double width = 0.3;
	double height0 = 3.0;

};

scaraRobot2::scaraRobot2() : material(std::make_shared<chrono::ChContactMaterialNSC>()) {
}
void scaraRobot2::initialize(ChSystemNSC& sys) {
    createGround(sys);
    createBodies(sys);
    createObstacles(sys);
    createMotors(sys);
}

void scaraRobot2::createGround(ChSystemNSC& sys) {
    groundBody = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 1, 3000, material);
    groundBody->SetPos(ChVector3d(0.0, 0.0, -0.5));
    groundBody->SetFixed(true);
    groundBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(groundBody);
}

void scaraRobot2::createBodies(ChSystemNSC& sys) {
    //Body1
    body1 = chrono_types::make_shared<ChBodyEasyBox>(0.5, 2.0, 0.5, 1000, material);
    body1->SetPos(ChVector3d(0.0, 0.0, height0));
    body1->SetName("bodyX");
    body1->GetVisualShape(0)->SetColor(chrono::ChColor(1.f, 0.0f, 0.0f));
    sys.Add(body1);

    body2 = chrono_types::make_shared<ChBodyEasyBox>(0.5, 2.0, 0.5, 1000, material);
    body2->SetPos(ChVector3d(0.0, -1.5, height0 + 0.8));
    body2->SetName("bodyX");
    body2->GetVisualShape(0)->SetColor(chrono::ChColor(1.f, 0.0f, 0.0f));
    sys.Add(body2);

    bodyZ = chrono_types::make_shared<ChBodyEasyBox>(0.3, 0.3, 0.3, 1000, material);
    bodyZ->SetPos(ChVector3d(0.0, -3.0, height0 + 0.8));
    bodyZ->SetName("bodyX");
    bodyZ->GetVisualShape(0)->SetColor(chrono::ChColor(1.f, 0.0f, 0.0f));
    sys.Add(bodyZ);
}

void scaraRobot2::createObstacles(ChSystemNSC& sys) {
    obstacle = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 2.5, 0.5, 1000, material);
    //obstacle = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.8, 2.5, 1.0, 1000, material);
    obstacle->SetPos(ChVector3d(1.0, 1.0, 1.6));
    obstacle->SetFixed(true);
    obstacle->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle->GetVisualShape(0)->SetOpacity(0.1);
    sys.Add(obstacle);

    obstacle2 = chrono_types::make_shared<chrono::ChBodyEasyBox>(1.2, 0.5, 1.0, 1000, material);
    obstacle2->SetPos(ChVector3d(2.5, 2.0, 1.6));
    obstacle2->SetFixed(true);
    obstacle2->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle2->GetVisualShape(0)->SetOpacity(0.1);
    sys.Add(obstacle2);

    obstacle3 = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 2.5, 1.0, 1000, material);
    obstacle3->SetPos(ChVector3d(3.5, 1.0, 1.6));
    obstacle3->SetFixed(true);
    obstacle3->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle3->GetVisualShape(0)->SetOpacity(0.1);
    sys.Add(obstacle3);

    obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(1.5, 0.5, 1.0, 1000, material);    //(x = 1.2)
    //obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(3, 0.5, 3.0, 1000, material);
    //obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(2.5, 0.5, 3.0, 1000, material);
    obstacle4->SetPos(ChVector3d(2.5, -0.8, 1.6));
    obstacle4->SetFixed(true);
    obstacle4->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle4->GetVisualShape(0)->SetOpacity(0.1);
    sys.Add(obstacle4);
}

void scaraRobot2::createMotors(ChSystemNSC& sys) {
    motor1 = chrono_types::make_shared<chrono::ChLinkMateRevolute>();
    motor2 = chrono_types::make_shared<chrono::ChLinkMateRevolute>();
    motorZ = chrono_types::make_shared<chrono::ChLinkMatePrismatic>();

    motor1->Initialize(body1,
        		        groundBody,
        		        ChFramed(ChVector3d(0.0, 1.0, height0), QUNIT));

    motor2->Initialize(body2,
                        body1,
                        ChFrame(ChVector3d(0.0, -1.0, height0 + 0.8), QUNIT));

    motorZ->Initialize(bodyZ,
                        body2,
                        ChFrame<>(bodyZ->GetPos(), QUNIT));

    sys.Add(motor1);
    sys.Add(motor2);
    sys.Add(motorZ);
}



int main(int argc, char* argv[]) {

    //SetChronoDataPath(CHRONO_DATA_DIR);
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    sys.SetCollisionSystemType(collision_type);

    sys.SetSolverType(ChSolver::Type::MINRES);
    //sys.GetSolver()->AsIterative()->SetMaxIterations(400);
    ChRealtimeStepTimer realtime_timer;
    double timestep = 0.01;

    /////////////////////////////////////////// CREATE OBSTACLES /////////////////////////////////////////////


    /////////////////////UR5 ///////////////////
    int obstacles_collision_family = 0;
    int robot_collision_family = 1;
    int robot_base_family = 2;
    int robot_shoulder_family = 3;
    int robot_biceps_family = 4;
    int robot_elbow_family = 5;
    int robot_wrist_family = 6;
    int robot_forearm_family = 7;
    int robot_end_effector_family = 8;
    int industrial_shelf_2_family = 9;
    int industrial_shelf_3_family = 10;
    int industrial_conveyor_1_family = 11;
    int box_collision_family = 12;
    int industrial_shelf_1_family = 13;
    int industrial_base_family = 14;

    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    auto groundBody = chrono_types::make_shared<ChBodyEasyBox>(5, 0.5, 5, 3000, material);
    groundBody->SetPos(ChVector3d(0.0, -0.4, 0.0));
    groundBody->GetCollisionModel()->SetFamily(obstacles_collision_family);
    groundBody->SetFixed(true);
    groundBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(groundBody);

    //// OBSTACLES UR5 ENV 1
    //auto obstacle = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.1, 0.25, 0.5, 1000, material);
    //obstacle->SetPos(ChVector3d(0.2, 0.8, -0.1));
    //obstacle->SetFixed(true);
    //obstacle->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    //obstacle->GetCollisionModel()->SetFamily(obstacles_collision_family);
    //obstacle->GetCollisionModel()->SetEnvelope(0.002);
    //sys.Add(obstacle);

    //auto obstacle2 = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 0.3, 0.1, 1000, material);
    //obstacle2->SetPos(ChVector3d(-0.15, 0.8, -0.3));
    //obstacle2->SetFixed(true);
    //obstacle2->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    //obstacle2->GetCollisionModel()->SetFamily(obstacles_collision_family);
    //obstacle2->GetCollisionModel()->SetEnvelope(0.002);
    //obstacle2->GetVisualShape(0)->SetOpacity(0.5);
    //sys.Add(obstacle2);

    //auto obstacle3 = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 0.1, 0.3, 1000, material);
    //obstacle3->SetPos(ChVector3d(-0.15, 0.15, -0.5));
    //obstacle3->SetFixed(true);
    //obstacle3->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    //obstacle3->GetCollisionModel()->SetFamily(obstacles_collision_family);
    //obstacle3->GetCollisionModel()->SetEnvelope(0.002);
    //sys.Add(obstacle3);

    //auto obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 0.1, 0.3, 1000, material);
    //obstacle4->SetPos(ChVector3d(-0.15, 0.35, 0.5));
    //obstacle4->SetFixed(true);
    //obstacle4->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    //obstacle4->GetCollisionModel()->SetFamily(obstacles_collision_family);
    //obstacle4->GetCollisionModel()->SetEnvelope(0.002);
    //sys.Add(obstacle4);

    //auto obstacle5 = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.1, 0.25, 0.5, 1000, material);
    //obstacle5->SetPos(ChVector3d(-0.50, 0.8, -0.1));
    //obstacle5->SetFixed(true);
    //obstacle5->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    //obstacle5->GetCollisionModel()->SetFamily(obstacles_collision_family);
    //obstacle5->GetCollisionModel()->SetEnvelope(0.002);
    //sys.Add(obstacle5);



////////////////////////IMPORT CAD //////////////////////////////


////////////////////// IMPORT UR5 /////////////////////////////////////
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>bodylist;
std::vector<std::shared_ptr<chrono::ChLinkBase>>linklist;
ImportSolidworksSystemCpp(bodylist, linklist);

     /////////////////////////// UR5 ////////////////
for (auto& body : bodylist) {
    if (body->GetCollisionModel()) {
        //    body->GetCollisionModel()->SetFamily(robot_collision_family);
        //    body->GetCollisionModel()->DisallowCollisionsWith(robot_collision_family);
        //    //body->GetCollisionModel()->DisallowCollisionsWith(obstacles_collision_family);
        body->GetCollisionModel()->SetEnvelope(0.001);
        // body->EnableCollision(false);
        if (body->GetName() == "goal_new-2") {
            body->GetCollisionModel()->SetFamily(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_forearm_family);
            body->GetCollisionModel()->DisallowCollisionsWith(obstacles_collision_family);
        }
        if (body->GetName() == "start_new-1") {
            body->GetCollisionModel()->SetFamily(obstacles_collision_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family);
            body->GetCollisionModel()->DisallowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_forearm_family);
        }
        if (body->GetName() == "industrial_shelf-1") {
            body->GetCollisionModel()->SetFamily(industrial_shelf_1_family);
            body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_2_family);
            body->GetCollisionModel()->AllowCollisionsWith(box_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family);

        }
        if (body->GetName() == "industrial_shelf-2") {
            body->GetCollisionModel()->SetFamily(industrial_shelf_1_family);
            body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(box_collision_family);

        }
        if (body->GetName() == "Base_UR5_STEP-1") {
            body->GetCollisionModel()->SetFamily(robot_base_family);
            body->GetCollisionModel()->DisallowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_forearm_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->DisallowCollisionsWith(industrial_base_family);
        }
        if (body->GetName() == "Link1_UR5_STEP-1") {
            body->GetCollisionModel()->SetFamily(robot_shoulder_family);
            body->GetCollisionModel()->DisallowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_forearm_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_1_family);

        }
        if (body->GetName() == "Link2_UR5_STEP-1") {
            body->GetCollisionModel()->SetFamily(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_forearm_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_1_family);

        }
        if (body->GetName() == "Link3_UR5_STEP-1") {
            body->GetCollisionModel()->SetFamily(robot_elbow_family);
            body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family); //////
            body->GetCollisionModel()->DisallowCollisionsWith(robot_forearm_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_1_family);

        }
        if (body->GetName() == "Link4_UR5_STEP_1000515-1") {
            body->GetCollisionModel()->SetFamily(robot_forearm_family);
            body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_end_effector_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_1_family);
        }
        if (body->GetName() == "Link5_UR5_STEP-1") {
            body->GetCollisionModel()->SetFamily(robot_wrist_family);
            body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_end_effector_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_forearm_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_elbow_family);
            body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_1_family);

        }
        if (body->GetName() == "End_effector_assembly2-1"/*"Link6_UR5_STEP-1"*/) {
            body->GetCollisionModel()->SetFamily(robot_end_effector_family);
            body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
            body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_elbow_family); ////// Allow
            body->GetCollisionModel()->DisallowCollisionsWith(robot_wrist_family);
            body->GetCollisionModel()->DisallowCollisionsWith(robot_forearm_family);
            body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_1_family);
        }


    }
    sys.AddBody(body);
}
for (auto& link : linklist) {
    sys.AddLink(link);
}

 
    industrial::ChRobot_6dof_CAD robot(&sys);

    std::shared_ptr<chrono::ChJointRevolute> joint_base_shoulder;
    std::shared_ptr<chrono::ChJointRevolute> joint_shoulder_biceps;
    std::shared_ptr<chrono::ChJointRevolute> joint_biceps_elbow;
    std::shared_ptr<chrono::ChJointRevolute> joint_elbow_forearm;
    std::shared_ptr<chrono::ChJointRevolute> joint_forearm_wrist;
    std::shared_ptr<chrono::ChJointRevolute> joint_wrist_end_effector;

    joint_base_shoulder = std::make_shared<ChJointRevolute>(-10, 10, false, robot.GeMotorBaseShoulder());
    joint_shoulder_biceps = std::make_shared<ChJointRevolute>(-10, 10, false, robot.GeMotorShoulderBiceps());
    joint_biceps_elbow = std::make_shared<ChJointRevolute>(-10, 10, false, robot.GeMotorBicepsElbow());
    joint_elbow_forearm = std::make_shared<ChJointRevolute>(-10, 10, false, robot.GeMotorElbowForearm());
    joint_forearm_wrist = std::make_shared<ChJointRevolute>(-10, 10, true, robot.GeMotorForearmWrist());
    joint_wrist_end_effector = std::make_shared<ChJointRevolute>(-10, 10, true, robot.GeMotorWristEndeffector());

    //////////////////////////////////////////////////////

    //CartesianRobot cartesianRobot;
    //cartesianRobot.initialize(sys);

    //scaraRobot scaraRobot;
    //scaraRobot.initialize(sys);



 /////////////////////////////////////////////////////////////////////////////////////////////////// 
  
  
 // DEFINE PLANNER

    ChPlanModel planModel(&sys);
  
    ChPlannerOMPL planner(&planModel, ChPlannerOMPL::PlannerType::RRTCONNECT);

    const unsigned int nmotors = 6;
    ChVectorDynamic<> start(nmotors);

    ChVectorDynamic<> goal(nmotors);


    
///////////////////////////////////////////////////////////
////   CARTESIAN ROBOT
///////////////////////////////////////////////////////////
//
        //start[0] = cartesianRobot.motorX->GetMotorPos();
        //std::cout << "Start: " << start[0] << std::endl;

        //start[1] = cartesianRobot.motorY->GetMotorPos();
        //std::cout << "Start: " << start[1] << std::endl;

        //start[2] = cartesianRobot.motorZ->GetMotorPos();
        //std::cout << "Start: " << start[2] << std::endl;


        //goal[0] = 3.4;                 
        //goal[1] = -1.8;
        //goal[2] = -0.1;
  
  ////////////////////////////////////////////////////////////////////////
  //SCARA ROBOT
  ////////////////////////////////////////////////////////////////////////
 

        //start[0] = scaraRobot.motor1->GetAngleOffset();
        //std::cout << "Start: " << start[0] << std::endl;

        //start[1] = scaraRobot.motor2->GetAngleOffset();
        //std::cout << "Start: " << start[1] << std::endl;

        //start[2] = scaraRobot.motorZ->GetMotorPos();
        //std::cout << "Start: " << start[2] << std::endl;



        //goal[0] = 2.36951;                 
        //goal[1] = -2.06501;
        //goal[2] = -2.97366;


// UR5 START AND GOAL FOR ENV1 AND ENV2

    //start[0] = robot.GeMotorBaseShoulder()->GetAngleOffset();
    //std::cout << "Start: " << start[0] << std::endl;

    //start[1] = robot.GeMotorShoulderBiceps()->GetAngleOffset();
    //std::cout << "Start: " << start[1] << std::endl;

    //start[2] = robot.GeMotorBicepsElbow()->GetAngleOffset();
    //std::cout << "Start: " << start[2] << std::endl;

    //start[3] = robot.GeMotorElbowForearm()->GetAngleOffset();
    //std::cout << "Start: " << start[3] << std::endl;

    //start[4] = robot.GeMotorForearmWrist()->GetAngleOffset();
    //std::cout << "Start: " << start[4] << std::endl;

    //start[5] = robot.GeMotorWristEndeffector()->GetAngleOffset();
    //std::cout << "Start: " << start[5] << std::endl;



    //goal[0] = -2.92067;
    //goal[1] = 1.21734;
    //goal[2] = 1.11218;
    //goal[3] = 3.03643;
    //goal[4] = 0.220921;
    //goal[5] = -1.47201;

////////////////////// START AND GOAL FOR SHELVES

        
    //start[0] = 0.247047;    
    //start[1] = -0.650365;
    //start[2] = 0.663831;
    //start[3] = 1.3142;
    //start[4] = 2.89455;
    //start[5] = 1.66958;




    //goal2[0] = -2.35;         
    //goal2[1] = -0.526721;
    //goal2[2] = 0.7;
    //goal2[3] = -2.02;
    //goal2[4] = 0.738725;
    //goal2[5] = -1.48671;

//////////////////////////////////////////////////

//////////////////// START AND GOAL FOR ENV3 (LASER)

    // env3
    start[0] = 1.60;
    start[1] = -0.35;
    start[2] = 1.15;
    start[3] = 3.2;
    start[4] = -1.45;
    start[5] = 0.0;



    // INTERMEDIATE GOAL
    //goal[0] = -0.6; 
    //goal[1] = -1.64;  
    //goal[2] = 1.70;
    //goal[3] = 8.5;
    //goal[4] = -1.05;
    //goal[5] = 4.53593619;


    //INTERMEDIATE GOAL 2
    //goal2[0] = -1.1;
    //goal2[1] = -1.7;
    //goal2[2] = 1.79;
    //goal2[3] = 8.3;
    //goal2[4] = -1.1;
    //goal2[5] = 4.53593619;

    //INTERMEDIATE GOAL 3
    //goal3[0] = -1.3;
    //goal3[1] = -1.8;
    //goal3[2] =1.85;
    //goal3[3] = 7.5;
    //goal3[4] = -1.5;
    //goal3[5] = 4.53593619;


    // GOAL
    goal4[0] = -1.4;
    goal4[1] = -1.9;
    goal4[2] = 0.52;
    goal4[3] = 7.0;
    goal4[4] = -1.55;
    goal4[5] = 3.8;

    std::vector<std::shared_ptr<ChJoint>> jointVect;

    //jointVect.push_back(cartesianRobot.jointX);
    //jointVect.push_back(cartesianRobot.jointY);
    //jointVect.push_back(cartesianRobot.jointZ);
  
    //jointVect.push_back(scaraRobot.joint1);
    //jointVect.push_back(scaraRobot.joint2);
    //jointVect.push_back(scaraRobot.jointZ);


    jointVect.push_back(joint_base_shoulder);
    jointVect.push_back(joint_shoulder_biceps);
    jointVect.push_back(joint_biceps_elbow);
    jointVect.push_back(joint_elbow_forearm);
    jointVect.push_back(joint_forearm_wrist);
    jointVect.push_back(joint_wrist_end_effector);
 
    int numjoints = jointVect.size();
    std::cout << "Number of Joints: " << numjoints << std::endl;

    for (int i = 0; i < numjoints; ++i) {
		planModel.AddJoint(jointVect[i]);
	}


    double duration = 1000;
    double duration1 = 1000;

    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(duration);
    //auto ptc = planner.firstSolutionPTC();


    // PLANNER SETUP
    planner.setDuration(duration);
    planner.setGoal(&goal4);
    planner.setStart(&start);

    // IN CASE OF MULTIPLE GOALS
    
    //planner2.setDuration(duration1);
    //planner2.setStart(&goal);
    //planner2.setGoal(&goal2/*goal3*/);

    //planner3.setDuration(duration);
    //planner3.setStart(&goal2);
    //planner3.setGoal(&goal4);

    //planner4.setDuration(duration);
    //planner4.setStart(&goal3);
    //planner4.setGoal(&goal4);

    std::vector<ChVectorDynamic<>> optim_path;      // DEFINITION OF THE OPTIMIZED PATH
    //std::vector<ChVectorDynamic<>> optim_path2;
    //std::vector<ChVectorDynamic<>> optim_path3;
    //std::vector<ChVectorDynamic<>> optim_path4;

    // SOLVE THE PLANNING PROBLEM AND DEFINE THE OPTIMIZED PATH

    if (write_mode) {

        bool isVerified = planner.verify();
        std::cout << "Configuration Verified: " << isVerified << std::endl;
        if (!isVerified) {
            std::cerr << "Start or Goal configuration are not valid." << std::endl;
            return -1;
        }

        bool isSolved = planner.solve(ptc);
        if (!isSolved) {
            std::cerr << "Cannot find solution." << std::endl;
            return -2;
        }

        std::cout << "Configuration Solved: " << isSolved << std::endl;
        std::cout << "Iteratio: " << planner.getIterationCount() << std::endl;
        optim_path = planner.getPath();
        

        // IN CASE OF MULTIPLE GOALS


        //bool isVerified2 = planner2.verify();
        //std::cout << "Configuration Verified: " << isVerified2 << std::endl;
        //if (!isVerified2) {
        //    std::cerr << "Start or Goal configuration are not valid." << std::endl;
        //    return -1;
        //}

        //bool isSolved2 = planner2.solve(duration);
        //if (!isSolved2) {
        //    std::cerr << "Cannot find solution." << std::endl;
        //    return -2;
        //}

        //std::cout << "Configuration Solved: " << isSolved2 << std::endl;
        //std::cout << "Iteratio: " << planner2.getIterationCount() << std::endl;
        //optim_path2 = planner2.getPath();

        //std::cout << "HERE" << std::endl;
        //bool isVerified3 = planner3.verify();
        //std::cout << "Configuration Verified: " << isVerified3 << std::endl;
        //if (!isVerified3) {
        //    std::cerr << "Start or Goal configuration are not valid." << std::endl;
        //    return -1;
        //}

        //bool isSolved3 = planner3.solve(duration);
        //
        //if (!isSolved3) {
        //    std::cerr << "Cannot find solution." << std::endl;
        //    return -2;
        //}

        //std::cout << "Configuration Solved: " << isSolved2 << std::endl;
        //std::cout << "Iteratio: " << planner3.getIterationCount() << std::endl;
        //optim_path3 = planner3.getPath();


        //std::cout << "HERE" << std::endl;
        //bool isVerified5 = planner4.verify();
        //std::cout << "Configuration Verified: " << isVerified5 << std::endl;
        //if (!isVerified5) {
        //    std::cerr << "Start or Goal configuration are not valid." << std::endl;
        //    return -1;
        //}

        //bool isSolved5 = planner4.solve(duration);
        //if (!isSolved5) {
        //    std::cerr << "Cannot find solution." << std::endl;
        //    return -2;
        //}

        //std::cout << "Configuration Solved: " << isSolved5 << std::endl;
        //std::cout << "Iteratio: " << planner4.getIterationCount() << std::endl;
        //optim_path4 = planner4.getPath();

        std::vector<ChVectorDynamic<>> final_path;




        std::ofstream file("optim_path.json");
        ChArchiveOutJSON archive(file);
        archive << CHNVP(optim_path);

        std::ofstream file2("optim_path2.json");
        ChArchiveOutJSON archive2(file2);
        archive2 << CHNVP(optim_path2);

        std::ofstream file3("optim_path3.json");
        ChArchiveOutJSON archive3(file3);
        archive3 << CHNVP(optim_path3);

        std::ofstream file4("optim_path4.json");
        ChArchiveOutJSON archive4(file4);
        archive4 << CHNVP(optim_path4);

        file.flush();
        file2.flush();
        file3.flush();
        file4.flush();
    }
    else {
        std::ifstream file("final_path_infRRT_star.json");
        ChArchiveInJSON archive(file);
        archive >> CHNVP(optim_path);

        std::ifstream file2("optim_path2.json");
        ChArchiveInJSON archive2(file2);
        archive2 >> CHNVP(optim_path2);

        std::ifstream file3("optim_path3.json");
        ChArchiveInJSON archive3(file3);
        archive3 >> CHNVP(optim_path3);


        std::ifstream file4("optim_path4.json");
        ChArchiveInJSON archive4(file4);
        archive4 >> CHNVP(optim_path4);
    }


/////////////////////// SCARA/CARTESIAN ROBOT /////////////////////////////////
    //std::vector<ChVector3d> recorded_posZ;
    //std::vector<ChVector3d> recorded_posY;
    //std::vector<ChVector3d> recorded_posX;

    //std::vector<ChVector3d> recorded_posZ2;
    //std::vector<ChVector3d> recorded_posY2;
    //std::vector<ChVector3d> recorded_posX2;

    //std::vector<ChVector3d> recorded_posZ3;
    //std::vector<ChVector3d> recorded_posY3;
    //std::vector<ChVector3d> recorded_posX3;
         
//////////////////////////////////////////6 DOF ROBOT //////////////////////////////////////

    std::vector<ChVector3d> recorded_pos_base;
    std::vector<ChVector3d> recorded_pos_shoulder;
    std::vector<ChVector3d> recorded_pos_biceps;
    std::vector<ChVector3d> recorded_pos_elbow;
    std::vector<ChVector3d> recorded_pos_forearm;
    std::vector<ChVector3d> recorded_pos_wrist;
    std::vector<ChVector3d> recorded_pos_end_effector;

    std::vector<ChVector3d> recorded_pos_base2;
    std::vector<ChVector3d> recorded_pos_shoulder2;
    std::vector<ChVector3d> recorded_pos_biceps2;
    std::vector<ChVector3d> recorded_pos_elbow2;
    std::vector<ChVector3d> recorded_pos_forearm2;
    std::vector<ChVector3d> recorded_pos_wrist2;
    std::vector<ChVector3d> recorded_pos_end_effector2;

    std::vector<ChVector3d> recorded_pos_base3;
    std::vector<ChVector3d> recorded_pos_shoulder3;
    std::vector<ChVector3d> recorded_pos_biceps3;
    std::vector<ChVector3d> recorded_pos_elbow3;
    std::vector<ChVector3d> recorded_pos_forearm3;
    std::vector<ChVector3d> recorded_pos_wrist3;
    std::vector<ChVector3d> recorded_pos_end_effector3;

    std::vector<ChVector3d> recorded_pos_base4;
    std::vector<ChVector3d> recorded_pos_shoulder4;
    std::vector<ChVector3d> recorded_pos_biceps4;
    std::vector<ChVector3d> recorded_pos_elbow4;
    std::vector<ChVector3d> recorded_pos_forearm4;
    std::vector<ChVector3d> recorded_pos_wrist4;
    std::vector<ChVector3d> recorded_pos_end_effector4;


    for (const auto& cfg : optim_path) {
        std::cout << "cfg: " << cfg << " | ";
        planModel.setPosition(cfg);

        sys.DoAssembly(AssemblyLevel::POSITION, 10);

        //recorded_posZ.push_back(cartesianRobot.bodyX->GetPos());
        //recorded_posY.push_back(cartesianRobot.bodyY->GetPos());
        //recorded_posX.push_back(cartesianRobot.bodyZ->GetPos());
       
        //recorded_posZ.push_back(scaraRobot.body1->GetPos());
        //recorded_posY.push_back(scaraRobot.body2->GetPos());
        //recorded_posX.push_back(scaraRobot.bodyZ->GetPos());

        recorded_pos_base.push_back(robot.GetBase()->GetPos());
        recorded_pos_shoulder.push_back(robot.GetShoulder()->GetPos());
        recorded_pos_biceps.push_back(robot.GetBiceps()->GetPos());
        recorded_pos_elbow.push_back(robot.GetElbow()->GetPos());
        recorded_pos_forearm.push_back(robot.GetForearm()->GetPos());
        recorded_pos_wrist.push_back(robot.GetWrist()->GetPos());
        recorded_pos_end_effector.push_back(robot.GetEndEffector()->GetPos());
    }


    /// IN CASE OF MULTIPLE GOALS

    //for (const auto& cfg2 : optim_path2) {
    //    std::cout << "cfg: " << cfg2 << " | ";
    //    planModel.setPosition(cfg2);

    //    sys.DoAssembly(AssemblyLevel::POSITION, 10);

    //    //recorded_posZ2.push_back(scaraRobot.body1->GetPos());
    //    //recorded_posY2.push_back(scaraRobot.body2->GetPos());
    //    //recorded_posX2.push_back(scaraRobot.bodyZ->GetPos());

    //    recorded_pos_base2.push_back(robot.GetBase()->GetPos());
    //    recorded_pos_shoulder2.push_back(robot.GetShoulder()->GetPos());
    //    recorded_pos_biceps2.push_back(robot.GetBiceps()->GetPos());
    //    recorded_pos_elbow2.push_back(robot.GetElbow()->GetPos());
    //    recorded_pos_forearm2.push_back(robot.GetForearm()->GetPos());
    //    recorded_pos_wrist2.push_back(robot.GetWrist()->GetPos());
    //    recorded_pos_end_effector2.push_back(robot.GetEndEffector()->GetPos());
    //}

    //for (const auto& cfg3 : optim_path3) {
    //    std::cout << "cfg: " << cfg3 << " | ";
    //    planModel.setPosition(cfg3);

    //    sys.DoAssembly(AssemblyLevel::POSITION, 10);

    //    //recorded_posZ3.push_back(scaraRobot.body1->GetPos());
    //    //recorded_posY3.push_back(scaraRobot.body2->GetPos());
    //    //recorded_posX3.push_back(scaraRobot.bodyZ->GetPos());

    //    recorded_pos_base3.push_back(robot.GetBase()->GetPos());
    //    recorded_pos_shoulder3.push_back(robot.GetShoulder()->GetPos());
    //    recorded_pos_biceps3.push_back(robot.GetBiceps()->GetPos());
    //    recorded_pos_elbow3.push_back(robot.GetElbow()->GetPos());
    //    recorded_pos_forearm3.push_back(robot.GetForearm()->GetPos());
    //    recorded_pos_wrist3.push_back(robot.GetWrist()->GetPos());
    //    recorded_pos_end_effector3.push_back(robot.GetEndEffector()->GetPos());
    //}


    //for (const auto& cfg4 : optim_path4) {
    //    std::cout << "cfg: " << cfg4 << " | ";
    //    planModel.setPosition(cfg4);

    //    sys.DoAssembly(AssemblyLevel::POSITION, 100);

    //    //recorded_posZ3.push_back(scaraRobot.body1->GetPos());
    //    //recorded_posY3.push_back(scaraRobot.body2->GetPos());
    //    //recorded_posX3.push_back(scaraRobot.bodyZ->GetPos());

    //    //recorded_posZ2.push_back(singleJoint.body1->GetPos());

    //    recorded_pos_base4.push_back(robot.GetBase()->GetPos());
    //    recorded_pos_shoulder4.push_back(robot.GetShoulder()->GetPos());
    //    recorded_pos_biceps4.push_back(robot.GetBiceps()->GetPos());
    //    recorded_pos_elbow4.push_back(robot.GetElbow()->GetPos());
    //    recorded_pos_forearm4.push_back(robot.GetForearm()->GetPos());
    //    recorded_pos_wrist4.push_back(robot.GetWrist()->GetPos());
    //    recorded_pos_end_effector4.push_back(robot.GetEndEffector()->GetPos());
    //}
    ///////////////////////////////////////////////////////////////////////////////////


    ///////////// LENGHT OF THE PATH FOR SCARA/CARTESIAN ROBOT /////////////////////////////

    //std::vector<double> segment_lengths;
    //segment_lengths.push_back(0.0);
    //double cum_length = 0.0;
    //for (size_t i = 1; i < recorded_posZ.size(); ++i) {
    //    ChVector3d point1 = recorded_posZ[i-1];
    //    ChVector3d point2 = recorded_posZ[i];
    //    double segment_length = std::sqrt(std::pow(point2.x() - point1.x(), 2) +
    //        std::pow(point2.y() - point1.y(), 2) +
    //        std::pow(point2.z() - point1.z(), 2));
    //    cum_length += segment_length;
    //    segment_lengths.push_back(cum_length);
    //    
    //}

    //std::vector<double> segment_lengths2;
    //segment_lengths2.push_back(0.0);
    //double cum_length2 = 0.0;
    //for (size_t i = 1; i < recorded_posZ2.size(); ++i) {
    //    ChVector3d point1 = recorded_posZ2[i - 1];
    //    ChVector3d point2 = recorded_posZ2[i];
    //    double segment_length2 = std::sqrt(std::pow(point2.x() - point1.x(), 2) +
    //        std::pow(point2.y() - point1.y(), 2) +
    //        std::pow(point2.z() - point1.z(), 2));
    //    cum_length2 += segment_length2;
    //    segment_lengths2.push_back(cum_length2);

    //}

    //std::vector<double> segment_lengths3;
    //segment_lengths3.push_back(0.0);
    //double cum_length3 = 0.0;
    //for (size_t i = 1; i < recorded_posZ3.size(); ++i) {
    //    ChVector3d point1 = recorded_posZ3[i - 1];
    //    ChVector3d point2 = recorded_posZ3[i];
    //    double segment_length3 = std::sqrt(std::pow(point2.x() - point1.x(), 2) +
    //        std::pow(point2.y() - point1.y(), 2) +
    //        std::pow(point2.z() - point1.z(), 2));
    //    cum_length3 += segment_length3;
    //    segment_lengths3.push_back(cum_length3);

    //}


//////////////////////////////////////////////////////////////////////////////////////////////


///////////// LENGHT OF THE PATH FOR 6 DOF ROBOT /////////////////////////////

    std::vector<double> segment_lengths;
    segment_lengths.push_back(0.0);
    double cum_length = 0.0;
    for (size_t i = 1; i < recorded_pos_end_effector.size(); ++i) {
        ChVector3d point1 = recorded_pos_end_effector[i - 1];
        ChVector3d point2 = recorded_pos_end_effector[i];
        double segment_length = std::sqrt(std::pow(point2.x() - point1.x(), 2) +
            std::pow(point2.y() - point1.y(), 2) +
            std::pow(point2.z() - point1.z(), 2));
        cum_length += segment_length;
        segment_lengths.push_back(cum_length);

    }

    // IN CASE OF MULTIPLE GOALS

    //std::vector<double> segment_lengths2;
    //segment_lengths2.push_back(0.0);
    //double cum_length2 = 0.0;
    //for (size_t i = 1; i < recorded_pos_end_effector2.size(); ++i) {
    //    ChVector3d point1 = recorded_pos_end_effector2[i - 1];
    //    ChVector3d point2 = recorded_pos_end_effector2[i];
    //    double segment_length2 = std::sqrt(std::pow(point2.x() - point1.x(), 2) +
    //        std::pow(point2.y() - point1.y(), 2) +
    //        std::pow(point2.z() - point1.z(), 2));
    //    cum_length2 += segment_length2;
    //    segment_lengths2.push_back(cum_length2);

    //}

    //std::vector<double> segment_lengths3;
    //segment_lengths3.push_back(0.0);
    //double cum_length3 = 0.0;
    //for (size_t i = 1; i < recorded_pos_end_effector3.size(); ++i) {
    //    ChVector3d point1 = recorded_pos_end_effector3[i - 1];
    //    ChVector3d point2 = recorded_pos_end_effector3[i];
    //    double segment_length3 = std::sqrt(std::pow(point2.x() - point1.x(), 2) +
    //        std::pow(point2.y() - point1.y(), 2) +
    //        std::pow(point2.z() - point1.z(), 2));
    //    cum_length3 += segment_length3;
    //    segment_lengths3.push_back(cum_length3);

    //}


    //std::vector<double> segment_lengths4;
    //segment_lengths4.push_back(0.0);
    //double cum_length4 = 0.0;
    //for (size_t i = 1; i < recorded_pos_end_effector4.size(); ++i) {
    //    ChVector3d point1 = recorded_pos_end_effector4[i - 1];
    //    ChVector3d point2 = recorded_pos_end_effector4[i];
    //    double segment_length4 = std::sqrt(std::pow(point2.x() - point1.x(), 2) +
    //        std::pow(point2.y() - point1.y(), 2) +
    //        std::pow(point2.z() - point1.z(), 2));
    //    cum_length4 += segment_length4;
    //    segment_lengths4.push_back(cum_length4);

    //}


    ///////////////////////////////////////////////////////////////////////////

    double total_time = 10;
    double total_length = *segment_lengths.rbegin();

    std::vector<ChFunctionInterp> interpJoints;
    interpJoints.resize(numjoints);

    for (size_t optpoint = 0; optpoint < optim_path.size(); ++optpoint) {
        for (size_t joint = 0; joint < numjoints; ++joint) {

            interpJoints[joint].AddPoint(segment_lengths[optpoint] / total_length, optim_path[optpoint][joint], true);
        }
    }
    std::cout << "total length: " << total_length << std::endl;

    // IN CASE OF MULTIPLE GOALS
     
    
    //double total_time2 = 5;
    //double total_length2 = *segment_lengths2.rbegin();

    //std::vector<ChFunctionInterp> interpJoints2;
    //interpJoints2.resize(numjoints);

    //for (size_t optpoint2 = 0; optpoint2 < optim_path2.size(); ++optpoint2) {
    //    for (size_t joint = 0; joint < numjoints; ++joint) {

    //        interpJoints2[joint].AddPoint(segment_lengths2[optpoint2] / total_length2, optim_path2[optpoint2][joint], true);
    //    }
    //}
    //std::cout << "total length2: " << total_length2 << std::endl;
    //double total_time3 = 5;
    //double total_length3 = *segment_lengths3.rbegin();

    //std::vector<ChFunctionInterp> interpJoints3;
    //interpJoints3.resize(numjoints);

    //for (size_t optpoint3 = 0; optpoint3 < optim_path3.size(); ++optpoint3) {
    //    for (size_t joint = 0; joint < numjoints; ++joint) {

    //        interpJoints3[joint].AddPoint(segment_lengths3[optpoint3] / total_length3, optim_path3[optpoint3][joint], true);
    //    }
    //}
    //std::cout << "total length3: " << total_length3 << std::endl;



    //double total_time4 = 5;
    //double total_length4 = *segment_lengths4.rbegin();

    //std::vector<ChFunctionInterp> interpJoints4;
    //interpJoints4.resize(numjoints);

    //for (size_t optpoint4 = 0; optpoint4 < optim_path4.size(); ++optpoint4) {
    //    for (size_t joint = 0; joint < numjoints; ++joint) {

    //        interpJoints4[joint].AddPoint(segment_lengths4[optpoint4] / total_length4, optim_path4[optpoint4][joint], true);
    //    }
    //}
    //std::cout << "total length4: " << total_length4 << std::endl;


    //////////////////////////////////////////////////////////////////////////////////////////////////////

    // 4 - Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("CartesianRobot");
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(1.56552, 0.635998, -1.06847));
    vis->AddTypicalLights();
    vis->AddLightWithShadow(ChVector3d(20.0, 35.0, 25.0), ChVector3d(0.0, 0.0, 0.0), 55, 20, 55, 35, 512,
        ChColor(0.6f, 0.8f, 1.0f));
    vis->EnableShadows();
    vis->EnableCollisionShapeDrawing(false);

    for (auto& body : sys.GetBodies()) {
        body->EnableCollision(true);
	}
 
    bool firstMovementCompleted = false;
    bool secondMovementCompleted = false;
    bool thirdMovementCompleted = false;
    ChTimer tim;
    tim.reset();
    tim.start();
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        //if (!firstMovementCompleted) {
            ChVectorDynamic<> curcfg(numjoints);
            for (size_t j = 0; j < numjoints; ++j) {
                curcfg[j] = interpJoints[j].GetVal(tim.GetTimeMilliseconds() / 1000.0 / total_time);
            }
            planModel.setPosition(curcfg);
            

////////////// IN CASE OF MULTIPLE GOALS AND MULTIPLE PATH /////////////////////////

        //    if ((tim.GetTimeMilliseconds() / 1000.0 >= total_time)) {
        //        firstMovementCompleted = true;
        //        tim.reset();
        //        tim.start();
        //    }
        //    
        //}
       
        //else if (!secondMovementCompleted) {
        //    ChVectorDynamic<> curcfg2(numjoints);
        //    for (size_t j = 0; j < numjoints; ++j) {
        //        curcfg2[j] = interpJoints2[j].GetVal(tim.GetTimeMilliseconds() / 1000.0 / total_time2);
        //    }
        //    planModel.setPosition(curcfg2);

        //    if ((tim.GetTimeMilliseconds() / 1000.0 >= total_time2)) {
        //        secondMovementCompleted = true;
        //        tim.reset();
        //        tim.start();

        //    }

        //}
        //else if(!thirdMovementCompleted){
        //    ChVectorDynamic<> curcfg3(numjoints);
        //    for (size_t j = 0; j < numjoints; ++j) {
        //        curcfg3[j] = interpJoints3[j].GetVal(tim.GetTimeMilliseconds() / 1000.0 / total_time3);
        //    }
        //    planModel.setPosition(curcfg3);

        //    if ((tim.GetTimeMilliseconds() / 1000.0 >= total_time3)) {
        //        thirdMovementCompleted = true;
        //        tim.reset();
        //        tim.start();

        //    }
        //}

        //else {
        //    ChVectorDynamic<> curcfg4(numjoints);
        //    for (size_t j = 0; j < numjoints; ++j) {
        //        curcfg4[j] = interpJoints4[j].GetVal(tim.GetTimeMilliseconds() / 1000.0 / total_time4);
        //    }
        //    planModel.setPosition(curcfg4);

        //    if ((tim.GetTimeMilliseconds() / 1000.0 >= total_time4)) {
        //        //break;
        //    }
        //}

    
        //////////////////////////////////////// SCARA/CARTESIAN ROBOT PATH VISUALIZATION IN THE CARTESIAN SPACE ///////////////////////////////////////
        
        //irrlicht::tools::drawPolyline(vis.get(), recorded_posZ, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_posY, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_posX, ChColor(1.f, 1.f, 0.f), true);

        //irrlicht::tools::drawPolyline(vis.get(), recorded_posZ2, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_posY2, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_posX2, ChColor(1.f, 1.f, 0.f), true);

        //irrlicht::tools::drawPolyline(vis.get(), recorded_posZ3, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_posY3, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_posX3, ChColor(1.f, 1.f, 0.f), true);


        ///////////////////////////////////// 6 DOF  ROBOT PATH VISUALIZATION IN THE CARTESIAN SPACE /////////////////////////////////////////////////

        irrlicht::tools::drawPolyline(vis.get(), recorded_pos_base, ChColor(1.f, 1.f, 0.f), true);
        irrlicht::tools::drawPolyline(vis.get(), recorded_pos_shoulder, ChColor(1.f, 1.f, 0.f), true);
        irrlicht::tools::drawPolyline(vis.get(), recorded_pos_biceps, ChColor(1.f, 1.f, 0.f), true);
        irrlicht::tools::drawPolyline(vis.get(), recorded_pos_elbow, ChColor(1.f, 1.f, 0.f), true);
        irrlicht::tools::drawPolyline(vis.get(), recorded_pos_forearm, ChColor(1.f, 1.f, 0.f), true);
        irrlicht::tools::drawPolyline(vis.get(), recorded_pos_wrist, ChColor(1.f, 1.f, 0.f), true);
        irrlicht::tools::drawPolyline(vis.get(), recorded_pos_end_effector, ChColor(1.f, 0.f, 0.f), true);  

        // IN CASE OF MULTIPLE GOALS

        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_base2, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_shoulder2, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_biceps2, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_elbow2, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_forearm2, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_wrist2, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_end_effector2, ChColor(1.f, 0.f, 0.f), true);

        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_base3, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_shoulder3, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_biceps3, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_elbow3, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_forearm3, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_wrist3, ChColor(1.f, 1.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_end_effector3, ChColor(1.f, 0.f, 0.f), true);
        //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_end_effector4, ChColor(1.f, 1.f, 0.f), true);


        // setPosition FUNCTION TO MOVE THE ROBOT IN A SPECIFIC CONFIGURATION
        //planModel.setPosition(start);

        sys.DoStepDynamics(timestep);

        vis->EndScene();
        realtime_timer.Spin(timestep);

    }

    //// 5 - Simulation loop
    //ChRealtimeStepTimer realtime_timer;
    //double step_size = 5e-3;

    //while (vis.Run()) {
    //    // Render scene
    //    vis.BeginScene();
    //    vis.Render();
    //    vis.EndScene();

    //    // Perform the integration stpe
    //    sys.DoStepDynamics(step_size);

    //    // Spin in place to maintain soft real-time
    //    realtime_timer.Spin(step_size);
    //}

    return 0;
}
