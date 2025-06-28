#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorAll.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChBasisToolsBSpline.h"
#include "chrono/geometry/ChLineBSpline.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/functions/ChFunctionPositionSetpoint.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/functions/ChFunctionRotationABCFunctions.h"
#include "chrono/functions/ChFunctionRotationSetpoint.h"
#include "chrono/functions/ChFunctionRotationAxis.h"
#include "chrono/functions/ChFunctionRotationSQUAD.h"
#include <random>

#include "chrono/physics/ChLinkMotionImposed.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "osqp/osqp.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include "osqp.h"
#include <stdlib.h>
#include <stdio.h>
#include <osqp/osqp_api_types.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include "chrono/core/ChGlobal.h"
#include "chrono/functions/ChFunctionInterp.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <random>
#include <algorithm>
#include <cmath>
#include <limits>
#include "chrono/serialization/ChArchiveJSON.h"
#include <fstream>
#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChOutputASCII.h"
#include "chrono/assets/ChVisualSystem.h"
#include <chrono>
#include <vector>
#include <tuple>
#include <array>
#include <random>
#include <algorithm>
#include <iostream>
#include "ChJoint.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "ChPlanModel.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono_irrlicht/ChIrrTools.h"
#include "ChPlanner.h"
#include "GbpfAlgorithmCube.h"
#include "CAD_export_new/new_coll.h"
#include "ChRobot_6dof_CAD.h"
#include "ChRobot_6dof_free.h"
#include "ChRobot_6dof_CAD_free.h"
#include "ChRobot_free.h"
#include "GbpfAlgorithmCuboid.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;
constexpr bool write_mode = true;
constexpr bool write_mode2 = true;
constexpr bool write3 = true;

ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;

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
    obstacle = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.7, 2.5, 0.5, 1000, material);
    //obstacle = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.8, 2.5, 1.0, 1000, material);
    obstacle->SetPos(ChVector3d(1.0, 1.0, 1.6));
    obstacle->SetFixed(true);
    obstacle->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle->GetVisualShape(0)->SetOpacity(0.1);
    obstacle->GetCollisionModel()->SetEnvelope(0.05);
    sys.Add(obstacle);

    obstacle2 = chrono_types::make_shared<chrono::ChBodyEasyBox>(1.2, 0.6, 3.0, 1000, material);
    obstacle2->SetPos(ChVector3d(2.5, 2.0, 1.6));
    obstacle2->SetFixed(true);
    obstacle2->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle2->GetVisualShape(0)->SetOpacity(0.1);
    obstacle2->GetCollisionModel()->SetEnvelope(0.05);
    sys.Add(obstacle2);

    obstacle3 = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.5, 2.5, 2.0, 1000, material);
    obstacle3->SetPos(ChVector3d(3.5, 1.0, 1.6));
    obstacle3->SetFixed(true);
    obstacle3->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle3->GetVisualShape(0)->SetOpacity(0.1);
    obstacle3->GetCollisionModel()->SetEnvelope(0.05);
    sys.Add(obstacle3);

    obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(1.5, 0.6, 3.0, 1000, material);    //(x = 1.2)
    //obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(3, 0.5, 3.0, 1000, material);
    //obstacle4 = chrono_types::make_shared<chrono::ChBodyEasyBox>(2.5, 0.5, 3.0, 1000, material);
    obstacle4->SetPos(ChVector3d(2.5, -0.8, 1.6));
    obstacle4->SetFixed(true);
    obstacle4->GetVisualShape(0)->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    obstacle4->GetVisualShape(0)->SetOpacity(0.1);
    obstacle4->GetCollisionModel()->SetEnvelope(0.05);
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
    joint1 = std::make_shared<ChJointRevolute>(-4, 4, true, motor1);
    joint2 = std::make_shared<ChJointRevolute>(-4, 4, true, motor2);
    jointZ = std::make_shared<ChJointPrismatic>(-5, 10, motorZ);
}



//int main(int argc, char* argv[]) {
//    // Set path to Chrono data directory
//    SetChronoDataPath(CHRONO_DATA_DIR);
//
//    // Create a Chrono physical system
//    ChSystemNSC sys;
//    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
//    ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;
//    sys.SetCollisionSystemType(collision_type);
//    sys.SetSolverType(ChSolver::Type::MINRES);
//    ChRealtimeStepTimer realtime_timer;
//    double timestep = 0.01;
//
//
//    ////////////////////////////////// SCARA ROBOT EXAMPLE /////////////////////////////////////
//    scaraRobot scaraRobot;
//    scaraRobot.initialize(sys);
//
//
//    ChPlanModel planModel(&sys);
//    ChPlanner planner(&planModel);
//    ChVectorDynamic<> nmotors(3);
//    ChVectorDynamic<> startPos(nmotors);
//    ChVectorDynamic<> goalPos(nmotors);
//    startPos[0] = scaraRobot.motor1->GetAngleOffset();
//    startPos[1] = scaraRobot.motor2->GetAngleOffset();
//    startPos[2] = scaraRobot.motorZ->GetMotorPos();
//    //goalPos[0] = 2.3;
//    //goalPos[1] = -2.5;
//    //goalPos[2] = -2.8;
//    //goalPos[0] = 0.3;
//    //goalPos[1] = 0.3;
//    //goalPos[2] = 0.3;
//    goalPos[0] = 2.36951;                 
//    goalPos[1] = -2.06501;
//    goalPos[2] = -2.97366;
//
//    std::vector<std::shared_ptr<ChJoint>> jointVect;
//
//    jointVect.push_back(scaraRobot.joint1);
//    jointVect.push_back(scaraRobot.joint2);
//    jointVect.push_back(scaraRobot.jointZ);
//
//    int numjoints = jointVect.size();
//    std::cout << "Number of Joints: " << numjoints << std::endl;
//
//    for (int i = 0; i < numjoints; ++i) {
//        planModel.AddJoint(jointVect[i]);
//    }
//
//    planner.setStart(&startPos);
//    planner.setGoal(&goalPos);
//    bool isVerified = planner.verify();
//    std::vector<ChVectorDynamic<>>  optim_path;
//    std::vector<ChVectorDynamic<>>  optimized_path;
//    std::vector<ChVectorDynamic<>>  reducedpath;
//
//    int max_iter = 20000;
//
//
//
//    if (write_mode) {
//        ///////////////////////////// SEARCH ALGORITHM /////////////////////////////
//        optim_path = GoalBiasedProbabilisticFoamCuboidConnect(startPos, goalPos, &planModel, sys, /*0.01*/0.01, max_iter);
//
//        std::cout << "Foam size: " << optim_path.size() << std::endl;
//
//        reducedpath = shortcutOptimize(optim_path, &planModel, 50, 100, 1.0); //0.5
//        std::cout << "Reduced path size: " << reducedpath.size() << std::endl;
//
//        optimized_path = PartialShortcut(reducedpath/*optim_path*/, &planModel, 1000);
//        std::cout << "Optimized path size: " << optimized_path.size() << std::endl;
//
//        auto final_channel_path = BuildChannelCuboids(optimized_path, &planModel, sys, 0.01, 0.2/*, startPos, goalPos*/);
//        SaveCuboidsFoamToJson(final_channel_path, "optimized_channel_cubes_data_partial_shortcut.json");
//
//        std::ofstream file("optimized_path.json");
//        ChArchiveOutJSON archive(file);
//        archive << CHNVP(optimized_path);
//
//
//        file.flush();
//    }
//    else {
//        std::ifstream file("optimized_path.json");
//        ChArchiveInJSON archive(file);
//        archive >> CHNVP(optimized_path);
//    }
//
//
//    std::cout << optimized_path.size() << std::endl;
//    std::vector<ChVector3d> recorded_posZ;
//    std::vector<ChVector3d> recorded_posY;
//    std::vector<ChVector3d> recorded_posX;
//
//    for (const auto& cfg : optimized_path) {
//        std::cout << "cfg: " << cfg << " | ";
//        planModel.setPosition(cfg);
//
//        sys.DoAssembly(AssemblyLevel::POSITION, 10);
//
//        recorded_posZ.push_back(scaraRobot.body1->GetPos());
//        recorded_posY.push_back(scaraRobot.body2->GetPos());
//        recorded_posX.push_back(scaraRobot.bodyZ->GetPos());
//        bool isColliding = planModel.isColliding(cfg);
//        bool isValid = planModel.isValid(cfg);
//    }
//
//
//
//    std::vector<double> segment_lengths;
//    segment_lengths.push_back(0.0);
//    double cum_length = 0.0;
//    for (size_t i = 1; i < recorded_posZ.size(); ++i) {
//        ChVector3d point1 = recorded_posZ[i - 1];
//        ChVector3d point2 = recorded_posZ[i];
//        double segment_length = std::sqrt(std::pow(point2.x() - point1.x(), 2) +
//            std::pow(point2.y() - point1.y(), 2) +
//            std::pow(point2.z() - point1.z(), 2));
//        cum_length += segment_length;
//        segment_lengths.push_back(cum_length);
//
//    }
//
//
//    double total_time = 10;
//    double total_length = *segment_lengths.rbegin();
//
//    std::vector<ChFunctionInterp> interpJoints;
//    interpJoints.resize(numjoints);
//
//    for (size_t optpoint = 0; optpoint < optimized_path.size(); ++optpoint) {
//        for (size_t joint = 0; joint < numjoints; ++joint) {
//
//            interpJoints[joint].AddPoint(segment_lengths[optpoint] / total_length, optimized_path[optpoint][joint], true);
//        }
//    }
//
//    // 4 - Create the Irrlicht visualization system
//    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
//    vis->AttachSystem(&sys);
//    vis->SetWindowSize(800, 600);
//    vis->SetWindowTitle("CartesianRobot");
//    vis->SetCameraVertical(CameraVerticalDir::Z);
//    vis->Initialize();
//    vis->AddLogo();
//    vis->AddSkyBox();
//    vis->AddCamera(ChVector3d(-5.0, -4.0, 6.0 /*1.5, 1.7, 1.5*/));
//    vis->AddTypicalLights();
//    vis->AddLightWithShadow(ChVector3d(20.0, 35.0, 25.0), ChVector3d(0.0, 0.0, 0.0), 55, 20, 55, 35, 512,
//        ChColor(0.6f, 0.8f, 1.0f));
//    vis->EnableShadows();
//    vis->EnableCollisionShapeDrawing(true);
//        
//    // 5 - Simulation loop
//    //ChRealtimeStepTimer realtime_timer;
//    double step_size = 5e-3;
//        
//
//        
//        
//    ChTimer tim;
//    tim.reset();
//    tim.start();
//        
//        
//    while (vis->Run()) {
//        // Render scene
//        vis->BeginScene();
//        vis->Render();
//
//        ChVectorDynamic<> curcfg(numjoints);
//        for (size_t j = 0; j < numjoints; ++j) {
//            curcfg[j] = interpJoints[j].GetVal(tim.GetTimeMilliseconds() / 1000.0 / total_time);
//        }
//        planModel.setPosition(curcfg);
//
//        irrlicht::tools::drawPolyline(vis.get(), recorded_posZ, ChColor(1.f, 1.f, 0.f), true);
//        irrlicht::tools::drawPolyline(vis.get(), recorded_posY, ChColor(1.f, 0.f, 1.f), true);
//        irrlicht::tools::drawPolyline(vis.get(), recorded_posX, ChColor(0.f, 1.f, 1.f), true);
//
//
//
//        sys.DoStepDynamics(step_size);
//
//        vis->EndScene();
//
//        // Perform the integration step
//
//        
//        // Spin in place to maintain soft real-time
//        realtime_timer.Spin(step_size);
//    }
//        
//return 0;
//        
//}








/////////////////////////////////////////////////// MAIN FOR 6 DOF EXAMPLE WITH CAD IMPORT ///////////////////////////////////////

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a Chrono physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;
    sys.SetCollisionSystemType(collision_type);
    sys.SetSolverType(ChSolver::Type::MINRES);
    sys.GetSolver()->AsIterative()->SetMaxIterations(1000);
    ChRealtimeStepTimer realtime_timer;
    double timestep = 0.01;


 







    ////////////////////////////////// 6 DOF EXAMPLE /////////////////////////////////////

    //////////////////////////////// COLLISION FAMILIES //////////////////////////
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
    int tazza_family = 10;
    int industrial_conveyor_1_family = 11;
    int box_collision_family = 12;
    int industrial_shelf_1_family = 13;
    int industrial_base_family = 14;
    int robot_gripper_family = 15;


    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    auto groundBody = chrono_types::make_shared<ChBodyEasyBox>(5, 0.5, 5, 3000, material);
    groundBody->SetPos(ChVector3d(0.0, -0.7/*-0.3*/, 0.0));
    groundBody->SetFixed(true);
    groundBody->GetCollisionModel()->SetFamily(obstacles_collision_family);
    groundBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(groundBody);
    /////////////////////////////// OBSTACLE FOR UR5e (BLOCKS) //////////////////////////
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
/////////////////////////////// END OBSTACLE FOR UR5e_2////////////////////////

///////////////////////////////////////////////////////////////////////////// IMPORT CAD //////////////////////////////



//Import CAD
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>bodylist;
    std::vector<std::shared_ptr<chrono::ChLinkBase>>linklist;
    ImportSolidworksSystemCpp(bodylist, linklist);
         /////////////////////////// UR5 ////////////////
        for (auto& body : bodylist) {
            if (body->GetCollisionModel()) {
                //    body->GetCollisionModel()->SetFamily(robot_collision_family);
                //    body->GetCollisionModel()->DisallowCollisionsWith(robot_collision_family);
                //    //body->GetCollisionModel()->DisallowCollisionsWith(obstacles_collision_family);
                body->GetCollisionModel()->SetEnvelope(0.002);
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
                }
                if (body->GetName() == "start_new-1") {
                    body->GetCollisionModel()->SetFamily(obstacles_collision_family);
                    body->GetCollisionModel()->DisallowCollisionsWith(robot_base_family);
                    body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
                    body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
                    body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
                    body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
                    body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family);
                    body->GetCollisionModel()->AllowCollisionsWith(robot_forearm_family);
                }
                //if (body->GetName() == "industrial_shelf-1") {
                //    body->GetCollisionModel()->SetFamily(industrial_shelf_1_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_biceps_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_2_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(box_collision_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_end_effector_family);

                //}
                //if (body->GetName() == "industrial_shelf-2") {
                //    body->GetCollisionModel()->SetFamily(industrial_shelf_1_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(obstacles_collision_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_base_family);
                //    body->GetCollisionModel()->DisallowCollisionsWith(robot_biceps_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_shoulder_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
                //    body->GetCollisionModel()->AllowCollisionsWith(box_collision_family);

                //}
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
                    body->GetCollisionModel()->AllowCollisionsWith(tazza_family);
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
                    body->GetCollisionModel()->AllowCollisionsWith(robot_wrist_family);
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
                    body->GetCollisionModel()->AllowCollisionsWith(robot_elbow_family);
                    body->GetCollisionModel()->AllowCollisionsWith(industrial_shelf_1_family);
                    body->GetCollisionModel()->AllowCollisionsWith(tazza_family);
                    body->GetCollisionModel()->DisallowCollisionsWith(robot_gripper_family);
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //// Definizione del problema: start, goal e area esplorata
    ChPlanModel planModel(&sys);
    ChPlanner planner(&planModel);
    ///////////////////////////////////
    ChPlanner planner2(&planModel);
    ChPlanner planner3(&planModel);
    ChPlanner planner4(&planModel);
    /////////////////////////////////////
    ChVectorDynamic<> nmotors(6);
    ChVectorDynamic<> startPos(nmotors);
    ChVectorDynamic<> goalPos(nmotors);
    ///////////////////////////////////////////
    ChVectorDynamic<> goalPos2(nmotors);
    ChVectorDynamic<> goalPos3(nmotors);
    ChVectorDynamic<> goalPos_1(nmotors);
    //////////////////////////////////////////

    //////////////////////////////////////////////// start and goal position //////////////////////////////////////////   
    //startPos[0] = robot.GeMotorBaseShoulder()->GetAngleOffset();
    //std::cout << "Start: " << startPos[0] << std::endl;

    //startPos[1] = robot.GeMotorShoulderBiceps()->GetAngleOffset();
    //std::cout << "Start: " << startPos[1] << std::endl;

    //startPos[2] = robot.GeMotorBicepsElbow()->GetAngleOffset();
    //std::cout << "Start: " << startPos[2] << std::endl;

    //startPos[3] = robot.GeMotorElbowForearm()->GetAngleOffset();
    //std::cout << "Start: " << startPos[3] << std::endl;

    //startPos[4] = robot.GeMotorForearmWrist()->GetAngleOffset();
    //std::cout << "Start: " << startPos[4] << std::endl;

    //startPos[5] = robot.GeMotorWristEndeffector()->GetAngleOffset();
    //std::cout << "Start: " << startPos[5] << std::endl;

    //startPos[0] = -1.5708;
    //startPos[1] = 0.0;
    //startPos[2] = 0.0;
    //startPos[3] = -1.5708;
    //startPos[4] = 0.0;
    //startPos[5] = 0.0;

    //this for env3
    startPos[0] = 1.60;
    startPos[1] = -0.35;
    startPos[2] = 1.15;
    startPos[3] = 3.2;
    startPos[4] = -1.45;
    startPos[5] = 0.0;


    //goalPos[0] = -1.74467;
    //goalPos[1] = -1.00408;
    //goalPos[2] = -0.426268;
    //goalPos[3] = -1.74299;
    //goalPos[4] = -3.11737;
    //goalPos[5] = -1.43244;

    //COMAU RACER
    //goalPos[0] = -2.00306;                         //conweyor first goal
    //goalPos[1] = -0.502838;
    //goalPos[2] = 0.37353;
    //goalPos[3] = 1.51137;
    //goalPos[4] = /*-0.428415*/0.0;
    //goalPos[5] = 2.99935;

    //UR5e goal
    //goalPos[0] = -2.92067;                        
    //goalPos[1] = 1.21734;
    //goalPos[2] = 1.11218;
    //goalPos[3] = 3.03643;
    //goalPos[4] = 0.220921;
    //goalPos[5] = -1.47201;

    //UR5e GOAL 1 shelf
    //goalPos[0] = 0.247047;
    //goalPos[1] = -0.650365;
    //goalPos[2] = 0.663831;
    //goalPos[3] = 1.3142;
    //goalPos[4] = 2.89455;
    //goalPos[5] = 1.66958;

    ////////////////////////////////
    //goalPos2[0] = -2.39944;
    //goalPos2[1] = -0.526721;
    //goalPos2[2] = 0.519458;
    //goalPos2[3] = -2.08163;
    //goalPos2[4] = 0.738725;
    //goalPos2[5] = -1.48671;

    //goalPos3[0] = 0.0;
    //goalPos3[1] = 0.0;
    //goalPos3[2] = 0.0;
    //goalPos3[3] = 0.0;
    //goalPos3[4] = 0.0;
    //goalPos3[5] = 0.0;
    ///////////////////////////////////



    //env3

    //this
    //goalPos[0] = -0.6;  //0.7
    //goalPos[1] = -1.64;
    //goalPos[2] = /*0.97092666*//*1.15*/1.70;
    //goalPos[3] = 8.5;
    //goalPos[4] = -1.05;
    //goalPos[5] = 4.53593619;




    //this
    //goalPos_1[0] = -0.9;  //0.7
    //goalPos_1[1] = -2.1;
    //goalPos_1[2] = /*0.97092666*//*1.15*/1.3;
    //goalPos_1[3] = 8.3;
    //goalPos_1[4] = -1.05;
    //goalPos_1[5] = 4.53593619;




    
    //goalPos2[0] = -1.1;
    //goalPos2[1] = -1.8;
    //goalPos2[2] = /*0.97092666*//*1.15*/1.79;
    //goalPos2[3] = 7.8;
    //goalPos2[4] = -1.4;
    //goalPos2[5] = 4.53593619;

    //this
    //goalPos2[0] = -1.1;
    //goalPos2[1] = -1.7;
    //goalPos2[2] = /*0.97092666*//*1.15*/1.79;
    //goalPos2[3] = 8.3;
    //goalPos2[4] = -1.1;
    //goalPos2[5] = 4.53593619;

    //try for 4 goal
    //goalPos2[0] = -1.3;
    //goalPos2[1] = -1.8;
    //goalPos2[2] = /*0.97092666*//*1.15*/1.79;
    //goalPos2[3] = 7.5;
    //goalPos2[4] = -1.5;
    //goalPos2[5] = 4.53593619;
    //
    //
    ////this
    goalPos3[0] = -1.4;
    goalPos3[1] = -1.9;
    goalPos3[2] = /*0.97092666*//*1.15*/0.52;
    goalPos3[3] = 7.0;
    goalPos3[4] = -1.55;
    goalPos3[5] = 3.8;


    std::vector<std::shared_ptr<ChJoint>> jointVect;

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

    //planner.setStart(&startPos);
    //planner.setGoal(&goalPos);
    ////////////////////////////////
    //planner2.setStart(&goalPos);
    //planner2.setGoal(&goalPos2);

    //planner3.setStart(&goalPos2);
    //planner3.setGoal(&goalPos3);
    //////////////////////////////////
    //bool isVerified = planner.verify();
    //std::cout << "Configuration Verified: " << isVerified << std::endl;
    //if (!isVerified) {
    //    std::cerr << "Start or Goal configuration are not valid." << std::endl;
    //    return -1;
    //}
    ////////////////////////////////////
    ////////////////////////////////////////
    std::vector<ChVectorDynamic<>>  optim_path;
    std::vector<ChVectorDynamic<>>  reducedpath;
    std::vector<ChVectorDynamic<>>  optimized_path;
    std::vector<ChVectorDynamic<>>  optimized_path2;
    std::vector<CubeNode> cube_foam;
    ///////////////////////////////////////////////////
    std::vector<ChVectorDynamic<>>  optim_path2;
    std::vector<ChVectorDynamic<>>  reducedpath2;
    std::vector<ChVectorDynamic<>>  optimized_path2_2;
    //std::vector<ChVectorDynamic<>> optimized_path2;

    std::vector<ChVectorDynamic<>>  optim_path3;
    std::vector<ChVectorDynamic<>>  reducedpath3;
    std::vector<ChVectorDynamic<>>  optimized_path3;
    std::vector<ChVectorDynamic<>>  optimized_path2_3;

    std::vector<ChVectorDynamic<>>  optim_path4;
    std::vector<ChVectorDynamic<>>  reducedpath4;
    std::vector<ChVectorDynamic<>>  optimized_path4;
    ///////////////////////////////////////////////////
    int max_iter1 = 200000;


    if (write_mode) {
        
        /////////////////////////////////////
        optim_path = GoalBiasedProbabilisticFoamCuboidConnect(startPos, goalPos3, &planModel, sys, 0.001, max_iter1);

        std::cout << "Foam size: " << optim_path.size() << std::endl;

        reducedpath = shortcutOptimize(optim_path, &planModel, 50, 50, 1.0);
        std::cout << "Reduced path size: " << reducedpath.size() << std::endl;

        optimized_path = PartialShortcut(reducedpath/*optim_path*/, &planModel, 250);
        std::cout << "Optimized path size: " << optimized_path.size() << std::endl;

        auto final_channel_path = BuildChannelCuboids(optimized_path, &planModel, sys, 0.01, 0.2);
        SaveCuboidsFoamToJson(final_channel_path, "optimized_channel_cubes_data_partial_shortcut.json");


        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



        std::ofstream file1("optim_path_6dof.json");
        ChArchiveOutJSON archive1(file1);
        archive1 << CHNVP(optim_path);


        file1.flush();

    }
    else {


        std::ifstream file1("optim_path_6dof.json");
        ChArchiveInJSON archive1(file1);
        archive1 >> CHNVP(optim_path);

    }



    if (write_mode2) {
        
        std::ofstream file("optimized_path_6dof.json");
        ChArchiveOutJSON archive(file);
        archive << CHNVP(optimized_path);
        file.flush();
     
    }
    else {
        std::ifstream file("optimized_path_6dof.json");
        ChArchiveInJSON archive(file);
        archive >> CHNVP(optimized_path);

    }


    std::cout << optim_path.size() << std::endl;
    std::vector<ChVector3d> recorded_pos_base;
    std::vector<ChVector3d> recorded_pos_shoulder;
    std::vector<ChVector3d> recorded_pos_biceps;
    std::vector<ChVector3d> recorded_pos_elbow;
    std::vector<ChVector3d> recorded_pos_forearm;
    std::vector<ChVector3d> recorded_pos_wrist;
    std::vector<ChVector3d> recorded_pos_end_effector;


    //////////////////////////////////////////////////////////////////////

    for (const auto& cfg : /*new_optim_path*/optimized_path/*optim_path*//*new_path*//*reducedpath*/) {
        std::cout << "cfg: " << cfg << " | ";
        planModel.setPosition(cfg);

        sys.DoAssembly(AssemblyLevel::POSITION, 10);

        recorded_pos_base.push_back(robot.GetBase()->GetPos());
        recorded_pos_shoulder.push_back(robot.GetShoulder()->GetPos());
        recorded_pos_biceps.push_back(robot.GetBiceps()->GetPos());
        recorded_pos_elbow.push_back(robot.GetElbow()->GetPos());
        recorded_pos_forearm.push_back(robot.GetForearm()->GetPos());
        recorded_pos_wrist.push_back(robot.GetWrist()->GetPos());
        recorded_pos_end_effector.push_back(robot.GetEndEffector()->GetPos());
        bool isColliding = planModel.isColliding(cfg);
        bool isValid = planModel.isValid(cfg);
    }


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


    double total_time = 20;
    double total_length = *segment_lengths.rbegin();
    std::cout << "Total length: " << total_length << std::endl;
    std::vector<ChFunctionInterp> interpJoints;
    interpJoints.resize(numjoints);

    for (size_t optpoint = 0; optpoint < /*new_optim_path*/optimized_path/*optim_path*//*new_path*//*reducedpath*/.size(); ++optpoint) {
        for (size_t joint = 0; joint < numjoints; ++joint) {

            interpJoints[joint].AddPoint(segment_lengths[optpoint] / total_length, /*new_optim_path*/optimized_path/*optim_path*//*new_path*//*reducedpath*/[optpoint][joint], true);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // 4 - Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("CartesianRobot");
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0.652997, 0.865272, -1.87558));

    vis->AddTypicalLights();
    vis->AddLightWithShadow(ChVector3d(20.0, 35.0, 25.0), ChVector3d(0.0, 0.0, 0.0), 55, 20, 55, 35, 512,
        ChColor(0.6f, 0.8f, 1.0f));
    vis->EnableShadows();
    vis->EnableCollisionShapeDrawing(false);

    // 5 - Simulation loop
    //ChRealtimeStepTimer realtime_timer;
    double step_size = 5e-3;

    bool firstMovementCompleted = false;
    bool secondMovementCompleted = false;
    bool thirdMovementCompleted = false;


    ChTimer tim;
    tim.reset();
    tim.start();
    

    while (vis->Run()) {
        // Render scene
        vis->BeginScene();
        vis->Render();
       
        /////////////////////////////////////
        //if (!firstMovementCompleted) {
            ///////////////////////////////
            ChVectorDynamic<> curcfg(numjoints);
            for (size_t j = 0; j < numjoints; ++j) {
                curcfg[j] = interpJoints[j].GetVal(tim.GetTimeMilliseconds() / 1000.0 / total_time);
            }
            planModel.setPosition(curcfg);

            ///////////////////////////////////////////////////////
        //    if ((tim.GetTimeMilliseconds() / 1000.0 >= total_time)) {
        //        firstMovementCompleted = true;
        //        tim.reset();
        //        tim.start();

        //    }
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
        //else if (!secondMovementCompleted) {
        //    ChVectorDynamic<> curcfg3(numjoints);
        //    for (size_t j = 0; j < numjoints; ++j) {
        //        curcfg3[j] = interpJoints3[j].GetVal(tim.GetTimeMilliseconds() / 1000.0 / total_time3);
        //    }
        //    planModel.setPosition(curcfg3);

        //    if ((tim.GetTimeMilliseconds() / 1000.0 >= total_time3)) {
        //        secondMovementCompleted = true;
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
            //////////////////////////////////////////////////////////////////////
            //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_base, ChColor(1.f, 1.f, 0.f), true);
            //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_shoulder, ChColor(1.f, 1.f, 0.f), true);
            //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_biceps, ChColor(1.f, 1.f, 0.f), true);
            //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_elbow, ChColor(1.f, 1.f, 0.f), true);
            //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_forearm, ChColor(1.f, 1.f, 0.f), true);
            //irrlicht::tools::drawPolyline(vis.get(), recorded_pos_wrist, ChColor(1.f, 1.f, 0.f), true);
            irrlicht::tools::drawPolyline(vis.get(), recorded_pos_end_effector, ChColor(1.f, 0.f, 1.f), true);      // alpha1 = 0.95, alpha2 = 0.05
            //saveTrajectoryToFile(recorded_pos_end_effector, "trajectory_1001_ompl_shelves_prm.json");
            //std::vector<ChVector3d> loaded_trajectory1 = loadTrajectoryFromFile("trajectory_1001_1_0_shelves.json");                // alpha1 = 1, alpha2 = 0.0
            //irrlicht::tools::drawPolyline(vis.get(), loaded_trajectory1, ChColor(0.f, 1.f, 0.f), true);   //green

            //std::vector<ChVector3d> loaded_trajectory095_005 = loadTrajectoryFromFile("trajectory_1001_095_005_shelves.json");                // alpha1 = 0.95, alpha2 = 0.05
            //irrlicht::tools::drawPolyline(vis.get(), loaded_trajectory095_005, ChColor(1.f, 1.f, 0.f), true);


            //std::vector<ChVector3d> loaded_trajectory0998_0002 = loadTrajectoryFromFile("trajectory_1001_0975_0025_shelves.json");                // alpha1 = 0.95, alpha2 = 0.05
            //irrlicht::tools::drawPolyline(vis.get(), loaded_trajectory0998_0002, ChColor(1.f, 0.f, 1.f), true);
            ///////////////////////////////////////////////////////
            
            //planModel.setPosition(goalPos);
            
            sys.DoStepDynamics(step_size);

            vis->EndScene();

            // Perform the integration step


            // Spin in place to maintain soft real-time
            realtime_timer.Spin(step_size);
        }
    
    return 0;

}
