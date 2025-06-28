// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: C:\Users\ricca\Desktop\comau_racer_obstacle\UR5e\ASSIEME_finale_UR5e_2.SLDASM


#include <string>
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "UR5e.h"


/// Function to import Solidworks assembly directly into Chrono ChSystem.
void ImportSolidworksSystemCpp(chrono::ChSystem& system, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;
std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;
ImportSolidworksSystemCpp(bodylist, linklist, motfun_map);
for (auto& body : bodylist)
    system.Add(body);
for (auto& link : linklist)
    system.Add(link);
}


/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& bodylist, std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {

// Some global settings
double sphereswept_r = 0.001;
chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.003);
chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(0.002);

std::string shapes_dir = "UR5e_shapes/";

// Prepare some data for later use
std::shared_ptr<chrono::ChVisualShapeModelFile> body_shape;
chrono::ChMatrix33<> mr;
std::shared_ptr<chrono::ChLinkBase> link;
chrono::ChVector3d cA;
chrono::ChVector3d cB;
chrono::ChVector3d dA;
chrono::ChVector3d dB;

// Assembly ground body
auto body_0 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_0->SetName("SLDW_GROUND");
body_0->SetFixed(true);
bodylist.push_back(body_0);

// Rigid body part
auto body_1 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_1->SetName("Link2_UR5_STEP-1");
body_1->SetPos(chrono::ChVector3d(0,0,0));
body_1->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_1->SetMass(13.4869578178459);
body_1->SetInertiaXX(chrono::ChVector3d(0.446831193472711,0.0363743847510672,0.440722635300511));
body_1->SetInertiaXY(chrono::ChVector3d(-1.77024123615327e-06,2.28317030040725e-07,0.000235444714445938));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-6.15152218940188e-07,0.366921489597504,0.14411017605433),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_1->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_1 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_1;
mr(0,0)=0; mr(1,0)=0; mr(2,0)=-1;
mr(0,1)=0; mr(1,1)=-1; mr(2,1)=0;
mr(0,2)=-1; mr(1,2)=0; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.141822171420531,0.58787819687177,0.12);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0,0.364109885978337,0.146268056886561), mr));
body_1->EnableCollision(true);

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("Link1_UR5_STEP-1");
body_2->SetPos(chrono::ChVector3d(0,0,0));
body_2->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_2->SetMass(5.30876614321518);
body_2->SetInertiaXX(chrono::ChVector3d(0.0307324141847837,0.0119449603656888,0.0303338138457506));
body_2->SetInertiaXY(chrono::ChVector3d(-0.00047153765263149,-0.000167981577263375,-0.000469095473076001));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.00957883477798284,0.141272822478325,0.00330414209848852),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_2->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_2 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_2;
chrono::ChVector3d p1_body_2(-0.0137401657281089,0.264,0);
chrono::ChVector3d p2_body_2(-0.0137401657281089,0,0);
body_2->GetCollisionModel()->AddCylinder(mat_2,0.0668003634838783,p1_body_2,p2_body_2);
body_2->EnableCollision(true);

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("Link6_UR5_STEP-1");
body_3->SetPos(chrono::ChVector3d(-8.44661016281028e-16,1.07945,0.232900000000001));
body_3->SetRot(chrono::ChQuaternion<>(6.08546238857962e-17,0.741156669705821,0.671332101832303,9.36224982858403e-17));
body_3->SetMass(0.531572472308725);
body_3->SetInertiaXX(chrono::ChVector3d(0.000345921264373931,0.000345224158878349,0.000469325826357456));
body_3->SetInertiaXY(chrono::ChVector3d(-6.97567041937565e-08,1.69646723943244e-07,-1.68194479350155e-08));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(4.93732275760031e-10,7.40815735846451e-06,0.0260552428320294),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_3->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_3 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_3;
chrono::ChVector3d p1_body_3(4.08006961549779E-18,-9.59302082215214E-18,0.049);
chrono::ChVector3d p2_body_3(0,0,0);
body_3->GetCollisionModel()->AddCylinder(mat_3,0.0450659241943092,p1_body_3,p2_body_3);
body_3->EnableCollision(true);

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("Link3_UR5_STEP-1");
body_4->SetPos(chrono::ChVector3d(2.43149693754354e-15,0.9355,0.00659999999999983));
body_4->SetRot(chrono::ChQuaternion<>(0.500000000000001,0.499999999999999,-0.499999999999999,-0.500000000000001));
body_4->SetMass(10.2485241510182);
body_4->SetInertiaXX(chrono::ChVector3d(0.216745059208228,0.21803542984154,0.0177569102851551));
body_4->SetInertiaXY(chrono::ChVector3d(-1.14520746177216e-05,-0.00419453180122824,-1.13634753356105e-05));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.164536440839439,0.00184603725784859,-4.13236819091921e-06),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_4_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_4->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_4 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_4;
mr(0,0)=0; mr(1,0)=1.26161707343768E-16; mr(2,0)=-1;
mr(0,1)=0; mr(1,1)=-1; mr(2,1)=-1.20676415720126E-16;
mr(0,2)=-1; mr(1,2)=0; mr(2,2)=0;
collshape_4 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_4,0.11,0.115,0.513);
body_4->GetCollisionModel()->AddShape(collshape_4,chrono::ChFramed(chrono::ChVector3d(0.1615,1.38777878078145E-17,-6.93889390390723E-18), mr));
body_4->EnableCollision(true);

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("Link4_UR5_STEP_1000515-1");
body_5->SetPos(chrono::ChVector3d(0.508789924723371,0.881299999999981,4.40619762898109e-16));
body_5->SetRot(chrono::ChQuaternion<>(0.500000000000016,8.85256044027966e-16,2.09952851344052e-16,0.86602540378443));
body_5->SetMass(1.98715438806532);
body_5->SetInertiaXX(chrono::ChVector3d(0.00374494513363848,0.00405290094593909,0.00333128550628577));
body_5->SetInertiaXY(chrono::ChVector3d(-0.000266854238789509,-8.58909661690962e-05,-4.98247245716833e-05));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.332131640132098,0.395740681286961,0.12723374477465),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_5_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_5->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_5 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_5;
mr(0,0)=-4.86759381965911E-16; mr(1,0)=0; mr(2,0)=-1;
mr(0,1)=-0.499999999999968; mr(1,1)=-0.866025403784457; mr(2,1)=0;
mr(0,2)=-0.866025403784457; mr(1,2)=0.499999999999968; mr(2,2)=4.21545990312901E-16;
collshape_5 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_5,0.114042283082579,0.100474309500103,0.116);
body_5->GetCollisionModel()->AddShape(collshape_5,chrono::ChFramed(chrono::ChVector3d(0.329522666139989,0.397250000000015,0.1333), mr));
body_5->EnableCollision(true);

bodylist.push_back(body_5);



// Rigid body part
auto body_6 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_6->SetName("Link5_UR5_STEP-1");
body_6->SetPos(chrono::ChVector3d(0.508789924723371,0.881299999999981,1.25561459484425e-15));
body_6->SetRot(chrono::ChQuaternion<>(0.500000000000016,4.32083215997192e-17,-1.4123247291576e-16,0.86602540378443));
body_6->SetMass(1.26653751294046);
body_6->SetInertiaXX(chrono::ChVector3d(0.00173387706481413,0.00179202686878704,0.00129124808161356));
body_6->SetInertiaXY(chrono::ChVector3d(-5.07900554045803e-05,-1.60832333990906e-05,-9.38652107154482e-06));
body_6->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.423155408864203,0.343195767732578,0.127539242186587),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_6_1.obj");
body_6->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_6->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_6 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_6;
chrono::ChVector3d p1_body_6(0.425997896121578,0.341550000000018,0.0716);
chrono::ChVector3d p2_body_6(0.425997896121578,0.341550000000018,0.1796);
body_6->GetCollisionModel()->AddCylinder(mat_6,0.0462646642673956,p1_body_6,p2_body_6);
body_6->EnableCollision(true);

bodylist.push_back(body_6);



// Rigid body part
auto body_7 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_7->SetName("Base_UR5_STEP-1");
body_7->SetPos(chrono::ChVector3d(0,0,0));
body_7->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_7->SetMass(4.80160121153642);
body_7->SetInertiaXX(chrono::ChVector3d(0.0275494062842208,0.0386615054177464,0.0199299189225059));
body_7->SetInertiaXY(chrono::ChVector3d(5.14955818071587e-07,6.4511125414673e-08,-0.00127937473743534));
body_7->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.27760013272622e-06,0.0531074916233878,-0.00611136468161377),chrono::ChQuaternion<>(1,0,0,0)));
body_7->SetFixed(true);

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_7_1.obj");
body_7->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_7->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_7 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_7;
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=-1; mr(2,2)=0;
collshape_7 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_7,0.209088419740169,0.18379402801915,0.105);
body_7->GetCollisionModel()->AddShape(collshape_7,chrono::ChFramed(chrono::ChVector3d(0,0.0525,0), mr));
body_7->EnableCollision(true);

bodylist.push_back(body_7);




// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("MARKER_1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-5.8418494793746E-16,0.0991,-1.05344719486481E-16),chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,4.13251699778365E-17,-4.13251699778365E-17)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("MARKER_2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(4.64239139559248E-16,0.1625,0.0728162915038653),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("MARKER_3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(2.38977494309316E-15,0.5875,0.0744),chrono::ChQuaternion<>(5.94981804072146E-17,5.25968142818241E-34,1,-8.840071061307E-18)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("MARKER_4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-2.15317939407874E-15,0.97975,0.0523000000000013),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_5 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_5->SetName("MARKER_5");
body_0->AddMarker(marker_0_5);
marker_0_5->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-8.88178419700125E-16,1.02605,0.133300000000001),chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,-1.96261557335476E-17,1.96261557335476E-17)));

// Auxiliary marker (coordinate system feature)
auto marker_0_6 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_6->SetName("MARKER_6");
body_0->AddMarker(marker_0_6);
marker_0_6->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-7.7715611723761E-16,1.07945,0.179600000000001),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_7 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_7->SetName("MARKER_TCP");
body_0->AddMarker(marker_0_7);
marker_0_7->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-8.44661016281028E-16,1.07945,0.232900000000001),chrono::ChQuaternion<>(1,0,0,0)));


} // end function
