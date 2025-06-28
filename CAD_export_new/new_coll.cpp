// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: C:\Users\ricca\Desktop\comau_racer_obstacle\UR5e\Ur5e_final.SLDASM


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
#include "new_coll.h"


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

std::string shapes_dir = "CAD_export_new/new_coll_shapes/";

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
body_1->SetName("Link3_UR5_STEP-1");
body_1->SetPos(chrono::ChVector3d(-0.914273342121355,1.71448489258473,0.872812395774357));
body_1->SetRot(chrono::ChQuaternion<>(0.705218772907502,0.0806743126378385,-0.051637993172058,-0.702489612222424));
body_1->SetMass(8.08235723122829);
body_1->SetInertiaXX(chrono::ChVector3d(0.165825680683714,0.0163631628632252,0.161394315337931));
body_1->SetInertiaXY(chrono::ChVector3d(0.00588849810557492,-0.00116396557415683,0.028609088113872));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.17770116613023,0.00383166700323644,-0.000120751085736446),chrono::ChQuaternion<>(1,0,0,0)));

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
chrono::ChVector3d p1_body_1(0.348,-0.0482,-2.25486296301367E-16);
chrono::ChVector3d p2_body_1(0.348,0.0678000000000002,2.22044604925032E-16);
body_1->GetCollisionModel()->AddCylinder(mat_1,0.065,p1_body_1,p2_body_1);
chrono::ChVector3d p3_body_1(-0.0442500000000001,-0.0566999999999987,-0.00159733932179073);
chrono::ChVector3d p4_body_1(-0.0442500000000001,0.0463000000000015,-0.00159733932179034);
body_1->GetCollisionModel()->AddCylinder(mat_1,0.0425,p3_body_1,p4_body_1);
chrono::ChVector3d p5_body_1(-0.00900000000000017,1.47451495458029E-17,-9.18675695743908E-16);
chrono::ChVector3d p6_body_1(0.271,1.47451495458029E-17,-9.18675695743908E-16);
body_1->GetCollisionModel()->AddCylinder(mat_1,0.05,p5_body_1,p6_body_1);
body_1->EnableCollision(true);

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("start_new-1");
body_2->SetPos(chrono::ChVector3d(-1.15241262841672,0.147142372501616,0.791527490034647));
body_2->SetRot(chrono::ChQuaternion<>(0.702798898660086,-0.702798898660086,-0.0779339979865664,-0.0779339979865664));
body_2->SetMass(777.061473540029);
body_2->SetInertiaXX(chrono::ChVector3d(82.9549016957675,61.1658753618951,109.684183095185));
body_2->SetInertiaXY(chrono::ChVector3d(5.23041424133356,0.512870888573728,-1.4175668823672));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.0015247067577785,-0.00165382711763383,0.111237734465079),chrono::ChQuaternion<>(1,0,0,0)));
body_2->SetFixed(true);

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
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=1.23389597017419E-16; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.148139846741344,0.168706943007384,0.22);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(0.0574348581100566,-0.303,0.579353471503692), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=1.24881578192201E-16;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.889020655165368,0.299196711450916,0.567);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(-0.000702831722705244,0.00600000000000006,-0.200401644274542), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.796680903303464,0.587968632112918,0.463);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(-0.00849427483404774,0.0035,0.293984316056459), mr));
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=-1; mr(2,2)=0;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.12078700493303,0.156583617578014,0.393);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(0.00910649753348487,0.0315,0.688291808789007), mr));
body_2->EnableCollision(true);

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("baseUR5e-1");
body_3->SetPos(chrono::ChVector3d(-0.914818230514467,0.757142372501616,0.840265586893949));
body_3->SetRot(chrono::ChQuaternion<>(0.993907734105968,-3.12755571846569e-18,-0.11021531692256,-7.32824405996053e-18));
body_3->SetMass(0.390700550874966);
body_3->SetInertiaXX(chrono::ChVector3d(0.00114309022827863,0.0022590006317847,0.0011443877363986));
body_3->SetInertiaXY(chrono::ChVector3d(-1.86758947993233e-08,3.06815804307727e-07,-8.31729306897557e-08));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(1.13108935270075e-17,0.00883215517656131,1.04958112140664e-05),chrono::ChQuaternion<>(1,0,0,0)));
body_3->SetFixed(true);

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("End_effector_assembly2-1");
body_4->SetPos(chrono::ChVector3d(-0.734404197595021,1.84771345569393,0.893683116829515));
body_4->SetRot(chrono::ChQuaternion<>(0.581610298112794,0.329130187082193,-0.690592010879158,-0.276560039756775));
body_4->SetMass(1.25092539405811);
body_4->SetInertiaXX(chrono::ChVector3d(0.00982600526271352,0.00984171448495844,0.00138395531290137));
body_4->SetInertiaXY(chrono::ChVector3d(-3.98695143803727e-05,-7.92856694741739e-05,0.000101288856527705));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.0068723604586369,0.0179620112359557,-0.0971096380965326),chrono::ChQuaternion<>(1,0,0,0)));

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
chrono::ChVector3d p1_body_4(0.00397817890081981,0.0216916002321598,-0.375082667793707);
chrono::ChVector3d p2_body_4(0.00397817890081981,0.0216916002321598,0.00791733220629359);
body_4->GetCollisionModel()->AddCylinder(mat_4,0.0625,p1_body_4,p2_body_4);
body_4->EnableCollision(true);

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("Link4_UR5_STEP_1000515-1");
body_5->SetPos(chrono::ChVector3d(-0.825439633185542,1.68785595180735,0.369003787663318));
body_5->SetRot(chrono::ChQuaternion<>(0.366937595612014,0.56078970071398,0.303886027994261,0.677144736736151));
body_5->SetMass(1.63038930409274);
body_5->SetInertiaXX(chrono::ChVector3d(0.00258396413104162,0.00241155539459893,0.00258281431379276));
body_5->SetInertiaXY(chrono::ChVector3d(-0.00033478659628426,0.00014728996358411,-0.000159736895660383));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.335801640555169,0.39362114302927,0.118242292293933),chrono::ChQuaternion<>(1,0,0,0)));

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
mr(0,0)=-5.04646829375071E-16; mr(1,0)=0; mr(2,0)=-1;
mr(0,1)=-0.499999999999969; mr(1,1)=-0.866025403784456; mr(2,1)=0;
mr(0,2)=-0.866025403784456; mr(1,2)=0.499999999999969; mr(2,2)=4.37036974178092E-16;
collshape_5 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_5,0.11,0.0850000000000001,0.104);
body_5->GetCollisionModel()->AddShape(collshape_5,chrono::ChFramed(chrono::ChVector3d(0.334718818562696,0.394250000000015,0.12045), mr));
body_5->EnableCollision(true);

bodylist.push_back(body_5);



// Rigid body part
auto body_6 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_6->SetName("Base_UR5_STEP-1");
body_6->SetPos(chrono::ChVector3d(-0.914818230514467,0.779642372501616,0.840265586893949));
body_6->SetRot(chrono::ChQuaternion<>(0.780732896646653,-1.77675850862547e-18,0.624864900673519,-2.13693318283948e-17));
body_6->SetMass(3.22413490953495);
body_6->SetInertiaXX(chrono::ChVector3d(0.00920548772785535,0.0230003210560511,0.0190350440589591));
body_6->SetInertiaXY(chrono::ChVector3d(0.00115869838989485,0.00232431499043948,-0.000259684885646951));
body_6->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(3.39195718030387e-06,0.0499745884667967,-0.00910146035533363),chrono::ChQuaternion<>(1,0,0,0)));
body_6->SetFixed(true);

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
mr(0,0)=1; mr(1,0)=0; mr(2,0)=-1.73472347597681E-16;
mr(0,1)=8.67361737988404E-17; mr(1,1)=0; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=-1; mr(2,2)=0;
collshape_6 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_6,0.16,0.16,0.096);
body_6->GetCollisionModel()->AddShape(collshape_6,chrono::ChFramed(chrono::ChVector3d(-6.93889390390723E-18,0.048,2.77555756156289E-17), mr));
body_6->EnableCollision(true);

bodylist.push_back(body_6);



// Rigid body part
auto body_7 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_7->SetName("Link2_UR5_STEP-1");
body_7->SetPos(chrono::ChVector3d(-0.913569625239967,0.779780587878785,0.833682131953119));
body_7->SetRot(chrono::ChQuaternion<>(0.770010345469714,0.0131535793983151,0.637698039659849,0.0158827400850448));
body_7->SetMass(10.3747911511588);
body_7->SetInertiaXX(chrono::ChVector3d(0.345888699745002,0.0224426175176436,0.349268449292396));
body_7->SetInertiaXY(chrono::ChVector3d(-0.0136048376169236,0.000553547539184782,1.93766980203433e-05));
body_7->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-7.99649674217208e-07,0.376650266336969,0.139531192215326),chrono::ChQuaternion<>(1,0,0,0)));

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
chrono::ChVector3d p1_body_7(-2.8167836289516E-16,0.2091,0.1378);
chrono::ChVector3d p2_body_7(1.37236539871527E-16,0.5531,0.1378);
body_7->GetCollisionModel()->AddCylinder(mat_7,0.055,p1_body_7,p2_body_7);
chrono::ChVector3d p3_body_7(2.53911659356505E-15,0.1625,0.2114);
chrono::ChVector3d p4_body_7(2.51837380209101E-15,0.1625,0.0744);
body_7->GetCollisionModel()->AddCylinder(mat_7,0.0650000000000029,p3_body_7,p4_body_7);
chrono::ChVector3d p5_body_7(2.3735914380224E-15,0.5875,0.2104);
chrono::ChVector3d p6_body_7(2.38977494309316E-15,0.5875,0.0744);
body_7->GetCollisionModel()->AddCylinder(mat_7,0.065,p5_body_7,p6_body_7);
body_7->EnableCollision(true);

bodylist.push_back(body_7);



// Rigid body part
auto body_8 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_8->SetName("Link1_UR5_STEP-1");
body_8->SetPos(chrono::ChVector3d(-0.914818230514467,0.779642372501616,0.840265586893948));
body_8->SetRot(chrono::ChQuaternion<>(0.77017413197471,-3.15988824154224e-17,0.637833682425914,-1.31464436092246e-16));
body_8->SetMass(4.94428012662587);
body_8->SetInertiaXX(chrono::ChVector3d(0.0250041481465757,0.0109158603581209,0.0255680330460476));
body_8->SetInertiaXY(chrono::ChVector3d(0.000600282536190405,0.000110894050527914,-0.000113846742614755));
body_8->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-6.7802427408491e-08,0.133183875072103,0.00354771921911234),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_8_1.obj");
body_8->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_8->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_8 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_8;
chrono::ChVector3d p1_body_8(0,0.238,0);
chrono::ChVector3d p2_body_8(0,0,0);
body_8->GetCollisionModel()->AddCylinder(mat_8,0.0668003634838783,p1_body_8,p2_body_8);
body_8->EnableCollision(true);

bodylist.push_back(body_8);



// Rigid body part
auto body_9 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_9->SetName("Link5_UR5_STEP-1");
body_9->SetPos(chrono::ChVector3d(-0.833897884381497,1.6877363138846,0.369652114857899));
body_9->SetRot(chrono::ChQuaternion<>(0.364234205352301,0.566065007629249,0.307126035364668,0.672738767420354));
body_9->SetMass(1.05804486698019);
body_9->SetInertiaXX(chrono::ChVector3d(0.00121639724910473,0.00107178025478346,0.00129059868936729));
body_9->SetInertiaXY(chrono::ChVector3d(-0.000198974093868049,1.11778662334954e-05,-7.02759593202176e-05));
body_9->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.422595283717868,0.343520074060012,0.129144707206793),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_9_1.obj");
body_9->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_9->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_9 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_9;
chrono::ChVector3d p1_body_9(0.425997896121578,0.341550000000018,0.0766);
chrono::ChVector3d p2_body_9(0.425997896121578,0.341550000000018,0.1796);
body_9->GetCollisionModel()->AddCylinder(mat_9,0.04,p1_body_9,p2_body_9);
body_9->EnableCollision(true);

bodylist.push_back(body_9);



// Rigid body part
auto body_10 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_10->SetName("goal_new-2");
body_10->SetPos(chrono::ChVector3d(-0.0101433723263411,-0.202857627498384,1.10182323056413));
body_10->SetRot(chrono::ChQuaternion<>(0.552061525514264,-0.552061525514264,0.441846208591704,0.441846208591704));
body_10->SetMass(898.603280943793);
body_10->SetInertiaXX(chrono::ChVector3d(83.5170922925026,173.698680671425,181.89955776386));
body_10->SetInertiaXY(chrono::ChVector3d(-19.0504245345857,7.13954422838335,-3.40055020519823));
body_10->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.0623409576506786,-0.0268694136675263,0.650393775393144),chrono::ChQuaternion<>(1,0,0,0)));
body_10->SetFixed(true);

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_10_1.obj");
body_10->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_10->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_10 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_10;
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=0;
mr(0,2)=0; mr(1,2)=0; mr(2,2)=1;
collshape_10 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_10,0.279763490260656,0.367953434119727,0.287);
body_10->GetCollisionModel()->AddShape(collshape_10,chrono::ChFramed(chrono::ChVector3d(0.113633396127579,-0.152001509818433,1.1885), mr));
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=0; mr(1,2)=-1; mr(2,2)=0;
collshape_10 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_10,0.282631025558075,0.479986147738391,0.375);
body_10->GetCollisionModel()->AddShape(collshape_10,chrono::ChFramed(chrono::ChVector3d(0.141315512779038,-0.1775,0.469993073869196), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=1.36190654519734E-16; mr(1,1)=0; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=-1; mr(2,2)=0;
collshape_10 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_10,0.299349754516315,0.356648974914153,0.115);
body_10->GetCollisionModel()->AddShape(collshape_10,chrono::ChFramed(chrono::ChVector3d(0.133312866104696,0.3035,1.21898054727048), mr));
mr(0,0)=0; mr(1,0)=0; mr(2,0)=1;
mr(0,1)=1.5211874251612E-16; mr(1,1)=-1; mr(2,1)=0;
mr(0,2)=1; mr(1,2)=1.5211874251612E-16; mr(2,2)=0;
collshape_10 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_10,1.4376107720486,0.729839733264626,0.323);
body_10->GetCollisionModel()->AddShape(collshape_10,chrono::ChFramed(chrono::ChVector3d(-0.2065,-0.000185507772236835,0.7188053860243), mr));
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=0;
mr(0,2)=0; mr(1,2)=0; mr(2,2)=-1;
collshape_10 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_10,0.311967933213396,0.209587453761089,0.0380000000000003);
body_10->GetCollisionModel()->AddShape(collshape_10,chrono::ChFramed(chrono::ChVector3d(0.110120243349996,0.109793726880545,1.366), mr));
mr(0,0)=0; mr(1,0)=0; mr(2,0)=1;
mr(0,1)=-1.50453914050671E-16; mr(1,1)=1; mr(2,1)=0;
mr(0,2)=-1; mr(1,2)=-1.50453914050671E-16; mr(2,2)=0;
collshape_10 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_10,1.39252400987308,0.737915681110993,0.0650000000000003);
body_10->GetCollisionModel()->AddShape(collshape_10,chrono::ChFramed(chrono::ChVector3d(0.3285,-0.0126908183596454,0.696262004936542), mr));
body_10->EnableCollision(true);

bodylist.push_back(body_10);




// Mate constraint: Coincidente6 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_3 , SW name: baseUR5e-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_6 , SW name: Base_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.843103900678865,0.779642372501616,0.856368533712171);
cB = chrono::ChVector3d(-0.914818230514467,0.779642372501616,0.840265586893949);
dA = chrono::ChVector3d(1.52566059866291e-17,1,-4.60163415175196e-18);
dB = chrono::ChVector3d(-3.11470126175463e-17,-1,2.94802384549803e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_3,body_6,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente6");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.843103900678865,0.779642372501616,0.856368533712171);
dA = chrono::ChVector3d(1.52566059866291e-17,1,-4.60163415175196e-18);
cB = chrono::ChVector3d(-0.914818230514467,0.779642372501616,0.840265586893949);
dB = chrono::ChVector3d(-3.11470126175463e-17,-1,2.94802384549803e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_3,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente6");
linklist.push_back(link);


// Mate constraint: Concentrico3 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_3 , SW name: baseUR5e-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_6 , SW name: Base_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.950128846552856,0.779642372501616,0.784505741171517);
dA = chrono::ChVector3d(-1.52566059866291e-17,-1,4.60163415175196e-18);
cB = chrono::ChVector3d(-0.950128846552856,0.780142372501616,0.784505741171516);
dB = chrono::ChVector3d(-3.11470126175463e-17,-1,2.94802384549803e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_3,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.950128846552856,0.779642372501616,0.784505741171517);
cB = chrono::ChVector3d(-0.950128846552856,0.780142372501616,0.784505741171516);
dA = chrono::ChVector3d(-1.52566059866291e-17,-1,4.60163415175196e-18);
dB = chrono::ChVector3d(-3.11470126175463e-17,-1,2.94802384549803e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_3,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico3");
linklist.push_back(link);


// Mate constraint: Concentrico4 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_3 , SW name: baseUR5e-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_6 , SW name: Base_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.970578076236899,0.779642372501616,0.875576202932338);
dA = chrono::ChVector3d(-1.52566059866291e-17,-1,4.60163415175196e-18);
cB = chrono::ChVector3d(-0.970578076236899,0.780142372501616,0.875576202932338);
dB = chrono::ChVector3d(-3.11470126175463e-17,-1,2.94802384549803e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_3,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.970578076236899,0.779642372501616,0.875576202932338);
cB = chrono::ChVector3d(-0.970578076236899,0.780142372501616,0.875576202932338);
dA = chrono::ChVector3d(-1.52566059866291e-17,-1,4.60163415175196e-18);
dB = chrono::ChVector3d(-3.11470126175463e-17,-1,2.94802384549803e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_3,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico4");
linklist.push_back(link);


// Mate constraint: Coincidente8 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_6 , SW name: Base_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_8 , SW name: Link1_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.914818230514467,0.878742372501616,0.840265586893949);
cB = chrono::ChVector3d(-0.8578340422005,0.878742372501616,0.851073097347257);
dA = chrono::ChVector3d(1.65080164832599e-16,1,-1.19202140861217e-16);
dB = chrono::ChVector3d(-1.6217102793256e-16,-1,2.16382029215825e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_6,body_8,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente8");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.914818230514467,0.878742372501616,0.840265586893949);
dA = chrono::ChVector3d(1.65080164832599e-16,1,-1.19202140861217e-16);
cB = chrono::ChVector3d(-0.8578340422005,0.878742372501616,0.851073097347257);
dB = chrono::ChVector3d(-1.6217102793256e-16,-1,2.16382029215825e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_6,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente8");
linklist.push_back(link);


// Mate constraint: Concentrico5 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_6 , SW name: Base_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_8 , SW name: Link1_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.914818230514467,0.878742372501616,0.840265586893949);
dA = chrono::ChVector3d(-2.20913348105412e-16,-1,1.80188802434537e-16);
cB = chrono::ChVector3d(-0.914818230514467,0.878742372501616,0.840265586893949);
dB = chrono::ChVector3d(-1.62191352842636e-16,-1,2.16378174432865e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_6,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.914818230514467,0.878742372501616,0.840265586893949);
cB = chrono::ChVector3d(-0.914818230514467,0.878742372501616,0.840265586893949);
dA = chrono::ChVector3d(-2.20913348105412e-16,-1,1.80188802434537e-16);
dB = chrono::ChVector3d(-1.62191352842636e-16,-1,2.16378174432865e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_6,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico5");
linklist.push_back(link);


// Mate constraint: Coincidente10 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_8 , SW name: Link1_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_7 , SW name: Link2_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.841721271711723,0.942142372501616,0.854129014096122);
cB = chrono::ChVector3d(-0.842166927748159,1.00009304024392,0.856478801090387);
dA = chrono::ChVector3d(0.982486005413223,-1.1903160709019e-16,0.186336387125995);
dB = chrono::ChVector3d(-0.982486005413223,1.3306884236027e-16,-0.186336387125995);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_8,body_7,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente10");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.841721271711723,0.942142372501616,0.854129014096122);
dA = chrono::ChVector3d(0.982486005413223,-1.1903160709019e-16,0.186336387125995);
cB = chrono::ChVector3d(-0.842166927748159,1.00009304024392,0.856478801090387);
dB = chrono::ChVector3d(-0.982486005413223,1.3306884236027e-16,-0.186336387125995);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_8,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente10");
linklist.push_back(link);


// Mate constraint: Concentrico6 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_8 , SW name: Link1_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_7 , SW name: Link2_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.841721271711723,0.942142372501616,0.854129014096122);
dA = chrono::ChVector3d(-0.982486005413223,1.37342597848876e-16,-0.186336387125995);
cB = chrono::ChVector3d(-0.841721271711723,0.942142372501616,0.854129014096122);
dB = chrono::ChVector3d(-0.982486005413223,1.20068268897576e-16,-0.186336387125995);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_8,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico6");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.841721271711723,0.942142372501616,0.854129014096122);
cB = chrono::ChVector3d(-0.841721271711723,0.942142372501616,0.854129014096122);
dA = chrono::ChVector3d(-0.982486005413223,1.37342597848876e-16,-0.186336387125995);
dB = chrono::ChVector3d(-0.982486005413223,1.20068268897576e-16,-0.186336387125995);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_8,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico6");
linklist.push_back(link);


// Mate constraint: Coincidente11 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_7 , SW name: Link2_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_1 , SW name: Link3_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.8445411987009,1.30883021838826,0.868997493870176);
cB = chrono::ChVector3d(-0.855785172795987,1.36438921135871,0.928283000925664);
dA = chrono::ChVector3d(-0.982486005413223,-1.25515223309357e-16,-0.186336387125996);
dB = chrono::ChVector3d(0.982486005413223,1.53657206772508e-16,0.186336387125996);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_7,body_1,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente11");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.8445411987009,1.30883021838826,0.868997493870176);
dA = chrono::ChVector3d(-0.982486005413223,-1.25515223309357e-16,-0.186336387125996);
cB = chrono::ChVector3d(-0.855785172795987,1.36438921135871,0.928283000925664);
dB = chrono::ChVector3d(0.982486005413223,1.53657206772508e-16,0.186336387125996);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_7,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente11");
linklist.push_back(link);


// Mate constraint: Concentrico7 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_7 , SW name: Link2_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_1 , SW name: Link3_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.844986854737336,1.36678088613056,0.871347280864441);
dA = chrono::ChVector3d(-0.982486005413223,-1.07850119140663e-16,-0.186336387125996);
cB = chrono::ChVector3d(-0.845772843541667,1.36678088613056,0.87119821175474);
dB = chrono::ChVector3d(0.982486005413223,1.11495188537186e-16,0.186336387125996);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_7,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico7");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.844986854737336,1.36678088613056,0.871347280864441);
cB = chrono::ChVector3d(-0.845772843541667,1.36678088613056,0.87119821175474);
dA = chrono::ChVector3d(-0.982486005413223,-1.07850119140663e-16,-0.186336387125996);
dB = chrono::ChVector3d(0.982486005413223,1.11495188537186e-16,0.186336387125996);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_7,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico7");
linklist.push_back(link);


// Mate constraint: Coincidente12 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Link3_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: Link4_UR5_STEP_1000515-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.869124244891419,1.75869725547438,0.883232495919559);
cB = chrono::ChVector3d(-0.869179645910856,1.75974254818176,0.88352460594231);
dA = chrono::ChVector3d(0.982486005413223,-1.69374824381762e-17,0.186336387125996);
dB = chrono::ChVector3d(-0.982486005413223,1.11985071820083e-16,-0.186336387125995);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_5,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente12");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.869124244891419,1.75869725547438,0.883232495919559);
dA = chrono::ChVector3d(0.982486005413223,-1.69374824381762e-17,0.186336387125996);
cB = chrono::ChVector3d(-0.869179645910856,1.75974254818176,0.88352460594231);
dB = chrono::ChVector3d(-0.982486005413223,1.11985071820083e-16,-0.186336387125995);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente12");
linklist.push_back(link);


// Mate constraint: Concentrico8 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: Link3_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_5 , SW name: Link4_UR5_STEP_1000515-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.869124244891419,1.75869725547438,0.883232495919559);
dA = chrono::ChVector3d(0.982486005413223,-1.95134253625823e-17,0.186336387125996);
cB = chrono::ChVector3d(-0.868617957987758,1.75869725547438,0.883328517310152);
dB = chrono::ChVector3d(0.982486005413223,-3.24883474273901e-17,0.186336387125995);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico8");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.869124244891419,1.75869725547438,0.883232495919559);
cB = chrono::ChVector3d(-0.868617957987758,1.75869725547438,0.883328517310152);
dA = chrono::ChVector3d(0.982486005413223,-1.95134253625823e-17,0.186336387125996);
dB = chrono::ChVector3d(0.982486005413223,-3.24883474273901e-17,0.186336387125995);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico8");
linklist.push_back(link);


// Mate constraint: Coincidente13 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_5 , SW name: Link4_UR5_STEP_1000515-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_9 , SW name: Link5_UR5_STEP-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.790598087117437,1.80492974755913,0.900669503142392);
cB = chrono::ChVector3d(-0.820298994275868,1.80344351822496,0.923018714232104);
dA = chrono::ChVector3d(-0.0100586838281184,0.998541945674989,0.0530358898035336);
dB = chrono::ChVector3d(0.0100586838281186,-0.998541945674989,-0.0530358898035343);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_9,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente13");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.790598087117437,1.80492974755913,0.900669503142392);
dA = chrono::ChVector3d(-0.0100586838281184,0.998541945674989,0.0530358898035336);
cB = chrono::ChVector3d(-0.820298994275868,1.80344351822496,0.923018714232104);
dB = chrono::ChVector3d(0.0100586838281186,-0.998541945674989,-0.0530358898035343);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente13");
linklist.push_back(link);


// Mate constraint: Concentrico9 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: Link4_UR5_STEP_1000515-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_9 , SW name: Link5_UR5_STEP-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.790598087117437,1.80492974755913,0.900669503142392);
dA = chrono::ChVector3d(-0.0100586838281184,0.998541945674989,0.0530358898035336);
cB = chrono::ChVector3d(-0.7906061340645,1.80572858111567,0.900711931854235);
dB = chrono::ChVector3d(-0.0100586838281184,0.998541945674989,0.0530358898035335);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico9");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.790598087117437,1.80492974755913,0.900669503142392);
cB = chrono::ChVector3d(-0.7906061340645,1.80572858111567,0.900711931854235);
dA = chrono::ChVector3d(-0.0100586838281184,0.998541945674989,0.0530358898035336);
dB = chrono::ChVector3d(-0.0100586838281184,0.998541945674989,0.0530358898035335);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_5,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico9");
linklist.push_back(link);


// Mate constraint: Coincidente14 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: Link5_UR5_STEP-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: End_effector_assembly2-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801);
cB = chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801);
dA = chrono::ChVector3d(0.985359365892694,0.000870704484685769,0.170488011077757);
dB = chrono::ChVector3d(-0.985359365892694,-0.000870704484685728,-0.170488011077757);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_9,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente14");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801);
dA = chrono::ChVector3d(0.985359365892694,0.000870704484685769,0.170488011077757);
cB = chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801);
dB = chrono::ChVector3d(-0.985359365892694,-0.000870704484685728,-0.170488011077757);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente14");
linklist.push_back(link);


// Mate constraint: Concentrico10 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: Link5_UR5_STEP-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_4 , SW name: End_effector_assembly2-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801);
dA = chrono::ChVector3d(0.985359365892694,0.000870704484685769,0.170488011077757);
cB = chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801);
dB = chrono::ChVector3d(-0.985359365892694,-0.000870704484685793,-0.170488011077757);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentrico10");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801);
cB = chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801);
dA = chrono::ChVector3d(0.985359365892694,0.000870704484685769,0.170488011077757);
dB = chrono::ChVector3d(-0.985359365892694,-0.000870704484685793,-0.170488011077757);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentrico10");
linklist.push_back(link);


// Mate constraint: Coincidente15 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_3 , SW name: baseUR5e-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: start_new-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.914818230514467,0.757142372501616,0.840265586893949);
cB = chrono::ChVector3d(-1.15241262841672,0.757142372501616,0.791527490034647);
dA = chrono::ChVector3d(-1.52566059866291e-17,-1,4.60163415175196e-18);
dB = chrono::ChVector3d(1.3540629282151e-17,1,3.0404527758341e-18);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_3,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente15");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.914818230514467,0.757142372501616,0.840265586893949);
dA = chrono::ChVector3d(-1.52566059866291e-17,-1,4.60163415175196e-18);
cB = chrono::ChVector3d(-1.15241262841672,0.757142372501616,0.791527490034647);
dB = chrono::ChVector3d(1.3540629282151e-17,1,3.0404527758341e-18);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_3,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente15");
linklist.push_back(link);


// Mate constraint: Distanza1 [MateDistanceDim] type:5 align:0 flip:False
//   Entity 0: C::E name: body_3 , SW name: baseUR5e-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: start_new-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.807286871346519,0.768392372501616,0.772170015852259);
cB = chrono::ChVector3d(-0.724374877588822,0.757142372501616,0.653963200096468);
dA = chrono::ChVector3d(0.97570516783132,-1.38777878078145e-17,0.219087711812545);
dB = chrono::ChVector3d(0.975705167831319,-1.38777878078145e-17,0.219087711812545);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_3,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(-0.055);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distanza1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.807286871346519,0.768392372501616,0.772170015852259);
dA = chrono::ChVector3d(0.97570516783132,-1.38777878078145e-17,0.219087711812545);
cB = chrono::ChVector3d(-0.724374877588822,0.757142372501616,0.653963200096468);
dB = chrono::ChVector3d(0.975705167831319,-1.38777878078145e-17,0.219087711812545);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_3,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distanza1");
linklist.push_back(link);


// Mate constraint: Distanza2 [MateDistanceDim] type:5 align:2 flip:False
//   Entity 0: C::E name: body_3 , SW name: baseUR5e-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: start_new-1 ,  SW ref.type:1 (1)

// Mate constraint: Coincidente18 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_2 , SW name: start_new-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_10 , SW name: goal_new-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-1.15241262841672,-0.202857627498384,0.791527490034647);
cB = chrono::ChVector3d(-0.0101433723263411,-0.202857627498384,1.10182323056413);
dA = chrono::ChVector3d(-1.3540629282151e-17,-1,-3.0404527758341e-18);
dB = chrono::ChVector3d(8.66614664847819e-18,-1,-1.77699551983796e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_2,body_10,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincidente18");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-1.15241262841672,-0.202857627498384,0.791527490034647);
dA = chrono::ChVector3d(-1.3540629282151e-17,-1,-3.0404527758341e-18);
cB = chrono::ChVector3d(-0.0101433723263411,-0.202857627498384,1.10182323056413);
dB = chrono::ChVector3d(8.66614664847819e-18,-1,-1.77699551983796e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincidente18");
linklist.push_back(link);


// Mate constraint: Distanza3 [MateDistanceDim] type:5 align:0 flip:True
//   Entity 0: C::E name: body_10 , SW name: goal_new-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: start_new-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.428320880201694,0.927142372501616,1.3666393401707);
cB = chrono::ChVector3d(-1.63520195353175,-0.202857627498384,0.967530238748913);
dA = chrono::ChVector3d(-0.219087711812545,-1.20050240048949e-17,0.97570516783132);
dB = chrono::ChVector3d(-0.219087711812545,1.54074395550979e-33,0.975705167831319);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_10,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0.125);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distanza3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.428320880201694,0.927142372501616,1.3666393401707);
dA = chrono::ChVector3d(-0.219087711812545,-1.20050240048949e-17,0.97570516783132);
cB = chrono::ChVector3d(-1.63520195353175,-0.202857627498384,0.967530238748913);
dB = chrono::ChVector3d(-0.219087711812545,1.54074395550979e-33,0.975705167831319);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_10,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distanza3");
linklist.push_back(link);


// Mate constraint: Distanza4 [MateDistanceDim] type:5 align:1 flip:False
//   Entity 0: C::E name: body_10 , SW name: goal_new-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: start_new-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.428320880201694,0.927142372501616,1.3666393401707);
cB = chrono::ChVector3d(-0.669623303301691,-0.202857627498384,0.615524741320382);
dA = chrono::ChVector3d(-0.97570516783132,-2.99644053816264e-17,-0.219087711812545);
dB = chrono::ChVector3d(0.975705167831319,-1.38777878078145e-17,0.219087711812545);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_10,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0.4);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distanza4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.428320880201694,0.927142372501616,1.3666393401707);
dA = chrono::ChVector3d(-0.97570516783132,-2.99644053816264e-17,-0.219087711812545);
cB = chrono::ChVector3d(-0.669623303301691,-0.202857627498384,0.615524741320382);
dB = chrono::ChVector3d(0.975705167831319,-1.38777878078145e-17,0.219087711812545);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_10,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distanza4");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("MARKER_1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.914818230514467,0.878742372501652,0.840265586893949),chrono::ChQuaternion<>(0.707106781186547,-0.707106781186548,5.83646519962619E-17,-5.83646519962619E-17)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("MARKER_2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.841721271711723,0.942142372501616,0.854129014096122),chrono::ChQuaternion<>(0.77017413197471,4.58375323341208E-17,0.637833682425914,-3.79611841377052E-17)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("MARKER_3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.844986854737336,1.36678088613056,0.871347280864441),chrono::ChQuaternion<>(0.637833682425914,4.00289185419592E-17,-0.77017413197471,4.83342890809481E-17)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("MARKER_4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.869124244891419,1.75869725547438,0.88323249591956),chrono::ChQuaternion<>(0.77017413197471,6.52240541732963E-18,0.637833682425914,-5.4016483972831E-18)));

// Auxiliary marker (coordinate system feature)
auto marker_0_5 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_5->SetName("MARKER_5");
body_0->AddMarker(marker_0_5);
marker_0_5->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.790598087117437,1.80492974755913,0.900669503142392),chrono::ChQuaternion<>(0.722755701038937,-0.684738781815611,-0.0679327297840056,-0.0643594710783237)));

// Auxiliary marker (coordinate system feature)
auto marker_0_6 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_6->SetName("MARKER_6");
body_0->AddMarker(marker_0_6);
marker_0_6->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.745513082193027,1.85829220107582,0.911395214570801),chrono::ChQuaternion<>(0.765012370442147,-0.000333049914015521,0.644015437345159,0.000280373617891312)));

// Auxiliary marker (coordinate system feature)
auto marker_0_7 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_7->SetName("MARKER_TCP");
body_0->AddMarker(marker_0_7);
marker_0_7->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.371429592583631,1.85862275679318,0.976119570301416),chrono::ChQuaternion<>(0.765012370442147,-0.000333049914015517,0.644015437345159,0.000280373617891309)));


} // end function
