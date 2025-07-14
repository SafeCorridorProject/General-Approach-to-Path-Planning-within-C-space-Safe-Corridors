

// ChRobot_6dof_CAD class implementation

#include "ChRobot_6dof_CAD.h"

namespace chrono {
namespace industrial {


ChRobot_6dof_CAD::ChRobot_6dof_CAD(ChSystem* system, const ChCoordsys<>& base_coord, unsigned int id, std::vector<std::string> bodynames)
        : m_id(id), m_bodynames(bodynames) {
    m_system = system;
    m_base_coord = base_coord;
    m_link_attach = chrono_types::make_shared<ChLinkMateFix>();

    m_encoder.resize(6);
    m_encoder << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_encoder_prev = m_encoder;

    SetupBodies();
    SetupMarkers();
    SetupLinks();

    SetBaseCoord(m_base_coord);
}

void ChRobot_6dof_CAD::SetupBodies() {  
    if (m_bodynames.size() == 7) {
        // All 7 default robot bodies (i.e. including a robot basement) are provided. Proceed normally.

        // Ground (virtual)
        m_ground = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody("SLDW_GROUND"));
        m_ground->SetName(("robot" + std::to_string(m_id) + "_ground").c_str());
        m_ground->SetMass(1e-4);
        m_ground->SetFixed(true);

        // Base (potential robot basement)
        m_base = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[0] + "-1").c_str()));
        m_base->SetName(("robot" + std::to_string(m_id) + "_base").c_str());

        // Shoulder
        m_shoulder = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[1] + "-1").c_str()));
        m_shoulder->SetName(("robot" + std::to_string(m_id) + "_shoulder").c_str());
        //m_shoulder->GetVisualShape(0)->SetColor(ChColor(0.0f, 1.0f, 0.0f));
        // Biceps
        m_biceps = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[2] + "-1").c_str()));
        m_biceps->SetName(("robot" + std::to_string(m_id) + "_biceps").c_str());

        // Elbow
        m_elbow = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[3] + "-1").c_str()));
        m_elbow->SetName(("robot" + std::to_string(m_id) + "_elbow").c_str());
        //m_elbow->GetVisualShape(0)->SetColor(ChColor(1.0f, 0.0f, 1.0f));
        // Forearm
        m_forearm = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[4] + "-1").c_str()));
        m_forearm->SetName(("robot" + std::to_string(m_id) + "_forearm").c_str());
        //m_forearm->GetVisualShape(0)->SetColor(ChColor(1.0f, 1.0f, 0.0f));
        // Wrist
        m_wrist = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[5] + "-1").c_str()));
        m_wrist->SetName(("robot" + std::to_string(m_id) + "_wrist").c_str());
        //m_wrist->GetVisualShape(0)->SetColor(ChColor(0.0f, 0.0f, 1.0f));

        // End effector
        m_end_effector = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[6] + "-1").c_str()));
        m_end_effector->SetName(("robot" + std::to_string(m_id) + "_end_effector").c_str());
        //m_end_effector->GetVisualShape(0)->SetColor(ChColor(1.0f, 0.0f, 1.0f));

        m_bodylist = { m_ground, m_base, m_shoulder, m_biceps, m_elbow, m_forearm, m_wrist, m_end_effector };
    }
    else if (m_bodynames.size() == 6) {
        // Only 6 robot bodies (i.e. no robot basement) are provided. Fallback to use internal 'ground' as base.

        // Ground (virtual)
        m_ground = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody("ground"));
        m_ground->SetName(("robot" + std::to_string(m_id) + "_ground").c_str());
        m_ground->SetMass(1e-4);
        m_ground->SetFixed(true);

        // Shoulder
        m_shoulder = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[0] + "-1").c_str()));
        m_shoulder->SetName(("robot" + std::to_string(m_id) + "_shoulder").c_str());

        // Biceps
        m_biceps = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[1] + "-1").c_str()));
        m_biceps->SetName(("robot" + std::to_string(m_id) + "_biceps").c_str());

        // Elbow
        m_elbow = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[2] + "-1").c_str()));
        m_elbow->SetName(("robot" + std::to_string(m_id) + "_elbow").c_str());

        // Forearm
        m_forearm = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[3] + "-1").c_str()));
        m_forearm->SetName(("robot" + std::to_string(m_id) + "_forearm").c_str());

        // Wrist
        m_wrist = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[4] + "-1").c_str()));
        m_wrist->SetName(("robot" + std::to_string(m_id) + "_wrist").c_str());

        // End effector
        m_end_effector = std::dynamic_pointer_cast<ChBodyAuxRef>(m_system->SearchBody((m_bodynames[5] + "-1").c_str()));
        m_end_effector->SetName(("robot" + std::to_string(m_id) + "_end_effector").c_str());

        m_bodylist = { m_ground, m_shoulder, m_biceps, m_elbow, m_forearm, m_wrist, m_end_effector };
    }
}

void ChRobot_6dof_CAD::SetupMarkers() {
    // Marker base-shoulder
    m_marker_base_shoulder = m_system->SearchMarker("MARKER_1");
    m_marker_base_shoulder->SetName(("robot" + std::to_string(m_id) + "_MARKER_1").c_str());
    if (m_base)
        m_base->AddMarker(m_marker_base_shoulder);
    else
        m_ground->AddMarker(m_marker_base_shoulder); // add to ground, instead
    m_marker_base_shoulder->ImposeAbsoluteTransform(m_marker_base_shoulder->GetAbsFrame());

    // Marker shoulder-biceps
    m_marker_shoulder_biceps = m_system->SearchMarker("MARKER_2");
    m_marker_shoulder_biceps->SetName(("robot" + std::to_string(m_id) + "_MARKER_2").c_str());
    m_shoulder->AddMarker(m_marker_shoulder_biceps);
    m_marker_shoulder_biceps->ImposeAbsoluteTransform(m_marker_shoulder_biceps->GetAbsFrame());

    // Marker biceps-elbow
    m_marker_biceps_elbow = m_system->SearchMarker("MARKER_3");
    m_marker_biceps_elbow->SetName(("robot" + std::to_string(m_id) + "_MARKER_3").c_str());
    m_biceps->AddMarker(m_marker_biceps_elbow);
    m_marker_biceps_elbow->ImposeAbsoluteTransform(m_marker_biceps_elbow->GetAbsFrame());

    // Marker elbow-forearm
    m_marker_elbow_forearm = m_system->SearchMarker("MARKER_4");
    m_marker_elbow_forearm->SetName(("robot" + std::to_string(m_id) + "_MARKER_4").c_str());
    m_elbow->AddMarker(m_marker_elbow_forearm);
    m_marker_elbow_forearm->ImposeAbsoluteTransform(m_marker_elbow_forearm->GetAbsFrame());

    // Marker forearm-wrist
    m_marker_forearm_wrist = m_system->SearchMarker("MARKER_5");
    m_marker_forearm_wrist->SetName(("robot" + std::to_string(m_id) + "_MARKER_5").c_str());
    m_forearm->AddMarker(m_marker_forearm_wrist);
    m_marker_forearm_wrist->ImposeAbsoluteTransform(m_marker_forearm_wrist->GetAbsFrame());

    // Marker wrist-end_effector
    m_marker_wrist_end_effector = m_system->SearchMarker("MARKER_6");
    m_marker_wrist_end_effector->SetName(("robot" + std::to_string(m_id) + "_MARKER_6").c_str());
    m_wrist->AddMarker(m_marker_wrist_end_effector);
    m_marker_wrist_end_effector->ImposeAbsoluteTransform(m_marker_wrist_end_effector->GetAbsFrame());

    // Marker TCP
    m_marker_TCP = m_system->SearchMarker("MARKER_TCP");
    m_marker_TCP->SetName(("robot" + std::to_string(m_id) + "_MARKER_TCP").c_str());
    m_end_effector->AddMarker(m_marker_TCP);
    m_marker_TCP->ImposeAbsoluteTransform(m_marker_TCP->GetAbsFrame());

    m_markerlist = { m_marker_base_shoulder, m_marker_shoulder_biceps, m_marker_biceps_elbow, m_marker_elbow_forearm, m_marker_forearm_wrist, m_marker_wrist_end_effector, m_marker_TCP };
    
    m_lengths = { 
        (m_marker_shoulder_biceps->GetAbsCoordsys().pos - m_marker_base_shoulder->GetAbsCoordsys().pos).Length(),
        (m_marker_biceps_elbow->GetAbsCoordsys().pos - m_marker_shoulder_biceps->GetAbsCoordsys().pos).Length(),
        (m_marker_forearm_wrist->GetAbsCoordsys().pos - m_marker_biceps_elbow->GetAbsCoordsys().pos).Length(),
        (m_marker_TCP->GetAbsCoordsys().pos - m_marker_forearm_wrist->GetAbsCoordsys().pos).Length() };

    m_joint_coords = {}; // no use
}

void ChRobot_6dof_CAD::SetupLinks() {
    m_motfunlist = {
         chrono_types::make_shared<ChFunctionSetpoint>(),
         chrono_types::make_shared<ChFunctionSetpoint>(),
         chrono_types::make_shared<ChFunctionSetpoint>(),
         chrono_types::make_shared<ChFunctionSetpoint>(),
         chrono_types::make_shared<ChFunctionSetpoint>(),
         chrono_types::make_shared<ChFunctionSetpoint>() };

    // Link base-shoulder
    m_link_base_shoulder = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    if (m_base)
        m_link_base_shoulder->Initialize(m_shoulder, m_base, m_marker_base_shoulder->GetAbsFrame());
    else
        m_link_base_shoulder->Initialize(m_shoulder, m_ground, m_marker_base_shoulder->GetAbsFrame()); // link to ground, instead
    //m_link_base_shoulder->SetMotorFunction(m_motfunlist[0]);
    m_system->Add(m_link_base_shoulder);

    // Link shoulder-biceps
    m_link_shoulder_biceps = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_shoulder_biceps->Initialize(m_biceps, m_shoulder, m_marker_shoulder_biceps->GetAbsFrame());
    //m_link_shoulder_biceps->SetMotorFunction(m_motfunlist[1]);
    m_system->Add(m_link_shoulder_biceps);

    // Link biceps-elbow
    m_link_biceps_elbow = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_biceps_elbow->Initialize(m_elbow, m_biceps, m_marker_biceps_elbow->GetAbsFrame());
    //m_link_biceps_elbow->SetMotorFunction(m_motfunlist[2]);
    m_system->Add(m_link_biceps_elbow);

    // Link elbow-forearm
    m_link_elbow_forearm = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_elbow_forearm->Initialize(m_forearm, m_elbow, m_marker_elbow_forearm->GetAbsFrame());
    m_link_elbow_forearm->SetMotorFunction(m_motfunlist[3]);
    m_system->Add(m_link_elbow_forearm);

    // Link forearm-wrist
    m_link_forearm_wrist = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_forearm_wrist->Initialize(m_wrist, m_forearm, m_marker_forearm_wrist->GetAbsFrame());
    //m_link_forearm_wrist->SetMotorFunction(m_motfunlist[4]);
    m_system->Add(m_link_forearm_wrist);

    // Link wrist-end effector
    m_link_wrist_end_effector = chrono_types::make_shared<ChLinkMotorRotationAngle>();  
    m_link_wrist_end_effector->Initialize(m_end_effector, m_wrist, m_marker_wrist_end_effector->GetAbsFrame());
    //m_link_wrist_end_effector->SetMotorFunction(m_motfunlist[5]);
    m_system->Add(m_link_wrist_end_effector);

    m_motorlist = { m_link_base_shoulder, m_link_shoulder_biceps, m_link_biceps_elbow, m_link_elbow_forearm, m_link_forearm_wrist, m_link_wrist_end_effector };
}


} // end namespace industrial
} // end namespace chrono
