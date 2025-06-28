// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Fusai
// =============================================================================

// ChRobot_6dof class implementation

#include "ChRobot_6dof.h"

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeBox.h"


namespace chrono {
namespace industrial {


ChRobot_6dof::ChRobot_6dof(ChSystem* system, const std::array<double, 4>& lengths, const ChCoordsys<>& base_coord) {
    m_system = system;
    m_lengths = lengths; // H, L1, L2, L3
    m_base_coord = base_coord;
    m_link_attach = chrono_types::make_shared<ChLinkMateFix>();

    // Starting joint coordinates. Assume motors rotation along local Z axes.
    m_joint_coords = {
        ChCoordsys<>(VNULL, Q_ROTATE_Z_TO_Y),                                                                                       // (0) base-shoulder
        ChCoordsys<>(ChVector3d(0, m_lengths[0], 0), QUNIT),                                                                        // (1) shoulder-biceps
        ChCoordsys<>(ChVector3d(0, m_lengths[0] + m_lengths[1], 0), QUNIT),                                                         // (2) biceps-elbow
        ChCoordsys<>(ChVector3d(0, m_lengths[0] + m_lengths[1], 0), Q_ROTATE_Z_TO_X * Q_ROTATE_X_TO_Y),                             // (3) elbow-forearm
        ChCoordsys<>(ChVector3d(m_lengths[2], m_lengths[0] + m_lengths[1], 0), Q_ROTATE_X_TO_Y),                                    // (4) forearm-wrist
        ChCoordsys<>(ChVector3d(m_lengths[2], m_lengths[0] + m_lengths[1], 0), Q_ROTATE_Y_TO_Z * Q_FLIP_AROUND_Z),                  // (5) wrist-end_effector
        ChCoordsys<>(ChVector3d(m_lengths[2], m_lengths[0] + m_lengths[1] - m_lengths[3], 0), Q_ROTATE_Y_TO_Z * Q_FLIP_AROUND_Z) }; // (6) TCP
    
    for (auto& jcood : m_joint_coords)
        jcood.ConcatenatePreTransformation(m_base_coord);

    m_encoder.resize(6);
    m_encoder << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_encoder_prev = m_encoder;

    SetupBodies();
    SetupMarkers();
    SetupLinks();
}

void ChRobot_6dof::SetupBodies() {
    // Base
    m_base = chrono_types::make_shared<ChBodyAuxRef>();
    m_base->SetFixed(true);
    m_base->SetPos(m_joint_coords[0].pos);
    m_system->Add(m_base);

    // Shoulder
    m_shoulder = chrono_types::make_shared<ChBodyAuxRef>();
    m_shoulder->SetPos((m_joint_coords[0].pos + m_joint_coords[1].pos) * 0.5);
    m_system->Add(m_shoulder);

    // Biceps
    m_biceps = chrono_types::make_shared<ChBodyAuxRef>();
    m_biceps->SetPos((m_joint_coords[1].pos + m_joint_coords[2].pos) * 0.5);
    m_system->Add(m_biceps);

    // Elbow
    m_elbow = chrono_types::make_shared<ChBodyAuxRef>();
    m_elbow->SetPos(m_joint_coords[2].pos);
    m_system->Add(m_elbow);

    // Forearm
    m_forearm = chrono_types::make_shared<ChBodyAuxRef>();
    m_forearm->SetPos((m_joint_coords[3].pos + m_joint_coords[4].pos) * 0.5);
    m_system->Add(m_forearm);

    // Wrist
    m_wrist = chrono_types::make_shared<ChBodyAuxRef>();
    m_wrist->SetPos((m_joint_coords[4].pos + m_joint_coords[5].pos) * 0.5);
    m_system->Add(m_wrist);

    // End effector
    m_end_effector = chrono_types::make_shared<ChBodyAuxRef>();
    m_end_effector->SetPos(m_joint_coords[6].pos);
    m_system->Add(m_end_effector);

    m_bodylist = { m_base, m_shoulder, m_biceps, m_elbow, m_forearm, m_wrist, m_end_effector };
}

void ChRobot_6dof::SetupMarkers() {
    // Marker base-shoulder
    m_marker_base_shoulder = chrono_types::make_shared<ChMarker>();
    m_base->AddMarker(m_marker_base_shoulder);
    m_marker_base_shoulder->ImposeAbsoluteTransform(ChFramed(m_joint_coords[0]));

    // Marker shoulder-biceps
    m_marker_shoulder_biceps = chrono_types::make_shared<ChMarker>();
    m_shoulder->AddMarker(m_marker_shoulder_biceps);
    m_marker_shoulder_biceps->ImposeAbsoluteTransform(ChFramed(m_joint_coords[1]));

    // Marker biceps-elbow
    m_marker_biceps_elbow = chrono_types::make_shared<ChMarker>();
    m_biceps->AddMarker(m_marker_biceps_elbow);
    m_marker_biceps_elbow->ImposeAbsoluteTransform(ChFramed(m_joint_coords[2]));

    // Marker elbow-forearm
    m_marker_elbow_forearm = chrono_types::make_shared<ChMarker>();
    m_elbow->AddMarker(m_marker_elbow_forearm);
    m_marker_elbow_forearm->ImposeAbsoluteTransform(ChFramed(m_joint_coords[3]));

    // Marker forearm-wrist
    m_marker_forearm_wrist = chrono_types::make_shared<ChMarker>();
    m_forearm->AddMarker(m_marker_forearm_wrist);
    m_marker_forearm_wrist->ImposeAbsoluteTransform(ChFramed(m_joint_coords[4]));

    // Marker wrist-end_effector
    m_marker_wrist_end_effector = chrono_types::make_shared<ChMarker>();
    m_wrist->AddMarker(m_marker_wrist_end_effector);
    m_marker_wrist_end_effector->ImposeAbsoluteTransform(ChFramed(m_joint_coords[5]));

    // Marker TCP
    m_marker_TCP = chrono_types::make_shared<ChMarker>();
    m_end_effector->AddMarker(m_marker_TCP);
    m_marker_TCP->ImposeAbsoluteTransform(ChFramed(m_joint_coords[6]));

    m_markerlist = { m_marker_base_shoulder, m_marker_shoulder_biceps, m_marker_biceps_elbow, m_marker_elbow_forearm, m_marker_forearm_wrist, m_marker_wrist_end_effector, m_marker_TCP };
}

void ChRobot_6dof::SetupLinks() {
    m_motfunlist = { 
        chrono_types::make_shared<ChFunctionSetpoint>(),
        chrono_types::make_shared<ChFunctionSetpoint>(),
        chrono_types::make_shared<ChFunctionSetpoint>(),
        chrono_types::make_shared<ChFunctionSetpoint>(),
        chrono_types::make_shared<ChFunctionSetpoint>(),
        chrono_types::make_shared<ChFunctionSetpoint>() };

    // Link base-shoulder
    m_link_base_shoulder = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_base_shoulder->Initialize(m_shoulder, m_base, ChFrame<>(m_joint_coords[0]));
    m_link_base_shoulder->SetMotorFunction(m_motfunlist[0]);
    m_system->Add(m_link_base_shoulder);

    // Link shoulder-biceps
    m_link_shoulder_biceps = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_shoulder_biceps->Initialize(m_biceps, m_shoulder, ChFrame<>(m_joint_coords[1]));
    m_link_shoulder_biceps->SetMotorFunction(m_motfunlist[1]);
    m_system->Add(m_link_shoulder_biceps);

    // Link biceps-elbow
    m_link_biceps_elbow = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_biceps_elbow->Initialize(m_elbow, m_biceps, ChFrame<>(m_joint_coords[2]));
    m_link_biceps_elbow->SetMotorFunction(m_motfunlist[2]);
    m_system->Add(m_link_biceps_elbow);

    // Link elbow-forearm
    m_link_elbow_forearm = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_elbow_forearm->Initialize(m_forearm, m_elbow, ChFrame<>(m_joint_coords[3]));
    m_link_elbow_forearm->SetMotorFunction(m_motfunlist[3]);
    m_system->Add(m_link_elbow_forearm);

    // Link forearm-wrist
    m_link_forearm_wrist = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_forearm_wrist->Initialize(m_wrist, m_forearm, ChFrame<>(m_joint_coords[4]));
    m_link_forearm_wrist->SetMotorFunction(m_motfunlist[4]);
    m_system->Add(m_link_forearm_wrist);

    // Link wrist-end effector
    m_link_wrist_end_effector = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_link_wrist_end_effector->Initialize(m_end_effector, m_wrist, ChFrame<>(m_joint_coords[5]));
    m_link_wrist_end_effector->SetMotorFunction(m_motfunlist[5]);
    m_system->Add(m_link_wrist_end_effector);

    m_motorlist = { m_link_base_shoulder, m_link_shoulder_biceps, m_link_biceps_elbow, m_link_elbow_forearm, m_link_forearm_wrist, m_link_wrist_end_effector };
}

ChVectorDynamic<> ChRobot_6dof::GetMotorsRot() const {
    ChVectorDynamic<> angles(6);
    angles <<
        m_link_base_shoulder->GetMotorAngle(),
        m_link_shoulder_biceps->GetMotorAngle(),
        m_link_biceps_elbow->GetMotorAngle(),
        m_link_elbow_forearm->GetMotorAngle(),
        m_link_forearm_wrist->GetMotorAngle(),
        m_link_wrist_end_effector->GetMotorAngle();
    return angles;
}

ChVectorDynamic<> ChRobot_6dof::GetMotorsRot_dt() const {
    ChVectorDynamic<> vels(6);
    vels <<
        m_link_base_shoulder->GetMotorAngleDt(),
        m_link_shoulder_biceps->GetMotorAngleDt(),
        m_link_biceps_elbow->GetMotorAngleDt(),
        m_link_elbow_forearm->GetMotorAngleDt(),
        m_link_forearm_wrist->GetMotorAngleDt(),
        m_link_wrist_end_effector->GetMotorAngleDt();
    return vels;
}

ChVectorDynamic<> ChRobot_6dof::GetMotorsRot_dtdt() const {
    ChVectorDynamic<> accels(6);
    accels <<
        m_link_base_shoulder->GetMotorAngleDt2(),
        m_link_shoulder_biceps->GetMotorAngleDt2(),
        m_link_biceps_elbow->GetMotorAngleDt2(),
        m_link_elbow_forearm->GetMotorAngleDt2(),
        m_link_forearm_wrist->GetMotorAngleDt2(),
        m_link_wrist_end_effector->GetMotorAngleDt2();
    return accels;
}

ChVectorDynamic<> ChRobot_6dof::GetMotorsTorque() const {
    ChVectorDynamic<> torques(6);
    torques <<
        m_link_base_shoulder->GetMotorTorque(),
        m_link_shoulder_biceps->GetMotorTorque(),
        m_link_biceps_elbow->GetMotorTorque(),
        m_link_elbow_forearm->GetMotorTorque(),
        m_link_forearm_wrist->GetMotorTorque(),
        m_link_wrist_end_effector->GetMotorTorque();
    return torques;
}

void ChRobot_6dof::Add1dShapes(const ChColor& col) {
    //// Link 1
    //auto shape1 = chrono_types::make_shared<ChLineShape>();
    //auto line1 = chrono_types::make_shared<geometry::ChLineSegment>();
    //line1->pA = m_shoulder->TransformPointParentToLocal(m_joint_coords[0].pos);
    //line1->pB = m_shoulder->TransformPointParentToLocal(m_joint_coords[1].pos);
    //shape1->SetLineGeometry(line1);
    //shape1->SetColor(col);
    //m_shoulder->AddVisualShape(shape1);

    //// Link 2
    //auto shape2 = chrono_types::make_shared<ChLineShape>();
    //auto line2 = chrono_types::make_shared<geometry::ChLineSegment>();
    //line2->pA = m_biceps->TransformPointParentToLocal(m_joint_coords[1].pos);
    //line2->pB = m_biceps->TransformPointParentToLocal(m_joint_coords[2].pos);
    //shape2->SetLineGeometry(line2);
    //shape2->SetColor(col);
    //m_biceps->AddVisualShape(shape2);

    //// Link 3
    //auto shape3 = chrono_types::make_shared<ChLineShape>();
    //auto line3 = chrono_types::make_shared<geometry::ChLineSegment>();
    //line3->pA = m_forearm->TransformPointParentToLocal(m_joint_coords[3].pos);
    //line3->pB = m_forearm->TransformPointParentToLocal(m_joint_coords[4].pos);
    //shape3->SetLineGeometry(line3);
    //shape3->SetColor(col);
    //m_forearm->AddVisualShape(shape3);

    //// Link 4
    //auto shape4 = chrono_types::make_shared<ChLineShape>();
    //auto line4 = chrono_types::make_shared<geometry::ChLineSegment>();
    //line4->pA = m_wrist->TransformPointParentToLocal(m_joint_coords[5].pos);
    //line4->pB = m_wrist->TransformPointParentToLocal(m_joint_coords[6].pos);
    //shape4->SetLineGeometry(line4);
    //shape4->SetColor(col);
    //m_wrist->AddVisualShape(shape4);

}

void ChRobot_6dof::Add3dShapes(double rad, const ChColor& col) {
    //geometry::ChLineSegment segm;

    //// Link 1
    //segm.pA = m_shoulder->TransformPointParentToLocal(m_joint_coords[0].pos);
    //segm.pB = m_shoulder->TransformPointParentToLocal(m_joint_coords[1].pos);
    //auto shape1a = chrono_types::make_shared<ChVisualShapeCylinder>(rad, segm.GetLength());
    //shape1a->SetColor(col);
    //m_shoulder->AddVisualShape(shape1a, segm.GetFrame());

    //segm.pA = m_shoulder->TransformPointParentToLocal(m_joint_coords[1].pos + ChVector3d(0, 0, 2 * rad));
    //segm.pB = m_shoulder->TransformPointParentToLocal(m_joint_coords[1].pos + ChVector3d(0, 0, -2 * rad));
    //auto shape1b = chrono_types::make_shared<ChVisualShapeCylinder>(rad, segm.GetLength());
    //shape1b->SetColor(col);
    //m_shoulder->AddVisualShape(shape1b, segm.GetFrame());

    //// Link 2
    //segm.pA = m_biceps->TransformPointParentToLocal(m_joint_coords[1].pos);
    //segm.pB = m_biceps->TransformPointParentToLocal(m_joint_coords[2].pos);
    //auto shape2a = chrono_types::make_shared<ChVisualShapeCylinder>(rad, segm.GetLength());
    //shape2a->SetColor(col);
    //m_biceps->AddVisualShape(shape2a, segm.GetFrame());

    //segm.pA = m_biceps->TransformPointParentToLocal(m_joint_coords[2].pos + ChVector3d(0, 0, 2 * rad));
    //segm.pB = m_biceps->TransformPointParentToLocal(m_joint_coords[2].pos + ChVector3d(0, 0, -2 * rad));
    //auto shape2b = chrono_types::make_shared<ChVisualShapeCylinder>(rad, segm.GetLength());
    //shape2b->SetColor(col);
    //m_biceps->AddVisualShape(shape2b, segm.GetFrame());

    //// Link 3
    //segm.pA = m_forearm->TransformPointParentToLocal(m_joint_coords[3].pos);
    //segm.pB = m_forearm->TransformPointParentToLocal(m_joint_coords[4].pos);
    //auto shape3a = chrono_types::make_shared<ChVisualShapeCylinder>(rad, segm.GetLength());
    //shape3a->SetColor(col);
    //m_forearm->AddVisualShape(shape3a, segm.GetFrame());

    //segm.pA = m_forearm->TransformPointParentToLocal(m_joint_coords[4].pos + ChVector3d(0, 0, 2 * rad));
    //segm.pB = m_forearm->TransformPointParentToLocal(m_joint_coords[4].pos + ChVector3d(0, 0, -2 * rad));
    //auto shape3b = chrono_types::make_shared<ChVisualShapeCylinder>(rad, segm.GetLength());
    //shape3b->SetColor(col);
    //m_forearm->AddVisualShape(shape3b, segm.GetFrame());

    //// Link 4
    //segm.pA = m_wrist->TransformPointParentToLocal(m_joint_coords[5].pos);
    //segm.pB = m_wrist->TransformPointParentToLocal(m_joint_coords[6].pos);
    //auto shape4 = chrono_types::make_shared<ChVisualShapeCylinder>(rad, segm.GetLength());
    //shape4->SetColor(col);
    //m_wrist->AddVisualShape(shape4, segm.GetFrame());

    //// Link 5
    //auto shape5 = chrono_types::make_shared<ChVisualShapeBox>(2 * rad, 2 * rad, 2 * rad);
    //shape5->SetColor(col);
    //m_end_effector->AddVisualShape(shape5);
}


} // end namespace industrial
} // end namespace chrono