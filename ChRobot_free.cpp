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

// ChRobot base class implementation

#include "ChRobot_free.h"

namespace chrono {
namespace industrial {


void ChRobot_free::SetSetpoints(const ChVectorDynamic<>& setpoints, double t) {
    for (int i = 0; i < m_motfunlist.size(); ++i)
        m_motfunlist[i]->SetSetpoint(setpoints[i], t);
}

void ChRobot_free::SetSetpoints(double setpoint, double t) {
    for (int i = 0; i < m_motfunlist.size(); ++i)
        m_motfunlist[i]->SetSetpoint(setpoint, t);
}

void ChRobot_free::SetBaseCoord(const ChCoordsys<>& base_coord) {
    for (auto& body : m_bodylist)
        body->ConcatenatePreTransformation(ChFrameMoving<>(base_coord));
    m_system->Update();
}

void ChRobot_free::SetColor(const ChColor& col) {
    for (int i = 0; i < m_bodylist.size(); ++i)
        if (m_bodylist[i]->GetVisualModel())
            for (int j = 0; j < m_bodylist[i]->GetVisualModel()->GetNumShapes(); ++j)
                m_bodylist[i]->GetVisualShape(j)->SetColor(col);
}

void ChRobot_free::AttachBody(std::shared_ptr<ChBody> slave, std::shared_ptr<ChBody> master, const ChFrame<>& frame) {
    m_body_attached = true;
    m_link_attach->Initialize(slave, m_bodylist.back(), frame);
    slave->SetFixed(false);
    m_system->AddLink(m_link_attach);
    m_system->Update();
}

void ChRobot_free::DetachBody(std::shared_ptr<ChBody> slave, bool setfix) {
    if (m_body_attached) {
        m_body_attached = false;
        m_system->RemoveLink(m_link_attach);
        slave->SetFixed(setfix);
        m_system->Update();
    }
}

void ChRobot_free::UpdateEncoder() {
    m_encoder += (GetMotorsRot() - m_encoder_prev).cwiseAbs();
    m_encoder_prev = GetMotorsRot();
}

double ChRobot_free::GetMass() const {
    double mass = 0;
    for (auto& body : m_bodylist)
        mass += body->GetMass();
    return mass;
}

void ChRobot_free::ClearShapes() {
    for (auto& body : m_bodylist)
        body->GetVisualModel()->Clear();
}


} // end namespace industrial
} // end namespace chrono