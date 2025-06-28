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

// ChRobot_6dof_CAD class header

#ifndef CH_ROBOT_6DOF_CAD_H
#define CH_ROBOT_6DOF_CAD_H

#include "ChRobot_6dof.h"

namespace chrono {
namespace industrial {


class ChRobot_6dof_CAD : public ChRobot_6dof {
//class CH_MODELS_API ChRobot_6dof_CAD : public ChRobot_6dof {
public:
    /// Default constructor.
    ChRobot_6dof_CAD() {};

    /// Build 6dof articulated robot model from CAD bodies already imported in system.
    ChRobot_6dof_CAD(
        ChSystem* system,                           ///< containing system
        const ChCoordsys<>& base_coord = CSYSNORM,  ///< place robot base in these coordinates
        unsigned int id = 0,                        ///< give robot a unique identifier (useful to import multiple instances of same CAD robot without name clashes)
        //std::vector<std::string> bodynames = { "base", "shoulder", "biceps", "elbow", "forearm", "wrist", "end_effector" } ///< name of bodies to search in system for building robot model
        std::vector<std::string> bodynames = {"Base_UR5_STEP", "Link1_UR5_STEP", "Link2_UR5_STEP", "Link3_UR5_STEP",  "Link4_UR5_STEP_1000515", "Link5_UR5_STEP", "End_effector_assembly2"/*"Link6_UR5_STEP"*/}                                                                                                                  ///  NB: if 'base' body is not provided here, robot arm is linked to internal 'ground' instead (fallback)
    );

    /// Get robot 'ground' body.
    /// NB: this is a "virtual" body that is automatically imported from Chrono::Solidworks plugin
    /// and represents the overall assembly fixed parent body (i.e. not an actual robot body).
    std::shared_ptr<ChBody> GetGround() { return m_ground; }

    /// Suppress parent class functions to add schematic visual shapes, since actual
    /// 3D body meshes are already imported.
    virtual void Add1dShapes(const ChColor& col = ChColor(0.0f, 0.0f, 0.0f)) override {};
    virtual void Add3dShapes(double rad, const ChColor& col = ChColor(0.2f, 0.2f, 0.2f)) override {};

private:
    /// Setup robot internal bodies, markers and motors.
    virtual void SetupBodies() override;
    virtual void SetupMarkers() override;
    virtual void SetupLinks() override;

    unsigned int m_id = 0;                  ///< robot model unique identifier
    std::vector<std::string> m_bodynames;   ///< name of bodies to search in system for building robot model
    std::shared_ptr<ChBodyAuxRef> m_ground; ///< robot 'ground' virtual body
};


} // end namespace industrial
} // end namespace chrono

#endif // end CH_ROBOT_6DOF_CAD_H