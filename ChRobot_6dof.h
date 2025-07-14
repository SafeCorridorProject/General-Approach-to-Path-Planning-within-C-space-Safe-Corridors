

// ChRobot_6dof class header

#ifndef CH_ROBOT_6DOF_H
#define CH_ROBOT_6DOF_H

#include "ChRobot.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"

namespace chrono {
namespace industrial {


class ChRobot_6dof : public ChRobot {
//class CH_MODELS_API ChRobot_6dof : public ChRobot {
public:
    /// Default constructor.
    ChRobot_6dof() {};

    /// Build 6dof articulated robot model from given arm lengths (as in scheme below) and add it to system.
    ///     L1   L2  L3
    ///   O----O---O--<
    /// H |
    ///   |
    ///  ---
    ChRobot_6dof(
        ChSystem* system,                           ///< containing system
        const std::array<double, 4>& lengths,       ///< arm lengths: shoulder height (H), biceps length (L1), forearm length (L2), wrist length up to TCP (L3)
        const ChCoordsys<>& base_coord = CSYSNORM   ///< place robot base in these coordinates
    );

    /// Get specific robot body.
    std::shared_ptr<ChBodyAuxRef> GetBase() { return m_base; };
    std::shared_ptr<ChBodyAuxRef> GetShoulder() { return m_shoulder; };
    std::shared_ptr<ChBodyAuxRef> GetBiceps() { return m_biceps; };
    std::shared_ptr<ChBodyAuxRef> GetForearm() { return m_forearm; };
    std::shared_ptr<ChBodyAuxRef> GetElbow() { return m_elbow; };
    std::shared_ptr<ChBodyAuxRef> GetWrist() { return m_wrist; };
    std::shared_ptr<ChBodyAuxRef> GetEndEffector() { return m_end_effector; };

    /// Get specific robot joint marker.
    std::shared_ptr<ChMarker> GetMarkerBaseShoulder() const { return m_marker_base_shoulder; }
    std::shared_ptr<ChMarker> GetMarkerShoulderBiceps() const { return m_marker_shoulder_biceps; }
    std::shared_ptr<ChMarker> GetMarkerBicepsElbow() const { return m_marker_biceps_elbow; }
    std::shared_ptr<ChMarker> GetMarkerElbowForearm() const { return m_marker_elbow_forearm; }
    std::shared_ptr<ChMarker> GetMarkerForearmWrist() const { return m_marker_forearm_wrist; }
    std::shared_ptr<ChMarker> GetMarkerWristEndeffector() const { return m_marker_wrist_end_effector; }
    std::shared_ptr<ChMarker> GetMarkerTCP() const { return m_marker_TCP; }

    /// Get specific robot motor.
    std::shared_ptr<ChLinkMotorRotationAngle> GeMotorBaseShoulder() { return m_link_base_shoulder; }
    std::shared_ptr<ChLinkMotorRotationAngle> GeMotorShoulderBiceps() { return m_link_shoulder_biceps; }
    std::shared_ptr<ChLinkMotorRotationAngle> GeMotorBicepsElbow() { return m_link_biceps_elbow; }
    std::shared_ptr<ChLinkMotorRotationAngle> GeMotorElbowForearm() { return m_link_elbow_forearm; }
    std::shared_ptr<ChLinkMotorRotationAngle> GeMotorForearmWrist() { return m_link_forearm_wrist; }
    std::shared_ptr<ChLinkMotorRotationAngle> GeMotorWristEndeffector() { return m_link_wrist_end_effector; }

    /// Get motors diplacement/velocity/acceleration/torque data.
    virtual ChVectorDynamic<> GetMotorsRot() const override;
    virtual ChVectorDynamic<> GetMotorsRot_dt() const override;
    virtual ChVectorDynamic<> GetMotorsRot_dtdt() const override;
    virtual ChVectorDynamic<> GetMotorsTorque() const override;

    /// Get robot arm lengths.
    std::array<double, 4> GetLengths() const { return m_lengths; }

    /// Add 1D visual shapes to model, representing robot arm as linear segments.
    virtual void Add1dShapes(const ChColor& col = ChColor(0.0f, 0.0f, 0.0f)) override;

    /// Add 3D visual shapes to model, loosely representing robot silhouette from given arm radius characteristic size.
    virtual void Add3dShapes(double rad, const ChColor& col = ChColor(0.2f, 0.2f, 0.2f)) override;

protected:
    /// Setup robot internal bodies, markers and motors.
    virtual void SetupBodies();
    virtual void SetupMarkers();
    virtual void SetupLinks();

    std::array<double, 4> m_lengths = { 0,0,0,0 }; ///< robot arm lengths (H, L1, L2, L3)
    std::shared_ptr<ChBodyAuxRef> m_base, m_shoulder, m_biceps, m_elbow, m_forearm, m_wrist, m_end_effector;
    std::shared_ptr<ChMarker> m_marker_base_shoulder, m_marker_shoulder_biceps, m_marker_biceps_elbow, m_marker_elbow_forearm, m_marker_forearm_wrist, m_marker_wrist_end_effector, m_marker_TCP;
    std::shared_ptr<ChLinkMotorRotationAngle> m_link_base_shoulder, m_link_shoulder_biceps, m_link_biceps_elbow, m_link_elbow_forearm, m_link_forearm_wrist, m_link_wrist_end_effector;
};


} // end namespace industrial
} // end namespace chrono

#endif // end CH_ROBOT_6DOF_H
