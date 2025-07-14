

// ChRobot base class header

#ifndef CH_ROBOT_H
#define CH_ROBOT_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkMotor.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace industrial {


class  ChRobot {
//class CH_MODELS_API ChRobot {
public:
    /// Default constructor.
    ChRobot() {};

    /// Default destructor.
    virtual ~ChRobot() = default;

    /// Get the containing system.
    ChSystem* GetSystem() { return m_system; }

    /// Get the list of bodies in robot model.
    std::vector<std::shared_ptr<ChBody>> GetBodylist() { return m_bodylist; }

    /// Get the list of joint markers in robot model.
    std::vector<std::shared_ptr<ChMarker>> GetMarkerlist() { return m_markerlist; }

    /// Get the list of motor functions in robot model.
    std::vector<std::shared_ptr<ChFunctionSetpoint>> GetMotfunlist() { return m_motfunlist; }

    /// Get the lsit of motors in robot model.
    std::vector<std::shared_ptr<ChLinkMotor>> GetMotorlist() { return m_motorlist; }

    /// Get motors rotations/positions, depending on specific robot architecture.
    virtual ChVectorDynamic<> GetMotorsRot() const = 0;

    /// Get motors angular velocities/linear velocities, depending on specific robot architecture.
    virtual ChVectorDynamic<> GetMotorsRot_dt() const = 0;

    /// Get motors angular accelerations/linear accelerations, depending on specific robot architecture.
    virtual ChVectorDynamic<> GetMotorsRot_dtdt() const = 0;

    /// Get motors torques/forces, depending on specific robot architecture.
    virtual ChVectorDynamic<> GetMotorsTorque() const = 0;

    /// Get motors travelled distance. 
    /// NB: needs runtime call to UpdateEncoder() function to be kept updated.
    ChVectorDynamic<> GetEncoderReading() const { return m_encoder; }

    /// Get overall robot mass.
    virtual double GetMass() const;

    /// Set robot base coordinates and consequently update other internal frames.
    virtual void SetBaseCoord(const ChCoordsys<>& base_coord);

    /// Set individual motors setpoints at given time.
    void SetSetpoints(const ChVectorDynamic<>& setpoints, double t);

    /// Set motors setpoints all equal to given value, at given time.
    void SetSetpoints(double setpoint, double t);
    
    /// Set robot color (if visual shapes are added).
    void SetColor(const ChColor& col);

    /// Kinematically link external body to a specific robot body, in given frame.
    /// Useful for grip and relocation of some object by robot end-effector.
    void AttachBody(std::shared_ptr<ChBody> slave, std::shared_ptr<ChBody> master, const ChFrame<>& frame);

    /// Unlink external body from robot (if previously attached) and potentially set it to 'fixed' thereafter.
    void DetachBody(std::shared_ptr<ChBody> slave, bool setfix = false);

    /// Update internal encoder to keep track of distance travelled by motors.
    /// NB: call this function at runtime and get reading with GetEncoderReading() function.
    virtual void UpdateEncoder();

    /// Add 1D visual shapes to model, representing robot arm as linear segments.
    virtual void Add1dShapes(const ChColor& col = ChColor(0.0f, 0.0f, 0.0f)) = 0;

    /// Add 3D visual shapes to model, loosely representing robot silhouette from given arm radius characteristic size.
    virtual void Add3dShapes(double rad, const ChColor& col = ChColor(0.2f, 0.2f, 0.2f)) = 0;

    /// Clear all visual shapes added to model.
    virtual void ClearShapes();

protected:
    ChSystem* m_system = nullptr;   ///< containing system
    ChCoordsys<> m_base_coord;      ///< coordinates of robot base

    std::vector<std::shared_ptr<ChBody>> m_bodylist;                ///< list of robot bodies
    std::vector<std::shared_ptr<ChMarker>> m_markerlist;            ///< list of robot joint markers
    std::vector<std::shared_ptr<ChFunctionSetpoint>> m_motfunlist; ///< list of robot motors functions
    std::vector<std::shared_ptr<ChLinkMotor>> m_motorlist;          ///< list of robot motors
    std::vector<ChCoordsys<>> m_joint_coords;                       ///< list of starting joint coordinates 

    std::shared_ptr<ChLinkMateFix> m_link_attach = nullptr; ///< internal link to grip external body
    bool m_body_attached = false;                           ///< flag for external body attached/not attached
    ChVectorDynamic<> m_encoder, m_encoder_prev;            ///< motors encoder internal data
};


} // end namespace industrial
} // end namespace chrono

#endif // end CH_ROBOT_H
