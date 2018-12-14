/*
 * Copyright 2010-2018, CNRS
 * Florent Lamiraux
 * Olivier Stasse
 *
 * CNRS
 *
 * See LICENSE.txt
 */

#ifndef SOT_DEVICE_HH
#define SOT_DEVICE_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <urdf_parser/urdf_parser.h>

/* -- MaaL --- */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include "sot/core/periodic-call.hh"
#include <sot/core/matrix-geometry.hh>
#include "sot/core/api.hh"
#include "sot/core/joint-device.hh"
#include <pinocchio/multibody/liegroup/special-euclidean.hpp>

namespace dynamicgraph {
  namespace sot {

    /// Deprecated:
    //@{
    /// Define the type of input expected by the robot
    enum ControlInput
    {
      CONTROL_INPUT_NO_INTEGRATION=0,
      CONTROL_INPUT_ONE_INTEGRATION=1,
      CONTROL_INPUT_TWO_INTEGRATION=2,
      CONTROL_INPUT_SIZE=3
    };

    const std::string ControlInput_s[] =
    {
      "noInteg", "oneInteg", "twoInteg"
    };
    //@}

    

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    class SOT_CORE_EXPORT Device
        :public Entity
    {
    public:
      static const std::string CLASS_NAME;
      virtual const std::string& getClassName(void) const {
        return CLASS_NAME;
      }

      enum ForceSignalSource
      {
        FORCE_SIGNAL_RLEG,
        FORCE_SIGNAL_LLEG,
        FORCE_SIGNAL_RARM,
        FORCE_SIGNAL_LARM
      };

      friend class JointDeviceVectorElement<FREE_FLYER>;
      friend class JointDeviceVectorElement<REVOLUTE>;
      friend class JointDeviceVectorElement<LINEAR>;
      
    protected:

      /// \name Vectors related to the state.
      ///@{
      /// State vector of the robot (deprecated)
      dg::Vector state_;

      /// Position of each actuator
      dg::Vector position_;
      /// Velocity vector of each actuator.
      dg::Vector velocity_;
      /// Acceleration vector of each actuator.
      dg::Vector acceleration_;
      /// Torque vector of each actuator.
      dg::Vector torque_;
      ///@}

      bool sanityCheck_;
      
      dg::Vector vel_control_;

      /// Specifies the control input by each element of the state vector.
      ControlInput controlInputType_;
      std::map<std::string,ControlType> sotControlType_;
      std::map<std::string,ControlType> hwControlType_;

      /// Maps of joint devices.
      std::map<std::string,JointDeviceVariant> jointDevices_;
      /// 
      bool withForceSignals[4];
      PeriodicCall periodicCallBefore_;
      PeriodicCall periodicCallAfter_;

      /// \name Robot bounds used for sanity checks
      /// \{
      Vector upperPosition_;
      Vector upperVelocity_;
      Vector upperTorque_;
      Vector lowerPosition_;
      Vector lowerVelocity_;
      Vector lowerTorque_;
      /// \}
    public:

      /* --- CONSTRUCTION --- */
      Device(const std::string& name);
      /* --- DESTRUCTION --- */
      virtual ~Device();

      virtual void setStateSize(const unsigned int& size);
      virtual void setState(const dg::Vector& st);
      void setVelocitySize(const unsigned int& size);
      virtual void setVelocity(const dg::Vector & vel);
      virtual void setSecondOrderIntegration();
      virtual void setNoIntegration();
      /// Set control input type.
      virtual void setControlInputType(const std::string& cit);
      virtual void setSoTControlType(const std::string &jointNames,
				     const std::string &sotCtrlType);
      virtual void setHWControlType(const std::string &jointNames,
				    const std::string &hwCtrlType);
      virtual void increment(const double & dt = 5e-2);
      /// Read directly the URDF model
      void setURDFModel(std::string &aURDFModel);
      
      /// \name Sanity check parameterization
      /// \{
      void setSanityCheck   (const bool & enableCheck);
      void setPositionBounds(const Vector& lower, const Vector& upper);
      void setVelocityBounds(const Vector& lower, const Vector& upper);
      void setTorqueBounds  (const Vector& lower, const Vector& upper);
      /// \}

      /// \name Set index in vector (position, velocity, acceleration, control)
      /// \{
      void setControlPos(const std::string &jointName,
			 const unsigned & index);
      void setPositionPos(const std::string &jointName,
			  const unsigned & index);
      void setVelocityPos(const std::string &jointName,
			  const unsigned & index);
      void setAccelerationPos(const std::string &jointName,
			      const unsigned & index);
      /// \}
    public: /* --- DISPLAY --- */
      virtual void display(std::ostream& os) const;
      SOT_CORE_EXPORT friend std::ostream&
      operator<<(std::ostream& os,const Device& r) {
        r.display(os); return os;
      }

    public: /* --- SIGNALS --- */

      /// Input signal handling the control vector
      /// This entity needs a control vector to be send to the hardware.
      /// The control vector can be position, velocity and effort.
      /// It depends on each of the actuator
      dynamicgraph::SignalPtr<dg::Vector,int> controlSIN;

      /// \name This part is specific to robot where a stabilizer is provided outside the
      /// SoT framework and needs input.
      /// @{ 
      /// Input signal handling the attitude of the freeflyer.
      dynamicgraph::SignalPtr<dg::Vector,int> attitudeSIN;
      /// Input signal handling the ZMP of the system 
      dynamicgraph::SignalPtr<dg::Vector,int> zmpSIN;
      ///@}
      
      /// \name Device current state.
      /// \{
      /// \brief Output integrated state from control.
      dynamicgraph::Signal<dg::Vector,int> stateSOUT;
      /// \brief Output integrated velocity from control
      dynamicgraph::Signal<dg::Vector,int> velocitySOUT;
      /// \brief Output attitude provided by the hardware
      /// Typically this can be provided by an external estimator
      /// such an integrated/hardware implemented EKF.
      dynamicgraph::Signal<MatrixRotation,int> attitudeSOUT;
      /*! \brief The current state of the robot from the command viewpoint. */
      dynamicgraph::Signal<dg::Vector,int> motorcontrolSOUT;
      dynamicgraph::Signal<dg::Vector,int> previousControlSOUT;
      /*! \brief The ZMP reference send by the previous controller. */
      dynamicgraph::Signal<dg::Vector,int> ZMPPreviousControllerSOUT;
      /// \}

      /// \name Real robot current state
      /// This corresponds to the real encoders values and take into
      /// account the stabilization step. Therefore, this usually
      /// does *not* match the state control input signal.
      /// \{
      /// Motor positions
      dynamicgraph::Signal<dg::Vector, int> robotState_;
      /// Motor velocities
      dynamicgraph::Signal<dg::Vector, int> robotVelocity_;
      /// The force torque sensors
      dynamicgraph::Signal<dg::Vector,int>* forcesSOUT[4];
      /// Motor torques
      /// \todo why pseudo ?
      dynamicgraph::Signal<dg::Vector,int> pseudoTorqueSOUT;
      /// \}

    protected:
      void setControlType(const std::string &jointNames,
			  const std::string &strCtrlType,
			  std::map<std::string, ControlType> &aCtrlType);
      
      /// Compute roll pitch yaw angles of freeflyer joint.
      void integrateRollPitchYaw(dg::Vector& state, const dg::Vector& control,
                                 double dt);
      /// Store Position of free flyer joint
      MatrixHomogeneous ffPose_;
      /// Compute the new position, from the current control.
      ///
      /// When sanity checks are enabled, this checks that the control is within
      /// bounds. There are three cases, depending on what the control is:
      /// - position: checks that the position is within bounds,
      /// - velocity: checks that the velocity and the future position are
      ///             within bounds,
      /// - acceleration: checks that the acceleration, the future velocity and
      ///                 position are within bounds.
      ///                 \todo in order to check the acceleration, we need
      ///                 pinocchio and the contact forces in order to estimate
      ///                 the joint torques for the given acceleration.
      virtual void integrate( const double & dt );
    protected:
      /// Get freeflyer pose
      const MatrixHomogeneous& freeFlyerPose() const;
    public:
      virtual void setRoot( const dg::Matrix & root );


      virtual void setRoot( const MatrixHomogeneous & worldMwaist );
    private:
      // Intermediate variable to avoid dynamic allocation
      dg::Vector forceZero6;

      // URDF Model of the robot
      urdf::ModelInterfaceSharedPtr modelURDF_;
      
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef SOT_DEVICE_HH */




