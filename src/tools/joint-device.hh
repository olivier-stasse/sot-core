/*
 * Copyright 2018, CNRS
 * Olivier Stasse
 *
 * CNRS
 *
 * See LICENSE.txt
 */

#ifndef SOT_JOINT_DEVICE_HH
#define SOT_JOINT_DEVICE_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include "sot/core/periodic-call.hh"
#include <sot/core/matrix-geometry.hh>
#include "sot/core/api.hh"

namespace dynamicgraph{
  namespace sot {
    /// Per Joint type
    
    /// Specifies the nature of one joint control
    /// It is used for both the SoT side and the hardware side.
    enum ControlType
      {
	POSITION=0,
	VELOCITY=1,
	ACCELERATION=2,
	TORQUE=3
      };

    /// Enum joint type
    enum JointType
      {
	FREE_FLYER=0,
	REVOLUTE=1,
	LINEAR=2
     };
    
    /// \brief Handle joint by joint the limits, and the integration.
    /// There is a difference between the SoT control side
    /// and the hardware side. Often the control is in velocity,
    /// effort/acceleration and the hardware control is in position.
    template<JointType temp_JointType>
    class JointDeviceVectorElement
    {
    protected:
      /// \brief Sanity check.
      bool sanityCheck_;
      /// \brief Boolean to verify sanity check.
      unsigned int sanityCheckFlags_;
      /// \brief Joint Name.
      string jointName_;
      /// \brief Control type from the SoT side.
      ControlType sotControlType_;
      /// \brief Hardware control type 
      ControlType hwControlType_;
      /// \brief Joint type
      JointType jointType_;
      /// \brief Size of the control information
      unsigned int size_;
      /// \brief Position of the joint state in the global state vector.
      unsigned int stateVectorPos_;
      /// \brief Position of the joint velocity in the global velocity vector.
      unsigned int velocityVectorPos_;
      /// \brief Position of the joint control in the global control vector.
      unsigned int controlVectorPos_;

      /// \brief Upper, lower velocity bounds;
      double lowerPos_,upperPos_,lowerVel_,upperVel_,
	lowerTorque_,upperTorque_;

    public:
      /// \name Set control type for the SoT side and the Hardware side.
      ///@{
      void setSotControlType(ControlType aControlType);
      void setHWControlType(ControlType aControlType);
      ///@}

      void setJointType(JointType aJointType);
      void setStateVectorPos(unsigned int aStateVectorPos_);
      void setVelocityVectorPos(unsigned int aVelocityVectorPos_);
      
      /// Integrate the control part 
      void integrate(const double &dt);

      /// \name Set limits
      /// Here we assume that Joint 
      ///@{
      /// \brief Set position limits
      void setPositionBounds(const double &lower, const double &upper);
      /// \brief Set velocity limits
      void setVelocityBounds(const double &lower, const double &upper);
      /// \brief Set torque limits
      void setTorqueBounds(const double &lower, const double &upper);
      /// \brief Modifies the control part
      void saturateBounds();
      /// \brief Check the boundaries and displays the problematic joint
      void checkBounds();

      ///@}
    };

  } //namespace sot
} // namespace dynamicgraph


#endif

