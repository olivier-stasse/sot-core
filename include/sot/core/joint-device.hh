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

#include <boost/variant.hpp>

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

    class Device;
    
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

    const std::string ControlType_s[] =
      {
	"POSITION","VELOCITY","ACCELERATION","TORQUE"
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
      std::string jointName_;
      /// \brief Control type from the SoT side.
      ControlType sotControlType_;
      /// \brief Hardware control type 
      ControlType hwControlType_;
      /// \brief Joint type
      JointType jointType_;

      /// Vector of velocity.
      dg::Vector velocity_;
      
      /// \brief Size of the control information
      unsigned int size_;
      /// \brief Position of the joint state in the global position vector.
      unsigned int positionVectorPos_;
      /// \brief Position of the joint velocity in the global velocity vector.
      unsigned int velocityVectorPos_;
      /// \brief Position of the joint control in the global control vector.
      unsigned int controlVectorPos_;
      /// \brief Acceleration of the joint control in the global acceleration vector.
      unsigned int accelerationVectorPos_;

      /// \brief Upper, lower velocity bounds;
      dg::Vector lowerPos_,upperPos_,lowerVel_,upperVel_,
	lowerTorque_,upperTorque_;

    public:
      // Contrustor.
      JointDeviceVectorElement(unsigned int lsize=1);

      /// \name Set joint name.
      void setJointName(const std::string &aname);
      
      /// \name Set control type for the SoT side and the Hardware side.
      ///@{
      void setSotControlType(ControlType aControlType);
      void setHWControlType(ControlType aControlType);
      ///@}

      /// \name Set the position of the actuator in their respective vector
      /// @{
      /// \brief Position vector position
      void setPositionVectorPos(unsigned int aStateVectorPos);
      /// \brief Velocity vector position
      void setVelocityVectorPos(unsigned int aVelocityVectorPos);
      /// \brief Acceleration vector position
      void setAccelerationVectorPos(unsigned int anAccelerationVectorPos);
      /// @}
      
      void setControlVectorPos(unsigned int aControlVectorPos);
      /// Integrate the control part 
      void integrate(Device &aDevice, const double &dt);
      const JointType & getJointType();
      
      /// \name Handle bounds
      /// @{
      /// \brief Saturate bounds
      bool saturateBounds(double &val,
			  const double& lower,
			  const double& upper);
      /// \brief Check bounds
      void checkBounds(dg::Vector &val, unsigned int start,
		       dg::Vector &lower,
		       dg::Vector &upper,
		       const std::string &what);
      
      /// \name Set limits
      /// Here we assume that Joint 
      ///@{
      /// \brief Set position limits
      void setPositionBounds(const Vector &lower, const Vector &upper);
      /// \brief Set velocity limits
      void setVelocityBounds(const Vector &lower, const Vector &upper);
      /// \brief Set torque limits
      void setTorqueBounds(const Vector &lower, const Vector &upper);
      /// \brief Modifies the control part
      void saturateBounds();
      /// \brief Check the boundaries and displays the problematic joint
      void checkBounds();

      ///@}
    };

    typedef JointDeviceVectorElement<FREE_FLYER> JointDeviceFreeFlyer;
    typedef JointDeviceVectorElement<REVOLUTE> JointDeviceRevolute;
    typedef JointDeviceVectorElement<LINEAR> JointDeviceLinear;
    
    typedef boost::variant<JointDeviceFreeFlyer,
			   JointDeviceRevolute,
			   JointDeviceLinear>
    JointDeviceVariant;
  } //namespace sot
} // namespace dynamicgraph


#endif

