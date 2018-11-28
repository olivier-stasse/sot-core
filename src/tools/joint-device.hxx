/*
 * Copyright 2018, CNRS
 * Olivier Stasse
 *
 * CNRS
 *
 * See LICENSE.txt
 */

#ifndef SOT_JOINT_DEVICE_HXX
#define SOT_JOINT_DEVICE_HXX

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

    enum SanityCheckFlagsType
      {
	SOT_CONTROL_TYPE=0,
	HW_CONTROL_TYPE=1,
	JOINT_TYPE=2,
	STATE_VECTOR_POS=3,
	CONTROL_VECTOR_POS=4,
	VELOCITY_VECTOR_POS=5,
	POSITION_BOUNDS=6,
	VELOCITY_BOUNDS=7,
	TORQUE_BOUNDS=8
      }
      
    template<JointType temp_JointType>
    JointDeviceVectorElement<temp_JointType>::
    JointDeviceVectorElement():
      sanityCheck(true),
      sanityCheckFlags_(0),
      sotControlType_(VELOCITY),
      hwControlType_(POSITION),
      jointType_(temp_JointType),
      size_(1),
      stateVectorPos_(0),
      velocityVectorPos_(0),
      lowerPos_(-3.14),
      upperPos_(3.14),
      lowerVel_(-3.14),
      upperVel_(3.14),
      lowerTorque_(-20),
      upperTorque_(20)
    {
      
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setSotControlType(ControlType aControlType)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< SOT_CONTROL_TYPE);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;
      sotControlType_ = aControlType;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setHWControlType(ControlType aControlType)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< HW_CONTROL_TYPE);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;
      
      hwControlType_ = aControlType;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setJointType(JointType aJointType)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< JOINT_TYPE);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;

      jointType_ = aJointType;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setStateVectorPos(unsigned int aStateVectorPos)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< STATE_VECTOR_POS);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;
      
      stateVectorPos_(aStateVectorPos);
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setVelocityVectorPos(unsigned int aVelocityVectorPos)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< VELOCITY_VECTOR_POS);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;
      
      velocityVectorPos_(aVelocityVectorPos);
    }
    
    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setVelocityVectorPos(unsigned int aControlVectorPos)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< CONTROL_VECTOR_POS);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;
      
      controlVectorPos_(aControlVectorPos);
    }
    
    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setPositionBounds(const double &lower, const double &upper)
    {
      lowerPos_ = lower; upperPos_ = upper;
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< POSITION_BOUNDS);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;
    }
    
    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setVelocityBounds(const double &lower, const double &upper)
    {
      lowerVel_ = lower; upperVel_ = upper;
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< VELOCITY_BOUNDS);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setTorqueBounds(const double &lower, const double &upper)
    {
      lowerTorque_ = lower; upperTorque_ = upper;
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< TORQUE_BOUNDS);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;      
    }
    
    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    checkBounds()
    {
      lowerTorque_ = lower; upperTorque_ = upper;
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< TORQUE_BOUNDS);
      std::cout << std::hex << sanityCheckFlags_ << std::endl;      
    }

    template<FREE_FLYER>
    void JointDeviceVectorElement<FREE_FLYER>::
    integrate(Device &aDevice, const double &dt)
    {
      if ((sotControlType_==VELOCITY)&&(hwControlType_==POSITION))
	{
	  using Eigen::AngleAxisd;
	  using Eigen::Vector3d;
	  using Eigen::QuaternionMapd;
	  
	  /// Creates references to good vectors.
	  const Vector & control = aDevice.controlSIN.accessCopy();
	  Vector & state = aDevice.state_;
	  MatrixHomogeneous & ffPose = aDevice.ffPose_;
	  
	  typedef se3::SpecialEuclideanOperation<3> SE3;
	  Eigen::Matrix<double, 7, 1> qin, qout;
	  qin.head<3>() = state.segment<3>(stateVectorPos_);
	  
	  QuaternionMapd quat (qin.tail<4>().data());
	  quat = AngleAxisd(state(5), Vector3d::UnitZ())
	    * AngleAxisd(state(4), Vector3d::UnitY())
	    * AngleAxisd(state(3), Vector3d::UnitX());
	  
	  SE3().integrate (qin, control.segment<6>(controlVectorPos_)*dt, qout);
	  
	  // Update freeflyer pose
	  ffPose.translation() = qout.head<3>();
	  state.segment<3>(stateVectorPos_) = qout.head<3>();
	  
	  ffPose.linear() = QuaternionMapd(qout.tail<4>().data()).toRotationMatrix();
	  state.segment<3>(stateVectorPos_+3) =
	    ffPose_.linear().eulerAngles(2,1,0).reverse();
	}
    }

    template<JointType aJointType>
    void JointDeviceVectorElement<aJointType>::
    integrate(Device &aDevice, const double &dt)
    {
      Vector & state = aDevice.state_;

      if (sotControlType_==TORQUE)
	{
	  /// Checking torque boundaries
	  if ( (controlIN(controlVectorPos_) > upperTorque_(stateVectorPos_)) ||
	       (controlIN(controlVectorPos_) < lowerTorque_(stateVectorPos_)))
	    {
	      dgRTLOG () << "Robot torque bound violation at DoF " << i << 
		": requested " << old << " but set " << val(i) << '\n';	
	    }
	}


      if (sotControlType_==ACCELERATION)
	{
	  /// Integrate acceleration
	  vel_control_ = velocity_(velocityVectorPos_) +
	    (0.5*dt)*controlIN(controlVectorPos_);
	  // Velocity integration.
	  velocity_(velocityVectorPos_) += controlIN(controlVectorPos_)*dt;

	  /// Check velocity
	  if ( (lowerVel_ > velocity_(velocityVectorPos_)) &&
	       (upperVel_ < velocity_(velocityVectorPos_)))
	    {
	      dgRTLOG () << "Robot velocity bound violation at DoF " << i << 
		": requested " << old << " but set " << val(i) << '\n';	
	    }
	}

      if (sotControlType_==VELOCITY)
	{ vel_control_(velocityVectorPos_) = controlIN(controlVectorPos_);}
      
      if ((sotControlType_==VELOCITY) || (sotControlType_==ACCELERATION))
	{
	  state(stateVectorPos_) +=
	    vel_control_(velocityVectorPos_) *dt;
	}
	  
      if (sotControlType_==POSITION)
	{
	  state(stateVectorPos_)= controlIN(controlVectorPos_);
	}
      
      if ( (state_ > lowerPosition_(stateVectorPos_)) &&
	   (state_ < upperPosition_(stateVectorPos_)))
	{
	  dgRTLOG () << "Robot position bound violation at DoF " << i << 
	    ": requested " << old << " but set " << val(i) << '\n';	
	}
      
    }
    

  } //namespace sot
} // namespace dynamicgraph


#endif // SOT_JOINT_DEVICE_HXX

