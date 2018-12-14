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

#include <iostream>

#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/real-time-logger.h>
#include "sot/core/periodic-call.hh"
#include <sot/core/matrix-geometry.hh>
#include "sot/core/debug.hh"
#include "sot/core/api.hh"

#include "sot/core/joint-device.hh"
#include "sot/core/device.hh"
namespace dynamicgraph{
  namespace sot {

    enum SanityCheckFlagsType
      {
	SOT_CONTROL_TYPE=0,
	HW_CONTROL_TYPE=1,
	STATE_VECTOR_POS=2,
	CONTROL_VECTOR_POS=3,
	VELOCITY_VECTOR_POS=4,
	ACCELERATION_VECTOR_POS=5,
	POSITION_BOUNDS=6,
	VELOCITY_BOUNDS=7,
	TORQUE_BOUNDS=8
      };
      
    template<JointType temp_JointType>
    JointDeviceVectorElement<temp_JointType>::
    JointDeviceVectorElement(unsigned int lsize):
      sanityCheckFlags_(0),
      sotControlType_(VELOCITY),
      hwControlType_(POSITION),
      jointType_(temp_JointType),
      size_(lsize),
      positionVectorPos_(0),
      velocityVectorPos_(0),
      accelerationVectorPos_(0),      
      lowerPos_(size_),
      upperPos_(size_),
      lowerVel_(size_),
      upperVel_(size_),
      lowerTorque_(size_),
      upperTorque_(size_)
    {
      
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setJointName(const std::string &aJointName)
    {
      jointName_ = aJointName;
    }
    
    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setSotControlType(ControlType aControlType)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< SOT_CONTROL_TYPE);
      sotControlType_ = aControlType;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setHWControlType(ControlType aControlType)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< HW_CONTROL_TYPE);
      
      hwControlType_ = aControlType;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setPositionVectorPos(unsigned int aPositionVectorPos)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< STATE_VECTOR_POS);
      
      positionVectorPos_ = aPositionVectorPos;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setVelocityVectorPos(unsigned int aVelocityVectorPos)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< VELOCITY_VECTOR_POS);
      
      velocityVectorPos_= aVelocityVectorPos;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setAccelerationVectorPos(unsigned int anAccelerationVectorPos)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< ACCELERATION_VECTOR_POS);
      
      accelerationVectorPos_= anAccelerationVectorPos;
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setControlVectorPos(unsigned int aControlVectorPos)
    {
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< CONTROL_VECTOR_POS);
      
      controlVectorPos_=aControlVectorPos;
    }
    
    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setPositionBounds(const Vector &lower, const Vector &upper)
    {
      lowerPos_ = lower; upperPos_ = upper;
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< POSITION_BOUNDS);
    }
    
    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setVelocityBounds(const Vector &lower, const Vector &upper)
    {
      lowerVel_ = lower; upperVel_ = upper;
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< VELOCITY_BOUNDS);
    }

    template<JointType temp_JointType>
    void JointDeviceVectorElement<temp_JointType>::
    setTorqueBounds(const Vector &lower, const Vector &upper)
    {
      lowerTorque_ = lower; upperTorque_ = upper;
      sanityCheckFlags_ = sanityCheckFlags_ | ( 1<< TORQUE_BOUNDS);
    }



      
    template<>
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
	  Vector & state = aDevice.position_;
	  MatrixHomogeneous & ffPose = aDevice.ffPose_;
	  
	  typedef se3::SpecialEuclideanOperation<3> SE3;
	  Eigen::Matrix<double, 7, 1> qin, qout;
	  qin.head<3>() = state.segment<3>(positionVectorPos_);
	  
	  QuaternionMapd quat (qin.tail<4>().data());
	  quat = AngleAxisd(state(5), Vector3d::UnitZ())
	    * AngleAxisd(state(4), Vector3d::UnitY())
	    * AngleAxisd(state(3), Vector3d::UnitX());
	  
	  SE3().integrate (qin, control.segment<6>(controlVectorPos_)*dt, qout);
	  
	  // Update freeflyer pose
	  ffPose.translation() = qout.head<3>();
	  state.segment<3>(positionVectorPos_) = qout.head<3>();
	  
	  ffPose.linear() = QuaternionMapd(qout.tail<4>().data()).toRotationMatrix();
	  state.segment<3>(positionVectorPos_+3) =
	    ffPose.linear().eulerAngles(2,1,0).reverse();
	  //	  std::cout << "ffPose.translation(): "<< ffPose.translation() << std::endl;
	  //std::cout << "ffPose.rotation(): "<< ffPose.rotation() << std::endl;
	}
    }

    template<JointType aJointType>
    const JointType & JointDeviceVectorElement<aJointType>::
    getJointType()
    {
      return jointType_;
    }
    // Return true if it saturates.
    template<JointType aJointType>
    bool JointDeviceVectorElement<aJointType>::
    saturateBounds (double& val, const double& lower, const double& upper)
    {
      assert (lower <= upper);
      if (val < lower) { val = lower; return true; }
      if (upper < val) { val = upper; return true; }
      return false;
    }
    
    template<JointType aJointType>
    void JointDeviceVectorElement<aJointType>::
    checkBounds(dg::Vector &val,
		unsigned int start,
		dg::Vector &lower,
		dg::Vector &upper,
		const std::string &what)
    {
      std::cout << "Checking bound for joint "
		<< jointName_ << ": ("
		<< lower(0) << ","
		<< upper(0) << ") val:"
		<< val(start) << " size: "
		<< size_
		<< std::endl;
      unsigned int j=0;
      for (unsigned int i = start; i < start+size_; ++i) {			
	double old = val(i);						
	if (saturateBounds (val(i), lower(j), upper(j)))
	  {
	    dgRTLOG () << "Robot " << what << " bound violation at DoF " << i <<	
	      ": requested " << old << " but set " << val(i) << '\n';
	    std::cout <<"Robot " << what << " bound violation at DoF " << i <<	
	      ": requested " << old << " but set " << val(i) << '\n';
	  }
	j++;
      }
    }

    template<JointType aJointType>
    void JointDeviceVectorElement<aJointType>::
    integrate(Device &aDevice, const double &dt)
    {
      Vector & position = aDevice.position_;
      const Vector & controlIN = aDevice.controlSIN.accessCopy();      
      Vector & velocity = aDevice.velocity_;
      Vector & acceleration = aDevice.acceleration_;
      Vector & torque = aDevice.torque_;
      std::cout << "integrate for :" << jointName_ << " sot:"
		<< sotControlType_ << " hw:" << hwControlType_ <<  std::endl;
      if (sotControlType_==TORQUE)
	{
	  checkBounds(torque,
		      controlVectorPos_,
		      lowerTorque_,
		      upperTorque_,
		      "torque");
	}


      if (sotControlType_==ACCELERATION)
	{
	  /// Set acceleration
	  acceleration(accelerationVectorPos_) = controlIN(controlVectorPos_);
	  
	  /// Integrate acceleration
	  velocity(velocityVectorPos_) = velocity(velocityVectorPos_) +
	    dt*controlIN(controlVectorPos_);

	  /// Check velocity
	  checkBounds(velocity,
		      velocityVectorPos_,
		      lowerVel_,
		      upperVel_,
		      "velocity");

	}

      if (sotControlType_==VELOCITY)
	{ velocity(velocityVectorPos_) = controlIN(controlVectorPos_);}
      
      if ((sotControlType_==VELOCITY) || (sotControlType_==ACCELERATION))
	{
	  position(positionVectorPos_) +=
	    velocity(velocityVectorPos_) *dt;
	  std::cout << "position control("<< positionVectorPos_  
		    << ") =" << position(positionVectorPos_)
		    << std::endl;
	}
	  
      if (sotControlType_==POSITION)
	{
	  position(positionVectorPos_)= controlIN(controlVectorPos_);
	}

      checkBounds(position,
		  positionVectorPos_,
		  lowerPos_, 
		  upperPos_,
		  "position");
      
      
    }

    struct JointDeviceVisitorIntegrate: public boost::static_visitor<>
    {
      Device & device_;
      const double & dt_;

      JointDeviceVisitorIntegrate(Device & aDevice_,
				  const double & dt):
	device_(aDevice_),
	dt_(dt)
      {}
      
      void operator()(JointDeviceFreeFlyer &aJointDeviceFreeFlyer)
      const
      {	aJointDeviceFreeFlyer.integrate(device_,dt_); }

      void operator()(JointDeviceRevolute &aJointDeviceRevolute)
      const
      {	aJointDeviceRevolute.integrate(device_,dt_); }
      
      void operator()(JointDeviceLinear &aJointDeviceLinear)
      const
      {	aJointDeviceLinear.integrate(device_,dt_); }
    };

    struct JointDeviceVisitorSetPositionBounds: public boost::static_visitor<>
    {
      const Vector & lower_;
      const Vector & upper_;

      JointDeviceVisitorSetPositionBounds(const Vector & lower,
					     const Vector & upper):
	lower_(lower),
	upper_(upper)
      {}

      template <typename T> 
      void operator()(T &aJointDeviceVariant)
      const
      {	aJointDeviceVariant.setPositionBounds(lower_,upper_); }

    };

    struct JointDeviceVisitorSetVelocityBounds: public boost::static_visitor<>
    {
      const Vector & lower_;
      const Vector & upper_;

      JointDeviceVisitorSetVelocityBounds(const Vector & lower,
					     const Vector & upper):
	lower_(lower),
	upper_(upper)
      {}

      template <typename T> 
      void operator()(T &aJointDeviceVariant)
      const
      {	aJointDeviceVariant.setVelocityBounds(lower_,upper_); }

    };

    struct JointDeviceVisitorSetTorqueBounds: public boost::static_visitor<>
    {
      const Vector & lower_;
      const Vector & upper_;

      JointDeviceVisitorSetTorqueBounds(const Vector & lower,
					   const Vector & upper):
	lower_(lower),
	upper_(upper)
      {}

      template <typename T> 
      void operator()(T &aJointDeviceVariant)
      const
      {	aJointDeviceVariant.setTorqueBounds(lower_,upper_); }

    };

    struct JointDeviceVisitorSetPositionVectorPos: public boost::static_visitor<>
    {
      const unsigned int & positionVectorPos_;

      JointDeviceVisitorSetPositionVectorPos(const unsigned int & positionVectorPos):
	positionVectorPos_(positionVectorPos)
      {}

      template <typename T> 
      void operator()(T &aJointDeviceVariant)
      const
      {	aJointDeviceVariant.setPositionVectorPos(positionVectorPos_); }

    };

    struct JointDeviceVisitorSetVelocityVectorPos: public boost::static_visitor<>
    {
      const unsigned int & velocityVectorPos_;

      JointDeviceVisitorSetVelocityVectorPos(const unsigned int & velocityVectorPos):
	velocityVectorPos_(velocityVectorPos)
      {}

      template <typename T> 
      void operator()(T &aJointDeviceVariant)
      const
      {	aJointDeviceVariant.setVelocityVectorPos(velocityVectorPos_); }

    };

    struct JointDeviceVisitorSetAccelerationVectorPos: public boost::static_visitor<>
    {
      const unsigned int & accelerationVectorPos_;

      JointDeviceVisitorSetAccelerationVectorPos(const unsigned int & accelerationVectorPos):
	accelerationVectorPos_(accelerationVectorPos)
      {}

      template <typename T> 
      void operator()(T &aJointDeviceVariant)
      const
      {	aJointDeviceVariant.setAccelerationVectorPos(accelerationVectorPos_); }

    };

    struct JointDeviceVisitorSetControlVectorPos: public boost::static_visitor<>
    {
      const unsigned int & controlVectorPos_;

      JointDeviceVisitorSetControlVectorPos(const unsigned int & controlVectorPos):
	controlVectorPos_(controlVectorPos)
      {}

      template <typename T> 
      void operator()(T &aJointDeviceVariant)
      const
      {	aJointDeviceVariant.setControlVectorPos(controlVectorPos_); }

    };

    struct JointDeviceVisitorSetJointName: public boost::static_visitor<>
    {
      const std::string & jointName_;

      JointDeviceVisitorSetJointName(const std::string & aJointName):
	jointName_(aJointName)
      {}

      template <typename T> 
      void operator()(T &aJointDeviceVariant)
      const
      {	aJointDeviceVariant.setJointName(jointName_); }

    };

  } //namespace sot
} // namespace dynamicgraph


#endif // SOT_JOINT_DEVICE_HXX

