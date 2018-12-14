/*
 * Copyright 2010,
 * Nicolas Mansard, Olivier Stasse, François Bleibel, Florent Lamiraux
 *
 * CNRS
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#define ENABLE_RT_LOG

#include "sot/core/joint-device.hxx"
#include "sot/core/device.hh"
#include <sot/core/debug.hh>
using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/real-time-logger.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-geometry.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string Device::CLASS_NAME = "Device";

/* --------------------------------------------------------------------- */
/* --- JointDeviceVectorElement----------------------------------------- */
/* --------------------------------------------------------------------- */




/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


const MatrixHomogeneous& Device::freeFlyerPose() const
{
  return ffPose_;
}

Device::
~Device( )
{
  for( unsigned int i=0; i<4; ++i ) {
    delete forcesSOUT[i];
  }
}

Device::
Device( const std::string& n )
  :Entity(n)
  ,position_(6)
  ,sanityCheck_(true)
  ,controlInputType_(CONTROL_INPUT_ONE_INTEGRATION)
  ,controlSIN( NULL,"Device("+n+")::input(double)::control" )   
  ,attitudeSIN(NULL,"Device("+ n +")::input(vector3)::attitudeIN")
  ,zmpSIN(NULL,"Device("+n+")::input(vector3)::zmp")
  ,stateSOUT( "Device("+n+")::output(vector)::state" )   
  //,attitudeSIN(NULL,"Device::input(matrixRot)::attitudeIN")
  ,velocitySOUT( "Device("+n+")::output(vector)::velocity"  )
  ,attitudeSOUT( "Device("+n+")::output(matrixRot)::attitude" )
  ,motorcontrolSOUT   ( "Device("+n+")::output(vector)::motorcontrol" )
  ,previousControlSOUT( "Device("+n+")::output(vector)::previousControl" )
  ,ZMPPreviousControllerSOUT( "Device("+n+")::output(vector)::zmppreviouscontroller" )

  ,robotState_     ("Device("+n+")::output(vector)::robotState")
  ,robotVelocity_  ("Device("+n+")::output(vector)::robotVelocity")
  ,pseudoTorqueSOUT("Device("+n+")::output(vector)::ptorque" )

  ,ffPose_()
  ,forceZero6 (6)
{
  forceZero6.fill (0);
  /* --- SIGNALS --- */
  for( int i=0;i<4;++i ){ withForceSignals[i] = false; }
  forcesSOUT[0] =
      new Signal<Vector, int>("Device("+n+")::output(vector6)::forceRLEG");
  forcesSOUT[1] =
      new Signal<Vector, int>("Device("+n+")::output(vector6)::forceLLEG");
  forcesSOUT[2] =
      new Signal<Vector, int>("Device("+n+")::output(vector6)::forceRARM");
  forcesSOUT[3] =
      new Signal<Vector, int>("Device("+n+")::output(vector6)::forceLARM");

  signalRegistration( controlSIN
		      << stateSOUT
		      << robotState_
		      << robotVelocity_
                      << velocitySOUT
		      << attitudeSOUT
                      << attitudeSIN
		      << zmpSIN
		      << *forcesSOUT[0]
		      << *forcesSOUT[1]
                      << *forcesSOUT[2]
		      << *forcesSOUT[3]
		      << previousControlSOUT
                      << pseudoTorqueSOUT
		      << motorcontrolSOUT
		      << ZMPPreviousControllerSOUT );
  position_.fill(.0); stateSOUT.setConstant( position_ );

  velocity_.resize(position_.size()); velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );

  /* --- Commands --- */
  {
    std::string docstring;
    /* Command setStateSize. */
    docstring =
        "\n"
        "    Set size of state vector\n"
        "\n";
    addCommand("resize",
               new command::Setter<Device, unsigned int>
               (*this, &Device::setStateSize, docstring));
    docstring =
        "\n"
        "    Set state vector value\n"
        "\n";
    addCommand("set",
               new command::Setter<Device, Vector>
               (*this, &Device::setState, docstring));

    docstring =
        "\n"
        "    Set velocity vector value\n"
        "\n";
    addCommand("setVelocity",
               new command::Setter<Device, Vector>
               (*this, &Device::setVelocity, docstring));

    void(Device::*setRootPtr)(const Matrix&) = &Device::setRoot;
    docstring
        = command::docCommandVoid1("Set the root position.",
                                   "matrix homogeneous");
    addCommand("setRoot",
               command::makeCommandVoid1(*this,setRootPtr,
                                         docstring));

    /* Second Order Integration set. */
    docstring =
        "\n"
        "    Set the position calculous starting from  \n"
        "    acceleration measure instead of velocity \n"
        "\n";

    addCommand("setSecondOrderIntegration",
               command::makeCommandVoid0(*this,&Device::setSecondOrderIntegration,
                                         docstring));

    /* SET of control input type. */
    docstring =
        "\n"
        "    Set the type of control input which can be  \n"
        "    acceleration, velocity, or position\n"
        "\n";

    addCommand("setControlInputType",
               new command::Setter<Device,string>
               (*this, &Device::setControlInputType, docstring));

    /* SET of SoT control input type per joint */
    docstring =
        "\n"
        "    Set the type of control input per joint on the SoT side \n"
        "    which can be  \n"
        "    torque, acceleration, velocity, or position\n"
        "\n";

    addCommand("setSoTControlType",
	       command::makeCommandVoid2(*this,&Device::setSoTControlType,
                 command::docCommandVoid2 ("Set SoT control input type per joint",
					   "Joint name",
					   "Control type: [TORQUE|ACCELERATION|VELOCITY|POSITION]")
                 ));


    /* SET of HW control input type per joint */
    docstring =
        "\n"
        "    Set the type of control input per joint which can be  \n"
        "    torque, acceleration, velocity, or position\n"
        "\n";

    addCommand("setHWControlType",
	       command::makeCommandVoid2(*this,&Device::setHWControlType,
                 command::docCommandVoid2 ("Set HW control input type per joint",
					   "Joint name",
					   "Control type: [TORQUE|ACCELERATION|VELOCITY|POSITION]")
                 ));
    
    docstring =
        "\n"
        "    Enable/Disable sanity checks\n"
        "\n";
    addCommand("setSanityCheck",
               new command::Setter<Device, bool>
               (*this, &Device::setSanityCheck, docstring));

    addCommand("setPositionBounds",
               command::makeCommandVoid2(*this,&Device::setPositionBounds,
                 command::docCommandVoid2 ("Set robot position bounds", "vector: lower bounds", "vector: upper bounds")
                 ));

    addCommand("setVelocityBounds",
               command::makeCommandVoid2(*this,&Device::setVelocityBounds,
                 command::docCommandVoid2 ("Set robot velocity bounds", "vector: lower bounds", "vector: upper bounds")
                 ));

    addCommand("setTorqueBounds",
               command::makeCommandVoid2(*this,&Device::setTorqueBounds,
                 command::docCommandVoid2 ("Set robot torque bounds", "vector: lower bounds", "vector: upper bounds")
                 ));

    addCommand("setPositionPos",
               command::makeCommandVoid2(*this,&Device::setPositionPos,
					 command::docCommandVoid2 ("Set position of the joint in vector of position", "Joint name", "index")
					 ));

    addCommand("setVelocityPos",
               command::makeCommandVoid2(*this,&Device::setVelocityPos,
                 command::docCommandVoid2 ("Set position of the joint in vector of velocity", "Joint name", "index")
                 ));

    addCommand("setAccelerationPos",
               command::makeCommandVoid2(*this,&Device::setAccelerationPos,
                 command::docCommandVoid2 ("Set position of the joint in vector of acceleration", "Joint name", "index")
                 ));


    // Handle commands and signals called in a synchronous way.
    periodicCallBefore_.addSpecificCommands(*this, commandMap, "before.");
    periodicCallAfter_.addSpecificCommands(*this, commandMap, "after.");

  }
}

void Device::
setStateSize( const unsigned int& size )
{
  position_.resize(size); position_.fill( .0 );
  position_.resize(size);
  acceleration_.resize(size);
  stateSOUT .setConstant( position_ );
  previousControlSOUT.setConstant( position_ );
  pseudoTorqueSOUT.setConstant( position_ );
  motorcontrolSOUT .setConstant( position_ );

  Device::setVelocitySize(size);

  Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );
}

void Device::
setVelocitySize( const unsigned int& size )
{
  velocity_.resize(size);
  velocity_.fill(.0);
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setState( const Vector& st )
{
  if (sanityCheck_) {
    const Vector::Index& s = st.size();
    switch (controlInputType_) {
      case CONTROL_INPUT_TWO_INTEGRATION:
        dgRTLOG()
          << "Sanity check for this control is not well supported. "
             "In order to make it work, use pinocchio and the contact forces "
             "to estimate the joint torques for the given acceleration.\n";
        if (   s != lowerTorque_.size()
            || s != upperTorque_.size() )
          throw std::invalid_argument ("Upper and/or lower torque bounds "
              "do not match state size. Set them first with setTorqueBounds");
      case CONTROL_INPUT_ONE_INTEGRATION:
        if (   s != lowerVelocity_.size()
            || s != upperVelocity_.size() )
          throw std::invalid_argument ("Upper and/or lower velocity bounds "
              "do not match state size. Set them first with setVelocityBounds");
      case CONTROL_INPUT_NO_INTEGRATION:
        if (   s != lowerPosition_.size()
            || s != upperPosition_.size() )
          throw std::invalid_argument ("Upper and/or lower position bounds "
              "do not match state size. Set them first with setPositionBounds");
        break;
      default:
        throw std::invalid_argument ("Invalid control mode type.");
    }
  }
  position_ = st;
  stateSOUT .setConstant( position_ );
  motorcontrolSOUT .setConstant( position_ );
}

void Device::
setVelocity( const Vector& vel )
{
  velocity_ = vel;
  velocitySOUT .setConstant( velocity_ );
}

void Device::
setRoot( const Matrix & root )
{
  Eigen::Matrix4d _matrix4d(root);
  MatrixHomogeneous _root(_matrix4d);
  setRoot( _root );
}

void Device::
setRoot( const MatrixHomogeneous & worldMwaist )
{
  VectorRollPitchYaw r = (worldMwaist.linear().eulerAngles(2,1,0)).reverse();
  Vector q = position_;
  q = worldMwaist.translation(); // abusive ... but working.
  for( unsigned int i=0;i<3;++i ) q(i+3) = r(i);
}

void Device::
setSecondOrderIntegration()
{
  std::cout << "setSecondOrderIntegration deprecated" << std::endl;
  controlInputType_ = CONTROL_INPUT_TWO_INTEGRATION;
  velocity_.resize(position_.size());
  velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setNoIntegration()
{
  std::cout << "setNoIntegration deprecated" << std::endl;  
  controlInputType_ = CONTROL_INPUT_NO_INTEGRATION;
  velocity_.resize(position_.size());
  velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setControlInputType(const std::string& cit)
{
  std::cout << "setControlInputType deprecated" << std::endl;  
  for(int i=0; i<CONTROL_INPUT_SIZE; i++)
    if(cit==ControlInput_s[i])
    {
      controlInputType_ = (ControlInput)i;
      sotDEBUG(25)<<"Control input type: "<<ControlInput_s[i]<<endl;
      return;
    }
  sotDEBUG(25)<<"Unrecognized control input type: "<<cit<<endl;
}

void Device::
setSanityCheck(const bool & enableCheck)
{
  if (enableCheck) {
    const Vector::Index& s = position_.size();
    switch (controlInputType_) {
      case CONTROL_INPUT_TWO_INTEGRATION:
        dgRTLOG()
          << "Sanity check for this control is not well supported. "
             "In order to make it work, use pinocchio and the contact forces "
             "to estimate the joint torques for the given acceleration.\n";
        if (   s != lowerTorque_.size()
            || s != upperTorque_.size() )
          throw std::invalid_argument ("Upper and/or lower torque bounds "
              "do not match state size. Set them first with setTorqueBounds");
      case CONTROL_INPUT_ONE_INTEGRATION:
        if (   s != lowerVelocity_.size()
            || s != upperVelocity_.size() )
          throw std::invalid_argument ("Upper and/or lower velocity bounds "
              "do not match state size. Set them first with setVelocityBounds");
      case CONTROL_INPUT_NO_INTEGRATION:
        if (   s != lowerPosition_.size()
            || s != upperPosition_.size() )
          throw std::invalid_argument ("Upper and/or lower position bounds "
              "do not match state size. Set them first with setPositionBounds");
        break;
      default:
        throw std::invalid_argument ("Invalid control mode type.");
    }
  }
  sanityCheck_ = enableCheck;
}

void Device::
setControlType(const std::string &jointName,
	       const std::string &strCtrlType,
	       std::map<std::string, ControlType> &aCtrlType)
{
  for(int j=0;j<4;j++)
    {
      if (strCtrlType==ControlType_s[j])
	aCtrlType[jointName] = (ControlType)j;
    }

}
	      
void Device::
setSoTControlType(const std::string &jointNames,
		  const std::string &strCtrlType)
{
  setControlType(jointNames,strCtrlType,sotControlType_);
}

void Device::
setHWControlType(const std::string &jointNames,
		 const std::string &strCtrlType)
{
  setControlType(jointNames,strCtrlType,hwControlType_);
}

void Device::
setPositionPos(const std::string &jointName,
	       const unsigned & index)
{
  
  JointDeviceVariant & aJointDeviceVariant = jointDevices_[jointName];
  JointDeviceVisitorSetPositionVectorPos
    aJointDeviceVisitorSetPositionVectorPos(index);
  boost::apply_visitor(aJointDeviceVisitorSetPositionVectorPos,
		       aJointDeviceVariant);
}

void Device::
setVelocityPos(const std::string &jointName,
	       const unsigned & index)
{
  
  JointDeviceVariant & aJointDeviceVariant = jointDevices_[jointName];
  JointDeviceVisitorSetVelocityVectorPos
    aJointDeviceVisitorSetVelocityVectorPos(index);
  boost::apply_visitor(aJointDeviceVisitorSetVelocityVectorPos,
		       aJointDeviceVariant);
}

void Device::
setAccelerationPos(const std::string &jointName,
	       const unsigned & index)
{
  
  JointDeviceVariant & aJointDeviceVariant = jointDevices_[jointName];
  JointDeviceVisitorSetAccelerationVectorPos
    aJointDeviceVisitorSetAccelerationVectorPos(index);
  boost::apply_visitor(aJointDeviceVisitorSetAccelerationVectorPos,
		       aJointDeviceVariant);
}

void Device::
setControlPos(const std::string &jointName,
	       const unsigned & index)
{
  
  JointDeviceVariant & aJointDeviceVariant = jointDevices_[jointName];
  JointDeviceVisitorSetControlVectorPos
    aJointDeviceVisitorSetControlVectorPos(index);
  boost::apply_visitor(aJointDeviceVisitorSetControlVectorPos,
		       aJointDeviceVariant);
}

void Device::
setURDFModel(std::string &aURDFModel)
{
  modelURDF_ = urdf::parseURDF(aURDFModel);
  
  /// Iterates over joint of the URDF model.
  for(std::map<std::string,urdf::JointSharedPtr>::iterator joint_it =
	modelURDF_->joints_.begin();
      joint_it != modelURDF_->joints_.end();
      joint_it++)
    {
      JointDeviceFreeFlyer aJointDeviceFreeFlyer(7);
      JointDeviceRevolute aJointDeviceRevolute;
      JointDeviceLinear aJointDeviceLinear;

      JointDeviceVariant aJointDeviceVariant;

      
      urdf::JointSharedPtr aJointSharedPtr = joint_it->second;
      if (aJointSharedPtr->type==urdf::Joint::REVOLUTE)
	aJointDeviceVariant = aJointDeviceRevolute;
      else if (aJointSharedPtr->type==urdf::Joint::PRISMATIC)
	aJointDeviceVariant = aJointDeviceLinear;
      else if (aJointSharedPtr->type==urdf::Joint::FIXED)
	aJointDeviceVariant = aJointDeviceFreeFlyer;
      else if (aJointSharedPtr->type==urdf::Joint::FLOATING)
	aJointDeviceVariant = aJointDeviceFreeFlyer;

      JointDeviceVisitorSetJointName aJointDeviceVisitorJointName(aJointSharedPtr->name);
      boost::apply_visitor(aJointDeviceVisitorJointName,aJointDeviceVariant);
      
      urdf::JointLimitsSharedPtr aJointLimitsSharedPtr = aJointSharedPtr->limits;
      if (aJointLimitsSharedPtr!=NULL)
	{
	  std::cout << "joint limits: "
		    << aJointSharedPtr->name << " " 
		    << aJointLimitsSharedPtr->lower << " "
		    << aJointLimitsSharedPtr->upper << " "
		    << aJointLimitsSharedPtr->effort << " "
		    << aJointLimitsSharedPtr->velocity << std::endl;
	  dg::Vector lower(1),upper(1);
	  
	  /// Set position bounds
	  lower(0) = aJointLimitsSharedPtr->lower;
	  upper(0) = aJointLimitsSharedPtr->upper;
	  JointDeviceVisitorSetPositionBounds
	    aJointDeviceVisitorSetPositionBounds(lower,upper);
	  boost::apply_visitor(aJointDeviceVisitorSetPositionBounds,
			       aJointDeviceVariant);

	  /// Set velocity bounds
	  lower(0) = -aJointLimitsSharedPtr->velocity;
	  upper(0) = aJointLimitsSharedPtr->velocity;
	  JointDeviceVisitorSetVelocityBounds
	    aJointDeviceVisitorSetVelocityBounds(lower,upper);
	  boost::apply_visitor(aJointDeviceVisitorSetVelocityBounds,
			       aJointDeviceVariant);
	  
	  /// Set torque bounds
	  lower(0) = -aJointLimitsSharedPtr->effort;
	  upper(0) = aJointLimitsSharedPtr->effort;
	  JointDeviceVisitorSetTorqueBounds
	    aJointDeviceVisitorSetTorqueBounds(lower,upper);
	  boost::apply_visitor(aJointDeviceVisitorSetTorqueBounds,
			       aJointDeviceVariant);
      
	}
      jointDevices_[aJointSharedPtr->name] = aJointDeviceVariant;
    }
  
}

void Device::
setPositionBounds(const Vector& lower, const Vector& upper)
{
  std::ostringstream oss;
  if (lower.size() != position_.size()) {
    oss << "Lower bound size should be " << position_.size();
    throw std::invalid_argument (oss.str());
  }
  if (upper.size() != position_.size()) {
    oss << "Upper bound size should be " << position_.size();
    throw std::invalid_argument (oss.str());
  }
  lowerPosition_ = lower;
  upperPosition_ = upper;
}

void Device::
setVelocityBounds(const Vector& lower, const Vector& upper)
{
  std::ostringstream oss;
  if (lower.size() != velocity_.size()) {
    oss << "Lower bound size should be " << velocity_.size();
    throw std::invalid_argument (oss.str());
  }
  if (upper.size() != velocity_.size()) {
    oss << "Upper bound size should be " << velocity_.size();
    throw std::invalid_argument (oss.str());
  }
  lowerVelocity_ = lower;
  upperVelocity_ = upper;
}

void Device::
setTorqueBounds  (const Vector& lower, const Vector& upper)
{
  // TODO I think the torque bounds size are position_.size()-6...
  std::ostringstream oss;
  if (lower.size() != position_.size()) {
    oss << "Lower bound size should be " << position_.size();
    throw std::invalid_argument (oss.str());
  }
  if (upper.size() != position_.size()) {
    oss << "Lower bound size should be " << position_.size();
    throw std::invalid_argument (oss.str());
  }
  lowerTorque_ = lower;
  upperTorque_ = upper;
}

void Device::
increment( const double & dt )
{
  int time = stateSOUT.getTime();
  sotDEBUG(25) << "Time : " << time << std::endl;

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodicCallBefore_.run(time+1);
  }
  catch (std::exception& e)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (before): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (before): "
        << str << std::endl;
  }
  catch (...)
  {
    dgRTLOG()
        << "unknown exception caught while"
        << " running periodical commands (before)" << std::endl;
  }


  /* Force the recomputation of the control. */
  controlSIN( time );
  sotDEBUG(25) << "u" <<time<<" = " << controlSIN.accessCopy() << endl;

  integrate(dt);
  
  sotDEBUG(25) << "q" << time << " = " << position_ << endl;

  /* Position the signals corresponding to sensors. */
  stateSOUT .setConstant( position_ ); stateSOUT.setTime( time+1 );

  //computation of the velocity signal
  if( controlInputType_==CONTROL_INPUT_TWO_INTEGRATION )
  {
    velocitySOUT.setConstant( velocity_ );
    velocitySOUT.setTime( time+1 );
  }
  else if (controlInputType_==CONTROL_INPUT_ONE_INTEGRATION)
  {
    velocitySOUT.setConstant( controlSIN.accessCopy() );
    velocitySOUT.setTime( time+1 );
  }
  for( int i=0;i<4;++i ){
    if(  !withForceSignals[i] ) forcesSOUT[i]->setConstant(forceZero6);
  }
  Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodicCallAfter_.run(time+1);
  }
  catch (std::exception& e)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (after): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (after): "
        << str << std::endl;
  }
  catch (...)
  {
    dgRTLOG()
        << "unknown exception caught while"
        << " running periodical commands (after)" << std::endl;
  }


  // Others signals.
  motorcontrolSOUT .setConstant( position_ );
}


void Device::integrate( const double & dt )
{
  const Vector & controlIN = controlSIN.accessCopy();

  if (sanityCheck_ && controlIN.hasNaN())
  {
    dgRTLOG () << "Device::integrate: Control has NaN values: " << '\n'
               << controlIN.transpose() << '\n';
    return;
  }

    /* Integration of numerical values. This function is virtual. */
  /// Iterates over joints.
  JointDeviceVisitorIntegrate
    aJointDeviceVisitorIntegrate(*this, dt);
  
  for(std::map<std::string,JointDeviceVariant>::iterator
	joint_it = jointDevices_.begin();
      joint_it != jointDevices_.end();
      joint_it++)
    {
      boost::apply_visitor(aJointDeviceVisitorIntegrate,
			   joint_it->second);
    }
  
}


/* --- DISPLAY ------------------------------------------------------------ */

void Device::display ( std::ostream& os ) const
{os <<name<<": "<<position_<<endl; }
