/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
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

#ifndef __SOT_FEATURE_POINT6D_HH__
#define __SOT_FEATURE_POINT6D_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/exception-task.h>
#include <sot-core/matrix-homogeneous.h>
#include "sot-core/exception-feature.h"

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_point6d_EXPORTS)
#    define SOTFEATUREPOINT6D_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATUREPOINT6D_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATUREPOINT6D_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeaturePoint6d
  \brief Class that defines point-3d control feature
*/
class SOTFEATUREPOINT6D_EXPORT FeaturePoint6d
: public FeatureAbstract
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  enum ComputationFrameType
    {
      FRAME_DESIRED
      ,FRAME_CURRENT
    };
  static const ComputationFrameType COMPUTATION_FRAME_DEFAULT;

 public:
  /// \brief Set computation frame
  void computationFrame(const std::string& inFrame)
  {
    if (inFrame == "current")
      computationFrame_ = FRAME_CURRENT;
    else if (inFrame == "desired")
      computationFrame_ = FRAME_DESIRED;
    else {
      std::string msg("FeaturePoint6d::computationFrame: "
		      + inFrame + ": invalid argument,\n"
		      "expecting 'current' or 'desired'");
      throw ExceptionFeature(ExceptionFeature::GENERIC, msg);
    }
  }

  /// \brief Get computation frame
  std::string computationFrame() const 
  {
    switch(computationFrame_) {
    case FRAME_CURRENT:
      return "current";
    case FRAME_DESIRED:
      return "desired";
    default:
      return "";
    }
    return "";
  }

 private:
  ComputationFrameType computationFrame_;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< MatrixHomogeneous,int > positionSIN;
  dg::SignalPtr< ml::Matrix,int > articularJacobianSIN;

  using FeatureAbstract::desiredValueSIN;
  using FeatureAbstract::selectionSIN;

  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::activationSOUT;

 public:
  FeaturePoint6d( const std::string& name );
  virtual ~FeaturePoint6d( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual ml::Vector& computeError( ml::Vector& res,int time );
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );
  virtual ml::Vector& computeActivation( ml::Vector& res,int time );

  /** Static Feature selection. */
  inline static Flags selectX( void )  { return FLAG_LINE_1; }
  inline static Flags selectY( void )  { return FLAG_LINE_2; }
  inline static Flags selectZ( void )  { return FLAG_LINE_3; }
  inline static Flags selectRX( void ) { return FLAG_LINE_4; }
  inline static Flags selectRY( void ) { return FLAG_LINE_5; }
  inline static Flags selectRZ( void ) { return FLAG_LINE_6; }

  inline static Flags selectTranslation( void ) { return Flags(7); }
  inline static Flags selectRotation( void ) { return Flags(56); }

  virtual void display( std::ostream& os ) const;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
 public:
  void servoCurrentPosition( void );

} ;

} // namespace sot

#endif // #ifndef __SOT_FEATURE_POINT6D_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */