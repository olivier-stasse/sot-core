/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFeatureVisualPoint.cpp
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
#include <sot-core/feature-visual-point.h>
#include <sot-core/exception-feature.h>
#include <sot-core/debug.h>
#include <sot-core/factory.h>
using namespace std;
using namespace sot;



SOT_FACTORY_FEATURE_PLUGIN(sotFeatureVisualPoint,"FeatureVisualPoint");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



sotFeatureVisualPoint::
sotFeatureVisualPoint( const string& pointName )
  : sotFeatureAbstract( pointName )
    ,L()
    ,xySIN( NULL,"sotFeatureVisualPoint("+name+")::input(vector)::xy" )
    ,ZSIN( NULL,"sotFeatureVisualPoint("+name+")::input(double)::Z" )
    ,articularJacobianSIN( NULL,"sotFeatureVisualPoint("+name+")::input(matrix)::Jq" )
{
  ZSIN=1.;

  jacobianSOUT.addDependancy( xySIN );
  jacobianSOUT.addDependancy( ZSIN );
  jacobianSOUT.addDependancy( articularJacobianSIN );

  errorSOUT.addDependancy( xySIN );
  errorSOUT.addDependancy( ZSIN );

  activationSOUT.removeDependancy( desiredValueSIN );

  signalRegistration( xySIN<<ZSIN<<articularJacobianSIN );
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& sotFeatureVisualPoint::
getDimension( unsigned int & dim, int time ) 
{
  sotDEBUG(25)<<"# In {"<<endl;

  const sotFlags &fl = selectionSIN.access(time);

  dim = 0;
  if( fl(0) ) dim++;
  if( fl(1) ) dim++;

  sotDEBUG(25)<<"# Out }"<<endl;
  return dim;
}


/** Compute the interaction matrix from a subset of
 * the possible features. 
 */
ml::Matrix& sotFeatureVisualPoint::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  sotDEBUG(15) << "Get selection flags." << endl;
  const sotFlags &fl = selectionSIN(time);

  const int dim = dimensionSOUT(time);
  std::cout<<" Dimension="<<dim<<std::endl;
  L.resize(dim,6) ;
  std::cout<<L.matrix<<std::endl;
  unsigned int cursorL = 0;

  sotDEBUG(5)<<std::endl;

  const double & Z = ZSIN(time);
  sotDEBUG(5)<<xySIN(time)<<std::endl;
  const double & x = xySIN(time)(0);
  const double & y = xySIN(time)(1);


  if( Z<0 )
    { throw(sotExceptionFeature(sotExceptionFeature::BAD_INIT,
				"VisualPoint is behind the camera"," (Z=%.1f).",Z)); }

  if( fabs(Z)<1e-6 )
    { throw(sotExceptionFeature(sotExceptionFeature::BAD_INIT,
				"VisualPoint Z coordinates is null"," (Z=%.3f)",Z)); }

  if( fl(0) )
    {
      L(cursorL,0) = -1/Z  ;
      L(cursorL,1) = 0 ;
      L(cursorL,2) = x/Z ;
      L(cursorL,3) = x*y ;
      L(cursorL,4) = -(1+x*x) ;
      L(cursorL,5) = y ;

      cursorL++;
    }

  if( fl(1) )
  {
    L(cursorL,0) = 0 ;
    L(cursorL,1)  = -1/Z ;
    L(cursorL,2) = y/Z ;
    L(cursorL,3) = 1+y*y ;
    L(cursorL,4) = -x*y ;
    L(cursorL,5) = -x ;
    
    cursorL++;
  }
  sotDEBUG(15) << "L:"<<endl<<L<<endl;
  sotDEBUG(15) << "Jq:"<<endl<<articularJacobianSIN(time)<<endl;
  
  L.multiply(articularJacobianSIN(time),J);

  sotDEBUG(15)<<"# Out }"<<endl;
  return J;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
sotFeatureVisualPoint::computeError( ml::Vector& error,int time )
{
  const sotFlags &fl = selectionSIN(time);
  sotDEBUGIN(15);
  error.resize(dimensionSOUT(time)) ;
  unsigned int cursorL = 0;

  sotFeatureVisualPoint * sdes 
    = dynamic_cast<sotFeatureVisualPoint*>(desiredValueSIN(time));
  
  if( NULL==sdes )
    { throw(sotExceptionFeature(sotExceptionFeature::BAD_INIT,
				"S* is not of adequate type.")); }
  if( fl(0) )
    { error( cursorL++ ) = xySIN(time)(0) - sdes->xySIN(time)(0) ;   }
  if( fl(1) )
    { error( cursorL++ ) = xySIN(time)(1) - sdes->xySIN(time)(1) ;   }

  sotDEBUGOUT(15);
  return error ;
}


/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
sotFeatureVisualPoint::computeActivation( ml::Vector& act,int time )
{
  selectionSIN(time);
  act.resize(dimensionSOUT(time)) ; act.fill(1);
  return act ;
}


void sotFeatureVisualPoint::
display( std::ostream& os ) const
{
  

  os <<"VisualPoint <"<<name<<">:";

  try{
    const ml::Vector& xy = xySIN;
    const sotFlags& fl = selectionSIN;
    if( fl(0) ) os << " x=" << xy(0) ;
    if( fl(1) ) os << " y=" << xy(1) ;
  }  catch(ExceptionAbstract e){ os<< " XY or select not set."; }
  
  try {
    const double& z = ZSIN; 
    os<<" Z=" << z << " ";
  }catch(ExceptionAbstract e){ os<< " Z not set."; }
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
