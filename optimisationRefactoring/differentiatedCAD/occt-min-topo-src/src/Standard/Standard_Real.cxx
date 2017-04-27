// Copyright (c) 1998-1999 Matra Datavision
// Copyright (c) 1999-2014 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include <float.h>
#include <Standard_Real.hxx>
#include <Standard_RangeError.hxx>
#include <Standard_NumericError.hxx>
#include <Standard_NullValue.hxx>
#include <Standard_Stream.hxx>
#include <Standard_OStream.hxx>

// ------------------------------------------------------------------
// Hascode : Computes a hascoding value for a given real
// ------------------------------------------------------------------
Standard_Integer HashCode(const double me, const Standard_Integer Upper)
{
  if (Upper < 1){
     Standard_RangeError::
      Raise("Try to apply HashCode method with negative or null argument.");
  }
  union 
    {
    double R;
    Standard_Integer I[2];
    } U;
//  U.R = Abs(me); // Treat me = -0.0 ADN 27/11/97
  U.R = me ;
  return HashCode( ( U.I[0] ^ U.I[1] ) , Upper ) ;
}

Standard_Integer HashCode(const myadouble me, const Standard_Integer Upper)
{
  if (Upper < 1){
     Standard_RangeError::
      Raise("Try to apply HashCode method with negative or null argument.");
  }
  union
    {
    double R;
    Standard_Integer I[2];
    } U;
//  U.R = Abs(me); // Treat me = -0.0 ADN 27/11/97
  U.R = me.getValue() ;
  return HashCode( ( U.I[0] ^ U.I[1] ) , Upper ) ;
}

//-------------------------------------------------------------------
// ACos : Returns the value of the arc cosine of a real
//-------------------------------------------------------------------
double ACos (const double Value)
{ 
  if ( (Value < -1.) || (Value > 1.) ){
    Standard_RangeError::Raise();
  } 
  return acos(Value); 
}

myadouble ACos (const myadouble Value)
{
  if ( (Value < -1.) || (Value > 1.) ){
    Standard_RangeError::Raise();
  }
  return adtl::acos(Value);
}

//-------------------------------------------------------------------
// ACosApprox : Returns the approximate value of the arc cosine of a real.
//              The max error is about 1 degree near Value=0.
//-------------------------------------------------------------------

inline double apx_for_ACosApprox (const double x)
{
  return  (-0.000007239283986332 +
    x * (2.000291665285952400 +
    x * (0.163910606547823220 +
    x * (0.047654245891495528 -
    x * (0.005516443930088506 +
    0.015098965761299077 * x))))) / sqrt(2*x);
}

inline myadouble apx_for_ACosApprox (const myadouble x)
{
  return  (-0.000007239283986332 +
    x * (2.000291665285952400 +
    x * (0.163910606547823220 +
    x * (0.047654245891495528 -
    x * (0.005516443930088506 +
    0.015098965761299077 * x))))) / sqrt(2*x);
}

double ACosApprox (const double Value)
{
  double XX;
  if (Value < 0.) {
    XX = 1.+Value;
    if (XX < RealSmall())
      return 0.;
    return M_PI - apx_for_ACosApprox(XX);
  }
  XX = 1.-Value;
  if (XX < RealSmall())
    return 0.;
  return apx_for_ACosApprox(XX);

// The code above is the same but includes 2 comparisons instead of 3
//   double xn = 1.+Value;
//   double xp = 1.-Value;
//   if (xp < RealSmall() || xn < RealSmall())
//     return 0.;
//   if (Value < 0.)
//     return M_PI - apx_for_ACosApprox (xn);
//   return apx_for_ACosApprox (xp);
}

myadouble ACosApprox (const myadouble Value)
{
  myadouble XX;
  if (Value < 0.) {
    XX = 1.+Value;
    if (XX < RealSmall())
      return 0.;
    return M_PI - apx_for_ACosApprox(XX);
  }
  XX = 1.-Value;
  if (XX < RealSmall())
    return 0.;
  return apx_for_ACosApprox(XX);

// The code above is the same but includes 2 comparisons instead of 3
//   double xn = 1.+Value;
//   double xp = 1.-Value;
//   if (xp < RealSmall() || xn < RealSmall())
//     return 0.;
//   if (Value < 0.)
//     return M_PI - apx_for_ACosApprox (xn);
//   return apx_for_ACosApprox (xp);
}

//-------------------------------------------------------------------
// ASin : Returns the value of the arc sine of a real
//-------------------------------------------------------------------
double ASin (const double Value)
{ 
  if ( Value < -1 || Value > 1 ){
    Standard_RangeError::Raise();
  }
  return asin(Value); 
}

myadouble ASin (const myadouble Value)
{
  if ( Value < -1 || Value > 1 ){
    Standard_RangeError::Raise();
  }
  return adtl::asin(Value);
}

//-------------------------------------------------------------------
// ATan2 : Returns the arc tangent of a real divide by an another real
//-------------------------------------------------------------------
double ATan2 (const double Value, const double Other)
{ 
  if ( Value == 0. && Other == 0. ){
    Standard_NullValue::Raise();
  }
  return atan2(Value,Other); 
}

myadouble ATan2 (const myadouble Value, const myadouble Other)
{
  if ( Value == 0. && Other == 0. ){
    Standard_NullValue::Raise();
  }
  return adtl::atan2(Value,Other);
}

//-------------------------------------------------------------------
// Sign : Returns |a| if B >= 0; -|a| if b < 0.
//             from x in the direction y
//-------------------------------------------------------------------
double Sign(const double a, const double b)
{
  //==== We use the function "nextafter()" fom library "math.h" ==============
  if (b >= 0.0) {
    return Abs(a);
  } else {
    return (-1.0 * Abs(a));
  }
}

myadouble Sign(const myadouble a, const myadouble b)
{
  //==== We use the function "nextafter()" fom library "math.h" ==============
  if (b >= 0.0) {
    return Abs(a);
  } else {
    return (-1.0 * Abs(a));
  }
}

//==========================================================================
//===== The special routines for "IEEE" and differents hardwares ===========
//==========================================================================
union RealMap {
  double real;
  unsigned int map[2];
};

//--------------------------------------------------------------------
// HardwareHighBitsOfDouble :  
//    Returns 1 if the low bits are at end.   (exemple: decmips and ALPHA )
//    Returns 0 if the low bits are at begin. (exemple: sun, sgi, ...)
//--------------------------------------------------------------------
static int HardwareHighBitsOfDouble()
{
  RealMap MaxDouble;
  MaxDouble.real = DBL_MAX;
  //=========================================================
  // reperesentation of the max double in IEEE is 
  //      "7fef ffff ffff ffff"   for the big indiens.
  //      "ffff ffff 7fef ffff"   for the littel indiens.
  //=========================================================

  if(MaxDouble.map[1] != 0xffffffff){
    return 1;
  } else {
    return 0;
  }
}

//--------------------------------------------------------------------
// HardwareLowBitsOfDouble :  
//    Returns 0 if the low bits are at end.   (exemple: decmips )
//    Returns 1 if the low bits are at begin. (exemple: sun, sgi, ...)
//--------------------------------------------------------------------
static int HardwareLowBitsOfDouble()
{
  RealMap MaxDouble;
  MaxDouble.real = DBL_MAX;
  //=========================================================
  // reperesentation of the max double in IEEE is 
  //      "7fef ffff ffff ffff"   for the big indiens.
  //      "ffff ffff 7fef ffff"   for the littel indiens.
  //=========================================================

  if(MaxDouble.map[1] != 0xffffffff){
    return 0;
  } else {
    return 1;
  }
}

static int HighBitsOfDouble = HardwareHighBitsOfDouble();
static int LowBitsOfDouble = HardwareLowBitsOfDouble();

double NextAfter(const double x, const double y)
{
  RealMap res;

  res.real=x;
  
  if (x == 0.0) {
	return DBL_MIN;
  }
  if(x==y) {
    //=========================================
    //   -oo__________0___________+oo
    //               x=y
    //  The direction is "Null", so there is nothing after 
    //=========================================

  } else if (((x<y) && (x>=0.0)) || ((x>y) && (x<0.0))) {
    //=========================================
    //   -oo__________0___________+oo
    //        y <- x     x -> y
    //
    //=========================================
    if (res.map[LowBitsOfDouble]==0xffffffff) {
      res.map[LowBitsOfDouble]=0;
      res.map[HighBitsOfDouble]++;
    } else {
      res.map[LowBitsOfDouble]++;
    }
  } else {
    //=========================================
    //   -oo__________0___________+oo
    //        x -> y     y <- x
    //
    //=========================================
    if (res.map[LowBitsOfDouble]==0) {
      if (res.map[HighBitsOfDouble]==0) {
	res.map[HighBitsOfDouble]=0x80000000;
	res.map[LowBitsOfDouble]=0x00000001;
      } else {
	res.map[LowBitsOfDouble]=0xffffffff;
	res.map[HighBitsOfDouble]--;
      }
    } else {
      res.map[LowBitsOfDouble]--;
    }
  }
  return res.real;
}

//-------------------------------------------------------------------
// ATanh : Returns the value of the hyperbolic arc tangent of a real
//-------------------------------------------------------------------
double     ATanh(const double Value)
{ 
  if ( (Value <= -1.) || (Value >= 1.) ){
    Standard_NumericError::Raise("Illegal agument in ATanh");
#ifdef OCCT_DEBUG
    cout << "Illegal agument in ATanh" << endl ;
#endif
  } 
  return atanh(Value); 
}

//-------------------------------------------------------------------
// ACosh : Returns the hyperbolic Arc cosine of a real
//-------------------------------------------------------------------
double     ACosh (const double Value)
{ 
  if ( Value < 1. ){
    Standard_NumericError::Raise("Illegal agument in ACosh");
#ifdef OCCT_DEBUG
    cout << "Illegal agument in ACosh" << endl ;
#endif
  } 
  return acosh(Value); 
}

//-------------------------------------------------------------------
// Log : Returns the naturaOPl logarithm of a real
//-------------------------------------------------------------------
double     Log (const double Value)
{   if ( Value <= 0. ){
    Standard_NumericError::Raise("Illegal agument in Log");
#ifdef OCCT_DEBUG
    cout << "Illegal agument in Log" << endl ;
#endif
  } 
 return log(Value); 
}

myadouble     Log (const myadouble Value)
{   if ( Value <= 0. ){
    Standard_NumericError::Raise("Illegal agument in Log");
#ifdef OCCT_DEBUG
    cout << "Illegal agument in Log" << endl ;
#endif
  }
 return adtl::log(Value);
}
//-------------------------------------------------------------------
// Sqrt : Returns the square root of a real
//-------------------------------------------------------------------
double     Sqrt (const double Value)
{ 
  if (  Value < 0. ){
    Standard_NumericError::Raise("Illegal agument in Sqrt");
#ifdef OCCT_DEBUG
    cout << "Illegal agument in Sqrt" << endl ;
#endif
  } 
 return sqrt(Value); 
}

myadouble     Sqrt (const myadouble Value)
{
  if (  Value < 0. ){
    Standard_NumericError::Raise("Illegal agument in Sqrt");
#ifdef OCCT_DEBUG
    cout << "Illegal agument in Sqrt" << endl ;
#endif
  }
 return adtl::sqrt(Value);
}

