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

#ifndef _double_HeaderFile
#define _double_HeaderFile

#include <float.h>
#include <math.h>
#include <Standard_values.h>
#include <Standard_math.hxx>
#include <Standard_TypeDef.hxx>

// ===============================================
// Methods from Standard_Entity class which are redefined:  
//    - Hascode
//    - IsEqual
// ===============================================

// ==================================
// Methods implemeted in double.cxx
// ==================================
__Standard_API Standard_Integer HashCode    (const double, const Standard_Integer);
__Standard_API Standard_Integer HashCode    (const myadouble, const Standard_Integer);

__Standard_API double    ACos        (const double );
__Standard_API myadouble    ACos        (const myadouble );
__Standard_API double    ACosApprox  (const double );
__Standard_API myadouble    ACosApprox  (const myadouble );
__Standard_API double    ASin        (const double );
__Standard_API myadouble    ASin        (const myadouble );
__Standard_API double    ATan2       (const double , const double );
__Standard_API myadouble    ATan2       (const myadouble , const myadouble );
__Standard_API double    NextAfter   (const double , const double );
//__Standard_API myadouble    NextAfter   (const myadouble , const myadouble ); //not for adoubles
__Standard_API double    Sign        (const double , const double );
__Standard_API myadouble    Sign        (const myadouble , const myadouble );
__Standard_API double    ATanh       (const double );
//__Standard_API myadouble    ATanh       (const myadouble ); //not for adoubles
__Standard_API double    ACosh       (const double );
//__Standard_API myadouble    ACosh       (const myadouble ); //not for adoubles
__Standard_API double    Log         (const double );
__Standard_API myadouble    Log         (const myadouble );
__Standard_API double    Sqrt        (const double );
__Standard_API myadouble    Sqrt        (const myadouble );

//-------------------------------------------------------------------
// RealSmall : Returns the smallest positive real
//-------------------------------------------------------------------
inline double     RealSmall()
{ return DBL_MIN; }

//-------------------------------------------------------------------
// Abs : Returns the absolute value of a real
//-------------------------------------------------------------------
inline double     Abs(const double Value)
{ return fabs(Value); }

inline myadouble     Abs(const myadouble Value)
{ return adtl::fabs(Value); }

//-------------------------------------------------------------------
// IsEqual : Returns Standard_True if two reals are equal
//-------------------------------------------------------------------
inline Standard_Boolean  IsEqual (const double Value1,
				  const double Value2)
{ return Abs((Value1 - Value2)) < RealSmall(); }

inline Standard_Boolean  IsEqual (const myadouble Value1,
          const myadouble Value2)
{ return Abs((Value1 - Value2)) < RealSmall(); }

         //  *********************************** //
         //       Class methods                  //
         //                                      //
         //  Machine-dependant values            //
         //  Should be taken from include file   //
         //  *********************************** //


//-------------------------------------------------------------------
// RealDigit : Returns the number of digits of precision in a real
//-------------------------------------------------------------------
inline Standard_Integer  RealDigits() 
{ return DBL_DIG; }

//-------------------------------------------------------------------
// RealEpsilon : Returns the minimum positive real such that 
//               1.0 + x is not equal to 1.0
//-------------------------------------------------------------------
inline double     RealEpsilon()
{ return DBL_EPSILON; }

//-------------------------------------------------------------------
// RealFirst : Returns the minimum negative value of a real
//-------------------------------------------------------------------
inline double     RealFirst()
{ return -DBL_MAX; }
  
//-------------------------------------------------------------------
// RealFirst10Exp : Returns the minimum value of exponent(base 10) of
//                  a real.
//-------------------------------------------------------------------
inline Standard_Integer  RealFirst10Exp() 
{ return DBL_MIN_10_EXP; }

//-------------------------------------------------------------------
// RealLast : Returns the maximum value of a real
//-------------------------------------------------------------------
inline double     RealLast()
{ return  DBL_MAX; }

//-------------------------------------------------------------------
// RealLast10Exp : Returns the maximum value of exponent(base 10) of
//                 a real.
//-------------------------------------------------------------------
inline Standard_Integer  RealLast10Exp() 
{ return  DBL_MAX_10_EXP; }

//-------------------------------------------------------------------
// RealMantissa : Returns the size in bits of the matissa part of a 
//                real.
//-------------------------------------------------------------------
inline Standard_Integer  RealMantissa() 
{ return  DBL_MANT_DIG; }

//-------------------------------------------------------------------
// RealRadix : Returns the radix of exponent representation
//-------------------------------------------------------------------
inline Standard_Integer  RealRadix() 
{ return  FLT_RADIX; }

//-------------------------------------------------------------------
// RealSize : Returns the size in bits of an integer
//-------------------------------------------------------------------
inline Standard_Integer  RealSize() 
{ return BITS(double); }



         //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//
         //   End of machine-dependant values   //
         //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//


//-------------------------------------------------------------------
// IntToReal : Converts an integer in a real
//-------------------------------------------------------------------
inline double     IntToReal(const Standard_Integer Value)
{ return Value; }

//-------------------------------------------------------------------
// ATan : Returns the value of the arc tangent of a real
//-------------------------------------------------------------------
inline double     ATan(const double Value)
{ return atan(Value); }

inline myadouble     ATan(const myadouble Value)
{ return adtl::atan(Value); }


//-------------------------------------------------------------------
// Ceiling : Returns the smallest integer not less than a real
//-------------------------------------------------------------------
inline double     Ceiling (const double Value)
{ return ceil(Value); }

inline myadouble     Ceiling (const myadouble Value)
{ return adtl::ceil(Value); }

//-------------------------------------------------------------------
// Cos : Returns the cosine of a real
//-------------------------------------------------------------------
inline double     Cos (const double Value)
{ return cos(Value); }

inline myadouble     Cos (const myadouble Value)
{ return adtl::cos(Value); }

//-------------------------------------------------------------------
// Cosh : Returns the hyperbolic cosine of a real
//-------------------------------------------------------------------
inline double     Cosh (const double Value)
{ return cosh(Value); }

inline myadouble     Cosh (const myadouble Value)
{ return adtl::cosh(Value); }


//-------------------------------------------------------------------
// Epsilon : The function returns absolute value of difference
//           between 'Value' and other nearest value of
//           double type.
//           Nearest value is choseen in direction of infinity
//           the same sign as 'Value'.
//           If 'Value' is 0 then returns minimal positive value
//           of double type.
//-------------------------------------------------------------------
inline double     Epsilon (const double Value)
{
  double aEpsilon;

  if (Value>=0.0){
    aEpsilon = NextAfter(Value, RealLast()) - Value;
  } else {
    aEpsilon = Value - NextAfter(Value, RealFirst());
  }
  return aEpsilon;
}

inline myadouble     Epsilon (const myadouble Value)
{
  myadouble aEpsilon;

  if (Value>=0.0){
    aEpsilon = NextAfter(Value.getValue(), RealLast()) - Value;
  } else {
    aEpsilon = Value - NextAfter(Value.getValue(), RealFirst());
  }
  return aEpsilon;
}

//-------------------------------------------------------------------
// Exp : Returns the exponential function of a real
//-------------------------------------------------------------------
inline double     Exp (const double Value)
{ return exp(Value); }

inline myadouble     Exp (const myadouble Value)
{ return adtl::exp(Value); }

//-------------------------------------------------------------------
// Floor : Return the largest integer not greater than a real
//-------------------------------------------------------------------
inline double     Floor (const double Value)
{ return floor(Value); }

inline myadouble     Floor (const myadouble Value)
{ return adtl::floor(Value); }

//-------------------------------------------------------------------
// IntegerPart : Returns the integer part of a real
//-------------------------------------------------------------------
inline double     IntegerPart (const double Value)
{ return ( (Value>0) ? floor(Value) : ceil(Value) ); }

inline myadouble     IntegerPart (const myadouble Value)
{ return ( (Value>0) ? adtl::floor(Value) : adtl::ceil(Value) ); }


//-------------------------------------------------------------------
// Log10 : Returns the base-10 logarithm of a real 
//-------------------------------------------------------------------
inline double     Log10 (const double Value)
{ return log10(Value); }

inline myadouble     Log10 (const myadouble Value)
{ return adtl::log10(Value); }

//-------------------------------------------------------------------
// Max : Returns the maximum value of two reals
//-------------------------------------------------------------------
inline double     Max (const double Val1,
                              const double Val2)
{
  return Val1 >= Val2 ? Val1 : Val2;
}

inline myadouble     Max (const myadouble Val1,
                              const myadouble Val2)
{
  return Val1 >= Val2 ? Val1 : Val2;
}

//-------------------------------------------------------------------
// Min : Returns the minimum value of two reals
//-------------------------------------------------------------------
inline double     Min (const double Val1,
                              const double Val2)
{
  return Val1 <= Val2 ? Val1 : Val2;
}

inline myadouble     Min (const myadouble Val1,
                              const myadouble Val2)
{
  return Val1 <= Val2 ? Val1 : Val2;
}

//-------------------------------------------------------------------
// Pow : Returns a real to a given power
//-------------------------------------------------------------------
inline double     Pow (const double Value, const double P)
{ return pow(Value,P); }

inline myadouble     Pow (const myadouble Value, const myadouble P)
{ return adtl::pow(Value,P); }

inline myadouble     Pow (const myadouble Value, const double P)
{ return adtl::pow(Value,P); }

inline myadouble     Pow (const double Value, const myadouble P)
{ return adtl::pow(Value,P); }

//-------------------------------------------------------------------
// RealPart : Returns the fractional part of a real.
//-------------------------------------------------------------------
inline  double    RealPart (const double Value)
{ return fabs(IntegerPart(Value) - Value); }

inline  myadouble    RealPart (const myadouble Value)
{ return adtl::fabs(IntegerPart(Value) - Value); }

//-------------------------------------------------------------------
// RealToInt : Returns the real converted to nearest valid integer.
//             If input value is out of valid range for integers,
//             minimal or maximal possible integer is returned.
//-------------------------------------------------------------------
inline  Standard_Integer RealToInt (const double Value)
{ 
  // Note that on WNT under MS VC++ 8.0 conversion of double value less 
  // than INT_MIN or greater than INT_MAX to integer will cause signal 
  // "Floating point multiple trap" (OCC17861)
  return Value < INT_MIN ? INT_MIN
    : Value > INT_MAX ? INT_MAX
    : (Standard_Integer)Value;
}

inline  Standard_Integer RealToInt (const myadouble Value)
{
  // Note that on WNT under MS VC++ 8.0 conversion of double value less
  // than INT_MIN or greater than INT_MAX to integer will cause signal
  // "Floating point multiple trap" (OCC17861)
  return Value < INT_MIN ? INT_MIN
    : Value > INT_MAX ? INT_MAX
    : (Standard_Integer)Value.getValue();
}

// =======================================================================
// function : RealToShortReal
// purpose  : Converts double value to the nearest valid
//            Standard_ShortReal. If input value is out of valid range
//            for Standard_ShortReal, minimal or maximal
//            Standard_ShortReal is returned.
// =======================================================================
inline Standard_ShortReal RealToShortReal (const double theVal)
{
  return theVal < -FLT_MAX ? -FLT_MAX
    : theVal > FLT_MAX ? FLT_MAX
    : (Standard_ShortReal)theVal;
}

inline Standard_ShortReal RealToShortReal (const myadouble theVal)
{
  return theVal < -FLT_MAX ? -FLT_MAX
    : theVal > FLT_MAX ? FLT_MAX
    : (Standard_ShortReal)theVal.getValue();
}

//-------------------------------------------------------------------
// Round : Returns the nearest integer of a real
//-------------------------------------------------------------------
inline double     Round (const double Value)
{ return IntegerPart(Value + (Value > 0 ? 0.5 : -0.5)); }

inline myadouble     Round (const myadouble Value)
{ return IntegerPart(Value + (Value > 0 ? 0.5 : -0.5)); }

//-------------------------------------------------------------------
// Sin : Returns the sine of a real
//-------------------------------------------------------------------
inline double     Sin (const double Value)
{ return sin(Value); }

inline myadouble     Sin (const myadouble Value)
{ return adtl::sin(Value); }

//-------------------------------------------------------------------
// Sinh : Returns the hyperbolic sine of a real
//-------------------------------------------------------------------
inline double     Sinh(const double Value)
{ return sinh(Value); }

inline myadouble     Sinh(const myadouble Value)
{ return adtl::sinh(Value); }

//-------------------------------------------------------------------
// ASinh : Returns the hyperbolic arc sine of a real
//-------------------------------------------------------------------
inline double     ASinh(const double Value)
{ return asinh(Value); }

//-------------------------------------------------------------------
// Square : Returns a real to the power 2
//-------------------------------------------------------------------
inline double     Square(const double Value)
{ return Value * Value; }

inline myadouble     Square(const myadouble Value)
{ return Value * Value; }

//-------------------------------------------------------------------
// Tan : Returns the tangent of a real
//-------------------------------------------------------------------
inline double     Tan (const double Value)
{ return tan(Value); }

inline myadouble     Tan (const myadouble Value)
{ return adtl::tan(Value); }

//-------------------------------------------------------------------
// Tanh : Returns the hyperbolic tangent of a real
//-------------------------------------------------------------------
inline double     Tanh (const double Value)
{ return tanh(Value); }

inline myadouble     Tanh (const myadouble Value)
{ return adtl::tanh(Value); }

#endif
