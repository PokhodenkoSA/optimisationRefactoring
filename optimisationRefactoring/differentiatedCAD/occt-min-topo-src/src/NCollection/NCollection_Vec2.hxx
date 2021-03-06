// Created by: Kirill GAVRILOV
// Copyright (c) 2013-2014 OPEN CASCADE SAS
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

#ifndef _NCollection_Vec2_H__
#define _NCollection_Vec2_H__

#include <cmath> // std::sqrt()
#include <Standard_Real.hxx>

//! Auxiliary macros to define couple of similar access components as vector methods.
//! @return 2 components by their names in specified order
#define NCOLLECTION_VEC_COMPONENTS_2D(theX, theY) \
  const NCollection_Vec2<Element_t> theX##theY() const { return NCollection_Vec2<Element_t>(theX(), theY()); } \
  const NCollection_Vec2<Element_t> theY##theX() const { return NCollection_Vec2<Element_t>(theY(), theX()); }

//! Defines the 2D-vector template.
//! The main target for this class - to handle raw low-level arrays (from/to graphic driver etc.).
template<typename Element_t>
class NCollection_Vec2
{

public:

  //! Returns the number of components.
  static int Length()
  {
    return 2;
  }

  //! Empty constructor. Construct the zero vector.
  NCollection_Vec2()
  {
    v[0] = v[1] = Element_t(0);
  }

  //! Initialize ALL components of vector within specified value.
  explicit NCollection_Vec2 (const Element_t theXY)
  {
    v[0] = v[1] = theXY;
  }

  //! Per-component constructor.
  explicit NCollection_Vec2 (const Element_t theX,
                             const Element_t theY)
  {
    v[0] = theX;
    v[1] = theY;
  }

  //! Alias to 1st component as X coordinate in XY.
  Element_t x() const { return v[0]; }

  //! Alias to 2nd component as Y coordinate in XY.
  Element_t y() const { return v[1]; }

  //! @return 2 components by their names in specified order (in GLSL-style)
  NCOLLECTION_VEC_COMPONENTS_2D(x, y)

  //! Alias to 1st component as X coordinate in XY.
  Element_t& x() { return v[0]; }

  //! Alias to 2nd component as Y coordinate in XY.
  Element_t& y() { return v[1]; }

  //! Raw access to the data (for OpenGL exchange).
  const Element_t* GetData()    const { return v; }
        Element_t* ChangeData()       { return v; }
  operator const   Element_t*() const { return v; }
  operator         Element_t*()       { return v; }

  //! Compute per-component summary.
  NCollection_Vec2& operator+= (const NCollection_Vec2& theAdd)
  {
    v[0] += theAdd.v[0];
    v[1] += theAdd.v[1];
    return *this;
  }

  //! Compute per-component summary.
  friend NCollection_Vec2 operator+ (const NCollection_Vec2& theLeft,
                                     const NCollection_Vec2& theRight)
  {
    return NCollection_Vec2 (theLeft.v[0] + theRight.v[0],
                             theLeft.v[1] + theRight.v[1]);
  }

  //! Compute per-component subtraction.
  NCollection_Vec2& operator-= (const NCollection_Vec2& theDec)
  {
    v[0] -= theDec.v[0];
    v[1] -= theDec.v[1];
    return *this;
  }

  //! Compute per-component subtraction.
  friend NCollection_Vec2 operator- (const NCollection_Vec2& theLeft,
                                     const NCollection_Vec2& theRight)
  {
    return NCollection_Vec2 (theLeft.v[0] - theRight.v[0],
                             theLeft.v[1] - theRight.v[1]);
  }

  //! Unary -.
  NCollection_Vec2 operator-() const
  {
    return NCollection_Vec2 (-x(), -y());
  }

  //! Compute per-component multiplication.
  NCollection_Vec2& operator*= (const NCollection_Vec2& theRight)
  {
    v[0] *= theRight.v[0];
    v[1] *= theRight.v[1];
    return *this;
  }

  //! Compute per-component multiplication.
  friend NCollection_Vec2 operator* (const NCollection_Vec2& theLeft,
                                     const NCollection_Vec2& theRight)
  {
    return NCollection_Vec2 (theLeft.v[0] * theRight.v[0],
                             theLeft.v[1] * theRight.v[1]);
  }

  //! Compute per-component multiplication by scale factor.
  void Multiply (const Element_t theFactor)
  {
    v[0] *= theFactor;
    v[1] *= theFactor;
  }

  //! Compute per-component multiplication by scale factor.
  NCollection_Vec2 Multiplied (const Element_t theFactor) const
  {
    return NCollection_Vec2 (v[0] * theFactor,
                             v[1] * theFactor);
  }

  //! Compute component-wise minimum of two vectors.
  NCollection_Vec2 cwiseMin (const NCollection_Vec2& theVec) const
  {
    return NCollection_Vec2 (v[0] < theVec.v[0] ? v[0] : theVec.v[0],
                             v[1] < theVec.v[1] ? v[1] : theVec.v[1]);
  }

  //! Compute component-wise maximum of two vectors.
  NCollection_Vec2 cwiseMax (const NCollection_Vec2& theVec) const
  {
    return NCollection_Vec2 (v[0] > theVec.v[0] ? v[0] : theVec.v[0],
                             v[1] > theVec.v[1] ? v[1] : theVec.v[1]);
  }

  //! Compute component-wise modulus of the vector.
  NCollection_Vec2 cwiseAbs() const
  {
    return NCollection_Vec2 (Abs (v[0]),
                             Abs (v[1]));
  }

  //! Compute maximum component of the vector.
  Element_t maxComp() const
  {
    return v[0] > v[1] ? v[0] : v[1];
  }

  //! Compute minimum component of the vector.
  Element_t minComp() const
  {
    return v[0] < v[1] ? v[0] : v[1];
  }

  //! Compute per-component multiplication by scale factor.
  NCollection_Vec2& operator*= (const Element_t theFactor)
  {
    Multiply (theFactor);
    return *this;
  }

  //! Compute per-component division by scale factor.
  NCollection_Vec2& operator/= (const Element_t theInvFactor)
  {
    v[0] /= theInvFactor;
    v[1] /= theInvFactor;
    return *this;
  }

  //! Compute per-component multiplication by scale factor.
  NCollection_Vec2 operator* (const Element_t theFactor) const
  {
    return Multiplied (theFactor);
  }

  //! Compute per-component division by scale factor.
  NCollection_Vec2 operator/ (const Element_t theInvFactor) const
  {
    return NCollection_Vec2(v[0] / theInvFactor,
            v[1] / theInvFactor);
  }

  //! Computes the dot product.
  Element_t Dot (const NCollection_Vec2& theOther) const
  {
    return x() * theOther.x() + y() * theOther.y();
  }

  //! Computes the vector modulus (magnitude, length).
  Element_t Modulus() const
  {
    return Sqrt (x() * x() + y() * y());
  }

  //! Computes the square of vector modulus (magnitude, length).
  //! This method may be used for performance tricks.
  Element_t SquareModulus() const
  {
    return x() * x() + y() * y();
  }

  //! Constuct DX unit vector.
  static NCollection_Vec2 DX()
  {
    return NCollection_Vec2 (Element_t(1), Element_t(0));
  }

  //! Constuct DY unit vector.
  static NCollection_Vec2 DY()
  {
    return NCollection_Vec2 (Element_t(0), Element_t(1));
  }

private:

  Element_t v[2];

};

#endif // _NCollection_Vec2_H__
