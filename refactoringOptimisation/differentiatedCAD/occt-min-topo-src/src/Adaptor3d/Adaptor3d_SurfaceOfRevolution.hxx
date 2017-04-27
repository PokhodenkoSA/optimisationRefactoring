// Created on: 1993-04-21
// Created by: Bruno DUMORTIER
// Copyright (c) 1993-1999 Matra Datavision
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

#ifndef _Adaptor3d_SurfaceOfRevolution_HeaderFile
#define _Adaptor3d_SurfaceOfRevolution_HeaderFile

#include <Standard.hxx>
#include <Standard_DefineAlloc.hxx>
#include <Standard_Handle.hxx>

#include <gp_Ax1.hxx>
#include <Standard_Boolean.hxx>
#include <gp_Ax3.hxx>
#include <Adaptor3d_Surface.hxx>
#include <Standard_Real.hxx>
#include <GeomAbs_Shape.hxx>
#include <Standard_Integer.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <GeomAbs_SurfaceType.hxx>
class Adaptor3d_HCurve;
class Standard_OutOfRange;
class Standard_NoSuchObject;
class Standard_DomainError;
class gp_Ax1;
class Adaptor3d_HSurface;
class gp_Pnt;
class gp_Vec;
class gp_Pln;
class gp_Cylinder;
class gp_Cone;
class gp_Sphere;
class gp_Torus;
class Geom_BezierSurface;
class Geom_BSplineSurface;
class gp_Ax3;
class gp_Dir;


//! This class defines a complete surface of revolution.
//! The surface is obtained by rotating a curve a complete revolution
//! about an axis. The curve and the axis must be in the same plane.
//! If the curve and the axis are not in the same plane it is always
//! possible to be in the previous case after a cylindrical projection
//! of the curve in a referenced plane.
//! For a complete surface of revolution the parametric range is
//! 0 <= U <= 2*PI.       --
//! The parametric range for V is defined with the revolved curve.
//! The origin of the U parametrization is given by the position
//! of the revolved curve (reference). The direction of the revolution
//! axis defines the positive sense of rotation (trigonometric sense)
//! corresponding to the increasing of the parametric value U.
//! The derivatives are always defined for the u direction.
//! For the v direction the definition of the derivatives depends on
//! the degree of continuity of the referenced curve.
//! Curve and Axis are coplanar.
//! Curve doesn't intersect Axis.
class Adaptor3d_SurfaceOfRevolution  : public Adaptor3d_Surface
{
public:

  DEFINE_STANDARD_ALLOC

  
  Standard_EXPORT Adaptor3d_SurfaceOfRevolution();
  
  //! The Curve is loaded.
  Standard_EXPORT Adaptor3d_SurfaceOfRevolution(const Handle(Adaptor3d_HCurve)& C);
  
  //! The Curve and the Direction are loaded.
  Standard_EXPORT Adaptor3d_SurfaceOfRevolution(const Handle(Adaptor3d_HCurve)& C, const gp_Ax1& V);
  
  //! Changes the Curve
  Standard_EXPORT void Load (const Handle(Adaptor3d_HCurve)& C);
  
  //! Changes the Direction
  Standard_EXPORT void Load (const gp_Ax1& V);
  
  Standard_EXPORT gp_Ax1 AxeOfRevolution() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Real FirstUParameter() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Real LastUParameter() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Real FirstVParameter() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Real LastVParameter() const Standard_OVERRIDE;
  
  Standard_EXPORT GeomAbs_Shape UContinuity() const Standard_OVERRIDE;
  
  //! Return CN.
  Standard_EXPORT GeomAbs_Shape VContinuity() const Standard_OVERRIDE;
  
  //! Returns the number of U intervals for  continuity
  //! <S>. May be one if UContinuity(me) >= <S>
  Standard_EXPORT Standard_Integer NbUIntervals (const GeomAbs_Shape S) const Standard_OVERRIDE;
  
  //! Returns the number of V intervals for  continuity
  //! <S>. May be one if VContinuity(me) >= <S>
  Standard_EXPORT Standard_Integer NbVIntervals (const GeomAbs_Shape S) const Standard_OVERRIDE;
  
  //! Returns the  intervals with the requested continuity
  //! in the U direction.
  Standard_EXPORT void UIntervals (TColStd_Array1OfReal& T, const GeomAbs_Shape S) const Standard_OVERRIDE;
  
  //! Returns the  intervals with the requested continuity
  //! in the V direction.
  Standard_EXPORT void VIntervals (TColStd_Array1OfReal& T, const GeomAbs_Shape S) const Standard_OVERRIDE;
  
  //! Returns    a  surface trimmed in the U direction
  //! equivalent   of  <me>  between
  //! parameters <First>  and <Last>. <Tol>  is used  to
  //! test for 3d points confusion.
  //! If <First> >= <Last>
  Standard_EXPORT Handle(Adaptor3d_HSurface) UTrim (const Standard_Real First, const Standard_Real Last, const Standard_Real Tol) const Standard_OVERRIDE;
  
  //! Returns    a  surface trimmed in the V direction  between
  //! parameters <First>  and <Last>. <Tol>  is used  to
  //! test for 3d points confusion.
  //! If <First> >= <Last>
  Standard_EXPORT Handle(Adaptor3d_HSurface) VTrim (const Standard_Real First, const Standard_Real Last, const Standard_Real Tol) const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Boolean IsUClosed() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Boolean IsVClosed() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Boolean IsUPeriodic() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Real UPeriod() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Boolean IsVPeriodic() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Real VPeriod() const Standard_OVERRIDE;
  
  //! Computes the point of parameters U,V on the surface.
  Standard_EXPORT gp_Pnt Value (const Standard_Real U, const Standard_Real V) const Standard_OVERRIDE;
  
  //! Computes the point of parameters U,V on the surface.
  Standard_EXPORT void D0 (const Standard_Real U, const Standard_Real V, gp_Pnt& P) const Standard_OVERRIDE;
  
  //! Computes the point  and the first derivatives on
  //! the surface.
  //! Raised   if  the continuity  of   the  current
  //! intervals is not C1.
  Standard_EXPORT void D1 (const Standard_Real U, const Standard_Real V, gp_Pnt& P, gp_Vec& D1U, gp_Vec& D1V) const Standard_OVERRIDE;
  
  //! Computes   the point,  the  first  and  second
  //! derivatives on the surface.
  //! Raised  if   the   continuity   of the current
  //! intervals is not C2.
  Standard_EXPORT void D2 (const Standard_Real U, const Standard_Real V, gp_Pnt& P, gp_Vec& D1U, gp_Vec& D1V, gp_Vec& D2U, gp_Vec& D2V, gp_Vec& D2UV) const Standard_OVERRIDE;
  
  //! Computes the point,  the first, second and third
  //! derivatives on the surface.
  //! Raised  if   the   continuity   of the current
  //! intervals is not C3.
  Standard_EXPORT void D3 (const Standard_Real U, const Standard_Real V, gp_Pnt& P, gp_Vec& D1U, gp_Vec& D1V, gp_Vec& D2U, gp_Vec& D2V, gp_Vec& D2UV, gp_Vec& D3U, gp_Vec& D3V, gp_Vec& D3UUV, gp_Vec& D3UVV) const Standard_OVERRIDE;
  
  //! Computes the derivative of order Nu
  //! in the direction U and Nv in the direction V
  //! at the point P(U, V).
  //! Raised if the current U  interval is not not CNu
  //! and the current V interval is not CNv.
  //! Raised if Nu + Nv < 1 or Nu < 0 or Nv < 0.
  Standard_EXPORT gp_Vec DN (const Standard_Real U, const Standard_Real V, const Standard_Integer Nu, const Standard_Integer Nv) const Standard_OVERRIDE;
  
  //! Returns the parametric U  resolution corresponding
  //! to the real space resolution <R3d>.
  Standard_EXPORT Standard_Real UResolution (const Standard_Real R3d) const Standard_OVERRIDE;
  
  //! Returns the parametric V  resolution corresponding
  //! to the real space resolution <R3d>.
  Standard_EXPORT Standard_Real VResolution (const Standard_Real R3d) const Standard_OVERRIDE;
  
  //! Returns the type of the surface : Plane, Cylinder,
  //! Cone,      Sphere,        Torus,    BezierSurface,
  //! BSplineSurface,               SurfaceOfRevolution,
  //! SurfaceOfExtrusion, OtherSurface
  Standard_EXPORT GeomAbs_SurfaceType GetType() const Standard_OVERRIDE;
  
  Standard_EXPORT gp_Pln Plane() const Standard_OVERRIDE;
  
  Standard_EXPORT gp_Cylinder Cylinder() const Standard_OVERRIDE;
  
  //! Apex of the Cone = Cone.Position().Location()
  //! ==> ReferenceRadius = 0.
  Standard_EXPORT gp_Cone Cone() const Standard_OVERRIDE;
  
  Standard_EXPORT gp_Sphere Sphere() const Standard_OVERRIDE;
  
  Standard_EXPORT gp_Torus Torus() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Integer UDegree() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Integer NbUPoles() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Integer VDegree() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Integer NbVPoles() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Integer NbUKnots() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Integer NbVKnots() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Boolean IsURational() const Standard_OVERRIDE;
  
  Standard_EXPORT Standard_Boolean IsVRational() const Standard_OVERRIDE;
  
  Standard_EXPORT Handle(Geom_BezierSurface) Bezier() const Standard_OVERRIDE;
  
  Standard_EXPORT Handle(Geom_BSplineSurface) BSpline() const Standard_OVERRIDE;
  
  Standard_EXPORT gp_Ax3 Axis() const;
  
  Standard_EXPORT gp_Dir Direction() const Standard_OVERRIDE;
  
  Standard_EXPORT Handle(Adaptor3d_HCurve) BasisCurve() const Standard_OVERRIDE;




protected:





private:



  Handle(Adaptor3d_HCurve) myBasisCurve;
  gp_Ax1 myAxis;
  Standard_Boolean myHaveAxis;
  gp_Ax3 myAxeRev;


};







#endif // _Adaptor3d_SurfaceOfRevolution_HeaderFile
