// Created on: 2015-07-07
// Created by: Irina KRYLOVA
// Copyright (c) 2015 OPEN CASCADE SAS
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

#include <StepDimTol_GeometricToleranceWithModifiers.hxx>

#include <StepDimTol_GeometricToleranceTarget.hxx>
#include <StepDimTol_HArray1OfGeometricToleranceModifier.hxx>

//=======================================================================
//function : StepDimTol_GeometricToleranceWithModifiers
//purpose  : 
//=======================================================================

StepDimTol_GeometricToleranceWithModifiers::StepDimTol_GeometricToleranceWithModifiers ()
{
}

//=======================================================================
//function : Init
//purpose  : 
//=======================================================================

void StepDimTol_GeometricToleranceWithModifiers::Init (const Handle(TCollection_HAsciiString) &theName,
                                                       const Handle(TCollection_HAsciiString) &theDescription,
                                                       const Handle(StepBasic_MeasureWithUnit) &theMagnitude,
                                                       const StepDimTol_GeometricToleranceTarget &theTolerancedShapeAspect,
                                                       const Handle(StepDimTol_HArray1OfGeometricToleranceModifier) &theModifiers)
{
  StepDimTol_GeometricTolerance::Init(theName, theDescription, theMagnitude, theTolerancedShapeAspect);
  myModifiers = theModifiers;
}
