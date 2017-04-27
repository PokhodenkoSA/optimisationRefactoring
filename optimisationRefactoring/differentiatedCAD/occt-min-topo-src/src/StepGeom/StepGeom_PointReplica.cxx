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


#include <Standard_Type.hxx>
#include <StepGeom_CartesianTransformationOperator.hxx>
#include <StepGeom_Point.hxx>
#include <StepGeom_PointReplica.hxx>
#include <TCollection_HAsciiString.hxx>

StepGeom_PointReplica::StepGeom_PointReplica ()  {}

void StepGeom_PointReplica::Init(
	const Handle(TCollection_HAsciiString)& aName)
{

	StepRepr_RepresentationItem::Init(aName);
}

void StepGeom_PointReplica::Init(
	const Handle(TCollection_HAsciiString)& aName,
	const Handle(StepGeom_Point)& aParentPt,
	const Handle(StepGeom_CartesianTransformationOperator)& aTransformation)
{
	// --- classe own fields ---
	parentPt = aParentPt;
	transformation = aTransformation;
	// --- classe inherited fields ---
	StepRepr_RepresentationItem::Init(aName);
}


void StepGeom_PointReplica::SetParentPt(const Handle(StepGeom_Point)& aParentPt)
{
	parentPt = aParentPt;
}

Handle(StepGeom_Point) StepGeom_PointReplica::ParentPt() const
{
	return parentPt;
}

void StepGeom_PointReplica::SetTransformation(const Handle(StepGeom_CartesianTransformationOperator)& aTransformation)
{
	transformation = aTransformation;
}

Handle(StepGeom_CartesianTransformationOperator) StepGeom_PointReplica::Transformation() const
{
	return transformation;
}
