// Created on: 2003-09-29
// Created by: Alexander SOLOVYOV and Sergey LITONIN
// Copyright (c) 2003-2014 OPEN CASCADE SAS
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

#ifndef _MeshVS_DummySensitiveEntity_HeaderFile
#define _MeshVS_DummySensitiveEntity_HeaderFile

#include <Standard.hxx>
#include <Standard_Type.hxx>

#include <Select3D_BndBox3d.hxx>
#include <SelectBasics_SensitiveEntity.hxx>
#include <SelectBasics_SelectingVolumeManager.hxx>

class SelectBasics_EntityOwner;


//! This class allows to create owners to all elements or nodes,
//! both hidden and shown, but these owners user cannot select "by hands"
//! in viewer. They means for internal application tasks, for example, receiving
//! all owners, both for hidden and shown entities.
class MeshVS_DummySensitiveEntity : public SelectBasics_SensitiveEntity
{
public:

  Standard_EXPORT MeshVS_DummySensitiveEntity (const Handle(SelectBasics_EntityOwner)& theOwnerId);

  Standard_EXPORT virtual Standard_Boolean Matches (SelectBasics_SelectingVolumeManager& theMgr,
                                                    SelectBasics_PickResult& thePickResult) Standard_OVERRIDE;

  Standard_EXPORT virtual Standard_Integer NbSubElements() Standard_OVERRIDE;

  Standard_EXPORT virtual Select3D_BndBox3d BoundingBox() Standard_OVERRIDE;

  Standard_EXPORT virtual void BVH() Standard_OVERRIDE;

  Standard_EXPORT virtual void Clear() Standard_OVERRIDE;

  Standard_EXPORT virtual Standard_Boolean HasInitLocation() const Standard_OVERRIDE;

  Standard_EXPORT virtual gp_Trsf InvInitLocation() const Standard_OVERRIDE;

  DEFINE_STANDARD_RTTI(MeshVS_DummySensitiveEntity, SelectBasics_SensitiveEntity)
};

DEFINE_STANDARD_HANDLE(MeshVS_DummySensitiveEntity, SelectBasics_SensitiveEntity)

#endif // _MeshVS_DummySensitiveEntity_HeaderFile
