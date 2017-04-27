// Created on: 2001-07-25
// Created by: Julia DOROVSKIKH
// Copyright (c) 2001-2014 OPEN CASCADE SAS
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

#ifndef _XmlLDrivers_DocumentStorageDriver_HeaderFile
#define _XmlLDrivers_DocumentStorageDriver_HeaderFile

#include <Standard.hxx>
#include <Standard_Type.hxx>

#include <XmlLDrivers_SequenceOfNamespaceDef.hxx>
#include <TCollection_ExtendedString.hxx>
#include <XmlObjMgt_SRelocationTable.hxx>
#include <PCDM_StorageDriver.hxx>
#include <Standard_Boolean.hxx>
#include <XmlObjMgt_Element.hxx>
#include <Standard_Integer.hxx>
class XmlMDF_ADriverTable;
class TCollection_ExtendedString;
class CDM_Document;
class TCollection_AsciiString;
class CDM_MessageDriver;


class XmlLDrivers_DocumentStorageDriver;
DEFINE_STANDARD_HANDLE(XmlLDrivers_DocumentStorageDriver, PCDM_StorageDriver)


class XmlLDrivers_DocumentStorageDriver : public PCDM_StorageDriver
{

public:

  
  Standard_EXPORT XmlLDrivers_DocumentStorageDriver(const TCollection_ExtendedString& theCopyright);
  
  Standard_EXPORT virtual TCollection_ExtendedString SchemaName() const Standard_OVERRIDE;
  
  Standard_EXPORT virtual void Write (const Handle(CDM_Document)& theDocument, const TCollection_ExtendedString& theFileName) Standard_OVERRIDE;
  
  Standard_EXPORT virtual Handle(XmlMDF_ADriverTable) AttributeDrivers (const Handle(CDM_MessageDriver)& theMsgDriver);




  DEFINE_STANDARD_RTTI(XmlLDrivers_DocumentStorageDriver,PCDM_StorageDriver)

protected:

  
  Standard_EXPORT virtual Standard_Boolean WriteToDomDocument (const Handle(CDM_Document)& theDocument, XmlObjMgt_Element& thePDoc, const TCollection_ExtendedString& theFileName);
  
  Standard_EXPORT virtual Standard_Integer MakeDocument (const Handle(CDM_Document)& theDocument, XmlObjMgt_Element& thePDoc);
  
  Standard_EXPORT void AddNamespace (const TCollection_AsciiString& thePrefix, const TCollection_AsciiString& theURI);
  
  Standard_EXPORT virtual Standard_Boolean WriteShapeSection (XmlObjMgt_Element& thePDoc);

  Handle(XmlMDF_ADriverTable) myDrivers;
  XmlObjMgt_SRelocationTable myRelocTable;


private:


  XmlLDrivers_SequenceOfNamespaceDef mySeqOfNS;
  TCollection_ExtendedString myCopyright;


};







#endif // _XmlLDrivers_DocumentStorageDriver_HeaderFile
