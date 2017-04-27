// Created on: 2004-01-15
// Created by: Sergey KUUL
// Copyright (c) 2004-2014 OPEN CASCADE SAS
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

#ifndef _XCAFDoc_Datum_HeaderFile
#define _XCAFDoc_Datum_HeaderFile

#include <Standard.hxx>
#include <Standard_Type.hxx>

#include <TDF_Attribute.hxx>
class TCollection_HAsciiString;
class Standard_GUID;
class TDF_Label;
class TDF_Attribute;
class TDF_RelocationTable;
class XCAFDimTolObjects_DatumObject;


class XCAFDoc_Datum;
DEFINE_STANDARD_HANDLE(XCAFDoc_Datum, TDF_Attribute)

//! attribute to store datum
class XCAFDoc_Datum : public TDF_Attribute
{

public:

  
  Standard_EXPORT XCAFDoc_Datum();
  
  Standard_EXPORT static const Standard_GUID& GetID();
  
  Standard_EXPORT static Handle(XCAFDoc_Datum) Set (const TDF_Label& label, const Handle(TCollection_HAsciiString)& aName, const Handle(TCollection_HAsciiString)& aDescription, const Handle(TCollection_HAsciiString)& anIdentification);

  Standard_EXPORT static   Handle(XCAFDoc_Datum) Set (const TDF_Label& theLabel);
  
  Standard_EXPORT void Set (const Handle(TCollection_HAsciiString)& aName, const Handle(TCollection_HAsciiString)& aDescription, const Handle(TCollection_HAsciiString)& anIdentification);
  
  Standard_EXPORT Handle(TCollection_HAsciiString) GetName() const;
  
  Standard_EXPORT Handle(TCollection_HAsciiString) GetDescription() const;
  
  Standard_EXPORT Handle(TCollection_HAsciiString) GetIdentification() const;
  
  Standard_EXPORT Handle(XCAFDimTolObjects_DatumObject) GetObject() const;
  
  Standard_EXPORT void SetObject (const Handle(XCAFDimTolObjects_DatumObject)& theObject);
      
  Standard_EXPORT const Standard_GUID& ID() const;
  
  Standard_EXPORT void Restore (const Handle(TDF_Attribute)& With);
  
  Standard_EXPORT Handle(TDF_Attribute) NewEmpty() const;
  
  Standard_EXPORT void Paste (const Handle(TDF_Attribute)& Into, const Handle(TDF_RelocationTable)& RT) const;




  DEFINE_STANDARD_RTTI(XCAFDoc_Datum,TDF_Attribute)

protected:




private:


  Handle(TCollection_HAsciiString) myName;
  Handle(TCollection_HAsciiString) myDescription;
  Handle(TCollection_HAsciiString) myIdentification;


};







#endif // _XCAFDoc_Datum_HeaderFile