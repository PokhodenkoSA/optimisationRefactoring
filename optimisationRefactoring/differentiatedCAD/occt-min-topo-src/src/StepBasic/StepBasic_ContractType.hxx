// Created on: 1999-11-26
// Created by: Andrey BETENEV
// Copyright (c) 1999 Matra Datavision
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

#ifndef _StepBasic_ContractType_HeaderFile
#define _StepBasic_ContractType_HeaderFile

#include <Standard.hxx>
#include <Standard_Type.hxx>

#include <MMgt_TShared.hxx>
class TCollection_HAsciiString;


class StepBasic_ContractType;
DEFINE_STANDARD_HANDLE(StepBasic_ContractType, MMgt_TShared)

//! Representation of STEP entity ContractType
class StepBasic_ContractType : public MMgt_TShared
{

public:

  
  //! Empty constructor
  Standard_EXPORT StepBasic_ContractType();
  
  //! Initialize all fields (own and inherited)
  Standard_EXPORT void Init (const Handle(TCollection_HAsciiString)& aDescription);
  
  //! Returns field Description
  Standard_EXPORT Handle(TCollection_HAsciiString) Description() const;
  
  //! Set field Description
  Standard_EXPORT void SetDescription (const Handle(TCollection_HAsciiString)& Description);




  DEFINE_STANDARD_RTTI(StepBasic_ContractType,MMgt_TShared)

protected:




private:


  Handle(TCollection_HAsciiString) theDescription;


};







#endif // _StepBasic_ContractType_HeaderFile
