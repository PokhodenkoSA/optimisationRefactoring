// Created on: 2002-02-20
// Created by: Andrey BETENEV
// Copyright (c) 2002-2014 OPEN CASCADE SAS
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

#ifndef _Message_ProgressScale_HeaderFile
#define _Message_ProgressScale_HeaderFile

#include <Standard.hxx>
#include <Standard_DefineAlloc.hxx>
#include <Standard_Handle.hxx>

#include <Standard_Real.hxx>
#include <Standard_Boolean.hxx>
#include <Standard_CString.hxx>
class TCollection_HAsciiString;


//! Internal data structure for scale in ProgressIndicator
//!
//! Basically it defines three things:
//! - name that can be used for generating user messages
//! - limits and characteristics of the current scale,
//! along with derived coefficients to map it into basic scale [0-1]
//! - methods for conversion of values from current scale
//! to basic one and back
//!
//! NOTE: There is no special protection against bad input data
//! like min > max etc. except cases when it can cause exception
class Message_ProgressScale 
{
public:

  DEFINE_STANDARD_ALLOC

  
  //! Creates scale ranged from 0 to 100 with step 1
  Standard_EXPORT Message_ProgressScale();
  
    void SetName (const Standard_CString theName);
  
  //! Sets scale name
    void SetName (const Handle(TCollection_HAsciiString)& theName);
  
  //! Gets scale name
  //! Name may be Null handle if not set
    Handle(TCollection_HAsciiString) GetName() const;
  
  //! Sets minimum value of scale
    void SetMin (const double theMin);
  
  //! Gets minimum value of scale
    double GetMin() const;
  
  //! Sets minimum value of scale
    void SetMax (const double theMax);
  
  //! Gets minimum value of scale
    double GetMax() const;
  
  //! Set both min and max
    void SetRange (const double min, const double max);
  
  //! Sets default step
    void SetStep (const double theStep);
  
  //! Gets default step
    double GetStep() const;
  
  //! Sets flag for infinite scale
    void SetInfinite (const Standard_Boolean theInfinite = Standard_True);
  
  //! Gets flag for infinite scale
    Standard_Boolean GetInfinite() const;
  
  //! Set all scale parameters
    void SetScale (const double min, const double max, const double step, const Standard_Boolean theInfinite = Standard_True);
  
  //! Defines span occupied by the scale on the basis scale
    void SetSpan (const double first, const double last);
  
    double GetFirst() const;
  
  //! Return information on span occupied by the scale on the base scale
    double GetLast() const;
  
  Standard_EXPORT double LocalToBase (const double val) const;
  
  //! Convert value from this scale to base one and back
  Standard_EXPORT double BaseToLocal (const double val) const;




protected:





private:



  Handle(TCollection_HAsciiString) myName;
  double myMin;
  double myMax;
  double myStep;
  Standard_Boolean myInfinite;
  double myFirst;
  double myLast;


};


#include <Message_ProgressScale.lxx>





#endif // _Message_ProgressScale_HeaderFile
