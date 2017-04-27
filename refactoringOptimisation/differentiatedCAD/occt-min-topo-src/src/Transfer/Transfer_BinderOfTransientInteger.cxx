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
#include <Transfer_BinderOfTransientInteger.hxx>

Transfer_BinderOfTransientInteger::Transfer_BinderOfTransientInteger ()
    : theintval (0)    {  }

    void  Transfer_BinderOfTransientInteger::SetInteger
  (const Standard_Integer val)
      {  theintval = val;  }

    Standard_Integer  Transfer_BinderOfTransientInteger::Integer () const
      {  return theintval;  }
