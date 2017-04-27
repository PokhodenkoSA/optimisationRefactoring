// Created on: 1991-02-26
// Created by: Isabelle GRIGNON
// Copyright (c) 1991-1999 Matra Datavision
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

#ifndef _Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC_HeaderFile
#define _Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC_HeaderFile

#include <Standard.hxx>
#include <Standard_Type.hxx>

#include <Extrema_POnCurv.hxx>
#include <TCollection_SeqNode.hxx>
#include <TCollection_SeqNodePtr.hxx>
class Extrema_POnCurv;
class Extrema_SeqPCOfPCFOfEPCOfELPCOfLocateExtPC;


class Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC;
DEFINE_STANDARD_HANDLE(Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC, TCollection_SeqNode)


class Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC : public TCollection_SeqNode
{

public:

  
    Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC(const Extrema_POnCurv& I, const TCollection_SeqNodePtr& n, const TCollection_SeqNodePtr& p);
  
    Extrema_POnCurv& Value() const;




  DEFINE_STANDARD_RTTI(Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC,TCollection_SeqNode)

protected:




private:


  Extrema_POnCurv myValue;


};

#define SeqItem Extrema_POnCurv
#define SeqItem_hxx <Extrema_POnCurv.hxx>
#define TCollection_SequenceNode Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC
#define TCollection_SequenceNode_hxx <Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC.hxx>
#define Handle_TCollection_SequenceNode Handle(Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC)
#define TCollection_Sequence Extrema_SeqPCOfPCFOfEPCOfELPCOfLocateExtPC
#define TCollection_Sequence_hxx <Extrema_SeqPCOfPCFOfEPCOfELPCOfLocateExtPC.hxx>

#include <TCollection_SequenceNode.lxx>

#undef SeqItem
#undef SeqItem_hxx
#undef TCollection_SequenceNode
#undef TCollection_SequenceNode_hxx
#undef Handle_TCollection_SequenceNode
#undef TCollection_Sequence
#undef TCollection_Sequence_hxx




#endif // _Extrema_SequenceNodeOfSeqPCOfPCFOfEPCOfELPCOfLocateExtPC_HeaderFile
