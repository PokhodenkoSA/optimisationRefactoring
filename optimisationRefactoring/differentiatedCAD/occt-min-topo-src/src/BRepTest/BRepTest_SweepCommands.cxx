// Created on: 1993-07-22
// Created by: Remi LEQUETTE
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

#include <BRepTest.hxx>
#include <DBRep.hxx>
#include <Draw_Interpretor.hxx>
#include <Draw_Appli.hxx>

#include <BRepFill.hxx>
#include <BRepBuilderAPI_PipeError.hxx>
#include <BRepFill_Generator.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakeEvolved.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_MiddlePath.hxx>

#include <BRepLib_MakeWire.hxx>
#include <TopoDS.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopExp_Explorer.hxx>

#include <Precision.hxx>
#include <Law_Interpol.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt2d.hxx>
#include <TColgp_Array1OfPnt2d.hxx>

static BRepOffsetAPI_MakePipeShell* Sweep= 0;

#include <stdio.h>
#include <Geom_Curve.hxx>
#include <GeomAdaptor_HCurve.hxx>
#include <GeomFill_Pipe.hxx>
#include <Geom_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <Geom_Circle.hxx>
#include <gp_Ax2.hxx>

//ubend includes
#include <DrawTrSurf.hxx>
#include <Geom_BezierCurve.hxx>
#include <gp_Pln.hxx>
#include <Geom_Plane.hxx>
#include <Geom2d_Circle.hxx>
#include <GCE2d_MakeCircle.hxx>
#include <GeomAPI.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TCollection_AsciiString.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TColStd_HSequenceOfReal.hxx>
//end ubend


//=======================================================================
// prism
//=======================================================================

static Standard_Integer prism(Draw_Interpretor& , Standard_Integer n, const char** a)
{
  if (n < 6) return 1;

  TopoDS_Shape base = DBRep::Get(a[2]);
  if (base.IsNull()) return 1;

  gp_Vec V(Draw::Atof(a[3]),Draw::Atof(a[4]),Draw::Atof(a[5]));
  
  Standard_Boolean copy = Standard_False;
  Standard_Boolean inf  = Standard_False;
  Standard_Boolean sinf = Standard_False;

  if (n > 6) {
    copy = (*a[6] == 'c') || (*a[6] == 'C');
    inf  = (*a[6] == 'i') || (*a[6] == 'I');
    sinf = (*a[6] == 's') || (*a[6] == 'S');
  }

  TopoDS_Shape res;

  if (inf || sinf) 
    res = BRepPrimAPI_MakePrism(base,gp_Dir(V),inf);
  else
    res = BRepPrimAPI_MakePrism(base,V,copy);

 DBRep::Set(a[1],res);

  return 0;
}


//=======================================================================
// revol
//=======================================================================

static Standard_Integer revol(Draw_Interpretor& ,
			      Standard_Integer n, const char** a)
{
  if (n < 10) return 1; 

  TopoDS_Shape base = DBRep::Get(a[2]);
  if (base.IsNull()) return 1;

  gp_Pnt P(Draw::Atof(a[3]),Draw::Atof(a[4]),Draw::Atof(a[5]));
  gp_Dir D(Draw::Atof(a[6]),Draw::Atof(a[7]),Draw::Atof(a[8]));
  gp_Ax1 A(P,D);

  Standard_Real angle = Draw::Atof(a[9]) * (M_PI / 180.0);
  
  Standard_Boolean copy = n > 10;

  TopoDS_Shape res = BRepPrimAPI_MakeRevol(base,A,angle,copy);

  DBRep::Set(a[1],res);

  return 0;
}


//=======================================================================
// pipe
//=======================================================================

static Standard_Integer pipe(Draw_Interpretor& di,
			     Standard_Integer n, const char** a)
{
  if (n == 1)
  {
    di << "pipe result Wire_spine Profile [Mode [Approx]]" << "\n";
    di << "Mode = 0 - CorrectedFrenet," << "\n";
    di << "     = 1 - Frenet," << "\n";
    di << "     = 2 - DiscreteTrihedron" << "\n";
    di << "Approx - force C1-approximation if result is C0" << "\n";
    return 0;
  }
  
  if (n > 1 && n < 4) return 1;

  TopoDS_Shape Spine = DBRep::Get(a[2],TopAbs_WIRE);
  if ( Spine.IsNull()) return 1;

  TopoDS_Shape Profile = DBRep::Get(a[3]);
  if ( Profile.IsNull()) return 1;

  GeomFill_Trihedron Mode = GeomFill_IsCorrectedFrenet;
  if (n >= 5)
  {
    Standard_Integer iMode = atoi(a[4]);
    if (iMode == 1)
      Mode = GeomFill_IsFrenet;
    else if (iMode == 2)
      Mode = GeomFill_IsDiscreteTrihedron;
  }

  Standard_Boolean ForceApproxC1 = Standard_False;
  if (n >= 6)
    ForceApproxC1 = Standard_True;
  
  TopoDS_Shape S = BRepOffsetAPI_MakePipe(TopoDS::Wire(Spine),
                                          Profile,
                                          Mode,
                                          ForceApproxC1);

  DBRep::Set(a[1],S);
  
  return 0;
}

//=======================================================================

static Standard_Integer geompipe(Draw_Interpretor& ,
			     Standard_Integer n, const char** a)
{
  TopoDS_Shape Spine = DBRep::Get(a[2],TopAbs_EDGE);
  if ( Spine.IsNull()) return 1;
  if ( n < 5) return 1;
  TopoDS_Shape Profile = DBRep::Get(a[3],TopAbs_EDGE);
  if ( Profile.IsNull()) return 1;
  Standard_Real aSpFirst,aSpLast,aPrFirst,aPrLast;
  Handle(Geom_Curve) SpineCurve = BRep_Tool::Curve(TopoDS::Edge(Spine),aSpFirst,aSpLast);
  Handle(Geom_Curve) ProfileCurve = BRep_Tool::Curve(TopoDS::Edge(Profile),aPrFirst,aPrLast);
  Handle(GeomAdaptor_HCurve) aAdaptCurve = new GeomAdaptor_HCurve(SpineCurve,aSpFirst,aSpLast);
  Standard_Boolean ByACR = Standard_False;
  Standard_Boolean rotate = Standard_False;
  Standard_Real Radius = Draw::Atof(a[4]);
  gp_Pnt ctr;
  gp_Vec norm;
  ProfileCurve->D1(aSpFirst,ctr,norm);
  gp_Vec xAxisStart(ctr,SpineCurve->Value(aSpFirst));
  gp_Ax2 aAx2Start(ctr,norm,xAxisStart);
  Handle(Geom_Circle) cStart=new Geom_Circle(aAx2Start,Radius);                       
  Standard_Integer k =5;
  if(n > k)
    ByACR = (Draw::Atoi(a[k++]) ==1);
  if(n > k)
    rotate = (Draw::Atoi(a[k++])==1);
  GeomFill_Pipe aPipe(ProfileCurve,aAdaptCurve,cStart,ByACR,rotate);
  aPipe.Perform(Standard_True);
  Handle(Geom_Surface) Sur=aPipe.Surface();
  TopoDS_Face F;
  if(!Sur.IsNull())
    F = BRepBuilderAPI_MakeFace(Sur, Precision::Confusion());
  DBRep::Set(a[1],F);
  return 0;
}

//=======================================================================
//function : evolved
//purpose  : 
//=======================================================================

Standard_Integer evolved(Draw_Interpretor& di, Standard_Integer n, const char** a)
{
  if ( n == 1) {
    //cout << " 1) evolved result base profil : "<< endl;
    //cout << "        The relative position of the profil on the base" << endl;
    //cout << "        is given in the referencial axis. " << endl;
    //cout << " 2) evolved result base profil o : "<< endl;
    //cout << "        This position is automatically computed." << endl;
    di << " 1) evolved result base profil : "<< "\n";
    di << "        The relative position of the profil on the base" << "\n";
    di << "        is given in the referencial axis. " << "\n";
    di << " 2) evolved result base profil o : "<< "\n";
    di << "        This position is automatically computed." << "\n";
    return 0;
  }

  if ( n < 4 ) return 1;
  Standard_Boolean IsAFace = Standard_False;
  Standard_Boolean Solid   = (!strcmp(a[0],"evolvedsolid"));


 
  TopoDS_Shape Base = DBRep::Get(a[2],TopAbs_WIRE,Standard_False);
  if ( Base.IsNull()) {
    Base = DBRep::Get(a[2],TopAbs_FACE,Standard_False);
    IsAFace = Standard_True; 
  }
  if ( Base.IsNull()) return 1;

  TopoDS_Shape InpuTShape(DBRep::Get(a[3],TopAbs_WIRE,Standard_False));
  TopoDS_Wire Prof = TopoDS::Wire(InpuTShape);
//  TopoDS_Wire Prof = 
//    TopoDS::Wire(DBRep::Get(a[3],TopAbs_WIRE,Standard_False));
  if ( Prof.IsNull()) return 1;

  if (IsAFace) {
    TopoDS_Shape Volevo 
      = BRepOffsetAPI_MakeEvolved(TopoDS::Face(Base),Prof,GeomAbs_Arc,n == 4,Solid);
    DBRep::Set(a[1],Volevo);
  }
  else {
    TopoDS_Shape Volevo 
      = BRepOffsetAPI_MakeEvolved(TopoDS::Wire(Base),Prof,GeomAbs_Arc,n == 4,Solid);
    DBRep::Set(a[1],Volevo);
  }

  return 0;
}


//=======================================================================
//function : pruled
//purpose  : 
//=======================================================================

static Standard_Integer pruled(Draw_Interpretor& ,
			       Standard_Integer n, const char** a)
{
  if ( n != 4) return 1;

  Standard_Boolean YaWIRE = Standard_False;
  TopoDS_Shape S1 = DBRep::Get(a[2],TopAbs_EDGE);
  if ( S1.IsNull()) {
    S1 = DBRep::Get(a[2],TopAbs_WIRE);
    if (S1.IsNull()) return 1;
    YaWIRE = Standard_True;
  }

  TopoDS_Shape S2 = DBRep::Get(a[3],TopAbs_EDGE);
  if ( S2.IsNull()) {
    S2 = DBRep::Get(a[3],TopAbs_WIRE);
    if ( S2.IsNull()) return 1;
    if (!YaWIRE) {
      S1 = BRepLib_MakeWire(TopoDS::Edge(S1));
      YaWIRE = Standard_True;
    }
  }
  else if ( YaWIRE) {
    S2 = BRepLib_MakeWire(TopoDS::Edge(S2));
  }

  TopoDS_Shape Result;
  if ( YaWIRE) {
    Result = BRepFill::Shell(TopoDS::Wire(S1),TopoDS::Wire(S2));
  }
  else {
    Result = BRepFill::Face(TopoDS::Edge(S1),TopoDS::Edge(S2));
  }

  DBRep::Set(a[1],Result);
  return 0;
}


//=======================================================================
//function : gener
//purpose  : Create a surface between generating wires
//=======================================================================

Standard_Integer gener(Draw_Interpretor&, Standard_Integer n, const char** a)
{
  if ( n < 4) return 1;

  TopoDS_Shape Shape; 

  BRepFill_Generator Generator;
  
  for ( Standard_Integer i = 2; i<= n-1 ; i++) {
    Shape = DBRep::Get(a[i],TopAbs_WIRE);
    if ( Shape.IsNull()) 
      return 1;

    Generator.AddWire(TopoDS::Wire(Shape));
  }

  Generator.Perform();

  TopoDS_Shell Shell = Generator.Shell();
  
  DBRep::Set(a[1], Shell);


  return 0;
}


//=======================================================================
//function : thrusections
//purpose  : 
//=======================================================================

Standard_Integer thrusections(Draw_Interpretor&, Standard_Integer n, const char** a)
{
  if (n<6) return 1;

  Standard_Boolean check = Standard_True;
  Standard_Boolean samenumber = Standard_True;
  Standard_Integer index = 2;
    // Lecture option
  if (!strcmp(a[1],"-N")) {
    if (n<7) return 1;
    check = Standard_False;
    index++;
  }

  TopoDS_Shape Shape; 

  Standard_Boolean issolid = ( Draw::Atoi(a[index]) == 1 );
  Standard_Boolean isruled = ( Draw::Atoi(a[index+1]) == 1 );

  BRepOffsetAPI_ThruSections Generator(issolid,isruled);
  
  Standard_Integer NbEdges = 0;
  Standard_Boolean IsFirstWire = Standard_False;
  for ( Standard_Integer i = index+2; i<= n-1 ; i++) {
    Standard_Boolean IsWire = Standard_True;
    Shape = DBRep::Get(a[i], TopAbs_WIRE);
    if (!Shape.IsNull())
      {
	Generator.AddWire(TopoDS::Wire(Shape));
	if (!IsFirstWire)
	  IsFirstWire = Standard_True;
	else
	  IsFirstWire = Standard_False;
      }
    else
      {
	Shape = DBRep::Get(a[i], TopAbs_VERTEX);
	IsWire = Standard_False;
	if (!Shape.IsNull())
	  Generator.AddVertex(TopoDS::Vertex(Shape));
	else
	  return 1;
      }

    Standard_Integer cpt = 0;
    TopExp_Explorer PE;
    for (PE.Init(Shape, TopAbs_EDGE); PE.More(); PE.Next()) {
      cpt++;
    }
    if (IsFirstWire) 
      NbEdges = cpt;
    else
      if (IsWire && cpt != NbEdges)
	samenumber = Standard_False;
    
  }

  check = (check || !samenumber);
  Generator.CheckCompatibility(check);

  Generator.Build();

  if (Generator.IsDone()) {
    TopoDS_Shape Shell = Generator.Shape();
    DBRep::Set(a[index-1], Shell);
  }
  else {
    cout << "Algorithm is not done" << endl;
  }
  return 0;
}

//=======================================================================
//  mksweep
//=======================================================================
static Standard_Integer mksweep(Draw_Interpretor& ,
			     Standard_Integer n, const char** a)
{
  if ( n != 2) return 1;
  TopoDS_Shape Spine = DBRep::Get(a[1],TopAbs_WIRE);
  if ( Spine.IsNull()) return 1;
  if (Sweep !=0)  {
    delete Sweep; 
    Sweep = 0;
  }
  Sweep = new BRepOffsetAPI_MakePipeShell(TopoDS::Wire(Spine));
  return 0;
}

//=======================================================================
//  setsweep
//=======================================================================
static Standard_Integer setsweep(Draw_Interpretor& di,
				 Standard_Integer n, const char** a)
{
  if ( n == 1) {
    //cout << "setsweep options [arg1 [arg2 [...]]] : options are :" << endl;
    //cout << "   -FR : Tangent and Normal are given by Frenet trihedron" <<endl;
    //cout << "   -CF : Tangente is given by Frenet," << endl;
    //cout << "         the Normal is computed to minimize the torsion " << endl;
    //cout << "   -DX Surf : Tangent and Normal are given by Darboux trihedron,"
    //  <<endl;     
    //cout << "       Surf have to be a shell or a face" <<endl;
    //cout << "   -CN dx dy dz : BiNormal is given by dx dy dz" << endl;
    //cout << "   -FX Tx Ty TZ [Nx Ny Nz] : Tangent and Normal are fixed" <<endl;
    //cout << "   -G guide  0|1(ACR|Plan)  0|1(contact|no contact) : with guide"<<endl;
    di << "setsweep options [arg1 [arg2 [...]]] : options are :" << "\n";
    di << "   -FR : Tangent and Normal are given by Frenet trihedron" <<"\n";
    di << "   -CF : Tangente is given by Frenet," << "\n";
    di << "         the Normal is computed to minimize the torsion " << "\n";
    di << "   -DT : discrete trihedron" << "\n";
    di << "   -DX Surf : Tangent and Normal are given by Darboux trihedron," <<"\n";     
    di << "       Surf have to be a shell or a face" <<"\n";
    di << "   -CN dx dy dz : BiNormal is given by dx dy dz" << "\n";
    di << "   -FX Tx Ty TZ [Nx Ny Nz] : Tangent and Normal are fixed" <<"\n";
    di << "   -G guide  0|1(Plan|ACR)  0|1|2(no contact|contact|contact on border) : with guide"<<"\n";
    return 0;
  }

   if (Sweep ==0) {
     //cout << "You have forgotten the <<mksweep>> command  !"<< endl;
     di << "You have forgotten the <<mksweep>> command  !"<< "\n";
     return 1;
   }
  if (!strcmp(a[1],"-FR")) {
    Sweep->SetMode(Standard_True);
  }
  else if (!strcmp(a[1],"-CF")) {
    Sweep->SetMode(Standard_False);
  }
  else if (!strcmp(a[1],"-DT")) {
    Sweep->SetDiscreteMode();
  }
  else if (!strcmp(a[1],"-DX")) {
    if (n!=3) {
      //cout << "bad arguments !" << endl;
      di << "bad arguments !" << "\n";
      return 1;
    }
    TopoDS_Shape Surf;
    Surf = DBRep::Get(a[2],TopAbs_SHAPE);
    if (Surf.IsNull()) {
       //cout << a[2] <<"is not a shape !" << endl;
       di << a[2] <<"is not a shape !" << "\n";
      return 1;
    }
    Sweep->SetMode(Surf);
  }
  else if (!strcmp(a[1],"-CN")) {
    if (n!=5) {
      //cout << "bad arguments !" << endl;
      di << "bad arguments !" << "\n";
      return 1;
    }
    gp_Dir D(Draw::Atof(a[2]), Draw::Atof(a[3]), Draw::Atof(a[4]));
    Sweep->SetMode(D);;
  }
  else if (!strcmp(a[1],"-FX")) {
    if ((n!=5)&&(n!=8)) {
      //cout << "bad arguments !" << endl;
      di << "bad arguments !" << "\n";
      return 1;
    }
    gp_Dir D(Draw::Atof(a[2]), Draw::Atof(a[3]), Draw::Atof(a[4]));
    if (n==8) {
      gp_Dir DN(Draw::Atof(a[5]), Draw::Atof(a[6]), Draw::Atof(a[7]));
      gp_Ax2 Axe(gp_Pnt(0., 0., 0.), D, DN);
      Sweep->SetMode(Axe);
    }
    else {
      gp_Ax2 Axe(gp_Pnt(0., 0., 0.), D);
      Sweep->SetMode(Axe);
    }
  }
  else if (!strcmp(a[1],"-G"))  // contour guide
    {
     if (n != 5)
       {
	 //cout << "bad arguments !" << endl;
	 di << "bad arguments !" << "\n";
	 return 1; 
       }
     else
	{  
	  TopoDS_Shape Guide = DBRep::Get(a[2],TopAbs_WIRE);
          Standard_Integer CurvilinearEquivalence = Draw::Atoi(a[3]);
          Standard_Integer KeepContact = Draw::Atoi(a[4]);
          Sweep->SetMode(TopoDS::Wire(Guide),
                         CurvilinearEquivalence,
                         (BRepFill_TypeOfContact)KeepContact);
	}
    }
 
  else {
    //cout << "The option "<< a[1] << " is unknown !" << endl;
    di << "The option "<< a[1] << " is unknown !" << "\n";
    return 1;
  }
  return 0;
}


//=======================================================================
//  addsweep
//=======================================================================
static Standard_Integer addsweep(Draw_Interpretor& di,
			     Standard_Integer n, const char** a)
{
  if ( n == 1) {
    //cout << "addsweep wire/vertex [Vertex] [-T] [-R] [u0 v0 u1 v1 [...[uN vN]]] : options are :" << endl;
    //cout << "   -T : the wire/vertex have to be translated to assume contact"<< endl;
    //cout << "        with the spine" <<endl;
    //cout << "   -R : the wire have to be rotated to assume orthogonality"<<endl;
    //cout << "        with the spine's tangent" << endl;
    di << "addsweep wire/vertex [Vertex] [-T] [-R] [u0 v0 u1 v1 [...[uN vN]]] : options are :" << "\n";
    di << "   -T : the wire/vertex have to be translated to assume contact"<< "\n";
    di << "        with the spine" <<"\n";
    di << "   -R : the wire have to be rotated to assume orthogonality"<<"\n";
    di << "        with the spine's tangent" << "\n";
    return 0;
  }

  if (Sweep ==0) {
    //cout << "You have forgotten the <<mksweep>> command  !"<< endl;
    di << "You have forgotten the <<mksweep>> command  !"<< "\n";
    return 1;
  }

  TopoDS_Shape  Section;
  TopoDS_Vertex Vertex;
  Handle(Law_Interpol) thelaw;

  Section = DBRep::Get(a[1], TopAbs_SHAPE);
  if (Section.ShapeType() != TopAbs_WIRE &&
      Section.ShapeType() != TopAbs_VERTEX)
    {
      //cout << a[1] <<"is not a wire and is not a vertex!" << endl;
      di << a[1] <<"is not a wire and is not a vertex!" << "\n";
      return 1;
    }

  Standard_Boolean HasVertex=Standard_False, 
                   isT=Standard_False, 
                   isR=Standard_False;

  if (n > 2) { 
    Standard_Integer cur = 2;
    // Reading of Vertex
    TopoDS_Shape InputVertex(DBRep::Get(a[cur],TopAbs_VERTEX));
    Vertex = TopoDS::Vertex(InputVertex);
//    Vertex = TopoDS::Vertex(DBRep::Get(a[cur],TopAbs_VERTEX));
    if (!Vertex.IsNull()) {
      cur++;
      HasVertex = Standard_True;
    }
   
    // Reading of the translation option
    if ((n>cur) && !strcmp(a[cur],"-T")) {
      cur++;
      isT = Standard_True;
    }

    // Reading of the rotation option
    if ((n>cur) && !strcmp(a[cur],"-R")) {
      cur++;
      isR = Standard_True;
    }

    // law ?
    if (n>cur) {
      Standard_Integer nbreal = n-cur;
      if ( (nbreal < 4) || (nbreal % 2 != 0) ) {
	//cout << "bad arguments ! :" <<a[cur] << endl;
	di << "bad arguments ! :" <<a[cur] << "\n";
      } else { //law of interpolation
	Standard_Integer ii, L= nbreal/2;
	TColgp_Array1OfPnt2d ParAndRad(1, L);
	for (ii=1; ii<=L; ii++, cur+=2) {
          ParAndRad(ii).SetX(Draw::Atof(a[cur]));
          ParAndRad(ii).SetY(Draw::Atof(a[cur+1]));
        }
	thelaw = new (Law_Interpol) ();
	thelaw->Set(ParAndRad, 
		   Abs(ParAndRad(1).Y() - ParAndRad(L).Y()) < Precision::Confusion());
      }
    }
  }

  if (thelaw.IsNull()) {
    if (HasVertex) Sweep->Add(Section, Vertex, isT, isR);
    else           Sweep->Add(Section, isT, isR);
  }
  else {
    if (HasVertex) Sweep->SetLaw(Section, thelaw, Vertex, isT, isR);
    else           Sweep->SetLaw(Section, thelaw, isT, isR);
  }

  return 0;
}

//=======================================================================
//  deletesweep
//=======================================================================
static Standard_Integer deletesweep(Draw_Interpretor& di,
				    Standard_Integer n, const char** a)
{
  if ( n != 2) {
    return 1;
  }
  TopoDS_Wire Section;
  TopoDS_Shape InputShape(DBRep::Get(a[1],TopAbs_SHAPE));
  Section = TopoDS::Wire(InputShape);
//  Section = TopoDS::Wire(DBRep::Get(a[1],TopAbs_SHAPE));
  if (Section.IsNull()) {
    //cout << a[1] <<"is not a wire !" << endl;
    di << a[1] <<"is not a wire !" << "\n";
    return 1;
  }  

  Sweep->Delete(Section);

  return 0;
}

//=======================================================================
//  buildsweep
//=======================================================================
static Standard_Integer buildsweep(Draw_Interpretor& di,
				   Standard_Integer n, const char** a)
{
  if ( n == 1) {
    //cout << "build sweep result [-M/-C/-R] [-S] [tol] : options are" << endl;
    //cout << "   -M : Discontinuities are treated by Modfication of"<< endl; 
    //cout << "        the sweeping mode : it is the default" <<endl;
    //cout << "   -C : Discontinuities are treated like Right Corner" << endl;
    //cout << "        Treatement is Extent && Intersect" << endl;
    //cout << "   -R : Discontinuities are treated like Round Corner" << endl;
    //cout << "        Treatement is Intersect and Fill" << endl;
    //cout << "   -S : To build a Solid" << endl;
    di << "build sweep result [-M/-C/-R] [-S] [tol] : options are" << "\n";
    di << "   -M : Discontinuities are treated by Modfication of"<< "\n"; 
    di << "        the sweeping mode : it is the default" <<"\n";
    di << "   -C : Discontinuities are treated like Right Corner" << "\n";
    di << "        Treatement is Extent && Intersect" << "\n";
    di << "   -R : Discontinuities are treated like Round Corner" << "\n";
    di << "        Treatement is Intersect and Fill" << "\n";
    di << "   -S : To build a Solid" << "\n";
    return 0;
  }

  Standard_Boolean mksolid = Standard_False;
  if (Sweep ==0) {
    //cout << "You have forgotten the <<mksweep>> command  !"<< endl;
    di << "You have forgotten the <<mksweep>> command  !"<< "\n";
    return 1;
  }

  if (!Sweep->IsReady()) {
    //cout << "You have forgotten the <<addsweep>> command  !"<< endl;
    di << "You have forgotten the <<addsweep>> command  !"<< "\n";
    return 1;
  }

  TopoDS_Shape result;
  Standard_Integer cur=2;
  if (n>cur) {
    BRepBuilderAPI_TransitionMode Transition = BRepBuilderAPI_Transformed;

    // Reading Transition
    if (!strcmp(a[cur],"-C")) {
      Transition = BRepBuilderAPI_RightCorner;
      cur++;
    }
    else if (!strcmp(a[cur],"-R")) {
      Transition = BRepBuilderAPI_RoundCorner;
      cur++;
    }
    Sweep->SetTransitionMode(Transition);
  }
  // Reading solid ?
  if ((n>cur) && (!strcmp(a[cur],"-S")) ) mksolid = Standard_True;

  // Calcul le resultat
  Sweep->Build();
  if (!Sweep->IsDone()) {
    //cout << "Buildsweep : Not Done" << endl;
    di << "Buildsweep : Not Done" << "\n";
    BRepBuilderAPI_PipeError Stat = Sweep->GetStatus(); 
    if (Stat == BRepBuilderAPI_PlaneNotIntersectGuide) {
      //cout << "Buildsweep : One Plane not intersect the guide" << endl;
      di << "Buildsweep : One Plane not intersect the guide" << "\n";
    }
    if (Stat == BRepBuilderAPI_ImpossibleContact) {
      //cout << "BuildSweep : One section can not be in contact with the guide" << endl;
      di << "BuildSweep : One section can not be in contact with the guide" << "\n";
    }
    return 1;
  }
  else {
    if (mksolid) {
      Standard_Boolean B;
      B = Sweep->MakeSolid();
      //if (!B) cout << " BuildSweep : It is impossible to make a solid !" << endl;
      if (!B) di << " BuildSweep : It is impossible to make a solid !" << "\n";
    }
    result = Sweep->Shape();
    DBRep::Set(a[1],result);
  }

  return 0;
}

//=======================================================================
//  simulsweep
//=======================================================================
static Standard_Integer simulsweep(Draw_Interpretor& di,
				   Standard_Integer n, const char** a)
{
  if ( (n!=3) && (n!=4) ) return 1;
  
  if (Sweep ==0) {
    //cout << "You have forgotten the <<mksweep>> command  !"<< endl;
    di << "You have forgotten the <<mksweep>> command  !"<< "\n";
    return 1;
  }
  
  if (!Sweep->IsReady()) {
    //cout << "You have forgotten the <<addsweep>> command  !"<< endl;
    di << "You have forgotten the <<addsweep>> command  !"<< "\n";
    return 1;
  }
  
  char name[100];
  TopTools_ListOfShape List;
  TopTools_ListIteratorOfListOfShape it;
  Standard_Integer N, ii;
  N = Draw::Atoi(a[2]);

  if (n>3) {
    BRepBuilderAPI_TransitionMode Transition = BRepBuilderAPI_Transformed;
    // Lecture Transition
    if (!strcmp(a[3],"-C")) {
      Transition = BRepBuilderAPI_RightCorner;
    }
    else if (!strcmp(a[3],"-R")) {
      Transition = BRepBuilderAPI_RoundCorner;
    }
    Sweep->SetTransitionMode(Transition);
  }

  // Calculate the result
  Sweep->Simulate(N, List);
  for (ii=1, it.Initialize(List); it.More(); it.Next(), ii++) {
    Sprintf(name,"%s_%d",a[1],ii);
    DBRep::Set(name, it.Value());
  }

  return 0;
}

//=======================================================================
//  middlepath
//=======================================================================
static Standard_Integer middlepath(Draw_Interpretor& /*di*/,
				   Standard_Integer n, const char** a)
{
  if (n < 5) return 1;

  TopoDS_Shape aShape = DBRep::Get(a[2]);
  if (aShape.IsNull()) return 1;

  TopoDS_Shape StartShape = DBRep::Get(a[3]);
  if (StartShape.IsNull()) return 1;
  
  TopoDS_Shape EndShape   = DBRep::Get(a[4]);
  if (EndShape.IsNull()) return 1;

  BRepOffsetAPI_MiddlePath Builder(aShape, StartShape, EndShape);
  Builder.Build();

  TopoDS_Shape Result = Builder.Shape();
  DBRep::Set(a[1], Result);

  return 0;
}

//=======================================================================
//=======================================================================
//U-BEND COMMAND
//=======================================================================
//=======================================================================

#define maxSlices 50

gp_Pnt centralUbendPoint(adouble centralControlPointYCoordinate)
{
  TColgp_Array1OfPnt pathPoles(1, 4);
  pathPoles(1) = gp_Pnt(0.,   0.,   0.);
  pathPoles(2) = gp_Pnt(20.,  50.,  0.);
  //double derivativeSeed = 1.;
  //Standard_Real yPoleCoord = 200.;
  //yPoleCoord.setADValue(&derivativeSeed);
  pathPoles(3) = gp_Pnt(60.,  centralControlPointYCoordinate, 0.);
  pathPoles(4) = gp_Pnt(150., 0.,   0.);
  Handle(Geom_BezierCurve) path = new Geom_BezierCurve(pathPoles);

  TopoDS_Shape tube;
  gp_Pnt P[maxSlices];
  gp_Vec V[maxSlices];
  gp_Dir d[maxSlices];
  gp_Pln Pl[maxSlices];
  TopoDS_Edge Edge_C[maxSlices];
  TopoDS_Wire Wire[maxSlices];
  Standard_Real r[maxSlices];
  Handle(Geom2d_Circle) C[maxSlices];

  const Standard_Boolean isSolid = Standard_False;
  const Standard_Boolean isRuled = Standard_False;
  const Standard_Real    pres3d  = 1.0e-02;

  Standard_Real u[maxSlices];

  u[0]             = path->FirstParameter();
  u[maxSlices - 1] = path->LastParameter();

  const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);
  //
  for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
  {
    u[i] = u[0] + i*ustep;
  }

  BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
  aGenerator.SetSmoothing(Standard_True);
  aGenerator.SetMaxDegree(5);

  for ( Standard_Integer i = 0; i < maxSlices; ++i )
  {
    path->D1(u[i], P[i], V[i]);
    //V[i].Coord(Vx, Vy, Vz);
    //cout << "Vx: " << Vx << ", Vy: " << Vy << ", Vz: " << Vz << endl;
    d[i] = gp_Dir(V[i]);
    //d[i].Coord(Vx, Vy, Vz);
    //cout << "Dx: " << Vx << ", Dy: " << Vy << ", Dz: " << Vz << endl;

    // Some inline drawings
    /*{
      DRAW_LINE3D(P[i], gp_Pnt(P[i].XYZ() + d[i].XYZ()*5.0), normals, Draw_vert)
    }*/

    // Set circle center and radius
    r[i] = 25;
    //Center[i] = gp_Pnt2d(cx[i], cy[i]);

    // Create a circular section
    C[i] = GCE2d_MakeCircle(gp_Pnt2d(), r[i]);
    // -- Create the host plane
    Pl[i] = gp_Pln(P[i], d[i]);

    // Create Edges as 3d entities
    Edge_C[i] = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(C[i], Pl[i]) );

    // Check if there is a necessity to reverse the edge
    {
      if ( i )
      {
        Standard_Real f1, l1, f2, l2;
        Handle(Geom_Curve) c_current = BRep_Tool::Curve(Edge_C[i], f1, l1);//f1 and l1 are the first and the last parameter of the edge Edge_C1[i]
        Handle(Geom_Curve) c_prev    = BRep_Tool::Curve(Edge_C[i-1], f2, l2);

        gp_Vec c3d_leader      = c_current->Value(f1).XYZ();
        gp_Vec c3d_leader_prev = c_prev->Value(f2).XYZ();
        gp_Vec outer_vec       = c3d_leader - c3d_leader_prev;

        // Some inline drawings
        /*{
          DRAW_LINE3D(c_current->Value(f1), c_prev->Value(f2), outer, Draw_blanc)
        }*/

        // Check angle between the outer line and path direction
        const Standard_Real ang = outer_vec.Angle(V[i]);
        if ( Abs(ang) > M_PI/6 )
        {
          Pl[i] = gp_Pln( P[i], d[i].Reversed() );
          Handle(Geom_Curve) c3d = GeomAPI::To3d(C[i], Pl[i]);
          c3d->Reverse();

          // Create a circular section in a problem space
          Edge_C[i] = BRepBuilderAPI_MakeEdge(c3d);
        }
      }
    }

    // Dump to Draw
    //{
      //Standard_Real f, l;
      //Handle(Geom_Curve) c3d = BRep_Tool::Curve(Edge_C[i], f, l);
      //TCollection_AsciiString name_s("S"); name_s += (i + 1);
      //DrawTrSurf::Set( name_s.ToCString(), c3d );
    //}

    // Create wire
    BRepBuilderAPI_MakeWire mkWire;
    mkWire.Add(Edge_C[i]);
    Wire[i] = mkWire.Wire();

    // Dump to Draw
    //{
      //TCollection_AsciiString name("W"); name += (i + 1);
      //DBRep::Set( name.ToCString(), Wire[i] );
    //}

    // Add section to skinner
    aGenerator.AddWire(Wire[i]);
  }

  // Finally, build the surface
  try
  {
    aGenerator.Build();
  }
  catch ( ... )
  {
    std::cout << "Error: crash in BRepOffsetAPI_ThruSections" << std::endl;
    return gp_Pnt();
  }
  //
  if ( !aGenerator.IsDone() )
  {
    std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
    return gp_Pnt();
  }

  tube = aGenerator.Shape();
  TopoDS_Face face;
  for(TopExp_Explorer ex(tube,TopAbs_FACE); ex.More(); ex.Next())
  {
    face = TopoDS::Face(ex.Current());
  }

  Handle(Geom_Surface) resultingSurface = BRep_Tool::Surface(face);

  Handle(Geom_BSplineSurface) nurbs = Handle(Geom_BSplineSurface)::DownCast(resultingSurface);

  //DrawTrSurf::Set("nurbs", nurbs);

  Standard_Real U1, U2, V1, V2;
  nurbs->Bounds(U1, U2, V1, V2);

  gp_Pnt aPnt;
  Standard_Real Uc = (U2 - U1) / 2., Vc = (V2 - V1) / 2.;
  Uc = 1.; Vc = 1.;
  //cout << "Uc: " << Uc.getValue() << ", Vc: " << Vc.getValue() << endl;
  nurbs->D0(Uc, Vc, aPnt);

  return aPnt;
}

Standard_Integer ubendADtoFDcomparison(Draw_Interpretor& di, Standard_Integer n, const char** a)
{
  adouble yCoordinate = 200.;
  double seed = 1.;
  yCoordinate.setADValue(&seed);

  gp_Pnt aPnt = centralUbendPoint(yCoordinate);

  cout << "---------------AD-------------------" << endl;
  cout << "aPnt.X: " << aPnt.X() << endl;
  cout << "aPnt.Y: " << aPnt.Y() << endl;
  cout << "aPnt.Z: " << aPnt.Z() << endl << endl;

  adouble yCoordinateFD = 201.;
  gp_Pnt aPntNext = centralUbendPoint(yCoordinateFD);
  cout << "---------------FD-------------------" << endl;
  cout << "aPntNext.X: " << aPntNext.X().getValue() << endl;
  cout << "aPntNext.Y: " << aPntNext.Y().getValue() << endl;
  cout << "aPntNext.Z: " << aPntNext.Z().getValue() << endl << endl;

  yCoordinateFD = 199.;
  gp_Pnt aPntPrev = centralUbendPoint(yCoordinateFD);
  cout << "aPntPrev.X: " << aPntPrev.X().getValue() << endl;
  cout << "aPntPrev.Y: " << aPntPrev.Y().getValue() << endl;
  cout << "aPntPrev.Z: " << aPntPrev.Z().getValue() << endl << endl;

  cout << "---------------Central FD-------------------" << endl;
  cout << "finite X = " << (aPntNext.X() - aPntPrev.X()).getValue() / 2. << endl;
  cout << "finite Y = " << (aPntNext.Y() - aPntPrev.Y()).getValue() / 2. << endl;
  cout << "finite Z = " << (aPntNext.Z() - aPntPrev.Z()).getValue() / 2. << endl;

  return 0;
}


Standard_Integer ubend(Draw_Interpretor& di, Standard_Integer n, const char** a)
{
  //---------------------------------------------------------------------
  // Stage 1: Start with a Bezier path for U-bend
  //---------------------------------------------------------------------
  TColgp_Array1OfPnt pathPoles(1, 4);
  pathPoles(1) = gp_Pnt(0.,   0.,   0.);
  pathPoles(2) = gp_Pnt(20.,  50.,  0.);
  double derivativeSeed = 1.;
  Standard_Real yPoleCoord = 200.;
  yPoleCoord.setADValue(&derivativeSeed);
  pathPoles(3) = gp_Pnt(60.,  yPoleCoord, 0.);
  pathPoles(4) = gp_Pnt(150., 0.,   0.);
  Handle(Geom_BezierCurve) path = new Geom_BezierCurve(pathPoles);

  DrawTrSurf::Set("path", path);

  TopoDS_Shape tube;
  gp_Pnt P[maxSlices];
  gp_Vec V[maxSlices];
  gp_Dir d[maxSlices];
  gp_Pln Pl[maxSlices];
//  Handle(Geom_Plane) s[maxSlices];
  TopoDS_Edge Edge_C[maxSlices];
  TopoDS_Wire Wire[maxSlices];

//  Standard_Real cx[maxSlices];
//  Standard_Real cy[maxSlices];
  Standard_Real r[maxSlices];
//  gp_Pnt2d Center[maxSlices];
  Handle(Geom2d_Circle) C[maxSlices];

  const Standard_Boolean isSolid = Standard_False;
  const Standard_Boolean isRuled = Standard_False;
  const Standard_Real    pres3d  = 1.0e-02;

  Standard_Real u[maxSlices];
  //
  u[0]             = path->FirstParameter();
  u[maxSlices - 1] = path->LastParameter();

  //cout << "path-u[0]: " << u[0] << endl;
  //cout << "path-v[49]: " << u[maxSlices - 1] << endl;
  //
  const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);
  //
  for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
  {
    u[i] = u[0] + i*ustep;
  }

  // Prepare skinning tool
  BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
  aGenerator.SetSmoothing(Standard_True);
  aGenerator.SetMaxDegree(5);
  Standard_Real Vx, Vy, Vz;
  for ( Standard_Integer i = 0; i < maxSlices; ++i )
  {
    path->D1(u[i], P[i], V[i]);
    V[i].Coord(Vx, Vy, Vz);
    //cout << "Vx: " << Vx << ", Vy: " << Vy << ", Vz: " << Vz << endl;
    d[i] = gp_Dir(V[i]);
    d[i].Coord(Vx, Vy, Vz);
    //cout << "Dx: " << Vx << ", Dy: " << Vy << ", Dz: " << Vz << endl;

    // Some inline drawings
    /*{
      DRAW_LINE3D(P[i], gp_Pnt(P[i].XYZ() + d[i].XYZ()*5.0), normals, Draw_vert)
    }*/

    // Set circle center and radius
    r[i] = 25;
    //Center[i] = gp_Pnt2d(cx[i], cy[i]);

    // Create a circular section
    C[i] = GCE2d_MakeCircle(gp_Pnt2d(), r[i]);
    // -- Create the host plane
    Pl[i] = gp_Pln(P[i], d[i]);

    // Create Edges as 3d entities
    Edge_C[i] = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(C[i], Pl[i]) );

    // Check if there is a necessity to reverse the edge
    {
      if ( i )
      {
        Standard_Real f1, l1, f2, l2;
        Handle(Geom_Curve) c_current = BRep_Tool::Curve(Edge_C[i], f1, l1);//f1 and l1 are the first and the last parameter of the edge Edge_C1[i]
        Handle(Geom_Curve) c_prev    = BRep_Tool::Curve(Edge_C[i-1], f2, l2);

        gp_Vec c3d_leader      = c_current->Value(f1).XYZ();
        gp_Vec c3d_leader_prev = c_prev->Value(f2).XYZ();
        gp_Vec outer_vec       = c3d_leader - c3d_leader_prev;

        // Some inline drawings
        /*{
          DRAW_LINE3D(c_current->Value(f1), c_prev->Value(f2), outer, Draw_blanc)
        }*/

        // Check angle between the outer line and path direction
        const Standard_Real ang = outer_vec.Angle(V[i]);
        if ( Abs(ang) > M_PI/6 )
        {
          Pl[i] = gp_Pln( P[i], d[i].Reversed() );
          Handle(Geom_Curve) c3d = GeomAPI::To3d(C[i], Pl[i]);
          c3d->Reverse();

          // Create a circular section in a problem space
          Edge_C[i] = BRepBuilderAPI_MakeEdge(c3d);
        }
      }
    }

    // Dump to Draw
    {
      Standard_Real f, l;
      Handle(Geom_Curve) c3d = BRep_Tool::Curve(Edge_C[i], f, l);
      TCollection_AsciiString name_s("S"); name_s += (i + 1);
      DrawTrSurf::Set( name_s.ToCString(), c3d );
    }

    // Create wire
    BRepBuilderAPI_MakeWire mkWire;
    mkWire.Add(Edge_C[i]);
    Wire[i] = mkWire.Wire();

    // Dump to Draw
    {
      TCollection_AsciiString name("W"); name += (i + 1);
      DBRep::Set( name.ToCString(), Wire[i] );
    }

    // Add section to skinner
    aGenerator.AddWire(Wire[i]);
  }

  // Finally, build the surface
  try
  {
    aGenerator.Build();
  }
  catch ( ... )
  {
    std::cout << "Error: crash in BRepOffsetAPI_ThruSections" << std::endl;
    return 1;
  }
  //
  if ( !aGenerator.IsDone() )
  {
    std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
    return 1;
  }

  tube = aGenerator.Shape();
  cout << "Shape type: " << tube.ShapeType() << endl;
  int sumOfFaces = 0;
  TopoDS_Face face;
  for(TopExp_Explorer ex(tube,TopAbs_FACE); ex.More(); ex.Next())
  {
    sumOfFaces++;
    face = TopoDS::Face(ex.Current());
  }
  cout << "Sum of faces: " << sumOfFaces << endl;

  Handle(Geom_Surface) resultingSurface = BRep_Tool::Surface(face);

  Handle(Geom_BSplineSurface) nurbs = Handle(Geom_BSplineSurface)::DownCast(resultingSurface);

  DrawTrSurf::Set("nurbs", nurbs);

  DBRep::Set("tube",tube);

  di.Eval("donly nurbs");

  Standard_Real U1, U2, V1, V2;
  nurbs->Bounds(U1, U2, V1, V2);
  //cout << "U1: " << U1.getValue() << ", U2: " << U2.getValue() << ", V1: " << V1.getValue() << ", V2: " << V2.getValue() << endl;

  gp_Pnt aPnt;
  Standard_Real Uc = (U2 - U1) / 2., Vc = (V2 - V1) / 2.;
  cout << "Uc: " << Uc.getValue() << ", Vc: " << Vc.getValue() << endl;
  nurbs->D0(Uc, Vc, aPnt);

  cout << "aPnt.X: " << aPnt.X() << endl;
  cout << "aPnt.Y: " << aPnt.Y() << endl;
  cout << "aPnt.Z: " << aPnt.Z() << endl;

  return 0;
}

//=======================================================================
//END U-BEND COMMAND
//=======================================================================
//=======================================================================

//=======================================================================
//U-BEND OPTIMIZATION SAMPLE
//=======================================================================

//! Type alias for the objective function to optimize.
typedef Standard_Real (*OptFunction)(const int pole_idx, const int coord_idx, const Standard_Real& coord_val, const bool setToDraw);

class Opt_ArmijoRule
{
public:

  //! Search parameters.
  struct t_search_params
  {
    double          max_alpha;      //!< Right bound.
    int             num_iterations; //!< Max number of iterations.
    int             pole_idx;       //!< Index of pole.
    int             coord_idx;      //!< Pole coordinate index (1=>x, 2=>y, 3=>z)
    Standard_Real   x_k;            //!< Position in the search space.
    Standard_Real   d_k;            //!< Direction of line search.
    Standard_Real   gradient;       //!< Calculated gradient.
    Standard_Real   f;
    OptFunction     pFunc;          //!< Target function.

    t_search_params() : max_alpha      (0.0),
                        num_iterations (0),
                        pole_idx       (0),
                        coord_idx      (0),
                        pFunc          (NULL)
    {}
  };

public:

  Opt_ArmijoRule() {}

public:

  //! Runs Armijo rule.
  //! \param params    [in]  search parameters.
  //! \param num_iters [out] consumed number of iterations.
  //! \param alpha     [out] resulting step size.
  //! \return true in case of success, false -- otherwise.
  bool Perform(const t_search_params& params,
               int&                   num_iters,
               double&                alpha)
  {
    num_iters = 0;
    alpha     = params.max_alpha;

    const double beta      = 0.2; // Interval reduction coefficient
    const double mu        = 1.0e-2;
    const double min_alpha = 1.0e-8;
    const double gradient  = params.gradient.getValue();
    //const double f         = (*params.pFunc)(params.pole_idx, params.coord_idx, params.x_k, false).getValue();
    const double phi_deriv = gradient;

    // Initial alpha
    bool isSolved = false;
    bool doStop   = false;

    // Main iterations
    do
    {
      ++num_iters;
      if ( num_iters < params.num_iterations )
      {
        double barrier = alpha*mu*phi_deriv;
        adouble new_x  = params.x_k + params.d_k*alpha;
        if ( new_x <= 0 )
        {
          alpha *= beta;

          if ( alpha < min_alpha )
          {
            doStop   = true;
            isSolved = false;
          }
          continue;
        }

        double phi = ((*params.pFunc)(params.pole_idx, params.coord_idx, new_x, false) - params.f).getValue();

        if ( phi < barrier )
        {
          doStop   = true;
          isSolved = true;
        }
        else
        {
          alpha *= beta;

          if ( alpha < min_alpha )
          {
            doStop   = true;
            isSolved = false;
          }
        }
      }
      else
      {
        doStop   = true;
        isSolved = false;
      }
    }
    while ( !doStop );

    return isSolved;
  }

};

namespace UBEND_AD
{
  Handle(Geom_BezierCurve) targetPath, customPath;

  Standard_Real uParams[maxSlices], vParams[maxSlices];
  Handle(Geom_BSplineSurface) targetSurface;

  Handle(Geom_BSplineSurface) constructUbendSurface(Handle(Geom_BezierCurve)& pathLine)
  {
    TopoDS_Shape tube;
    gp_Pnt P[maxSlices];
    gp_Vec V[maxSlices];
    gp_Dir d[maxSlices];
    gp_Pln Pl[maxSlices];

    TopoDS_Edge Edge_C[maxSlices];
    TopoDS_Wire Wire[maxSlices];

    const Standard_Real r = 25.;

    // Create a circular section
    Handle(Geom2d_Circle) C = GCE2d_MakeCircle(gp_Pnt2d(), r);

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;
    const Standard_Real    pres3d  = 1.0e-02;

    Standard_Real u[maxSlices];

    u[0]             = pathLine->FirstParameter();
    u[maxSlices - 1] = pathLine->LastParameter();

    const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);

    for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
    {
      u[i] = u[0] + i*ustep;
    }

    // Prepare skinning tool
    BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);

    for ( Standard_Integer i = 0; i < maxSlices; ++i )
    {
      pathLine->D1(u[i], P[i], V[i]);
      d[i] = gp_Dir(V[i]);

      // Create the host plane
      Pl[i] = gp_Pln(P[i], d[i]);

      // Create Edges as 3d entities
      Edge_C[i] = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(C, Pl[i]) );

      // Check if there is a necessity to reverse the edge
      if(i)
      {
        Standard_Real f1, l1, f2, l2;
        Handle(Geom_Curve) c_current = BRep_Tool::Curve(Edge_C[i], f1, l1);//f1 and l1 are the first and the last parameter of the edge Edge_C1[i]
        Handle(Geom_Curve) c_prev    = BRep_Tool::Curve(Edge_C[i-1], f2, l2);

        gp_Vec c3d_leader      = c_current->Value(f1).XYZ();
        gp_Vec c3d_leader_prev = c_prev->Value(f2).XYZ();
        gp_Vec outer_vec       = c3d_leader - c3d_leader_prev;

        const Standard_Real ang = outer_vec.Angle(V[i]);
        if ( Abs(ang) > M_PI/6 )
        {
          Pl[i] = gp_Pln( P[i], d[i].Reversed() );
          Handle(Geom_Curve) c3d = GeomAPI::To3d(C, Pl[i]);
          c3d->Reverse();

          // Create a circular section in a problem space
          Edge_C[i] = BRepBuilderAPI_MakeEdge(c3d);
        }
      }

      // Dump to Draw
//      {
//        Standard_Real f, l;
//        Handle(Geom_Curve) c3d = BRep_Tool::Curve(Edge_C[i], f, l);
//        TCollection_AsciiString name_s("S"); name_s += (i + 1);
//        DrawTrSurf::Set( name_s.ToCString(), c3d );
//      }

      // Create wire
      BRepBuilderAPI_MakeWire mkWire;
      mkWire.Add(Edge_C[i]);
      Wire[i] = mkWire.Wire();

      // Dump to Draw
//      {
//        TCollection_AsciiString name("W"); name += (i + 1);
//        DBRep::Set( name.ToCString(), Wire[i] );
//      }

      // Add section to skinner
      aGenerator.AddWire(Wire[i]);
    }

    // Finally, build the surface
    try
    {
      aGenerator.Build();
    }
    catch ( ... )
    {
      cout << "Error: crash in BRepOffsetAPI_ThruSections" << endl;
      return NULL;
    }

    if ( !aGenerator.IsDone() )
    {
      cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << endl;
      return NULL;
    }

    tube = aGenerator.Shape();
    cout << "Shape type: " << tube.ShapeType() << endl;

    //shell should contain only one face
    TopExp_Explorer ex(tube,TopAbs_FACE);
    TopoDS_Face face;
    if(ex.More())
    {
      face = TopoDS::Face(ex.Current());
    }

    Handle(Geom_Surface) resultingSurface = BRep_Tool::Surface(face);

    Handle(Geom_BSplineSurface) nurbs = Handle(Geom_BSplineSurface)::DownCast(resultingSurface);

    //DrawTrSurf::Set("nurbs", nurbs);
    //DBRep::Set("tube",tube);

    return nurbs;
  }

  void initTargetSurfaceAndParams()
  {
    TColgp_Array1OfPnt pathPoles(1, 4);
    pathPoles(1) = gp_Pnt(0.,   0.,   0.);
    pathPoles(2) = gp_Pnt(20.,  50.,  0.);
    pathPoles(3) = gp_Pnt(60.,  200., 0.);
    pathPoles(4) = gp_Pnt(150., 0.,   0.);

    targetPath = new Geom_BezierCurve(pathPoles);

    targetSurface = constructUbendSurface(targetPath);

    Standard_Real U1, U2, V1, V2;
    targetSurface->Bounds(U1, U2, V1, V2);

    cout << "U1: " << U1.getValue() << ", U2: " << U2.getValue() << ", V1: " << V1.getValue() << ", V2: " << V2.getValue() << endl;

    uParams[0] = U1;
    uParams[maxSlices - 1] = U2;
    vParams[0] = V1;
    vParams[maxSlices - 1] = V2;

    const Standard_Real uStep = (U2 - U1) / (maxSlices - 1);
    for(Standard_Integer i = 1; i < maxSlices - 1; ++i)
    {
      uParams[i] = uParams[0] + i*uStep;
    }

    const Standard_Real vStep = (V2 - V1) / (maxSlices - 1);
    for(Standard_Integer i = 1; i < maxSlices - 1; ++i)
    {
      vParams[i] = vParams[0] + i*vStep;
    }
  }

  Standard_Real Dist(const int pole_idx, const int coord_idx, const Standard_Real& coord_val, bool setToDraw = false)
  {
    //update coordinate
    gp_Pnt pole = customPath->Pole(pole_idx);
    pole.SetCoord(coord_idx, coord_val);
    customPath->SetPole(pole_idx, pole);

    //construct the u-bend and extract resulting surface
    Handle(Geom_BSplineSurface) customSurface = constructUbendSurface(customPath);
    if(setToDraw)
    {
      DrawTrSurf::Set("nurbsCustom", customSurface);
    }

    // Calculate total squared distance
    Standard_Real squares = 0.;
    for(Standard_Integer i = 0; i < maxSlices; i++)
    {
      for(Standard_Integer j = 0; j < maxSlices; j++)
      {
        gp_Pnt targetPnt;
        targetSurface->D0(uParams[i], vParams[j], targetPnt);

        gp_Pnt customPnt;
        customSurface->D0(uParams[i], vParams[j], customPnt);

        Standard_Real d = (targetPnt.X() - customPnt.X()) * (targetPnt.X() - customPnt.X()) +
            (targetPnt.Y() - customPnt.Y()) * (targetPnt.Y() - customPnt.Y()) +
            (targetPnt.Z() - customPnt.Z()) * (targetPnt.Z() - customPnt.Z());

        squares += d;
      }
    }

    return squares;
  }
}

Standard_Integer ubendAD(Draw_Interpretor& di, Standard_Integer n, const char** a)
{
  UBEND_AD::initTargetSurfaceAndParams();
  DrawTrSurf::Set("nurbs", UBEND_AD::targetSurface);

  TColgp_Array1OfPnt customPathPoles(1, 4);
  customPathPoles(1) = gp_Pnt(0.,   0.,   0.);
  customPathPoles(2) = gp_Pnt(20.,  50.,  0.);
  customPathPoles(3) = gp_Pnt(60.,  250., 0.);
  customPathPoles(4) = gp_Pnt(150., 0.,   0.);

  UBEND_AD::customPath = new Geom_BezierCurve(customPathPoles);

  Handle(Geom_BSplineSurface) customSurface = UBEND_AD::constructUbendSurface(UBEND_AD::customPath);

  //DrawTrSurf::Set("nurbsCustom", customSurface);
  di.Eval("axo");
  //di.Eval("fit");

  const double def_step = 1.;
  const double prec = 1.0e-2;

  // Prepare Armijo rule
  Opt_ArmijoRule Armijo;
  Opt_ArmijoRule::t_search_params Armijo_params;
  Armijo_params.max_alpha      = 1.0;
  Armijo_params.num_iterations = 100;
  Armijo_params.pFunc          = UBEND_AD::Dist;

  const int maxNumberOfIterations = 5000;
  int numberOfIterations = 0;

  Standard_Real y;
  Standard_Real independetVar;

  //for optimization, take y coordinate of 3rd pole
  Armijo_params.pole_idx = 3;
  Armijo_params.coord_idx = 2;

  independetVar = customPathPoles.Value(3).Coord(2);
  cout << "independent var: " << independetVar << endl;

  const double derivativeSeed = 1.;
  independetVar.setADValue(&derivativeSeed);

  const double *grad_y;
  while(1)
  {
    di.Eval("clear");
    di.Eval("display nurbs");
    di.Eval("unset nurbsCustom");
    y = UBEND_AD::Dist(Armijo_params.pole_idx, Armijo_params.coord_idx, independetVar, true);
    di.Eval("fit");
    grad_y = y.getADValue();

    if(fabs(*grad_y) < prec)
      break;

    // Configure Armijo rule
    Armijo_params.x_k = independetVar;
    Armijo_params.d_k = -(*grad_y);
    Armijo_params.gradient = *grad_y;
    Armijo_params.f = y;

    int num_armijo_iters = 0;

    // Choose step adaptively
    double actual_step;
    if ( !Armijo.Perform(Armijo_params, num_armijo_iters, actual_step) )
      actual_step = def_step;

    // Steepest descent
    independetVar -= actual_step * (*grad_y);

    // Check to avoid infinite loops
    numberOfIterations++;
    if ( numberOfIterations > maxNumberOfIterations )
    {
      cout << "Error: max number of iterations exceeded" << endl;
      break;
    }

    cout << "y coordinate value: " << independetVar << endl;
    //di.Eval("display nurbsCustom");
  }

  cout << "Total number of iterations: " << numberOfIterations << endl;
  cout << "Final y coordinate value: " << independetVar << endl;

  return 0;
}



//=======================================================================
//function : SweepCommands
//purpose  : 
//=======================================================================

void  BRepTest::SweepCommands(Draw_Interpretor& theCommands)
{
  static Standard_Boolean done = Standard_False;
  if (done) return;
  done = Standard_True;

  DBRep::BasicCommands(theCommands);

  const char* g = "Sweep commands";
  
  theCommands.Add("prism",
		  "prism result base dx dy dz [Copy | Inf | Seminf]",
		  __FILE__,prism,g);
  
  theCommands.Add("revol",
		  "revol result base px py pz dx dy dz angle [Copy]",
		  __FILE__,revol,g);
  
  theCommands.Add("pipe",
		  "pipe result Wire_spine Profile [Mode [Approx]], no args to get help",
		  __FILE__,pipe,g);
  
  theCommands.Add("evolved",
		  "evolved , no args to get help",
		  __FILE__,evolved,g);  

  theCommands.Add("evolvedsolid",
		  "evolved , no args to get help",
		  __FILE__,evolved,g);  
  
  theCommands.Add("pruled",
		  "pruled result Edge1/Wire1 Edge2/Wire2",
		  __FILE__,pruled,g);

  theCommands.Add("gener", "gener result wire1 wire2 [..wire..]",
		  __FILE__,gener,g);

  theCommands.Add("thrusections", "thrusections [-N] result issolid isruled shape1 shape2 [..shape..], the option -N means no check on wires, shapes must be wires or vertices (only first or last)",
		  __FILE__,thrusections,g);

  
  theCommands.Add("mksweep", "mksweep wire",
		  __FILE__,mksweep,g);

  theCommands.Add("setsweep", "setsweep  no args to get help",
		  __FILE__,setsweep,g);
  
  theCommands.Add("addsweep", 
		  "addsweep wire [vertex] [-M ] [-C] [auxiilaryshape]:no args to get help",
		  __FILE__,addsweep,g);

 theCommands.Add("deletesweep", 
		  "deletesweep wire, To delete a section",
		  __FILE__,deletesweep,g);

  theCommands.Add("buildsweep", "builsweep [r] [option] [Tol] , no args to get help"
		  __FILE__,buildsweep,g);

  theCommands.Add("simulsweep", "simulsweep r [n] [option]"
		  __FILE__,simulsweep,g);
  theCommands.Add("geompipe", "geompipe r spineedge profileedge radius [byACR [byrotate]]"
		  __FILE__,geompipe,g);
  
  theCommands.Add("middlepath", "middlepath res shape startshape endshape",
		  __FILE__,middlepath,g);

  theCommands.Add("ubend", "ubend",
        __FILE__,ubend,g);

  theCommands.Add("ubend_adfd", "ubend_adfd",
          __FILE__,ubendADtoFDcomparison,g);

  theCommands.Add("ubend_ad", "ubend_ad",
          __FILE__,ubendAD,g);
}

