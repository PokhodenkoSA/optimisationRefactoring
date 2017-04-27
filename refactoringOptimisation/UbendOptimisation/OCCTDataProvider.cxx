#include "OCCTDataProvider.h"
#include <BRepPrimAPI_MakePrism.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Convert_ParameterisationType.hxx>
#include <GeomConvert.hxx>
#include <DBRep.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Line.hxx>
#include <gp_Lin2d.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>


TopoDS_Shape OCCTDataProvider::UbendCorked(vector<Standard_Real> a){
    //---------------------------------------------------------------------------
    // Stage 1: Start with a Bezier path for U-bend
    //---------------------------------------------------------------------------
    int maxSlices = 50;

    TColgp_Array1OfPnt pathPoles(1, 4);
    pathPoles(1) = gp_Pnt(0.,   0.,   0.);
    pathPoles(2) = gp_Pnt(100.,  3.,  0.);
    pathPoles(3) = gp_Pnt(105.,  3., 0.);
    pathPoles(4) = gp_Pnt(150., 0.,   0.);
    Handle(Geom_BezierCurve) path = new Geom_BezierCurve(pathPoles);
    //


    //---------------------------------------------------------------------------
    // Stage 2: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    // Create Points

    // Create Directions

    TopoDS_Shape tube;
    gp_Pnt P[maxSlices];
    gp_Vec V[maxSlices];
    gp_Dir d[maxSlices];
    gp_Pln Pl[maxSlices];
    Handle(Geom_Plane) s[maxSlices];
    TopoDS_Edge Edge_C[maxSlices];
    TopoDS_Wire Wire[maxSlices];

    Standard_Real cx[maxSlices];
    Standard_Real cy[maxSlices];
    Standard_Real r[maxSlices];
    gp_Pnt2d Center[maxSlices];
    Handle(Geom2d_Circle) C[maxSlices];

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;
    const Standard_Real    pres3d  = 1.0e-02;

    Standard_Real u[maxSlices];
    //
    u[0]             = path->FirstParameter();
    u[maxSlices - 1] = path->LastParameter();
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
    //
    for ( Standard_Integer i = 0; i < maxSlices; ++i )
    {
        path->D1(u[i], P[i], V[i]);
        d[i] = gp_Dir(V[i]);

        // Set circle center and radius
        r[i]=5;
        if (i==30){
            r[i] = a[0];
        }
        if (i==31){
            r[i]=a[1];
        }

        Center[i] = gp_Pnt2d(cx[i], cy[i]);

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

        // Create wire
        BRepBuilderAPI_MakeWire mkWire;
        mkWire.Add(Edge_C[i]);
        Wire[i] = mkWire.Wire();

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
        //return 1;
    }
    //
    if ( !aGenerator.IsDone() )
    {
        std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
        // return 1;
    }


    tube = aGenerator.Shape();

    // Create boundary faces
    TopoDS_Wire C0 = Wire[0];
    TopoDS_Wire Cmaxslices = Wire[maxSlices-1];
    TopoDS_Face faceC0 = BRepBuilderAPI_MakeFace(Pl[0], C0).Face();
    Pl[maxSlices-1] = gp_Pln( P[maxSlices-1], d[maxSlices-1].Reversed() );
    TopoDS_Face faceCmaxslices = BRepBuilderAPI_MakeFace(Pl[maxSlices-1], Cmaxslices).Face();

    tube = BRepAlgoAPI_Fuse(faceC0, tube);
    tube=BRepAlgoAPI_Fuse(tube, faceCmaxslices);

    return tube;

}

TopoDS_Shape OCCTDataProvider::UbendSquared(vector<double> a) {
    //---------------------------------------------------------------------------
    // Stage 1: Start with a Bezier path for U-bend
    //---------------------------------------------------------------------------

    TColgp_Array1OfPnt pathPoles(1, 4);
    pathPoles(1) = gp_Pnt(0., 0., 0.);
    pathPoles(2) = gp_Pnt(20., 50., 0.);
    pathPoles(3) = gp_Pnt(60., 100., 0.);
    pathPoles(4) = gp_Pnt(150., 0., 0.);
    Handle(Geom_BezierCurve) path = new Geom_BezierCurve(pathPoles);

    Standard_Real Length;
    GeomAdaptor_Curve GAC;
    //
    GAC.Load(path);
    Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve
    cout << "Path length is " << Length << endl;

    //---------------------------------------------------------------------------
    // Stage 2: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    // Create Points

    // Create Directions
    int maxSlices=30;

    TopoDS_Shape tube;
    //Handle(Geom_BSplineSurface) nurbs;
    gp_Pnt P[maxSlices];
    gp_Vec V[maxSlices];
    gp_Dir d[maxSlices];
    gp_Vec V2[maxSlices];
    gp_Dir d2[maxSlices];
    gp_Pln Pl1[maxSlices];
    gp_Pln Pl[maxSlices];
    //Geom_Plane Pl[maxSlices];
    Handle(Geom_Plane) s[maxSlices];
    TopoDS_Edge Edge_B11[maxSlices];
    TopoDS_Edge Edge_B21[maxSlices];
    TopoDS_Edge Edge_B31[maxSlices];
    TopoDS_Edge Edge_B41[maxSlices];
    TopoDS_Wire Wire[maxSlices];
    Standard_Real P_Length[maxSlices];
    Standard_Real CP1_Length[maxSlices];
    Standard_Real Length_step[maxSlices];
    Standard_Real CP1_Length_step[maxSlices];
    gp_Pln constructionPl[maxSlices];
    Handle(Geom_Plane) constructionPlane[maxSlices];
    gp_Pnt P_path[maxSlices];
    gp_Pnt InterPoint[maxSlices];

    TColStd_HArray1OfReal DesignParameters(1, 96);

/*  (IP11x_, IP12x_, IP13x, IP14x,
    IP11y_, IP12y_, IP13y, IP14y,

    IP21x, IP22x, IP23x, IP24x,
    IP21y, IP22y, IP23y, IP24y,

    IP31x, IP32x, IP33x, IP34x,
    IP31y, IP32y, IP33y, IP34y,

    IP41x, IP42x, IP43x, IP44x,
    IP41y, IP42y, IP43y, IP44y,

    IP51x, IP52x, IP53x, IP54x,
    IP51y, IP52y, IP53y, IP54y,

    IP61x, IP62x, IP63x, IP64x,
    IP61y, IP62y, IP63y, IP64y,

    IP71x, IP72x, IP73x, IP74x,
    IP71y, IP72y, IP73y, IP74y,

    IP81x, IP82x, IP83x, IP84x,
    IP81y, IP82y, IP83y, IP84y,

    IP91x, IP92x, IP93x, IP94x,
    IP91y, IP92y, IP93y, IP94y,

    IP101x, IP102x, IP103x, IP104x,
    IP101y, IP102y, IP103y, IP104y,

    IP111x, IP112x, IP113x, IP114x,
    IP111y, IP112y, IP113y, IP114y,

    IP121x, IP122x, IP123x, IP124x
    IP121y, IP122y, IP123y, IP124y)*/

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;
    const Standard_Real pres3d = 1.0e-02;

    Standard_Real u[maxSlices];
    //
    u[0] = path->FirstParameter();
    u[maxSlices - 1] = path->LastParameter();
    //
    const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);
    //
    for (Standard_Integer i = 1; i < maxSlices - 1; ++i) {
        u[i] = u[0] + i * ustep;
    }

    // Prepare skinning tool
    BRepOffsetAPI_ThruSections aGenerator(isSolid, isRuled, pres3d);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);
    //
    for (Standard_Integer i = 0; i < maxSlices; ++i) {

        path->D2(u[i], P[i], V[i], V2[i]);
        d[i] = gp_Dir(V[i]);
        d2[i] = gp_Dir(V2[i]);

        //---------------------------------------------------------------------------
        // build law.
        //---------------------------------------------------------------------------

        //Number Control Points

        //IP1 = Intersection Point
        //Standard_Real IP11x_ = -5, IP12x_ = -5, IP13x = -10, IP14x = -10;
        //Standard_Real IP11y_ = 5, IP12y_ = 5, IP13y = 7.5, IP14y = 7.5;
        Standard_Real IP11x_ = -5, IP12x_ = -5, IP13x = -5, IP14x = -5;
        Standard_Real IP11y_ = 5, IP12y_ = 5, IP13y = 5, IP14y = 5;
        //IP2 = Intersection Point
        Standard_Real IP21x = -2.5, IP22x = -2.5, IP23x = -2.5, IP24x = -2.5;
        Standard_Real IP21y = 5, IP22y = 5, IP23y = 5, IP24y = 5;
//IP3 = Intersection Point
        Standard_Real IP31x = 2.5, IP32x = 2.5, IP33x = 2.5, IP34x = 2.5;
        Standard_Real IP31y = 5, IP32y = 5, IP33y = 5, IP34y = 5;
//IP4 = Intersection Point
        //Standard_Real IP41x = 5, IP42x = 5, IP43x = 7.5, IP44x = 7.5;
        //Standard_Real IP41y = 5, IP42y = 5, IP43y = 10, IP44y = 10;
        Standard_Real IP41x = 5, IP42x = 5, IP43x = 5, IP44x = 5;
        Standard_Real IP41y = 5, IP42y = 5, IP43y = 5, IP44y = 5;
//IP5 = Intersection Point
        //Standard_Real IP51x = 5, IP52x = 5, IP53x = 0, IP54x = 2.5;
        //Standard_Real IP51y = 2.5, IP52y = 2.5, IP53y = 2.5, IP54y = 2.5;
        Standard_Real IP51x = 5, IP52x = 5, IP53x = 5, IP54x = 5;
        Standard_Real IP51y = 2.5, IP52y = 2.5, IP53y = 2.5, IP54y = 2.5;
//IP6 = Intersection Point
        //Standard_Real IP61x = 5, IP62x = 5, IP63x = 0, IP64x = 2.5;
        //Standard_Real IP61y = -2.5, IP62y = -2.5, IP63y = -2.5, IP64y = -2.5;
        Standard_Real IP61x = 5, IP62x = 5, IP63x = 5, IP64x = 5;
        Standard_Real IP61y = -2.5, IP62y = -2.5, IP63y = -2.5, IP64y = -2.5;
//IP7 = Intersection Point
        Standard_Real IP71x = 5, IP72x = 5, IP73x = 5, IP74x = 5;
        Standard_Real IP71y = -5, IP72y = -5, IP73y = -5, IP74y = -5;
//IP8 = Intersection Point
        Standard_Real IP81x = 2.5, IP82x = 2.5, IP83x = 2.5, IP84x = 2.5;
        Standard_Real IP81y = -5, IP82y = -5, IP83y = -5, IP84y = -5;
//IP9 = Intersection Point
        Standard_Real IP91x = -2.5, IP92x = -2.5, IP93x = -2.5, IP94x = -2.5;
        Standard_Real IP91y = -5, IP92y = -5, IP93y = -5, IP94y = -5;
//IP10 = Intersection Point
        Standard_Real IP101x = -5, IP102x = -5, IP103x = -5, IP104x = -5;
        Standard_Real IP101y = -5, IP102y = -5, IP103y = -5, IP104y = -5;
//IP11 = Intersection Point
        //Standard_Real IP111x = -5, IP112x = -5, IP113x = -2.5, IP114x = -2.5;
        //Standard_Real IP111y = -2.5, IP112y = -2.5, IP113y = -2.5, IP114y = -2.5;
        Standard_Real IP111x = -5, IP112x = -5, IP113x = -5, IP114x = -5;
        Standard_Real IP111y = -2.5, IP112y = -2.5, IP113y = -2.5, IP114y = -2.5;
//IP12 = Intersection Point
        //Standard_Real IP121x = -5, IP122x = -5, IP123x = -2.5, IP124x = -2.5;
        //Standard_Real IP121y = 2.5, IP122y = 2.5, IP123y = 2.5, IP124y = 2.5;
        Standard_Real IP121x = -5, IP122x = -5, IP123x = -5, IP124x = -5;
        Standard_Real IP121y = 2.5, IP122y = 2.5, IP123y = 2.5, IP124y = 2.5;

        //CP1
        TColgp_Array1OfPnt CPlaw1(1, 4);
        gp_Pnt CP1_P1(IP11x_, IP11y_, 0), CP1_P2(IP12x_, IP12y_, 0.3), CP1_P3(IP13x, IP13y, 0.6), CP1_P4(IP14x,
                                                                                                         IP14y, 1);
        CPlaw1(1) = CP1_P1;
        CPlaw1(2) = CP1_P2;
        CPlaw1(3) = CP1_P3;
        CPlaw1(4) = CP1_P4;
        Handle(Geom_BezierCurve) BezierCP1 = new Geom_BezierCurve(CPlaw1);

        /*Standard_Real LengthLaw1;
        GeomAdaptor_Curve GAC_Law1;
        //
        GAC_Law1.Load(BezierCP1);
        LengthLaw1 = GCPnts_AbscissaPoint::Length(GAC_Law1);//length of the curve
        CP1_Length[i] = GCPnts_AbscissaPoint::Length(GAC_Law1, u[0], u[i]);
        CP1_Length_step[i] = CP1_Length[i]/LengthLaw1;*/


        //CP2
        TColgp_Array1OfPnt CPlaw2(1, 4);
        gp_Pnt CP2_P1(IP21x, IP21y, 0), CP2_P2(IP22x, IP22y, 0.3), CP2_P3(IP23x, IP23y, 0.6), CP2_P4(IP24x, IP24y,
                                                                                                     1);
        CPlaw2(1) = CP2_P1;
        CPlaw2(2) = CP2_P2;
        CPlaw2(3) = CP2_P3;
        CPlaw2(4) = CP2_P4;
        Handle(Geom_BezierCurve) BezierCP2 = new Geom_BezierCurve(CPlaw2);


        //CP3
        TColgp_Array1OfPnt CPlawP3(1, 4);
        gp_Pnt CP3_P1(IP31x, IP31y, 0), CP3_P2(IP32x, IP32y, 0.3), CP3_P3(IP33x, IP33y, 0.6), CP3_P4(IP34x, IP34y,
                                                                                                     1);
        CPlawP3(1) = CP3_P1;
        CPlawP3(2) = CP3_P2;
        CPlawP3(3) = CP3_P3;
        CPlawP3(4) = CP3_P4;
        Handle(Geom_BezierCurve) BezierCP3 = new Geom_BezierCurve(CPlawP3);

        //CP4
        TColgp_Array1OfPnt CPlawP4(1, 4);
        gp_Pnt CP4_P1(IP41x, IP41y, 0), CP4_P2(IP42x, IP42y, 0.3), CP4_P3(IP43x, IP43y, 0.6), CP4_P4(IP44x, IP44y,
                                                                                                     1);
        CPlawP4(1) = CP4_P1;
        CPlawP4(2) = CP4_P2;
        CPlawP4(3) = CP4_P3;
        CPlawP4(4) = CP4_P4;
        Handle(Geom_BezierCurve) BezierCP4 = new Geom_BezierCurve(CPlawP4);

        //CP5
        TColgp_Array1OfPnt CPlawP5(1, 4);
        gp_Pnt CP5_P1(IP51x, IP51y, 0), CP5_P2(IP52x, IP52y, 0.3), CP5_P3(IP53x, IP53y, 0.6), CP5_P4(IP54x, IP54y,
                                                                                                     1);
        CPlawP5(1) = CP5_P1;
        CPlawP5(2) = CP5_P2;
        CPlawP5(3) = CP5_P3;
        CPlawP5(4) = CP5_P4;
        Handle(Geom_BezierCurve) BezierCP5 = new Geom_BezierCurve(CPlawP5);

        //CP6
        TColgp_Array1OfPnt CPlawP6(1, 4);
        gp_Pnt CP6_P1(IP61x, IP61y, 0), CP6_P2(IP62x, IP62y, 0.3), CP6_P3(IP63x, IP63y, 0.6), CP6_P4(IP64x, IP64y,
                                                                                                     1);
        CPlawP6(1) = CP6_P1;
        CPlawP6(2) = CP6_P2;
        CPlawP6(3) = CP6_P3;
        CPlawP6(4) = CP6_P4;
        Handle(Geom_BezierCurve) BezierCP6 = new Geom_BezierCurve(CPlawP6);

        //CP7
        TColgp_Array1OfPnt CPlawP7(1, 4);
        gp_Pnt CP7_P1(IP71x, IP71y, 0), CP7_P2(IP72x, IP72y, 0.3), CP7_P3(IP73x, IP73y, 0.6), CP7_P4(IP74x, IP74y,
                                                                                                     1);
        CPlawP7(1) = CP7_P1;
        CPlawP7(2) = CP7_P2;
        CPlawP7(3) = CP7_P3;
        CPlawP7(4) = CP7_P4;
        Handle(Geom_BezierCurve) BezierCP7 = new Geom_BezierCurve(CPlawP7);

        //CP8
        TColgp_Array1OfPnt CPlawP8(1, 4);
        gp_Pnt CP8_P1(IP81x, IP81y, 0), CP8_P2(IP82x, IP82y, 0.3), CP8_P3(IP83x, IP83y, 0.6), CP8_P4(IP84x, IP84y,
                                                                                                     1);
        CPlawP8(1) = CP8_P1;
        CPlawP8(2) = CP8_P2;
        CPlawP8(3) = CP8_P3;
        CPlawP8(4) = CP8_P4;
        Handle(Geom_BezierCurve) BezierCP8 = new Geom_BezierCurve(CPlawP8);

        //CP9
        TColgp_Array1OfPnt CPlawP9(1, 4);
        gp_Pnt CP9_P1(IP91x, IP91y, 0), CP9_P2(IP92x, IP92y, 0.3), CP9_P3(IP93x, IP93y, 0.6), CP9_P4(IP94x, IP94y,
                                                                                                     1);
        CPlawP9(1) = CP9_P1;
        CPlawP9(2) = CP9_P2;
        CPlawP9(3) = CP9_P3;
        CPlawP9(4) = CP9_P4;
        Handle(Geom_BezierCurve) BezierCP9 = new Geom_BezierCurve(CPlawP9);


        //CP10
        TColgp_Array1OfPnt CPlawP10(1, 4);
        gp_Pnt CP10_P1(IP101x, IP101y, 0), CP10_P2(IP102x, IP102y, 0.3), CP10_P3(IP103x, IP103y, 0.6), CP10_P4(
                IP104x, IP104y, 1);
        CPlawP10(1) = CP10_P1;
        CPlawP10(2) = CP10_P2;
        CPlawP10(3) = CP10_P3;
        CPlawP10(4) = CP10_P4;
        Handle(Geom_BezierCurve) BezierCP10 = new Geom_BezierCurve(CPlawP10);


        //CP11
        TColgp_Array1OfPnt CPlawP11(1, 4);
        gp_Pnt CP11_P1(IP111x, IP111y, 0), CP11_P2(IP112x, IP112y, 0.3), CP11_P3(IP113x, IP113y, 0.6), CP11_P4(
                IP114x, IP114y, 1);
        CPlawP11(1) = CP11_P1;
        CPlawP11(2) = CP11_P2;
        CPlawP11(3) = CP11_P3;
        CPlawP11(4) = CP11_P4;
        Handle(Geom_BezierCurve) BezierCP11 = new Geom_BezierCurve(CPlawP11);


        //CP12
        TColgp_Array1OfPnt CPlawP12(1, 4);
        gp_Pnt CP12_P1(IP121x, IP121y, 0), CP12_P2(IP122x, IP122y, 0.3), CP12_P3(IP123x, IP123y, 0.6), CP12_P4(
                IP124x, IP124y, 1);
        CPlawP12(1) = CP12_P1;
        CPlawP12(2) = CP12_P2;
        CPlawP12(3) = CP12_P3;
        CPlawP12(4) = CP12_P4;
        Handle(Geom_BezierCurve) BezierCP12 = new Geom_BezierCurve(CPlawP12);


        P_Length[i] = GCPnts_AbscissaPoint::Length(GAC, u[0], u[i]);//length of the curve

        Length_step[i] = (P_Length[i] / Length);

        cout << "Length step =" << Length_step[i] << endl;
        //cout << "path length is " << P_Length[i] << endl;

        // -- Create the intersection plane
        P_path[i].SetCoord(0, 0, Length_step[i]);

        gp_Dir constrDir(0, 0, 1);
        constructionPl[i] = gp_Pln(P_path[i], constrDir);
        constructionPlane[i] = new Geom_Plane(constructionPl[i]);

        //IP1
        GeomAPI_IntCS IntCS1(BezierCP1, constructionPlane[i]);
        IntCS1.Perform(BezierCP1, constructionPlane[i]);
        //IP2
        GeomAPI_IntCS IntCS2(BezierCP2, constructionPlane[i]);
        IntCS2.Perform(BezierCP2, constructionPlane[i]);
        //IP3
        GeomAPI_IntCS IntCS3(BezierCP3, constructionPlane[i]);
        IntCS3.Perform(BezierCP3, constructionPlane[i]);
        //IP4
        GeomAPI_IntCS IntCS4(BezierCP4, constructionPlane[i]);
        IntCS4.Perform(BezierCP4, constructionPlane[i]);
        //IP5
        GeomAPI_IntCS IntCS5(BezierCP5, constructionPlane[i]);
        IntCS5.Perform(BezierCP5, constructionPlane[i]);
        //IP6
        GeomAPI_IntCS IntCS6(BezierCP6, constructionPlane[i]);
        IntCS6.Perform(BezierCP6, constructionPlane[i]);
        //IP7
        GeomAPI_IntCS IntCS7(BezierCP7, constructionPlane[i]);
        IntCS7.Perform(BezierCP7, constructionPlane[i]);
        //IP8
        GeomAPI_IntCS IntCS8(BezierCP8, constructionPlane[i]);
        IntCS8.Perform(BezierCP8, constructionPlane[i]);
        //IP9
        GeomAPI_IntCS IntCS9(BezierCP9, constructionPlane[i]);
        IntCS9.Perform(BezierCP9, constructionPlane[i]);
        //IP10
        GeomAPI_IntCS IntCS10(BezierCP10, constructionPlane[i]);
        IntCS10.Perform(BezierCP10, constructionPlane[i]);
        //IP11
        GeomAPI_IntCS IntCS11(BezierCP11, constructionPlane[i]);
        IntCS11.Perform(BezierCP11, constructionPlane[i]);
        //IP12
        GeomAPI_IntCS IntCS12(BezierCP12, constructionPlane[i]);
        IntCS12.Perform(BezierCP12, constructionPlane[i]);


        Standard_Integer NbPoints1 = IntCS1.NbPoints();
        cout << "NB OF INTERSECTION POINTS = " << NbPoints1 << endl;
        Standard_Integer NbPoints2 = IntCS2.NbPoints();
        cout << "NB OF INTERSECTION POINTS2 = " << NbPoints2 << endl;
        gp_Pnt InterPoint1 = IntCS1.Point(1);
        gp_Pnt InterPoint2 = IntCS2.Point(1);
        gp_Pnt InterPoint3 = IntCS3.Point(1);
        gp_Pnt InterPoint4 = IntCS4.Point(1);
        gp_Pnt InterPoint5 = IntCS5.Point(1);
        gp_Pnt InterPoint6 = IntCS6.Point(1);
        gp_Pnt InterPoint7 = IntCS7.Point(1);
        gp_Pnt InterPoint8 = IntCS8.Point(1);
        gp_Pnt InterPoint9 = IntCS9.Point(1);
        gp_Pnt InterPoint10 = IntCS10.Point(1);
        gp_Pnt InterPoint11 = IntCS11.Point(1);
        gp_Pnt InterPoint12 = IntCS12.Point(1);
        cout << "X coord is " << InterPoint1.X() << endl;
        cout << "Y coord is " << InterPoint1.Y() << endl;
//    cout << "Z coord is " << InterPoint.Z() << endl;
        cout << "X12 coord is " << InterPoint12.X() << endl;
        cout << "Y12 coord is " << InterPoint12.Y() << endl;
        cout << "X6 coord is " << InterPoint6.X() << endl;
        cout << "Y6 coord is " << InterPoint6.Y() << endl;

//IP1 = Intersection Point
        Standard_Real IP1x = InterPoint1.X();
        Standard_Real IP1y = InterPoint1.Y();
//IP2 = Intersection Point
        Standard_Real IP2x = InterPoint2.X();
        Standard_Real IP2y = InterPoint2.Y();
//IP3 = Intersection Point
        Standard_Real IP3x = InterPoint3.X();
        Standard_Real IP3y = InterPoint3.Y();
//IP4 = Intersection Point
        Standard_Real IP4x = InterPoint4.X();
        Standard_Real IP4y = InterPoint4.Y();
//IP5 = Intersection Point
        Standard_Real IP5x = InterPoint5.X();
        Standard_Real IP5y = InterPoint5.Y();
//IP6 = Intersection Point
        Standard_Real IP6x = InterPoint6.X();
        Standard_Real IP6y = InterPoint6.Y();
//IP7 = Intersection Point
        Standard_Real IP7x = InterPoint7.X();
        Standard_Real IP7y = InterPoint7.Y();
//IP8 = Intersection Point
        Standard_Real IP8x = InterPoint8.X();
        Standard_Real IP8y = InterPoint8.Y();
//IP9 = Intersection Point
        Standard_Real IP9x = InterPoint9.X();
        Standard_Real IP9y = InterPoint9.Y();
//IP10 = Intersection Point
        Standard_Real IP10x = InterPoint10.X();
        Standard_Real IP10y = InterPoint10.Y();
//IP11 = Intersection Point
        Standard_Real IP11x = InterPoint11.X();
        Standard_Real IP11y = InterPoint11.Y();
//IP12 = Intersection Point
        Standard_Real IP12x = InterPoint12.X();
        Standard_Real IP12y = InterPoint12.Y();


        // Create 2d Section

        // Create 2d points
        gp_Pnt2d IP1;
        gp_Pnt2d IP2;
        gp_Pnt2d IP3;
        gp_Pnt2d IP4;
        gp_Pnt2d IP5;
        gp_Pnt2d IP6;
        gp_Pnt2d IP7;
        gp_Pnt2d IP8;
        gp_Pnt2d IP9;
        gp_Pnt2d IP10;
        gp_Pnt2d IP11;
        gp_Pnt2d IP12;

        // Assign Coord
        IP1.SetCoord(IP1x, IP1y);
        IP2.SetCoord(IP2x, IP2y);
        IP3.SetCoord(IP3x, IP3y);
        IP4.SetCoord(IP4x, IP4y);
        IP5.SetCoord(IP5x, IP5y);
        IP6.SetCoord(IP6x, IP6y);
        IP7.SetCoord(IP7x, IP7y);
        IP8.SetCoord(IP8x, IP8y);
        IP9.SetCoord(IP9x, IP9y);
        IP10.SetCoord(IP10x, IP10y);
        IP11.SetCoord(IP11x, IP11y);
        IP12.SetCoord(IP12x, IP12y);

        cout << "IP1 = " << IP1.X() << endl;
        cout << "IP6 = " << IP6.X() << endl;
        //Standard_Real a=-5, b=5, c=-2.5, e=2.5;

        TColgp_Array1OfPnt2d Bezier11(1, 4);
        Bezier11(1) = IP1;
        Bezier11(2) = IP2;
        Bezier11(3) = IP3;
        Bezier11(4) = IP4;
        Handle(Geom2d_BezierCurve) Bezier1 = new Geom2d_BezierCurve(Bezier11);

        TColgp_Array1OfPnt2d Bezier12(1, 4);
        Bezier12(1) = IP4;
        Bezier12(2) = IP5;
        Bezier12(3) = IP6;
        Bezier12(4) = IP7;
        Handle(Geom2d_BezierCurve) Bezier2 = new Geom2d_BezierCurve(Bezier12);

        TColgp_Array1OfPnt2d Bezier13(1, 4);
        Bezier13(1) = IP7;
        Bezier13(2) = IP8;
        Bezier13(3) = IP9;
        Bezier13(4) = IP10;
        Handle(Geom2d_BezierCurve) Bezier3 = new Geom2d_BezierCurve(Bezier13);

        TColgp_Array1OfPnt2d Bezier14(1, 4);
        Bezier14(1) = IP10;
        Bezier14(2) = IP11;
        Bezier14(3) = IP12;
        Bezier14(4) = IP1;
        Handle(Geom2d_BezierCurve) Bezier4 = new Geom2d_BezierCurve(Bezier14);


        // -- Create the host plane
        Pl1[i] = gp_Pln(gp_Ax3(P[i], d[i], d2[i]));
        s[i] = new Geom_Plane(Pl1[i]);


        Edge_B11[i] = BRepBuilderAPI_MakeEdge(Bezier1, s[i]);
        ShapeBuild_Edge().BuildCurve3d(Edge_B11[i]);
        Edge_B21[i] = BRepBuilderAPI_MakeEdge(Bezier2, s[i]);
        ShapeBuild_Edge().BuildCurve3d(Edge_B21[i]);
        Edge_B31[i] = BRepBuilderAPI_MakeEdge(Bezier3, s[i]);
        ShapeBuild_Edge().BuildCurve3d(Edge_B31[i]);
        Edge_B41[i] = BRepBuilderAPI_MakeEdge(Bezier4, s[i]);
        ShapeBuild_Edge().BuildCurve3d(Edge_B41[i]);
        /*TopoDS_Edge Edge_B21[i]   = BRepBuilderAPI_MakeEdge(Bezier2, Pl[i]);
        TopoDS_Edge Edge_B31[i]   = BRepBuilderAPI_MakeEdge(Bezier3, Pl[i]);
        TopoDS_Edge Edge_B41[i]   = BRepBuilderAPI_MakeEdge(Bezier4, Pl[i]);*/


        // Create wire

        BRepBuilderAPI_MakeWire mkWire;
        mkWire.Add(Edge_B11[i]);
        mkWire.Add(Edge_B21[i]);
        mkWire.Add(Edge_B31[i]);
        mkWire.Add(Edge_B41[i]);
        Wire[i] = mkWire.Wire();



        // Add section to skinner
        aGenerator.AddWire(Wire[i]);

    }

    // Finally, build the surface
    try {
        aGenerator.Build();
    }
    catch (...) {
        std::cout << "Error: crash in BRepOffsetAPI_ThruSections" << std::endl;

    }
    //
    if (!aGenerator.IsDone()) {
        std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;

    }

    tube = aGenerator.Shape();


    return tube;
}

TopoDS_Shape OCCTDataProvider::ConstructSquaredUbend(){

//    std::vector<Standard_Real> design =
//            {
//                    Standard_Real(-5.), Standard_Real(5.), Standard_Real(-5.), Standard_Real(5.), Standard_Real(-5.), Standard_Real(5.), Standard_Real(-5.), Standard_Real(5.),
//                    Standard_Real(-2.5), Standard_Real(5.), Standard_Real(-2.5), Standard_Real(5.), Standard_Real(-2.5), Standard_Real(5.), Standard_Real(-2.5), Standard_Real(5.),
//                    Standard_Real(2.5), Standard_Real(5.), Standard_Real(2.5), Standard_Real(5.), Standard_Real(2.5), Standard_Real(5.), Standard_Real(2.5), Standard_Real(5.),
//                    Standard_Real(5.), Standard_Real(5.), Standard_Real(5.), Standard_Real(5.), Standard_Real(5.), Standard_Real(5.), Standard_Real(5.), Standard_Real(5.),
//                    Standard_Real(5.), Standard_Real(2.5), Standard_Real(5.), Standard_Real(2.5), Standard_Real(5.), Standard_Real(2.5), Standard_Real(5.), Standard_Real(2.5),
//                    Standard_Real(5.), Standard_Real(-2.5), Standard_Real(5.), Standard_Real(-2.5), Standard_Real(5.), Standard_Real(-2.5), Standard_Real(5.), Standard_Real(-2.5),
//                    Standard_Real(5.), Standard_Real(-5.), Standard_Real(5.), Standard_Real(-5.), Standard_Real(5.), Standard_Real(-5.), Standard_Real(5.), Standard_Real(-5.),
//                    Standard_Real(2.5), Standard_Real(-5.), Standard_Real(2.5), Standard_Real(-5.), Standard_Real(2.5), Standard_Real(-5.), Standard_Real(2.5), Standard_Real(-5.),
//                    Standard_Real(-2.5), Standard_Real(-5.), Standard_Real(-2.5), Standard_Real(-5.), Standard_Real(-2.5), Standard_Real(-5.), Standard_Real(-2.5), Standard_Real(-5.),
//                    Standard_Real(-5.), Standard_Real(-5.), Standard_Real(-5.), Standard_Real(-5.), Standard_Real(-5.), Standard_Real(-5.), Standard_Real(-5.), Standard_Real(-5.),
//                    Standard_Real(-5.), Standard_Real(-2.5), Standard_Real(-5.), Standard_Real(-2.5), Standard_Real(-5.), Standard_Real(-2.5), Standard_Real(-5.), Standard_Real(-2.5),
//                    Standard_Real(-5.), Standard_Real(2.5), Standard_Real(-5.), Standard_Real(2.5), Standard_Real(-5.), Standard_Real(2.5), Standard_Real(-5.), Standard_Real(2.5)
//            };


    int maxSlices= 70;
    int maxLengthVerticalPipes= 250;
    //---------------------------------------------------------------------------
    // Stage 1: Construct laws from design parameters
    //---------------------------------------------------------------------------

    //12 laws
    Handle(Geom_BezierCurve) BezierCP[12];

    for(Standard_Integer i = 0; i < 12; i++)
    {
        TColgp_Array1OfPnt CPlaw(1, 4);
        CPlaw(1) = gp_Pnt(designParameters[i*8], designParameters[i*8 + 1], 0.);
        CPlaw(2) = gp_Pnt(designParameters[i*8 + 2], designParameters[i*8 + 3], 0.3);
        CPlaw(3) = gp_Pnt(designParameters[i*8 + 4], designParameters[i*8 + 5], 0.6);
        CPlaw(4) = gp_Pnt(designParameters[i*8 + 6], designParameters[i*8 + 7], 1.);
        BezierCP[i] = new Geom_BezierCurve(CPlaw);
    }

    //---------------------------------------------------------------------------
    // Stage 2: U-bend: Start with a Bezier path for U-bend
    //---------------------------------------------------------------------------

    TColgp_Array1OfPnt pathPoles(1, 4);
    pathPoles(1) = gp_Pnt(50., -25.,   0.);
    pathPoles(2) = gp_Pnt(0., -25.,   0.);
    pathPoles(3) = gp_Pnt(0., 25.,  0.);
    pathPoles(4) = gp_Pnt(50., 25., 0.);
    Handle(Geom_BezierCurve) path = new Geom_BezierCurve(pathPoles);

    //DrawTrSurf::Set("path", path);

    GeomAdaptor_Curve GAC;
    GAC.Load(path);
    Standard_Real Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve

    //---------------------------------------------------------------------------
    // Stage 3: U-bend: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    TopoDS_Shape tube;
    gp_Dir d[maxSlices];
    gp_Dir d2(0,0,1);
    TopoDS_Wire Wire[maxSlices];

    Standard_Real u[maxSlices];

    u[0]             = path->FirstParameter();
    u[maxSlices - 1] = path->LastParameter();

    const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);

    for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
    {
        u[i] = u[0] + i*ustep;
    }

    // Prepare skinning tool

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;
    const Standard_Real    pres3d  = 1.0e-02;

    BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);
    aGenerator.CheckCompatibility(Standard_False);
    //
    for ( Standard_Integer i = 0; i < maxSlices; ++i )
    {

        gp_Pnt P;
        gp_Vec V;
        path->D1(u[i], P, V);
        d[i] = gp_Dir(V);

        Standard_Real P_Length = GCPnts_AbscissaPoint::Length(GAC, u[0], u[i]);//length of the curve

        Standard_Real Length_step = (P_Length/Length);

        // -- Create the intersection plane
        gp_Pnt P_path(0, 0, Length_step);

        gp_Pln constructionPl(P_path, d2);
        Handle(Geom_Plane) constructionPlane = new Geom_Plane (constructionPl);

        //IP1
        GeomAPI_IntCS IntCS1(BezierCP[0], constructionPlane);
        //IP2
        GeomAPI_IntCS IntCS2(BezierCP[1], constructionPlane);
        //IP3
        GeomAPI_IntCS IntCS3(BezierCP[2], constructionPlane);
        //IP4
        GeomAPI_IntCS IntCS4(BezierCP[3], constructionPlane);
        //IP5
        GeomAPI_IntCS IntCS5(BezierCP[4], constructionPlane);
        //IP6
        GeomAPI_IntCS IntCS6(BezierCP[5], constructionPlane);
        //IP7
        GeomAPI_IntCS IntCS7(BezierCP[6], constructionPlane);
        //IP8
        GeomAPI_IntCS IntCS8(BezierCP[7], constructionPlane);
        //IP9
        GeomAPI_IntCS IntCS9(BezierCP[8], constructionPlane);
        //IP10
        GeomAPI_IntCS IntCS10(BezierCP[9], constructionPlane);
        //IP11
        GeomAPI_IntCS IntCS11(BezierCP[10], constructionPlane);
        //IP12
        GeomAPI_IntCS IntCS12(BezierCP[11], constructionPlane);

        TColgp_Array1OfPnt2d Bezier11(1, 4);
        Bezier11(1) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Bezier11(2) = gp_Pnt2d(IntCS2.Point(1).X(), IntCS2.Point(1).Y());
        Bezier11(3) = gp_Pnt2d(IntCS3.Point(1).X(), IntCS3.Point(1).Y());
        Bezier11(4) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier1 = new Geom2d_BezierCurve(Bezier11);

        TColgp_Array1OfPnt2d Bezier12(1, 4);
        Bezier12(1) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Bezier12(2) = gp_Pnt2d(IntCS5.Point(1).X(), IntCS5.Point(1).Y());
        Bezier12(3) = gp_Pnt2d(IntCS6.Point(1).X(), IntCS6.Point(1).Y());
        Bezier12(4) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier2 = new Geom2d_BezierCurve(Bezier12);

        TColgp_Array1OfPnt2d Bezier13(1, 4);
        Bezier13(1) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Bezier13(2) = gp_Pnt2d(IntCS8.Point(1).X(), IntCS8.Point(1).Y());
        Bezier13(3) = gp_Pnt2d(IntCS9.Point(1).X(), IntCS9.Point(1).Y());
        Bezier13(4) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier3 = new Geom2d_BezierCurve(Bezier13);

        TColgp_Array1OfPnt2d Bezier14(1, 4);
        Bezier14(1) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Bezier14(2) = gp_Pnt2d(IntCS11.Point(1).X(), IntCS11.Point(1).Y());
        Bezier14(3) = gp_Pnt2d(IntCS12.Point(1).X(), IntCS12.Point(1).Y());
        Bezier14(4) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier4 = new Geom2d_BezierCurve(Bezier14);

        // -- Create the host plane
        gp_Pln Pl1(gp_Ax3(P, d[i], d2 ));
        Handle(Geom_Plane) s = new Geom_Plane (Pl1);


        TopoDS_Edge Edge_B11 = BRepBuilderAPI_MakeEdge(Bezier1, s);
        ShapeBuild_Edge().BuildCurve3d(Edge_B11);
        TopoDS_Edge Edge_B21 = BRepBuilderAPI_MakeEdge(Bezier2, s);
        ShapeBuild_Edge().BuildCurve3d(Edge_B21);
        TopoDS_Edge Edge_B31 = BRepBuilderAPI_MakeEdge(Bezier3, s);
        ShapeBuild_Edge().BuildCurve3d(Edge_B31);
        TopoDS_Edge Edge_B41 = BRepBuilderAPI_MakeEdge(Bezier4, s);
        ShapeBuild_Edge().BuildCurve3d(Edge_B41);

        // Create wire

        BRepBuilderAPI_MakeWire mkWire;
        mkWire.Add(Edge_B11);
        mkWire.Add(Edge_B21);
        mkWire.Add(Edge_B31);
        mkWire.Add(Edge_B41);
        Wire[i] = mkWire.Wire();

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
    }
    //
    if ( !aGenerator.IsDone() )
    {
        std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
    }

    tube = aGenerator.Shape();

    //---------------------------------------------------------------------------
    // Stage 4 - // Inlet and Outlet Pipe
    //---------------------------------------------------------------------------

    gp_Vec V0(d[0]);
    BRepPrimAPI_MakePrism mkPrismIn(Wire[0], -maxLengthVerticalPipes*V0);
    mkPrismIn.Build();
    TopoDS_Shape InletPipe = mkPrismIn.Shape();
    // Get the Inlet wire
    TopoDS_Wire wbaseIn = TopoDS::Wire(mkPrismIn.LastShape());
    // Create the Inlet face from this wire
    TopoDS_Face fbaseIn = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseIn)).Face();

    gp_Vec Vmaxslices(d[maxSlices-1]);
    BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], maxLengthVerticalPipes*Vmaxslices);
    mkPrismOut.Build();
    TopoDS_Shape OutletPipe = mkPrismOut.Shape();
    // Get the outlet wire
    TopoDS_Wire wbaseOut = TopoDS::Wire(mkPrismOut.LastShape());
    // Create the outlet face from this wire
    TopoDS_Face fbaseOut = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseOut)).Face();

    //---------------------------------------------------------------------------
    // Stage 5 - // Build the shell or solid
    //---------------------------------------------------------------------------

    BRepBuilderAPI_Sewing Sewing;
    Sewing.Add(fbaseIn);
    Sewing.Add(InletPipe);
    Sewing.Add(tube);
    Sewing.Add(OutletPipe);
    Sewing.Add(fbaseOut);
    Sewing.Perform();
    tube = Sewing.SewedShape();
    tube = TopoDS::Shell(tube);
    //  TopoDS_Solid solidTube;
    //  BRep_Builder mkSolid;
    //  mkSolid.MakeSolid(solidTube);
    //  mkSolid.Add(solidTube, tube);

    return tube;
}

TopoDS_Shape OCCTDataProvider::UbendHorizontalLegs()
{

   // vector<Standard_Real>design={Standard_Real(5),Standard_Real(5)};

    //---------------------------------------------------------------------------
    // Stage 1: Definition Design Parameters
    //---------------------------------------------------------------------------
    int maxSlices  = 20;
    int maxLengthVerticalPipes=50;

    TColgp_Array1OfPnt pathPoles(1, 6);

    pathPoles(1) = gp_Pnt(0.,   0.,   0.);
    pathPoles(2) = gp_Pnt(20.,   0.,   0.);
    // pathPoles(3) = gp_Pnt(20.,   0.,   0.);
    pathPoles(3) = gp_Pnt(45.,designParameters[0],0.);//(0.,  50.,  0.);20.,50.,0.
    pathPoles(4) = gp_Pnt(50.,designParameters[1],0.);//(50., 50., 0.);60.,100.,0.
    pathPoles(5) = gp_Pnt(80.,0.,0.);
    //  pathPoles(7) = gp_Pnt(140.,0.,0.);
    pathPoles(6) = gp_Pnt(100.,0.,0.);//(50., 0.,   0.);150.,0.,0.

    Handle(Geom_BezierCurve) path = new Geom_BezierCurve(pathPoles);

//IP1 = Intersection Point
    //Standard_Real IPx11 = -5, IPx12 = -5, IPx13 = -10, IPx14 = -5;
    //Standard_Real IPy11 = 5, IPy12 = 5, IPy13 = 10, IPy14 = 5;
    Standard_Real IPx11 = -5, IPx12 = -5, IPx13 = -5, IPx14 = -5;
    Standard_Real IPy11 = 5, IPy12 = 5, IPy13 = 5, IPy14 = 5;
//IP2 = Intersection Point
    //Standard_Real IPx21 = -2.5, IPx22 = -2.5, IPx23 = -2.5, IPx24 = -2.5;
    //Standard_Real IPy21 = 5, IPy22 = 5, IPy23 = 5, IPy24 = 5;
    Standard_Real IPx21 = -2.5, IPx22 = -2.5, IPx23 = -2.5, IPx24 = -2.5;
    Standard_Real IPy21 = 5, IPy22 = 5, IPy23 = 5, IPy24 = 5;
//IP3 = Intersection Point
    Standard_Real IPx31 = 2.5, IPx32 = 2.5, IPx33 = 2.5, IPx34 = 2.5;
    Standard_Real IPy31 = 5, IPy32 = 5, IPy33 = 5, IPy34 = 5;
//IP4 = Intersection Point
    Standard_Real IPx41 = 5, IPx42 = 5, IPx43 = 5, IPx44 = 5;
    Standard_Real IPy41 = 5, IPy42 = 5, IPy43 = 5, IPy44 = 5;
//IP5 = Intersection Point
    Standard_Real IPx51 = 5, IPx52 = 5, IPx53 = 5, IPx54 = 5;
    Standard_Real IPy51 = 2.5, IPy52 = 2.5, IPy53 = 2.5, IPy54 = 2.5;
//IP6 = Intersection Point
    Standard_Real IPx61 = 5, IPx62 = 5, IPx63 = 5, IPx64 = 5;
    Standard_Real IPy61 = -2.5, IPy62 = -2.5, IPy63 = -2.5, IPy64 = -2.5;
//IP7 = Intersection Point
    Standard_Real IPx71 = 5, IPx72 = 5, IPx73 = 5, IPx74 = 5;
    Standard_Real IPy71 = -5, IPy72 = -5, IPy73 = -5, IPy74 = -5;
//IP8 = Intersection Point
    Standard_Real IPx81 = 2.5, IPx82 = 2.5, IPx83 = 2.5, IPx84 = 2.5;
    Standard_Real IPy81 = -5, IPy82 = -5, IPy83 = -5, IPy84 = -5;
//IP9 = Intersection Point
    //Standard_Real IPx91 = -2.5, IPx92 = -2.5, IPx93 = -2.5, IPx94 = -2.5;
    //Standard_Real IPy91 = -5, IPy92 = -5, IPy93 = -5, IPy94 = -5;
    Standard_Real IPx91 = -2.5, IPx92 = -2.5, IPx93 = -2.5, IPx94 = -2.5;
    Standard_Real IPy91 = -5, IPy92 = -5, IPy93 = -5, IPy94 = -5;
//IP10 = Intersection Point
    //Standard_Real IPx101 =-5, IPx102 = -5, IPx103 = -10, IPx104 = -5;
    //Standard_Real IPy101 = -5, IPy102 = -5, IPy103 = -10, IPy104 = -5;
    Standard_Real IPx101 =-5, IPx102 = -5, IPx103 = -5, IPx104 = -5;
    Standard_Real IPy101 = -5, IPy102 = -5, IPy103 = -5, IPy104 = -5;
//IP11 = Intersection Point
    //Standard_Real IPx111 = -5, IPx112 = -5, IPx113 = -25, IPx114 = -5;
    //Standard_Real IPy111 = -2.5, IPy112 = -2.5, IPy113 = -25, IPy114 = -2.5;
    Standard_Real IPx111 = -5, IPx112 = -5, IPx113 = -5, IPx114 = -5;
    Standard_Real IPy111 = -2.5, IPy112 = -2.5, IPy113 = -2.5, IPy114 = -2.5;
//IP12 = Intersection Point
    //Standard_Real IPx121 = -5, IPx122 = -5, IPx123 = -25, IPx124 = -5;
    //Standard_Real IPy121 = 2.5, IPy122 = 2.5, IPy123 = 25, IPy124 = 2.5;
    Standard_Real IPx121 = -5, IPx122 = -5, IPx123 = -5, IPx124 = -5;
    Standard_Real IPy121 = 2.5, IPy122 = 2.5, IPy123 = 2.5, IPy124 = 2.5;




    //---------------------------------------------------------------------------
    // Stage 3: U-bend: Start with a Bezier path for U-bend
    //---------------------------------------------------------------------------


    //


    Standard_Real Length;
    GeomAdaptor_Curve GAC;
    //
    GAC.Load(path);
    Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve
    //cout << "Path length is " << Length << endl;



    //---------------------------------------------------------------------------
    // Stage 4: U-bend: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    //General

    TopoDS_Shape tube;
    //Handle(Geom_BSplineSurface) nurbs;
    gp_Pnt P[maxSlices];
    gp_Vec V[maxSlices];
    gp_Dir d[maxSlices];
    gp_Vec V2[maxSlices];
    gp_Dir d2[maxSlices];
    gp_Pln Pl1[maxSlices];
    gp_Pln Pl[maxSlices];
    //Geom_Plane Pl[maxSlices];
    Handle(Geom_Plane) s[maxSlices];
    TopoDS_Edge Edge_B11[maxSlices];
    TopoDS_Edge Edge_B21[maxSlices];
    TopoDS_Edge Edge_B31[maxSlices];
    TopoDS_Edge Edge_B41[maxSlices];
    TopoDS_Wire Wire[maxSlices];
    Standard_Real P_Length[maxSlices];
    Standard_Real CP1_Length[maxSlices];
    Standard_Real Length_step[maxSlices];
    Standard_Real CP1_Length_step[maxSlices];
    gp_Pln constructionPl[maxSlices];
    Handle(Geom_Plane) constructionPlane[maxSlices];
    gp_Pnt P_path[maxSlices];
    gp_Pnt InterPoint[maxSlices];


    Standard_Real u[maxSlices];
    //Standard_Real lPath[maxSlices];
    //
    u[0]             = path->FirstParameter();
    u[maxSlices - 1] = path->LastParameter();
    //
    const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);
    //const Standard_Real lStep = (length) / (maxSlices - 1);
    //
    for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
    {
        u[i] = u[0] + i*ustep;
        //lPath[i] = i*lStep;
    }

    // Prepare skinning tool

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;
    const Standard_Real    pres3d  = 1.0e-02;

    BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
    aGenerator.CheckCompatibility(Standard_True);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);
    //
    for ( Standard_Integer i = 0; i < maxSlices; ++i )
    {

        path->D2(u[i], P[i], V[i], V2[i]);
        d[i] = gp_Dir(V[i]);
        d2[i] = gp_Dir(V2[i]);

        //---------------------------------------------------------------------------
        // build law.
        //---------------------------------------------------------------------------

        //CP1
        TColgp_Array1OfPnt CPlaw1(1, 4);
        gp_Pnt CP1_P1(IPx11, IPy11,0), CP1_P2(IPx12, IPy12, 0.3), CP1_P3(IPx13, IPy13, 0.6), CP1_P4(IPx14, IPy14, 1);
        CPlaw1(1) = CP1_P1;
        CPlaw1(2) = CP1_P2;
        CPlaw1(3) = CP1_P3;
        CPlaw1(4) = CP1_P4;
        Handle(Geom_BezierCurve) BezierCP1 = new Geom_BezierCurve(CPlaw1);

        /*Standard_Real LengthLaw1;
        GeomAdaptor_Curve GAC_Law1;
        //
        GAC_Law1.Load(BezierCP1);
        LengthLaw1 = GCPnts_AbscissaPoint::Length(GAC_Law1);//length of the curve
        CP1_Length[i] = GCPnts_AbscissaPoint::Length(GAC_Law1, u[0], u[i]);
        CP1_Length_step[i] = CP1_Length[i]/LengthLaw1;*/


        //CP2
        TColgp_Array1OfPnt CPlaw2(1, 4);
        gp_Pnt CP2_P1(IPx21, IPy21,0), CP2_P2(IPx22, IPy22, 0.3), CP2_P3(IPx23, IPy23, 0.6), CP2_P4(IPx24, IPy24, 1);
        CPlaw2(1) = CP2_P1;
        CPlaw2(2) = CP2_P2;
        CPlaw2(3) = CP2_P3;
        CPlaw2(4) = CP2_P4;
        Handle(Geom_BezierCurve) BezierCP2 = new Geom_BezierCurve(CPlaw2);


        //CP3
        TColgp_Array1OfPnt CPlawP3(1, 4);
        gp_Pnt CP3_P1(IPx31, IPy31,0), CP3_P2(IPx32, IPy32, 0.3), CP3_P3(IPx33, IPy33, 0.6), CP3_P4(IPx34, IPy34, 1);
        CPlawP3(1) = CP3_P1;
        CPlawP3(2) = CP3_P2;
        CPlawP3(3) = CP3_P3;
        CPlawP3(4) = CP3_P4;
        Handle(Geom_BezierCurve) BezierCP3 = new Geom_BezierCurve(CPlawP3);

        //CP4
        TColgp_Array1OfPnt CPlawP4(1, 4);
        gp_Pnt CP4_P1(IPx41, IPy41,0), CP4_P2(IPx42, IPy42, 0.3), CP4_P3(IPx43, IPy43, 0.6), CP4_P4(IPx44, IPy44, 1);
        CPlawP4(1) = CP4_P1;
        CPlawP4(2) = CP4_P2;
        CPlawP4(3) = CP4_P3;
        CPlawP4(4) = CP4_P4;
        Handle(Geom_BezierCurve) BezierCP4 = new Geom_BezierCurve(CPlawP4);

        //CP5
        TColgp_Array1OfPnt CPlawP5(1, 4);
        gp_Pnt CP5_P1(IPx51, IPy51,0), CP5_P2(IPx52, IPy52, 0.3), CP5_P3(IPx53, IPy53, 0.6), CP5_P4(IPx54, IPy54, 1);
        CPlawP5(1) = CP5_P1;
        CPlawP5(2) = CP5_P2;
        CPlawP5(3) = CP5_P3;
        CPlawP5(4) = CP5_P4;
        Handle(Geom_BezierCurve) BezierCP5 = new Geom_BezierCurve(CPlawP5);

        //CP6
        TColgp_Array1OfPnt CPlawP6(1, 4);
        gp_Pnt CP6_P1(IPx61,IPy61,0), CP6_P2(IPx62,IPy62, 0.3), CP6_P3(IPx63,IPy63, 0.6), CP6_P4(IPx64,IPy64, 1);
        CPlawP6(1) = CP6_P1;
        CPlawP6(2) = CP6_P2;
        CPlawP6(3) = CP6_P3;
        CPlawP6(4) = CP6_P4;
        Handle(Geom_BezierCurve) BezierCP6 = new Geom_BezierCurve(CPlawP6);

        //CP7
        TColgp_Array1OfPnt CPlawP7(1, 4);
        gp_Pnt CP7_P1(IPx71,IPy71,0), CP7_P2(IPx72,IPy72, 0.3), CP7_P3(IPx73,IPy73, 0.6), CP7_P4(IPx74,IPy74, 1);
        CPlawP7(1) = CP7_P1;
        CPlawP7(2) = CP7_P2;
        CPlawP7(3) = CP7_P3;
        CPlawP7(4) = CP7_P4;
        Handle(Geom_BezierCurve) BezierCP7 = new Geom_BezierCurve(CPlawP7);

        //CP8
        TColgp_Array1OfPnt CPlawP8(1, 4);
        gp_Pnt CP8_P1(IPx81,IPy81,0), CP8_P2(IPx82,IPy82, 0.3), CP8_P3(IPx83,IPy83, 0.6), CP8_P4(IPx84,IPy84, 1);
        CPlawP8(1) = CP8_P1;
        CPlawP8(2) = CP8_P2;
        CPlawP8(3) = CP8_P3;
        CPlawP8(4) = CP8_P4;
        Handle(Geom_BezierCurve) BezierCP8 = new Geom_BezierCurve(CPlawP8);

        //CP9
        TColgp_Array1OfPnt CPlawP9(1, 4);
        gp_Pnt CP9_P1(IPx91,IPy91,0), CP9_P2(IPx92, IPy92, 0.3), CP9_P3(IPx93, IPy93, 0.6), CP9_P4(IPx94, IPy94, 1);
        CPlawP9(1) = CP9_P1;
        CPlawP9(2) = CP9_P2;
        CPlawP9(3) = CP9_P3;
        CPlawP9(4) = CP9_P4;
        Handle(Geom_BezierCurve) BezierCP9 = new Geom_BezierCurve(CPlawP9);


        //CP10
        TColgp_Array1OfPnt CPlawP10(1, 4);
        gp_Pnt CP10_P1(IPx101, IPy101,0), CP10_P2(IPx102, IPy102, 0.3), CP10_P3(IPx103, IPy103, 0.6), CP10_P4(IPx104, IPy104, 1);
        CPlawP10(1) = CP10_P1;
        CPlawP10(2) = CP10_P2;
        CPlawP10(3) = CP10_P3;
        CPlawP10(4) = CP10_P4;
        Handle(Geom_BezierCurve) BezierCP10 = new Geom_BezierCurve(CPlawP10);


        //CP11
        TColgp_Array1OfPnt CPlawP11(1, 4);
        gp_Pnt CP11_P1(IPx111, IPy111,0), CP11_P2(IPx112, IPy112, 0.3), CP11_P3(IPx113, IPy113, 0.6), CP11_P4(IPx114, IPy114, 1);
        CPlawP11(1) = CP11_P1;
        CPlawP11(2) = CP11_P2;
        CPlawP11(3) = CP11_P3;
        CPlawP11(4) = CP11_P4;
        Handle(Geom_BezierCurve) BezierCP11 = new Geom_BezierCurve(CPlawP11);


        //CP12
        TColgp_Array1OfPnt CPlawP12(1, 4);
        gp_Pnt CP12_P1(IPx121, IPy121,0), CP12_P2(IPx122, IPy122, 0.3), CP12_P3(IPx123, IPy123, 0.6), CP12_P4(IPx124, IPy124, 1);
        CPlawP12(1) = CP12_P1;
        CPlawP12(2) = CP12_P2;
        CPlawP12(3) = CP12_P3;
        CPlawP12(4) = CP12_P4;
        Handle(Geom_BezierCurve) BezierCP12 = new Geom_BezierCurve(CPlawP12);


        P_Length[i] = GCPnts_AbscissaPoint::Length(GAC, u[0], u[i]);//length of the curve

        Length_step[i] = (P_Length[i]/Length);

        //cout << "Length step =" << Length_step[i] << endl;
        //cout << "la lunghezza del path è " << P_Length[i] << endl;

        // -- Create the intersection plane
        P_path[i].SetCoord(0, 0, Length_step[i]);

        gp_Dir constrDir (0,0,1);
        constructionPl[i] = gp_Pln(P_path[i], constrDir);
        constructionPlane[i] = new Geom_Plane (constructionPl[i]);

        //IP1
        GeomAPI_IntCS IntCS1(BezierCP1, constructionPlane[i]);
        IntCS1.Perform(BezierCP1, constructionPlane[i]);
        //IP2
        GeomAPI_IntCS IntCS2(BezierCP2, constructionPlane[i]);
        IntCS2.Perform(BezierCP2, constructionPlane[i]);
        //IP3
        GeomAPI_IntCS IntCS3(BezierCP3, constructionPlane[i]);
        IntCS3.Perform(BezierCP3, constructionPlane[i]);
        //IP4
        GeomAPI_IntCS IntCS4(BezierCP4, constructionPlane[i]);
        IntCS4.Perform(BezierCP4, constructionPlane[i]);
        //IP5
        GeomAPI_IntCS IntCS5(BezierCP5, constructionPlane[i]);
        IntCS5.Perform(BezierCP5, constructionPlane[i]);
        //IP6
        GeomAPI_IntCS IntCS6(BezierCP6, constructionPlane[i]);
        IntCS6.Perform(BezierCP6, constructionPlane[i]);
        //IP7
        GeomAPI_IntCS IntCS7(BezierCP7, constructionPlane[i]);
        IntCS7.Perform(BezierCP7, constructionPlane[i]);
        //IP8
        GeomAPI_IntCS IntCS8(BezierCP8, constructionPlane[i]);
        IntCS8.Perform(BezierCP8, constructionPlane[i]);
        //IP9
        GeomAPI_IntCS IntCS9(BezierCP9, constructionPlane[i]);
        IntCS9.Perform(BezierCP9, constructionPlane[i]);
        //IP10
        GeomAPI_IntCS IntCS10(BezierCP10, constructionPlane[i]);
        IntCS10.Perform(BezierCP10, constructionPlane[i]);
        //IP11
        GeomAPI_IntCS IntCS11(BezierCP11, constructionPlane[i]);
        IntCS11.Perform(BezierCP11, constructionPlane[i]);
        //IP12
        GeomAPI_IntCS IntCS12(BezierCP12, constructionPlane[i]);
        IntCS12.Perform(BezierCP12, constructionPlane[i]);


        /*if (IntCS.IsDone())
        {
        Standard_Integer NbPoints = IntCS.NbPoints();
        //gp_Pnt InterPoint = IntCS.Point(1);
        cout << "X coord is " << InterPoint.X() << endl;
        cout << "Y coord is " << InterPoint.Y() << endl;
        cout << "Z coord is " << InterPoint.Z() << endl;
        cout << "NB OF INTERSECTION POINTS = " << NbPoints << endl;
        }*/
        Standard_Integer NbPoints1 = IntCS1.NbPoints();
        //cout << "NB OF INTERSECTION POINTS = " << NbPoints1 << endl;
        Standard_Integer NbPoints2 = IntCS2.NbPoints();
        //cout << "NB OF INTERSECTION POINTS2 = " << NbPoints2 << endl;
        gp_Pnt InterPoint1 = IntCS1.Point(1);
        gp_Pnt InterPoint2 = IntCS2.Point(1);
        gp_Pnt InterPoint3 = IntCS3.Point(1);
        gp_Pnt InterPoint4 = IntCS4.Point(1);
        gp_Pnt InterPoint5 = IntCS5.Point(1);
        gp_Pnt InterPoint6 = IntCS6.Point(1);
        gp_Pnt InterPoint7 = IntCS7.Point(1);
        gp_Pnt InterPoint8 = IntCS8.Point(1);
        gp_Pnt InterPoint9 = IntCS9.Point(1);
        gp_Pnt InterPoint10 = IntCS10.Point(1);
        gp_Pnt InterPoint11 = IntCS11.Point(1);
        gp_Pnt InterPoint12 = IntCS12.Point(1);
        //cout << "X coord is " << InterPoint1.X() << endl;
        //cout << "Y coord is " << InterPoint1.Y() << endl;
        //cout << "Z coord is " << InterPoint.Z() << endl;
        //cout << "X12 coord is " << InterPoint12.X() << endl;
        //cout << "Y12 coord is " << InterPoint12.Y() << endl;
        //cout << "X6 coord is " << InterPoint6.X() << endl;
        //cout << "Y6 coord is " << InterPoint6.Y() << endl;

//IP1 = Intersection Point
        Standard_Real IP1x = InterPoint1.X();
        Standard_Real IP1y = InterPoint1.Y();
//IP2 = Intersection Point
        Standard_Real IP2x = InterPoint2.X();
        Standard_Real IP2y = InterPoint2.Y();
//IP3 = Intersection Point
        Standard_Real IP3x = InterPoint3.X();
        Standard_Real IP3y = InterPoint3.Y();
//IP4 = Intersection Point
        Standard_Real IP4x = InterPoint4.X();
        Standard_Real IP4y = InterPoint4.Y();
//IP5 = Intersection Point
        Standard_Real IP5x = InterPoint5.X();
        Standard_Real IP5y = InterPoint5.Y();
//IP6 = Intersection Point
        Standard_Real IP6x = InterPoint6.X();
        Standard_Real IP6y = InterPoint6.Y();
//IP7 = Intersection Point
        Standard_Real IP7x = InterPoint7.X();
        Standard_Real IP7y = InterPoint7.Y();
//IP8 = Intersection Point
        Standard_Real IP8x = InterPoint8.X();
        Standard_Real IP8y = InterPoint8.Y();
//IP9 = Intersection Point
        Standard_Real IP9x = InterPoint9.X();
        Standard_Real IP9y = InterPoint9.Y();
//IP10 = Intersection Point
        Standard_Real IP10x = InterPoint10.X();
        Standard_Real IP10y = InterPoint10.Y();
//IP11 = Intersection Point
        Standard_Real IP11x = InterPoint11.X();
        Standard_Real IP11y = InterPoint11.Y();
//IP12 = Intersection Point
        Standard_Real IP12x = InterPoint12.X();
        Standard_Real IP12y = InterPoint12.Y();



        //---------------------------------------------------------------------------
        //
        //---------------------------------------------------------------------------

        // Construction parameters
        //const double        prec       = 1.0e-6;
        //const GeomAbs_Shape continuity = GeomAbs_C2;
        //const GeomAbs_Shape continuity = GeomAbs_C0;
        //const int           maxDegree  = 25;
        //const int           maxSegment = 1000;

        // Create 2d Section

        // Create 2d points
        gp_Pnt2d IP1;
        gp_Pnt2d IP2;
        gp_Pnt2d IP3;
        gp_Pnt2d IP4;
        gp_Pnt2d IP5;
        gp_Pnt2d IP6;
        gp_Pnt2d IP7;
        gp_Pnt2d IP8;
        gp_Pnt2d IP9;
        gp_Pnt2d IP10;
        gp_Pnt2d IP11;
        gp_Pnt2d IP12;

        // Assign Coord
        IP1.SetCoord(IP1x, IP1y);
        IP2.SetCoord(IP2x, IP2y);
        IP3.SetCoord(IP3x, IP3y);
        IP4.SetCoord(IP4x, IP4y);
        IP5.SetCoord(IP5x, IP5y);
        IP6.SetCoord(IP6x, IP6y);
        IP7.SetCoord(IP7x, IP7y);
        IP8.SetCoord(IP8x, IP8y);
        IP9.SetCoord(IP9x, IP9y);
        IP10.SetCoord(IP10x, IP10y);
        IP11.SetCoord(IP11x, IP11y);
        IP12.SetCoord(IP12x, IP12y);

        //cout << "IP1 = " << IP1.X() << endl;
        //cout << "IP6 = " << IP6.X() << endl;
        //Standard_Real a=-5, b=5, c=-2.5, e=2.5;

        TColgp_Array1OfPnt2d Bezier11(1, 4);
        Bezier11(1) = IP1;
        Bezier11(2) = IP2;
        Bezier11(3) = IP3;
        Bezier11(4) = IP4;
        Handle(Geom2d_BezierCurve) Bezier1 = new Geom2d_BezierCurve(Bezier11);

        TColgp_Array1OfPnt2d Bezier12(1, 4);
        Bezier12(1) = IP4;
        Bezier12(2) = IP5;
        Bezier12(3) = IP6;
        Bezier12(4) = IP7;
        Handle(Geom2d_BezierCurve) Bezier2 = new Geom2d_BezierCurve(Bezier12);

        TColgp_Array1OfPnt2d Bezier13(1, 4);
        Bezier13(1) = IP7;
        Bezier13(2) = IP8;
        Bezier13(3) = IP9;
        Bezier13(4) = IP10;
        Handle(Geom2d_BezierCurve) Bezier3 = new Geom2d_BezierCurve(Bezier13);

        TColgp_Array1OfPnt2d Bezier14(1, 4);
        Bezier14(1) = IP10;
        Bezier14(2) = IP11;
        Bezier14(3) = IP12;
        Bezier14(4) = IP1;
        Handle(Geom2d_BezierCurve) Bezier4 = new Geom2d_BezierCurve(Bezier14);


        // -- Create the host plane
        Pl1[i] = gp_Pln( gp_Ax3(P[i], d[i], d2[i] ));
        s[i] = new Geom_Plane (Pl1[i]);


        Edge_B11[i]   = BRepBuilderAPI_MakeEdge(Bezier1, s[i]);
        ShapeBuild_Edge().BuildCurve3d(Edge_B11[i]);
        Edge_B21[i]   = BRepBuilderAPI_MakeEdge(Bezier2, s[i]);
        ShapeBuild_Edge().BuildCurve3d(Edge_B21[i]);
        Edge_B31[i]   = BRepBuilderAPI_MakeEdge(Bezier3, s[i]);
        ShapeBuild_Edge().BuildCurve3d(Edge_B31[i]);
        Edge_B41[i]   = BRepBuilderAPI_MakeEdge(Bezier4, s[i]);
        ShapeBuild_Edge().BuildCurve3d(Edge_B41[i]);
        /*TopoDS_Edge Edge_B21[i]   = BRepBuilderAPI_MakeEdge(Bezier2, Pl[i]);
        TopoDS_Edge Edge_B31[i]   = BRepBuilderAPI_MakeEdge(Bezier3, Pl[i]);
        TopoDS_Edge Edge_B41[i]   = BRepBuilderAPI_MakeEdge(Bezier4, Pl[i]);*/





        // Create wire

        BRepBuilderAPI_MakeWire mkWire;
        mkWire.Add(Edge_B11[i]);
        mkWire.Add(Edge_B21[i]);
        mkWire.Add(Edge_B31[i]);
        mkWire.Add(Edge_B41[i]);
        Wire[i] = mkWire.Wire();




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

    }
    //
    if ( !aGenerator.IsDone() )
    {
        std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;

    }

    tube = aGenerator.Shape();

    //DBRep::Set("tube",tube);
    //return 0;


// Inlet and Outlet Pipe

    gp_Vec V0(d[0]);
    BRepPrimAPI_MakePrism mkPrismIn(Wire[0], -maxLengthVerticalPipes*V0);
    mkPrismIn.Build();
    TopoDS_Shape InletPipe = mkPrismIn.Shape();
    // Get the Inlet wire
    TopoDS_Wire wbaseIn = TopoDS::Wire(mkPrismIn.LastShape());
    // Create the Inlet face from this wire
    TopoDS_Face fbaseIn = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseIn)).Face();

    gp_Vec Vmaxslices(d[maxSlices-1]);
    BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], maxLengthVerticalPipes*Vmaxslices);
    mkPrismOut.Build();
    TopoDS_Shape OutletPipe = mkPrismOut.Shape();
    // Get the outlet wire
    TopoDS_Wire wbaseOut = TopoDS::Wire(mkPrismOut.LastShape());
    // Create the outlet face from this wire
    TopoDS_Face fbaseOut = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseOut)).Face();

    //cout << "V0 IS " << V0.Magnitude() << endl;
    //cout << "VlastSlice IS " << Vmaxslices.Magnitude() << endl;

 //   InletPipe = BRepAlgoAPI_Fuse (InletPipe,fbaseIn);
  //  OutletPipe = BRepAlgoAPI_Fuse (OutletPipe,fbaseOut);
  //  tube = BRepAlgoAPI_Fuse (tube,InletPipe);
  //  tube = BRepAlgoAPI_Fuse (tube,OutletPipe);

    BRepBuilderAPI_Sewing Sewing;
    Sewing.Add(fbaseIn);
    Sewing.Add(InletPipe);
    Sewing.Add(tube);
    Sewing.Add(OutletPipe);
    Sewing.Add(fbaseOut);
    Sewing.Perform();
    tube = Sewing.SewedShape();
    tube = TopoDS::Shell(tube);

    return tube;

}

//March 4th
TopoDS_Shape OCCTDataProvider::ConstructSquaredUbendWithPathAndLawsAsBSplines()
{
//    std::vector<Standard_Real> designParameters =
//    {
//      Standard_Real(-37.5), Standard_Real(37.5), Standard_Real(-37.5), Standard_Real(37.5), Standard_Real(-37.5), Standard_Real(37.5), Standard_Real(-37.5), Standard_Real(37.5),
//      Standard_Real(-18.75), Standard_Real(37.5), Standard_Real(-18.75), Standard_Real(37.5), Standard_Real(-18.75), Standard_Real(37.5), Standard_Real(-18.75), Standard_Real(37.5),
//      Standard_Real(18.75), Standard_Real(37.5), Standard_Real(18.75), Standard_Real(37.5), Standard_Real(18.75), Standard_Real(37.5), Standard_Real(18.75), Standard_Real(37.5),
//      Standard_Real(37.5), Standard_Real(37.5), Standard_Real(37.5), Standard_Real(37.5), Standard_Real(37.5), Standard_Real(37.5), Standard_Real(37.5), Standard_Real(37.5),
//      Standard_Real(37.5), Standard_Real(18.75), Standard_Real(37.5), Standard_Real(18.75), Standard_Real(37.5), Standard_Real(18.75), Standard_Real(37.5), Standard_Real(18.75),
//      Standard_Real(37.5), Standard_Real(-18.75), Standard_Real(37.5), Standard_Real(-18.75), Standard_Real(37.5), Standard_Real(-18.75), Standard_Real(37.5), Standard_Real(-18.75),
//      Standard_Real(37.5), Standard_Real(-37.5), Standard_Real(37.5), Standard_Real(-37.5), Standard_Real(37.5), Standard_Real(-37.5), Standard_Real(37.5), Standard_Real(-37.5),
//      Standard_Real(18.75), Standard_Real(-37.5), Standard_Real(18.75), Standard_Real(-37.5), Standard_Real(18.75), Standard_Real(-37.5), Standard_Real(18.75), Standard_Real(-37.5),
//      Standard_Real(-18.75), Standard_Real(-37.5), Standard_Real(-18.75), Standard_Real(-37.5), Standard_Real(-18.75), Standard_Real(-37.5), Standard_Real(-18.75), Standard_Real(-37.5),
//      Standard_Real(-37.5), Standard_Real(-37.5), Standard_Real(-37.5), Standard_Real(-37.5), Standard_Real(-37.5), Standard_Real(-37.5), Standard_Real(-37.5), Standard_Real(-37.5),
//      Standard_Real(-37.5), Standard_Real(-18.75), Standard_Real(-37.5), Standard_Real(-18.75), Standard_Real(-37.5), Standard_Real(-18.75), Standard_Real(-37.5), Standard_Real(-18.75),
//      Standard_Real(-37.5), Standard_Real(18.75), Standard_Real(-37.5), Standard_Real(18.75), Standard_Real(-37.5), Standard_Real(18.75), Standard_Real(-37.5), Standard_Real(18.75)
//    };
    //---------------------------------------------------------------------------
    // Stage 1: Construct laws from design parameters
    //---------------------------------------------------------------------------

    const Standard_Integer maxSlices = 35;
    const Standard_Real maxLengthVerticalPipes = 600.;

    //Knots, Multipl and Degree of the laws

    TColStd_Array1OfReal lawKnots(1,6);
    lawKnots(1)=0.;
    lawKnots(2)=0.2;
    lawKnots(3)=0.4;
    lawKnots(4)=0.6;
    lawKnots(5)=0.8;
    lawKnots(6)=1;

    TColStd_Array1OfInteger lawMults(1,6);
    lawMults(1)=4;
    lawMults(2)=1;
    lawMults(3)=1;
    lawMults(4)=1;
    lawMults(5)=1;
    lawMults(6)=4;

    Standard_Integer lawDegree = 3;

    //CP1
    TColgp_Array1OfPnt lawCP1(1, 8);
    lawCP1(1) = gp_Pnt(-37.5, 37.5, 0.); //CP1_P1;
    lawCP1(2) = gp_Pnt(-37.5, 37.5, 0.15); //CP1_P2;
    lawCP1(3) = gp_Pnt(designParameters[0], designParameters[1], 0.30); //CP1_P3;
    lawCP1(4) = gp_Pnt(designParameters[2], designParameters[3], 0.45); //CP1_P4;
    lawCP1(5) = gp_Pnt(designParameters[4], designParameters[5], 0.60); //CP1_P5;
    lawCP1(6) = gp_Pnt(designParameters[6], designParameters[7], 0.75); //CP1_P6;
    lawCP1(7) = gp_Pnt(-37.5, 37.5, 0.85); //CP1_P7;
    lawCP1(8) = gp_Pnt(-37.5, 37.5, 1); //CP1_P8;
    Handle(Geom_BSplineCurve) BSplineCP1 = new Geom_BSplineCurve(lawCP1, lawKnots, lawMults, lawDegree, Standard_False);

    //CP2
    TColgp_Array1OfPnt lawCP2(1, 8);
    lawCP2(1) = gp_Pnt(-18.75, 37.5, 0.); //CP2_P1;
    lawCP2(2) = gp_Pnt(-18.75, 37.5, 0.15); //CP2_P2;
    lawCP2(3) = gp_Pnt(designParameters[8], designParameters[9], 0.30); //CP2_P3;
    lawCP2(4) = gp_Pnt(designParameters[10], designParameters[11], 0.45); //CP2_P4;
    lawCP2(5) = gp_Pnt(designParameters[12], designParameters[13], 0.60); //CP2_P5;
    lawCP2(6) = gp_Pnt(designParameters[14], designParameters[15], 0.75); //CP2_P6;
    lawCP2(7) = gp_Pnt(-18.75, 37.5, 0.85); //CP2_P7;
    lawCP2(8) = gp_Pnt(-18.75, 37.5, 1.); //CP2_P8;
    Handle(Geom_BSplineCurve) BSplineCP2 = new Geom_BSplineCurve(lawCP2, lawKnots, lawMults, lawDegree, Standard_False);

    //CP3
    TColgp_Array1OfPnt lawCP3(1, 8);
    lawCP3(1) = gp_Pnt(18.75, 37.5, 0.); //CP3_P1;
    lawCP3(2) = gp_Pnt(18.75, 37.5, 0.15); //CP3_P2;
    lawCP3(3) = gp_Pnt(designParameters[16], designParameters[17], 0.30); //CP3_P3;
    lawCP3(4) = gp_Pnt(designParameters[18], designParameters[19], 0.45); //CP3_P4;
    lawCP3(5) = gp_Pnt(designParameters[20], designParameters[21], 0.60); //CP3_P5;
    lawCP3(6) = gp_Pnt(designParameters[22], designParameters[23], 0.75); //CP3_P6;
    lawCP3(7) = gp_Pnt(18.75, 37.5, 0.85); //CP3_P7;
    lawCP3(8) = gp_Pnt(18.75, 37.5, 1.); //CP3_P8;
    Handle(Geom_BSplineCurve) BSplineCP3 = new Geom_BSplineCurve(lawCP3, lawKnots, lawMults, lawDegree, Standard_False);

    //CP4
    TColgp_Array1OfPnt lawCP4(1, 8);
    lawCP4(1) = gp_Pnt(37.5, 37.5, 0.); //CP4_P1;
    lawCP4(2) = gp_Pnt(37.5, 37.5, 0.15); //CP4_P2;
    lawCP4(3) = gp_Pnt(designParameters[24], designParameters[25], 0.30); //CP4_P3;
    lawCP4(4) = gp_Pnt(designParameters[26], designParameters[27], 0.45); //CP4_P4;
    lawCP4(5) = gp_Pnt(designParameters[28], designParameters[29], 0.60); //CP4_P5;
    lawCP4(6) = gp_Pnt(designParameters[30], designParameters[31], 0.75); //CP4_P6;
    lawCP4(7) = gp_Pnt(37.5, 37.5, 0.85); //CP4_P7;
    lawCP4(8) = gp_Pnt(37.5, 37.5, 1.); //CP4_P8;
    Handle(Geom_BSplineCurve) BSplineCP4 = new Geom_BSplineCurve(lawCP4, lawKnots, lawMults, lawDegree, Standard_False);

    //CP5
    TColgp_Array1OfPnt lawCP5(1, 8);
    lawCP5(1) = gp_Pnt(37.5, 18.75, 0.); //CP5_P1;
    lawCP5(2) = gp_Pnt(37.5, 18.75, 0.15); //CP5_P2;
    lawCP5(3) = gp_Pnt(designParameters[32], designParameters[33], 0.30); //CP5_P3;
    lawCP5(4) = gp_Pnt(designParameters[34], designParameters[35], 0.45); //CP5_P4;
    lawCP5(5) = gp_Pnt(designParameters[36], designParameters[37], 0.60); //CP5_P5;
    lawCP5(6) = gp_Pnt(designParameters[38], designParameters[39], 0.75); //CP5_P6;
    lawCP5(7) = gp_Pnt(37.5, 18.75, 0.85); //CP5_P7;
    lawCP5(8) = gp_Pnt(37.5, 18.75, 1.); //CP5_P8;
    Handle(Geom_BSplineCurve) BSplineCP5 = new Geom_BSplineCurve(lawCP5, lawKnots, lawMults, lawDegree, Standard_False);

    //CP6
    TColgp_Array1OfPnt lawCP6(1, 8);
    lawCP6(1) = gp_Pnt(37.5,-18.75, 0.); //CP6_P1;
    lawCP6(2) = gp_Pnt(37.5,-18.75, 0.15); //CP6_P2;
    lawCP6(3) = gp_Pnt(designParameters[40], designParameters[41], 0.30); //CP6_P3;
    lawCP6(4) = gp_Pnt(designParameters[42], designParameters[43], 0.45); //CP6_P4;
    lawCP6(5) = gp_Pnt(designParameters[44], designParameters[45], 0.60); //CP6_P5;
    lawCP6(6) = gp_Pnt(designParameters[46], designParameters[47], 0.75); //CP6_P6;
    lawCP6(7) = gp_Pnt(37.5,-18.75, 0.85); //CP6_P7;
    lawCP6(8) = gp_Pnt(37.5,-18.75, 1.); //CP6_P8;
    Handle(Geom_BSplineCurve) BSplineCP6 = new Geom_BSplineCurve(lawCP6, lawKnots, lawMults, lawDegree, Standard_False);

    //CP7
    TColgp_Array1OfPnt lawCP7(1, 8);
    lawCP7(1) = gp_Pnt(37.5, -37.5, 0.); //CP7_P1;
    lawCP7(2) = gp_Pnt(37.5, -37.5, 0.15); //CP7_P2;
    lawCP7(3) = gp_Pnt(designParameters[48], designParameters[49], 0.30); //CP7_P3;
    lawCP7(4) = gp_Pnt(designParameters[50], designParameters[51], 0.45); //CP7_P4;
    lawCP7(5) = gp_Pnt(designParameters[52], designParameters[53], 0.60); //CP7_P5;
    lawCP7(6) = gp_Pnt(designParameters[54], designParameters[55], 0.75); //CP7_P6;
    lawCP7(7) = gp_Pnt(37.5, -37.5, 0.85); //CP7_P7;
    lawCP7(8) = gp_Pnt(37.5, -37.5, 1.); //CP7_P8;
    Handle(Geom_BSplineCurve) BSplineCP7 = new Geom_BSplineCurve(lawCP7, lawKnots, lawMults, lawDegree, Standard_False);

    //CP8
    TColgp_Array1OfPnt lawCP8(1, 8);
    lawCP8(1) = gp_Pnt(18.75, -37.5, 0.); //CP8_P1;
    lawCP8(2) = gp_Pnt(18.75, -37.5, 0.15); //CP8_P2;
    lawCP8(3) = gp_Pnt(designParameters[56], designParameters[57], 0.30); //CP8_P3;
    lawCP8(4) = gp_Pnt(designParameters[58], designParameters[59], 0.45); //CP8_P4;
    lawCP8(5) = gp_Pnt(designParameters[60], designParameters[61], 0.60); //CP8_P5;
    lawCP8(6) = gp_Pnt(designParameters[62], designParameters[63], 0.75); //CP8_P6;
    lawCP8(7) = gp_Pnt(18.75, -37.5, 0.85); //CP8_P7;
    lawCP8(8) = gp_Pnt(18.75, -37.5, 1); //CP8_P8;
    Handle(Geom_BSplineCurve) BSplineCP8 = new Geom_BSplineCurve(lawCP8, lawKnots, lawMults, lawDegree, Standard_False);

    //CP9
    TColgp_Array1OfPnt lawCP9(1, 8);
    lawCP9(1) = gp_Pnt(-18.75,-37.5, 0.); //CP9_P1;
    lawCP9(2) = gp_Pnt(-18.75, -37.5, 0.15); //CP9_P2;
    lawCP9(3) = gp_Pnt(designParameters[64], designParameters[65], 0.30); //CP9_P3;
    lawCP9(4) = gp_Pnt(designParameters[66], designParameters[67], 0.45); //CP9_P4;
    lawCP9(5) = gp_Pnt(designParameters[68], designParameters[69], 0.60); //CP9_P5;
    lawCP9(6) = gp_Pnt(designParameters[70], designParameters[71], 0.75); //CP9_P6;
    lawCP9(7) = gp_Pnt(-18.75, -37.5, 0.85); //CP9_P7;
    lawCP9(8) = gp_Pnt(-18.75, -37.5, 1.); //CP9_P8;
    Handle(Geom_BSplineCurve) BSplineCP9 = new Geom_BSplineCurve(lawCP9, lawKnots, lawMults, lawDegree, Standard_False);

    //CP10
    TColgp_Array1OfPnt lawCP10(1, 8);
    lawCP10(1) = gp_Pnt(-37.5, -37.5, 0.); //CP10_P1;
    lawCP10(2) = gp_Pnt(-37.5, -37.5, 0.15); //CP10_P2;
    lawCP10(3) = gp_Pnt(designParameters[72], designParameters[73], 0.30); //CP10_P3;
    lawCP10(4) = gp_Pnt(designParameters[74], designParameters[75], 0.45); //CP10_P4;
    lawCP10(5) = gp_Pnt(designParameters[76], designParameters[77], 0.60); //CP10_P5;
    lawCP10(6) = gp_Pnt(designParameters[78], designParameters[79], 0.75); //CP10_P6;
    lawCP10(7) = gp_Pnt(-37.5, -37.5, 0.85); //CP10_P7;
    lawCP10(8) = gp_Pnt(-37.5, -37.5, 1.); //CP10_P8;
    Handle(Geom_BSplineCurve) BSplineCP10 = new Geom_BSplineCurve(lawCP10, lawKnots, lawMults, lawDegree, Standard_False);

    //CP11
    TColgp_Array1OfPnt lawCP11(1, 8);
    lawCP11(1) = gp_Pnt(-37.5, -18.75, 0.); //CP11_P1;
    lawCP11(2) = gp_Pnt(-37.5, -18.75, 0.15); //CP11_P2;
    lawCP11(3) = gp_Pnt(designParameters[80], designParameters[81], 0.30); //CP11_P3;
    lawCP11(4) = gp_Pnt(designParameters[82], designParameters[83], 0.45); //CP11_P4;
    lawCP11(5) = gp_Pnt(designParameters[84], designParameters[85], 0.60); //CP11_P5;
    lawCP11(6) = gp_Pnt(designParameters[86], designParameters[87], 0.75); //CP11_P6;
    lawCP11(7) = gp_Pnt(-37.5, -18.75, 0.85); //CP11_P7;
    lawCP11(8) = gp_Pnt(-37.5, -18.75, 1.); //CP11_P8;
    Handle(Geom_BSplineCurve) BSplineCP11 = new Geom_BSplineCurve(lawCP11, lawKnots, lawMults, lawDegree, Standard_False);

    //CP12
    TColgp_Array1OfPnt lawCP12(1, 8);
    lawCP12(1) = gp_Pnt(-37.5, 18.75, 0.); //CP12_P1;
    lawCP12(2) = gp_Pnt(-37.5, 18.75, 0.15); //CP12_P2;
    lawCP12(3) = gp_Pnt(designParameters[88], designParameters[89], 0.30); //CP12_P3;
    lawCP12(4) = gp_Pnt(designParameters[90], designParameters[91], 0.45); //CP12_P4;
    lawCP12(5) = gp_Pnt(designParameters[92], designParameters[93], 0.60); //CP12_P5;
    lawCP12(6) = gp_Pnt(designParameters[94], designParameters[95], 0.75); //CP12_P6;
    lawCP12(7) = gp_Pnt(-37.5, 18.75, 0.85); //CP12_P7;
    lawCP12(8) = gp_Pnt(-37.5, 18.75, 1.); //CP12_P8;
    Handle(Geom_BSplineCurve) BSplineCP12 = new Geom_BSplineCurve(lawCP12, lawKnots, lawMults, lawDegree, Standard_False);

    //---------------------------------------------------------------------------
    // Stage 2: U-bend: B-Spline path for U-bend
    //---------------------------------------------------------------------------

    TColgp_Array1OfPnt pathPoles(1, 9);
    pathPoles(1) = gp_Pnt(57., -150.,   0.);
    pathPoles(2) = gp_Pnt(57., -75.,   0.);
    pathPoles(3) = gp_Pnt(57., 0.,  0.);
    pathPoles(4) = gp_Pnt(57., 57., 0.);
    pathPoles(5) = gp_Pnt(3.49024337756996e-15, 57., 0.);
    pathPoles(6) = gp_Pnt(-57., 57., 0.);
    pathPoles(7) = gp_Pnt(-57., 0.,   0.);
    pathPoles(8) = gp_Pnt(-57., -75.,   0.);
    pathPoles(9) = gp_Pnt(-57., -150.,   0.);

    TColStd_Array1OfReal pathWeights(1,9);
    pathWeights(1)=1;
    pathWeights(2)=1;
    pathWeights(3)=1;
    pathWeights(4)=0.707106781186548;
    pathWeights(5)=1;
    pathWeights(6)=0.707106781186548;
    pathWeights(7)=1;
    pathWeights(8)=1;
    pathWeights(9)=1;

    TColStd_Array1OfReal pathKnots(1,5);
    pathKnots(1)=0;
    pathKnots(2)=0.25;
    pathKnots(3)=0.5;
    pathKnots(4)=0.75;
    pathKnots(5)=1;

    TColStd_Array1OfInteger pathMults(1,5);
    pathMults(1)=3;
    pathMults(2)=2;
    pathMults(3)=2;
    pathMults(4)=2;
    pathMults(5)=3;

    Standard_Integer pathDegree = 2;

    Handle(Geom_BSplineCurve) path = new Geom_BSplineCurve(pathPoles, pathWeights, pathKnots, pathMults, pathDegree, Standard_False, Standard_True);

    //DrawTrSurf::Set("path", path);

    GeomAdaptor_Curve GAC;
    GAC.Load(path);
    Standard_Real Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve

    //---------------------------------------------------------------------------
    // Stage 3: U-bend: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    TopoDS_Edge Edge_B11;
    TopoDS_Edge Edge_B21;
    TopoDS_Edge Edge_B31;
    TopoDS_Edge Edge_B41;
    TopoDS_Shape tube;
    gp_Dir d[maxSlices];
    gp_Dir d2(0,0,1);
    TopoDS_Wire Wire[maxSlices];

    Standard_Real u[maxSlices];

    u[0]             = path->FirstParameter();
    u[maxSlices - 1] = path->LastParameter();

    const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);

    for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
    {
      u[i] = u[0] + i*ustep;
    }

    // Prepare skinning tool

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;
    const Standard_Real    pres3d  = 1.0e-02;

    BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);
    aGenerator.CheckCompatibility(Standard_False);
    //
    for ( Standard_Integer i = 0; i < maxSlices; ++i )
    {

      gp_Pnt P;
      gp_Vec V;
      path->D1(u[i], P, V);
      d[i] = gp_Dir(V);

      Standard_Real P_Length = GCPnts_AbscissaPoint::Length(GAC, u[0], u[i]);//length of the curve

      Standard_Real Length_step = (P_Length/Length);

      // -- Create the intersection plane
      gp_Pnt P_path(0, 0, Length_step);

      gp_Pln constructionPl(P_path, d2);
      Handle(Geom_Plane) constructionPlane = new Geom_Plane (constructionPl);

      //IP1
      GeomAPI_IntCS IntCS1(BSplineCP1, constructionPlane);
      //cout << "Nb of Int1 points is " << IntCS1.NbPoints() << endl;
      //IP2
      GeomAPI_IntCS IntCS2(BSplineCP2, constructionPlane);
      //IP3
      GeomAPI_IntCS IntCS3(BSplineCP3, constructionPlane);
      //IP4
      GeomAPI_IntCS IntCS4(BSplineCP4, constructionPlane);
      //IP5
      GeomAPI_IntCS IntCS5(BSplineCP5, constructionPlane);
      //IP6
      GeomAPI_IntCS IntCS6(BSplineCP6, constructionPlane);
      //IP7
      GeomAPI_IntCS IntCS7(BSplineCP7, constructionPlane);
      //IP8
      GeomAPI_IntCS IntCS8(BSplineCP8, constructionPlane);
      //IP9
      GeomAPI_IntCS IntCS9(BSplineCP9, constructionPlane);
      //IP10
      GeomAPI_IntCS IntCS10(BSplineCP10, constructionPlane);
      //IP11
      GeomAPI_IntCS IntCS11(BSplineCP11, constructionPlane);
      //IP12
      GeomAPI_IntCS IntCS12(BSplineCP12, constructionPlane);

      TColgp_Array1OfPnt2d Bezier11(1, 4);
      Bezier11(1) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
      Bezier11(2) = gp_Pnt2d(IntCS2.Point(1).X(), IntCS2.Point(1).Y());
      Bezier11(3) = gp_Pnt2d(IntCS3.Point(1).X(), IntCS3.Point(1).Y());
      Bezier11(4) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
      Handle(Geom2d_BezierCurve) Bezier1 = new Geom2d_BezierCurve(Bezier11);

      TColgp_Array1OfPnt2d Bezier12(1, 4);
      Bezier12(1) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
      Bezier12(2) = gp_Pnt2d(IntCS5.Point(1).X(), IntCS5.Point(1).Y());
      Bezier12(3) = gp_Pnt2d(IntCS6.Point(1).X(), IntCS6.Point(1).Y());
      Bezier12(4) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
      Handle(Geom2d_BezierCurve) Bezier2 = new Geom2d_BezierCurve(Bezier12);

      TColgp_Array1OfPnt2d Bezier13(1, 4);
      Bezier13(1) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
      Bezier13(2) = gp_Pnt2d(IntCS8.Point(1).X(), IntCS8.Point(1).Y());
      Bezier13(3) = gp_Pnt2d(IntCS9.Point(1).X(), IntCS9.Point(1).Y());
      Bezier13(4) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
      Handle(Geom2d_BezierCurve) Bezier3 = new Geom2d_BezierCurve(Bezier13);

      TColgp_Array1OfPnt2d Bezier14(1, 4);
      Bezier14(1) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
      Bezier14(2) = gp_Pnt2d(IntCS11.Point(1).X(), IntCS11.Point(1).Y());
      Bezier14(3) = gp_Pnt2d(IntCS12.Point(1).X(), IntCS12.Point(1).Y());
      Bezier14(4) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
      Handle(Geom2d_BezierCurve) Bezier4 = new Geom2d_BezierCurve(Bezier14);

      // -- Create the host plane
      gp_Pln Pln(gp_Ax3(P, d[i], d2 ));
      //Handle(Geom_Plane) s = new Geom_Plane (Pl1);

      Edge_B11 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier1, Pln) );
      Edge_B21 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier2, Pln) );
      Edge_B31 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier3, Pln) );
      Edge_B41 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier4, Pln) );

      // Create wire

      BRepBuilderAPI_MakeWire mkWire;
      mkWire.Add(Edge_B11);
      mkWire.Add(Edge_B21);
      mkWire.Add(Edge_B31);
      mkWire.Add(Edge_B41);
      Wire[i] = mkWire.Wire();


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
    }
    //
    if ( !aGenerator.IsDone() )
    {
      std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
    }

    tube = aGenerator.Shape();

    //---------------------------------------------------------------------------
    // Stage 4 - // Inlet and Outlet Pipe
    //---------------------------------------------------------------------------

    gp_Vec V0(d[0]);
    BRepPrimAPI_MakePrism mkPrismIn(Wire[0], -maxLengthVerticalPipes*V0);
    mkPrismIn.Build();
    TopoDS_Shape InletPipe = mkPrismIn.Shape();
    // Get the Inlet wire
    TopoDS_Wire wbaseIn = TopoDS::Wire(mkPrismIn.LastShape());
    // Create the Inlet face from this wire
    TopoDS_Face fbaseIn = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseIn)).Face();

    gp_Vec Vmaxslices(d[maxSlices-1]);
    BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], 0.6438*maxLengthVerticalPipes*Vmaxslices);
    mkPrismOut.Build();
    TopoDS_Shape OutletPipe = mkPrismOut.Shape();
    // Get the outlet wire
    TopoDS_Wire wbaseOut = TopoDS::Wire(mkPrismOut.LastShape());
    // Create the outlet face from this wire
    TopoDS_Face fbaseOut = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseOut)).Face();

    //---------------------------------------------------------------------------
    // Stage 5 - // Build the shell or solid
    //---------------------------------------------------------------------------

    BRepBuilderAPI_Sewing Sewing;
    Sewing.Add(fbaseIn);
    Sewing.Add(InletPipe);
    Sewing.Add(tube);
    Sewing.Add(OutletPipe);
    Sewing.Add(fbaseOut);
    Sewing.Perform();
    tube = Sewing.SewedShape();
    tube = TopoDS::Shell(tube);
  //  TopoDS_Solid solidTube;
  //  BRep_Builder mkSolid;
  //  mkSolid.MakeSolid(solidTube);
  //  mkSolid.Add(solidTube, tube);

    return tube;
}

//March 8th
TopoDS_Shape OCCTDataProvider::ConstructSquaredUbendWithRealDimensions()
{
    int maxSlices = 100;
    // double maxLengthVerticalPipes = 2.3475;
    // double maxLengthVerticalPipes = 0.75;
    double maxLengthVerticalPipes = 0.67755;

//    std::vector<Standard_Real> designParameters =
//            {
//                    Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375),
//                    Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375),
//                    Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375),
//                    Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375),
//                    Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875),
//                    Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875),
//                    Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375),
//                    Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375),
//                    Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375),
//                    Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375),
//                    Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875),
//                    Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875)
//            };
//#define maxNbDPcp 4 //number of design parameters
    //---------------------------------------------------------------------------
    // Stage 1: Construct laws from design parameters
    //---------------------------------------------------------------------------

    //Knots, Multipl and Degree of the laws

    TColStd_Array1OfReal lawKnots(1,6);
    lawKnots(1)=0.;
    lawKnots(2)=0.2;
    lawKnots(3)=0.4;
    lawKnots(4)=0.6;
    lawKnots(5)=0.8;
    lawKnots(6)=1;

    TColStd_Array1OfInteger lawMults(1,6);
    lawMults(1)=4;
    lawMults(2)=1;
    lawMults(3)=1;
    lawMults(4)=1;
    lawMults(5)=1;
    lawMults(6)=4;

    Standard_Integer lawDegree = 3;

    //CP1
    TColgp_Array1OfPnt lawCP1(1, 8);
    lawCP1(1) = gp_Pnt(-0.0375, 0.0375, 0.); //CP1_P1;
    lawCP1(2) = gp_Pnt(-0.0375, 0.0375, 0.15); //CP1_P2;
    lawCP1(3) = gp_Pnt(designParameters[0], designParameters[1], 0.30); //CP1_P3;
    lawCP1(4) = gp_Pnt(designParameters[2], designParameters[3], 0.45); //CP1_P4;
    lawCP1(5) = gp_Pnt(designParameters[4], designParameters[5], 0.60); //CP1_P5;
    lawCP1(6) = gp_Pnt(designParameters[6], designParameters[7], 0.75); //CP1_P6;
    lawCP1(7) = gp_Pnt(-0.0375, 0.0375, 0.85); //CP1_P7;
    lawCP1(8) = gp_Pnt(-0.0375, 0.0375, 1); //CP1_P8;
    Handle(Geom_BSplineCurve) BSplineCP1 = new Geom_BSplineCurve(lawCP1, lawKnots, lawMults, lawDegree, Standard_False);

    //CP2
    TColgp_Array1OfPnt lawCP2(1, 8);
    lawCP2(1) = gp_Pnt(-0.01875, 0.0375, 0.); //CP2_P1;
    lawCP2(2) = gp_Pnt(-0.01875, 0.0375, 0.15); //CP2_P2;
    lawCP2(3) = gp_Pnt(designParameters[8], designParameters[9], 0.30); //CP2_P3;
    lawCP2(4) = gp_Pnt(designParameters[10], designParameters[11], 0.45); //CP2_P4;
    lawCP2(5) = gp_Pnt(designParameters[12], designParameters[13], 0.60); //CP2_P5;
    lawCP2(6) = gp_Pnt(designParameters[14], designParameters[15], 0.75); //CP2_P6;
    lawCP2(7) = gp_Pnt(-0.01875, 0.0375, 0.85); //CP2_P7;
    lawCP2(8) = gp_Pnt(-0.01875, 0.0375, 1.); //CP2_P8;
    Handle(Geom_BSplineCurve) BSplineCP2 = new Geom_BSplineCurve(lawCP2, lawKnots, lawMults, lawDegree, Standard_False);

    //CP3
    TColgp_Array1OfPnt lawCP3(1, 8);
    lawCP3(1) = gp_Pnt(0.01875, 0.0375, 0.); //CP3_P1;
    lawCP3(2) = gp_Pnt(0.01875, 0.0375, 0.15); //CP3_P2;
    lawCP3(3) = gp_Pnt(designParameters[16], designParameters[17], 0.30); //CP3_P3;
    lawCP3(4) = gp_Pnt(designParameters[18], designParameters[19], 0.45); //CP3_P4;
    lawCP3(5) = gp_Pnt(designParameters[20], designParameters[21], 0.60); //CP3_P5;
    lawCP3(6) = gp_Pnt(designParameters[22], designParameters[23], 0.75); //CP3_P6;
    lawCP3(7) = gp_Pnt(0.01875, 0.0375, 0.85); //CP3_P7;
    lawCP3(8) = gp_Pnt(0.01875, 0.0375, 1.); //CP3_P8;
    Handle(Geom_BSplineCurve) BSplineCP3 = new Geom_BSplineCurve(lawCP3, lawKnots, lawMults, lawDegree, Standard_False);

    //CP4
    TColgp_Array1OfPnt lawCP4(1, 8);
    lawCP4(1) = gp_Pnt(0.0375, 0.0375, 0.); //CP4_P1;
    lawCP4(2) = gp_Pnt(0.0375, 0.0375, 0.15); //CP4_P2;
    lawCP4(3) = gp_Pnt(designParameters[24], designParameters[25], 0.30); //CP4_P3;
    lawCP4(4) = gp_Pnt(designParameters[26], designParameters[27], 0.45); //CP4_P4;
    lawCP4(5) = gp_Pnt(designParameters[28], designParameters[29], 0.60); //CP4_P5;
    lawCP4(6) = gp_Pnt(designParameters[30], designParameters[31], 0.75); //CP4_P6;
    lawCP4(7) = gp_Pnt(0.0375, 0.0375, 0.85); //CP4_P7;
    lawCP4(8) = gp_Pnt(0.0375, 0.0375, 1.); //CP4_P8;
    Handle(Geom_BSplineCurve) BSplineCP4 = new Geom_BSplineCurve(lawCP4, lawKnots, lawMults, lawDegree, Standard_False);

    //CP5
    TColgp_Array1OfPnt lawCP5(1, 8);
    lawCP5(1) = gp_Pnt(0.0375, 0.01875, 0.); //CP5_P1;
    lawCP5(2) = gp_Pnt(0.0375, 0.01875, 0.15); //CP5_P2;
    lawCP5(3) = gp_Pnt(designParameters[32], designParameters[33], 0.30); //CP5_P3;
    lawCP5(4) = gp_Pnt(designParameters[34], designParameters[35], 0.45); //CP5_P4;
    lawCP5(5) = gp_Pnt(designParameters[36], designParameters[37], 0.60); //CP5_P5;
    lawCP5(6) = gp_Pnt(designParameters[38], designParameters[39], 0.75); //CP5_P6;
    lawCP5(7) = gp_Pnt(0.0375, 0.01875, 0.85); //CP5_P7;
    lawCP5(8) = gp_Pnt(0.0375, 0.01875, 1.); //CP5_P8;
    Handle(Geom_BSplineCurve) BSplineCP5 = new Geom_BSplineCurve(lawCP5, lawKnots, lawMults, lawDegree, Standard_False);

    //CP6
    TColgp_Array1OfPnt lawCP6(1, 8);
    lawCP6(1) = gp_Pnt(0.0375,-0.01875, 0.); //CP6_P1;
    lawCP6(2) = gp_Pnt(0.0375,-0.01875, 0.15); //CP6_P2;
    lawCP6(3) = gp_Pnt(designParameters[40], designParameters[41], 0.30); //CP6_P3;
    lawCP6(4) = gp_Pnt(designParameters[42], designParameters[43], 0.45); //CP6_P4;
    lawCP6(5) = gp_Pnt(designParameters[44], designParameters[45], 0.60); //CP6_P5;
    lawCP6(6) = gp_Pnt(designParameters[46], designParameters[47], 0.75); //CP6_P6;
    lawCP6(7) = gp_Pnt(0.0375,-0.01875, 0.85); //CP6_P7;
    lawCP6(8) = gp_Pnt(0.0375,-0.01875, 1.); //CP6_P8;
    Handle(Geom_BSplineCurve) BSplineCP6 = new Geom_BSplineCurve(lawCP6, lawKnots, lawMults, lawDegree, Standard_False);

    //CP7
    TColgp_Array1OfPnt lawCP7(1, 8);
    lawCP7(1) = gp_Pnt(0.0375, -0.0375, 0.); //CP7_P1;
    lawCP7(2) = gp_Pnt(0.0375, -0.0375, 0.15); //CP7_P2;
    lawCP7(3) = gp_Pnt(designParameters[48], designParameters[49], 0.30); //CP7_P3;
    lawCP7(4) = gp_Pnt(designParameters[50], designParameters[51], 0.45); //CP7_P4;
    lawCP7(5) = gp_Pnt(designParameters[52], designParameters[53], 0.60); //CP7_P5;
    lawCP7(6) = gp_Pnt(designParameters[54], designParameters[55], 0.75); //CP7_P6;
    lawCP7(7) = gp_Pnt(0.0375, -0.0375, 0.85); //CP7_P7;
    lawCP7(8) = gp_Pnt(0.0375, -0.0375, 1.); //CP7_P8;
    Handle(Geom_BSplineCurve) BSplineCP7 = new Geom_BSplineCurve(lawCP7, lawKnots, lawMults, lawDegree, Standard_False);

    //CP8
    TColgp_Array1OfPnt lawCP8(1, 8);
    lawCP8(1) = gp_Pnt(0.01875, -0.0375, 0.); //CP8_P1;
    lawCP8(2) = gp_Pnt(0.01875, -0.0375, 0.15); //CP8_P2;
    lawCP8(3) = gp_Pnt(designParameters[56], designParameters[57], 0.30); //CP8_P3;
    lawCP8(4) = gp_Pnt(designParameters[58], designParameters[59], 0.45); //CP8_P4;
    lawCP8(5) = gp_Pnt(designParameters[60], designParameters[61], 0.60); //CP8_P5;
    lawCP8(6) = gp_Pnt(designParameters[62], designParameters[63], 0.75); //CP8_P6;
    lawCP8(7) = gp_Pnt(0.01875, -0.0375, 0.85); //CP8_P7;
    lawCP8(8) = gp_Pnt(0.01875, -0.0375, 1); //CP8_P8;
    Handle(Geom_BSplineCurve) BSplineCP8 = new Geom_BSplineCurve(lawCP8, lawKnots, lawMults, lawDegree, Standard_False);

    //CP9
    TColgp_Array1OfPnt lawCP9(1, 8);
    lawCP9(1) = gp_Pnt(-0.01875,-0.0375, 0.); //CP9_P1;
    lawCP9(2) = gp_Pnt(-0.01875, -0.0375, 0.15); //CP9_P2;
    lawCP9(3) = gp_Pnt(designParameters[64], designParameters[65], 0.30); //CP9_P3;
    lawCP9(4) = gp_Pnt(designParameters[66], designParameters[67], 0.45); //CP9_P4;
    lawCP9(5) = gp_Pnt(designParameters[68], designParameters[69], 0.60); //CP9_P5;
    lawCP9(6) = gp_Pnt(designParameters[70], designParameters[71], 0.75); //CP9_P6;
    lawCP9(7) = gp_Pnt(-0.01875, -0.0375, 0.85); //CP9_P7;
    lawCP9(8) = gp_Pnt(-0.01875, -0.0375, 1.); //CP9_P8;
    Handle(Geom_BSplineCurve) BSplineCP9 = new Geom_BSplineCurve(lawCP9, lawKnots, lawMults, lawDegree, Standard_False);

    //CP10
    TColgp_Array1OfPnt lawCP10(1, 8);
    lawCP10(1) = gp_Pnt(-0.0375, -0.0375, 0.); //CP10_P1;
    lawCP10(2) = gp_Pnt(-0.0375, -0.0375, 0.15); //CP10_P2;
    lawCP10(3) = gp_Pnt(designParameters[72], designParameters[73], 0.30); //CP10_P3;
    lawCP10(4) = gp_Pnt(designParameters[74], designParameters[75], 0.45); //CP10_P4;
    lawCP10(5) = gp_Pnt(designParameters[76], designParameters[77], 0.60); //CP10_P5;
    lawCP10(6) = gp_Pnt(designParameters[78], designParameters[79], 0.75); //CP10_P6;
    lawCP10(7) = gp_Pnt(-0.0375, -0.0375, 0.85); //CP10_P7;
    lawCP10(8) = gp_Pnt(-0.0375, -0.0375, 1.); //CP10_P8;
    Handle(Geom_BSplineCurve) BSplineCP10 = new Geom_BSplineCurve(lawCP10, lawKnots, lawMults, lawDegree, Standard_False);

    //CP11
    TColgp_Array1OfPnt lawCP11(1, 8);
    lawCP11(1) = gp_Pnt(-0.0375, -0.01875, 0.); //CP11_P1;
    lawCP11(2) = gp_Pnt(-0.0375, -0.01875, 0.15); //CP11_P2;
    lawCP11(3) = gp_Pnt(designParameters[80], designParameters[81], 0.30); //CP11_P3;
    lawCP11(4) = gp_Pnt(designParameters[82], designParameters[83], 0.45); //CP11_P4;
    lawCP11(5) = gp_Pnt(designParameters[84], designParameters[85], 0.60); //CP11_P5;
    lawCP11(6) = gp_Pnt(designParameters[86], designParameters[87], 0.75); //CP11_P6;
    lawCP11(7) = gp_Pnt(-0.0375, -0.01875, 0.85); //CP11_P7;
    lawCP11(8) = gp_Pnt(-0.0375, -0.01875, 1.); //CP11_P8;
    Handle(Geom_BSplineCurve) BSplineCP11 = new Geom_BSplineCurve(lawCP11, lawKnots, lawMults, lawDegree, Standard_False);

    //CP12
    TColgp_Array1OfPnt lawCP12(1, 8);
    lawCP12(1) = gp_Pnt(-0.0375, 0.01875, 0.); //CP12_P1;
    lawCP12(2) = gp_Pnt(-0.0375, 0.01875, 0.15); //CP12_P2;
    lawCP12(3) = gp_Pnt(designParameters[88], designParameters[89], 0.30); //CP12_P3;
    lawCP12(4) = gp_Pnt(designParameters[90], designParameters[91], 0.45); //CP12_P4;
    lawCP12(5) = gp_Pnt(designParameters[92], designParameters[93], 0.60); //CP12_P5;
    lawCP12(6) = gp_Pnt(designParameters[94], designParameters[95], 0.75); //CP12_P6;
    lawCP12(7) = gp_Pnt(-0.0375, 0.01875, 0.85); //CP12_P7;
    lawCP12(8) = gp_Pnt(-0.0375, 0.01875, 1.); //CP12_P8;
    Handle(Geom_BSplineCurve) BSplineCP12 = new Geom_BSplineCurve(lawCP12, lawKnots, lawMults, lawDegree, Standard_False);

    //---------------------------------------------------------------------------
    // Stage 2: U-bend: B-Spline path for U-bend
    //---------------------------------------------------------------------------

    TColgp_Array1OfPnt pathPoles(1, 9);
    pathPoles(1) = gp_Pnt(-0.150,  0.057,   0.);
    pathPoles(2) = gp_Pnt(-0.075, 0.057,   0.);
    pathPoles(3) = gp_Pnt(0., 0.057,  0.);
    pathPoles(4) = gp_Pnt(0.057, 0.057, 0.);
    pathPoles(5) = gp_Pnt(0.057, 3.49024337756996e-15, 0.);
    pathPoles(6) = gp_Pnt(0.057, -0.057, 0.);
    pathPoles(7) = gp_Pnt(0. , -0.057,   0.);
    pathPoles(8) = gp_Pnt(-0.075, -0.057,   0.);
    pathPoles(9) = gp_Pnt(-0.150, -0.057,   0.);

    TColStd_Array1OfReal pathWeights(1,9);
    pathWeights(1)=1;
    pathWeights(2)=1;
    pathWeights(3)=1;
    pathWeights(4)=0.707106781186548;
    pathWeights(5)=1;
    pathWeights(6)=0.707106781186548;
    pathWeights(7)=1;
    pathWeights(8)=1;
    pathWeights(9)=1;

    TColStd_Array1OfReal pathKnots(1,5);
    pathKnots(1)=0;
    pathKnots(2)=0.25;
    pathKnots(3)=0.5;
    pathKnots(4)=0.75;
    pathKnots(5)=1;

    TColStd_Array1OfInteger pathMults(1,5);
    pathMults(1)=3;
    pathMults(2)=2;
    pathMults(3)=2;
    pathMults(4)=2;
    pathMults(5)=3;

    Standard_Integer pathDegree = 2;

    Handle(Geom_BSplineCurve) path = new Geom_BSplineCurve(pathPoles, pathWeights, pathKnots, pathMults, pathDegree, Standard_False, Standard_True);

    //DrawTrSurf::Set("path", path);

    GeomAdaptor_Curve GAC;
    GAC.Load(path);
    Standard_Real Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve

    //---------------------------------------------------------------------------
    // Stage 3: U-bend: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    TopoDS_Edge Edge_B11;
    TopoDS_Edge Edge_B21;
    TopoDS_Edge Edge_B31;
    TopoDS_Edge Edge_B41;
    TopoDS_Shape tube;
    gp_Dir d[maxSlices];
    gp_Dir d2(0,0,1);
    TopoDS_Wire Wire[maxSlices];

    Standard_Real u[maxSlices];

    u[0]             = path->FirstParameter();
    u[maxSlices - 1] = path->LastParameter();

    const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);

    for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
    {
        u[i] = u[0] + i*ustep;
    }

    // Prepare skinning tool

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;
    const Standard_Real    pres3d  = 1.0e-02;

    BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);
    aGenerator.CheckCompatibility(Standard_False);
    //
    for ( Standard_Integer i = 0; i < maxSlices; ++i )
    {
        gp_Pnt P;
        gp_Vec V;
        path->D1(u[i], P, V);
        d[i] = gp_Dir(V);

        Standard_Real P_Length = GCPnts_AbscissaPoint::Length(GAC, u[0], u[i]);//length of the curve

        Standard_Real Length_step = (P_Length/Length);

        // -- Create the intersection plane
        gp_Pnt P_path(0, 0, Length_step);

        gp_Pln constructionPl(P_path, d2);
        Handle(Geom_Plane) constructionPlane = new Geom_Plane (constructionPl);

        //IP1
        GeomAPI_IntCS IntCS1(BSplineCP1, constructionPlane);
        //cout << "Nb of Int1 points is " << IntCS1.NbPoints() << endl;
        //IP2
        GeomAPI_IntCS IntCS2(BSplineCP2, constructionPlane);
        //IP3
        GeomAPI_IntCS IntCS3(BSplineCP3, constructionPlane);
        //IP4
        GeomAPI_IntCS IntCS4(BSplineCP4, constructionPlane);
        //IP5
        GeomAPI_IntCS IntCS5(BSplineCP5, constructionPlane);
        //IP6
        GeomAPI_IntCS IntCS6(BSplineCP6, constructionPlane);
        //IP7
        GeomAPI_IntCS IntCS7(BSplineCP7, constructionPlane);
        //IP8
        GeomAPI_IntCS IntCS8(BSplineCP8, constructionPlane);
        //IP9
        GeomAPI_IntCS IntCS9(BSplineCP9, constructionPlane);
        //IP10
        GeomAPI_IntCS IntCS10(BSplineCP10, constructionPlane);
        //IP11
        GeomAPI_IntCS IntCS11(BSplineCP11, constructionPlane);
        //IP12
        GeomAPI_IntCS IntCS12(BSplineCP12, constructionPlane);

        TColgp_Array1OfPnt2d Bezier11(1, 4);
        Bezier11(1) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Bezier11(2) = gp_Pnt2d(IntCS2.Point(1).X(), IntCS2.Point(1).Y());
        Bezier11(3) = gp_Pnt2d(IntCS3.Point(1).X(), IntCS3.Point(1).Y());
        Bezier11(4) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier1 = new Geom2d_BezierCurve(Bezier11);

        TColgp_Array1OfPnt2d Bezier12(1, 4);
        Bezier12(1) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Bezier12(2) = gp_Pnt2d(IntCS5.Point(1).X(), IntCS5.Point(1).Y());
        Bezier12(3) = gp_Pnt2d(IntCS6.Point(1).X(), IntCS6.Point(1).Y());
        Bezier12(4) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier2 = new Geom2d_BezierCurve(Bezier12);

        TColgp_Array1OfPnt2d Bezier13(1, 4);
        Bezier13(1) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Bezier13(2) = gp_Pnt2d(IntCS8.Point(1).X(), IntCS8.Point(1).Y());
        Bezier13(3) = gp_Pnt2d(IntCS9.Point(1).X(), IntCS9.Point(1).Y());
        Bezier13(4) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier3 = new Geom2d_BezierCurve(Bezier13);

        TColgp_Array1OfPnt2d Bezier14(1, 4);
        Bezier14(1) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Bezier14(2) = gp_Pnt2d(IntCS11.Point(1).X(), IntCS11.Point(1).Y());
        Bezier14(3) = gp_Pnt2d(IntCS12.Point(1).X(), IntCS12.Point(1).Y());
        Bezier14(4) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier4 = new Geom2d_BezierCurve(Bezier14);

        // -- Create the host plane
        gp_Pln Pln(gp_Ax3(P, d[i], d2 ));
        //Handle(Geom_Plane) s = new Geom_Plane (Pl1);

        Edge_B11 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier1, Pln) );
        Edge_B21 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier2, Pln) );
        Edge_B31 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier3, Pln) );
        Edge_B41 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier4, Pln) );

        // Create wire

        BRepBuilderAPI_MakeWire mkWire;
        mkWire.Add(Edge_B11);
        mkWire.Add(Edge_B21);
        mkWire.Add(Edge_B31);
        mkWire.Add(Edge_B41);
        Wire[i] = mkWire.Wire();


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
    }
    //
    if ( !aGenerator.IsDone() )
    {
        std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
    }

    tube = aGenerator.Shape();

    //---------------------------------------------------------------------------
    // Stage 4 - // Inlet and Outlet Pipe
    //---------------------------------------------------------------------------

    gp_Vec V0(d[0]);
    BRepPrimAPI_MakePrism mkPrismIn(Wire[0], -maxLengthVerticalPipes*V0);
    mkPrismIn.Build();
    TopoDS_Shape InletPipe = mkPrismIn.Shape();
    // Get the Inlet wire
    TopoDS_Wire wbaseIn = TopoDS::Wire(mkPrismIn.LastShape());
    // Create the Inlet face from this wire
    TopoDS_Face fbaseIn = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseIn)).Face();

    gp_Vec Vmaxslices(d[maxSlices-1]);
    //BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], (28/31.3)*maxLengthVerticalPipes*Vmaxslices);
    // BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], (8.0/10.0)*maxLengthVerticalPipes*Vmaxslices);
    BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], (0.6/0.67755)*maxLengthVerticalPipes*Vmaxslices);
    mkPrismOut.Build();
    TopoDS_Shape OutletPipe = mkPrismOut.Shape();
    // Get the outlet wire
    TopoDS_Wire wbaseOut = TopoDS::Wire(mkPrismOut.LastShape());
    // Create the outlet face from this wire
    TopoDS_Face fbaseOut = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseOut)).Face();

    //---------------------------------------------------------------------------
    // Stage 5 - // Build the shell or solid
    //---------------------------------------------------------------------------

    BRepBuilderAPI_Sewing Sewing;
    Sewing.Add(fbaseIn);
    Sewing.Add(InletPipe);
    Sewing.Add(tube);
    Sewing.Add(OutletPipe);
    Sewing.Add(fbaseOut);
    Sewing.Perform();
    tube = Sewing.SewedShape();
    tube = TopoDS::Shell(tube);
//  TopoDS_Solid solidTube;
//  BRep_Builder mkSolid;
//  mkSolid.MakeSolid(solidTube);
//  mkSolid.Add(solidTube, tube);

    return tube;
}

//March21th
TopoDS_Shape OCCTDataProvider::ConstructSquaredUbendWithRealDimensionsFixedLegs()
{
    int maxSlices = 35;
    // double maxLengthVerticalPipes = 2.3475;
     double maxLengthVerticalPipes = 0.82755;
    //double maxLengthVerticalPipes = 0.67755;

//    std::vector<Standard_Real> designParameters =
//            {
//                    Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375),
//                    Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375),
//                    Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375),
//                    Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375),
//                    Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875),
//                    Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875),
//                    Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375),
//                    Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375),
//                    Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375),
//                    Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375),
//                    Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875),
//                    Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875)
//            };
//#define maxNbDPcp 4 //number of design parameters
    //---------------------------------------------------------------------------
    // Stage 1: Construct laws from design parameters
    //---------------------------------------------------------------------------

    //Knots, Multipl and Degree of the laws

    TColStd_Array1OfReal lawKnots(1,6);
    lawKnots(1)=0.;
    lawKnots(2)=0.2;
    lawKnots(3)=0.4;
    lawKnots(4)=0.6;
    lawKnots(5)=0.8;
    lawKnots(6)=1;

    TColStd_Array1OfInteger lawMults(1,6);
    lawMults(1)=4;
    lawMults(2)=1;
    lawMults(3)=1;
    lawMults(4)=1;
    lawMults(5)=1;
    lawMults(6)=4;

    Standard_Integer lawDegree = 3;

    //CP1
    TColgp_Array1OfPnt lawCP1(1, 8);
    lawCP1(1) = gp_Pnt(-0.0375, 0.0375, 0.); //CP1_P1;
    lawCP1(2) = gp_Pnt(-0.0375, 0.0375, 0.15); //CP1_P2;
    lawCP1(3) = gp_Pnt(designParameters[0], designParameters[1], 0.30); //CP1_P3;
    lawCP1(4) = gp_Pnt(designParameters[2], designParameters[3], 0.45); //CP1_P4;
    lawCP1(5) = gp_Pnt(designParameters[4], designParameters[5], 0.60); //CP1_P5;
    lawCP1(6) = gp_Pnt(designParameters[6], designParameters[7], 0.75); //CP1_P6;
    lawCP1(7) = gp_Pnt(-0.0375, 0.0375, 0.85); //CP1_P7;
    lawCP1(8) = gp_Pnt(-0.0375, 0.0375, 1); //CP1_P8;
    Handle(Geom_BSplineCurve) BSplineCP1 = new Geom_BSplineCurve(lawCP1, lawKnots, lawMults, lawDegree, Standard_False);

    //CP2
    TColgp_Array1OfPnt lawCP2(1, 8);
    lawCP2(1) = gp_Pnt(-0.01875, 0.0375, 0.); //CP2_P1;
    lawCP2(2) = gp_Pnt(-0.01875, 0.0375, 0.15); //CP2_P2;
    lawCP2(3) = gp_Pnt(designParameters[8], designParameters[9], 0.30); //CP2_P3;
    lawCP2(4) = gp_Pnt(designParameters[10], designParameters[11], 0.45); //CP2_P4;
    lawCP2(5) = gp_Pnt(designParameters[12], designParameters[13], 0.60); //CP2_P5;
    lawCP2(6) = gp_Pnt(designParameters[14], designParameters[15], 0.75); //CP2_P6;
    lawCP2(7) = gp_Pnt(-0.01875, 0.0375, 0.85); //CP2_P7;
    lawCP2(8) = gp_Pnt(-0.01875, 0.0375, 1.); //CP2_P8;
    Handle(Geom_BSplineCurve) BSplineCP2 = new Geom_BSplineCurve(lawCP2, lawKnots, lawMults, lawDegree, Standard_False);

    //CP3
    TColgp_Array1OfPnt lawCP3(1, 8);
    lawCP3(1) = gp_Pnt(0.01875, 0.0375, 0.); //CP3_P1;
    lawCP3(2) = gp_Pnt(0.01875, 0.0375, 0.15); //CP3_P2;
    lawCP3(3) = gp_Pnt(designParameters[16], designParameters[17], 0.30); //CP3_P3;
    lawCP3(4) = gp_Pnt(designParameters[18], designParameters[19], 0.45); //CP3_P4;
    lawCP3(5) = gp_Pnt(designParameters[20], designParameters[21], 0.60); //CP3_P5;
    lawCP3(6) = gp_Pnt(designParameters[22], designParameters[23], 0.75); //CP3_P6;
    lawCP3(7) = gp_Pnt(0.01875, 0.0375, 0.85); //CP3_P7;
    lawCP3(8) = gp_Pnt(0.01875, 0.0375, 1.); //CP3_P8;
    Handle(Geom_BSplineCurve) BSplineCP3 = new Geom_BSplineCurve(lawCP3, lawKnots, lawMults, lawDegree, Standard_False);

    //CP4
    TColgp_Array1OfPnt lawCP4(1, 8);
    lawCP4(1) = gp_Pnt(0.0375, 0.0375, 0.); //CP4_P1;
    lawCP4(2) = gp_Pnt(0.0375, 0.0375, 0.15); //CP4_P2;
    lawCP4(3) = gp_Pnt(designParameters[24], designParameters[25], 0.30); //CP4_P3;
    lawCP4(4) = gp_Pnt(designParameters[26], designParameters[27], 0.45); //CP4_P4;
    lawCP4(5) = gp_Pnt(designParameters[28], designParameters[29], 0.60); //CP4_P5;
    lawCP4(6) = gp_Pnt(designParameters[30], designParameters[31], 0.75); //CP4_P6;
    lawCP4(7) = gp_Pnt(0.0375, 0.0375, 0.85); //CP4_P7;
    lawCP4(8) = gp_Pnt(0.0375, 0.0375, 1.); //CP4_P8;
    Handle(Geom_BSplineCurve) BSplineCP4 = new Geom_BSplineCurve(lawCP4, lawKnots, lawMults, lawDegree, Standard_False);

    //CP5
    TColgp_Array1OfPnt lawCP5(1, 8);
    lawCP5(1) = gp_Pnt(0.0375, 0.01875, 0.); //CP5_P1;
    lawCP5(2) = gp_Pnt(0.0375, 0.01875, 0.15); //CP5_P2;
    lawCP5(3) = gp_Pnt(designParameters[32], designParameters[33], 0.30); //CP5_P3;
    lawCP5(4) = gp_Pnt(designParameters[34], designParameters[35], 0.45); //CP5_P4;
    lawCP5(5) = gp_Pnt(designParameters[36], designParameters[37], 0.60); //CP5_P5;
    lawCP5(6) = gp_Pnt(designParameters[38], designParameters[39], 0.75); //CP5_P6;
    lawCP5(7) = gp_Pnt(0.0375, 0.01875, 0.85); //CP5_P7;
    lawCP5(8) = gp_Pnt(0.0375, 0.01875, 1.); //CP5_P8;
    Handle(Geom_BSplineCurve) BSplineCP5 = new Geom_BSplineCurve(lawCP5, lawKnots, lawMults, lawDegree, Standard_False);

    //CP6
    TColgp_Array1OfPnt lawCP6(1, 8);
    lawCP6(1) = gp_Pnt(0.0375,-0.01875, 0.); //CP6_P1;
    lawCP6(2) = gp_Pnt(0.0375,-0.01875, 0.15); //CP6_P2;
    lawCP6(3) = gp_Pnt(designParameters[40], designParameters[41], 0.30); //CP6_P3;
    lawCP6(4) = gp_Pnt(designParameters[42], designParameters[43], 0.45); //CP6_P4;
    lawCP6(5) = gp_Pnt(designParameters[44], designParameters[45], 0.60); //CP6_P5;
    lawCP6(6) = gp_Pnt(designParameters[46], designParameters[47], 0.75); //CP6_P6;
    lawCP6(7) = gp_Pnt(0.0375,-0.01875, 0.85); //CP6_P7;
    lawCP6(8) = gp_Pnt(0.0375,-0.01875, 1.); //CP6_P8;
    Handle(Geom_BSplineCurve) BSplineCP6 = new Geom_BSplineCurve(lawCP6, lawKnots, lawMults, lawDegree, Standard_False);

    //CP7
    TColgp_Array1OfPnt lawCP7(1, 8);
    lawCP7(1) = gp_Pnt(0.0375, -0.0375, 0.); //CP7_P1;
    lawCP7(2) = gp_Pnt(0.0375, -0.0375, 0.15); //CP7_P2;
    lawCP7(3) = gp_Pnt(designParameters[48], designParameters[49], 0.30); //CP7_P3;
    lawCP7(4) = gp_Pnt(designParameters[50], designParameters[51], 0.45); //CP7_P4;
    lawCP7(5) = gp_Pnt(designParameters[52], designParameters[53], 0.60); //CP7_P5;
    lawCP7(6) = gp_Pnt(designParameters[54], designParameters[55], 0.75); //CP7_P6;
    lawCP7(7) = gp_Pnt(0.0375, -0.0375, 0.85); //CP7_P7;
    lawCP7(8) = gp_Pnt(0.0375, -0.0375, 1.); //CP7_P8;
    Handle(Geom_BSplineCurve) BSplineCP7 = new Geom_BSplineCurve(lawCP7, lawKnots, lawMults, lawDegree, Standard_False);

    //CP8
    TColgp_Array1OfPnt lawCP8(1, 8);
    lawCP8(1) = gp_Pnt(0.01875, -0.0375, 0.); //CP8_P1;
    lawCP8(2) = gp_Pnt(0.01875, -0.0375, 0.15); //CP8_P2;
    lawCP8(3) = gp_Pnt(designParameters[56], designParameters[57], 0.30); //CP8_P3;
    lawCP8(4) = gp_Pnt(designParameters[58], designParameters[59], 0.45); //CP8_P4;
    lawCP8(5) = gp_Pnt(designParameters[60], designParameters[61], 0.60); //CP8_P5;
    lawCP8(6) = gp_Pnt(designParameters[62], designParameters[63], 0.75); //CP8_P6;
    lawCP8(7) = gp_Pnt(0.01875, -0.0375, 0.85); //CP8_P7;
    lawCP8(8) = gp_Pnt(0.01875, -0.0375, 1); //CP8_P8;
    Handle(Geom_BSplineCurve) BSplineCP8 = new Geom_BSplineCurve(lawCP8, lawKnots, lawMults, lawDegree, Standard_False);

    //CP9
    TColgp_Array1OfPnt lawCP9(1, 8);
    lawCP9(1) = gp_Pnt(-0.01875,-0.0375, 0.); //CP9_P1;
    lawCP9(2) = gp_Pnt(-0.01875, -0.0375, 0.15); //CP9_P2;
    lawCP9(3) = gp_Pnt(designParameters[64], designParameters[65], 0.30); //CP9_P3;
    lawCP9(4) = gp_Pnt(designParameters[66], designParameters[67], 0.45); //CP9_P4;
    lawCP9(5) = gp_Pnt(designParameters[68], designParameters[69], 0.60); //CP9_P5;
    lawCP9(6) = gp_Pnt(designParameters[70], designParameters[71], 0.75); //CP9_P6;
    lawCP9(7) = gp_Pnt(-0.01875, -0.0375, 0.85); //CP9_P7;
    lawCP9(8) = gp_Pnt(-0.01875, -0.0375, 1.); //CP9_P8;
    Handle(Geom_BSplineCurve) BSplineCP9 = new Geom_BSplineCurve(lawCP9, lawKnots, lawMults, lawDegree, Standard_False);

    //CP10
    TColgp_Array1OfPnt lawCP10(1, 8);
    lawCP10(1) = gp_Pnt(-0.0375, -0.0375, 0.); //CP10_P1;
    lawCP10(2) = gp_Pnt(-0.0375, -0.0375, 0.15); //CP10_P2;
    lawCP10(3) = gp_Pnt(designParameters[72], designParameters[73], 0.30); //CP10_P3;
    lawCP10(4) = gp_Pnt(designParameters[74], designParameters[75], 0.45); //CP10_P4;
    lawCP10(5) = gp_Pnt(designParameters[76], designParameters[77], 0.60); //CP10_P5;
    lawCP10(6) = gp_Pnt(designParameters[78], designParameters[79], 0.75); //CP10_P6;
    lawCP10(7) = gp_Pnt(-0.0375, -0.0375, 0.85); //CP10_P7;
    lawCP10(8) = gp_Pnt(-0.0375, -0.0375, 1.); //CP10_P8;
    Handle(Geom_BSplineCurve) BSplineCP10 = new Geom_BSplineCurve(lawCP10, lawKnots, lawMults, lawDegree, Standard_False);

    //CP11
    TColgp_Array1OfPnt lawCP11(1, 8);
    lawCP11(1) = gp_Pnt(-0.0375, -0.01875, 0.); //CP11_P1;
    lawCP11(2) = gp_Pnt(-0.0375, -0.01875, 0.15); //CP11_P2;
    lawCP11(3) = gp_Pnt(designParameters[80], designParameters[81], 0.30); //CP11_P3;
    lawCP11(4) = gp_Pnt(designParameters[82], designParameters[83], 0.45); //CP11_P4;
    lawCP11(5) = gp_Pnt(designParameters[84], designParameters[85], 0.60); //CP11_P5;
    lawCP11(6) = gp_Pnt(designParameters[86], designParameters[87], 0.75); //CP11_P6;
    lawCP11(7) = gp_Pnt(-0.0375, -0.01875, 0.85); //CP11_P7;
    lawCP11(8) = gp_Pnt(-0.0375, -0.01875, 1.); //CP11_P8;
    Handle(Geom_BSplineCurve) BSplineCP11 = new Geom_BSplineCurve(lawCP11, lawKnots, lawMults, lawDegree, Standard_False);

    //CP12
    TColgp_Array1OfPnt lawCP12(1, 8);
    lawCP12(1) = gp_Pnt(-0.0375, 0.01875, 0.); //CP12_P1;
    lawCP12(2) = gp_Pnt(-0.0375, 0.01875, 0.15); //CP12_P2;
    lawCP12(3) = gp_Pnt(designParameters[88], designParameters[89], 0.30); //CP12_P3;
    lawCP12(4) = gp_Pnt(designParameters[90], designParameters[91], 0.45); //CP12_P4;
    lawCP12(5) = gp_Pnt(designParameters[92], designParameters[93], 0.60); //CP12_P5;
    lawCP12(6) = gp_Pnt(designParameters[94], designParameters[95], 0.75); //CP12_P6;
    lawCP12(7) = gp_Pnt(-0.0375, 0.01875, 0.85); //CP12_P7;
    lawCP12(8) = gp_Pnt(-0.0375, 0.01875, 1.); //CP12_P8;
    Handle(Geom_BSplineCurve) BSplineCP12 = new Geom_BSplineCurve(lawCP12, lawKnots, lawMults, lawDegree, Standard_False);

    //---------------------------------------------------------------------------
    // Stage 2: U-bend: B-Spline path for U-bend
    //---------------------------------------------------------------------------
    gp_Pnt O (0,0,0);
    gp_Ax2 xyzAxis (O, gp::DZ(), -gp::DY());
    Standard_Real r = 0.057;
    Handle(Geom_Circle) Cxyz = new Geom_Circle( xyzAxis, r );

  //

    Handle (Geom_TrimmedCurve) path2 = new Geom_TrimmedCurve(Cxyz , 0, M_PI);
    Handle(Geom_BSplineCurve) path = GeomConvert::CurveToBSplineCurve(path2, Convert_QuasiAngular);



//    TColgp_Array1OfPnt pathPoles(1, 5);
//    //  pathPoles(1) = gp_Pnt(-0.150,  0.057,   0.);
//    //  pathPoles(2) = gp_Pnt(-0.075, 0.057,   0.);
//    pathPoles(1) = gp_Pnt(0., 0.057,  0.);
//    pathPoles(2) = gp_Pnt(0.057, 0.057, 0.);
//  //  pathPoles(3) = gp_Pnt(0.057, 3.49024337756996e-15, 0.);
//    pathPoles(3) = gp_Pnt(0.057, 0, 0.);
//    pathPoles(4) = gp_Pnt(0.057, -0.057, 0.);
//    pathPoles(5) = gp_Pnt(0. , -0.057,   0.);
////    pathPoles(8) = gp_Pnt(-0.075, -0.057,   0.);
////    pathPoles(9) = gp_Pnt(-0.150, -0.057,   0.);
//
//    TColStd_Array1OfReal pathWeights(1,5);
////    pathWeights(1)=1;
////    pathWeights(2)=1;
//    pathWeights(1)=1;
//    pathWeights(2)=0.707106781186548;
//    pathWeights(3)=1;
//    pathWeights(4)=0.707106781186548;
//    pathWeights(5)=1;
////    pathWeights(8)=1;
////    pathWeights(9)=1;
//
//    TColStd_Array1OfReal pathKnots(1,4);
//    pathKnots(1)=0;
//    pathKnots(2)=0.3;
//    pathKnots(3)=0.6;
//    pathKnots(4)=1;
//
//    TColStd_Array1OfInteger pathMults(1,4);
//    pathMults(1)=3;
//    pathMults(2)=1;
//    pathMults(3)=1;
//    pathMults(4)=3;
//
//
//    Standard_Integer pathDegree = 2;
//
//    Handle(Geom_BSplineCurve) path = new Geom_BSplineCurve(pathPoles, pathWeights, pathKnots, pathMults, pathDegree, Standard_False, Standard_True);

    //DrawTrSurf::Set("path", path);

    GeomAdaptor_Curve GAC;
    GAC.Load(path);
    Standard_Real Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve

    //---------------------------------------------------------------------------
    // Stage 3: U-bend: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    TopoDS_Edge Edge_B11;
    TopoDS_Edge Edge_B21;
    TopoDS_Edge Edge_B31;
    TopoDS_Edge Edge_B41;
    TopoDS_Shape tube;
    gp_Dir d[maxSlices];
    gp_Dir d2(0,0,1);
    TopoDS_Wire Wire[maxSlices];

    Standard_Real u[maxSlices];

    u[0]             = path->FirstParameter();
    u[maxSlices - 1] = path->LastParameter();

    const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);

    for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
    {
        u[i] = u[0] + i*ustep;
    }

    // Prepare skinning tool

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;
    const Standard_Real    pres3d  = 1.0e-02;

    BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);
    aGenerator.CheckCompatibility(Standard_False);
    //
    for ( Standard_Integer i = 0; i < maxSlices; ++i )
    {

        gp_Pnt P;
        gp_Vec V;
        path->D1(u[i], P, V);
        d[i] = gp_Dir(V);

        Standard_Real P_Length = GCPnts_AbscissaPoint::Length(GAC, u[0], u[i]);//length of the curve

        Standard_Real Length_step = (P_Length/Length);

        // -- Create the intersection plane
        gp_Pnt P_path(0, 0, Length_step);

        gp_Pln constructionPl(P_path, d2);
        Handle(Geom_Plane) constructionPlane = new Geom_Plane (constructionPl);

        //IP1
        GeomAPI_IntCS IntCS1(BSplineCP1, constructionPlane);
        //cout << "Nb of Int1 points is " << IntCS1.NbPoints() << endl;
        //IP2
        GeomAPI_IntCS IntCS2(BSplineCP2, constructionPlane);
        //IP3
        GeomAPI_IntCS IntCS3(BSplineCP3, constructionPlane);
        //IP4
        GeomAPI_IntCS IntCS4(BSplineCP4, constructionPlane);
        //IP5
        GeomAPI_IntCS IntCS5(BSplineCP5, constructionPlane);
        //IP6
        GeomAPI_IntCS IntCS6(BSplineCP6, constructionPlane);
        //IP7
        GeomAPI_IntCS IntCS7(BSplineCP7, constructionPlane);
        //IP8
        GeomAPI_IntCS IntCS8(BSplineCP8, constructionPlane);
        //IP9
        GeomAPI_IntCS IntCS9(BSplineCP9, constructionPlane);
        //IP10
        GeomAPI_IntCS IntCS10(BSplineCP10, constructionPlane);
        //IP11
        GeomAPI_IntCS IntCS11(BSplineCP11, constructionPlane);
        //IP12
        GeomAPI_IntCS IntCS12(BSplineCP12, constructionPlane);

        TColgp_Array1OfPnt2d Bezier11(1, 4);
        Bezier11(1) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Bezier11(2) = gp_Pnt2d(IntCS2.Point(1).X(), IntCS2.Point(1).Y());
        Bezier11(3) = gp_Pnt2d(IntCS3.Point(1).X(), IntCS3.Point(1).Y());
        Bezier11(4) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier1 = new Geom2d_BezierCurve(Bezier11);

        TColgp_Array1OfPnt2d Bezier12(1, 4);
        Bezier12(1) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Bezier12(2) = gp_Pnt2d(IntCS5.Point(1).X(), IntCS5.Point(1).Y());
        Bezier12(3) = gp_Pnt2d(IntCS6.Point(1).X(), IntCS6.Point(1).Y());
        Bezier12(4) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier2 = new Geom2d_BezierCurve(Bezier12);

        TColgp_Array1OfPnt2d Bezier13(1, 4);
        Bezier13(1) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Bezier13(2) = gp_Pnt2d(IntCS8.Point(1).X(), IntCS8.Point(1).Y());
        Bezier13(3) = gp_Pnt2d(IntCS9.Point(1).X(), IntCS9.Point(1).Y());
        Bezier13(4) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier3 = new Geom2d_BezierCurve(Bezier13);

        TColgp_Array1OfPnt2d Bezier14(1, 4);
        Bezier14(1) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Bezier14(2) = gp_Pnt2d(IntCS11.Point(1).X(), IntCS11.Point(1).Y());
        Bezier14(3) = gp_Pnt2d(IntCS12.Point(1).X(), IntCS12.Point(1).Y());
        Bezier14(4) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier4 = new Geom2d_BezierCurve(Bezier14);

        // -- Create the host plane
        gp_Pln Pln(gp_Ax3(P, d[i], d2 ));
        //Handle(Geom_Plane) s = new Geom_Plane (Pl1);

        Edge_B11 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier1, Pln) );
        Edge_B21 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier2, Pln) );
        Edge_B31 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier3, Pln) );
        Edge_B41 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier4, Pln) );

        // Create wire

        BRepBuilderAPI_MakeWire mkWire;
        mkWire.Add(Edge_B11);
        mkWire.Add(Edge_B21);
        mkWire.Add(Edge_B31);
        mkWire.Add(Edge_B41);
        Wire[i] = mkWire.Wire();


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
    }
    //
    if ( !aGenerator.IsDone() )
    {
        std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
    }

    tube = aGenerator.Shape();

    //---------------------------------------------------------------------------
    // Stage 4 - // Inlet and Outlet Pipe
    //---------------------------------------------------------------------------
//inlet and outlet changed
    gp_Vec V0(d[0]);
    BRepPrimAPI_MakePrism mkPrismIn(Wire[0], -(0.75/0.82755)*maxLengthVerticalPipes*V0);
    mkPrismIn.Build();
    TopoDS_Shape InletPipe = mkPrismIn.Shape();
    // Get the Inlet wire
    TopoDS_Wire wbaseIn = TopoDS::Wire(mkPrismIn.LastShape());
    // Create the Inlet face from this wire
    TopoDS_Face fbaseIn = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseIn)).Face();

    gp_Vec Vmaxslices(d[maxSlices-1]);
    //BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], (28/31.3)*maxLengthVerticalPipes*Vmaxslices);
    // BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], (8.0/10.0)*maxLengthVerticalPipes*Vmaxslices);
    BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], maxLengthVerticalPipes*Vmaxslices);
    mkPrismOut.Build();
    TopoDS_Shape OutletPipe = mkPrismOut.Shape();
    // Get the outlet wire
    TopoDS_Wire wbaseOut = TopoDS::Wire(mkPrismOut.LastShape());
    // Create the outlet face from this wire
    TopoDS_Face fbaseOut = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseOut)).Face();

    //---------------------------------------------------------------------------
    // Stage 5 - // Build the shell or solid
    //---------------------------------------------------------------------------

    BRepBuilderAPI_Sewing Sewing;
    Sewing.Add(fbaseIn);
    Sewing.Add(InletPipe);
    Sewing.Add(tube);
    Sewing.Add(OutletPipe);
    Sewing.Add(fbaseOut);
    Sewing.Perform();
    tube = Sewing.SewedShape();
    tube = TopoDS::Shell(tube);
//  TopoDS_Solid solidTube;
//  BRep_Builder mkSolid;
//  mkSolid.MakeSolid(solidTube);
//  mkSolid.Add(solidTube, tube);

    return tube;
}

//June 10th
TopoDS_Shape OCCTDataProvider::ConstructSquaredUbend_June10th(bool activateDesignParametersForReverse)
{
#if REVERSE_MODE
    if(activateDesignParametersForReverse)
    {
      for(int cnt = 0; cnt < nParams; cnt++)
        designParameters[cnt] <<= designParameters[cnt].getValue();
    }
#endif
    int maxSlices = 70;

    double maxLengthVerticalPipes = 0.67755;
    //---------------------------------------------------------------------------
    // Stage 1: Construct laws from design parameters
    //---------------------------------------------------------------------------

    //Knots, Multipl and Degree of the laws

    TColStd_Array1OfReal lawKnots(1,6);
    lawKnots(1)=0.;
    lawKnots(2)=0.2;
    lawKnots(3)=0.4;
    lawKnots(4)=0.6;
    lawKnots(5)=0.8;
    lawKnots(6)=1;

    TColStd_Array1OfInteger lawMults(1,6);
    lawMults(1)=4;
    lawMults(2)=1;
    lawMults(3)=1;
    lawMults(4)=1;
    lawMults(5)=1;
    lawMults(6)=4;

    Standard_Integer lawDegree = 3;

    //CP1
    TColgp_Array1OfPnt lawCP1(1, 8);
    lawCP1(1) = gp_Pnt(-0.0375, 0.0375, 0.); //CP1_P1;
    lawCP1(2) = gp_Pnt(-0.0375, 0.0375, 0.15); //CP1_P2;
    lawCP1(3) = gp_Pnt(designParameters[0], designParameters[1], 0.30); //CP1_P3;
    lawCP1(4) = gp_Pnt(designParameters[2], designParameters[3], 0.45); //CP1_P4;
    lawCP1(5) = gp_Pnt(designParameters[4], designParameters[5], 0.60); //CP1_P5;
    lawCP1(6) = gp_Pnt(designParameters[6], designParameters[7], 0.75); //CP1_P6;
    lawCP1(7) = gp_Pnt(-0.0375, 0.0375, 0.85); //CP1_P7;
    lawCP1(8) = gp_Pnt(-0.0375, 0.0375, 1); //CP1_P8;
    Handle(Geom_BSplineCurve) BSplineCP1 = new Geom_BSplineCurve(lawCP1, lawKnots, lawMults, lawDegree, Standard_False);

    //CP2
    TColgp_Array1OfPnt lawCP2(1, 8);
    lawCP2(1) = gp_Pnt(-0.01875, 0.0375, 0.); //CP2_P1;
    lawCP2(2) = gp_Pnt(-0.01875, 0.0375, 0.15); //CP2_P2;
    lawCP2(3) = gp_Pnt(designParameters[8], designParameters[9], 0.30); //CP2_P3;
    lawCP2(4) = gp_Pnt(designParameters[10], designParameters[11], 0.45); //CP2_P4;
    lawCP2(5) = gp_Pnt(designParameters[12], designParameters[13], 0.60); //CP2_P5;
    lawCP2(6) = gp_Pnt(designParameters[14], designParameters[15], 0.75); //CP2_P6;
    lawCP2(7) = gp_Pnt(-0.01875, 0.0375, 0.85); //CP2_P7;
    lawCP2(8) = gp_Pnt(-0.01875, 0.0375, 1.); //CP2_P8;
    Handle(Geom_BSplineCurve) BSplineCP2 = new Geom_BSplineCurve(lawCP2, lawKnots, lawMults, lawDegree, Standard_False);

    //CP3
    TColgp_Array1OfPnt lawCP3(1, 8);
    lawCP3(1) = gp_Pnt(0.01875, 0.0375, 0.); //CP3_P1;
    lawCP3(2) = gp_Pnt(0.01875, 0.0375, 0.15); //CP3_P2;
    lawCP3(3) = gp_Pnt(designParameters[16], designParameters[17], 0.30); //CP3_P3;
    lawCP3(4) = gp_Pnt(designParameters[18], designParameters[19], 0.45); //CP3_P4;
    lawCP3(5) = gp_Pnt(designParameters[20], designParameters[21], 0.60); //CP3_P5;
    lawCP3(6) = gp_Pnt(designParameters[22], designParameters[23], 0.75); //CP3_P6;
    lawCP3(7) = gp_Pnt(0.01875, 0.0375, 0.85); //CP3_P7;
    lawCP3(8) = gp_Pnt(0.01875, 0.0375, 1.); //CP3_P8;
    Handle(Geom_BSplineCurve) BSplineCP3 = new Geom_BSplineCurve(lawCP3, lawKnots, lawMults, lawDegree, Standard_False);

    //CP4
    TColgp_Array1OfPnt lawCP4(1, 8);
    lawCP4(1) = gp_Pnt(0.0375, 0.0375, 0.); //CP4_P1;
    lawCP4(2) = gp_Pnt(0.0375, 0.0375, 0.15); //CP4_P2;
    lawCP4(3) = gp_Pnt(designParameters[24], designParameters[25], 0.30); //CP4_P3;
    lawCP4(4) = gp_Pnt(designParameters[26], designParameters[27], 0.45); //CP4_P4;
    lawCP4(5) = gp_Pnt(designParameters[28], designParameters[29], 0.60); //CP4_P5;
    lawCP4(6) = gp_Pnt(designParameters[30], designParameters[31], 0.75); //CP4_P6;
    lawCP4(7) = gp_Pnt(0.0375, 0.0375, 0.85); //CP4_P7;
    lawCP4(8) = gp_Pnt(0.0375, 0.0375, 1.); //CP4_P8;
    Handle(Geom_BSplineCurve) BSplineCP4 = new Geom_BSplineCurve(lawCP4, lawKnots, lawMults, lawDegree, Standard_False);

    //CP5
    TColgp_Array1OfPnt lawCP5(1, 8);
    lawCP5(1) = gp_Pnt(0.0375, 0.01875, 0.); //CP5_P1;
    lawCP5(2) = gp_Pnt(0.0375, 0.01875, 0.15); //CP5_P2;
    lawCP5(3) = gp_Pnt(designParameters[32], designParameters[33], 0.30); //CP5_P3;
    lawCP5(4) = gp_Pnt(designParameters[34], designParameters[35], 0.45); //CP5_P4;
    lawCP5(5) = gp_Pnt(designParameters[36], designParameters[37], 0.60); //CP5_P5;
    lawCP5(6) = gp_Pnt(designParameters[38], designParameters[39], 0.75); //CP5_P6;
    lawCP5(7) = gp_Pnt(0.0375, 0.01875, 0.85); //CP5_P7;
    lawCP5(8) = gp_Pnt(0.0375, 0.01875, 1.); //CP5_P8;
    Handle(Geom_BSplineCurve) BSplineCP5 = new Geom_BSplineCurve(lawCP5, lawKnots, lawMults, lawDegree, Standard_False);

    //CP6
    TColgp_Array1OfPnt lawCP6(1, 8);
    lawCP6(1) = gp_Pnt(0.0375,-0.01875, 0.); //CP6_P1;
    lawCP6(2) = gp_Pnt(0.0375,-0.01875, 0.15); //CP6_P2;
    lawCP6(3) = gp_Pnt(designParameters[40], designParameters[41], 0.30); //CP6_P3;
    lawCP6(4) = gp_Pnt(designParameters[42], designParameters[43], 0.45); //CP6_P4;
    lawCP6(5) = gp_Pnt(designParameters[44], designParameters[45], 0.60); //CP6_P5;
    lawCP6(6) = gp_Pnt(designParameters[46], designParameters[47], 0.75); //CP6_P6;
    lawCP6(7) = gp_Pnt(0.0375,-0.01875, 0.85); //CP6_P7;
    lawCP6(8) = gp_Pnt(0.0375,-0.01875, 1.); //CP6_P8;
    Handle(Geom_BSplineCurve) BSplineCP6 = new Geom_BSplineCurve(lawCP6, lawKnots, lawMults, lawDegree, Standard_False);

    //CP7
    TColgp_Array1OfPnt lawCP7(1, 8);
    lawCP7(1) = gp_Pnt(0.0375, -0.0375, 0.); //CP7_P1;
    lawCP7(2) = gp_Pnt(0.0375, -0.0375, 0.15); //CP7_P2;
    lawCP7(3) = gp_Pnt(designParameters[48], designParameters[49], 0.30); //CP7_P3;
    lawCP7(4) = gp_Pnt(designParameters[50], designParameters[51], 0.45); //CP7_P4;
    lawCP7(5) = gp_Pnt(designParameters[52], designParameters[53], 0.60); //CP7_P5;
    lawCP7(6) = gp_Pnt(designParameters[54], designParameters[55], 0.75); //CP7_P6;
    lawCP7(7) = gp_Pnt(0.0375, -0.0375, 0.85); //CP7_P7;
    lawCP7(8) = gp_Pnt(0.0375, -0.0375, 1.); //CP7_P8;
    Handle(Geom_BSplineCurve) BSplineCP7 = new Geom_BSplineCurve(lawCP7, lawKnots, lawMults, lawDegree, Standard_False);

    //CP8
    TColgp_Array1OfPnt lawCP8(1, 8);
    lawCP8(1) = gp_Pnt(0.01875, -0.0375, 0.); //CP8_P1;
    lawCP8(2) = gp_Pnt(0.01875, -0.0375, 0.15); //CP8_P2;
    lawCP8(3) = gp_Pnt(designParameters[56], designParameters[57], 0.30); //CP8_P3;
    lawCP8(4) = gp_Pnt(designParameters[58], designParameters[59], 0.45); //CP8_P4;
    lawCP8(5) = gp_Pnt(designParameters[60], designParameters[61], 0.60); //CP8_P5;
    lawCP8(6) = gp_Pnt(designParameters[62], designParameters[63], 0.75); //CP8_P6;
    lawCP8(7) = gp_Pnt(0.01875, -0.0375, 0.85); //CP8_P7;
    lawCP8(8) = gp_Pnt(0.01875, -0.0375, 1); //CP8_P8;
    Handle(Geom_BSplineCurve) BSplineCP8 = new Geom_BSplineCurve(lawCP8, lawKnots, lawMults, lawDegree, Standard_False);

    //CP9
    TColgp_Array1OfPnt lawCP9(1, 8);
    lawCP9(1) = gp_Pnt(-0.01875,-0.0375, 0.); //CP9_P1;
    lawCP9(2) = gp_Pnt(-0.01875, -0.0375, 0.15); //CP9_P2;
    lawCP9(3) = gp_Pnt(designParameters[64], designParameters[65], 0.30); //CP9_P3;
    lawCP9(4) = gp_Pnt(designParameters[66], designParameters[67], 0.45); //CP9_P4;
    lawCP9(5) = gp_Pnt(designParameters[68], designParameters[69], 0.60); //CP9_P5;
    lawCP9(6) = gp_Pnt(designParameters[70], designParameters[71], 0.75); //CP9_P6;
    lawCP9(7) = gp_Pnt(-0.01875, -0.0375, 0.85); //CP9_P7;
    lawCP9(8) = gp_Pnt(-0.01875, -0.0375, 1.); //CP9_P8;
    Handle(Geom_BSplineCurve) BSplineCP9 = new Geom_BSplineCurve(lawCP9, lawKnots, lawMults, lawDegree, Standard_False);

    //CP10
    TColgp_Array1OfPnt lawCP10(1, 8);
    lawCP10(1) = gp_Pnt(-0.0375, -0.0375, 0.); //CP10_P1;
    lawCP10(2) = gp_Pnt(-0.0375, -0.0375, 0.15); //CP10_P2;
    lawCP10(3) = gp_Pnt(designParameters[72], designParameters[73], 0.30); //CP10_P3;
    lawCP10(4) = gp_Pnt(designParameters[74], designParameters[75], 0.45); //CP10_P4;
    lawCP10(5) = gp_Pnt(designParameters[76], designParameters[77], 0.60); //CP10_P5;
    lawCP10(6) = gp_Pnt(designParameters[78], designParameters[79], 0.75); //CP10_P6;
    lawCP10(7) = gp_Pnt(-0.0375, -0.0375, 0.85); //CP10_P7;
    lawCP10(8) = gp_Pnt(-0.0375, -0.0375, 1.); //CP10_P8;
    Handle(Geom_BSplineCurve) BSplineCP10 = new Geom_BSplineCurve(lawCP10, lawKnots, lawMults, lawDegree, Standard_False);

    //CP11
    TColgp_Array1OfPnt lawCP11(1, 8);
    lawCP11(1) = gp_Pnt(-0.0375, -0.01875, 0.); //CP11_P1;
    lawCP11(2) = gp_Pnt(-0.0375, -0.01875, 0.15); //CP11_P2;
    lawCP11(3) = gp_Pnt(designParameters[80], designParameters[81], 0.30); //CP11_P3;
    lawCP11(4) = gp_Pnt(designParameters[82], designParameters[83], 0.45); //CP11_P4;
    lawCP11(5) = gp_Pnt(designParameters[84], designParameters[85], 0.60); //CP11_P5;
    lawCP11(6) = gp_Pnt(designParameters[86], designParameters[87], 0.75); //CP11_P6;
    lawCP11(7) = gp_Pnt(-0.0375, -0.01875, 0.85); //CP11_P7;
    lawCP11(8) = gp_Pnt(-0.0375, -0.01875, 1.); //CP11_P8;
    Handle(Geom_BSplineCurve) BSplineCP11 = new Geom_BSplineCurve(lawCP11, lawKnots, lawMults, lawDegree, Standard_False);

    //CP12
    TColgp_Array1OfPnt lawCP12(1, 8);
    lawCP12(1) = gp_Pnt(-0.0375, 0.01875, 0.); //CP12_P1;
    lawCP12(2) = gp_Pnt(-0.0375, 0.01875, 0.15); //CP12_P2;
    lawCP12(3) = gp_Pnt(designParameters[88], designParameters[89], 0.30); //CP12_P3;
    lawCP12(4) = gp_Pnt(designParameters[90], designParameters[91], 0.45); //CP12_P4;
    lawCP12(5) = gp_Pnt(designParameters[92], designParameters[93], 0.60); //CP12_P5;
    lawCP12(6) = gp_Pnt(designParameters[94], designParameters[95], 0.75); //CP12_P6;
    lawCP12(7) = gp_Pnt(-0.0375, 0.01875, 0.85); //CP12_P7;
    lawCP12(8) = gp_Pnt(-0.0375, 0.01875, 1.); //CP12_P8;
    Handle(Geom_BSplineCurve) BSplineCP12 = new Geom_BSplineCurve(lawCP12, lawKnots, lawMults, lawDegree, Standard_False);

    //---------------------------------------------------------------------------
    // Stage 2: U-bend: B-Spline path for U-bend
    //---------------------------------------------------------------------------

    /*TColgp_Array1OfPnt pathPoles(1, 9);
    pathPoles(1) = gp_Pnt(-0.150,  0.057,   0.);
    pathPoles(2) = gp_Pnt(-0.075, 0.057,   0.);
    pathPoles(3) = gp_Pnt(0., 0.057,  0.);
    pathPoles(4) = gp_Pnt(0.057, 0.057, 0.);
    pathPoles(5) = gp_Pnt(0.057, 3.49024337756996e-15, 0.);
    pathPoles(6) = gp_Pnt(0.057, -0.057, 0.);
    pathPoles(7) = gp_Pnt(0. , -0.057,   0.);
    pathPoles(8) = gp_Pnt(-0.075, -0.057,   0.);
    pathPoles(9) = gp_Pnt(-0.150, -0.057,   0.);

    TColStd_Array1OfReal pathWeights(1,9);
    pathWeights(1)=1;
    pathWeights(2)=1;
    pathWeights(3)=1;
    pathWeights(4)=0.707106781186548;
    pathWeights(5)=1;
    pathWeights(6)=0.707106781186548;
    pathWeights(7)=1;
    pathWeights(8)=1;
    pathWeights(9)=1;

    TColStd_Array1OfReal pathKnots(1,5);
    pathKnots(1)=0;
    pathKnots(2)=0.25;
    pathKnots(3)=0.5;
    pathKnots(4)=0.75;
    pathKnots(5)=1;

    TColStd_Array1OfInteger pathMults(1,5);
    pathMults(1)=3;
    pathMults(2)=2;
    pathMults(3)=2;
    pathMults(4)=2;
    pathMults(5)=3;

    Standard_Integer pathDegree = 2;

    Handle(Geom_BSplineCurve) path = new Geom_BSplineCurve(pathPoles, pathWeights, pathKnots, pathMults, pathDegree, Standard_False, Standard_True);*/

    /*gp_Pnt O (0,-0.057,0);
    gp_Ax2 xyzAxis (O, -gp::DZ(), -gp::DY());
    Standard_Real r = 0.057;
    Handle(Geom_Circle) Cxyz = new Geom_Circle( xyzAxis, r );

  //

    Handle (Geom_TrimmedCurve) path = new Geom_TrimmedCurve(Cxyz , M_PI, 2*M_PI);*/
    //Handle(Geom_BSplineCurve) path = GeomConvert::CurveToBSplineCurve(path2, Convert_QuasiAngular);

    TColgp_Array1OfPnt pathPoles(1, 9);
    pathPoles(1) = gp_Pnt(-0.150,  0.057,   0.);
    pathPoles(2) = gp_Pnt(-0.075, 0.057,   0.);
    pathPoles(3) = gp_Pnt(0, 0.057,  0.);
    pathPoles(4) = gp_Pnt(0.057, 0.057, 0.);
    pathPoles(5) = gp_Pnt(0.057, 0, 0.);
    pathPoles(6) = gp_Pnt(0.057, -0.057, 0.);
    pathPoles(7) = gp_Pnt(0. , -0.057,   0.);
    pathPoles(8) = gp_Pnt(-0.075, -0.057,   0.);
    pathPoles(9) = gp_Pnt(-0.150, -0.057,   0.);

    TColStd_Array1OfReal pathWeights(1,9);
    pathWeights(1)=1;
    pathWeights(2)=1;
    pathWeights(3)=1;
    pathWeights(4)=0.707106781186548;
    pathWeights(5)=1;
    pathWeights(6)=0.707106781186548;
    pathWeights(7)=1;
    pathWeights(8)=1;
    pathWeights(9)=1;

    TColStd_Array1OfReal pathKnots(1,5);
    pathKnots(1)=0;
    pathKnots(2)=0.25;
    pathKnots(3)=0.5;
    pathKnots(4)=0.75;
    pathKnots(5)=1;

    TColStd_Array1OfInteger pathMults(1,5);
    pathMults(1)=3;
    pathMults(2)=2;
    pathMults(3)=2;
    pathMults(4)=2;
    pathMults(5)=3;

    Standard_Integer pathDegree = 2;

    Handle(Geom_BSplineCurve) path = new Geom_BSplineCurve(pathPoles, pathWeights, pathKnots, pathMults, pathDegree, Standard_False, Standard_True);



    GeomAdaptor_Curve GAC;
    GAC.Load(path);
    Standard_Real Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve

    //---------------------------------------------------------------------------
    // Stage 3: U-bend: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    TopoDS_Edge Edge_B11;
    TopoDS_Edge Edge_B21;
    TopoDS_Edge Edge_B31;
    TopoDS_Edge Edge_B41;
    TopoDS_Shape tube;
    gp_Dir d[maxSlices];
    gp_Dir d2(0,0,1);
    TopoDS_Wire Wire[maxSlices];

    Standard_Real u[maxSlices];

    u[0]             = path->FirstParameter();
    u[maxSlices - 1] = path->LastParameter();

    const Standard_Real ustep = (u[maxSlices - 1] - u[0]) / (maxSlices - 1);

    for ( Standard_Integer i = 1; i < maxSlices - 1; ++i )
    {
        u[i] = u[0] + i*ustep;
    }

    // Prepare skinning tool

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;

    //!!!!!!! Careful - changed
    const Standard_Real    pres3d  = 2.0e-06;

    BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);
    aGenerator.CheckCompatibility(Standard_False);
    //
    for ( Standard_Integer i = 0; i < maxSlices; ++i )
    {

        gp_Pnt P;
        gp_Vec V;
        path->D1(u[i], P, V);
        d[i] = gp_Dir(V);

        Standard_Real P_Length = GCPnts_AbscissaPoint::Length(GAC, u[0], u[i]);//length of the curve

        Standard_Real Length_step = (P_Length/Length);

        // -- Create the intersection plane
        gp_Pnt P_path(0, 0, Length_step);

        gp_Pln constructionPl(P_path, d2);
        Handle(Geom_Plane) constructionPlane = new Geom_Plane (constructionPl);

        //IP1
        GeomAPI_IntCS IntCS1(BSplineCP1, constructionPlane);
        //cout << "Nb of Int1 points is " << IntCS1.NbPoints() << endl;
        //IP2
        GeomAPI_IntCS IntCS2(BSplineCP2, constructionPlane);
        //IP3
        GeomAPI_IntCS IntCS3(BSplineCP3, constructionPlane);
        //IP4
        GeomAPI_IntCS IntCS4(BSplineCP4, constructionPlane);
        //IP5
        GeomAPI_IntCS IntCS5(BSplineCP5, constructionPlane);
        //IP6
        GeomAPI_IntCS IntCS6(BSplineCP6, constructionPlane);
        //IP7
        GeomAPI_IntCS IntCS7(BSplineCP7, constructionPlane);
        //IP8
        GeomAPI_IntCS IntCS8(BSplineCP8, constructionPlane);
        //IP9
        GeomAPI_IntCS IntCS9(BSplineCP9, constructionPlane);
        //IP10
        GeomAPI_IntCS IntCS10(BSplineCP10, constructionPlane);
        //IP11
        GeomAPI_IntCS IntCS11(BSplineCP11, constructionPlane);
        //IP12
        GeomAPI_IntCS IntCS12(BSplineCP12, constructionPlane);

        TColgp_Array1OfPnt2d Bezier11(1, 4);
        Bezier11(1) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Bezier11(2) = gp_Pnt2d(IntCS2.Point(1).X(), IntCS2.Point(1).Y());
        Bezier11(3) = gp_Pnt2d(IntCS3.Point(1).X(), IntCS3.Point(1).Y());
        Bezier11(4) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier1 = new Geom2d_BezierCurve(Bezier11);

        TColgp_Array1OfPnt2d Bezier12(1, 4);
        Bezier12(1) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Bezier12(2) = gp_Pnt2d(IntCS5.Point(1).X(), IntCS5.Point(1).Y());
        Bezier12(3) = gp_Pnt2d(IntCS6.Point(1).X(), IntCS6.Point(1).Y());
        Bezier12(4) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier2 = new Geom2d_BezierCurve(Bezier12);

        TColgp_Array1OfPnt2d Bezier13(1, 4);
        Bezier13(1) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Bezier13(2) = gp_Pnt2d(IntCS8.Point(1).X(), IntCS8.Point(1).Y());
        Bezier13(3) = gp_Pnt2d(IntCS9.Point(1).X(), IntCS9.Point(1).Y());
        Bezier13(4) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier3 = new Geom2d_BezierCurve(Bezier13);

        TColgp_Array1OfPnt2d Bezier14(1, 4);
        Bezier14(1) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Bezier14(2) = gp_Pnt2d(IntCS11.Point(1).X(), IntCS11.Point(1).Y());
        Bezier14(3) = gp_Pnt2d(IntCS12.Point(1).X(), IntCS12.Point(1).Y());
        Bezier14(4) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier4 = new Geom2d_BezierCurve(Bezier14);

        // -- Create the host plane
        gp_Pln Pln(gp_Ax3(P, d[i], d2 ));
        //Handle(Geom_Plane) s = new Geom_Plane (Pl1);

        Edge_B11 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier1, Pln) );
        Edge_B21 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier2, Pln) );
        Edge_B31 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier3, Pln) );
        Edge_B41 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier4, Pln) );

        // Create wire

        BRepBuilderAPI_MakeWire mkWire;
        mkWire.Add(Edge_B11);
        mkWire.Add(Edge_B21);
        mkWire.Add(Edge_B31);
        mkWire.Add(Edge_B41);
        Wire[i] = mkWire.Wire();


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
    }
    //
    if ( !aGenerator.IsDone() )
    {
        std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
    }

    tube = aGenerator.Shape();

    //---------------------------------------------------------------------------
    // Stage 4 - // Inlet and Outlet Pipe
    //---------------------------------------------------------------------------

//    gp_Vec V0(d[0]);
//    BRepPrimAPI_MakePrism mkPrismIn(Wire[0], -maxLengthVerticalPipes*V0);
//    mkPrismIn.Build();
//    TopoDS_Shape InletPipe = mkPrismIn.Shape();
//    // Get the Inlet wire
//    TopoDS_Wire wbaseIn = TopoDS::Wire(mkPrismIn.LastShape());
//    // Create the Inlet face from this wire
//    TopoDS_Face fbaseIn = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseIn)).Face();
//
//    gp_Vec Vmaxslices(d[maxSlices-1]);
//    BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], (0.6/0.67755)*maxLengthVerticalPipes*Vmaxslices);
//    mkPrismOut.Build();
//    TopoDS_Shape OutletPipe = mkPrismOut.Shape();
//    // Get the outlet wire
//    TopoDS_Wire wbaseOut = TopoDS::Wire(mkPrismOut.LastShape());
//    // Create the outlet face from this wire
//    TopoDS_Face fbaseOut = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseOut)).Face();
//
//    //---------------------------------------------------------------------------
//    // Stage 5 - // Build the shell or solid
//    //---------------------------------------------------------------------------
//
//    BRepBuilderAPI_Sewing Sewing;
//    Sewing.Add(fbaseIn);
//    Sewing.Add(InletPipe);
//    Sewing.Add(tube);
//    Sewing.Add(OutletPipe);
//    Sewing.Add(fbaseOut);
//    Sewing.Perform();
//    tube = Sewing.SewedShape();
//    tube = TopoDS::Shell(tube);
    return tube;
}


//December 1st
TopoDS_Shape OCCTDataProvider::ConstructSquaredUbend_December1st(bool activateDesignParametersForReverse)
{
#if REVERSE_MODE
    if(activateDesignParametersForReverse)
    {
      for(int cnt = 0; cnt < nParams; cnt++)
        designParameters[cnt] <<= designParameters[cnt].getValue();
    }
#endif
    int maxSlices = 70;

    double maxLengthVerticalPipes = 0.67755;
    //---------------------------------------------------------------------------
    // Stage 1: Construct laws from design parameters
    //---------------------------------------------------------------------------

    //Knots, Multipl and Degree of the laws

    TColStd_Array1OfReal lawKnots(1,6);
    lawKnots(1)=0.;
    lawKnots(2)=0.2;
    lawKnots(3)=0.4;
    lawKnots(4)=0.6;
    lawKnots(5)=0.8;
    lawKnots(6)=1;

    TColStd_Array1OfInteger lawMults(1,6);
    lawMults(1)=4;
    lawMults(2)=1;
    lawMults(3)=1;
    lawMults(4)=1;
    lawMults(5)=1;
    lawMults(6)=4;

    Standard_Integer lawDegree = 3;

    //CP1
    TColgp_Array1OfPnt lawCP1(1, 8);
    lawCP1(1) = gp_Pnt(-0.0375, 0.0375, 0.); //CP1_P1;
    lawCP1(2) = gp_Pnt(-0.0375, 0.0375, 0.15); //CP1_P2;
    lawCP1(3) = gp_Pnt(designParameters[0], designParameters[1], 0.30); //CP1_P3;
    lawCP1(4) = gp_Pnt(designParameters[2], designParameters[3], 0.45); //CP1_P4;
    lawCP1(5) = gp_Pnt(designParameters[4], designParameters[5], 0.60); //CP1_P5;
    lawCP1(6) = gp_Pnt(designParameters[6], designParameters[7], 0.75); //CP1_P6;
    lawCP1(7) = gp_Pnt(-0.0375, 0.0375, 0.85); //CP1_P7;
    lawCP1(8) = gp_Pnt(-0.0375, 0.0375, 1); //CP1_P8;
    Handle(Geom_BSplineCurve) BSplineCP1 = new Geom_BSplineCurve(lawCP1, lawKnots, lawMults, lawDegree, Standard_False);

    //CP2
    TColgp_Array1OfPnt lawCP2(1, 8);
    lawCP2(1) = gp_Pnt(-0.01875, 0.0375, 0.); //CP2_P1;
    lawCP2(2) = gp_Pnt(-0.01875, 0.0375, 0.15); //CP2_P2;
    lawCP2(3) = gp_Pnt(designParameters[8], designParameters[9], 0.30); //CP2_P3;
    lawCP2(4) = gp_Pnt(designParameters[10], designParameters[11], 0.45); //CP2_P4;
    lawCP2(5) = gp_Pnt(designParameters[12], designParameters[13], 0.60); //CP2_P5;
    lawCP2(6) = gp_Pnt(designParameters[14], designParameters[15], 0.75); //CP2_P6;
    lawCP2(7) = gp_Pnt(-0.01875, 0.0375, 0.85); //CP2_P7;
    lawCP2(8) = gp_Pnt(-0.01875, 0.0375, 1.); //CP2_P8;
    Handle(Geom_BSplineCurve) BSplineCP2 = new Geom_BSplineCurve(lawCP2, lawKnots, lawMults, lawDegree, Standard_False);

    //CP3
    TColgp_Array1OfPnt lawCP3(1, 8);
    lawCP3(1) = gp_Pnt(0.01875, 0.0375, 0.); //CP3_P1;
    lawCP3(2) = gp_Pnt(0.01875, 0.0375, 0.15); //CP3_P2;
    lawCP3(3) = gp_Pnt(designParameters[16], designParameters[17], 0.30); //CP3_P3;
    lawCP3(4) = gp_Pnt(designParameters[18], designParameters[19], 0.45); //CP3_P4;
    lawCP3(5) = gp_Pnt(designParameters[20], designParameters[21], 0.60); //CP3_P5;
    lawCP3(6) = gp_Pnt(designParameters[22], designParameters[23], 0.75); //CP3_P6;
    lawCP3(7) = gp_Pnt(0.01875, 0.0375, 0.85); //CP3_P7;
    lawCP3(8) = gp_Pnt(0.01875, 0.0375, 1.); //CP3_P8;
    Handle(Geom_BSplineCurve) BSplineCP3 = new Geom_BSplineCurve(lawCP3, lawKnots, lawMults, lawDegree, Standard_False);

    //CP4
    TColgp_Array1OfPnt lawCP4(1, 8);
    lawCP4(1) = gp_Pnt(0.0375, 0.0375, 0.); //CP4_P1;
    lawCP4(2) = gp_Pnt(0.0375, 0.0375, 0.15); //CP4_P2;
    lawCP4(3) = gp_Pnt(designParameters[24], designParameters[25], 0.30); //CP4_P3;
    lawCP4(4) = gp_Pnt(designParameters[26], designParameters[27], 0.45); //CP4_P4;
    lawCP4(5) = gp_Pnt(designParameters[28], designParameters[29], 0.60); //CP4_P5;
    lawCP4(6) = gp_Pnt(designParameters[30], designParameters[31], 0.75); //CP4_P6;
    lawCP4(7) = gp_Pnt(0.0375, 0.0375, 0.85); //CP4_P7;
    lawCP4(8) = gp_Pnt(0.0375, 0.0375, 1.); //CP4_P8;
    Handle(Geom_BSplineCurve) BSplineCP4 = new Geom_BSplineCurve(lawCP4, lawKnots, lawMults, lawDegree, Standard_False);

    //CP5
    TColgp_Array1OfPnt lawCP5(1, 8);
    lawCP5(1) = gp_Pnt(0.0375, 0.01875, 0.); //CP5_P1;
    lawCP5(2) = gp_Pnt(0.0375, 0.01875, 0.15); //CP5_P2;
    lawCP5(3) = gp_Pnt(designParameters[32], designParameters[33], 0.30); //CP5_P3;
    lawCP5(4) = gp_Pnt(designParameters[34], designParameters[35], 0.45); //CP5_P4;
    lawCP5(5) = gp_Pnt(designParameters[36], designParameters[37], 0.60); //CP5_P5;
    lawCP5(6) = gp_Pnt(designParameters[38], designParameters[39], 0.75); //CP5_P6;
    lawCP5(7) = gp_Pnt(0.0375, 0.01875, 0.85); //CP5_P7;
    lawCP5(8) = gp_Pnt(0.0375, 0.01875, 1.); //CP5_P8;
    Handle(Geom_BSplineCurve) BSplineCP5 = new Geom_BSplineCurve(lawCP5, lawKnots, lawMults, lawDegree, Standard_False);

    //CP6
    TColgp_Array1OfPnt lawCP6(1, 8);
    lawCP6(1) = gp_Pnt(0.0375,-0.01875, 0.); //CP6_P1;
    lawCP6(2) = gp_Pnt(0.0375,-0.01875, 0.15); //CP6_P2;
    lawCP6(3) = gp_Pnt(designParameters[40], designParameters[41], 0.30); //CP6_P3;
    lawCP6(4) = gp_Pnt(designParameters[42], designParameters[43], 0.45); //CP6_P4;
    lawCP6(5) = gp_Pnt(designParameters[44], designParameters[45], 0.60); //CP6_P5;
    lawCP6(6) = gp_Pnt(designParameters[46], designParameters[47], 0.75); //CP6_P6;
    lawCP6(7) = gp_Pnt(0.0375,-0.01875, 0.85); //CP6_P7;
    lawCP6(8) = gp_Pnt(0.0375,-0.01875, 1.); //CP6_P8;
    Handle(Geom_BSplineCurve) BSplineCP6 = new Geom_BSplineCurve(lawCP6, lawKnots, lawMults, lawDegree, Standard_False);

    //CP7
    TColgp_Array1OfPnt lawCP7(1, 8);
    lawCP7(1) = gp_Pnt(0.0375, -0.0375, 0.); //CP7_P1;
    lawCP7(2) = gp_Pnt(0.0375, -0.0375, 0.15); //CP7_P2;
    lawCP7(3) = gp_Pnt(designParameters[48], designParameters[49], 0.30); //CP7_P3;
    lawCP7(4) = gp_Pnt(designParameters[50], designParameters[51], 0.45); //CP7_P4;
    lawCP7(5) = gp_Pnt(designParameters[52], designParameters[53], 0.60); //CP7_P5;
    lawCP7(6) = gp_Pnt(designParameters[54], designParameters[55], 0.75); //CP7_P6;
    lawCP7(7) = gp_Pnt(0.0375, -0.0375, 0.85); //CP7_P7;
    lawCP7(8) = gp_Pnt(0.0375, -0.0375, 1.); //CP7_P8;
    Handle(Geom_BSplineCurve) BSplineCP7 = new Geom_BSplineCurve(lawCP7, lawKnots, lawMults, lawDegree, Standard_False);

    //CP8
    TColgp_Array1OfPnt lawCP8(1, 8);
    lawCP8(1) = gp_Pnt(0.01875, -0.0375, 0.); //CP8_P1;
    lawCP8(2) = gp_Pnt(0.01875, -0.0375, 0.15); //CP8_P2;
    lawCP8(3) = gp_Pnt(designParameters[56], designParameters[57], 0.30); //CP8_P3;
    lawCP8(4) = gp_Pnt(designParameters[58], designParameters[59], 0.45); //CP8_P4;
    lawCP8(5) = gp_Pnt(designParameters[60], designParameters[61], 0.60); //CP8_P5;
    lawCP8(6) = gp_Pnt(designParameters[62], designParameters[63], 0.75); //CP8_P6;
    lawCP8(7) = gp_Pnt(0.01875, -0.0375, 0.85); //CP8_P7;
    lawCP8(8) = gp_Pnt(0.01875, -0.0375, 1); //CP8_P8;
    Handle(Geom_BSplineCurve) BSplineCP8 = new Geom_BSplineCurve(lawCP8, lawKnots, lawMults, lawDegree, Standard_False);

    //CP9
    TColgp_Array1OfPnt lawCP9(1, 8);
    lawCP9(1) = gp_Pnt(-0.01875,-0.0375, 0.); //CP9_P1;
    lawCP9(2) = gp_Pnt(-0.01875, -0.0375, 0.15); //CP9_P2;
    lawCP9(3) = gp_Pnt(designParameters[64], designParameters[65], 0.30); //CP9_P3;
    lawCP9(4) = gp_Pnt(designParameters[66], designParameters[67], 0.45); //CP9_P4;
    lawCP9(5) = gp_Pnt(designParameters[68], designParameters[69], 0.60); //CP9_P5;
    lawCP9(6) = gp_Pnt(designParameters[70], designParameters[71], 0.75); //CP9_P6;
    lawCP9(7) = gp_Pnt(-0.01875, -0.0375, 0.85); //CP9_P7;
    lawCP9(8) = gp_Pnt(-0.01875, -0.0375, 1.); //CP9_P8;
    Handle(Geom_BSplineCurve) BSplineCP9 = new Geom_BSplineCurve(lawCP9, lawKnots, lawMults, lawDegree, Standard_False);

    //CP10
    TColgp_Array1OfPnt lawCP10(1, 8);
    lawCP10(1) = gp_Pnt(-0.0375, -0.0375, 0.); //CP10_P1;
    lawCP10(2) = gp_Pnt(-0.0375, -0.0375, 0.15); //CP10_P2;
    lawCP10(3) = gp_Pnt(designParameters[72], designParameters[73], 0.30); //CP10_P3;
    lawCP10(4) = gp_Pnt(designParameters[74], designParameters[75], 0.45); //CP10_P4;
    lawCP10(5) = gp_Pnt(designParameters[76], designParameters[77], 0.60); //CP10_P5;
    lawCP10(6) = gp_Pnt(designParameters[78], designParameters[79], 0.75); //CP10_P6;
    lawCP10(7) = gp_Pnt(-0.0375, -0.0375, 0.85); //CP10_P7;
    lawCP10(8) = gp_Pnt(-0.0375, -0.0375, 1.); //CP10_P8;
    Handle(Geom_BSplineCurve) BSplineCP10 = new Geom_BSplineCurve(lawCP10, lawKnots, lawMults, lawDegree, Standard_False);

    //CP11
    TColgp_Array1OfPnt lawCP11(1, 8);
    lawCP11(1) = gp_Pnt(-0.0375, -0.01875, 0.); //CP11_P1;
    lawCP11(2) = gp_Pnt(-0.0375, -0.01875, 0.15); //CP11_P2;
    lawCP11(3) = gp_Pnt(designParameters[80], designParameters[81], 0.30); //CP11_P3;
    lawCP11(4) = gp_Pnt(designParameters[82], designParameters[83], 0.45); //CP11_P4;
    lawCP11(5) = gp_Pnt(designParameters[84], designParameters[85], 0.60); //CP11_P5;
    lawCP11(6) = gp_Pnt(designParameters[86], designParameters[87], 0.75); //CP11_P6;
    lawCP11(7) = gp_Pnt(-0.0375, -0.01875, 0.85); //CP11_P7;
    lawCP11(8) = gp_Pnt(-0.0375, -0.01875, 1.); //CP11_P8;
    Handle(Geom_BSplineCurve) BSplineCP11 = new Geom_BSplineCurve(lawCP11, lawKnots, lawMults, lawDegree, Standard_False);

    //CP12
    TColgp_Array1OfPnt lawCP12(1, 8);
    lawCP12(1) = gp_Pnt(-0.0375, 0.01875, 0.); //CP12_P1;
    lawCP12(2) = gp_Pnt(-0.0375, 0.01875, 0.15); //CP12_P2;
    lawCP12(3) = gp_Pnt(designParameters[88], designParameters[89], 0.30); //CP12_P3;
    lawCP12(4) = gp_Pnt(designParameters[90], designParameters[91], 0.45); //CP12_P4;
    lawCP12(5) = gp_Pnt(designParameters[92], designParameters[93], 0.60); //CP12_P5;
    lawCP12(6) = gp_Pnt(designParameters[94], designParameters[95], 0.75); //CP12_P6;
    lawCP12(7) = gp_Pnt(-0.0375, 0.01875, 0.85); //CP12_P7;
    lawCP12(8) = gp_Pnt(-0.0375, 0.01875, 1.); //CP12_P8;
    Handle(Geom_BSplineCurve) BSplineCP12 = new Geom_BSplineCurve(lawCP12, lawKnots, lawMults, lawDegree, Standard_False);

    //---------------------------------------------------------------------------
    // Stage 2: U-bend: B-Spline path for U-bend
    //---------------------------------------------------------------------------

    TColgp_Array1OfPnt pathPoles(1, 9);
    pathPoles(1) = gp_Pnt(-0.150,  0.057,   0.);
    pathPoles(2) = gp_Pnt(-0.075, 0.057,   0.);
    pathPoles(3) = gp_Pnt(0, 0.057,  0.);
    pathPoles(4) = gp_Pnt(0.057, 0.057, 0.);
    pathPoles(5) = gp_Pnt(0.057, 0, 0.);
    pathPoles(6) = gp_Pnt(0.057, -0.057, 0.);
    pathPoles(7) = gp_Pnt(0. , -0.057,   0.);
    pathPoles(8) = gp_Pnt(-0.075, -0.057,   0.);
    pathPoles(9) = gp_Pnt(-0.150, -0.057,   0.);

    TColStd_Array1OfReal pathWeights(1,9);
    pathWeights(1)=1;
    pathWeights(2)=1;
    pathWeights(3)=1;
    pathWeights(4)=0.707106781186548;
    pathWeights(5)=1;
    pathWeights(6)=0.707106781186548;
    pathWeights(7)=1;
    pathWeights(8)=1;
    pathWeights(9)=1;

    TColStd_Array1OfReal pathKnots(1,5);
    pathKnots(1)=0;
    pathKnots(2)=0.25;
    pathKnots(3)=0.5;
    pathKnots(4)=0.75;
    pathKnots(5)=1;

    TColStd_Array1OfInteger pathMults(1,5);
    pathMults(1)=3;
    pathMults(2)=2;
    pathMults(3)=2;
    pathMults(4)=2;
    pathMults(5)=3;

    Standard_Integer pathDegree = 2;

    Handle(Geom_BSplineCurve) path = new Geom_BSplineCurve(pathPoles, pathWeights, pathKnots, pathMults, pathDegree, Standard_False, Standard_True);



    GeomAdaptor_Curve GAC;
    GAC.Load(path);
    Standard_Real Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve

    //---------------------------------------------------------------------------
    // Stage 3: U-bend: Define Sections of the Ubend
    //---------------------------------------------------------------------------

    TopoDS_Edge Edge_B11;
    TopoDS_Edge Edge_B21;
    TopoDS_Edge Edge_B31;
    TopoDS_Edge Edge_B41;
    TopoDS_Shape tube;
    gp_Dir d[maxSlices+1];
    gp_Dir d2(0,0,1);
    TopoDS_Wire Wire[maxSlices+1];

    GCPnts_UniformAbscissa pathDiscretizer (GAC, maxSlices, -1);

    // Prepare skinning tool

    const Standard_Boolean isSolid = Standard_False;
    const Standard_Boolean isRuled = Standard_False;

    //!!!!!!! Careful - changed
    const Standard_Real    pres3d  = 2.0e-06;

    BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
    aGenerator.SetSmoothing(Standard_True);
    aGenerator.SetMaxDegree(5);
    aGenerator.CheckCompatibility(Standard_False);
    //
    for ( Standard_Integer i = 1; i <= maxSlices; ++i )
    {

        gp_Pnt P;
        gp_Vec V;
        path->D1(pathDiscretizer.Parameter(i), P, V);
        d[i] = gp_Dir(V);

        Standard_Real P_Length = GCPnts_AbscissaPoint::Length(GAC, pathDiscretizer.Parameter(1), pathDiscretizer.Parameter(i));//length of the curve

        Standard_Real Length_step = (P_Length/Length);

        // -- Create the intersection plane
        gp_Pnt P_path(0, 0, Length_step);

        gp_Pln constructionPl(P_path, d2);
        Handle(Geom_Plane) constructionPlane = new Geom_Plane (constructionPl);

        //IP1
        GeomAPI_IntCS IntCS1(BSplineCP1, constructionPlane);
        //cout << "Nb of Int1 points is " << IntCS1.NbPoints() << endl;
        //IP2
        GeomAPI_IntCS IntCS2(BSplineCP2, constructionPlane);
        //IP3
        GeomAPI_IntCS IntCS3(BSplineCP3, constructionPlane);
        //IP4
        GeomAPI_IntCS IntCS4(BSplineCP4, constructionPlane);
        //IP5
        GeomAPI_IntCS IntCS5(BSplineCP5, constructionPlane);
        //IP6
        GeomAPI_IntCS IntCS6(BSplineCP6, constructionPlane);
        //IP7
        GeomAPI_IntCS IntCS7(BSplineCP7, constructionPlane);
        //IP8
        GeomAPI_IntCS IntCS8(BSplineCP8, constructionPlane);
        //IP9
        GeomAPI_IntCS IntCS9(BSplineCP9, constructionPlane);
        //IP10
        GeomAPI_IntCS IntCS10(BSplineCP10, constructionPlane);
        //IP11
        GeomAPI_IntCS IntCS11(BSplineCP11, constructionPlane);
        //IP12
        GeomAPI_IntCS IntCS12(BSplineCP12, constructionPlane);

        TColgp_Array1OfPnt2d Bezier11(1, 4);
        Bezier11(1) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Bezier11(2) = gp_Pnt2d(IntCS2.Point(1).X(), IntCS2.Point(1).Y());
        Bezier11(3) = gp_Pnt2d(IntCS3.Point(1).X(), IntCS3.Point(1).Y());
        Bezier11(4) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier1 = new Geom2d_BezierCurve(Bezier11);

        TColgp_Array1OfPnt2d Bezier12(1, 4);
        Bezier12(1) = gp_Pnt2d(IntCS4.Point(1).X(), IntCS4.Point(1).Y());
        Bezier12(2) = gp_Pnt2d(IntCS5.Point(1).X(), IntCS5.Point(1).Y());
        Bezier12(3) = gp_Pnt2d(IntCS6.Point(1).X(), IntCS6.Point(1).Y());
        Bezier12(4) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier2 = new Geom2d_BezierCurve(Bezier12);

        TColgp_Array1OfPnt2d Bezier13(1, 4);
        Bezier13(1) = gp_Pnt2d(IntCS7.Point(1).X(), IntCS7.Point(1).Y());
        Bezier13(2) = gp_Pnt2d(IntCS8.Point(1).X(), IntCS8.Point(1).Y());
        Bezier13(3) = gp_Pnt2d(IntCS9.Point(1).X(), IntCS9.Point(1).Y());
        Bezier13(4) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier3 = new Geom2d_BezierCurve(Bezier13);

        TColgp_Array1OfPnt2d Bezier14(1, 4);
        Bezier14(1) = gp_Pnt2d(IntCS10.Point(1).X(), IntCS10.Point(1).Y());
        Bezier14(2) = gp_Pnt2d(IntCS11.Point(1).X(), IntCS11.Point(1).Y());
        Bezier14(3) = gp_Pnt2d(IntCS12.Point(1).X(), IntCS12.Point(1).Y());
        Bezier14(4) = gp_Pnt2d(IntCS1.Point(1).X(), IntCS1.Point(1).Y());
        Handle(Geom2d_BezierCurve) Bezier4 = new Geom2d_BezierCurve(Bezier14);

        // -- Create the host plane
        gp_Pln Pln(gp_Ax3(P, d[i], d2 ));
        //Handle(Geom_Plane) s = new Geom_Plane (Pl1);

        Edge_B11 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier1, Pln) );
        Edge_B21 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier2, Pln) );
        Edge_B31 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier3, Pln) );
        Edge_B41 = BRepBuilderAPI_MakeEdge( GeomAPI::To3d(Bezier4, Pln) );

        // Create wire

        BRepBuilderAPI_MakeWire mkWire;
        mkWire.Add(Edge_B11);
        mkWire.Add(Edge_B21);
        mkWire.Add(Edge_B31);
        mkWire.Add(Edge_B41);
        Wire[i] = mkWire.Wire();

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
    }
    //
    if ( !aGenerator.IsDone() )
    {
        std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
    }

    tube = aGenerator.Shape();

    //---------------------------------------------------------------------------
    // Stage 4 - // Inlet and Outlet Pipe
    //---------------------------------------------------------------------------

//    gp_Vec V0(d[0]);
//    BRepPrimAPI_MakePrism mkPrismIn(Wire[0], -maxLengthVerticalPipes*V0);
//    mkPrismIn.Build();
//    TopoDS_Shape InletPipe = mkPrismIn.Shape();
//    // Get the Inlet wire
//    TopoDS_Wire wbaseIn = TopoDS::Wire(mkPrismIn.LastShape());
//    // Create the Inlet face from this wire
//    TopoDS_Face fbaseIn = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseIn)).Face();
//
//    gp_Vec Vmaxslices(d[maxSlices-1]);
//    BRepPrimAPI_MakePrism mkPrismOut(Wire[maxSlices-1], (0.6/0.67755)*maxLengthVerticalPipes*Vmaxslices);
//    mkPrismOut.Build();
//    TopoDS_Shape OutletPipe = mkPrismOut.Shape();
//    // Get the outlet wire
//    TopoDS_Wire wbaseOut = TopoDS::Wire(mkPrismOut.LastShape());
//    // Create the outlet face from this wire
//    TopoDS_Face fbaseOut = BRepBuilderAPI_MakeFace(TopoDS::Wire(wbaseOut)).Face();
//
//    //---------------------------------------------------------------------------
//    // Stage 5 - // Build the shell or solid
//    //---------------------------------------------------------------------------
//
//    BRepBuilderAPI_Sewing Sewing;
//    Sewing.Add(fbaseIn);
//    Sewing.Add(InletPipe);
//    Sewing.Add(tube);
//    Sewing.Add(OutletPipe);
//    Sewing.Add(fbaseOut);
//    Sewing.Perform();
//    tube = Sewing.SewedShape();
//    tube = TopoDS::Shell(tube);
    return tube;
}

/*
 * *************************************************************************************************************************
 * ********************************************* TU BERLIN TURBOLAB STATOR *************************************************
 * *************************************************************************************************************************
 */
TopoDS_Shape OCCTDataProvider::ConstructTUBstatorWithRealDimensions_December14th(bool activateDesignParametersForReverse)
{
#if REVERSE_MODE
  if(activateDesignParametersForReverse)
  {
    for(int cnt = 0; cnt < nParams; cnt++)
      designParameters[cnt] <<= designParameters[cnt].getValue();
  }
#endif

   const int maxSlices = 70;

   //---------------------------------------------------------------------------
   // Stage 1: Construct laws from design parameters
   //---------------------------------------------------------------------------

   //Knots, Multipl and Degree of the laws

   TColStd_Array1OfReal lawKnots(1,6);
   lawKnots(1)=0.;
   lawKnots(2)=0.2;
   lawKnots(3)=0.40;
   lawKnots(4)=0.60;
   lawKnots(5)=0.80;
   lawKnots(6)=1.;

   TColStd_Array1OfInteger lawMults(1,6);
   lawMults(1)=4;
   lawMults(2)=1;
   lawMults(3)=1;
   lawMults(4)=1;
   lawMults(5)=1;
   lawMults(6)=4;

   Standard_Integer lawDegree = 3;

   //CP1
   TColgp_Array1OfPnt controlPoints2law(1, 8);
   controlPoints2law(1) = gp_Pnt(designParameters[0], 1., 0.); //CP1_P3;
   controlPoints2law(2) = gp_Pnt(designParameters[1], 1., 0.15); //CP1_P4;
   controlPoints2law(3) = gp_Pnt(designParameters[2], 1., 0.30); //CP1_P5;
   controlPoints2law(4) = gp_Pnt(designParameters[3], 1., 0.45); //CP1_P6;
   controlPoints2law(5) = gp_Pnt(designParameters[4], 1., 0.6); //CP1_P6;
   controlPoints2law(6) = gp_Pnt(designParameters[5], 1., 0.75); //CP1_P6;
   controlPoints2law(7) = gp_Pnt(designParameters[6], 1., 0.9); //CP1_P6;
   controlPoints2law(8) = gp_Pnt(designParameters[7], 1., 1.); //CP1_P6;
   Handle(Geom_BSplineCurve) controlPoints2 = new Geom_BSplineCurve(controlPoints2law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP2
   TColgp_Array1OfPnt leadingEdgeRadiusLaw(1, 8);
   leadingEdgeRadiusLaw(1) = gp_Pnt(designParameters[8], 2., 0.); //CP2_P3;
   leadingEdgeRadiusLaw(2) = gp_Pnt(designParameters[9], 2., 0.15); //CP2_P4;
   leadingEdgeRadiusLaw(3) = gp_Pnt(designParameters[10], 2., 0.30); //CP2_P5;
   leadingEdgeRadiusLaw(4) = gp_Pnt(designParameters[11], 2., 0.45); //CP2_P6;
   leadingEdgeRadiusLaw(5) = gp_Pnt(designParameters[12], 2., 0.60); //CP2_P6;
   leadingEdgeRadiusLaw(6) = gp_Pnt(designParameters[13], 2., 0.75); //CP2_P6;
   leadingEdgeRadiusLaw(7) = gp_Pnt(designParameters[14], 2., 0.90); //CP2_P6;
   leadingEdgeRadiusLaw(8) = gp_Pnt(designParameters[15], 2., 1.); //CP2_P6;
   Handle(Geom_BSplineCurve) leadingEdgeRadius = new Geom_BSplineCurve(leadingEdgeRadiusLaw, lawKnots, lawMults, lawDegree, Standard_False);

   //CP3
   TColgp_Array1OfPnt controlPoints3law(1, 8);
   controlPoints3law(1) = gp_Pnt(designParameters[16], 3., 0.); //CP3_P3;
   controlPoints3law(2) = gp_Pnt(designParameters[17], 3., 0.15); //CP3_P4;
   controlPoints3law(3) = gp_Pnt(designParameters[18], 3., 0.30); //CP3_P5;
   controlPoints3law(4) = gp_Pnt(designParameters[19], 3., 0.45); //CP3_P6;
   controlPoints3law(5) = gp_Pnt(designParameters[20], 3., 0.6); //CP3_P6;
   controlPoints3law(6) = gp_Pnt(designParameters[21], 3., 0.75); //CP3_P6;
   controlPoints3law(7) = gp_Pnt(designParameters[22], 3., 0.9); //CP3_P6;
   controlPoints3law(8) = gp_Pnt(designParameters[23], 3., 1.); //CP3_P6;
   Handle(Geom_BSplineCurve) controlPoints3 = new Geom_BSplineCurve(controlPoints3law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP4
   TColgp_Array1OfPnt controlPoints4law(1, 8);
   controlPoints4law(1) = gp_Pnt(designParameters[24], 4., 0.); //CP4_P3;
   controlPoints4law(2) = gp_Pnt(designParameters[25], 4., 0.15); //CP4_P4;
   controlPoints4law(3) = gp_Pnt(designParameters[26], 4., 0.30); //CP4_P5;
   controlPoints4law(4) = gp_Pnt(designParameters[27], 4., 0.45); //CP4_P6;
   controlPoints4law(5) = gp_Pnt(designParameters[28], 4., 0.6); //CP4_P6;
   controlPoints4law(6) = gp_Pnt(designParameters[29], 4., 0.75); //CP4_P6;
   controlPoints4law(7) = gp_Pnt(designParameters[30], 4., 0.9); //CP4_P6;
   controlPoints4law(8) = gp_Pnt(designParameters[31], 4., 1.); //CP4_P6;
   Handle(Geom_BSplineCurve) controlPoints4 = new Geom_BSplineCurve(controlPoints4law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP5
   TColgp_Array1OfPnt controlPoints5law(1, 8);
   controlPoints5law(1) = gp_Pnt(designParameters[32], 5., 0.); //CP5_P3;
   controlPoints5law(2) = gp_Pnt(designParameters[33], 5., 0.15); //CP5_P4;
   controlPoints5law(3) = gp_Pnt(designParameters[34], 5., 0.30); //CP5_P5;
   controlPoints5law(4) = gp_Pnt(designParameters[35], 5., 0.45); //CP5_P6;
   controlPoints5law(5) = gp_Pnt(designParameters[36], 5., 0.60); //CP5_P6;
   controlPoints5law(6) = gp_Pnt(designParameters[37], 5., 0.75); //CP5_P6;
   controlPoints5law(7) = gp_Pnt(designParameters[38], 5., 0.9); //CP5_P6;
   controlPoints5law(8) = gp_Pnt(designParameters[39], 5., 1.); //CP5_P6;
   Handle(Geom_BSplineCurve) controlPoints5 = new Geom_BSplineCurve(controlPoints5law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP6
   TColgp_Array1OfPnt controlPoints6law(1, 8);
   controlPoints6law(1) = gp_Pnt(designParameters[40], 6., 0.); //CP6_P3;
   controlPoints6law(2) = gp_Pnt(designParameters[41], 6., 0.15); //CP6_P4;
   controlPoints6law(3) = gp_Pnt(designParameters[42], 6., 0.30); //CP6_P5;
   controlPoints6law(4) = gp_Pnt(designParameters[43], 6., 0.45); //CP6_P6;
   controlPoints6law(5) = gp_Pnt(designParameters[44], 6., 0.6); //CP6_P6;
   controlPoints6law(6) = gp_Pnt(designParameters[45], 6., 0.75); //CP6_P6;
   controlPoints6law(7) = gp_Pnt(designParameters[46], 6., 0.9); //CP6_P6;
   controlPoints6law(8) = gp_Pnt(designParameters[47], 6., 1.); //CP6_P6;
   Handle(Geom_BSplineCurve) controlPoints6 = new Geom_BSplineCurve(controlPoints6law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP7
   TColgp_Array1OfPnt controlPoints7law(1, 8);
   controlPoints7law(1) = gp_Pnt(designParameters[48], 7., 0.); //CP7_P3;
   controlPoints7law(2) = gp_Pnt(designParameters[49], 7., 0.15); //CP7_P4;
   controlPoints7law(3) = gp_Pnt(designParameters[50], 7., 0.30); //CP7_P5;
   controlPoints7law(4) = gp_Pnt(designParameters[51], 7., 0.45); //CP7_P6;
   controlPoints7law(5) = gp_Pnt(designParameters[52], 7., 0.6); //CP7_P6;
   controlPoints7law(6) = gp_Pnt(designParameters[53], 7., 0.75); //CP7_P6;
   controlPoints7law(7) = gp_Pnt(designParameters[54], 7., 0.9); //CP7_P6;
   controlPoints7law(8) = gp_Pnt(designParameters[55], 7., 1.); //CP7_P6;
   Handle(Geom_BSplineCurve) controlPoints7 = new Geom_BSplineCurve(controlPoints7law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP8
   TColgp_Array1OfPnt controlPoints8law(1, 8);
   controlPoints8law(1) = gp_Pnt(designParameters[56], 8., 0.); //CP8_P3;
   controlPoints8law(2) = gp_Pnt(designParameters[57], 8., 0.15); //CP8_P4;
   controlPoints8law(3) = gp_Pnt(designParameters[58], 8., 0.30); //CP8_P5;
   controlPoints8law(4) = gp_Pnt(designParameters[59], 8., 0.45); //CP8_P6;
   controlPoints8law(5) = gp_Pnt(designParameters[60], 8., 0.60); //CP8_P6;
   controlPoints8law(6) = gp_Pnt(designParameters[61], 8., 0.75); //CP8_P6;
   controlPoints8law(7) = gp_Pnt(designParameters[62], 8., 0.9); //CP8_P6;
   controlPoints8law(8) = gp_Pnt(designParameters[63], 8., 1.); //CP8_P6;
   Handle(Geom_BSplineCurve) controlPoints8 = new Geom_BSplineCurve(controlPoints8law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP9
   TColgp_Array1OfPnt controlPoints9law(1, 8);
   controlPoints9law(1) = gp_Pnt(designParameters[64], 9., 0.); //CP9_P3;
   controlPoints9law(2) = gp_Pnt(designParameters[65], 9., 0.15); //CP9_P4;
   controlPoints9law(3) = gp_Pnt(designParameters[66], 9., 0.30); //CP9_P5;
   controlPoints9law(4) = gp_Pnt(designParameters[67], 9., 0.45); //CP9_P6;
   controlPoints9law(5) = gp_Pnt(designParameters[68], 9., 0.60); //CP9_P6;
   controlPoints9law(6) = gp_Pnt(designParameters[69], 9., 0.75); //CP9_P6;
   controlPoints9law(7) = gp_Pnt(designParameters[70], 9., 0.9); //CP9_P6;
   controlPoints9law(8) = gp_Pnt(designParameters[71], 9., 1.); //CP9_P6;
   Handle(Geom_BSplineCurve) controlPoints9 = new Geom_BSplineCurve(controlPoints9law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP10
   TColgp_Array1OfPnt trailingEdgeRadiusLaw(1, 8);
   trailingEdgeRadiusLaw(1) = gp_Pnt(designParameters[72], 10., 0.); //CP10_P3;
   trailingEdgeRadiusLaw(2) = gp_Pnt(designParameters[73], 10., 0.15); //CP10_P4;
   trailingEdgeRadiusLaw(3) = gp_Pnt(designParameters[74], 10., 0.30); //CP10_P5;
   trailingEdgeRadiusLaw(4) = gp_Pnt(designParameters[75], 10., 0.45); //CP10_P6;
   trailingEdgeRadiusLaw(5) = gp_Pnt(designParameters[76], 10., 0.60); //CP10_P6;
   trailingEdgeRadiusLaw(6) = gp_Pnt(designParameters[77], 10., 0.75); //CP10_P6;
   trailingEdgeRadiusLaw(7) = gp_Pnt(designParameters[78], 10., 0.90); //CP10_P6;
   trailingEdgeRadiusLaw(8) = gp_Pnt(designParameters[79], 10., 1.); //CP10_P6;
   Handle(Geom_BSplineCurve) trailingEdgeRadius = new Geom_BSplineCurve(trailingEdgeRadiusLaw, lawKnots, lawMults, lawDegree, Standard_False);

   //CP11
   TColgp_Array1OfPnt camberlineCP1law(1, 8);
   camberlineCP1law(1) = gp_Pnt(designParameters[80], designParameters[81], 0.); //CP12_P3;
   camberlineCP1law(2) = gp_Pnt(designParameters[82], designParameters[83], 0.15); //CP12_P4;
   camberlineCP1law(3) = gp_Pnt(designParameters[84], designParameters[85], 0.30); //CP12_P5;
   camberlineCP1law(4) = gp_Pnt(designParameters[86], designParameters[87], 0.45); //CP12_P6;
   camberlineCP1law(5) = gp_Pnt(designParameters[88], designParameters[89], 0.6); //CP12_P6;
   camberlineCP1law(6) = gp_Pnt(designParameters[90], designParameters[91], 0.75); //CP12_P6;
   camberlineCP1law(7) = gp_Pnt(designParameters[92], designParameters[93], 0.9); //CP12_P6;
   camberlineCP1law(8) = gp_Pnt(designParameters[94], designParameters[95], 1.); //CP12_P6;
   Handle(Geom_BSplineCurve) camberline_CP1 = new Geom_BSplineCurve(camberlineCP1law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP12
   TColgp_Array1OfPnt camberlineCP2law(1, 8);
   camberlineCP2law(1) = gp_Pnt(designParameters[96], designParameters[97], 0.); //CP12_P3;
   camberlineCP2law(2) = gp_Pnt(designParameters[98], designParameters[99], 0.15); //CP12_P4;
   camberlineCP2law(3) = gp_Pnt(designParameters[100], designParameters[101], 0.30); //CP12_P5;
   camberlineCP2law(4) = gp_Pnt(designParameters[102], designParameters[103], 0.45); //CP12_P6;
   camberlineCP2law(5) = gp_Pnt(designParameters[104], designParameters[105], 0.60); //CP12_P6;
   camberlineCP2law(6) = gp_Pnt(designParameters[106], designParameters[107], 0.75); //CP12_P6;
   camberlineCP2law(7) = gp_Pnt(designParameters[108], designParameters[109], 0.90); //CP12_P6;
   camberlineCP2law(8) = gp_Pnt(designParameters[110], designParameters[111], 1.); //CP12_P6;
   Handle(Geom_BSplineCurve) camberline_CP2 = new Geom_BSplineCurve(camberlineCP2law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP13
   TColgp_Array1OfPnt camberlineCP3law(1, 8);
   camberlineCP3law(1) = gp_Pnt(designParameters[112], designParameters[113], 0.); //CP12_P3;
   camberlineCP3law(2) = gp_Pnt(designParameters[114], designParameters[115], 0.15); //CP12_P4;
   camberlineCP3law(3) = gp_Pnt(designParameters[116], designParameters[117], 0.30); //CP12_P5;
   camberlineCP3law(4) = gp_Pnt(designParameters[118], designParameters[119], 0.45); //CP12_P6;
   camberlineCP3law(5) = gp_Pnt(designParameters[120], designParameters[121], 0.60); //CP12_P6;
   camberlineCP3law(6) = gp_Pnt(designParameters[122], designParameters[123], 0.75); //CP12_P6;
   camberlineCP3law(7) = gp_Pnt(designParameters[124], designParameters[125], 0.9); //CP12_P6;
   camberlineCP3law(8) = gp_Pnt(designParameters[126], designParameters[127], 1.); //CP12_P6;
   Handle(Geom_BSplineCurve) camberline_CP3 = new Geom_BSplineCurve(camberlineCP3law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP14
   TColgp_Array1OfPnt camberlineCP4law(1, 8);
   camberlineCP4law(1) = gp_Pnt(designParameters[128], designParameters[129], 0.); //CP12_P3;
   camberlineCP4law(2) = gp_Pnt(designParameters[130], designParameters[131], 0.15); //CP12_P4;
   camberlineCP4law(3) = gp_Pnt(designParameters[132], designParameters[133], 0.30); //CP12_P5;
   camberlineCP4law(4) = gp_Pnt(designParameters[134], designParameters[135], 0.45); //CP12_P6;
   camberlineCP4law(5) = gp_Pnt(designParameters[136], designParameters[137], 0.6); //CP12_P6;
   camberlineCP4law(6) = gp_Pnt(designParameters[138], designParameters[139], 0.75); //CP12_P6;
   camberlineCP4law(7) = gp_Pnt(designParameters[140], designParameters[141], 0.9); //CP12_P6;
   camberlineCP4law(8) = gp_Pnt(designParameters[142], designParameters[143], 1.); //CP12_P6;
   Handle(Geom_BSplineCurve) camberline_CP4 = new Geom_BSplineCurve(camberlineCP4law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP15
   TColgp_Array1OfPnt camberlineCP5law(1, 8);
   camberlineCP5law(1) = gp_Pnt(designParameters[144], designParameters[145], 0.); //CP12_P3;
   camberlineCP5law(2) = gp_Pnt(designParameters[146], designParameters[147], 0.15); //CP12_P4;
   camberlineCP5law(3) = gp_Pnt(designParameters[148], designParameters[149], 0.30); //CP12_P5;
   camberlineCP5law(4) = gp_Pnt(designParameters[150], designParameters[151], 0.45); //CP12_P6;
   camberlineCP5law(5) = gp_Pnt(designParameters[152], designParameters[153], 0.6); //CP12_P6;
   camberlineCP5law(6) = gp_Pnt(designParameters[154], designParameters[155], 0.75); //CP12_P6;
   camberlineCP5law(7) = gp_Pnt(designParameters[156], designParameters[157], 0.9); //CP12_P6;
   camberlineCP5law(8) = gp_Pnt(designParameters[158], designParameters[159], 1.); //CP12_P6;
   Handle(Geom_BSplineCurve) camberline_CP5 = new Geom_BSplineCurve(camberlineCP5law, lawKnots, lawMults, lawDegree, Standard_False);

   //CP16
   TColgp_Array1OfPnt camberlineCP6law(1, 8);
   camberlineCP6law(1) = gp_Pnt(designParameters[160], designParameters[161], 0.); //CP12_P3;
   camberlineCP6law(2) = gp_Pnt(designParameters[162], designParameters[163], 0.15); //CP12_P4;
   camberlineCP6law(3) = gp_Pnt(designParameters[164], designParameters[165], 0.30); //CP12_P5;
   camberlineCP6law(4) = gp_Pnt(designParameters[166], designParameters[167], 0.45); //CP12_P6;
   camberlineCP6law(5) = gp_Pnt(designParameters[168], designParameters[169], 0.60); //CP12_P6;
   camberlineCP6law(6) = gp_Pnt(designParameters[170], designParameters[171], 0.75); //CP12_P6;
   camberlineCP6law(7) = gp_Pnt(designParameters[172], designParameters[173], 0.90); //CP12_P6;
   camberlineCP6law(8) = gp_Pnt(designParameters[174], designParameters[175], 1.); //CP12_P6;
   Handle(Geom_BSplineCurve) camberline_CP6 = new Geom_BSplineCurve(camberlineCP6law, lawKnots, lawMults, lawDegree, Standard_False);

	Standard_Real axialChord = 183.403;

   //CP17
   TColgp_Array1OfPnt camberlineCP7law(1, 8);
   camberlineCP7law(1) = gp_Pnt(designParameters[80]+axialChord, designParameters[176], 0.); //CP12_P3;
   camberlineCP7law(2) = gp_Pnt(designParameters[82]+axialChord, designParameters[177], 0.15); //CP12_P4;
   camberlineCP7law(3) = gp_Pnt(designParameters[84]+axialChord, designParameters[178], 0.30); //CP12_P5;
   camberlineCP7law(4) = gp_Pnt(designParameters[86]+axialChord, designParameters[179], 0.45); //CP12_P6;
   camberlineCP7law(5) = gp_Pnt(designParameters[88]+axialChord, designParameters[180], 0.6); //CP12_P6;
   camberlineCP7law(6) = gp_Pnt(designParameters[90]+axialChord, designParameters[181], 0.75); //CP12_P6;
   camberlineCP7law(7) = gp_Pnt(designParameters[92]+axialChord, designParameters[182], 0.9); //CP12_P6;
   camberlineCP7law(8) = gp_Pnt(designParameters[94]+axialChord, designParameters[183], 1.); //CP12_P6;
   Handle(Geom_BSplineCurve) camberline_CP7 = new Geom_BSplineCurve(camberlineCP7law, lawKnots, lawMults, lawDegree, Standard_False);
   

   //---------------------------------------------------------------------------
   // Stage 2: TUBstator: B-Spline path for TUBstator
   //---------------------------------------------------------------------------


  TColStd_Array1OfReal pathKnots(1,4);
  pathKnots(1)=0.;
  pathKnots(2)=0.4;
  pathKnots(3)=0.8;
  pathKnots(4)=1.;

  TColStd_Array1OfInteger pathMults(1,4);
  pathMults(1)=3;
  pathMults(2)=1;
  pathMults(3)=1;
  pathMults(4)=3;

  TColgp_Array1OfPnt pathCP(1, 5);
  pathCP(1) = gp_Pnt(0., 0, 120.);//147.5);//
  pathCP(2) = gp_Pnt(0., 0, 170.);//185.);//   
  pathCP(3) = gp_Pnt(0., 0, 220.);//220.5);// 
  pathCP(4) = gp_Pnt(0., 0, 270.);//258.);// 
  pathCP(5) = gp_Pnt(0., 0, 320.);//297.5);//


  
  //Handle(Geom2d_BezierCurve) pathCP = new Geom2d_BezierCurve(Bezier11);
  Handle(Geom_BSplineCurve) path = new Geom_BSplineCurve(pathCP, pathKnots, pathMults, 2, Standard_False);

   GeomAdaptor_Curve GAC;
   GAC.Load(path);
   Standard_Real Length = GCPnts_AbscissaPoint::Length(GAC);//length of the curve


   //---------------------------------------------------------------------------
   // Stage 4: TUBstator: Define approximation algo
   //---------------------------------------------------------------------------

   TopoDS_Edge Edge1;
   TopoDS_Edge Edge2;

   TopoDS_Edge Edge1_in;
   TopoDS_Edge Edge2_in;

   TopoDS_Edge camberEdge; //edge3
   
   Handle(Geom_BSplineCurve) midLineBS;
   Handle(Geom_BSplineCurve) suctionMidBS;
   Handle(Geom_BSplineCurve) pressureMidBS;

   TopoDS_Shape blade;
   gp_Dir d[maxSlices+1];
   gp_Dir d2(0,0,1);
   TopoDS_Wire Wire[maxSlices+1];
   TopoDS_Wire camberWire[maxSlices+1];
   gp_Pln Planes[maxSlices+1];
   

   // DISCRETIZE PATHLINE:

   GCPnts_UniformAbscissa pathDiscretizer (GAC, maxSlices, -1);

   // Prepare skinning tool

   const Standard_Boolean isSolid = Standard_False;
   const Standard_Boolean isRuled = Standard_False;
   const Standard_Real    pres3d  = 1e-6;

   BRepOffsetAPI_ThruSections aGenerator(isSolid,isRuled,pres3d);
   aGenerator.SetSmoothing(Standard_True);
   aGenerator.SetMaxDegree(5);
   aGenerator.CheckCompatibility(Standard_False);

   //
   int prevGoodWireIdx = 0;
   for ( Standard_Integer i = 1; i <= maxSlices; ++i )
   {

       gp_Pnt P;
       gp_Vec V;
       path->D1(pathDiscretizer.Parameter(i), P, V);
       d[i] = gp_Dir(V);

       Standard_Real P_Length = GCPnts_AbscissaPoint::Length(GAC, pathDiscretizer.Parameter(1), pathDiscretizer.Parameter(i));//length of the curve

       Standard_Real Length_step = (P_Length/Length);

       // -- Create the intersection plane
       gp_Pnt P_path(0, 0, Length_step);

       gp_Pln constructionPl(P_path, d2);
       Handle(Geom_Plane) constructionPlane = new Geom_Plane (constructionPl);

        //IP1
       GeomAPI_IntCS IntCS1(controlPoints2, constructionPlane);
       //IP2
       GeomAPI_IntCS IntCS2(leadingEdgeRadius, constructionPlane);
       //IP3
       GeomAPI_IntCS IntCS3(controlPoints3, constructionPlane);
       //IP4
       GeomAPI_IntCS IntCS4(controlPoints4, constructionPlane);
       //IP5
       GeomAPI_IntCS IntCS5(controlPoints5, constructionPlane);
       //IP6
       GeomAPI_IntCS IntCS6(controlPoints6, constructionPlane);
       //IP7
       GeomAPI_IntCS IntCS7(controlPoints7, constructionPlane);
       //IP8
       GeomAPI_IntCS IntCS8(controlPoints8, constructionPlane);
       //IP9
       GeomAPI_IntCS IntCS9(controlPoints9, constructionPlane);
       //IP10
       GeomAPI_IntCS IntCS10(trailingEdgeRadius, constructionPlane);
       //IP11
       GeomAPI_IntCS IntCS11(camberline_CP1, constructionPlane);
       //IP12
       GeomAPI_IntCS IntCS12(camberline_CP2, constructionPlane);
       //IP13
       GeomAPI_IntCS IntCS13(camberline_CP3, constructionPlane);
       //IP14
       GeomAPI_IntCS IntCS14(camberline_CP4, constructionPlane);
       //IP15
       GeomAPI_IntCS IntCS15(camberline_CP5, constructionPlane);
       //IP16
       GeomAPI_IntCS IntCS16(camberline_CP6, constructionPlane);
       //IP17
       GeomAPI_IntCS IntCS17(camberline_CP7, constructionPlane);



    //===================================================================
    //step 1: Create Camberline
    //===================================================================

    TColStd_Array1OfReal camberKnots(1,5);
    camberKnots(1)=0.;
    camberKnots(2)=0.25;
    camberKnots(3)=0.5;
    camberKnots(4)=0.75;
    camberKnots(5)=1.;

    TColStd_Array1OfInteger camberMults(1,5);
    camberMults(1)=4.;
    camberMults(2)=1.;
    camberMults(3)=1.;
    camberMults(4)=1.;
    camberMults(5)=4.;


    //Standard_Real Y_F = -5.;

    TColgp_Array1OfPnt2d camberCP(1, 7);
    camberCP(1) = gp_Pnt2d(IntCS11.Point(1).X(), IntCS11.Point(1).Y());
    camberCP(2) = gp_Pnt2d(IntCS12.Point(1).X(), IntCS12.Point(1).Y());   
    camberCP(3) = gp_Pnt2d(IntCS13.Point(1).X(), IntCS13.Point(1).Y());   
    camberCP(4) = gp_Pnt2d(IntCS14.Point(1).X(), IntCS14.Point(1).Y());   
    camberCP(5) = gp_Pnt2d(IntCS15.Point(1).X(), IntCS15.Point(1).Y());   
    camberCP(6) = gp_Pnt2d(IntCS16.Point(1).X(), IntCS16.Point(1).Y());   
    camberCP(7) = gp_Pnt2d(IntCS17.Point(1).X(), IntCS17.Point(1).Y()); 

    //   oss: chord = 183.402635138;

    //Handle(Geom2d_BezierCurve) camberCP = new Geom2d_BezierCurve(Bezier11);
    Handle(Geom2d_BSplineCurve) camberline = new Geom2d_BSplineCurve(camberCP, camberKnots, camberMults, 3, Standard_False);


    TColgp_Array1OfPnt camberCP3d(1, 7);
    camberCP3d(1) = gp_Pnt(camberCP(1).X(), camberCP(1).Y(), P.Z());
    camberCP3d(2) = gp_Pnt(camberCP(2).X(), camberCP(2).Y(), P.Z());   
    camberCP3d(3) = gp_Pnt(camberCP(3).X(), camberCP(3).Y(), P.Z());   
    camberCP3d(4) = gp_Pnt(camberCP(4).X(), camberCP(4).Y(), P.Z());   
    camberCP3d(5) = gp_Pnt(camberCP(5).X(), camberCP(5).Y(), P.Z());   
    camberCP3d(6) = gp_Pnt(camberCP(6).X(), camberCP(6).Y(), P.Z());   
    camberCP3d(7) = gp_Pnt(camberCP(7).X(), camberCP(7).Y(), P.Z()); 

    Handle(Geom_BSplineCurve) camberline3D = new Geom_BSplineCurve(camberCP3d, camberKnots, camberMults, 3, Standard_False);
    
    TopoDS_Edge camberEdge1 = BRepBuilderAPI_MakeEdge(camberline3D);
    
    
    // sample camber line points, uniform distribution:

    Geom2dAdaptor_Curve camberAdaptor (camberline);
    GCPnts_UniformAbscissa camberDiscretizer (camberAdaptor, 10, -1);
    
    
    // sample camber points, cosinus distribution:
    
    Standard_Real camberLength = GCPnts_AbscissaPoint::Length(camberAdaptor);//length of the curve
    
    Standard_Real camberL_1 = camberLength*0.0301536896;
    Standard_Real camberParam1 = GCPnts_AbscissaPoint(camberAdaptor, camberL_1, 0.).Parameter();
    
        Standard_Real camberL_2 = camberLength*0.1169777784;
    Standard_Real camberParam2 = GCPnts_AbscissaPoint(camberAdaptor, camberL_2, 0.).Parameter();
    
        Standard_Real camberL_3 = camberLength*0.25;
    Standard_Real camberParam3 = GCPnts_AbscissaPoint(camberAdaptor, camberL_3, 0.).Parameter();
    
        Standard_Real camberL_4 = camberLength*0.4131759112;
    Standard_Real camberParam4 = GCPnts_AbscissaPoint(camberAdaptor, camberL_4, 0.).Parameter();
    
        Standard_Real camberL_5 = camberLength*0.5868240888;
    Standard_Real camberParam5 = GCPnts_AbscissaPoint(camberAdaptor, camberL_5, 0.).Parameter();
    
        Standard_Real camberL_6 = camberLength*0.75;
    Standard_Real camberParam6 = GCPnts_AbscissaPoint(camberAdaptor, camberL_6, 0.).Parameter();
    
        Standard_Real camberL_7 = camberLength*0.8830222216;
    Standard_Real camberParam7 = GCPnts_AbscissaPoint(camberAdaptor, camberL_7, 0.).Parameter();
    
        Standard_Real camberL_8 = camberLength*0.9698463104;
    Standard_Real camberParam8 = GCPnts_AbscissaPoint(camberAdaptor, camberL_8, 0.).Parameter();

    

    gp_Pnt2d leadingEdge;
    gp_Vec2d firstV1;
    camberline->D1(0., leadingEdge, firstV1);//camberDiscretizer.Parameter(1), leadingEdge, firstV1);
    gp_Dir2d firstD1 (firstV1);
    gp_Lin2d firstL1 (leadingEdge, firstD1);
    gp_Lin2d firstN1 = firstL1.Normal(leadingEdge);
    Handle(Geom2d_Line) firstNormal = new Geom2d_Line(firstN1);



    gp_Pnt2d Pp2;
    gp_Vec2d P2_v1;
    camberline->D1(camberParam1, Pp2, P2_v1);//camberDiscretizer.Parameter(2), Pp2, P2_v1);
    gp_Lin2d P2_l1 (Pp2, P2_v1);
    gp_Lin2d P2_n1 = P2_l1.Normal(Pp2);
    Handle(Geom2d_Line) P2_Normal = new Geom2d_Line(P2_n1);

    Handle(Geom2d_Circle) C_2 = GCE2d_MakeCircle(Pp2, IntCS1.Point(1).X());

    gp_Pnt2d suctionCP2 = Geom2dAPI_InterCurveCurve(C_2, P2_Normal, 1e-6).Point(1);
    gp_Pnt2d pressureCP2 = Geom2dAPI_InterCurveCurve(C_2, P2_Normal, 1e-6).Point(2);



    gp_Lin2d P2_l1_suctionCP2 (suctionCP2, firstV1);
    Handle(Geom2d_Line) P2_norm_suctionCP2 = new Geom2d_Line(P2_l1_suctionCP2);
    gp_Pnt2d suctionCP2_proj = Geom2dAPI_InterCurveCurve(P2_norm_suctionCP2, firstNormal, 1e-6).Point(1);


    Standard_Real dist_suctionCP2 = 0;
    for(int j = 1; j < 3; j++)
    {
        dist_suctionCP2 += (suctionCP2_proj.Coord(j) - suctionCP2.Coord(j))*(suctionCP2_proj.Coord(j) - suctionCP2.Coord(j));
    }


    gp_Lin2d P2_l1_pressureCP2 (pressureCP2, firstV1);
    Handle(Geom2d_Line) P2_norm_pressureCP2 = new Geom2d_Line(P2_l1_pressureCP2);
    gp_Pnt2d pressureCP2_proj = Geom2dAPI_InterCurveCurve(P2_norm_pressureCP2, firstNormal, 1e-6).Point(1);


    Standard_Real dist_pressureCP2 = 0;
    for(int j = 1; j < 3; j++)
    {
        dist_pressureCP2 += (pressureCP2_proj.Coord(j) - pressureCP2.Coord(j))*(pressureCP2_proj.Coord(j) - pressureCP2.Coord(j));
    }


    // Create circles C1 and C2

    Standard_Real r_LE = IntCS2.Point(1).X();
    Standard_Integer suctionDegree = 3;
    Standard_Integer pressureDegree = 3;

    Standard_Real distSuctionCP1 = pow(dist_suctionCP2, 0.25)*pow((r_LE*(suctionDegree-1)/suctionDegree), 0.5);
    Standard_Real distPressureCP1 = pow(dist_pressureCP2, 0.25)*pow((r_LE*(pressureDegree-1)/pressureDegree), 0.5);



    Handle(Geom2d_Circle) C_LE_s = GCE2d_MakeCircle(camberCP(1), distSuctionCP1);
    gp_Pnt2d suctionCP1 = Geom2dAPI_InterCurveCurve(C_LE_s, firstNormal, 1e-6).Point(1);

    Handle(Geom2d_Circle) C_LE_p = GCE2d_MakeCircle(camberCP(1), distPressureCP1);
    gp_Pnt2d pressureCP1 = Geom2dAPI_InterCurveCurve(C_LE_p, firstNormal, 1e-6).Point(2);




    gp_Pnt2d Pp3;
    gp_Vec2d P3_v1;
    camberline->D1(camberParam2, Pp3, P3_v1);//camberDiscretizer.Parameter(3), Pp3, P3_v1);
    gp_Lin2d P3_l1 (Pp3, P3_v1);
    gp_Lin2d P3_n1 = P3_l1.Normal(Pp3);
    Handle(Geom2d_Line) P3_Normal = new Geom2d_Line(P3_n1);

    Handle(Geom2d_Circle) C_3 = GCE2d_MakeCircle(Pp3, IntCS3.Point(1).X());
    gp_Pnt2d suctionCP3 = Geom2dAPI_InterCurveCurve(C_3, P3_Normal, 1e-6).Point(1);
    gp_Pnt2d pressureCP3 = Geom2dAPI_InterCurveCurve(C_3, P3_Normal, 1e-6).Point(2);


    gp_Pnt2d Pp4;
    gp_Vec2d P4_v1;
    camberline->D1(camberParam3, Pp4, P4_v1);//camberDiscretizer.Parameter(4), Pp4, P4_v1);
    gp_Lin2d P4_l1 (Pp4, P4_v1);
    gp_Lin2d P4_n1 = P4_l1.Normal(Pp4);
    Handle(Geom2d_Line) P4_Normal = new Geom2d_Line(P4_n1);

    Handle(Geom2d_Circle) C_4 = GCE2d_MakeCircle(Pp4, IntCS4.Point(1).X());
    gp_Pnt2d suctionCP4 = Geom2dAPI_InterCurveCurve(C_4, P4_Normal, 1e-6).Point(1);
    gp_Pnt2d pressureCP4 = Geom2dAPI_InterCurveCurve(C_4, P4_Normal, 1e-6).Point(2);


    gp_Pnt2d Pp5;
    gp_Vec2d P5_v1;
    camberline->D1(camberParam4, Pp5, P5_v1);//camberDiscretizer.Parameter(5), Pp5, P5_v1);
    gp_Lin2d P5_l1 (Pp5, P5_v1);
    gp_Lin2d P5_n1 = P5_l1.Normal(Pp5);
    Handle(Geom2d_Line) P5_Normal = new Geom2d_Line(P5_n1);

    Handle(Geom2d_Circle) C_5 = GCE2d_MakeCircle(Pp5, IntCS5.Point(1).X());
    gp_Pnt2d suctionCP5 = Geom2dAPI_InterCurveCurve(C_5, P5_Normal, 1e-6).Point(1);
    gp_Pnt2d pressureCP5 = Geom2dAPI_InterCurveCurve(C_5, P5_Normal, 1e-6).Point(2);


    gp_Pnt2d Pp6;
    gp_Vec2d P6_v1;
    camberline->D1(camberParam5, Pp6, P6_v1);//camberDiscretizer.Parameter(6), Pp6, P6_v1);
    gp_Lin2d P6_l1 (Pp6, P6_v1);
    gp_Lin2d P6_n1 = P6_l1.Normal(Pp6);
    Handle(Geom2d_Line) P6_Normal = new Geom2d_Line(P6_n1);

    Handle(Geom2d_Circle) C_6 = GCE2d_MakeCircle(Pp6, IntCS6.Point(1).X());
    gp_Pnt2d suctionCP6 = Geom2dAPI_InterCurveCurve(C_6, P6_Normal, 1e-6).Point(1);
    gp_Pnt2d pressureCP6 = Geom2dAPI_InterCurveCurve(C_6, P6_Normal, 1e-6).Point(2);


    gp_Pnt2d Pp7;
    gp_Vec2d P7_v1;
    camberline->D1(camberParam6, Pp7, P7_v1);//camberDiscretizer.Parameter(7), Pp7, P7_v1);
    gp_Lin2d P7_l1 (Pp7, P7_v1);
    gp_Lin2d P7_n1 = P7_l1.Normal(Pp7);
    Handle(Geom2d_Line) P7_Normal = new Geom2d_Line(P7_n1);

    Handle(Geom2d_Circle) C_7 = GCE2d_MakeCircle(Pp7, IntCS7.Point(1).X());
    gp_Pnt2d suctionCP7 = Geom2dAPI_InterCurveCurve(C_7, P7_Normal, 1e-6).Point(1);
    gp_Pnt2d pressureCP7 = Geom2dAPI_InterCurveCurve(C_7, P7_Normal, 1e-6).Point(2);


    gp_Pnt2d Pp8;
    gp_Vec2d P8_v1;
    camberline->D1(camberParam7, Pp8, P8_v1);//camberDiscretizer.Parameter(8), Pp8, P8_v1);
    gp_Lin2d P8_l1 (Pp8, P8_v1);
    gp_Lin2d P8_n1 = P8_l1.Normal(Pp8);
    Handle(Geom2d_Line) P8_Normal = new Geom2d_Line(P8_n1);

    Handle(Geom2d_Circle) C_8 = GCE2d_MakeCircle(Pp8, IntCS8.Point(1).X());
    gp_Pnt2d suctionCP8 = Geom2dAPI_InterCurveCurve(C_8, P8_Normal, 1e-6).Point(1);
    gp_Pnt2d pressureCP8 = Geom2dAPI_InterCurveCurve(C_8, P8_Normal, 1e-6).Point(2);


    gp_Pnt2d Pp9;
    gp_Vec2d P9_v1;
    camberline->D1(camberParam8, Pp9, P9_v1);//camberDiscretizer.Parameter(9), Pp9, P9_v1);
    gp_Lin2d P9_l1 (Pp9, P9_v1);
    gp_Lin2d P9_n1 = P9_l1.Normal(Pp9);
    Handle(Geom2d_Line) P9_Normal = new Geom2d_Line(P9_n1);

    Handle(Geom2d_Circle) C_9 = GCE2d_MakeCircle(Pp9, IntCS9.Point(1).X());

    gp_Pnt2d suctionCP9 = Geom2dAPI_InterCurveCurve(C_9, P9_Normal, 1e-6).Point(1);
    gp_Pnt2d pressureCP9 = Geom2dAPI_InterCurveCurve(C_9, P9_Normal, 1e-6).Point(2);



    gp_Pnt2d trailingEdge;
    gp_Vec2d lastV1;
    camberline->D1(1., trailingEdge, lastV1);//camberDiscretizer.Parameter(10), trailingEdge, lastV1);
    gp_Dir2d lastD1 (lastV1);
    gp_Lin2d lastL1 (trailingEdge, lastD1);
    gp_Lin2d lastN1 = lastL1.Normal(trailingEdge);
    Handle(Geom2d_Line) lastNormal = new Geom2d_Line(lastN1);


    gp_Lin2d P9_l1_suctionCP9 (suctionCP9, lastV1);
    Handle(Geom2d_Line) P9_norm_suctionCP9 = new Geom2d_Line(P9_l1_suctionCP9);
    gp_Pnt2d suctionCP9_proj = Geom2dAPI_InterCurveCurve(P9_norm_suctionCP9, lastNormal, 1e-6).Point(1);


    Standard_Real dist_suctionCP9 = 0;
    for(int j = 1; j < 3; j++)
    {
        dist_suctionCP9 += (suctionCP9_proj.Coord(j) - suctionCP9.Coord(j))*(suctionCP9_proj.Coord(j) - suctionCP9.Coord(j));
    }

    gp_Lin2d P9_l1_pressureCP9 (pressureCP9, lastV1);
    Handle(Geom2d_Line) P9_norm_pressureCP9 = new Geom2d_Line(P9_l1_pressureCP9);
    gp_Pnt2d pressureCP9_proj = Geom2dAPI_InterCurveCurve(P9_norm_pressureCP9, lastNormal, 1e-6).Point(1);


    Standard_Real dist_pressureCP9 = 0;
    for(int j = 1; j < 3; j++)
    {
        dist_pressureCP9 += (pressureCP9_proj.Coord(j) - pressureCP9.Coord(j))*(pressureCP9_proj.Coord(j) - pressureCP9.Coord(j));
    }

   
    Standard_Real r_TE = IntCS10.Point(1).X();

    Standard_Real distSuctionCP10 = pow(dist_suctionCP9, 0.25)*pow((r_TE*(suctionDegree-1)/suctionDegree), 0.5);
    Standard_Real distPressureCP10 = pow(dist_pressureCP9, 0.25)*pow((r_TE*(pressureDegree-1)/pressureDegree), 0.5);

    Handle(Geom2d_Circle) C_TE_s = GCE2d_MakeCircle(camberCP(7), distSuctionCP10);
    gp_Pnt2d suctionCP10 = Geom2dAPI_InterCurveCurve(C_TE_s, lastNormal, 1e-6).Point(1);

    Handle(Geom2d_Circle) C_TE_p = GCE2d_MakeCircle(camberCP(7), distPressureCP10);
    gp_Pnt2d pressureCP10 = Geom2dAPI_InterCurveCurve(C_TE_p, lastNormal, 1e-6).Point(2);
    

    TColStd_Array1OfReal suctionKnots(1,10);
    suctionKnots(1)=0.;
    suctionKnots(2)=0.10;
    suctionKnots(3)=0.20;
    suctionKnots(4)=0.30;
    suctionKnots(5)=0.40;
    suctionKnots(6)=0.50;
    suctionKnots(7)=0.60;
    suctionKnots(8)=0.70;
    suctionKnots(9)=0.80;    
    suctionKnots(10)=1.;

    TColStd_Array1OfInteger suctionMults(1,10);
    suctionMults(1)=4.;
    suctionMults(2)=1.;
    suctionMults(3)=1.;
    suctionMults(4)=1.;
    suctionMults(5)=1.;
    suctionMults(6)=1.;
    suctionMults(7)=1.;
    suctionMults(8)=1.;
    suctionMults(9)=1.;    
    suctionMults(10)=4.;

    TColgp_Array1OfPnt suctionBsplineCP(1,12);
    suctionBsplineCP(1) = gp_Pnt(camberCP(1).X(), camberCP(1).Y(), P.Z());
    suctionBsplineCP(2) = gp_Pnt(suctionCP1.X(), suctionCP1.Y(), P.Z());
    suctionBsplineCP(3) = gp_Pnt(suctionCP2.X(), suctionCP2.Y(), P.Z());
    suctionBsplineCP(4) = gp_Pnt(suctionCP3.X(), suctionCP3.Y(), P.Z());
    suctionBsplineCP(5) = gp_Pnt(suctionCP4.X(), suctionCP4.Y(), P.Z());
    suctionBsplineCP(6) = gp_Pnt(suctionCP5.X(), suctionCP5.Y(), P.Z());
    suctionBsplineCP(7) = gp_Pnt(suctionCP6.X(), suctionCP6.Y(), P.Z());
    suctionBsplineCP(8) = gp_Pnt(suctionCP7.X(), suctionCP7.Y(), P.Z());
    suctionBsplineCP(9) = gp_Pnt(suctionCP8.X(), suctionCP8.Y(), P.Z());
    suctionBsplineCP(10) = gp_Pnt(suctionCP9.X(), suctionCP9.Y(), P.Z());
    suctionBsplineCP(11) = gp_Pnt(suctionCP10.X(), suctionCP10.Y(), P.Z());
    suctionBsplineCP(12) = gp_Pnt(camberCP(7).X(), camberCP(7).Y(), P.Z());
    //Handle(Geom_BezierCurve) suctionBS = new Geom2d_BezierCurve(suction2dBsplineCP);
    Handle(Geom_BSplineCurve) suctionBS = new Geom_BSplineCurve(suctionBsplineCP, suctionKnots, suctionMults, suctionDegree, Standard_False);
    
    TColgp_Array1OfPnt pressureBsplineCP(1,12);
    pressureBsplineCP(1) = gp_Pnt(camberCP(1).X(), camberCP(1).Y(), P.Z());
    pressureBsplineCP(2) = gp_Pnt(pressureCP1.X(), pressureCP1.Y(), P.Z());
    pressureBsplineCP(3) = gp_Pnt(pressureCP2.X(), pressureCP2.Y(), P.Z());
    pressureBsplineCP(4) = gp_Pnt(pressureCP3.X(), pressureCP3.Y(), P.Z());
    pressureBsplineCP(5) = gp_Pnt(pressureCP4.X(), pressureCP4.Y(), P.Z());
    pressureBsplineCP(6) = gp_Pnt(pressureCP5.X(), pressureCP5.Y(), P.Z());
    pressureBsplineCP(7) = gp_Pnt(pressureCP6.X(), pressureCP6.Y(), P.Z());
    pressureBsplineCP(8) = gp_Pnt(pressureCP7.X(), pressureCP7.Y(), P.Z());
    pressureBsplineCP(9) = gp_Pnt(pressureCP8.X(), pressureCP8.Y(), P.Z());
    pressureBsplineCP(10) = gp_Pnt(pressureCP9.X(), pressureCP9.Y(), P.Z());
    pressureBsplineCP(11) = gp_Pnt(pressureCP10.X(), pressureCP10.Y(), P.Z());
    pressureBsplineCP(12) = gp_Pnt(camberCP(7).X(), camberCP(7).Y(), P.Z());
    Handle(Geom_BSplineCurve) pressureBS = new Geom_BSplineCurve(pressureBsplineCP, suctionKnots, suctionMults, pressureDegree, Standard_False);
    pressureBS->Reverse();
    
   
	Edge1 = BRepBuilderAPI_MakeEdge( suctionBS );
	Edge2 = BRepBuilderAPI_MakeEdge( pressureBS );

       BRepBuilderAPI_MakeWire mkWire;
       mkWire.Add(Edge1); if(i==0) Edge1_in = mkWire.Edge(); 
       mkWire.Add(Edge2); if(i==0) Edge2_in = mkWire.Edge(); 
       Wire[i] = mkWire.Wire();



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
   }
   //
   if ( !aGenerator.IsDone() )
   {
       std::cout << "Error: IsDone() false in BRepOffsetAPI_ThruSections" << std::endl;
   }

   blade = aGenerator.Shape();

    TopoDS_Shape face_E1 = aGenerator.GeneratedFace(Edge1_in);
    TopoDS_Shape face_E2 = aGenerator.GeneratedFace(Edge2_in);



   return blade;
   
   
}





