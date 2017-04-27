//
// Created by orestmykhaskiv on 04/02/16.
//

#ifndef U_BENDOPTIMISATION_OCCTDATAPROVIDER_H
#define U_BENDOPTIMISATION_OCCTDATAPROVIDER_H

#include <iostream>
#include <TopoDS_Shape.hxx>
#include <STEPControl_Reader.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <Geom_Circle.hxx>
#include <TopExp.hxx>
#include <NCollection_Array1.hxx>
#include <vector>
#include <stdexcept>
#include <BRepBndLib.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <BRepTools.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <Geom_BezierCurve.hxx>
#include <gp_Pln.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <GCE2d_MakeCircle.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GeomAPI.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <Geom_Plane.hxx>
#include <STEPControl_Writer.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TColStd_HArray1OfReal.hxx>
#include <ShapeBuild_Edge.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <GeomAPI_IntCS.hxx>
#include <OSD_Timer.hxx>
#include <Geom_BSplineSurface.hxx>
#include "hdmf5IO.h"
#include "Settings.h"

//ADOL-C includes
#include <adolc/taping.h>
#include <adolc/drivers/drivers.h>


////Display vtk includes
//
//#include <IVtkOCC_Shape.hxx>
//#include <IVtkTools_ShapeDataSource.hxx>
//#include <vtkPolyDataMapper.h>
//#include <vtkActor.h>
//#include <vtkRenderer.h>
//#include <vtkRenderWindow.h>

using namespace std;
class OCCTDataProvider{

public:

    int nParams;
    vector<Standard_Real> designParameters;

    TopoDS_Shape myShape;

    vector<gp_Pnt> meshpointsCartesian;
    vector<gp_Pnt> meshpointsCartesianPerturbed;
    vector<vector<int>> designPointTable;

    vector<gp_Pnt2d> meshpointsParametric;
    vector<int> meshpointsFaceIndex;
    vector<Standard_Real> meshpointsGap;
    vector<int> meshPointsOnFace;

    TopTools_IndexedDataMapOfShapeListOfShape mapEdgesOfFace;
    TopTools_IndexedMapOfShape facemap;
    TopTools_IndexedMapOfShape edgeMap;

    size_t tape_stats[STAT_SIZE];
    int numberOfOutputsUsedForReverseMode;

    OCCTDataProvider(vector<Standard_Real> p_params){
                designParameters =p_params;
                nParams=p_params.size();

                cout << "Building shape" << endl;
                OSD_Timer aTimer;
                aTimer.Start();
                BuildShape();
                cout << "Building connectivity" << endl;
                BuildConnectivity();
                aTimer.Stop();
                cout << "OCCTDataProvider construction time: " << aTimer.ElapsedTime() << endl;
    }

    OCCTDataProvider(){
        nParams=0;
        designParameters =vector<Standard_Real>();
        meshpointsCartesian=vector<gp_Pnt>();
        meshpointsCartesianPerturbed=vector<gp_Pnt>();
        designPointTable=vector<vector<int>>();
        meshpointsFaceIndex=vector<int>();
        meshpointsParametric=vector<gp_Pnt2d>();
        meshpointsGap=vector<Standard_Real>();
        meshPointsOnFace=vector<int>();


        mapEdgesOfFace=TopTools_IndexedDataMapOfShapeListOfShape();
        edgeMap=TopTools_IndexedMapOfShape();
        facemap=TopTools_IndexedMapOfShape();
    }


   //TODO: Salvatores JOb
    void BuildShape(bool activateDesignParametersForReverse = false){

       //string dummy step file
       //string stepfile = "/home/orestmykhaskiv/QMUL/mgoptCustomCases/pipeLow/PipeCorkedLow.stp";
       //myShape = StepReader(stepfile.c_str());
       //myShape = UbendCorked(designParameters);

       //pipeHorizontal
       //myShape=UbendHorizontalLegs();

       //March8
      // myShape=ConstructSquaredUbendWithRealDimensionsFixedLegs();
     //  myShape = ConstructSquaredUbendWithRealDimensions();
      //December 1st
      myShape = ConstructSquaredUbend_December1st(activateDesignParametersForReverse);
      //TU BERLIN
      //myShape = ConstructTUBstatorWithRealDimensions_December14th(activateDesignParametersForReverse);

    }

    void BuildConnectivity(){
        mapEdgesOfFace.Clear();
        facemap.Clear();
        edgeMap.Clear();
        TopExp::MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, mapEdgesOfFace);
        TopExp::MapShapes(myShape, TopAbs_FACE, facemap);
        TopExp::MapShapes(myShape,TopAbs_EDGE, edgeMap);
    }

    void SetMeshes(vector<vector<double>> meshpoints, vector<vector<int>> designVariablesTable){
        designPointTable = designVariablesTable;
        meshpointsCartesian = vector<gp_Pnt>(meshpoints[0].size());
        for (int i=0; i <meshpoints[0].size(); i++){
            meshpointsCartesian[i] = gp_Pnt(meshpoints[0][i],meshpoints[1][i],meshpoints[2][i]);
        }
        FindDesignMeshProjections(facemap, meshpointsCartesian); //for the ubend it is without any arguments

    }

    void SetMeshesNoProjection(vector<vector<double>> meshpoints, vector<vector<int>> designVariablesTable){
        designPointTable = designVariablesTable;
        meshpointsCartesian = vector<gp_Pnt>(meshpoints[0].size());
        for (int i=0; i <meshpoints[0].size(); i++){
            meshpointsCartesian[i] = gp_Pnt(meshpoints[0][i],meshpoints[1][i],meshpoints[2][i]);
        }

    }

    void PrintNormalVectors(){
        Standard_Real umin, umax, vmin, vmax;
        for (int i=1; i<=facemap.Extent(); i++){
           TopoDS_Face aCurrentFace = TopoDS::Face(facemap(i));
        BRepTools::UVBounds(aCurrentFace,umin, umax, vmin, vmax);
        Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aCurrentFace);
        GeomLProp_SLProps props(aSurface, umin, vmin,1, 0.01);
        gp_Dir normal = props.Normal();

            cout << "Surface is " << (BRep_Tool::Surface(aCurrentFace))->DynamicType() <<  normal.Coord(1) << "," << normal.Coord(2) << ","<< normal.Coord(3) << "," <<endl;
        }
    }

    void CalculatePertubedMesh() {

        meshpointsCartesianPerturbed = vector<gp_Pnt>(meshpointsCartesian.size());
        for (int i = 0; i < meshpointsParametric.size(); i++) {
            gp_Pnt point3d(meshpointsCartesian[i]);
            if (isDesignPointByMeshIndex(i) && meshpointsFaceIndex[i]!=-1) {
                BRep_Tool::Surface(TopoDS::Face(facemap(meshpointsFaceIndex[i])))->D0(meshpointsParametric[i].Coord(1),
                                                                                   meshpointsParametric[i].Coord(2),
                                                                                   point3d);
            }
            for (int j = 0; j < 3; j++) {
                meshpointsCartesianPerturbed[i].SetCoord(j+1, point3d.Coord(j+1));
            }
        }
    }

    void UpdateMeshFromPerturbed(){
        for (int i=0;i<meshpointsCartesian.size();i++)
            for(int j=0;j<3; j++)
                meshpointsCartesian[i].SetCoord(j+1,meshpointsCartesianPerturbed[i].Coord(j+1));
    }

    vector<vector<double>> Find3dMeshDisplacement(){
        vector<vector<double>> displacement(3,vector<double>(meshpointsParametric.size()));
        int testindex = 0;
        for (int i=0; i<meshpointsParametric.size(); i++){

               for (int j=0;j<3; j++){
                    displacement[j][i]=(meshpointsCartesianPerturbed[i].Coord(j+1)-meshpointsCartesian[i].Coord(j+1)).getValue();
                    if(abs(displacement[j][i])>10e-20)
                        testindex++;
               }
        }
        cout <<"Displacemnet nodes:  " << testindex/3 << endl;
        return displacement;
    }

    vector<vector<Standard_Real>> GetCompleteDesignSensitivityUsingVectorMode()
    {
      vector<vector<Standard_Real>> sensitivityMatrix(3,vector<Standard_Real>(meshpointsParametric.size()));
#if !REVERSE_MODE
      //create identity matrix
      for(int i = 0; i < nParams; i++)
        designParameters[i].setADValue(i,1.);

      BuildShape();
      BuildConnectivity();

      NCollection_Array1<Handle(Geom_Surface)> surfaces = NCollection_Array1<Handle(Geom_Surface)>(1, facemap.Extent());
      for(int i = 1; i <= facemap.Extent(); i++)
      {
          surfaces(i) = BRep_Tool::Surface(TopoDS::Face(facemap(i)));
      }

      for(int i = 0; i < meshpointsParametric.size(); i++)
      {
        if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
        {
          gp_Pnt aPnt;
          surfaces(meshpointsFaceIndex[i])->D0(meshpointsParametric[i].X(), meshpointsParametric[i].Y(), aPnt);
          sensitivityMatrix[0][i] = aPnt.X();
          sensitivityMatrix[1][i] = aPnt.Y();
          sensitivityMatrix[2][i] = aPnt.Z();
        }
        else
        {
          sensitivityMatrix[0][i] = 0.;
          sensitivityMatrix[1][i] = 0.;
          sensitivityMatrix[2][i] = 0.;
        }
      }
#else
      cout << "GetCompleteDesignSensitivityUsingVectorMode not implemented, REVERSE_MODE = 1" << endl;
#endif
      return sensitivityMatrix;
    }

    vector<vector<double>> GetDesignSensitivity(int parameterIndex){
      //  cout << "GetDesignSensitivity[" << parameterIndex << "]" << endl;
        vector<vector<double>> sensitivityMatrix(3,vector<double>(meshpointsParametric.size()));
#if !REVERSE_MODE
        //first, set all AD values of designParameters to 0
        for(int i = 0; i < designParameters.size(); i++)
        {
            designParameters[i].setADValue(0,0.);
            //cout << "designParameters[" << i << "]" << designParameters[i] << endl;
        }
        //set derivative seed only for current parameter index
        designParameters[parameterIndex].setADValue(0,1.);

        BuildShape();
        BuildConnectivity();

        NCollection_Array1<Handle(Geom_Surface)> surfaces = NCollection_Array1<Handle(Geom_Surface)>(1, facemap.Extent());
        for(int i = 1; i <= facemap.Extent(); i++)
        {
            surfaces(i) = BRep_Tool::Surface(TopoDS::Face(facemap(i)));
        }

        for(int i = 0; i < meshpointsParametric.size(); i++)
        {
            if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
            {
                gp_Pnt aPnt;
                surfaces(meshpointsFaceIndex[i])->D0(meshpointsParametric[i].X(), meshpointsParametric[i].Y(), aPnt);
                sensitivityMatrix[0][i] = *(aPnt.X().getADValue());
                sensitivityMatrix[1][i] = *(aPnt.Y().getADValue());
                sensitivityMatrix[2][i] = *(aPnt.Z().getADValue());
            }
            else
            {
                sensitivityMatrix[0][i] = 0.;
                sensitivityMatrix[1][i] = 0.;
                sensitivityMatrix[2][i] = 0.;
            }
        }
#else
        cout << "GetDesignSensitivity not implemented, REVERSE_MODE = 1" << endl;
#endif
       return sensitivityMatrix;

    }

    void GenerateADTrace()
    {
#if REVERSE_MODE
      double *output = new double[numberOfOutputsUsedForReverseMode];

      trace_on(1);

      BuildShape(true);
      BuildConnectivity();

      NCollection_Array1<Handle(Geom_Surface)> surfaces = NCollection_Array1<Handle(Geom_Surface)>(1, facemap.Extent());
      for(int i = 1; i <= facemap.Extent(); i++)
      {
          surfaces(i) = BRep_Tool::Surface(TopoDS::Face(facemap(i)));
      }

      int j = 0;
      for(int i = 0; i < meshpointsParametric.size(); i++)
      {
        if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
        {
          gp_Pnt aPnt;
          surfaces(meshpointsFaceIndex[i])->D0(meshpointsParametric[i].X(), meshpointsParametric[i].Y(), aPnt);
          aPnt.X() >>= output[j]; j++;
          aPnt.Y() >>= output[j]; j++;
          aPnt.Z() >>= output[j]; j++;
        }
      }

      trace_off(1);
      tapestats(1,tape_stats);
      cout<<"maxlive "<<tape_stats[NUM_MAX_LIVES]<<"\n";
      cout<<"No Independents: "<<tape_stats[NUM_INDEPENDENTS]<<"\n";
      cout<<"No dependents: "<<tape_stats[NUM_DEPENDENTS]<<"\n";

      delete[] output;
#endif
    }

    vector<vector<double>> GetDesignSensitivityByEvaluatingADTrace(int paramIndex)
    {
      vector<vector<double>> sensitivityMatrix(3,vector<double>(meshpointsParametric.size()));
#if REVERSE_MODE
      double *independent = new double[nParams];
      for(int i = 0; i < nParams; i++)
        independent[i] = designParameters[i].getValue();

      double *tangent = new double[nParams];
      for(int i = 0; i < nParams; i++)
        tangent[i] = 0.;
      tangent[paramIndex] = 1.;

      double *result = new double[numberOfOutputsUsedForReverseMode];

      int retVal;
      retVal = jac_vec(1, numberOfOutputsUsedForReverseMode, nParams, independent, tangent, result);

      int j = 0;
      for(int i = 0; i < meshpointsParametric.size(); i++)
      {
        if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
        {
          sensitivityMatrix[0][i] = result[j]; j++;
          sensitivityMatrix[1][i] = result[j]; j++;
          sensitivityMatrix[2][i] = result[j]; j++;
        }
        else
        {
          sensitivityMatrix[0][i] = 0.;
          sensitivityMatrix[1][i] = 0.;
          sensitivityMatrix[2][i] = 0.;
        }
      }

      delete[] independent;
      delete[] tangent;
      delete[] result;
#endif

      return sensitivityMatrix;
    }

    //calculated using the reverse mode
    vector<double> GetGradientByEvaluatingADTraceWithCFDSensitivity(vector<vector<double>> cfdSens)
    {
      vector<double> gradient(nParams);
#if REVERSE_MODE
      double *independent = new double[nParams];
      for(int i = 0; i < nParams; i++)
        independent[i] = designParameters[i].getValue();

      double *rangeVectorU = new double[numberOfOutputsUsedForReverseMode];
      int j=0;
      for(int i = 0; i < meshpointsParametric.size(); i++)
      {
        if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
        {
          rangeVectorU[j] = cfdSens[0][i]; j++;
          rangeVectorU[j] = cfdSens[1][i]; j++;
          rangeVectorU[j] = cfdSens[2][i]; j++;
        }
      }

      double *result = new double[nParams];

      int retVal;
      //check cfd sensitivities
//      std::ofstream os;
//      string filename = "/home/mbanovic/IODA/Development/Ubend/2016-09-22/optimisation/CFDsensitivities.txt";
//      os.open(filename,ios::app);
//      os.precision(10);
//      for(int i = 0; i < 11628; i++)
//      {
//        os << rangeVectorU[i] << endl;
//      }
//      os << "========================================================================================" << endl;
//      os.close();
      //end
      retVal = vec_jac(1, numberOfOutputsUsedForReverseMode, nParams, 0, independent, rangeVectorU, result);

      for(int i = 0; i < nParams; i++)
        gradient[i] = result[i];

      delete[] independent;
      delete[] rangeVectorU;
      delete[] result;
#endif
      return gradient;
    }

    int CalculateNumberOfOutputs(){
      int outputs = 0;
      for(int i = 0; i < meshpointsParametric.size(); i++)
      {
        if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
        {
          outputs+=3;
        }
      }
      numberOfOutputsUsedForReverseMode = outputs;
      return outputs;
    }

    void UpdateMeshOnCAD() {

        NCollection_Array1<Handle(Geom_Surface) > surfaces = NCollection_Array1<Handle(Geom_Surface) >(1, facemap.Extent());
        for (int i = 1; i <= facemap.Extent(); i++) {
            surfaces(i) = BRep_Tool::Surface(TopoDS::Face(facemap(i)));
        }
        int index = 0;
        for (int i = 0; i < meshpointsParametric.size(); i++) {
            if (meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i)) {
                gp_Pnt aPnt;
                surfaces(meshpointsFaceIndex[i])->D0(meshpointsParametric[i].X(), meshpointsParametric[i].Y(), aPnt);
                meshpointsCartesian[i] = aPnt;
                index ++;
            }
        }
        cout << index << "points setted to CAD" << endl;
    }

    //central finite difference
    vector<vector<double>> GetDesignSensitivityFD(int parameterIndex){
        const double FD_STEP = 10E-8;
      //  cout << "GetDesignSensitivity[" << parameterIndex << "]" << endl;
        vector<vector<double>> sensitivityMatrix(3,vector<double>(meshpointsParametric.size()));
        vector<vector<double>> meshPointValuesPrev(3,vector<double>(meshpointsParametric.size()));
        vector<vector<double>> meshPointValuesNext(3,vector<double>(meshpointsParametric.size()));

        //store original value of parameter before perturbation
        double originalDesignParameterValue = designParameters[parameterIndex].getValue();

        //FD_prev
        designParameters[parameterIndex] = originalDesignParameterValue - FD_STEP;
        //cout.precision(6);
        //cout << "designParameters[" << parameterIndex << "] = " << designParameters[parameterIndex].getValue() << endl;

        BuildShape();
        BuildConnectivity();

        NCollection_Array1<Handle(Geom_Surface)> surfaces_prev = NCollection_Array1<Handle(Geom_Surface)>(1, facemap.Extent());
        for(int i = 1; i <= facemap.Extent(); i++)
        {
          surfaces_prev(i) = BRep_Tool::Surface(TopoDS::Face(facemap(i)));
        }

        for(int i = 0; i < meshpointsParametric.size(); i++)
        {
            if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
            {
                gp_Pnt aPnt;
                surfaces_prev(meshpointsFaceIndex[i])->D0(meshpointsParametric[i].X(), meshpointsParametric[i].Y(), aPnt);
                meshPointValuesPrev[0][i] = aPnt.X().getValue();
                meshPointValuesPrev[1][i] = aPnt.Y().getValue();
                meshPointValuesPrev[2][i] = aPnt.Z().getValue();
            }
        }

        //FD_next
        designParameters[parameterIndex] = originalDesignParameterValue + FD_STEP;
        //cout << "designParameters[" << parameterIndex << "] = " << designParameters[parameterIndex].getValue() << endl << endl;

        BuildShape();
        BuildConnectivity();

        NCollection_Array1<Handle(Geom_Surface)> surfaces_next = NCollection_Array1<Handle(Geom_Surface)>(1, facemap.Extent());
        for(int i = 1; i <= facemap.Extent(); i++)
        {
          surfaces_next(i) = BRep_Tool::Surface(TopoDS::Face(facemap(i)));
        }

        for(int i = 0; i < meshpointsParametric.size(); i++)
        {
            if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
            {
                gp_Pnt aPnt;
                surfaces_next(meshpointsFaceIndex[i])->D0(meshpointsParametric[i].X(), meshpointsParametric[i].Y(), aPnt);
                meshPointValuesNext[0][i] = aPnt.X().getValue();
                meshPointValuesNext[1][i] = aPnt.Y().getValue();
                meshPointValuesNext[2][i] = aPnt.Z().getValue();
            }
        }

        //compute central differences
        const double divider = (2 * FD_STEP);
        for(int i = 0; i < meshpointsParametric.size(); i++)
        {
            if(meshpointsParametric[i].X() > -1 && meshpointsParametric[i].Y() > -1 && isDesignPointByMeshIndex(i))
            {
                sensitivityMatrix[0][i] = (meshPointValuesNext[0][i] - meshPointValuesPrev[0][i]) / divider;
                sensitivityMatrix[1][i] = (meshPointValuesNext[1][i] - meshPointValuesPrev[1][i]) / divider;
                sensitivityMatrix[2][i] = (meshPointValuesNext[2][i] - meshPointValuesPrev[2][i]) / divider;
            }
            else
            {
                sensitivityMatrix[0][i] = 0.;
                sensitivityMatrix[1][i] = 0.;
                sensitivityMatrix[2][i] = 0.;
            }
        }

        //return to original design parameter
        designParameters[parameterIndex] = originalDesignParameterValue;
        BuildShape();
        BuildConnectivity();

        return sensitivityMatrix;
    }


    IFSelect_ReturnStatus StepWriter(Standard_CString filename){
        STEPControl_Writer writer;
        STEPControl_StepModelType mode = STEPControl_StepModelType::STEPControl_AsIs;
        IFSelect_ReturnStatus stat = writer.Transfer(myShape,mode);
        auto status = writer.Write(filename);
        return status;
    }

    IFSelect_ReturnStatus StepWriter(Standard_CString filename, TopoDS_Shape shape){
        STEPControl_Writer writer;
        STEPControl_StepModelType mode = STEPControl_StepModelType::STEPControl_AsIs;
        IFSelect_ReturnStatus stat = writer.Transfer(shape,mode);
        auto status = writer.Write(filename);
        return status;
    }

    //Destructor
    ~OCCTDataProvider(){
        if (!myShape.IsNull())
            myShape.Nullify();
//        designParameters = NULL;
        // mapEdgesOfFace = NULL;
        // facemap = NULL;
        // edgeMap = NULL;
    }


    bool isDesignPointByMeshIndex(int meshindex){
        bool designPoint = false;
        for (int i=0;i<3;i++){
            if (designPointTable[i][meshindex] == 1)
                designPoint=true;
        }
        return designPoint;
    }

    void FindDesignMeshProjections(){
        cout << "finding mesh projections: Generating shape Analysis tools" << endl;
        NCollection_Array1<Handle(ShapeAnalysis_Surface)> shapeAnalysis = NCollection_Array1<Handle(ShapeAnalysis_Surface)>(1, facemap.Extent());
        NCollection_Array1<Handle(Geom_Surface)> surfaces = NCollection_Array1<Handle(Geom_Surface)>(1, facemap.Extent());
        NCollection_Array1<Bnd_Box> boundingBoxes = NCollection_Array1<Bnd_Box>(1, facemap.Extent());
        NCollection_Array1<int> faceMeshCount  = NCollection_Array1<int>(1, facemap.Extent());

        for (int j = 1; j<=facemap.Extent(); j++) {
            //populate bounnding boxes b;
            BRepBndLib::Add(facemap(j), boundingBoxes(j));

            surfaces(j) =  BRep_Tool::Surface(TopoDS::Face(facemap(j)));
            shapeAnalysis(j) = new ShapeAnalysis_Surface(surfaces(j));
            faceMeshCount(j) = 0;

        }

        int designPointQuantity =0;
        int meshpointsSize=meshpointsCartesian.size();
        meshpointsParametric=vector<gp_Pnt2d>(meshpointsSize);
        meshpointsGap=vector<Standard_Real>(meshpointsSize);
        meshpointsFaceIndex=vector<int>(meshpointsSize);


        cout << "finding mesh projections: Projecting points" << endl;

        for (int i = 1; i <= meshpointsSize; i++){
            if (i%1000==0) {
                cout << i << " points projected out of  " << meshpointsSize << endl;
            }
            gp_Pnt point = meshpointsCartesian[i-1];
            bool designPoint = isDesignPointByMeshIndex(i-1);
            bool projectionFound = false;

            if (designPoint) {
                designPointQuantity ++;
                for (int j = 1; j <= facemap.Extent(); j++) {

                    //if in bounding box lets find the projection and add to meshpoints
                    if (!boundingBoxes(j).IsOut(point)) {
                        gp_Pnt2d parametricCoordinates  = shapeAnalysis(j)->ValueOfUV(point, 10E-7);
                        double gap = (shapeAnalysis(j)->Gap()).getValue();
                        gap = fabs(gap);
                        if (gap < (10E-4)/1.63) { //TODO Orest's new value here is: 10E-1, but the current one is working for U-bend
                            meshpointsParametric[i-1] = parametricCoordinates;
                            meshpointsFaceIndex[i-1]=j;
                            meshpointsGap[i-1]=gap;

                            int facecount = faceMeshCount(j);
                            facecount++;
                            faceMeshCount.SetValue(j, facecount);

                            projectionFound = true;
                            break;
                        }

                    }
                }
            }
            if (!projectionFound){
                //projection not Found or Gap too big
                meshpointsParametric[i-1]=gp_Pnt2d(-1,-1);
                if(designPoint) {
                    meshpointsFaceIndex[i-1] = -1;
                    meshpointsGap[i-1]=100;
                }else{
                    meshpointsFaceIndex[i-1] =-2;
                    meshpointsGap[i-1]=200;
                }
            }
        }
        //
        meshPointsOnFace=vector<int>(facemap.Extent());

        for (int j = 0; j<facemap.Extent(); j++) {
            meshPointsOnFace[j]=faceMeshCount(j+1);
        }

        cout << "number of design meshpoint = " << designPointQuantity << endl;
        int desPointsOnFaces=0;
        for (int i=0;i<meshPointsOnFace.size();i++){
            cout << "face_" << i << "  " << meshPointsOnFace[i] << endl;
            desPointsOnFaces+=meshPointsOnFace[i];
        }
        cout << "Total design points="  << desPointsOnFaces << endl;


    }

    void  FindDesignMeshProjections(TopTools_IndexedMapOfShape facemap, vector<gp_Pnt> meshpointsCartesian) {
        //WARNING: boundingBoxes are commented for TUB Stator case
        OSD_Timer atimer;
        atimer.Start();
        cout << "finding mesh projections: Generating shape Analysis tools" << endl;
        NCollection_Array1<Handle(ShapeAnalysis_Surface) > shapeAnalysis = NCollection_Array1<Handle(ShapeAnalysis_Surface) >(1, facemap.Extent());
        NCollection_Array1<Handle(Geom_Surface) > surfaces = NCollection_Array1<Handle(Geom_Surface) >(1, facemap.Extent());
//        NCollection_Array1<Bnd_Box> boundingBoxes = NCollection_Array1<Bnd_Box>(1, facemap.Extent());
        NCollection_Array1<int> faceMeshCount = NCollection_Array1<int>(1, facemap.Extent());

        for (int j = 1; j <= facemap.Extent(); j++) {
            //populate bounnding boxes b;
//            BRepBndLib::Add(facemap(j), boundingBoxes(j));

            surfaces(j) = BRep_Tool::Surface(TopoDS::Face(facemap(j)));
            shapeAnalysis(j) = new ShapeAnalysis_Surface(surfaces(j));
            faceMeshCount(j) = 0;
        }

        int designPointQuantity = 0;
        int meshpointsSize = meshpointsCartesian.size();

        meshpointsParametric = vector<gp_Pnt2d>(meshpointsSize);
        meshpointsGap = vector<Standard_Real>(meshpointsSize);
        meshpointsFaceIndex = vector<int>(meshpointsSize);

        cout << "finding mesh projections: Projecting points" << endl;

        for (int i = 1; i <= meshpointsSize; i++) {
            if (i % 1000 == 0) {
                cout << i << " points projected out of  " << meshpointsSize << endl;
            }
            gp_Pnt point = meshpointsCartesian[i - 1];

            //!!!!!!1
            bool designPoint = isDesignPointByMeshIndex(i-1);
//            bool designPoint = true;
            bool projectionFound = false;

            if (designPoint) {
                designPointQuantity++;
                Standard_Real currentBestGap = 100;
                int currentBestSurface = 1;
                for (int j = 1; j <= facemap.Extent(); j++) {

                    //if in bounding box lets find the projection and add to meshpoints
                    //Finding the best surface = means smallest gap
//                    auto box = boundingBoxes(j);
//                    if (!boundingBoxes(j).IsOut(point)) {

                        gp_Pnt2d parametricCoordinates = shapeAnalysis(j)->ValueOfUV(point, 10E-6);
                        Standard_Real gap = (shapeAnalysis(j)->Gap());
                        gap = fabs(gap);
                        if (gap < (10E-1*5)) { //10E-2
                            if (gap < currentBestGap) {
                                currentBestGap = gap;
                                currentBestSurface = j;

                                meshpointsParametric[i - 1] = parametricCoordinates;
                                meshpointsFaceIndex[i - 1] = j;
                                meshpointsGap[i - 1] = gap;

                                projectionFound = true;
                            }
                        }

//                    }
                }

                if (projectionFound) {
                    int facecount = faceMeshCount(currentBestSurface);
                    facecount++;
                    faceMeshCount.SetValue(currentBestSurface, facecount);
                }
            }
            if (!projectionFound) {
                //projection not Found or Gap too big
                meshpointsParametric[i - 1] = gp_Pnt2d(-1, -1);
                if (designPoint) {
                    meshpointsFaceIndex[i - 1] = -1;
                    meshpointsGap[i - 1] = 100;
                } else {
                    meshpointsFaceIndex[i - 1] = -2;
                    meshpointsGap[i - 1] = 200;
                }
            }
        }
        //
        meshPointsOnFace = vector<int>(facemap.Extent());

        for (int j = 0; j < facemap.Extent(); j++) {
            meshPointsOnFace[j] = faceMeshCount(j + 1);
        }


        cout << "number of design meshpoint = " << designPointQuantity << endl;
        int desPointsOnFaces = 0;
        for (int i = 0; i < meshPointsOnFace.size(); i++) {
            cout << "face_" << i + 1 << "  " << meshPointsOnFace[i] << endl;
            desPointsOnFaces += meshPointsOnFace[i];
        }
        cout << "Total design points=" << desPointsOnFaces << endl;
        atimer.Stop();
        cout << atimer.ElapsedTime() << endl;


        vector<int> notFoundIndexes(0);
        for (int i=0;i<meshpointsSize; i++) {
            if (meshpointsFaceIndex[i] == -1) {
                notFoundIndexes.push_back(i+1);
            }
        }
        if (notFoundIndexes.size()>0) {
            cout << "Not found projection indexes" << endl;
            for (int k = 0; k < notFoundIndexes.size(); ++k) {
                cout << notFoundIndexes[k] << "  ";
            }
            cout << endl;
        }
    }

    void DisplayShape(){

//            IVtkOCC_Shape::Handle ivtkShape = new IVtkOCC_Shape(myShape);
//
//            IVtkTools_ShapeDataSource *DS = IVtkTools_ShapeDataSource::New();
//            DS->SetShape(ivtkShape);
//
//            vtkPolyDataMapper *pVtkPolyDataMapper = vtkPolyDataMapper::New();
//            pVtkPolyDataMapper->SetInputConnection(DS->GetOutputPort() );
//
//            vtkActor *bottleActor = vtkActor::New();
//            bottleActor->SetMapper(pVtkPolyDataMapper);
//
//            vtkRenderer *ren1= vtkRenderer::New();
//            ren1->AddActor( bottleActor );
//            ren1->SetBackground( 0.1, 0.2, 0.4 );
//
//            vtkRenderWindow *renWin = vtkRenderWindow::New();
//            renWin->AddRenderer( ren1 );
//            renWin->SetSize( 700, 700 );
//            //renWin->Render();
//
//            int i;
//            for (i = 0; i < 360; ++i) {
//                // render the image
//                renWin->Render();
//                // rotate the active camera by one degree
//                ren1->GetActiveCamera()->Azimuth( 1 );
//            }
//
//            pVtkPolyDataMapper->Delete();
//            bottleActor->Delete();
//            ren1->Delete();
//            renWin->Delete();

    }

    TopoDS_Shape StepReader(const char *filename){
        cout << string(filename)<<endl;
        STEPControl_Reader reader;
        auto readerSuccess = reader.ReadFile(filename);
        if (readerSuccess!=IFSelect_ReturnStatus::IFSelect_RetDone){
            string message = "Step Read Failed";
            throw std::runtime_error(message);
        }
        // auto readerSuccess = reader.ReadFile(filename);
        cout << "Success = " << readerSuccess << endl;
        reader.PrintCheckLoad(Standard_False, IFSelect_PrintCount::IFSelect_CountByItem);
        // Loads file MyFile.stp

        // gets the number of transferable roots
        cout << "Number of roots in STEP file: " << reader.NbRootsForTransfer() << endl;
        Standard_Integer NbTrans = reader.TransferRoots();
        // translates all transferable roots, and returns the number of    //successful translations
        cout << "STEP roots transferred: " << NbTrans << endl;

        cout << "Number of resulting shapes is: " << reader.NbShapes() << endl;
        auto result = reader.OneShape();
        return result;
    }

    void SewShape(){
        BRepBuilderAPI_Sewing sewing;
        sewing.Add(myShape);
        sewing.Perform();

        myShape  = sewing.SewedShape();
    }

    void WriteProjectionsData_OCCT(Standard_CString filename){
        vector<vector<double>> parametricTable(2,vector<double>(meshpointsParametric.size()));
        vector<double> gaps(meshpointsParametric.size());

        for (int i=0;i<meshpointsParametric.size();i++) {
            parametricTable[0][i] = meshpointsParametric[i].X().getValue();
            parametricTable[1][i] = meshpointsParametric[i].Y().getValue();
            gaps[i] = meshpointsGap[i].getValue();
        }

        cout << "populating meshpointsParametric" << endl;

        CreateAndWriteTable<double>(parametricTable,filename,"meshpointsParametric");
        cout << "populating meshpointFaceIndex" << endl;
        CreateAndWriteVector(meshpointsFaceIndex,filename, "meshpointFaceIndex");
        cout << "populating meshpointsGap" << endl;
        CreateAndWriteVector(gaps,filename,"meshpointsGap");
        cout << "populating meshPointsOnFace" << endl;
        CreateAndWriteVector(meshpointsFaceIndex,filename,"meshPointsOnFace");
    }

    void ReadProjectionData_OCCT(Standard_CString filename){

        cout << " Read Data Projections" << endl;
        auto parametricTable =  getTable<double>(filename,"meshpointsParametric");
        auto gaps = getVector<double>(filename,"meshpointsGap");
        int meshpointsSize = parametricTable[0].size();
        meshpointsParametric=vector<gp_Pnt2d>(meshpointsSize);
        meshpointsGap=vector<Standard_Real>(meshpointsSize);

        for (int i=0;i<meshpointsSize;i++){
            meshpointsParametric[i]=gp_Pnt2d(parametricTable[0][i],parametricTable[1][i]);
            meshpointsGap[i] = gaps[i];
        }

        meshpointsFaceIndex = getVector<int>(filename,"meshpointFaceIndex");
        meshPointsOnFace = getVector<int>(filename,"meshPointsOnFace");

    }

    void AnalyseShape(){

        cout << "Analysing Shape" << endl;

        for (int i = 1; i <=facemap.Extent() ; ++i) {
            auto face_current = TopoDS::Face(facemap(i));
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face_current);
            cout << "Surface " << i << "   " << surface->DynamicType() << endl;

            if (surface->DynamicType() == STANDARD_TYPE(Geom_BSplineSurface)) {

                Handle(Geom_BSplineSurface) bsplineSurface = Handle(Geom_BSplineSurface)::DownCast(surface);
                cout << "Nb ControlPoints U and V = " << bsplineSurface->NbUPoles() << " / "
                     << bsplineSurface->NbVPoles() << endl;
                cout << "spline degree U and V = " << bsplineSurface->UDegree() << " / " << bsplineSurface->VDegree()
                     << endl;
                cout << "Is Rational U and V " << bsplineSurface->IsURational() << bsplineSurface->IsVRational()
                     << endl;

                Standard_Real u1, u2, v1, v2;
                bsplineSurface->Bounds(u1, u2, v1, v2);

                cout << "Max UV" << u1.getValue() << " , " << u2.getValue() << " , " << v1.getValue() << " , " << v2.getValue() << endl;
            }

            else{
                cout << "Surface " << i << "   " << surface->DynamicType() << endl;
            }


        }
        cout << endl << endl;
    }

private:




    //Declaration of methods to build shape
    TopoDS_Shape ConstructSquaredUbend();
    TopoDS_Shape UbendCorked(vector<Standard_Real> a);
    TopoDS_Shape UbendSquared(vector<double> a);
    TopoDS_Shape UbendHorizontalLegs();
    TopoDS_Shape ConstructSquaredUbendWithPathAndLawsAsBSplines(); //March 4th
    TopoDS_Shape ConstructSquaredUbendWithRealDimensions();  //March8;
    TopoDS_Shape ConstructSquaredUbend_June10th(bool activateDesignParametersForReverse = false);
    TopoDS_Shape ConstructSquaredUbendWithRealDimensionsFixedLegs(); //March21
    TopoDS_Shape ConstructSquaredUbend_December1st(bool activateDesignParametersForReverse = false);
    TopoDS_Shape ConstructTUBstatorWithRealDimensions_December14th(bool activateDesignParametersForReverse = false);

};




#endif //U_BENDOPTIMISATION_OCCTDATAPROVIDER_H
