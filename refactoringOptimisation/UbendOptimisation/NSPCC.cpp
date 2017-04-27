//
// Created by orestmykhaskiv on 27/09/16.
//
#include "NSPCC.h"

TopoDS_Shape occtContinuityConstraints::generalFuse(const TopoDS_Shape &Body, const TopoDS_Shape &Contour)
//                                 const double fuzz,
// TopoDS_Shape& ResultBody,
// TopoDS_Shape& ResultContour) const
{
    Handle(NCollection_BaseAllocator) Alloc = new NCollection_IncAllocator;

    // Prepare the arguments
    TopTools_ListOfShape BOP_args;
    BOP_args.Append(Contour);
    BOP_args.Append(Body);

    const int bRunParallel = 0;

    BOPAlgo_PaveFiller DSFiller(Alloc);
    DSFiller.SetArguments(BOP_args);
    DSFiller.SetRunParallel(bRunParallel);
    //   DSFiller.SetFuzzyValue(fuzz);
    DSFiller.Perform();
    int iErr = DSFiller.ErrorStatus();
    if ( iErr )
    {
        //       return false;
    }

    BOPAlgo_Builder BOP(Alloc);
    BOP.SetArguments(BOP_args);
    BOP.SetRunParallel(bRunParallel);
    BOP.PerformWithFiller(DSFiller);
    iErr = BOP.ErrorStatus();
    if ( iErr )
    {
        //   return false;
    }

    // Set results
    auto ResultBody    = BOP.Shape();
    return ResultBody;
//        ResultContour = this->updateContour(BOP, Contour);
//        return true;
}

vector<Standard_Real> occtContinuityConstraints::VectorProduct(gp_Dir a, gp_Dir b) {
    vector<Standard_Real> r(3);
    r[0] = a.Coord(2)*b.Coord(3)-a.Coord(3)*b.Coord(2);
    r[1] = a.Coord(3)*b.Coord(1)-a.Coord(1)*b.Coord(3);
    r[2] = a.Coord(1)*b.Coord(2)-a.Coord(2)*b.Coord(1);
    return r;
}

void occtContinuityConstraints::ConstructConstraintMatrix(int k, int xyz, int adIndex) {
#if !REVERSE_MODE
    int testpoints3xIndex = 0;
    gp_Dir normal_left, normal_right;

    for (int i = 0; i < edgesContinuity.size(); ++i) {

        auto mesh_surface_left = bsplineSurfaces[edgesContinuity[i].faceIndexLeft-1];
        auto mesh_surface_right = bsplineSurfaces[edgesContinuity[i].faceIndexRight-1];

        for (int l = 0; l < edgesContinuity[i].numberofTestPoits; ++l) {
            gp_Pnt point_left,point_right;
            auto leftParam = edgesContinuity[i].testPointsParametricLeft[l];
            auto rightParam = edgesContinuity[i].testPointsParametricRight[l];

            mesh_surface_left->D0(leftParam.X(), leftParam.Y(), point_left);
            mesh_surface_right->D0(rightParam.X(), rightParam.Y(), point_right);

            auto G1_Continuity = (edgesContinuity[i].continuityLevel == Continuity::G1);
            vector<Standard_Real> G1;
            if (G1_Continuity){
                GeomLProp_SLProps props(mesh_surface_left,leftParam.X(), leftParam.Y(),1, 0.01);
                normal_left = props.Normal();

                GeomLProp_SLProps propsRight(mesh_surface_right, rightParam.X(), rightParam.Y(),1, 0.01);
                normal_right = propsRight.Normal();

                G1 = VectorProduct(normal_left,normal_right);
            }


            for (int j = 1; j <= 3; ++j) {
                auto G0 = (point_left.Coord(j) - point_right.Coord(j));

                mx_C(k * 3 + xyz - 1,testpoints3xIndex) = (G0.getADValue(adIndex));
                testpoints3xIndex++;

                if (G1_Continuity){
                   mx_C(k * 3 + xyz - 1,testpoints3xIndex) = (G1[j].getADValue(adIndex));
                    testpoints3xIndex++;
                }
            }
        }
    }
#else
#endif
}

void occtContinuityConstraints::ConstructMeshPointsDerivativeMatrix(int k, int xyz, int adIndex) {
#if !REVERSE_MODE
    gp_Pnt point;
    int testindex = 0;
    //Detect also not found projections: they are nullified
    for (int i = 0; i < meshpointsCartesian.size(); ++i) {
        if (meshpointsFaceIndex[i] == designControlPoints.faceIndex[k]) {
            auto mesh_surface = bsplineSurfaces[meshpointsFaceIndex[i]-1];
            mesh_surface->D0(meshpointsParametric[i].X(), meshpointsParametric[i].Y(), point);

            for (int j = 1; j <= 3; ++j) {
                mx_R(testindex,k * 3 + xyz - 1) = (point.Coord(j).getADValue(adIndex));
                testindex ++;
            }
        }
        else{
            for (int j = 1; j <= 3; ++j) {
                mx_R(testindex,k * 3 + xyz - 1) = 0;
                testindex ++;
            }
        }
    }
#else
#endif
}

void occtContinuityConstraints::ConstructConstraintMatrix_VectorMode(int startK, int endK) {
#if !REVERSE_MODE
    int testpoints3xIndex = 0;
    gp_Dir normal_left, normal_right;

    for (int i = 0; i < edgesContinuity.size(); ++i) {

        auto mesh_surface_left = bsplineSurfaces[edgesContinuity[i].faceIndexLeft - 1];
        auto mesh_surface_right = bsplineSurfaces[edgesContinuity[i].faceIndexRight - 1];

        for (int l = 0; l < edgesContinuity[i].numberofTestPoits; ++l) {
            gp_Pnt point_left, point_right;
            auto leftParam = edgesContinuity[i].testPointsParametricLeft[l];
            auto rightParam = edgesContinuity[i].testPointsParametricRight[l];

            mesh_surface_left->D0(leftParam.X(), leftParam.Y(), point_left);
            mesh_surface_right->D0(rightParam.X(), rightParam.Y(), point_right);

            auto G1_Continuity = (edgesContinuity[i].continuityLevel == Continuity::G1);
            vector<Standard_Real> G1;
            if (G1_Continuity) {
                GeomLProp_SLProps props(mesh_surface_left, leftParam.X(), leftParam.Y(), 1, 0.01);
                normal_left = props.Normal();

                GeomLProp_SLProps propsRight(mesh_surface_right, rightParam.X(), rightParam.Y(), 1, 0.01);
                normal_right = propsRight.Normal();

                G1 = VectorProduct(normal_left, normal_right);
            }
            vector<Standard_Real> G0(3);
            for (int j = 0; j < 3; ++j) {
                G0[j] = (point_left.Coord(j+1) - point_right.Coord(j+1));
            }


            int adIndex = 0;
            for (int k = startK; k < endK; ++k) {
                int currentTestpoint = testpoints3xIndex;
                for (int xyz = 1; xyz <= 3; ++xyz) {
                    mx_C(k * 3 + xyz - 1, currentTestpoint) = (G0[xyz - 1].getADValue(adIndex));
                   currentTestpoint++;

                    if (G1_Continuity) {
                        mx_C(k * 3 + xyz - 1, currentTestpoint) = (G1[xyz-1].getADValue(adIndex));
                        currentTestpoint ++;
                    }


                    adIndex ++;
                }
            }

            testpoints3xIndex +=3;
            if (G1_Continuity)
                testpoints3xIndex +=3;


        }
    }
#else
#endif
}

void occtContinuityConstraints::ConstructMeshPointsDerivativeMatrix_VectorModer(int startK, int endK) {
#if !REVERSE_MODE
    gp_Pnt point;
    //Detect also not found projections: they are nullified
    int meshpointIndex = 0;

    for (int i = 0; i < meshpointsCartesian.size(); ++i) {

        if (meshpointsFaceIndex[i]!=-2) {
            auto mesh_surface = bsplineSurfaces[meshpointsFaceIndex[i] - 1];
            mesh_surface->D0(meshpointsParametric[i].X(), meshpointsParametric[i].Y(), point);

            int derivativeIndex = 0;
            for (int k = startK; k < endK; ++k) {
                for (int xyz = 1; xyz <= 3; ++xyz) {
                    mx_R(meshpointIndex + xyz-1, k * 3 + xyz - 1) = (point.Coord(xyz).getADValue(derivativeIndex));
                    derivativeIndex++;
                }
            }
        }
        meshpointIndex += 3;
    }
#else
#endif
}

void occtContinuityConstraints::CastBSplineSurfaces() {

    int n = facemap.Extent();
    bsplineSurfaces = vector<Handle(Geom_BSplineSurface)>(n);

    for (int i = 1; i <= n ; ++i) {
        bsplineSurfaces[i-1] = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(TopoDS::Face(facemap(i))));
    }
//
//    for (int j = 0; j < bsplineSurfaces.size(); ++j) {
//        cout  << "bspline degree" << bsplineSurfaces[j]->MaxDegree() << endl;
//        bsplineSurfaces[j]->IncreaseDegree(2,2);
//    }
}

int occtContinuityConstraints::GetTestPointsNumber() {
    int counter = 0;
    for (int i = 0; i < edgesContinuity.size() ; ++i) {

        counter+= edgesContinuity[i].numberofTestPoits;
        if (edgesContinuity[i].continuityLevel == Continuity::G1){
            counter+= edgesContinuity[i].numberofTestPoits;
        }
    }
    return counter;
}

NCollection_Array1<gp_Pnt> occtContinuityConstraints::ReadMeshPoints(string mesh_filename, bool pureCoordinates) {

    ifstream infile(mesh_filename);
    //infile.precision(20);
    int numberOfMeshPoints;
    infile >> numberOfMeshPoints ;
    NCollection_Array1<gp_Pnt> meshpoints = NCollection_Array1<gp_Pnt>(1,numberOfMeshPoints);

    if (pureCoordinates){
        Standard_Integer index=1;
        double X, Y, Z;
        //cout.precision(20);
        while (infile >> X >> Y >> Z) {
            meshpoints.SetValue(index, gp_Pnt(X, Y, Z));
            index++;
        }
    }
    else {
        Standard_Integer index;
        double X, Y, Z;
        //cout.precision(20);
        while (infile >> index >> X >> Y >> Z) {
            meshpoints.SetValue(index, gp_Pnt(X, Y, Z));
        }
    }

    return meshpoints;
}

void occtContinuityConstraints::WriteMeshPoints_ToTextFile(string mesh_filename, vector<gp_Pnt> meshpointsToFile, bool pureCoordinates, double meshScale=1.0){
    cout << "Writing Mesh to files" << endl;
    ofstream outfile;
    outfile.open(mesh_filename);
    //infile.precision(20);
    int numberOfMeshPoints = meshpointsToFile.size();
    outfile << numberOfMeshPoints << endl;

    for (int i = 0; i <numberOfMeshPoints; ++i) {
        if (pureCoordinates) {
            outfile << meshpointsToFile[i].X().getValue()*meshScale << " " << meshpointsToFile[i].Y().getValue()*meshScale << " "
                    << meshpointsToFile[i].Z().getValue()*meshScale
                    << endl;
        }
        else {
            outfile << i << " " << meshpointsToFile[i].X() << " " << meshpointsToFile[i].Y() << " "
                    << meshpointsToFile[i].Z() << endl;
        }
    }
    outfile.close();
    cout << "MeshFile created: "<< mesh_filename <<endl;
}

ContinuityInfo occtContinuityConstraints::SetTestPointsOnEdge(ContinuityInfo cIIn) {

    ContinuityInfo cI = cIIn;
    Standard_Real aPFirstLeft, aPLastLeft, aPFirstRight, aPLastRight;

    Handle(Geom2d_Curve) aPCurveLeft = BRep_Tool::CurveOnSurface (TopoDS::Edge(edgeMap(cI.edgeIndex)), TopoDS::Face(facemap(cI.faceIndexLeft)), aPFirstLeft, aPLastLeft);
    Handle(Geom2d_Curve) aPCurveRight = BRep_Tool::CurveOnSurface (TopoDS::Edge(edgeMap(cI.edgeIndex)), TopoDS::Face(facemap(cI.faceIndexRight)), aPFirstRight, aPLastRight);

    Standard_Real intervalLeft = -aPFirstLeft+aPLastLeft;
    Standard_Real intervalRight = -aPFirstRight + aPLastRight;

    Standard_Real stepLeft = intervalLeft/(cI.numberofTestPoits-1);
    Standard_Real stepRight = intervalRight/(cI.numberofTestPoits-1);

    auto testPointsParametricLeft = vector<gp_Pnt2d>(cI.numberofTestPoits);
    auto testPointsParametricRight = vector<gp_Pnt2d>(cI.numberofTestPoits);

    Standard_Real current = aPFirstLeft;
    int index = 0;

    while((index < cI.numberofTestPoits)){
        gp_Pnt2d pPoint;
        aPCurveLeft->D0(current,pPoint);
        testPointsParametricLeft[index] = pPoint;
        current+=stepLeft;
        index++;
    }

    current = aPFirstRight;
    index = 0;
    while(index < cI.numberofTestPoits){
        gp_Pnt2d pPoint;
        aPCurveRight->D0(current,pPoint);
        testPointsParametricRight[index] = pPoint;
        current+=stepRight;
        index++;
    }

    cI.testPointsParametricLeft = testPointsParametricLeft;
    cI.testPointsParametricRight = testPointsParametricRight;
    return cI;
}

void occtContinuityConstraints::FindMesh(Standard_CString meshfile, bool findProjection, double scale) {

    cout << "Finiding Mesh" << endl;
    auto mesh  = ReadMeshPoints(meshfile, false);

    meshpointsCartesian =  vector<gp_Pnt>(mesh.Size());
    for (int i = 0; i <meshpointsCartesian.size() ; ++i) {
        // meshpointsCartesian[i] = mesh(i+1);
        meshpointsCartesian[i] =gp_Pnt(scale*mesh(i+1).Coord(1), scale*mesh(i+1).Coord(2) ,scale*mesh(i+1).Coord(3));
    }
    if(findProjection)
        FindDesignMeshProjections(facemap,meshpointsCartesian);

    //Set design point to ones
    designPointTable = vector<vector<int>>(3,vector<int>(meshpointsCartesian.size()));
    for (int j = 0; j < 3; ++j) {
          for (int i = 0; i < designPointTable[j].size(); ++i) {
            designPointTable[j][i] = 1;
        }
    }
}

void occtContinuityConstraints::MatrixInitialization() {
    for (int m = 0; m <mx_C.rows() ; ++m) {
        for (int i = 0; i < mx_C.cols(); ++i) {
            mx_C(m,i) = 0;
        }
    }

    for (int m = 0; m <mx_R.rows() ; ++m) {
        for (int i = 0; i < mx_R.cols(); ++i) {
            mx_R(m,i) = 0;
        }
    }
}

void occtContinuityConstraints::ReadCFDGradient(Standard_CString sensitivityFile) {

    ifstream infile(sensitivityFile);
    int numberOfMeshPoints  = meshpointsCartesian.size();

    CFDGradinet.resize(1,3*numberOfMeshPoints);

    Standard_Integer index=0;
        double X, Y, Z;
        while (infile >> X >> Y >> Z && index < 3*numberOfMeshPoints) {
            X = 1; Y = 1; Z = 1;
            CFDGradinet(0,index) = X;
            CFDGradinet(0,index+1) = Y;
            CFDGradinet(0,index+2) = Z;
            index+=3;
        }
}


void occtContinuityConstraints::ReadCFDGradient(vector<vector<double>> cfdSens){
    int numberOfMeshPoints  = meshpointsCartesian.size();

    CFDGradinet.resize(1,3*numberOfMeshPoints);
    Standard_Integer index=0;

    for (int i = 0; i < numberOfMeshPoints; ++i) {
        CFDGradinet(0,index) = cfdSens[0][i];
        CFDGradinet(0,index+1) = cfdSens[1][i];;
        CFDGradinet(0,index+2) = cfdSens[2][i];;
        index+=3;
    }
}

void occtContinuityConstraints::PrintTestPointsInfo() {
    for (int i = 0; i < edgesContinuity.size(); ++i) {
        cout << "Edge" << edgesContinuity[i].edgeIndex << endl;
        for (int j = 0; j < edgesContinuity[i].testPointsParametricRight.size() ; ++j) {
          //  cout << edgesContinuity[i].testPointsParametricRight[j].X().getNumDir() << endl;
            cout << "uv Right" << edgesContinuity[i].testPointsParametricRight[j].X().getValue() << " | " << edgesContinuity[i].testPointsParametricRight[j].Y().getValue() << endl;
        }
        cout << " ----------- " << endl;
        for (int j = 0; j < edgesContinuity[i].testPointsParametricLeft.size() ; ++j) {
            cout << "uv Left" << edgesContinuity[i].testPointsParametricLeft[j].X().getValue() << " | " << edgesContinuity[i].testPointsParametricLeft[j].Y().getValue() << endl;
        }

        cout << " ===================================== " << endl;
    }
}

void occtContinuityConstraints::CalculateNormalizedGradient() {
    cout << "Normalizing Gradient" << endl;
    auto absValueMax = abs(gradient.maxCoeff());
    auto absValueMin = abs(gradient.minCoeff());
    auto maxAbs = absValueMax > absValueMin ?  absValueMax : absValueMin;

    normalizedGradient.resize(gradient.rows(),gradient.cols());
    for (int i = 0; i < gradient.cols() ; ++i) {
        normalizedGradient(0,i) = gradient(0,i)/maxAbs;
    }
}

void occtContinuityConstraints::PerformDesignStep(double scaleFactor) {
  cout << "Design Step" << endl;
    cout << KerC_Stencil.rows() << "/"<< KerC_Stencil.cols()<< "/" << normalizedGradient.rows()<< "/" << normalizedGradient.cols() << endl;
    MatrixXd sigmaP = KerC_Stencil * normalizedGradient.transpose();
    sigmaP = scaleFactor * sigmaP;
    cout << "sigma P" << sigmaP.rows() << "/" << sigmaP.cols() << endl;
    cout << sigmaP << endl;
    //update control points
    int totalIndex = 0;
    for (int j = 0; j < designControlPoints.faceIndex.size(); ++j) {
        auto uindex = designControlPoints.uIndex[j];
        auto vindex = designControlPoints.vIndex[j];
        gp_Pnt cp = bsplineSurfaces[designControlPoints.faceIndex[j]-1]->Pole(uindex,vindex);
        for (int k = 1; k <=3 ; ++k) {
            cp.SetCoord(k,cp.Coord(k)+sigmaP(totalIndex,0));
            totalIndex++;
        }
        bsplineSurfaces[designControlPoints.faceIndex[j]-1]->SetPole(uindex,vindex,cp);
    }


}

void occtContinuityConstraints::WriteUpdatedFaces(string postfix) {
    cout << "StepWriting" << endl;

//
    for (int m = 0; m <bsplineSurfaces.size() ; ++m) {
        auto stepfile = settings.nspcc_dir + "facePerturbed_4f_SmallPerturb_FixedEdges" +postfix + to_string(m) + ".stp";
        StepWriter(stepfile.c_str(),BRepBuilderAPI_MakeFace(bsplineSurfaces[m],10E-3));
    }

//    auto result = BRepBuilderAPI_MakeFace(bsplineSurfaces[0],10E-3).Shape();
//    for (int l = 1; l <bsplineSurfaces.size() ; ++l) {
//        auto nextFace = BRepBuilderAPI_MakeFace(bsplineSurfaces[l],10E-3).Shape();
//        result = generalFuse(result,nextFace);
//    }
//
//    auto stepfile_updated = settings.nspcc_dir + "faces_generalFused_FixedEdges" + to_string(edgesContinuity[0].numberofTestPoits)+".stp";
//    StepWriter(stepfile_updated.c_str(),result);
}

void occtContinuityConstraints::FreeMatrices() {
    mx_C.resize(0,0);
    mx_R.resize(0,0);
}

void occtContinuityConstraints::ReadGradintMatrices_Final(Standard_CString filename, string postfix) {

    cout << "Getting Gradient" << endl;
    gradient = getTable_Eigen<MatrixXd>(filename, ("CFDCADGradient"+postfix).c_str());
    cout << "Getting normalised Gradient" << endl;
    normalizedGradient = getTable_Eigen<MatrixXd>(filename, ("normalisedGradient"+postfix).c_str());
    cout << "Getting Ker_C" << endl;
    KerC_Stencil = getTable_Eigen<MatrixXd>(filename, ("Ker_C"+postfix).c_str());

}

void occtContinuityConstraints::SensitivityBasedInsertion(vector<int> indexes) {
    gp_Pnt2d point2Insert;

    cout << "sensitivityMesh" << endl;
    for (int i = 0; i < indexes.size(); ++i) {
        int meshIndex = indexes[i] - 1;
        int surfaceIndex = meshpointsFaceIndex[meshIndex] - 1;

        if (surfaceIndex > -1) {

            point2Insert = meshpointsParametric[meshIndex];

            cout << "Projected" << point2Insert.X().getValue() << " | " << point2Insert.Y().getValue() << endl;

            gp_Pnt point3d;
            bsplineSurfaces[surfaceIndex]->D0(point2Insert.X(), point2Insert.Y(), point3d);
            cout << point3d.X().getValue() << "|" << point3d.Y().getValue() << "|" << point3d.Z().getValue() << endl;
            point3d = meshpointsCartesian[meshIndex];
            cout << point3d.X().getValue() << "|" << point3d.Y().getValue() << "|" << point3d.Z().getValue() << endl;

            bsplineSurfaces[surfaceIndex]->InsertVKnot(point2Insert.Y(), 1, Precision::Confusion());
            //  bsplineSurfaces[surfaceIndex]->InsertUKnot(point2Insert.X(),1,Precision::Confusion());
        }
        else{
            cout << "Face index of meshpoint  " << indexes[i] <<  " = " << surfaceIndex+1 << " knot was not inserted" << endl;
        }
    }

}

void occtContinuityConstraints::ManualVKnotInsertion(vector<double> vValues) {
    for (int i = 0; i < bsplineSurfaces.size() ; ++i) {
        for (int j = 0; j <vValues.size() ; ++j) {
            bsplineSurfaces[i]->InsertVKnot(vValues[j], 1, Precision::Confusion());
        }
    }
}

void occtContinuityConstraints::ManualUKnotInsertion(vector<double> vValues) {
    for (int i = 1; i < bsplineSurfaces.size() ; ++i) {
        for (int j = 0; j <vValues.size() ; ++j) {
            bsplineSurfaces[i]->InsertUKnot(vValues[j], 1, Precision::Confusion());
        }
    }
}

void occtContinuityConstraints::PrintCP_Udistribution(int faceindex) {
    for (int i = 1; i <=bsplineSurfaces[faceindex]->NbUKnots() ; ++i) {
        cout << bsplineSurfaces[faceindex]->UKnot(i).getValue() << endl;
    }
}

void occtContinuityConstraints::PrintCP_XDistribution(int faceindex) {
    for (int i = 1; i <=bsplineSurfaces[faceindex]->NbUPoles() ; ++i) {
        cout << bsplineSurfaces[faceindex]->Pole(i,1).X().getValue() << endl;
    }
}






