//
// Created by orestmykhaskiv on 22/04/16.
//

#ifndef U_BENDOPTIMISATION_NSPCC_H
#define U_BENDOPTIMISATION_NSPCC_H

#include "Eigen/SVD"
#include "iostream"
#include <NCollection_IncAllocator.hxx>
#include <BOPAlgo_PaveFiller.hxx>
#include <BOPAlgo_Builder.hxx>
#include "Geom_BSplineSurface.hxx"
#include "Standard_CString.hxx"
#include "Settings.h"
#include "OCCTDataProvider.h"
#include "BRepTools_ReShape.hxx"

using namespace std;
using namespace Eigen;
//void SVDTest(){
//    MatrixXd m = MatrixXd::Random(4,5);
//    cout << "matrix Created " << endl;
//
//    m << 1,0,0,0,2,0,0,3,0,0,0,0,0,0,0,0,2,0,0,0;
//
//    cout << "Here is the matrix m:" << endl << m << endl;
//    JacobiSVD<MatrixXd> svd(m, ComputeThinU | ComputeFullV);
//    cout << "Its singular values are:" << endl;
//    cout << svd.singularValues() << endl;
//    cout << "rank = " << svd.rank() << endl;
//   cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
//   cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
////    Vector3f rhs(1, 0, 0);
////    cout << "Now consider this rhs vector:" << endl << rhs << endl;
////    cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(rhs) << endl;
//
//
//    auto vMatr = svd.matrixV();
////
//    VectorXd v(5);
//    for (int i=0;i<=4; i++)
//        v(i) =  vMatr(i,4);
//
//    cout << "Vector v = " << endl << v << endl;
//    auto c = m*v;
//    cout <<"Matrxix c"<< endl << c;
//
//}

enum Continuity { G0 = 0, G1=1, G2 = 2, NotSpecified = -1 };

class ContinuityInfo{
public:
    int edgeIndex;
    int faceIndexLeft;
    int faceIndexRight;

    ContinuityInfo(){};

    int numberofTestPoits;
    vector<gp_Pnt2d> testPointsParametricLeft;
    vector<gp_Pnt2d> testPointsParametricRight;
    Continuity continuityLevel;
};

class NSPCC_DesignControlPoints{
public:
    vector<int> faceIndex;
    vector<int> uIndex;
    vector<int> vIndex;
};

class occtContinuityConstraints : public OCCTDataProvider  {

public:

    vector<int> designFaces;
    vector<ContinuityInfo> edgesContinuity;
    NSPCC_DesignControlPoints designControlPoints;
    vector<Handle(Geom_BSplineSurface)> bsplineSurfaces;

    MatrixXd mx_R;
    MatrixXd mx_C;

    MatrixXd KerC_Stencil;
    MatrixXd CADGradient;
    MatrixXd CFDGradinet;
    MatrixXd gradient;
    MatrixXd normalizedGradient;

    Settings_Stator settings;

    occtContinuityConstraints(const char *stepfilename) {

        if (stepfilename!= nullptr) {
            cout << "Building shape" << endl;
            myShape = StepReader(stepfilename);

            //SewShape();

            cout << "Building connectivity" << endl;
            BuildConnectivity();

//        cout << "Casting BSlplineSurfaces"<< endl;
            CastBSplineSurfaces();
        }

    }



    void SetContinuityInfo(const char *constraintsFile){

        cout << "Reading Continuity Info" << endl;
        edgesContinuity = vector<ContinuityInfo>();

        ifstream infile(constraintsFile);

        int edgeID, continuity, numberofTestPoints;

        while ( infile >> edgeID >> continuity >> numberofTestPoints){

            auto cI =  ContinuityInfo();
            cI.edgeIndex = edgeID;
            auto neighbourgingFaces = mapEdgesOfFace.FindFromIndex(edgeID);
            if (neighbourgingFaces.Extent()!=2){
                cout << "Not 2 faces for the edge " << edgeID << endl;
            }

            TopTools_ListIteratorOfListOfShape itFace;
            int index = 0;
            itFace.Initialize(neighbourgingFaces);
            cI.faceIndexLeft = facemap.FindIndex(itFace.Value());
            itFace.Next();
            cI.faceIndexRight = facemap.FindIndex(itFace.Value());

            cI.numberofTestPoits = numberofTestPoints;
            cI.continuityLevel = Continuity(continuity);

            edgesContinuity.push_back(cI);

        }
       cout <<"Constrained Edges: " << edgesContinuity.size() << endl;

    }

    void SetTestPoints() {

        cout << "Test Points Distribution" << endl;
        for (int i = 0; i < edgesContinuity.size(); ++i) {
            ContinuityInfo cI = edgesContinuity[i];
            edgesContinuity[i] = SetTestPointsOnEdge(cI);;
        }
    }

    void PopulateDesignFacesList(vector<int> designFaces_param){
        designFaces = designFaces_param;
    }

    void PopulateDesignFacesList(){
        for (int i = 0; i < meshPointsOnFace.size() ; ++i) {
            if (meshPointsOnFace[i] > 0)
                designFaces.push_back(i+1);
        }
    }

    void SetDesignControlPoints() {
        //all design faces all control points

        for (int i = 0; i <designFaces.size() ; ++i) {
            auto currentFace = TopoDS::Face(facemap(designFaces[i]));
            auto currentSurface = BRep_Tool::Surface(currentFace);
            Handle(Geom_BSplineSurface) bsplineSurface = Handle(Geom_BSplineSurface)::DownCast(currentSurface);

            int fixedPointsV = 5;
            int fixedPointsU = 0;

            for (int j = 1; j <= bsplineSurface->NbUPoles() ; ++j) {
                for (int k = 1 +fixedPointsV; k <= bsplineSurface->NbVPoles()-fixedPointsV; ++k) {
               //     cout  << i << j << k << bsplineSurface->Pole(j,k).Coord().X() << endl;
                    designControlPoints.faceIndex.push_back(designFaces[i]);
                    designControlPoints.uIndex.push_back(j);
                    designControlPoints.vIndex.push_back(k);
                }
            }

        }
    }

    void MatrixInitialization();

    void GetDerivativeMatrix_ScalarMode_Transposed() {

        cout << "Calculating Derivative Matrices Scalar Mode" << endl;
        //If weights needed - mx_R, mx_C sizes should change

        int N = designControlPoints.faceIndex.size();
        int T = GetTestPointsNumber();
        int M = meshpointsCartesian.size();

        mx_R.resize(3*M,3*N);
        mx_C.resize(3*N,3*T);

        //Probably not needed
        MatrixInitialization();

        for (int k = 0; k < designControlPoints.faceIndex.size(); ++k) {
            if (k%100 == 0){
                cout << " Populated << " << k << " derivatives entries out of " << designControlPoints.faceIndex.size() << endl;
            }

            for (int xyz = 1; xyz <=3 ; ++xyz) {

                Handle(Geom_BSplineSurface) bspline = bsplineSurfaces[designControlPoints.faceIndex[k]-1];
                //seeding CPs
                gp_Pnt p = bspline->Pole(designControlPoints.uIndex[k], designControlPoints.vIndex[k]);
                auto coord_xyz = p.Coord(xyz);
#if !REVERSE_MODE
                coord_xyz.setADValue(0, 1);
#endif
                p.SetCoord(xyz, coord_xyz);
                bspline->SetPole(designControlPoints.uIndex[k], designControlPoints.vIndex[k], p);

                //Matrix_R calculation by column
                ConstructMeshPointsDerivativeMatrix(k,xyz);

                //Matrix_C calculation by row
                ConstructConstraintMatrix(k,xyz);

                //unseed current CP
#if !REVERSE_MODE
                coord_xyz.setADValue(0, 0);
#endif
                p.SetCoord(xyz, coord_xyz);
                bspline->SetPole(designControlPoints.uIndex[k], designControlPoints.vIndex[k], p);
            }
        }

//        Settings_Halfcylinder settings;
////
//             CreateAndWriteTable<MatrixXd>(mx_C,settings.nspcc__projections_meshfile,"mx_C_Last");
//             CreateAndWriteTable<MatrixXd>(mx_R,settings.nspcc__projections_meshfile,"mx_R_Last");

    }

    void GetDerivativeMatrix_VectorMode_Transposed() {
        OSD_Timer aTimer;
        aTimer.Start();

        cout << "Calculating Derivative Matrices Vector MODE!!!: Warning: SET chunk size%3 = 0" << endl;
        //If weights needed - mx_R, mx_C sizes should change

        int N = designControlPoints.faceIndex.size();
        int T = GetTestPointsNumber();
        int M = meshpointsCartesian.size();

        mx_R.resize(3 * M, (3 * N));
        mx_C.resize(3 * N, (3 * T));

        MatrixInitialization();


        //Constructing chuncks logic

        //Could be 4 if we would go for weigths
        //
        int designCPsize = 3;
#if !REVERSE_MODE
        int adChunkSize = adtl::getNumDir();
        int designNumber = designControlPoints.faceIndex.size() * designCPsize;
        int chunksNumber = (designNumber) / adChunkSize +1;

        cout << "chunksNumber" << chunksNumber << " size of element | "<< adChunkSize << endl;

        int k = 0;
        int chunkIndex = 0;
        int currentChunk = 0;

        while (currentChunk < chunksNumber) {
            cout << "Gradients of " << currentChunk << " of " << chunksNumber << "  Calculated" << endl;
            int staringk = k;

            while (k < designControlPoints.faceIndex.size() && chunkIndex < adChunkSize) {

                Handle(Geom_BSplineSurface) bspline = bsplineSurfaces[designControlPoints.faceIndex[k] - 1];
                //seeding CPs
                gp_Pnt p = bspline->Pole(designControlPoints.uIndex[k], designControlPoints.vIndex[k]);
                for (int xyz = 1; xyz <= 3; ++xyz) {
                     auto coord_xyz = p.Coord(xyz);
                    coord_xyz.setADValue(chunkIndex, 1);
                    p.SetCoord(xyz, coord_xyz);
                     chunkIndex++;
                }
                bspline->SetPole(designControlPoints.uIndex[k], designControlPoints.vIndex[k], p);
                k++;
            }

            //Current chunk is seeded

//            //Matrix_R calculation by column
            ConstructMeshPointsDerivativeMatrix_VectorModer(staringk, k);
//
//            //Matrix_C calculation by row
            ConstructConstraintMatrix_VectorMode(staringk, k);

            //Unseed current chunk
            k = staringk;
            chunkIndex = 0;

            while (k < designControlPoints.faceIndex.size() && chunkIndex < adChunkSize) {

                Handle(Geom_BSplineSurface) bspline = bsplineSurfaces[designControlPoints.faceIndex[k] - 1];
                //seeding CPs
                gp_Pnt p = bspline->Pole(designControlPoints.uIndex[k], designControlPoints.vIndex[k]);
                for (int xyz = 1; xyz <= 3; ++xyz) {
                    auto coord_xyz = p.Coord(xyz);
                    coord_xyz.setADValue(chunkIndex, 0);
                    p.SetCoord(xyz, coord_xyz);
                    chunkIndex++;
                }
                bspline->SetPole(designControlPoints.uIndex[k], designControlPoints.vIndex[k], p);
                k++;
            }

            chunkIndex = 0;
            currentChunk++;

        }
        aTimer.Stop();
        cout << aTimer.ElapsedTime() << endl;
#endif
    }

    void FindMesh(Standard_CString meshfile, bool findProjection = true, double scale = 1.0);

    void SVD_Decompose_Transpose(){

        cout << "Calculating SVD" << endl;
        cout << "mx_Eigen_C size " << mx_C.rows() << " / " << mx_C.cols() << endl;

        //  cout << "Here is the matrix m:" << endl << m << endl;
        JacobiSVD<MatrixXd> svd(mx_C.transpose(), ComputeThinU | ComputeFullV);

        cout << "rank = " << svd.rank() << endl;
        int rank  = svd.rank();

        //   cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
      //  cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;



        auto vMatr = svd.matrixV();
        cout << vMatr.rows() << " / " << vMatr.cols() << endl;

        KerC_Stencil.resize(vMatr.cols(),vMatr.cols() - rank);
        for (int k = rank; k < vMatr.cols(); ++k) {
            for (int j = 0; j < vMatr.cols(); ++j) {
                KerC_Stencil(j, k - rank) = vMatr(j, k);
            }
        }
    }

    void CalculateCADGradient(){

        cout << "Calculate CAD gradient: Carefull of the size" << endl;
        CADGradient.resize(mx_R.rows(),KerC_Stencil.cols());
        CADGradient = mx_R * KerC_Stencil;
    }

    void CalculateNormalizedGradient();

    void CalculateCADCFDGradient(){
        gradient = CFDGradinet * CADGradient;
        CalculateNormalizedGradient();
    }

    void CalculateCFDCADGradient_BigSizes(){
        cout << "Calculateing CFD x mx_R" << endl;
        MatrixXd CFD_CP = CFDGradinet * mx_R;
        cout << "Calculateing CFD_CP x Ker_C_Stencil" << endl;
        gradient = CFD_CP * KerC_Stencil;
        CalculateNormalizedGradient();

    }

    void ReadCFDGradient(Standard_CString sensitivityfile);

    void ReadCFDGradient(vector<vector<double>> cfdSens);

    void WriteGradientMatrices_Intermediate(Standard_CString filename){

        cout << "populating MX_C" << endl;
        CreateAndWriteTable<double >(mx_C,filename,"mx_C");
        cout << "populating MX_R" << endl;
        CreateAndWriteTable<double>(mx_R,filename, "mx_R");
        cout << "populating Ker_Stencil" << endl;
        CreateAndWriteTable<MatrixXd>(KerC_Stencil, filename, "Ker_Stencil");
        cout << "populating Gradient" << endl;
        CreateAndWriteTable<MatrixXd>(CADGradient, filename, "CADGradient");

    }

    void WriteGradintMatrices_Final(Standard_CString filename, string postfix = ""){

        cout << "Continuity Gradient" << endl;
        CreateAndWriteTable<MatrixXd>(gradient, filename, ("CFDCADGradient"+postfix).c_str());
        cout << "populating normalised Gradient" << endl;
        CreateAndWriteTable<MatrixXd>(normalizedGradient, filename, ("normalisedGradient"+postfix).c_str());
        cout << "populating Ker_C" << endl;
        CreateAndWriteTable<MatrixXd>(KerC_Stencil, filename, ("Ker_C"+postfix).c_str());

    }

    void ReadGradientMatrices(Standard_CString filename){

        cout << "Reading Gradient Matrices" << endl;
        mx_C = getTable_Eigen<MatrixXd>(filename,"mx_C");
        mx_R =   getTable_Eigen<MatrixXd>(filename,"mx_R");
        KerC_Stencil = getTable_Eigen<MatrixXd>(filename,"Ker_Stencil");
        CADGradient = getTable_Eigen<MatrixXd>(filename,"CADGradient");

      }

//#pragma region Testing Methods

static vector<vector<double>> MatrixToVector(MatrixXd a){
    vector<vector<double>> vec_a(a.rows(),vector<double>(a.cols()));
    for (int i = 0; i < a.rows(); ++i) {
        for (int j = 0; j < a.cols() ; ++j) {
            vec_a[i][j] = a(i,j);
        }

    }
    return vec_a;

}
    static void Eigen_Testing() {
//        MatrixXd A(3,2);
//        cout << A.cols() << A.rows() << endl;
//        cout << A << endl;
//        vector<vector<int>> a(3,vector<int>(2));
//        cout << a.size() << a[0].size() << endl;

        MatrixXd mR = MatrixXd::Random(3300, 3366);
       cout << mR(0,0) << endl;
        cout << mR.cols() << mR.rows() << endl;
//        MatrixXd res = mR * KerC;

        CreateAndWriteTable<double>(mR,Settings_Stator().gradientMatrices,"EigenTestRandom3300/3366Vector");

        cout << "matrix Created " << endl;

//       // m << 1, 0, 0, 0, 2, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0;
//        m << 2,4,1,3,0,0,0,0;
//
//        cout << "Here is the matrix m:" << endl << m << endl;
//        JacobiSVD<MatrixXd> svd(m, ComputeFullU | ComputeFullV);
//
//        cout << "svd computed" << endl;
//        cout << svd.rank() << endl;
//        cout << svd.singularValues()<<endl;
//        cout << svd.matrixV() << endl;
//
//        MatrixXd S = MatrixXd::Random(4, 2);
//        S<< 5.47, 0, 0, 0.37,0,0,0,0;
//
//        cout << "multiplied matr" << endl;
//        cout << svd.matrixU() * S * svd.matrixV().transpose() << endl;

    }

    void SVD_Testing(){

        cout << "Testing SVD" << endl;
        cout << "Ker Stencil rows: " << KerC_Stencil.rows() << endl;

        int alpha_size = KerC_Stencil.rows();
        VectorXd alpha(alpha_size);

        vector<double> coeff = {1,2,3,4,5,6,3,7,2,6,3,7,3,7,1,7,83,7,3,6,9 ,124,235,125,1};

        for (int i = 0; i < alpha_size; ++i) {
            alpha[i]  = 0;
            for (int j = 0; j < KerC_Stencil.cols(); ++j) {
                // alpha[i]+=coeff[j]*KerC_Stencil(i,j);
                alpha[i]+=KerC_Stencil(i,j);

            }
        }
        cout << alpha_size << endl;
        cout << mx_C.rows() << " / " << mx_C.cols() << endl;

        cout << mx_C.transpose() * alpha << endl;
    }

    void KernelTesting(Standard_CString updatedStepFile){
        int size_alpha = KerC_Stencil.cols();
        MatrixXd allpha(size_alpha,1);
        for (int i = 0; i < size_alpha ; ++i) {
//            if (i == 1 || i==2 || i == 5) {
            if (i%10 == 0){
                allpha(i, 0) = 1;
            }
            else{
                allpha(i,0) = 0;
            }
        }
        cout << "Kernel Testing" << size_alpha << " // " << endl;
         MatrixXd sigmaP = KerC_Stencil * allpha;
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
        cout << "StepWriting" << endl;


//        Handle(BRepTools_ReShape) theReShape = new BRepTools_ReShape();
//
//        for (int m = 0; m <bsplineSurfaces.size() ; ++m) {
//           auto newface = BRepBuilderAPI_MakeFace(bsplineSurfaces[m],10E-3).Shape();
//            theReShape->Replace(facemap(m+1),newface);
//        }
//        auto result = theReShape->Apply(myShape);
//
        for (int m = 0; m <bsplineSurfaces.size() ; ++m) {
            auto stepfile = settings.nspcc_dir + "facePerturbed_4f_SmallPerturb_FixedEdges" + to_string(m) + ".stp";
            StepWriter(stepfile.c_str(),BRepBuilderAPI_MakeFace(bsplineSurfaces[m],10E-3));
        }

        auto result = BRepBuilderAPI_MakeFace(bsplineSurfaces[0],10E-3).Shape();
        for (int l = 1; l <bsplineSurfaces.size() ; ++l) {
            auto nextFace = BRepBuilderAPI_MakeFace(bsplineSurfaces[l],10E-3).Shape();
            result = generalFuse(result,nextFace);
        }
//
       auto stepfile_updated = settings.nspcc_dir + "faces_generalFused_FixedEdges" + to_string(edgesContinuity[0].numberofTestPoits)+".stp";
        StepWriter(stepfile_updated.c_str(),result);
    }

    void MatrixC_FD_Test() {

        int poleindexU = 2;
        int poleindexV = 1;

        gp_Pnt point = bsplineSurfaces[1]->Pole(poleindexU, poleindexV);
        vector<double> fd(edgesContinuity[0].numberofTestPoits * 3);
        vector<double> fdp(edgesContinuity[0].numberofTestPoits * 3);
        int testindex = 0;

        auto surfaceLeft = bsplineSurfaces[edgesContinuity[0].faceIndexLeft - 1];
        auto surfaceRight = bsplineSurfaces[edgesContinuity[0].faceIndexRight - 1];

        cout << "UV Bounds " << endl;
        Standard_Real u1, u2, v1, v2;
        bsplineSurfaces[0]->Bounds(u1, u2, v1, v2);
        cout << "Surface 0 bounds" << u1 << " | " << u2 << " | " << v1 << " | " << v2 << " | " << "U Poles"
             << bsplineSurfaces[0]->NbUPoles() << "   V Poles" << bsplineSurfaces[0]->NbVPoles() << endl;

        bsplineSurfaces[1]->Bounds(u1, u2, v1, v2);
        cout << "Surface 0 bounds" << u1 << " | " << u2 << " | " << v1 << " | " << v2 << " | " << "U Poles"
             << bsplineSurfaces[1]->NbUPoles() << "   V Poles" << bsplineSurfaces[0]->NbVPoles() << endl;
        cout << "Testpoints Parametric " << endl;

        for (int i = 0; i < edgesContinuity[0].numberofTestPoits; ++i) {

            cout << "point" << i << ":  " << endl;
            cout << "Left: " << edgesContinuity[0].testPointsParametricLeft[i].X().getValue() << " | "
                 << edgesContinuity[0].testPointsParametricLeft[i].Y().getValue() << endl;
            cout << "Right: " << edgesContinuity[0].testPointsParametricRight[i].X().getValue() << " | "
                 << edgesContinuity[0].testPointsParametricRight[i].Y().getValue() << endl;
            cout << "------" << endl;
            gp_Pnt pL, pR;
            surfaceLeft->D0(edgesContinuity[0].testPointsParametricLeft[i].X(),
                            edgesContinuity[0].testPointsParametricLeft[i].Y(), pL);
            surfaceRight->D0(edgesContinuity[0].testPointsParametricRight[i].X(),
                             edgesContinuity[0].testPointsParametricRight[i].Y(), pR);
            for (int j = 1; j <= 3; ++j) {
                fd[testindex] = (pL.Coord(j).getValue() - pR.Coord(j).getValue());
                testindex++;
            }

        }

        testindex = 0;
        point.SetCoord(3, point.Z() + 10e-3);
        bsplineSurfaces[1]->SetPole(poleindexU, poleindexV, point);

        for (int i = 0; i < edgesContinuity[0].numberofTestPoits; ++i) {
            gp_Pnt pL, pR;
            surfaceLeft->D0(edgesContinuity[0].testPointsParametricLeft[i].X(),
                            edgesContinuity[0].testPointsParametricLeft[i].Y(), pL);
            surfaceRight->D0(edgesContinuity[0].testPointsParametricRight[i].X(),
                             edgesContinuity[0].testPointsParametricRight[i].Y(), pR);
            for (int j = 1; j <= 3; ++j) {
                fdp[testindex] = pL.Coord(j).getValue() - pR.Coord(j).getValue();
                testindex++;
            }

        }

        for (int k = 0; k < fd.size(); ++k) {
            cout << (fdp[k] - fd[k]) / 10e-3 << " | ";
        }
        cout << endl;

        for (int k = 0; k < fd.size(); ++k) {
            cout << fd[k] << " | ";
        }
        cout << endl;

        for (int k = 0; k < fd.size(); ++k) {
            cout << fdp[k] << " | ";
        }
        cout << endl;

    }

    void FacePerturb_Test(){

        int poleindexU = 2;
        int poleindexV = 1;

        gp_Pnt point =  bsplineSurfaces[1]->Pole(poleindexU,poleindexV);
        vector<double> fd(edgesContinuity[0].numberofTestPoits*3);
        vector<double> fdp(edgesContinuity[0].numberofTestPoits*3);
        int testindex = 0;

        auto surfaceLeft = bsplineSurfaces[edgesContinuity[0].faceIndexLeft-1];
        auto surfaceRight = bsplineSurfaces[edgesContinuity[0].faceIndexRight-1];


        testindex =0;
        point.SetCoord(3,point.Z()+3);
        bsplineSurfaces[1]->SetPole(poleindexU,poleindexV, point);

        auto face = BRepBuilderAPI_MakeFace(bsplineSurfaces[1],10e-6);
        StepWriter("/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/facePert.stp", face);

    }

//#pragma endregion Testing Methods


//! \param Body          [in]  body shell.
//! \param Contour       [in]  contour to fuse into the body shell.
//! \param fuzz          [in]  fuzzy value to use.
//! \param ResultBody    [out] fused shape.
//! \param ResultContour [out] contour image after general fuse.
//! \return true in case of success, false -- otherwise.
TopoDS_Shape generalFuse(const TopoDS_Shape& Body,  const TopoDS_Shape& Contour);

    void PrintTestPointsInfo();

    void PerformDesignStep(double scaleFactor=10);

    void WriteUpdatedFaces(string postfix="");

    void WriteMeshPoints_ToTextFile(string mesh_filename, vector<gp_Pnt> meshpointsToFile, bool pureCoordinates, double meshScale);

    void FreeMatrices();

    void ReadGradintMatrices_Final(Standard_CString mystring, string postfix="");

    void SensitivityBasedInsertion(vector<int> indexes);

    void ManualVKnotInsertion(vector<double> vValues);

    void CastBSplineSurfaces();

    double DesignStep_NSPCC();

    void StepWriter_FacesNSPCC(const char *string);

    void ManualUKnotInsertion(vector<double> list);

    void PrintCP_Udistribution(int faceindex);

    void PrintCP_XDistribution(int faceindex);

private:

     ContinuityInfo SetTestPointsOnEdge(ContinuityInfo cIIn);

     NCollection_Array1<gp_Pnt> ReadMeshPoints(string mesh_filename, bool pureCoordinates = false);

     int GetTestPointsNumber();

    void ConstructConstraintMatrix(int k, int xyz, int adIndex=0);

     void ConstructConstraintMatrix_VectorMode(int startK, int endK);

     void ConstructMeshPointsDerivativeMatrix(int k, int xyz, int adIndex=0);

    void ConstructMeshPointsDerivativeMatrix_VectorModer(int startK, int endK);

    vector<Standard_Real> VectorProduct(gp_Dir a, gp_Dir b);

};



#endif //U_BENDOPTIMISATION_NSPCC_H
