//
// Created by orestmykhaskiv on 25/10/16.
//

#ifndef U_BENDOPTIMISATION_NSPCC_CASES_RUN_H
#define U_BENDOPTIMISATION_NSPCC_CASES_RUN_H

#include "NSPCC.h"

void TestingNSPCC(){

    string continuityPath ="/home/orestmykhaskiv/QMUL/mgoptCustomCases/ubdend12_10Scale10/continuity";
    string CADfile = continuityPath +"/currentDesign_0.stp";
    string constraintFile = continuityPath +"/constraintFile";


  //  string CADfile = "/home/orestmykhaskiv/QMUL/mgoptCustomCases/ubdend12_10Scale10/continuity/cu"
    cout << CADfile << endl;


    occtContinuityConstraints occtConstraints(CADfile.c_str());

    occtConstraints.SetContinuityInfo(constraintFile.c_str());
    occtConstraints.SetTestPoints();



//        for(auto &edgesIter : occtConstraints.edgesContinuity) {
//
////           // auto x = occtConstraints.edgesContinuity.find(14);
////            if (x!=occtConstraints.edgesContinuity.end()) {
////                cout << x->second.faceIndexLeft << endl;
////            }
//            cout << "edge "<< edgesIter.edgeIndex << endl;
//            ContinuityInfo cI = occtConstraints.edgesContinuity.find(edgesIter.first)->second;
//            auto parametricTestPointsLeft =cI.testPointsParametricLeft;
//            auto parametricTestPointsRight =cI.testPointsParametricLeft;
//
//            cout << parametricTestPointsLeft.size() << endl;
//            for (int j = 0; j < parametricTestPointsLeft.size(); ++j) {
//                cout << "(" << parametricTestPointsLeft[j].X().getValue() << "," << parametricTestPointsLeft[j].Y().getValue() << ")"  << "   <===>    "<< "(" << parametricTestPointsRight[j].X().getValue() << "," << parametricTestPointsRight[j].Y().getValue() << ")" << endl;
//            }
//
//        }
}

void TestingNSPCC_Halfcylinder(){

    Settings_Halfcylinder nspccSettings;

    //Probably should go in occtConstructor
    occtContinuityConstraints occtConstraints(nspccSettings.nspcc__cadfile);
    //occtConstraints.Eigen_Testing();
    occtConstraints.AnalyseShape();
    occtConstraints.SetContinuityInfo(nspccSettings.nspcc__constraintfile);
    occtConstraints.SetTestPoints();
    occtConstraints.PopulateDesignFacesList(nspccSettings.nspcc_designFaces);


    occtConstraints.FindMesh(nspccSettings.nspcc__meshfile,true);
    occtConstraints.WriteProjectionsData_OCCT(nspccSettings.nspcc__projections_meshfile2);

    occtConstraints.ReadProjectionData_OCCT(nspccSettings.nspcc__projections_meshfile2);

    occtConstraints.SetDesignControlPoints();
    occtConstraints.SensitivityBasedInsertion({73});

    occtConstraints.AnalyseShape();
    occtConstraints.StepWriter(nspccSettings.nspcc__cadfile_updated);
//    occtConstraints.GetDerivativeMatrix_VectorMode_Transposed();
//
//    occtConstraints.SVD_Decompose_Transpose();
//    occtConstraints.WriteGradientMatrices_Intermediate(nspccSettings.gradientMatrices);
////    Settings_Stator settings_stator;
////    occtConstraints.ReadCFDGradient(settings_stator.cfdSensitivityFile);
////    occtConstraints.CalculateCADCFDGradient();
//  //  occtConstraints.SVD_Testing();
//
////
//    occtConstraints.KernelTesting(nspccSettings.nspcc__cadfile_updated);

//    occtConstraints.MatrixC_FD_Test();
   //SVDTest();
//   occtConstraints.Eigen_Testing();

}

void TestingNSPCC_Stator(){

    //occtContinuityConstraints::Eigen_Testing();
    //!!! stator TESTING !!!!
    Settings_Stator nspccSettings;

    //Probably should go in occtConstructor
    occtContinuityConstraints occtConstraints(nspccSettings.nspcc__cadfile_sewed);
//    occtConstraints.AnalyseShape();
//
    occtConstraints.SetContinuityInfo(nspccSettings.nspcc__constraintfile);
    occtConstraints.SetTestPoints();
  //  occtConstraints.PrintTestPointsInfo();
    occtConstraints.PopulateDesignFacesList(nspccSettings.nspcc_designFaces);

    occtConstraints.FindMesh(nspccSettings.nspcc__meshfile, false,1000);
   //occtConstraints.WriteProjectionsData_OCCT(nspccSettings.nspcc__projections_meshfile);
    occtConstraints.ReadProjectionData_OCCT(nspccSettings.nspcc__projections_meshfile);
////
    occtConstraints.SetDesignControlPoints();
//    occtConstraints.GetDerivativeMatrix_VectorMode_Transposed();
//    occtConstraints.SVD_Decompose_Transpose();
//
    occtConstraints.ReadCFDGradient(nspccSettings.cfdSensitivityFile);
    occtConstraints.CalculateCFDCADGradient_BigSizes();
    occtConstraints.FreeMatrices();
//    occtConstraints.WriteGradintMatrices_Final(nspccSettings.gradientMatrices,"_CFD1_no_constraints");
    occtConstraints.ReadGradintMatrices_Final(nspccSettings.gradientMatrices,"_CFD1_no_constraints");
//
//
//
     occtConstraints.PerformDesignStep();
    occtConstraints.CalculatePertubedMesh();
    auto  displacemnet  = occtConstraints.Find3dMeshDisplacement();
    for (int i = 0; i <3 ; ++i) {
        cout << "maxDisplacement" <<i << " = " << *max_element(displacemnet[i].begin(),displacemnet[i].end()) << endl;
    }

    occtConstraints.WriteMeshPoints_ToTextFile(nspccSettings.updateMeshFile, occtConstraints.meshpointsCartesianPerturbed,true,10E-4);
//
////
////
    occtConstraints.WriteUpdatedFaces("_CFD1_no_constraints");



  //  occtConstraints.KernelTesting(nspccSettings.nspcc__cadfile_updated);
}

void TestingNSPCC_WingFaces(){
    //!!! stator TESTING !!!!
    Settings_WingFaces nspccSettings;

    //Probably should go in occtConstructor
    occtContinuityConstraints occtConstraints(nspccSettings.nspcc__cadfile_sewed);
    occtConstraints.AnalyseShape();
//    occtConstraints.StepWriter(nspccSettings.nspcc__cadfile_sewed);

    occtConstraints.SetContinuityInfo(nspccSettings.nspcc__constraintfile);
    occtConstraints.SetTestPoints();
    occtConstraints.PopulateDesignFacesList(nspccSettings.nspcc_designFaces);

    occtConstraints.FindMesh(nspccSettings.nspcc__meshfile, false);
  // occtConstraints.WriteProjectionsData_OCCT(nspccSettings.nspcc__projections_meshfile);
    occtConstraints.ReadProjectionData_OCCT(nspccSettings.nspcc__projections_meshfile);
    occtConstraints.SensitivityBasedInsertion({100});

    occtConstraints.AnalyseShape();
    occtConstraints.StepWriter(nspccSettings.nspcc__cadfile_updated);
////
//    occtConstraints.SetDesignControlPoints();
//    occtConstraints.GetDerivativeMatrix_ScalarMode_Transposed();
//    occtConstraints.SVD_Decompose_Transpose();
//    occtConstraints.SVD_Testing();
////
//    occtConstraints.KernelTesting(nspccSettings.nspcc__cadfile_updated);
}

void TestingNSPCC_Ubend(){
    Settings_Ubend_NSPCC nspccSettings;
    occtContinuityConstraints occtConstraints(nspccSettings.nspcc__cadfile);


    std::vector<Standard_Real> design;

        design=
                {
                        Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375),
                        Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375),
                        Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375),
                        Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375), Standard_Real(0.0375),
                        Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875), Standard_Real(0.0375), Standard_Real(0.01875),
                        Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875), Standard_Real(0.0375), Standard_Real(-0.01875),
                        Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375), Standard_Real(0.0375), Standard_Real(-0.0375),
                        Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375),
                        Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375),
                        Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375), Standard_Real(-0.0375),
                        Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875), Standard_Real(-0.0375), Standard_Real(-0.01875),
                        Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875), Standard_Real(-0.0375), Standard_Real(0.01875)
                };
    occtConstraints.designParameters = design;

    occtConstraints.BuildShape();
    occtConstraints.BuildConnectivity();
    occtConstraints.CastBSplineSurfaces();
    occtConstraints.AnalyseShape();

    occtConstraints.FindMesh(nspccSettings.nspcc__meshfile, false);
    occtConstraints.ReadProjectionData_OCCT(nspccSettings.nspcc__projections_meshfile);

    occtConstraints.SensitivityBasedInsertion(nspccSettings.sensitivityKnotIndexes);
    cout << "Updated Analyse" << endl;
    occtConstraints.AnalyseShape();
    occtConstraints.StepWriter(nspccSettings.nspcc__cadfile_updated);
}


#endif //U_BENDOPTIMISATION_NSPCC_CASES_RUN_H
