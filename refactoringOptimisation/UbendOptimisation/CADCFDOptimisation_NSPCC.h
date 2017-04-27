//
// Created by orestmykhaskiv on 26/10/16.
//

#ifndef U_BENDOPTIMISATION_CADCFDOPTIMISATION_NSPCC_H
#define U_BENDOPTIMISATION_CADCFDOPTIMISATION_NSPCC_H

#include "CADCFDOptimisation.h"
#include "NSPCC.h"

class CADCFDOptimisation_NSPCC: public CADCFDOptimisation{
public:

    occtContinuityConstraints nspcc_cad = occtContinuityConstraints(nullptr);
    Settings_Optimisation_TUB_NSPCC settings;


    CADCFDOptimisation_NSPCC(bool isNSPCC);

    double DesignStepMeshToCAD_NSPCC();


    void CADSensitivity_NSPCC();

    vector<double> ConvertToVector(MatrixXd matrix);

    double DesignStep_NSPCC();

    void StepWriter_FacesNSPCC(int iteration);

    void RunNSPCCOptimisation_SteepestDescent(){
#if !REVERSE_MODE
        cout << "Running NSPCC with AD directions = "<< adtl::getNumDir()<<endl;
#endif
     //was   double maxStepSize = 0.0005 * 4;// * 10e-8;
        double maxStepSize = 0.5 * 4;// * 10e-8;
        vector<Standard_Real> a={};
        bool restart = true;
        auto obj = CFDCostFunction(restart);

        auto cfdSens = CFDSensitivity(true);
        nspcc_cad.ReadCFDGradient(cfdSens);
        CADSensitivity_NSPCC();


        auto g = ConvertToVector(nspcc_cad.normalizedGradient);

        int k = 1;           //k = # iterations
//
        OptimisationState optimisationState(mgopt,nspcc_cad,k,obj,g,a);

        string iterationStepFile = mgopt.CasePath + "/optimisation/CAD/initialDesign_00.stp";
        nspcc_cad.StepWriter(iterationStepFile.c_str());
      //  StepWriter_FacesNSPCC(k);
        mgopt.BackupSolution("/optimisation/CFD", "iteration_00_MeshOnCAD");

        optimisationState.WriteState();
        optimisationState.WriteCostData(true);
       // optimisationState.WriteParamsData(true);

        int nf = 1;        // nf = # function eval.
        int armijosteps = 0;


        while (norm(g) > 1e-3 && k <= 10) {

            int cfdBackUpIteration = 1;
            double scaleFactor =  - maxStepSize;
            nspcc_cad.PerformDesignStep(scaleFactor);
            StepWriter_FacesNSPCC(k);


            double maxMeshDeform;
            maxMeshDeform = DesignStep_NSPCC();

            nf = nf + 1;
            auto newobj = CFDCostFunction(restart);


            obj = newobj;

            iterationStepFile = mgopt.CasePath + "/optimisation/CAD/currentDesign_" + std::to_string(k) + ".stp";
            nspcc_cad.StepWriter(iterationStepFile.c_str());
            StepWriter_FacesNSPCC(k);


            auto cfdSens = CFDSensitivity(restart);
            nspcc_cad.ReadCFDGradient(cfdSens);
            CADSensitivity_NSPCC();
            g = ConvertToVector(nspcc_cad.normalizedGradient);

            OptimisationState optimisationState(mgopt,nspcc_cad,k,obj,g,a);


            optimisationState.WriteState();
            optimisationState.WriteCostData();


            if (k % cfdBackUpIteration == 0)
                mgopt.BackupSolution("/optimisation/CFD", "iteration_" + to_string(k + 1));

            if (optimisationState.GradNorm > 1e+20) {
                cout << "Norm too big, exiting ..." << endl;
                break;
            }


            k = k + 1;


        }




    };

    void SetMeshOnCAD(bool projectMesh=false);

private:

    void SetCAD();

    void SetCFD();


};









CADCFDOptimisation_NSPCC::CADCFDOptimisation_NSPCC(bool isNSPCC) : CADCFDOptimisation(isNSPCC) {
    SetCAD();
    SetCFD();
}

void CADCFDOptimisation_NSPCC::SetCAD() {
    nspcc_cad  = occtContinuityConstraints(settings.nspcc__cadfile);
    nspcc_cad.SetContinuityInfo(settings.nspcc__constraintfile);
    nspcc_cad.SetTestPoints();
    nspcc_cad.PrintTestPointsInfo();
    nspcc_cad.PopulateDesignFacesList(settings.nspcc_designFaces);
    nspcc_cad.SetDesignControlPoints();
}

void CADCFDOptimisation_NSPCC::SetCFD() {
    string mgoptFolder = settings.mgoptFolder;
    string mgoptCase =settings.mgoptCase;
    string mgoptJsonFile=settings.mgoptJsonFile;
    string ExternalLibraryFolder = settings.ExternalLibraryFolder;
    string RestartSettingsFile = settings.RestartSettingsFile;

    mgopt=MgoptSolver(mgoptFolder,mgoptCase,mgoptJsonFile,ExternalLibraryFolder);
    mgopt.RestartSettingsFile = RestartSettingsFile;

}

double CADCFDOptimisation_NSPCC::DesignStepMeshToCAD_NSPCC() {

        cout << "Calculating Pertubed Mesh" << endl;
        nspcc_cad.CalculatePertubedMesh();
        auto deformations = nspcc_cad.Find3dMeshDisplacement();
        mgopt.PopulateDisplacementsTable(deformations);

        mgopt.RunSolverByMode(7);

        //Dont forget to update your mesh!!!!
        nspcc_cad.UpdateMeshFromPerturbed();

        return maxDeformation(deformations);
    }

void CADCFDOptimisation_NSPCC::SetMeshOnCAD(bool projectMesh) {

    auto designTable = mgopt.GetDesignTable();
    auto meshTable = mgopt.GetMeshTable();


    if (projectMesh) {
        nspcc_cad.SetMeshes(meshTable, designTable);
        nspcc_cad.WriteProjectionsData_OCCT(settings.nspcc__projections_meshfile);
    }
    else {
        auto projectionFile = settings.nspcc__projections_meshfile;
        nspcc_cad.SetMeshesNoProjection(meshTable, designTable);
        nspcc_cad.ReadProjectionData_OCCT(projectionFile);
        cout << "Projection data read successfully" << endl;

    }

    DesignStepMeshToCAD_NSPCC();
}

void CADCFDOptimisation_NSPCC::CADSensitivity_NSPCC() {
#if !REVERSE_MODE
    if (adtl::getNumDir() > 1) {
        nspcc_cad.GetDerivativeMatrix_VectorMode_Transposed();
    }
    else{
        nspcc_cad.GetDerivativeMatrix_ScalarMode_Transposed();
    }
    nspcc_cad.SVD_Decompose_Transpose();
    nspcc_cad.CalculateCFDCADGradient_BigSizes();
#endif
}

vector<double> CADCFDOptimisation_NSPCC::ConvertToVector(MatrixXd matrix) {
    int n  = matrix.cols();
    vector<double> vec(n);
    for (int i = 0; i < n ; ++i) {
        vec[i] = matrix(0,i);
    }
    return vec;
}

double CADCFDOptimisation_NSPCC::DesignStep_NSPCC() {

    cout << "Calculating Pertubed Mesh" << endl;
    nspcc_cad.CalculatePertubedMesh();
    auto deformations = nspcc_cad.Find3dMeshDisplacement();
    mgopt.PopulateDisplacementsTable(deformations);
    mgopt.RunSolverByMode(7);
    nspcc_cad.UpdateMeshFromPerturbed();

    return maxDeformation(deformations);
}

void CADCFDOptimisation_NSPCC::StepWriter_FacesNSPCC(int iteration) {

        for (int m = 0; m <nspcc_cad.bsplineSurfaces.size() ; ++m) {
            auto stepfile = settings.nspcc_dir_faces + "iter" + to_string(iteration)+"_face" + to_string(m) + ".stp";
            nspcc_cad.StepWriter(stepfile.c_str(),BRepBuilderAPI_MakeFace(nspcc_cad.bsplineSurfaces[m],10E-3));
        }
    }


#endif //U_BENDOPTIMISATION_CADCFDOPTIMISATION_NSPCC_H
