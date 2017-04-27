//
// Created by orestmykhaskiv on 08/02/16.
//

#ifndef U_BENDOPTIMISATION_CADCFDOPTIMISATION_H
#define U_BENDOPTIMISATION_CADCFDOPTIMISATION_H

#include "OCCTDataProvider.h"
#include "MgoptSolver.h"
#include "ConjugateGradient.h"
#include "OptimisationState.h"
#include "Settings.h"

extern "C" {
   void setulb_(int*,int*,double*,double*,double*,int*,double*,double*,double*,double*,double*,int*,char*,int*,char*,bool*,int*,double*);
}

class CADCFDOptimisation {
public:

    MgoptSolver mgopt;
    OCCTDataProvider occt;


    CADCFDOptimisation(bool isNSPCC){
      //Do nothing
    }


    CADCFDOptimisation(MgoptSolver mgoptSolver, OCCTDataProvider occtDataProvider){
        mgopt=mgoptSolver;
        occt = occtDataProvider;
    }

    CADCFDOptimisation(string params = ""){

        std::vector<Standard_Real> design;
        //UBend-March8
        if (params ==""){

          // UBEND

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


          //TUB design parameters - December 14th
          
       /*   design =
          {

Standard_Real(2.6308183963009), 
Standard_Real(2.6308183963009), 
Standard_Real(2.6308183963009), 
Standard_Real(2.6308183963009), 
Standard_Real(2.6308183963009), 
Standard_Real(2.6308183963009), 
Standard_Real(2.6308183963009), 
Standard_Real(2.6308183963009), 

Standard_Real(1.1), 
Standard_Real(1.1), 
Standard_Real(1.1), 
Standard_Real(1.1), 
Standard_Real(1.1), 
Standard_Real(1.1), 
Standard_Real(1.1), 
Standard_Real(1.1), 

Standard_Real(5.8268821212689),
Standard_Real(5.8268821212689),
Standard_Real(5.8268821212689),
Standard_Real(5.8268821212689),
Standard_Real(5.8268821212689),
Standard_Real(5.8268821212689),
Standard_Real(5.8268821212689),
Standard_Real(5.8268821212689),

Standard_Real(7.6154289775438), 
Standard_Real(7.6154289775438), 
Standard_Real(7.6154289775438), 
Standard_Real(7.6154289775438), 
Standard_Real(7.6154289775438), 
Standard_Real(7.6154289775438), 
Standard_Real(7.6154289775438), 
Standard_Real(7.6154289775438), 

Standard_Real(7.5525115843064), 
Standard_Real(7.5525115843064), 
Standard_Real(7.5525115843064), 
Standard_Real(7.5525115843064), 
Standard_Real(7.5525115843064), 
Standard_Real(7.5525115843064), 
Standard_Real(7.5525115843064), 
Standard_Real(7.5525115843064),  

Standard_Real(6.1700380544063), 
Standard_Real(6.1700380544063), 
Standard_Real(6.1700380544063), 
Standard_Real(6.1700380544063), 
Standard_Real(6.1700380544063), 
Standard_Real(6.1700380544063), 
Standard_Real(6.1700380544063), 
Standard_Real(6.1700380544063),  

Standard_Real(4.146690552365), 
Standard_Real(4.146690552365), 
Standard_Real(4.146690552365), 
Standard_Real(4.146690552365), 
Standard_Real(4.146690552365), 
Standard_Real(4.146690552365), 
Standard_Real(4.146690552365), 
Standard_Real(4.146690552365), 

Standard_Real(2.6558797950932), 
Standard_Real(2.6558797950932), 
Standard_Real(2.6558797950932), 
Standard_Real(2.6558797950932), 
Standard_Real(2.6558797950932), 
Standard_Real(2.6558797950932), 
Standard_Real(2.6558797950932), 
Standard_Real(2.6558797950932),  

Standard_Real(1.1177341135095), 
Standard_Real(1.1177341135095), 
Standard_Real(1.1177341135095), 
Standard_Real(1.1177341135095), 
Standard_Real(1.1177341135095), 
Standard_Real(1.1177341135095), 
Standard_Real(1.1177341135095), 
Standard_Real(1.1177341135095),  

Standard_Real(1.1),
Standard_Real(1.1),
Standard_Real(1.1),
Standard_Real(1.1),
Standard_Real(1.1),
Standard_Real(1.1),
Standard_Real(1.1),
Standard_Real(1.1),

Standard_Real(0.),
Standard_Real(0.),  
Standard_Real(0.),
Standard_Real(0.), 
Standard_Real(0.),
Standard_Real(0.), 
Standard_Real(0.),
Standard_Real(0.), 
Standard_Real(0.),
Standard_Real(0.), 
Standard_Real(0.),
Standard_Real(0.), 
Standard_Real(0.),
Standard_Real(0.), 
Standard_Real(0.),
Standard_Real(0.), 

Standard_Real(3.9273781816492), 
Standard_Real(-3.1238645178197), 
Standard_Real(3.9273781816492), 
Standard_Real(-3.1238645178197), 
Standard_Real(3.9273781816492), 
Standard_Real(-3.1238645178197), 
Standard_Real(3.9273781816492), 
Standard_Real(-3.1238645178197), 
Standard_Real(3.9273781816492), 
Standard_Real(-3.1238645178197), 
Standard_Real(3.9273781816492), 
Standard_Real(-3.1238645178197), 
Standard_Real(3.9273781816492), 
Standard_Real(-3.1238645178197), 
Standard_Real(3.9273781816492), 
Standard_Real(-3.1238645178197), 

Standard_Real(15.069818079703), 
Standard_Real(-16.263721964807),
Standard_Real(15.069818079703), 
Standard_Real(-16.263721964807),
Standard_Real(15.069818079703), 
Standard_Real(-16.263721964807),
Standard_Real(15.069818079703), 
Standard_Real(-16.263721964807),
Standard_Real(15.069818079703), 
Standard_Real(-16.263721964807),
Standard_Real(15.069818079703), 
Standard_Real(-16.263721964807),
Standard_Real(15.069818079703), 
Standard_Real(-16.263721964807),
Standard_Real(15.069818079703), 
Standard_Real(-16.263721964807),

Standard_Real(47.427835157742), 
Standard_Real(-28.465840456847), 
Standard_Real(47.427835157742), 
Standard_Real(-28.465840456847), 
Standard_Real(47.427835157742), 
Standard_Real(-28.465840456847), 
Standard_Real(47.427835157742), 
Standard_Real(-28.465840456847), 
Standard_Real(47.427835157742), 
Standard_Real(-28.465840456847), 
Standard_Real(47.427835157742), 
Standard_Real(-28.465840456847), 
Standard_Real(47.427835157742), 
Standard_Real(-28.465840456847), 
Standard_Real(47.427835157742), 
Standard_Real(-28.465840456847), 

Standard_Real(94.234518669567), 
Standard_Real(-37.422788759926), 
Standard_Real(94.234518669567), 
Standard_Real(-37.422788759926), 
Standard_Real(94.234518669567), 
Standard_Real(-37.422788759926), 
Standard_Real(94.234518669567), 
Standard_Real(-37.422788759926), 
Standard_Real(94.234518669567), 
Standard_Real(-37.422788759926), 
Standard_Real(94.234518669567), 
Standard_Real(-37.422788759926), 
Standard_Real(94.234518669567), 
Standard_Real(-37.422788759926), 
Standard_Real(94.234518669567), 
Standard_Real(-37.422788759926), 

Standard_Real(163.00530607726), 
Standard_Real(-39.127239784476),
Standard_Real(163.00530607726), 
Standard_Real(-39.127239784476),
Standard_Real(163.00530607726), 
Standard_Real(-39.127239784476),
Standard_Real(163.00530607726), 
Standard_Real(-39.127239784476),
Standard_Real(163.00530607726), 
Standard_Real(-39.127239784476),
Standard_Real(163.00530607726), 
Standard_Real(-39.127239784476),
Standard_Real(163.00530607726), 
Standard_Real(-39.127239784476),
Standard_Real(163.00530607726), 
Standard_Real(-39.127239784476),

Standard_Real(-38.94255081924), 
Standard_Real(-38.94255081924), 
Standard_Real(-38.94255081924), 
Standard_Real(-38.94255081924), 
Standard_Real(-38.94255081924), 
Standard_Real(-38.94255081924), 
Standard_Real(-38.94255081924), 
Standard_Real(-38.94255081924)  

          };*/
          
        }
        else
        {
            istringstream iss(params);
            vector<string> tokens{istream_iterator<string>{iss},
                                  istream_iterator<string>{}};

            design = vector<Standard_Real>(tokens.size());

            for (int i = 0; i< tokens.size();i++)
                design[i] = Standard_Real(stod(tokens[i]));
        }

        occt=OCCTDataProvider(design);

        //Settings file
        Settings_Ubend_Parametric settings; //Settings_TUB_Parametric settings;//
        string mgoptFolder = settings.mgoptFolder;
        string mgoptCase =settings.mgoptCase;
        string mgoptJsonFile=settings.mgoptJsonFile;
        string ExternalLibraryFolder = settings.ExternalLibraryFolder;
        string RestartSettingsFile = settings.RestartSettingsFile;

        mgopt=MgoptSolver(mgoptFolder,mgoptCase,mgoptJsonFile,ExternalLibraryFolder);
        mgopt.RestartSettingsFile = RestartSettingsFile;

    }



    double CFDCostFunction(bool restart = false) {
        if (!restart) {
            mgopt.RunSolverByMode(1);//run primal
        }
        else{
            mgopt.RunSolverByMode(2,mgopt.RestartSettingsFile);
            auto residualXMomentum =  mgopt.GetXMomentumResidual();
            int indexer = 0;
            //was -8
            while (residualXMomentum > -7 && indexer <= 10){ //TUB: residualXMomentum > -2 && indexer <= 20 //UBEND: residualXMomentum > -7 or -8 && indexer <= 10
                mgopt.RunSolverByMode(2,mgopt.RestartSettingsFile);
                residualXMomentum = mgopt.GetXMomentumResidual();
                indexer++;
            }
            if (indexer>5){
                cout << "WARNING = more than 10 restarts of mgopt PRIMAL!!!!!" << endl;
            }
        }

        cout << "Get costFuncVal" << endl;
        return mgopt.GetCostFunctionValue();
    }

    vector<vector<double>> CFDSensitivity(bool restart = false){
        if (!restart) {
            mgopt.RunSolverByMode(3);//run adjoint
        }
        else {

            mgopt.RunSolverByMode(4,mgopt.RestartSettingsFile);

            auto residualXAdjiont = mgopt.GetXAdjointMomentumResidual();
            int indexer = 0;
            //was -6
            while (residualXAdjiont > -5 && indexer <= 7){ //TUB: residualXAdjiont > -5 && indexer <= 20   UBEND: residualXAdjiont > -5 && indexer <= 7
                mgopt.RunSolverByMode(4,mgopt.RestartSettingsFile);
                residualXAdjiont = mgopt.GetXAdjointMomentumResidual();
                indexer++;
            }
            if (indexer>4){
                cout << "WARNING = more than 5 restarts of mgopt Adjoint!!!!!" << endl;
            }

        }
        mgopt.RunSolverByMode(6);//calculate flow sensitivities (Matrix 3*Meshpointisze)

       return mgopt.GetSensitivity();
    }

    vector<double> CFDCADGradient(bool restart = false) {

        auto cfdSens = CFDSensitivity(restart);

        vector<double> gradient(occt.nParams);
        for (int i=0;i<occt.nParams;i++)
            gradient[i] = 0;
/*
//        vector<int> specialNodesIndexes(2*20);
//        for (int i=0;i<20;i++){
//            specialNodesIndexes[i] = 1667 + i*1700;
//            specialNodesIndexes[20+i]=1666+i*1700;
////            specialNodesIndexes[40+i]=1665+i*1700;
////            specialNodesIndexes[60+i]=1668+i*1700;
//        }
//
//        for (int i=0;i<specialNodesIndexes.size();i++){
//            cout << "    i ="<< i << specialNodesIndexes[i] << cfdSens[0][specialNodesIndexes[i]] <<","<<cfdSens[1][specialNodesIndexes[i]] <<","<<cfdSens[2][specialNodesIndexes[i]]<< endl;
//            for (int j=0;j<3;j++){
//                cfdSens[j][specialNodesIndexes[i]-1] = 0;
//            }
//        }

       // vector<int> activeIndexes = {0,5,10};
//
//
//        for (int i = 0; i < activeIndexes.size(); i++) {
//
//            cout << "CAD Sens" << i << " calculation out of " << activeIndexes.size() <<  endl;
//            auto cadSens = occt.GetDesignSensitivity(activeIndexes[i]);
//
////            string name = "CAD_Sensitivity_"+to_string(activeIndexes[i]);
////            mgopt.PopulateTableByName(cadSens,name);
//
//            gradient[activeIndexes[i]] = scalarMatrixMultiplication(cfdSens, cadSens);
//        }
 */
#if !REVERSE_MODE
        vector<vector<Standard_Real>> fullCadSens = occt.GetCompleteDesignSensitivityUsingVectorMode();
        for (int i = 0; i < gradient.size(); i++) {
            if (i%5==0)
            cout << "CAD Sens" << i << " calculation out of " << gradient.size() <<  endl;
            vector<vector<double>> cadSens(3,vector<double>(fullCadSens[0].size()));
            for(int j = 0; j < 3; j++)
            {
              for(int k = 0; k < fullCadSens[0].size(); k++)
              {
                cadSens[j][k] = fullCadSens[j][k].getADValue(i);
              }
            }

//            string name = "CAD_Sensitivity_"+to_string(i);
//            mgopt.PopulateTableByName(cadSens,name);

            gradient[i] = scalarMatrixMultiplication(cfdSens, cadSens);
        }
#endif
        return gradient;

    }

    vector<double> CFDCADGradientUsingADTraceMode(bool restart = false) {

        vector<double> gradient(occt.nParams);
        for (int i=0;i<occt.nParams;i++)
            gradient[i] = 0;

        if (!restart) {
            mgopt.RunSolverByMode(3);//run adjoint
        }
        else {

            mgopt.RunSolverByMode(4,mgopt.RestartSettingsFile);

            auto residualXAdjiont = mgopt.GetXAdjointMomentumResidual();
            int indexer = 0;
            //UBEND residualXAdjiont > -5 && indexer <= 7
            while (residualXAdjiont > -4 && indexer <= 20){ //TUB: residualXAdjiont > -2 && indexer <= 7   UBEND: residualXAdjiont > -5 && indexer <= 7
                mgopt.RunSolverByMode(4,mgopt.RestartSettingsFile);
                residualXAdjiont = mgopt.GetXAdjointMomentumResidual();
                indexer++;
            }
            if (indexer>4){
                cout << "WARNING = more than 5 restarts of mgopt Adjoint!!!!!" << endl;
            }

        }
//        mgopt.SettingsFile = "ubend-Rejish-spring.json";
        mgopt.RunSolverByMode(6);//calculate flow sensitivities (Matrix 3*Meshpointisze)
//        mgopt.SettingsFile = "ubend-Rejish.json";

        auto cfdSens = mgopt.GetSensitivity();

#if REVERSE_MODE
        //GENERATE TRACE
        OSD_Timer aTimer;
        aTimer.Start();
        occt.GenerateADTrace();
        aTimer.Stop();
        cout << "Time for generating trace: " << aTimer.ElapsedTime() << endl;

        aTimer.Reset();

        aTimer.Start();
        //execute vec_jac
        gradient = occt.GetGradientByEvaluatingADTraceWithCFDSensitivity(cfdSens);
        aTimer.Stop();
        cout << "Time for REVERSE run (total gradient calculation): " << aTimer.ElapsedTime() << endl;
#endif
        return gradient;

    }

    vector<double> CFDCADGradient(vector<int> activeIndexes, bool restart=false) {

        //!!!!!! WARNING - ACTIVE INDEXES VERSION
        vector<double> gradient(occt.nParams);
        for (int i=0;i<occt.nParams;i++)
            gradient[i] = 0;

        if (!restart) {
            mgopt.RunSolverByMode(3);//run adjoint
        }
        else {
            mgopt.RunSolverByMode(4,mgopt.RestartSettingsFile);
        }
        mgopt.RunSolverByMode(6);//calculate flow sensitivities (Matrix 3*Meshpointisze)

        auto cfdSens = mgopt.GetSensitivity();

        // vector<int> activeIndexes = {0,5,10};
//
//
#if !REVERSE_MODE
        for (int i = 0; i < activeIndexes.size(); i++) {

            cout << "CAD Sens" << i << " calculation out of " << activeIndexes.size() << "CAD Index = "<<activeIndexes[i]<<  endl;
            auto cadSens = occt.GetDesignSensitivity(activeIndexes[i]);

//            string name = "CAD_Sensitivity_"+to_string(activeIndexes[i]);
//            mgopt.PopulateTableByName(cadSens,name);

            gradient[activeIndexes[i]] = scalarMatrixMultiplication(cfdSens, cadSens);
        }
#endif

        return gradient;

    }



    double DesignStep(vector<Standard_Real> newparams) {
        //shape builded on new parameters
        occt.designParameters = newparams;

        occt.BuildShape();
        occt.BuildConnectivity();

        cout << "Calculating Pertubed Mesh" << endl;
        occt.CalculatePertubedMesh();
        auto deformations = occt.Find3dMeshDisplacement();
        mgopt.PopulateDisplacementsTable(deformations);

//        int meshIndex = 13294;
//        cout <<"13294 mesh point X Y Z = " << occt.meshpointsCartesian[meshIndex].Coord(1).getValue() <<";  "   << occt.meshpointsCartesian[meshIndex].Coord(2).getValue() <<";  " << occt.meshpointsCartesian[meshIndex].Coord(3).getValue() <<";  "<< endl;
//        cout <<"13294  defromed mesh point X Y Z = " << occt.meshpointsCartesianPerturbed[meshIndex].Coord(1).getValue() <<";  "   << occt.meshpointsCartesianPerturbed[meshIndex].Coord(2).getValue() <<";  " << occt.meshpointsCartesianPerturbed[meshIndex].Coord(3).getValue() <<";  "<< endl;
//
//       cout << "Displacement" << deformations[0][meshIndex] <<";  " << deformations[1][meshIndex] <<";   " << deformations[2][meshIndex] <<";  " << endl;
       mgopt.RunSolverByMode(7);
////
////        //Dont forget to update your mesh!!!!
        occt.UpdateMeshFromPerturbed();
     //   cout <<"13294 mesh point X Y Z = " << occt.meshpointsCartesian[meshIndex].Coord(1).getValue() <<";  "   << occt.meshpointsCartesian[meshIndex].Coord(2).getValue() <<";  " << occt.meshpointsCartesian[meshIndex].Coord(3).getValue() <<";  "<< endl;
//

        return maxDeformation(deformations);
    }

    double DesignStepIterations(vector<Standard_Real> newparams, int iterationsSteps, string meshPerturbationsSettings) {
        //shape builded on new parameters
        vector<Standard_Real> updateParams = vector<Standard_Real>(newparams.size());
        for (int i = 0; i < newparams.size() ; ++i) {
            updateParams[i] = (newparams[i]-occt.designParameters[i])/iterationsSteps;
        }
        string generalSettings = mgopt.SettingsFile;

        mgopt.SettingsFile = meshPerturbationsSettings;

        for (int i = 0; i < newparams.size(); ++i) {
            newparams[i] = occt.designParameters[i];
        }

        vector<vector<double>> deformations;
        for (int i=0;i<iterationsSteps;i++) {

            for (int j=0;j<newparams.size();j++)
                newparams[j]+=updateParams[j];

            occt.designParameters = newparams;

            occt.BuildShape();
            occt.BuildConnectivity();

            cout << "Calculating Pertubed Mesh" << endl;
            occt.CalculatePertubedMesh();

            deformations = occt.Find3dMeshDisplacement();
            mgopt.PopulateDisplacementsTable(deformations);

            mgopt.RunSolverByMode(7);
            mgopt.RunSolverByMode(1);
            mgopt.RunSolverByMode(3);
            mgopt.RunSolverByMode(6);

           // mgopt.BackupSolution("/optimisation/CFD","iteration_Forward_iterationStep_"+std::to_string(i));
            //Dont forget to update your mesh!!!!
            occt.UpdateMeshFromPerturbed();
        }
        mgopt.SettingsFile = generalSettings;

        return maxDeformation(deformations);
    }

    double DesignStepMeshToCAD(){
        cout << "Calculating Pertubed Mesh" << endl;
        occt.CalculatePertubedMesh();
        auto deformations = occt.Find3dMeshDisplacement();
        mgopt.PopulateDisplacementsTable(deformations);

        mgopt.RunSolverByMode(7);

        //Dont forget to update your mesh!!!!
        occt.UpdateMeshFromPerturbed();

        return maxDeformation(deformations);
    }

    double DesignStepMeshToCAD(string settingDeformation){
        cout << "Calculating Pertubed Mesh" << endl;
        occt.CalculatePertubedMesh();
        auto deformations = occt.Find3dMeshDisplacement();
        mgopt.PopulateDisplacementsTable(deformations);

        auto oldSettings = mgopt.SettingsFile;
        mgopt.SettingsFile = settingDeformation;
        mgopt.RunSolverByMode(7);

        mgopt.SettingsFile = oldSettings;
        //Dont forget to update your mesh!!!!
        occt.UpdateMeshFromPerturbed();

        return maxDeformation(deformations);
    }



    //Optimisation Methods
    int RunCADCFDOptimisationConstantStep() {

        int cfdBackUpIteration = 3;

        cout << "Running CAD CFD Optimisation" << endl;
        vector<Standard_Real> x = occt.designParameters;

        //use for all params
        double scalingStep = 0.0005/10;
       // double scalingStep = 0.05/10;

        //Close to 10% of smallest cell
        double maxAllowedDeformation = 0.00125;

        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshes(meshTable, designTable);
        DesignStepMeshToCAD();

        auto obj = CFDCostFunction();
        auto g = CFDCADGradient();

        int k = 0;           //k = # iterations

        OptimisationState optimisationState(mgopt.CasePath);
        optimisationState.iteration=k;
        optimisationState.CADSurfaces=occt.facemap.Extent();
        optimisationState.CostFunction=obj;
        optimisationState.GradNorm=norm(g);
        optimisationState.params=x;
        optimisationState.primalConvergence=mgopt.GetPrimalCovergence();
        optimisationState.adjointConvergence=mgopt.GetAdjointConvergence();
        optimisationState.Gradient=g;
        optimisationState.EffectiveParametersIndexes = maxNElementsAbsVector(5,g);

        optimisationState.WriteState();
        optimisationState.WriteCostData(true);
        optimisationState.WriteParamsData(true);

        while ((norm(g) > 1e-3) && (k <=300)) {
//            if (k==12)
//                scalingStep=scalingStep/10;
            if (k==150)
                scalingStep=scalingStep/2;

            cout << "=================================================iteration " << to_string(k+1) <<"===================" <<endl;
            cout << "=====================================================================================================" <<endl;
            auto d = scaleVector(-1/maxabs(g), g);             // steepest     descent direction
           // auto d =  scaleVector(1,g);
            double a = scalingStep;          //scale with norm

            x = vectorSum(x, scaleVector(a, d));      // x = x+ad

            cout << "Our new desing" << endl;
            for (int i = 0; i < x.size(); i++) {
                cout << x[i] << endl;
            }

            double maxDeform = DesignStep(x);

            string iterationStepFile = mgopt.CasePath +"/optimisation/CAD/currentDesign_" + std::to_string(k) + ".stp";
            occt.StepWriter(iterationStepFile.c_str());


            obj = CFDCostFunction();
            g = CFDCADGradient();

            optimisationState.iteration=k+1;
            optimisationState.CADSurfaces=occt.facemap.Extent();
            optimisationState.CostFunction=obj;
            optimisationState.GradNorm=norm(g);
            optimisationState.params=x;
            optimisationState.primalConvergence=mgopt.GetPrimalCovergence();
            optimisationState.adjointConvergence=mgopt.GetAdjointConvergence();
            optimisationState.Gradient=g;
            optimisationState.EffectiveParametersIndexes = maxNElementsAbsVector(5,g);
            optimisationState.MaxMeshPerturbation =maxDeform;

            optimisationState.WriteState();
            optimisationState.WriteCostData();
            optimisationState.WriteParamsData();

            if (k%cfdBackUpIteration==0)
                mgopt.BackupSolution("/optimisation/CFD","iteration_"+to_string(k+1));

            if (optimisationState.GradNorm > 1e+10){
                cout << "Norm too big, exiting ..." << endl;
                break;
            }


            k = k + 1;

        }

        cout << "Exiting the cycle, norm = " << optimisationState.GradNorm << endl << "iterations done:" << optimisationState.iteration << endl;

    }



    int RunCADCFDOptimisationWithLineSearch(bool projectMesh = true, bool params=false) {
        bool restart = true;


        //Armijo stepsize rule parameters
        double sigma = 0.001;
        double beta = .5;
        int cfdBackUpIteration =1;

        //double maxStepSize = 0.0005/2;
        double maxStepSize = 0.0005*4;

        cout << "Running CAD CFD Optimisation Line Search" << endl;
        vector<Standard_Real> x = occt.designParameters;


        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();



        if (projectMesh) {
            occt.SetMeshes(meshTable, designTable);
        }
        else
        {
          Settings_Ubend_Parametric settings_ubend;
          string projectionFile = settings_ubend.projectionFile;
          occt.SetMeshesNoProjection(meshTable, designTable);
          ReadProjectionData(projectionFile);
          cout << "Projection data read successfully" << endl;
        }
#if REVERSE_MODE
        int noOfOutputs = occt.CalculateNumberOfOutputs();
        cout << "Number of outputs = " << noOfOutputs << endl;
#endif

        if(!params) {
            DesignStepMeshToCAD();
        }
        else{
            MoveToNewParams();
        }



        auto obj = CFDCostFunction(restart);
#if REVERSE_MODE
        auto g = CFDCADGradientUsingADTraceMode(restart);
#else
        auto g = CFDCADGradient(restart);
#endif

        int k = 0;           //k = # iterations
  //
        OptimisationState optimisationState(mgopt.CasePath);
        optimisationState.iteration=k;
        optimisationState.CADSurfaces=occt.facemap.Extent();
        optimisationState.CostFunction=obj;
        optimisationState.GradNorm=norm(g);
        optimisationState.params=x;
        optimisationState.primalConvergence=mgopt.GetPrimalCovergence();
        optimisationState.adjointConvergence=mgopt.GetAdjointConvergence();
        optimisationState.Gradient=g;
        optimisationState.EffectiveParametersIndexes = maxNElementsAbsVector(5,g);
        optimisationState.ArmijoStepsPerformed = 0;
        optimisationState.StepSize = 0;

        string iterationStepFile = mgopt.CasePath +"/optimisation/CAD/initialDesign_00.stp";
        occt.StepWriter(iterationStepFile.c_str());
        mgopt.BackupSolution("/optimisation/CFD","iteration_00_MeshOnCAD");

        optimisationState.WriteState();
        optimisationState.WriteCostData(true);
        optimisationState.WriteParamsData(true);

        k++;

        int nf = 1;        // nf = # function eval.
        int armijosteps = 0;



        while (norm(g) > 1e-3 && k <= 100) {
            auto d = scaleVector(-1, g);
            // steepest     descent direction
            double a = 1;
            double maxnormg = maxabs(g);
            if (maxnormg>1){
                a = maxStepSize/maxnormg;
            }

            auto x_new = (vectorSum(x, scaleVector(a, d)));

	   // verify minimum limits are respected

	  // =============================================== BFGS LIMITS IMPOSITION AND VERIFICATION LOOP =========================================================================


  Standard_Real controlPoints2_lb = 2.; 
  Standard_Real controlPoints2_ub = 3.; 

  Standard_Real leadingEdge_lb = 1.; 
  Standard_Real leadingEdge_ub = 1.5; 
  
  Standard_Real controlPoints3_lb = 3.;
  Standard_Real controlPoints3_ub = 5.86;

  Standard_Real controlPoints4_lb = 4.3; 
  Standard_Real controlPoints4_ub = 7.7; 

  Standard_Real controlPoints5_lb = 4.; 
  Standard_Real controlPoints5_ub = 7.58; 

  Standard_Real controlPoints6_lb = 3.5; 
  Standard_Real controlPoints6_ub = 6.3; 

  Standard_Real controlPoints7_lb = 3.; 
  Standard_Real controlPoints7_ub = 4.2; 

  Standard_Real controlPoints8_lb = 2.3; 
  Standard_Real controlPoints8_ub = 2.67; 

  Standard_Real controlPoints9_lb = 1.114; 
  Standard_Real controlPoints9_ub = 1.2; 

  Standard_Real trailingEdge_lb = 1.;
  Standard_Real trailingEdge_ub = 1.1111;

  Standard_Real camberlineCP1x_lb = -0.0001;
  Standard_Real camberlineCP1x_ub = 0.6;

  Standard_Real camberlineCP1y_lb = -0.0001;  
  Standard_Real camberlineCP1y_ub = 0.3;  

  Standard_Real camberlineCP2x_lb = 1.;
  Standard_Real camberlineCP2x_ub = 4.5772;

  Standard_Real camberlineCP2y_lb = -7.9068;
  Standard_Real camberlineCP2y_ub = -3.;

  Standard_Real camberlineCP3x_lb = 7.52290;
  Standard_Real camberlineCP3x_ub = 20.52299;

  Standard_Real camberlineCP3y_lb = -20.4609;
  Standard_Real camberlineCP3y_ub = -10.4608;

  Standard_Real camberlineCP4x_lb = 37.5224;
  Standard_Real camberlineCP4x_ub = 57.5226;

  Standard_Real camberlineCP4y_lb = -37.1068;
  Standard_Real camberlineCP4y_ub = -17.1066;

  Standard_Real camberlineCP5x_lb = 85.;
  Standard_Real camberlineCP5x_ub = 105.;

  Standard_Real camberlineCP5y_lb = -50.;
  Standard_Real camberlineCP5y_ub = -28.9;

  Standard_Real camberlineCP6x_lb = 150.;
  Standard_Real camberlineCP6x_ub = 180.;

  Standard_Real camberlineCP6y_lb = -50.;
  Standard_Real camberlineCP6y_ub = -24.;

  Standard_Real camberlineCP7y_lb = -45.;
  Standard_Real camberlineCP7y_ub = -20.;



  std::vector<Standard_Real> lb_sd(184); 
  std::vector<Standard_Real> ub_sd(184);

//parameter[0]
  lb_sd[0] = controlPoints2_lb;
  ub_sd[0] = controlPoints2_ub;
  //parameter[1]
  lb_sd[1] = controlPoints2_lb;
  ub_sd[1] = controlPoints2_ub;
  //parameter[2]
  lb_sd[2] = controlPoints2_lb;
  ub_sd[2] = controlPoints2_ub;
  //parameter[3]
  lb_sd[3] = controlPoints2_lb;
  ub_sd[3] = controlPoints2_ub;
//parameter[4]
  lb_sd[4] = controlPoints2_lb;
  ub_sd[4] = controlPoints2_ub;
  //parameter[5]
  lb_sd[5] = controlPoints2_lb;
  ub_sd[5] = controlPoints2_ub;
  //parameter[6]
  lb_sd[6] = controlPoints2_lb;
  ub_sd[6] = controlPoints2_ub;
  //parameter[7]
  lb_sd[7] = controlPoints2_lb;
  ub_sd[7] = controlPoints2_ub;
  
  //parameter[8]
  lb_sd[8] = leadingEdge_lb;
  ub_sd[8] = leadingEdge_ub;
  //parameter[9]
  lb_sd[9] = leadingEdge_lb;
  ub_sd[9] = leadingEdge_ub;
  //parameter[10]
  lb_sd[10] = leadingEdge_lb;
  ub_sd[10] = leadingEdge_ub;
  //parameter[11]
  lb_sd[11] = leadingEdge_lb;
  ub_sd[11] = leadingEdge_ub;
  //parameter[12]
  lb_sd[12] = leadingEdge_lb;
  ub_sd[12] = leadingEdge_ub;
  //parameter[13]
  lb_sd[13] = leadingEdge_lb;
  ub_sd[13] = leadingEdge_ub;
  //parameter[14]
  lb_sd[14] = leadingEdge_lb;
  ub_sd[14] = leadingEdge_ub;
  //parameter[15]
  lb_sd[15] = leadingEdge_lb;
  ub_sd[15] = leadingEdge_ub;
  
  //parameter[16]
  lb_sd[16] = controlPoints3_lb;
  ub_sd[16] = controlPoints3_ub;
  //parameter[17]
  lb_sd[17] = controlPoints3_lb;
  ub_sd[17] = controlPoints3_ub;
  //parameter[18]
  lb_sd[18] = controlPoints3_lb;
  ub_sd[18] = controlPoints3_ub;
  //parameter[19]
  lb_sd[19] = controlPoints3_lb;
  ub_sd[19] = controlPoints3_ub;
  //parameter[20]
  lb_sd[20] = controlPoints3_lb;
  ub_sd[20] = controlPoints3_ub;
  //parameter[21]
  lb_sd[21] = controlPoints3_lb;
  ub_sd[21] = controlPoints3_ub;
  //parameter[22]
  lb_sd[22] = controlPoints3_lb;
  ub_sd[22] = controlPoints3_ub;
  //parameter[23]
  lb_sd[23] = controlPoints3_lb;
  ub_sd[23] = controlPoints3_ub;
  
  //parameter[24]
  lb_sd[24] = controlPoints4_lb;
  ub_sd[24] = controlPoints4_ub;
  //parameter[25]
  lb_sd[25] = controlPoints4_lb;
  ub_sd[25] = controlPoints4_ub;
  //parameter[26]
  lb_sd[26] = controlPoints4_lb;
  ub_sd[26] = controlPoints4_ub;
  //parameter[27]
  lb_sd[27] = controlPoints4_lb;
  ub_sd[27] = controlPoints4_ub;
  //parameter[28]
  lb_sd[28] = controlPoints4_lb;
  ub_sd[28] = controlPoints4_ub;
  //parameter[29]
  lb_sd[29] = controlPoints4_lb;
  ub_sd[29] = controlPoints4_ub;
  //parameter[30]
  lb_sd[30] = controlPoints4_lb;
  ub_sd[30] = controlPoints4_ub;
  //parameter[31]
  lb_sd[31] = controlPoints4_lb;
  ub_sd[31] = controlPoints4_ub;
  
  
  //parameter[32]
  lb_sd[32] = controlPoints5_lb;
  ub_sd[32] = controlPoints5_ub;
  //parameter[33]
  lb_sd[33] = controlPoints5_lb;
  ub_sd[33] = controlPoints5_ub;
 //parameter[34]
  lb_sd[34] = controlPoints5_lb;
  ub_sd[34] = controlPoints5_ub;
  //parameter[35]
  lb_sd[35] = controlPoints5_lb;
  ub_sd[35] = controlPoints5_ub;
  //parameter[36]
  lb_sd[36] = controlPoints5_lb;
  ub_sd[36] = controlPoints5_ub;
  //parameter[37]
  lb_sd[37] = controlPoints5_lb;
  ub_sd[37] = controlPoints5_ub;
 //parameter[38]
  lb_sd[38] = controlPoints5_lb;
  ub_sd[38] = controlPoints5_ub;
  //parameter[39]
  lb_sd[39] = controlPoints5_lb;
  ub_sd[39] = controlPoints5_ub;
  
 //parameter[40]
  lb_sd[40] = controlPoints6_lb;
  ub_sd[40] = controlPoints6_ub;
  //parameter[41]
  lb_sd[41] = controlPoints6_lb;
  ub_sd[41] = controlPoints6_ub;
 //parameter[42]
  lb_sd[42] = controlPoints6_lb;
  ub_sd[42] = controlPoints6_ub;
  //parameter[43]
  lb_sd[43] = controlPoints6_lb;
  ub_sd[43] = controlPoints6_ub;
 //parameter[44]
  lb_sd[44] = controlPoints6_lb;
  ub_sd[44] = controlPoints6_ub;
  //parameter[45]
  lb_sd[45] = controlPoints6_lb;
  ub_sd[45] = controlPoints6_ub;
 //parameter[46]
  lb_sd[46] = controlPoints6_lb;
  ub_sd[46] = controlPoints6_ub;
  //parameter[47]
  lb_sd[47] = controlPoints6_lb;
  ub_sd[47] = controlPoints6_ub;
  
    
  //parameter[48]
  lb_sd[48] = controlPoints7_lb;
  ub_sd[48] = controlPoints7_ub;
  //parameter[49]
  lb_sd[49] = controlPoints7_lb;
  ub_sd[49] = controlPoints7_ub;
 //parameter[50]
  lb_sd[50] = controlPoints7_lb;
  ub_sd[50] = controlPoints7_ub;
  //parameter[51]
  lb_sd[51] = controlPoints7_lb;
  ub_sd[51] = controlPoints7_ub;
  //parameter[52]
  lb_sd[52] = controlPoints7_lb;
  ub_sd[52] = controlPoints7_ub;
  //parameter[53]
  lb_sd[53] = controlPoints7_lb;
  ub_sd[53] = controlPoints7_ub;
 //parameter[54]
  lb_sd[54] = controlPoints7_lb;
  ub_sd[54] = controlPoints7_ub;
  //parameter[55]
  lb_sd[55] = controlPoints7_lb;
  ub_sd[55] = controlPoints7_ub;
  
 //parameter[56]
  lb_sd[56] = controlPoints8_lb;
  ub_sd[56] = controlPoints8_ub;
  //parameter[57]
  lb_sd[57] = controlPoints8_lb;
  ub_sd[57] = controlPoints8_ub;
 //parameter[58]
  lb_sd[58] = controlPoints8_lb;
  ub_sd[58] = controlPoints8_ub;
  //parameter[59]
  lb_sd[59] = controlPoints8_lb;
  ub_sd[59] = controlPoints8_ub;
 //parameter[60]
  lb_sd[60] = controlPoints8_lb;
  ub_sd[60] = controlPoints8_ub;
  //parameter[61]
  lb_sd[61] = controlPoints8_lb;
  ub_sd[61] = controlPoints8_ub;
 //parameter[62]
  lb_sd[62] = controlPoints8_lb;
  ub_sd[62] = controlPoints8_ub;
  //parameter[63]
  lb_sd[63] = controlPoints8_lb;
  ub_sd[63] = controlPoints8_ub;

  //parameter[64]
  lb_sd[64] = controlPoints9_lb;
  ub_sd[64] = controlPoints9_ub;
  //parameter[65]
  lb_sd[65] = controlPoints9_lb;
  ub_sd[65] = controlPoints9_ub;
  //parameter[66]
  lb_sd[66] = controlPoints9_lb;
  ub_sd[66] = controlPoints9_ub;
  //parameter[67]
  lb_sd[67] = controlPoints9_lb;
  ub_sd[67] = controlPoints9_ub;
  //parameter[68]
  lb_sd[68] = controlPoints9_lb;
  ub_sd[68] = controlPoints9_ub;
  //parameter[69]
  lb_sd[69] = controlPoints9_lb;
  ub_sd[69] = controlPoints9_ub;
  //parameter[70]
  lb_sd[70] = controlPoints9_lb;
  ub_sd[70] = controlPoints9_ub;
  //parameter[71]
  lb_sd[71] = controlPoints9_lb;
  ub_sd[71] = controlPoints9_ub;
  
  //parameter[72]
  lb_sd[72] = trailingEdge_lb;
  ub_sd[72] = trailingEdge_ub;
  //parameter[73]
  lb_sd[73] = trailingEdge_lb;
  ub_sd[73] = trailingEdge_ub;
  //parameter[74]
  lb_sd[74] = trailingEdge_lb;
  ub_sd[74] = trailingEdge_ub;
  //parameter[75]
  lb_sd[75] = trailingEdge_lb;
  ub_sd[75] = trailingEdge_ub;
  //parameter[76]
  lb_sd[76] = trailingEdge_lb;
  ub_sd[76] = trailingEdge_ub;
  //parameter[77]
  lb_sd[77] = trailingEdge_lb;
  ub_sd[77] = trailingEdge_ub;
  //parameter[78]
  lb_sd[78] = trailingEdge_lb;
  ub_sd[78] = trailingEdge_ub;
  //parameter[79]
  lb_sd[79] = trailingEdge_lb;
  ub_sd[79] = trailingEdge_ub;
  
  
  //parameter[80]
  lb_sd[80] = camberlineCP1x_lb;
  ub_sd[80] = camberlineCP1x_ub;
  //parameter[81]
  lb_sd[81] = camberlineCP1y_lb;
  ub_sd[81] = camberlineCP1y_ub;
    //parameter[82]
  lb_sd[82] = camberlineCP1x_lb;
  ub_sd[82] = camberlineCP1x_ub;
  //parameter[83]
  lb_sd[83] = camberlineCP1y_lb;
  ub_sd[83] = camberlineCP1y_ub;
    //parameter[84]
  lb_sd[84] = camberlineCP1x_lb;
  ub_sd[84] = camberlineCP1x_ub;
  //parameter[85]
  lb_sd[85] = camberlineCP1y_lb;
  ub_sd[85] = camberlineCP1y_ub;
    //parameter[86]
  lb_sd[86] = camberlineCP1x_lb;
  ub_sd[86] = camberlineCP1x_ub;
  //parameter[87]
  lb_sd[87] = camberlineCP1y_lb;
  ub_sd[87] = camberlineCP1y_ub;
    //parameter[88]
  lb_sd[88] = camberlineCP1x_lb;
  ub_sd[88] = camberlineCP1x_ub;
  //parameter[89]
  lb_sd[89] = camberlineCP1y_lb;
  ub_sd[89] = camberlineCP1y_ub;
    //parameter[90]
  lb_sd[90] = camberlineCP1x_lb;
  ub_sd[90] = camberlineCP1x_ub;
  //parameter[91]
  lb_sd[91] = camberlineCP1y_lb;
  ub_sd[91] = camberlineCP1y_ub;
    //parameter[92]
  lb_sd[92] = camberlineCP1x_lb;
  ub_sd[92] = camberlineCP1x_ub;
  //parameter[93]
  lb_sd[93] = camberlineCP1y_lb;
  ub_sd[93] = camberlineCP1y_ub;
    //parameter[94]
  lb_sd[94] = camberlineCP1x_lb;
  ub_sd[94] = camberlineCP1x_ub;
  //parameter[95]
  lb_sd[95] = camberlineCP1y_lb;
  ub_sd[95] = camberlineCP1y_ub;
    
  //parameter[96]
  lb_sd[96] = camberlineCP2x_lb;
  ub_sd[96] = camberlineCP2x_ub;
//  parameter[97]
  lb_sd[97] = camberlineCP2y_lb;
  ub_sd[97] = camberlineCP2y_ub;
    //parameter[98]
  lb_sd[98] = camberlineCP2x_lb;
  ub_sd[98] = camberlineCP2x_ub;
//  parameter[99]
  lb_sd[99] = camberlineCP2y_lb;
  ub_sd[99] = camberlineCP2y_ub;
    //parameter[100]
  lb_sd[100] = camberlineCP2x_lb;
  ub_sd[100] = camberlineCP2x_ub;
//  parameter[101]
  lb_sd[101] = camberlineCP2y_lb;
  ub_sd[101] = camberlineCP2y_ub;
    //parameter[102]
  lb_sd[102] = camberlineCP2x_lb;
  ub_sd[102] = camberlineCP2x_ub;
//  parameter[103]
  lb_sd[103] = camberlineCP2y_lb;
  ub_sd[103] = camberlineCP2y_ub;
    //parameter[104]
  lb_sd[104] = camberlineCP2x_lb;
  ub_sd[104] = camberlineCP2x_ub;
//  parameter[105]
  lb_sd[105] = camberlineCP2y_lb;
  ub_sd[105] = camberlineCP2y_ub;
    //parameter[106]
  lb_sd[106] = camberlineCP2x_lb;
  ub_sd[106] = camberlineCP2x_ub;
//  parameter[107]
  lb_sd[107] = camberlineCP2y_lb;
  ub_sd[107] = camberlineCP2y_ub;
    //parameter[108]
  lb_sd[108] = camberlineCP2x_lb;
  ub_sd[108] = camberlineCP2x_ub;
//  parameter[109]
  lb_sd[109] = camberlineCP2y_lb;
  ub_sd[109] = camberlineCP2y_ub;
    //parameter[110]
  lb_sd[110] = camberlineCP2x_lb;
  ub_sd[110] = camberlineCP2x_ub;
//  parameter[111]
  lb_sd[111] = camberlineCP2y_lb;
  ub_sd[111] = camberlineCP2y_ub;
  
  
  //parameter[112]
  lb_sd[112] = camberlineCP3x_lb;
  ub_sd[112] = camberlineCP3x_ub;
  //parameter[113]
  lb_sd[113] = camberlineCP3y_lb;
  ub_sd[113] = camberlineCP3y_ub;
  //parameter[114]
  lb_sd[114] = camberlineCP3x_lb;
  ub_sd[114] = camberlineCP3x_ub;
  //parameter[115]
  lb_sd[115] = camberlineCP3y_lb;
  ub_sd[115] = camberlineCP3y_ub;
  //parameter[116]
  lb_sd[116] = camberlineCP3x_lb;
  ub_sd[116] = camberlineCP3x_ub;
  //parameter[117]
  lb_sd[117] = camberlineCP3y_lb;
  ub_sd[117] = camberlineCP3y_ub;
  //parameter[118]
  lb_sd[118] = camberlineCP3x_lb;
  ub_sd[118] = camberlineCP3x_ub;
  //parameter[119]
  lb_sd[119] = camberlineCP3y_lb;
  ub_sd[119] = camberlineCP3y_ub;
  //parameter[120]
  lb_sd[120] = camberlineCP3x_lb;
  ub_sd[120] = camberlineCP3x_ub;
  //parameter[121]
  lb_sd[121] = camberlineCP3y_lb;
  ub_sd[121] = camberlineCP3y_ub;
  //parameter[122]
  lb_sd[122] = camberlineCP3x_lb;
  ub_sd[122] = camberlineCP3x_ub;
  //parameter[123]
  lb_sd[123] = camberlineCP3y_lb;
  ub_sd[123] = camberlineCP3y_ub;
  //parameter[124]
  lb_sd[124] = camberlineCP3x_lb;
  ub_sd[124] = camberlineCP3x_ub;
  //parameter[125]
  lb_sd[125] = camberlineCP3y_lb;
  ub_sd[125] = camberlineCP3y_ub;
  //parameter[126]
  lb_sd[126] = camberlineCP3x_lb;
  ub_sd[126] = camberlineCP3x_ub;
  //parameter[127]
  lb_sd[127] = camberlineCP3y_lb;
  ub_sd[127] = camberlineCP3y_ub;
   
  //parameter[128]
  lb_sd[128] = camberlineCP4x_lb;
  ub_sd[128] = camberlineCP4x_ub;
  //parameter[129]
  lb_sd[129] = camberlineCP4y_lb;
  ub_sd[129] = camberlineCP4y_ub;
  //parameter[130]
  lb_sd[130] = camberlineCP4x_lb;
  ub_sd[130] = camberlineCP4x_ub;
  //parameter[131]
  lb_sd[131] = camberlineCP4y_lb;
  ub_sd[131] = camberlineCP4y_ub;
  //parameter[132]
  lb_sd[132] = camberlineCP4x_lb;
  ub_sd[132] = camberlineCP4x_ub;
  //parameter[133]
  lb_sd[133] = camberlineCP4y_lb;
  ub_sd[133] = camberlineCP4y_ub;
  //parameter[134]
  lb_sd[134] = camberlineCP4x_lb;
  ub_sd[134] = camberlineCP4x_ub;
  //parameter[135]
  lb_sd[135] = camberlineCP4y_lb;
  ub_sd[135] = camberlineCP4y_ub;
  //parameter[136]
  lb_sd[136] = camberlineCP4x_lb;
  ub_sd[136] = camberlineCP4x_ub;
  //parameter[137]
  lb_sd[137] = camberlineCP4y_lb;
  ub_sd[137] = camberlineCP4y_ub;
  //parameter[138]
  lb_sd[138] = camberlineCP4x_lb;
  ub_sd[138] = camberlineCP4x_ub;
  //parameter[139]
  lb_sd[139] = camberlineCP4y_lb;
  ub_sd[139] = camberlineCP4y_ub;
  //parameter[140]
  lb_sd[140] = camberlineCP4x_lb;
  ub_sd[140] = camberlineCP4x_ub;
  //parameter[141]
  lb_sd[141] = camberlineCP4y_lb;
  ub_sd[141] = camberlineCP4y_ub;
  //parameter[142]
  lb_sd[142] = camberlineCP4x_lb;
  ub_sd[142] = camberlineCP4x_ub;
  //parameter[143]
  lb_sd[143] = camberlineCP4y_lb;
  ub_sd[143] = camberlineCP4y_ub;
    
  
  //parameter[144]
  lb_sd[144] = camberlineCP5x_lb;
  ub_sd[144] = camberlineCP5x_ub;
  //parameter[145]
  lb_sd[145] = camberlineCP5y_lb;
  ub_sd[145] = camberlineCP5y_ub;
  //parameter[146]
  lb_sd[146] = camberlineCP5x_lb;
  ub_sd[146] = camberlineCP5x_ub;
  //parameter[147]
  lb_sd[147] = camberlineCP5y_lb;
  ub_sd[147] = camberlineCP5y_ub;
  //parameter[148]
  lb_sd[148] = camberlineCP5x_lb;
  ub_sd[148] = camberlineCP5x_ub;
  //parameter[149]
  lb_sd[149] = camberlineCP5y_lb;
  ub_sd[149] = camberlineCP5y_ub;
  //parameter[150]
  lb_sd[150] = camberlineCP5x_lb;
  ub_sd[150] = camberlineCP5x_ub;
  //parameter[151]
  lb_sd[151] = camberlineCP5y_lb;
  ub_sd[151] = camberlineCP5y_ub;
  //parameter[152]
  lb_sd[152] = camberlineCP5x_lb;
  ub_sd[152] = camberlineCP5x_ub;
  //parameter[153]
  lb_sd[153] = camberlineCP5y_lb;
  ub_sd[153] = camberlineCP5y_ub;
  //parameter[154]
  lb_sd[154] = camberlineCP5x_lb;
  ub_sd[154] = camberlineCP5x_ub;
  //parameter[155]
  lb_sd[155] = camberlineCP5y_lb;
  ub_sd[155] = camberlineCP5y_ub;
  //parameter[156]
  lb_sd[156] = camberlineCP5x_lb;
  ub_sd[156] = camberlineCP5x_ub;
  //parameter[157]
  lb_sd[157] = camberlineCP5y_lb;
  ub_sd[157] = camberlineCP5y_ub;
  //parameter[158]
  lb_sd[158] = camberlineCP5x_lb;
  ub_sd[158] = camberlineCP5x_ub;
  //parameter[159]
  lb_sd[159] = camberlineCP5y_lb;
  ub_sd[159] = camberlineCP5y_ub;
  
  //parameter[160]
  lb_sd[160] = camberlineCP6x_lb;
  ub_sd[160] = camberlineCP6x_ub;
  //parameter[161]
  lb_sd[161] = camberlineCP6y_lb;
  ub_sd[161] = camberlineCP6y_ub;
  //parameter[162]
  lb_sd[162] = camberlineCP6x_lb;
  ub_sd[162] = camberlineCP6x_ub;
//parameter[163]
  lb_sd[163] = camberlineCP6y_lb;
  ub_sd[163] = camberlineCP6y_ub;
  //parameter[164]
  lb_sd[164] = camberlineCP6x_lb;
  ub_sd[164] = camberlineCP6x_ub;
//parameter[165]
  lb_sd[165] = camberlineCP6y_lb;
  ub_sd[165] = camberlineCP6y_ub;
  //parameter[166]
  lb_sd[166] = camberlineCP6x_lb;
  ub_sd[166] = camberlineCP6x_ub;
//parameter[167]
  lb_sd[167] = camberlineCP6y_lb;
  ub_sd[167] = camberlineCP6y_ub;
  //parameter[168]
  lb_sd[168] = camberlineCP6x_lb;
  ub_sd[168] = camberlineCP6x_ub;
//parameter[169]
  lb_sd[169] = camberlineCP6y_lb;
  ub_sd[169] = camberlineCP6y_ub;
  //parameter[170]
  lb_sd[170] = camberlineCP6x_lb;
  ub_sd[170] = camberlineCP6x_ub;
//parameter[171]
  lb_sd[171] = camberlineCP6y_lb;
  ub_sd[171] = camberlineCP6y_ub;
  //parameter[172]
  lb_sd[172] = camberlineCP6x_lb;
  ub_sd[172] = camberlineCP6x_ub;
//parameter[173]
  lb_sd[173] = camberlineCP6y_lb;
  ub_sd[173] = camberlineCP6y_ub;
 //parameter[174]
  lb_sd[174] = camberlineCP6x_lb;
  ub_sd[174] = camberlineCP6x_ub;
  //parameter[175]
  lb_sd[175] = camberlineCP6y_lb;
  ub_sd[175] = camberlineCP6y_ub;
  
  
  //parameter[176]
  lb_sd[176] = camberlineCP7y_lb;
  ub_sd[176] = camberlineCP7y_ub;
  //parameter[177]
  lb_sd[177] = camberlineCP7y_lb;
  ub_sd[177] = camberlineCP7y_ub;
  //parameter[178]
  lb_sd[178] = camberlineCP7y_lb;
  ub_sd[178] = camberlineCP7y_ub;
  //parameter[179]
  lb_sd[179] = camberlineCP7y_lb;
  ub_sd[179] = camberlineCP7y_ub;
  //parameter[180]
  lb_sd[180] = camberlineCP7y_lb;
  ub_sd[180] = camberlineCP7y_ub;
  //parameter[181]
  lb_sd[181] = camberlineCP7y_lb;
  ub_sd[181] = camberlineCP7y_ub;
  //parameter[182]
  lb_sd[182] = camberlineCP7y_lb;
  ub_sd[182] = camberlineCP7y_ub;
  //parameter[183]
  lb_sd[183] = camberlineCP7y_lb;
  ub_sd[183] = camberlineCP7y_ub;


            for (int j=0; j<184; ++j)
	    { 
              if (x_new[j] < lb_sd[j]) x_new[j] = lb_sd[j];    
            } 

            double maxMeshDeform;
            maxMeshDeform = DesignStep(x_new);

            nf = nf + 1;
            auto newobj = CFDCostFunction(restart);
            armijosteps = 0;

//          while (((newobj - obj) / a) > (sigma * (vectorTransposeMultiplication(g, d)))) {
//                cout << "============================================" << endl;
//                cout << "newobj = " << newobj << " | obj=" << obj << " | a = " << a << " | vectorTransposeMultiplication=" <<
//                vectorTransposeMultiplication(g, d) << endl;
//                cout << "norm g = " << norm(g) << " | maxnorm g = " << maxabs(g) << " | norm d = " << norm(d) << " | maxnorm d = " << maxabs(d) << endl;
//                cout << "newobj-obj/a=" <<  (newobj - obj) / a << "| sigmaScalarMult=" << sigma * (vectorTransposeMultiplication(
//                        g, d)) << endl;
//                cout << "============================================" << endl;
//
//                armijosteps++;
//                a = a * beta;
//                x_new = vectorSum(x, scaleVector(a, d));
//                maxMeshDeform=DesignStep(x_new);
//                newobj = CFDCostFunction(restart);
//                nf = nf + 1;
//
//            }

            x = vectorSum(x, scaleVector(a, d));
            obj = newobj;

            iterationStepFile = mgopt.CasePath +"/optimisation/CAD/currentDesign_" + std::to_string(k) + ".stp";
            occt.StepWriter(iterationStepFile.c_str());

#if REVERSE_MODE
            auto g = CFDCADGradientUsingADTraceMode(restart);
#else
            auto g = CFDCADGradient(restart);
#endif

            OptimisationState optimisationState(mgopt.CasePath);
            optimisationState.iteration=k;
            optimisationState.CADSurfaces=occt.facemap.Extent();
            optimisationState.CostFunction=obj;
            optimisationState.GradNorm=norm(g);
            optimisationState.params=x;
            optimisationState.primalConvergence=mgopt.GetPrimalCovergence();
            optimisationState.adjointConvergence=mgopt.GetAdjointConvergence();
            optimisationState.Gradient=g;
            optimisationState.EffectiveParametersIndexes = maxNElementsAbsVector(5,g);
            optimisationState.ArmijoStepsPerformed = armijosteps;
            optimisationState.StepSize = a;
            optimisationState.MaxParamPerturbation = a*maxnormg;
            optimisationState.MaxMeshPerturbation = maxMeshDeform;

            optimisationState.WriteState();
            optimisationState.WriteCostData();
            optimisationState.WriteParamsData();

            mgopt.BackupSolution("/optimisation/CFD","iteration_"+to_string(k));

            if (optimisationState.GradNorm > 1e+20){
                cout << "Norm too big, exiting ..." << endl;
                break;
            }


            k = k + 1;



        }

    }


    void RunCADCFDOptimisationWithBFGSAndTraceMode(bool projectMesh = true, bool params=false){
      bool restart = true;

      auto designTable = mgopt.GetDesignTable();
      auto meshTable = mgopt.GetMeshTable();

      if (projectMesh) {
          occt.SetMeshes(meshTable, designTable);
      }
      else
      {
        Settings_Ubend_Parametric settings_ubend;
        string projectionFile = settings_ubend.projectionFile;
        occt.SetMeshesNoProjection(meshTable, designTable);
        ReadProjectionData(projectionFile);
        cout << "Projection data read successfully" << endl;
      }

      int noOfOutputs = occt.CalculateNumberOfOutputs();
      cout << "Number of outputs = " << noOfOutputs << endl;

      if(!params) {
          DesignStepMeshToCAD();
      }
      else{
          MoveToNewParams();
      }


      //set initial target function, required for bfgs
      double target = CFDCostFunction(restart);
      vector<double> g = CFDCADGradientUsingADTraceMode(restart);

      int iterationNumber = 0;           //k = # iterations
//
      OptimisationState optimisationState(mgopt.CasePath);
      optimisationState.iteration=iterationNumber;
      optimisationState.CADSurfaces=occt.facemap.Extent();
      optimisationState.CostFunction=target;
      optimisationState.GradNorm=norm(g);
      optimisationState.params=occt.designParameters;
      optimisationState.primalConvergence=mgopt.GetPrimalCovergence();
      optimisationState.adjointConvergence=mgopt.GetAdjointConvergence();
      optimisationState.Gradient=g;
      optimisationState.EffectiveParametersIndexes = maxNElementsAbsVector(5,g);

      string iterationStepFile = mgopt.CasePath +"/optimisation/CAD/initialDesign_00.stp";
      occt.StepWriter(iterationStepFile.c_str());
      mgopt.BackupSolution("/optimisation/CFD","iteration_00_MeshOnCAD");

      optimisationState.WriteState();
      optimisationState.WriteCostData(true);
      optimisationState.WriteParamsData(true);


      /**********************************************************************************************
       *******************************SETUP BFGS PARAMETERS******************************************
       **********************************************************************************************/
      double   *x;
      double   *gradient;
      double   *ub, *lb;
      double   grad_norm;  /* norm of gradient       */

      int n, nmax, m, mmax;
      double  *wa, *dsave;
      int *iwa, *isave;
      double factr, pgtol;
      char *task, *csave;
      bool *lsave;
      int info = 1;
      int *nbd;
      int i;

      // number of parameters
      n = occt.nParams;

      // maximal number of independents and directions
      nmax=n;
      mmax=10;

      // current number of directions used for BFGS update
      m=mmax;

      // state
      x = new double[n];

      // gradient
      gradient = new double[n];

      // lower and upper bounds
      lb = new double[nmax];
      ub = new double[nmax];

      // parameters of BFGS
      nbd = new int[nmax];
      factr = 1.0e-60;   // tolerance in the termination test for the algorithm.
      pgtol =  1.0e-45; // The iteration will stop when max{|proj g_i | i = 1, ..., n} <= pgtol where pg_i is the ith component of the projected gradient.
      wa = new double[(2*mmax*nmax+4*nmax+12*mmax*mmax+12*mmax)+1];// array used as workspace.
      iwa = new int[(3*nmax)+1];// used as workspace.
      task = new char[61];// tells what to do
      // working arrays
      csave = new char[61];  lsave = new bool[5];  isave = new int[45];  dsave = new double[30];

      // init all values
      for (i=0;i<n;i++)  gradient[i] = 0;
      for (i=0;i<60;i++) task[i] = char(0);
      for (i=0;i<60;i++) csave[i] = char(0);
      for (i=0;i<(2*mmax*nmax+4*nmax+12*mmax*mmax+12*mmax);i++)  wa[i] = 0.0;
      for (i=0;i<3*nmax;i++) iwa[i] = 0;
      for (i=0;i<4;i++)  lsave[i] = 0;
      for (i=0;i<44;i++) isave[i] = 0;
      for (i=0;i<29;i++) dsave[i] = 0.0;
      task[0] = 'S'; task[1] = 'T'; task[2] = 'A'; task[3] = 'R'; task[4] = 'T';

      double sumOfDesignParametersValues = 0.;
      for(i=0;i<n;i++)
      {
        x[i] = occt.designParameters[i].getValue();;
        nbd[i] = 2;
//        sumOfDesignParametersValues += fabs(x[i]);
      }

//      double avgOfDesignValues = sumOfDesignParametersValues / n;
//      cout << "An average of all design parameters values is: " << sumOfDesignParametersValues << endl;
      //take 30%
//      avgOfDesignValues *= 0.35;
      //set the limits according to that
//      for(i=0;i<n;i++)
//      {
//        lb[i] = x[i] - avgOfDesignValues; cout << "lb[" << i << "]" << lb[i] << endl; cout << "x[i]: " << x[i] << endl;
//        ub[i] = x[i] + avgOfDesignValues; cout << "ub[" << i << "]" << ub[i] << endl << endl;
//      }

      //setting bounds manually
      /***********************************************************************
       **************************** SYMMETRIC TIGHT U-BEND LIMITS v2 Salvatore****************************
       ***********************************************************************/
      /*
      //parameter[0]
      lb[0] = -0.05;
      ub[0] = -0.037;
      //  parameter[1]
      lb[1] = 0.037;
      ub[1] = 0.05;
      //parameter[2]
      lb[2] = -0.05;
      ub[2] = -0.037;
      //  parameter[3]
      lb[3] = 0.037;
      ub[3] = 0.05;
      //parameter[4]
      lb[4] = -0.05;
      ub[4] = -0.037;
      //  parameter[5]    ;
      lb[5] = 0.037;
      ub[5] = 0.05;
      //parameter[6]      ;
      lb[6] = -0.05;
      ub[6] = -0.037;
      //parameter[7]      ;
      lb[7] = 0.037;
      ub[7] = 0.05;

      //parameter[8]      ;
      lb[8] = -0.023;
      ub[8] = -0.018;
      //parameter[9]      ;
      lb[9] = 0.037;
      ub[9] = 0.06;
      //parameter[10]     ;
      lb[10]  = -0.023;
      ub[10]  = -0.018;
      //parameter[11]     ;
      lb[11]  = 0.037;
      ub[11]  = 0.065;
      //parameter[12]     ;
      lb[12]  = -0.023;
      ub[12]  = -0.018;
      //parameter[13]     ;
      lb[13]  = 0.037;
      ub[13]  = 0.06;
      //parameter[14]     ;
      lb[14]  = -0.023;
      ub[14]  = -0.018;
      //parameter[15]     ;
      lb[15]  = 0.037;
      ub[15]  = 0.05;

      //parameter[16]     ;
      lb[16]  = 0.018;
      ub[16]  = 0.023;
      //parameter[17]     ;
      lb[17]  = 0.037;
      ub[17]  = 0.06;
      //parameter[18]     ;
      lb[18]  = 0.018;
      ub[18]  = 0.023;
      //parameter[19]     ;
      lb[19]  = 0.037;
      ub[19]  = 0.065;
      //parameter[20]     ;
      lb[20]  = 0.018;
      ub[20]  = 0.023;
      //parameter[21]     ;
      lb[21]  = 0.037;
      ub[21]  = 0.06;
      //parameter[22]     ;
      lb[22]  = 0.018;
      ub[22]  = 0.023;
      //parameter[23]     ;
      lb[23]  = 0.037;
      ub[23]  = 0.05;

      //parameter[24]     ;
      lb[24]  = 0.037;
      ub[24]  = 0.05;
      //  parameter[25]   ;
      lb[25]  = 0.037;
      ub[25]  = 0.05;
      //parameter[26]     ;
      lb[26]  = 0.037;
      ub[26]  = 0.05;
      //  parameter[27]   ;
      lb[27]  = 0.037;
      ub[27]  = 0.05;
      //parameter[28]     ;
      lb[28]  = 0.037;
      ub[28]  = 0.05;
      //  parameter[29]   ;
      lb[29]  = 0.037;
      ub[29]  = 0.05;
      //parameter[30]     ;
      lb[30]  = 0.037;
      ub[30]  = 0.05;
      //parameter[31]     ;
      lb[31]  = 0.037;
      ub[31]  = 0.05;

      //parameter[32]     ;
      lb[32]  = 0.037;
      ub[32]  = 0.06;
      //parameter[33]     ;
      lb[33]  = 0.018;
      ub[33]  = 0.02;
      //parameter[34]     ;
      lb[34]  = 0.037;
      ub[34]  = 0.06;
      //parameter[35]     ;
      lb[35]  = 0.018;
      ub[35]  = 0.02;
      //parameter[36]     ;
      lb[36]  = 0.037;
      ub[36]  = 0.06;
      //parameter[37]     ;
      lb[37]  = 0.018;
      ub[37]  = 0.02;
      //parameter[38]     ;
      lb[38]  = 0.037;
      ub[38]  = 0.06;
      //parameter[39]     ;
      lb[39]  = 0.018;
      ub[39]  = 0.02;

      //parameter[40]     ;
      lb[40]  = 0.037;
      ub[40]  = 0.05;
      //parameter[41]     ;
      lb[41]  = -0.02;
      ub[41]  = -0.018;
      //parameter[42]     ;
      lb[42]  = 0.037;
      ub[42]  = 0.05;
      //parameter[43]     ;
      lb[43]  = -0.02;
      ub[43]  = -0.018;
      //parameter[44]     ;
      lb[44]  = 0.037;
      ub[44]  = 0.05;
      //parameter[45]     ;
      lb[45]  = -0.02;
      ub[45]  = -0.018;
      //parameter[46]     ;
      lb[46]  = 0.037;
      ub[46]  = 0.05;
      //parameter[47]     ;
      lb[47]  = -0.02;
      ub[47]  = -0.018;

      //parameter[48]     ;
      lb[48]  = 0.03;
      ub[48]  = 0.05;
      //parameter[49]     ;
      lb[49]  = -0.04;
      ub[49]  = -0.037;
      //parameter[50]     ;
      lb[50]  = 0.03;
      ub[50]  = 0.05;
      //parameter[51]     ;
      lb[51]  = -0.04;
      ub[51]  = -0.037;
      //parameter[52]     ;
      lb[52]  = 0.03;
      ub[52]  = 0.05;
      //parameter[53]     ;
      lb[53]  = -0.04;
      ub[53]  = -0.037;
      //parameter[54]     ;
      lb[54]  = 0.03;
      ub[54]  = 0.05;
      //parameter[55]     ;
      lb[55]  = -0.04;
      ub[55]  = -0.037;

      //parameter[56]     ;
      lb[56]  = 0.018;
      ub[56]  = 0.023;
      //parameter[57]     ;
      lb[57]  = -0.06;
      ub[57]  = -0.037;
      //parameter[58]     ;
      lb[58]  = 0.018;
      ub[58]  = 0.023;
      //parameter[59]     ;
      lb[59]  = -0.065;
      ub[59]  = -0.037;
      //parameter[60]     ;
      lb[60]  = 0.018;
      ub[60]  = 0.023;
      //parameter[61]     ;
      lb[61]  = -0.06;
      ub[61]  = -0.037;
      //parameter[62]     ;
      lb[62]  = 0.018;
      ub[62]  = 0.023;
      //parameter[63]     ;
      lb[63]  = -0.05;
      ub[63]  = -0.037;

      //parameter[64]     ;
      lb[64]  = -0.023;
      ub[64]  = -0.018;
      //parameter[65]     ;
      lb[65]  = -0.06;
      ub[65]  = -0.037;
      //parameter[66]     ;
      lb[66]  = -0.023;
      ub[66]  = -0.018;
      //parameter[67]     ;
      lb[67]  = -0.065;
      ub[67]  = -0.037;
      //parameter[68]     ;
      lb[68]  = -0.023;
      ub[68]  = -0.018;
      //parameter[69]     ;
      lb[69]  = -0.06;
      ub[69]  = -0.037;
      //parameter[70]     ;
      lb[70]  = -0.023;
      ub[70]  = -0.018;
      //parameter[71]     ;
      lb[71]  = -0.05;
      ub[71]  = -0.037;

      //parameter[72]     ;
      lb[72]  = -0.05;
      ub[72]  = -0.03;
      //parameter[73]     ;
      lb[73]  = -0.04;
      ub[73]  = -0.037;
      //parameter[74]     ;
      lb[74]  = -0.05;
      ub[74]  = -0.03;
      //parameter[75]     ;
      lb[75]  = -0.04;
      ub[75]  = -0.037;
      //parameter[76]     ;
      lb[76]  = -0.05;
      ub[76]  = -0.03;
      //parameter[77]     ;
      lb[77]  = -0.04;
      ub[77]  = -0.037;
      //parameter[78]     ;
      lb[78]  = -0.05;
      ub[78]  = -0.03;
      //parameter[79]     ;
      lb[79]  = -0.04;
      ub[79]  = -0.037;

      //parameter[80]     ;
      lb[80]  = -0.05;
      ub[80]  = -0.037;
      //parameter[81]     ;
      lb[81]  = -0.02;
      ub[81]  = -0.018;
      //parameter[82]     ;
      lb[82]  = -0.05;
      ub[82]  = -0.037;
      //parameter[83]     ;
      lb[83]  = -0.02;
      ub[83]  = -0.018;
      //parameter[84]     ;
      lb[84]  = -0.05;
      ub[84]  = -0.037;
      //parameter[85]     ;
      lb[85]  = -0.02;
      ub[85]  = -0.018;
      //parameter[86]     ;
      lb[86]  = -0.05;
      ub[86]  = -0.037;
      //parameter[87]     ;
      lb[87]  = -0.02;
      ub[87]  = -0.018;

      //parameter[88]     ;
      lb[88]  = -0.06;
      ub[88]  = -0.037;
      //parameter[89]     ;
      lb[89]  = 0.018;
      ub[89]  = 0.02;
      //parameter[90]     ;
      lb[90]  = -0.06;
      ub[90]  = -0.037;
      //parameter[91]     ;
      lb[91]  = 0.018;
      ub[91]  = 0.02;
      //parameter[92]     ;
      lb[92]  = -0.06;
      ub[92]  = -0.037;
      //parameter[93]     ;
      lb[93]  = 0.018;
      ub[93]  = 0.02;
      //parameter[94]     ;
      lb[94]  = -0.06;
      ub[94]  = -0.037;
      //parameter[95]     ;
      lb[95]  = 0.018;
      ub[95]  = 0.02;*/

      /***********************************************************************
       **************************** SYMMETRIC TIGHT U-BEND LIMITS v2 ****************************
       ***********************************************************************/
      /*
      //parameter[0]
      lb[0] = -0.05;
      ub[0] = -0.03;
      //parameter[1]
      lb[1] = 0.037;
      ub[1] = 0.04;
      //parameter[2]
      lb[2] = -0.05;
      ub[2] = -0.03;
      //parameter[3]
      lb[3] = 0.037;
      ub[3] = 0.04;
      //parameter[4]
      lb[4] = -0.05;
      ub[4] = -0.03;
      //parameter[5]
      lb[5] = 0.037;
      ub[5] = 0.04;
      //parameter[6]
      lb[6] = -0.05;
      ub[6] = -0.03;
      //parameter[7]
      lb[7] = 0.037;
      ub[7] = 0.04;
      //parameter[8]
      lb[8] = -0.023;
      ub[8] = -0.018;
      //parameter[9]
      lb[9] = 0.037;
      ub[9] = 0.05;
      //parameter[10]
      lb[10] = -0.023;
      ub[10] = -0.018;
      //parameter[11]
      lb[11] = 0.037;
      ub[11] = 0.06;
      //parameter[12]
      lb[12] = -0.023;
      ub[12] = -0.018;
      //parameter[13]
      lb[13] = 0.037;
      ub[13] = 0.055;
      //parameter[14]
      lb[14] = -0.023;
      ub[14] = -0.018;
      //parameter[15]
      lb[15] = 0.037;
      ub[15] = 0.05;
      //parameter[16]
      lb[16] = 0.018;
      ub[16] = 0.022;
      //parameter[17]
      lb[17] = 0.037;
      ub[17] = 0.06;
      //parameter[18]
      lb[18] = 0.018;
      ub[18] = 0.022;
      //parameter[19]
      lb[19] = 0.037;
      ub[19] = 0.065;
      //parameter[20]
      lb[20] = 0.018;
      ub[20] = 0.022;
      //parameter[21]
      lb[21] = 0.037;
      ub[21] = 0.06;
      //parameter[22]
      lb[22] = 0.018;
      ub[22] = 0.022;
      //parameter[23]
      lb[23] = 0.037;
      ub[23] = 0.05;
      //parameter[24]
      lb[24] = 0.03;
      ub[24] = 0.05;
      //  parameter[25]
      lb[25] = 0.037;
      ub[25] = 0.04;
      //parameter[26]
      lb[26] = 0.03;
      ub[26] = 0.05;
      //  parameter[27]
      lb[27] = 0.037;
      ub[27] = 0.04;
      //parameter[28]
      lb[28] = 0.03;
      ub[28] = 0.05;
      //  parameter[29]
      lb[29] = 0.037;
      ub[29] = 0.04;
      //parameter[30]
      lb[30] = 0.03;
      ub[30] = 0.05;
      //parameter[31]
      lb[31] = 0.037;
      ub[31] = 0.04;
      //parameter[32]
      lb[32] = 0.037;
      ub[32] = 0.06;
      //parameter[33]
      lb[33] = 0.018;
      ub[33] = 0.02;
      //parameter[34]
      lb[34] = 0.037;
      ub[34] = 0.06;
      //parameter[35]
      lb[35] = 0.018;
      ub[35] = 0.02;
      //parameter[36]
      lb[36] = 0.037;
      ub[36] = 0.06;
      //parameter[37]
      lb[37] = 0.018;
      ub[37] = 0.02;
      //parameter[38]
      lb[38] = 0.037;
      ub[38] = 0.06;
      //parameter[39]
      lb[39] = 0.018;
      ub[39] = 0.02;
      //parameter[40]
      lb[40] = 0.037;
      ub[40] = 0.05;
      //parameter[41]
      lb[41] = -0.02;
      ub[41] = -0.018;
      //parameter[42]
      lb[42] = 0.037;
      ub[42] = 0.05;
      //parameter[43]
      lb[43] = -0.02;
      ub[43] = -0.018;
      //parameter[44]
      lb[44] = 0.037;
      ub[44] = 0.05;
      //parameter[45]
      lb[45] = -0.02;
      ub[45] = -0.018;
      //parameter[46]
      lb[46] = 0.037;
      ub[46] = 0.05;
      //parameter[47]
      lb[47] = -0.02;
      ub[47] = -0.018;
      //parameter[48]
      lb[48] = 0.03;
      ub[48] = 0.05;
      //  parameter[49]
      lb[49] = -0.04;
      ub[49] = -0.037;
      //parameter[50]
      lb[50] = 0.03;
      ub[50] = 0.05;
      //  parameter[51]
      lb[51] = -0.04;
      ub[51] = -0.037;
      //parameter[52]
      lb[52] = 0.03;
      ub[52] = 0.05;
      //  parameter[53]
      lb[53] = -0.04;
      ub[53] = -0.037;
      //parameter[54]
      lb[54] = 0.03;
      ub[54] = 0.05;
      //parameter[55]
      lb[55] = -0.04;
      ub[55] = -0.037;
      //parameter[56]
      lb[56] = 0.018;
      ub[56] = 0.022;
      //parameter[57]
      lb[57] = -0.06;
      ub[57] = -0.037;
      //parameter[58]
      lb[58] = 0.018;
      ub[58] = 0.022;
      //parameter[59]
      lb[59] = -0.065;
      ub[59] = -0.037;
      //parameter[60]
      lb[60] = 0.018;
      ub[60] = 0.022;
      //parameter[61]
      lb[61] = -0.06;
      ub[61] = -0.037;
      //parameter[62]
      lb[62] = 0.018;
      ub[62] = 0.022;
      //parameter[63]
      lb[63] = -0.05;
      ub[63] = -0.037;
      //parameter[64]
      lb[64] = -0.023;
      ub[64] = -0.018;
      //parameter[65]
      lb[65] = -0.05;
      ub[65] = -0.037;
      //parameter[66]
      lb[66] = -0.023;
      ub[66] = -0.018;
      //parameter[67]
      lb[67] = -0.06;
      ub[67] = -0.037;
      //parameter[68]
      lb[68] = -0.023;
      ub[68] = -0.018;
      //parameter[69]
      lb[69] = -0.055;
      ub[69] = -0.037;
      //parameter[70]
      lb[70] = -0.023;
      ub[70] = -0.018;
      //parameter[71]
      lb[71] = -0.05;
      ub[71] = -0.037;
      //parameter[72]
      lb[72] = -0.05;
      ub[72] = -0.03;
      //parameter[73]
      lb[73] = -0.04;
      ub[73] = -0.037;
      //parameter[74]
      lb[74] = -0.05;
      ub[74] = -0.03;
      //parameter[75]
      lb[75] = -0.04;
      ub[75] = -0.037;
      //parameter[76]
      lb[76] = -0.05;
      ub[76] = -0.03;
      //parameter[77]
      lb[77] = -0.04;
      ub[77] = -0.037;
      //parameter[78]
      lb[78] = -0.05;
      ub[78] = -0.03;
      //parameter[79]
      lb[79] = -0.04;
      ub[79] = -0.037;
      //parameter[80]
      lb[80] = -0.05;
      ub[80] = -0.037;
      //parameter[81]
      lb[81] = -0.02;
      ub[81] = -0.018;
      //parameter[82]
      lb[82] = -0.05;
      ub[82] = -0.037;
      //parameter[83]
      lb[83] = -0.02;
      ub[83] = -0.018;
      //parameter[84]
      lb[84] = -0.05;
      ub[84] = -0.037;
      //parameter[85]
      lb[85] = -0.02;
      ub[85] = -0.018;
      //parameter[86]
      lb[86] = -0.05;
      ub[86] = -0.037;
      //parameter[87]
      lb[87] = -0.02;
      ub[87] = -0.018;
      //parameter[88]
      lb[88] = -0.06;
      ub[88] = -0.037;
      //parameter[89]
      lb[89] = 0.018;
      ub[89] = 0.02;
      //parameter[90]
      lb[90] = -0.06;
      ub[90] = -0.037;
      //parameter[91]
      lb[91] = 0.018;
      ub[91] = 0.02;
      //parameter[92]
      lb[92] = -0.06;
      ub[92] = -0.037;
      //parameter[93]
      lb[93] = 0.018;
      ub[93] = 0.02;
      //parameter[94]
      lb[94] = -0.06;
      ub[94] = -0.037;
      //parameter[95]
      lb[95] = 0.018;
      ub[95] = 0.02;
      */
      /***********************************************************************
       **************************** SYMMETRIC TIGHT U-BEND LIMITS ****************************
       ***********************************************************************/
      /*
      //parameter[0]
      lb[0] = -0.05;
      ub[0] = -0.03;
      //parameter[1]
      lb[1] = 0.037;
      ub[1] = 0.04;
      //parameter[2]
      lb[2] = -0.05;
      ub[2] = -0.03;
      //parameter[3]
      lb[3] = 0.037;
      ub[3] = 0.04;
      //parameter[4]
      lb[4] = -0.05;
      ub[4] = -0.03;
      //parameter[5]
      lb[5] = 0.037;
      ub[5] = 0.04;
      //parameter[6]
      lb[6] = -0.05;
      ub[6] = -0.03;
      //parameter[7]
      lb[7] = 0.037;
      ub[7] = 0.04;
      //parameter[8]
      lb[8] = -0.023;
      ub[8] = -0.018;
      //parameter[9]
      lb[9] = 0.037;
      ub[9] = 0.05;
      //parameter[10]
      lb[10] = -0.023;
      ub[10] = -0.018;
      //parameter[11]
      lb[11] = 0.037;
      ub[11] = 0.06;
      //parameter[12]
      lb[12] = -0.023;
      ub[12] = -0.018;
      //parameter[13]
      lb[13] = 0.037;
      ub[13] = 0.055;
      //parameter[14]
      lb[14] = -0.023;
      ub[14] = -0.018;
      //parameter[15]
      lb[15] = 0.037;
      ub[15] = 0.05;
      //parameter[16]
      lb[16] = 0.018;
      ub[16] = 0.022;
      //parameter[17]
      lb[17] = 0.037;
      ub[17] = 0.06;
      //parameter[18]
      lb[18] = 0.018;
      ub[18] = 0.022;
      //parameter[19]
      lb[19] = 0.037;
      ub[19] = 0.065;
      //parameter[20]
      lb[20] = 0.018;
      ub[20] = 0.022;
      //parameter[21]
      lb[21] = 0.037;
      ub[21] = 0.06;
      //parameter[22]
      lb[22] = 0.018;
      ub[22] = 0.022;
      //parameter[23]
      lb[23] = 0.037;
      ub[23] = 0.05;
      //parameter[24]
      lb[24] = 0.037;
      ub[24] = 0.05;
      //  parameter[25]
      lb[25] = 0.037;
      ub[25] = 0.05;
      //parameter[26]
      lb[26] = 0.037;
      ub[26] = 0.05;
      //  parameter[27]
      lb[27] = 0.037;
      ub[27] = 0.05;
      //parameter[28]
      lb[28] = 0.037;
      ub[28] = 0.05;
      //  parameter[29]
      lb[29] = 0.037;
      ub[29] = 0.05;
      //parameter[30]
      lb[30] = 0.037;
      ub[30] = 0.05;
      //parameter[31]
      lb[31] = 0.037;
      ub[31] = 0.05;
      //parameter[32]
      lb[32] = 0.037;
      ub[32] = 0.06;
      //parameter[33]
      lb[33] = 0.018;
      ub[33] = 0.02;
      //parameter[34]
      lb[34] = 0.037;
      ub[34] = 0.06;
      //parameter[35]
      lb[35] = 0.018;
      ub[35] = 0.02;
      //parameter[36]
      lb[36] = 0.037;
      ub[36] = 0.06;
      //parameter[37]
      lb[37] = 0.018;
      ub[37] = 0.02;
      //parameter[38]
      lb[38] = 0.037;
      ub[38] = 0.06;
      //parameter[39]
      lb[39] = 0.018;
      ub[39] = 0.02;
      //parameter[40]
      lb[40] = 0.037;
      ub[40] = 0.05;
      //parameter[41]
      lb[41] = -0.020;
      ub[41] = -0.018;
      //parameter[42]
      lb[42] = 0.037;
      ub[42] = 0.05;
      //parameter[43]
      lb[43] = -0.020;
      ub[43] = -0.018;
      //parameter[44]
      lb[44] = 0.037;
      ub[44] = 0.05;
      //parameter[45]
      lb[45] = -0.020;
      ub[45] = -0.018;
      //parameter[46]
      lb[46] = 0.037;
      ub[46] = 0.05;
      //parameter[47]
      lb[47] = -0.020;
      ub[47] = -0.018;
      //parameter[48]
      lb[48] = 0.037;
      ub[48] = 0.05;
      //  parameter[49]
      lb[49] = -0.05;
      ub[49] = -0.037;
      //parameter[50]
      lb[50] = 0.037;
      ub[50] = 0.05;
      //  parameter[51]
      lb[51] = -0.05;
      ub[51] = -0.037;
      //parameter[52]
      lb[52] = 0.037;
      ub[52] = 0.05;
      //  parameter[53]
      lb[53] = -0.05;
      ub[53] = -0.037;
      //parameter[54]
      lb[54] = 0.037;
      ub[54] = 0.05;
      //parameter[55]
      lb[55] = -0.05;
      ub[55] = -0.037;
      //parameter[56]
      lb[56] = 0.018;
      ub[56] = 0.022;
      //parameter[57]
      lb[57] = -0.06;
      ub[57] = -0.037;
      //parameter[58]
      lb[58] = 0.018;
      ub[58] = 0.022;
      //parameter[59]
      lb[59] = -0.065;
      ub[59] = -0.037;
      //parameter[60]
      lb[60] = 0.018;
      ub[60] = 0.022;
      //parameter[61]
      lb[61] = -0.06;
      ub[61] = -0.037;
      //parameter[62]
      lb[62] = 0.018;
      ub[62] = 0.022;
      //parameter[63]
      lb[63] = -0.05;
      ub[63] = -0.037;
      //parameter[64]
      lb[64] = -0.023;
      ub[64] = -0.018;
      //parameter[65]
      lb[65] = -0.05;
      ub[65] = -0.037;
      //parameter[66]
      lb[66] = -0.023;
      ub[66] = -0.018;
      //parameter[67]
      lb[67] = -0.06;
      ub[67] = -0.037;
      //parameter[68]
      lb[68] = -0.023;
      ub[68] = -0.018;
      //parameter[69]
      lb[69] = -0.055;
      ub[69] = -0.037;
      //parameter[70]
      lb[70] = -0.023;
      ub[70] = -0.018;
      //parameter[71]
      lb[71] = -0.05;
      ub[71] = -0.037;
      //parameter[72]
      lb[72] = -0.05;
      ub[72] = -0.03;
      //parameter[73]
      lb[73] = -0.04;
      ub[73] = -0.037;
      //parameter[74]
      lb[74] = -0.05;
      ub[74] = -0.03;
      //parameter[75]
      lb[75] = -0.04;
      ub[75] = -0.037;
      //parameter[76]
      lb[76] = -0.05;
      ub[76] = -0.03;
      //parameter[77]
      lb[77] = -0.04;
      ub[77] = -0.037;
      //parameter[78]
      lb[78] = -0.05;
      ub[78] = -0.03;
      //parameter[79]
      lb[79] = -0.04;
      ub[79] = -0.037;
      //parameter[80]
      lb[80] = -0.05;
      ub[80] = -0.037;
      //parameter[81]
      lb[81] = -0.020;
      ub[81] = -0.018;
      //parameter[82]
      lb[82] = -0.05;
      ub[82] = -0.037;
      //parameter[83]
      lb[83] = -0.020;
      ub[83] = -0.018;
      //parameter[84]
      lb[84] = -0.05;
      ub[84] = -0.037;
      //parameter[85]
      lb[85] = -0.020;
      ub[85] = -0.018;
      //parameter[86]
      lb[86] = -0.05;
      ub[86] = -0.037;
      //parameter[87]
      lb[87] = -0.020;
      ub[87] = -0.018;
      //parameter[88]
      lb[88] = -0.06;
      ub[88] = -0.037;
      //parameter[89]
      lb[89] = 0.018;
      ub[89] = 0.02;
      //parameter[90]
      lb[90] = -0.06;
      ub[90] = -0.037;
      //parameter[91]
      lb[91] = 0.018;
      ub[91] = 0.02;
      //parameter[92]
      lb[92] = -0.06;
      ub[92] = -0.037;
      //parameter[93]
      lb[93] = 0.018;
      ub[93] = 0.02;
      //parameter[94]
      lb[94] = -0.06;
      ub[94] = -0.037;
      //parameter[95]
      lb[95] = 0.018;
      ub[95] = 0.02;
      */
      /***********************************************************************
       **************************** TIGHT U-BEND LIMITS ****************************
       ***********************************************************************/
      /*
      //parameter[0]
      lb[0] = -0.045;
      ub[0] = -0.034;
      //parameter[1]
      lb[1] = 0.035;
      ub[1] = 0.042;
      //parameter[2]
      lb[2] = -0.045;
      ub[2] = -0.034;
      //parameter[3]
      lb[3] = 0.035;
      ub[3] = 0.042;
      //parameter[4]
      lb[4] = -0.045;
      ub[4] = -0.034;
      //parameter[5]
      lb[5] = 0.035;
      ub[5] = 0.042;
      //parameter[6]
      lb[6] = -0.045;
      ub[6] = -0.034;
      //parameter[7]
      lb[7] = 0.035;
      ub[7] = 0.042;
      //parameter[8]
      lb[8] = -0.025;
      ub[8] = -0.018;
      //parameter[9]
      lb[9] = 0.037;
      ub[9] = 0.05;
      //parameter[10]
      lb[10] = -0.025;
      ub[10] = -0.018;
      //parameter[11]
      lb[11] = 0.037;
      ub[11] = 0.05;
      //parameter[12]
      lb[12] = -0.025;
      ub[12] = -0.018;
      //parameter[13]
      lb[13] = 0.037;
      ub[13] = 0.05;
      //parameter[14]
      lb[14] = -0.025;
      ub[14] = -0.018;
      //parameter[15]
      lb[15] = 0.037;
      ub[15] = 0.05;
      //parameter[16]
      lb[16] = 0.018;
      ub[16] = 0.025;
      //parameter[17]
      lb[17] = 0.037;
      ub[17] = 0.055;
      //parameter[18]
      lb[18] = 0.018;
      ub[18] = 0.025;
      //parameter[19]
      lb[19] = 0.037;
      ub[19] = 0.055;
      //parameter[20]
      lb[20] = 0.018;
      ub[20] = 0.025;
      //parameter[21]
      lb[21] = 0.037;
      ub[21] = 0.055;
      //parameter[22]
      lb[22] = 0.018;
      ub[22] = 0.025;
      //parameter[23]
      lb[23] = 0.037;
      ub[23] = 0.055;
      //parameter[24]
      lb[24] = 0.037;
      ub[24] = 0.040;
      //parameter[25]
      lb[25] = 0.037;
      ub[25] = 0.045;
      //parameter[26]
      lb[26] = 0.037;
      ub[26] = 0.040;
      //parameter[27]
      lb[27] = 0.037;
      ub[27] = 0.045;
      //parameter[28]
      lb[28] = 0.037;
      ub[28] = 0.040;
      //parameter[29]
      lb[29] = 0.037;
      ub[29] = 0.045;
      //parameter[30]
      lb[30] = 0.037;
      ub[30] = 0.040;
      //parameter[31]
      lb[31] = 0.037;
      ub[31] = 0.045;
      //parameter[32]
      lb[32] = 0.037;
      ub[32] = 0.06;
      //parameter[33]
      lb[33] = 0.018;
      ub[33] = 0.02;
      //parameter[34]
      lb[34] = 0.037;
      ub[34] = 0.075;
      //parameter[35]
      lb[35] = 0.018;
      ub[35] = 0.02;
      //parameter[36]
      lb[36] = 0.037;
      ub[36] = 0.065;
      //parameter[37]
      lb[37] = 0.018;
      ub[37] = 0.02;
      //parameter[38]
      lb[38] = 0.037;
      ub[38] = 0.045;
      //parameter[39]
      lb[39] = 0.018;
      ub[39] = 0.02;
      //parameter[40]
      lb[40] = 0.037;
      ub[40] = 0.05;
      //parameter[41]
      lb[41] = -0.023;
      ub[41] = -0.018;
      //parameter[42]
      lb[42] = 0.037;
      ub[42] = 0.07;
      //parameter[43]
      lb[43] = -0.023;
      ub[43] = -0.018;
      //parameter[44]
      lb[44] = 0.037;
      ub[44] = 0.065;
      //parameter[45]
      lb[45] = -0.023;
      ub[45] = -0.018;
      //parameter[46]
      lb[46] = 0.037;
      ub[46] = 0.057;
      //parameter[47]
      lb[47] = -0.023;
      ub[47] = -0.018;
      //parameter[48]
      lb[48] = 0.037;
      ub[48] = 0.05;
      //  parameter[49]
      lb[49] = -0.05;
      ub[49] = -0.037;
      //parameter[50]
      lb[50] = 0.037;
      ub[50] = 0.05;
      //  parameter[51]
      lb[51] = -0.05;
      ub[51] = -0.037;
      //parameter[52]
      lb[52] = 0.037;
      ub[52] = 0.05;
      //  parameter[53]
      lb[53] = -0.05;
      ub[53] = -0.037;
      //parameter[54]
      lb[54] = 0.037;
      ub[54] = 0.05;
      //parameter[55]
      lb[55] = -0.05;
      ub[55] = -0.037;
      //parameter[56]
      lb[56] = 0.018;
      ub[56] = 0.022;
      //parameter[57]
      lb[57] = -0.06;
      ub[57] = -0.037;
      //parameter[58]
      lb[58] = 0.018;
      ub[58] = 0.022;
      //parameter[59]
      lb[59] = -0.065;
      ub[59] = -0.037;
      //parameter[60]
      lb[60] = 0.018;
      ub[60] = 0.022;
      //parameter[61]
      lb[61] = -0.06;
      ub[61] = -0.037;
      //parameter[62]
      lb[62] = 0.018;
      ub[62] = 0.022;
      //parameter[63]
      lb[63] = -0.05;
      ub[63] = -0.037;
      //parameter[64]
      lb[64] = -0.023;
      ub[64] = -0.018;
      //parameter[65]
      lb[65] = -0.05;
      ub[65] = -0.037;
      //parameter[66]
      lb[66] = -0.023;
      ub[66] = -0.018;
      //parameter[67]
      lb[67] = -0.06;
      ub[67] = -0.037;
      //parameter[68]
      lb[68] = -0.023;
      ub[68] = -0.018;
      //parameter[69]
      lb[69] = -0.055;
      ub[69] = -0.037;
      //parameter[70]
      lb[70] = -0.023;
      ub[70] = -0.018;
      //parameter[71]
      lb[71] = -0.05;
      ub[71] = -0.037;
      //parameter[72]
      lb[72] = -0.05;
      ub[72] = -0.03;
      //parameter[73]
      lb[73] = -0.04;
      ub[73] = -0.037;
      //parameter[74]
      lb[74] = -0.05;
      ub[74] = -0.03;
      //parameter[75]
      lb[75] = -0.04;
      ub[75] = -0.037;
      //parameter[76]
      lb[76] = -0.05;
      ub[76] = -0.03;
      //parameter[77]
      lb[77] = -0.04;
      ub[77] = -0.037;
      //parameter[78]
      lb[78] = -0.05;
      ub[78] = -0.03;
      //parameter[79]
      lb[79] = -0.04;
      ub[79] = -0.037;
      //parameter[80]
      lb[80] = -0.05;
      ub[80] = -0.037;
      //parameter[81]
      lb[81] = -0.020;
      ub[81] = -0.018;
      //parameter[82]
      lb[82] = -0.05;
      ub[82] = -0.037;
      //parameter[83]
      lb[83] = -0.020;
      ub[83] = -0.018;
      //parameter[84]
      lb[84] = -0.05;
      ub[84] = -0.037;
      //parameter[85]
      lb[85] = -0.020;
      ub[85] = -0.018;
      //parameter[86]
      lb[86] = -0.05;
      ub[86] = -0.037;
      //parameter[87]
      lb[87] = -0.020;
      ub[87] = -0.018;
      //parameter[88]
      lb[88] = -0.06;
      ub[88] = -0.037;
      //parameter[89]
      lb[89] = 0.018;
      ub[89] = 0.02;
      //parameter[90]
      lb[90] = -0.06;
      ub[90] = -0.037;
      //parameter[91]
      lb[91] = 0.018;
      ub[91] = 0.02;
      //parameter[92]
      lb[92] = -0.06;
      ub[92] = -0.037;
      //parameter[93]
      lb[93] = 0.018;
      ub[93] = 0.02;
      //parameter[94]
      lb[94] = -0.06;
      ub[94] = -0.037;
      //parameter[95]
      lb[95] = 0.018;
      ub[95] = 0.02;
      */
      /***********************************************************************
       **************************** U-BEND LIMITS ****************************
       ***********************************************************************/
      /*
      //parameter[0]
      lb[0] = -0.055;
      ub[0] = -0.037;
      //parameter[1]
      lb[1] = 0.037;
      ub[1] = 0.0425;
      //parameter[2]
      lb[2] = -0.060;
      ub[2] = -0.037;
      //parameter[3]
      lb[3] = 0.037;
      ub[3] = 0.0425;
      //parameter[4]
      lb[4] = -0.065;
      ub[4] = -0.030;
      //parameter[5]
      lb[5] = 0.037;
      ub[5] = 0.039;
      //parameter[6]
      lb[6] = -0.055;
      ub[6] = -0.030;
      //parameter[7]
      lb[7] = 0.036;
      ub[7] = 0.038;
      //parameter[8]
      lb[8] = -0.020;
      ub[8] = -0.018;//-0.013;
      //parameter[9]
      lb[9] = 0.037;
      ub[9] = 0.048;
      //parameter[10]
      lb[10] = -0.019;
      ub[10] = -0.015;
      //parameter[11]
      lb[11] = 0.037;
      ub[11] = 0.055;
      //parameter[12]
      lb[12] = -0.020;
      ub[12] = -0.015;
      //parameter[13]
      lb[13] = 0.037;
      ub[13] = 0.045;
      //parameter[14]
      lb[14] = -0.020;
      ub[14] = -0.015;
      //parameter[15]
      lb[15] = 0.037;
      ub[15] = 0.0501;
      //parameter[16]
      lb[16] = 0.013;
      ub[16] = 0.025;
      //parameter[17]
      lb[17] = 0.037;
      ub[17] = 0.055;
     //parameter[18]
      lb[18] = 0.013;
      ub[18] = 0.020;
      //parameter[19]
      lb[19] = 0.037;
      ub[19] = 0.055;
     //parameter[20]
      lb[20] = 0.013;
      ub[20] = 0.025;
      //parameter[21]
      lb[21] = 0.037;
      ub[21] = 0.055;
     //parameter[22]
      lb[22] = 0.010;
      ub[22] = 0.020;
      //parameter[23]
      lb[23] = 0.037;
      ub[23] = 0.060;
      //parameter[24]
      lb[24] = 0.037;
      ub[24] = 0.0401;//0.040;
      //parameter[25]
      lb[25] = 0.037;
      ub[25] = 0.046;
     //parameter[26]
      lb[26] = 0.0365;
      ub[26] = 0.038;//0.055;//
      //parameter[27]
      lb[27] = 0.037;
      ub[27] = 0.0465;
     //parameter[28]
      lb[28] = 0.037;
      ub[28] = 0.050;//0.040;
      //parameter[29]
      lb[29] = 0.025;
      ub[29] = 0.047;
     //parameter[30]
      lb[30] = 0.037;
      ub[30] = 0.055;//0.040;
      //parameter[31]
      lb[31] = 0.037;
      ub[31] = 0.045;
      //parameter[32]
      lb[32] = 0.037;
      ub[32] = 0.075;
      //parameter[33]
      lb[33] = 0.018;
      ub[33] = 0.030;
      //parameter[34]
      lb[34] = 0.037;
      ub[34] = 0.085;
      //parameter[35]
      lb[35] = 0.015;
      ub[35] = 0.025;
      //parameter[36]
      lb[36] = 0.037;
      ub[36] = 0.085;
      //parameter[37]
      lb[37] = 0.012;
      ub[37] = 0.02;
      //parameter[38]
      lb[38] = 0.037;
      ub[38] = 0.065;
      //parameter[39]
      lb[39] = 0.010;
      ub[39] = 0.02;
      //parameter[40]
      lb[40] = 0.037;
      ub[40] = 0.080;
      //parameter[41]
      lb[41] = -0.035;
      ub[41] = -0.018;
      //parameter[42]
      lb[42] = 0.037;
      ub[42] = 0.085;
      //parameter[43]
      lb[43] = -0.040;
      ub[43] = -0.018;
      //parameter[44]
      lb[44] = 0.037;
      ub[44] = 0.095;
      //parameter[45]
      lb[45] = -0.035;
      ub[45] = -0.018;
      //parameter[46]
      lb[46] = 0.037;
      ub[46] = 0.070;
      //parameter[47]
      lb[47] = -0.040;
      ub[47] = -0.018;
      //parameter[48]
      lb[48] = 0.02;
      ub[48] = 0.050;
    //  parameter[49]
      lb[49] = -0.038;
      ub[49] = -0.018;
    //parameter[50]
      lb[50] = 0.037;
      ub[50] = 0.07;
    //  parameter[51]
      lb[51] = -0.045;
      ub[51] = -0.015;
    //parameter[52]
      lb[52] = 0.037;
      ub[52] = 0.095;
    //  parameter[53]
      lb[53] = -0.07;
      ub[53] = -0.037;
    //parameter[54]
      lb[54] = 0.037;
      ub[54] = 0.075;
    //  parameter[55]
      lb[55] = -0.1;//-0.055;
      ub[55] = -0.037;
      //parameter[56]
      lb[56] = 0.018;
      ub[56] = 0.035;
      //parameter[57]
      lb[57] = -0.075;
      ub[57] = -0.037;
      //parameter[58]
      lb[58] = 0.018;
      ub[58] = 0.035;
      //parameter[59]
      lb[59] = -0.038;
      ub[59] = -0.018;
      //parameter[60]
      lb[60] = 0.018;
      ub[60] = 0.040;
      //parameter[61]
      lb[61] = -0.080;
      ub[61] = -0.037;
      //parameter[62]
      lb[62] = 0.018;
      ub[62] = 0.040;
      //parameter[63]
      lb[63] = -0.1;//-0.060;
      ub[63] = -0.037;
      //parameter[64]
      lb[64] = -0.023;
      ub[64] = -0.010;
      //parameter[65]
      lb[65] = -0.038;
      ub[65] = -0.015;
      //parameter[66]
      lb[66] = -0.023;
      ub[66] = -0.010;
      //parameter[67]
      lb[67] = -0.038;
      ub[67] = -0.015;
      //parameter[68]
      lb[68] = -0.035;
      ub[68] = -0.018;
      //parameter[69]
      lb[69] = -0.080;
      ub[69] = -0.037;
      //parameter[70]
      lb[70] = -0.030;
      ub[70] = -0.018;
      //parameter[71]
      lb[71] = -0.09;//-0.075;
      ub[71] = -0.037;
      //parameter[72]
      lb[72] = -0.040;
      ub[72] = -0.020;
      //parameter[73]
      lb[73] = -0.04;
      ub[73] = -0.0369;
      //parameter[74]
      lb[74] = -0.075;//-0.065;
      ub[74] = -0.037;
      //parameter[75]
      lb[75] = -0.04;
      ub[75] = -0.0369;
      //parameter[76]
      lb[76] = -0.090;//-0.070;
      ub[76] = -0.037;
      //parameter[77]
      lb[77] = -0.041;
      ub[77] = -0.0369;
      //parameter[78]
      lb[78] = -0.1;//-0.065;
      ub[78] = -0.037;
      //parameter[79]
      lb[79] = -0.1;//-0.0405;
      ub[79] = -0.037;
      //parameter[80]
      lb[80] = -0.075;//-0.065;
      ub[80] = -0.037;
      //parameter[81]
      lb[81] = -0.0201;
      ub[81] = -0.018;
    //parameter[82]
      lb[82] = -0.090;//-0.065;
      ub[82] = -0.037;
      //parameter[83]
      lb[83] = -0.0205;
      ub[83] = -0.0179;
    //parameter[84]
      lb[84] = -0.1;//-0.065;
      ub[84] = -0.037;
      //parameter[85]
      lb[85] = -0.0205;
      ub[85] = -0.018;
    //parameter[86]
      lb[86] = -0.1;//-0.070;
      ub[86] = -0.037;
      //parameter[87]
      lb[87] = -0.1;//-0.0205;
      ub[87] = -0.018;
      //parameter[88]
      lb[88] = -0.075;//-0.075;
      ub[88] = -0.037;
      //parameter[89]
      lb[89] = 0.0175;
      ub[89] = 0.02;
      //parameter[90]
      lb[90] = -0.090;//-0.075;
      ub[90] = -0.037;
      //parameter[91]
      lb[91] = 0.0175;
      ub[91] = 0.02;
      //parameter[92]
      lb[92] = -0.1;//-0.075;
      ub[92] = -0.037;
      //parameter[93]
      lb[93] = 0.0175;
      ub[93] = 0.02;
      //parameter[94]
      lb[94] = -0.1;//-0.070;
      ub[94] = -0.037;
      //parameter[95]
      lb[95] = -0.1;//0.0179;
      ub[95] = 0.02;
      */


      //setting bounds manually
      /***********************************************************************
       **************************** TIGHT TUB LIMITS ****************************
       ***********************************************************************/
      
//parameter[0]
  lb[0] = 2.;
  ub[0] = 3.;
  //parameter[1]
  lb[1] = 2.;
  ub[1] = 3.;
  //parameter[2]
  lb[2] = 2.;
  ub[2] = 3.;
  //parameter[3]
  lb[3] = 2.;
  ub[3] = 3.;
//parameter[4]
  lb[4] = 2.;
  ub[4] = 3.;
  //parameter[5]
  lb[5] = 2.;
  ub[5] = 3.;
  //parameter[6]
  lb[6] = 2.;
  ub[6] = 3.;
  //parameter[7]
  lb[7] = 2.;
  ub[7] = 3.;
  
  //parameter[8]
  lb[8] = 1.;
  ub[8] = 1.5;
  //parameter[9]
  lb[9] = 1.;
  ub[9] = 1.5;
  //parameter[10]
  lb[10] = 1.;
  ub[10] = 1.5;
  //parameter[11]
  lb[11] = 1.;
  ub[11] = 1.5;
  //parameter[12]
  lb[12] = 1.;
  ub[12] = 1.5;
  //parameter[13]
  lb[13] = 1.;
  ub[13] = 1.5;
  //parameter[14]
  lb[14] = 1.;
  ub[14] = 1.5;
  //parameter[15]
  lb[15] = 1.;
  ub[15] = 1.5;
  
  //parameter[16]
  lb[16] = 3.;
  ub[16] = 5.75;
  //parameter[17]
  lb[17] = 3.;
  ub[17] = 5.75;
  //parameter[18]
  lb[18] = 3.;
  ub[18] = 5.75;
  //parameter[19]
  lb[19] = 3.;
  ub[19] = 5.75;
  //parameter[20]
  lb[20] = 3.;
  ub[20] = 5.75;
  //parameter[21]
  lb[21] = 3.;
  ub[21] = 5.75;
  //parameter[22]
  lb[22] = 3.;
  ub[22] = 5.75;
  //parameter[23]
  lb[23] = 3.;
  ub[23] = 5.75;
  
  //parameter[24]
  lb[24] = 4.3;
  ub[24] = 7.7;
  //parameter[25]
  lb[25] = 4.3;
  ub[25] = 7.7;
  //parameter[26]
  lb[26] = 4.3;
  ub[26] = 7.7;
  //parameter[27]
  lb[27] = 4.3;
  ub[27] = 7.7;
  //parameter[28]
  lb[28] = 4.3;
  ub[28] = 7.7;
  //parameter[29]
  lb[29] = 4.3;
  ub[29] = 7.7;
  //parameter[30]
  lb[30] = 4.3;
  ub[30] = 7.7;
  //parameter[31]
  lb[31] = 4.3;
  ub[31] = 7.7;
  
  
  //parameter[32]
  lb[32] = 4.;
  ub[32] = 7.55;
  //parameter[33]
  lb[33] = 4.;
  ub[33] = 7.55;
 //parameter[34]
  lb[34] = 4.;
  ub[34] = 7.55;
  //parameter[35]
  lb[35] = 4.;
  ub[35] = 7.55;
  //parameter[36]
  lb[36] = 4.;
  ub[36] = 7.55;
  //parameter[37]
  lb[37] = 4.;
  ub[37] = 7.55;
 //parameter[38]
  lb[38] = 4.;
  ub[38] = 7.55;
  //parameter[39]
  lb[39] = 4.;
  ub[39] = 7.55;
  
 //parameter[40]
  lb[40] = 3.5;
  ub[40] = 6.2;
  //parameter[41]
  lb[41] = 3.5;
  ub[41] = 6.2;
 //parameter[42]
  lb[42] = 3.5;
  ub[42] = 6.2;
  //parameter[43]
  lb[43] = 3.5;
  ub[43] = 6.2;
 //parameter[44]
  lb[44] = 3.5;
  ub[44] = 6.2;
  //parameter[45]
  lb[45] = 3.5;
  ub[45] = 6.2;
 //parameter[46]
  lb[46] = 3.5;
  ub[46] = 6.2;
  //parameter[47]
  lb[47] = 3.5;
  ub[47] = 6.2;
  
    
  //parameter[48]
  lb[48] = 3.;
  ub[48] = 4.2;
  //parameter[49]
  lb[49] = 3.;
  ub[49] = 4.2;
 //parameter[50]
  lb[50] = 3.;
  ub[50] = 4.2;
  //parameter[51]
  lb[51] = 3.;
  ub[51] = 4.2;
  //parameter[52]
  lb[52] = 3.;
  ub[52] = 4.2;
  //parameter[53]
  lb[53] = 3.;
  ub[53] = 4.2;
 //parameter[54]
  lb[54] = 3.;
  ub[54] = 4.2;
  //parameter[55]
  lb[55] = 3.;
  ub[55] = 4.2;
  
 //parameter[56]
  lb[56] = 2.3;
  ub[56] = 2.65;
  //parameter[57]
  lb[57] = 2.3;
  ub[57] = 2.65;
 //parameter[58]
  lb[58] = 2.3;
  ub[58] = 2.65;
  //parameter[59]
  lb[59] = 2.3;
  ub[59] = 2.65;
 //parameter[60]
  lb[60] = 2.3;
  ub[60] = 2.65;
  //parameter[61]
  lb[61] = 2.3;
  ub[61] = 2.65;
 //parameter[62]
  lb[62] = 2.3;
  ub[62] = 2.65;
  //parameter[63]
  lb[63] = 2.3;
  ub[63] = 2.65;

  //parameter[64]
  lb[64] = 1.15;
  ub[64] = 1.2;
  //parameter[65]
  lb[65] = 1.15;
  ub[65] = 1.2;
  //parameter[66]
  lb[66] = 1.15;
  ub[66] = 1.2;
  //parameter[67]
  lb[67] = 1.15;
  ub[67] = 1.2;
  //parameter[68]
  lb[68] = 1.15;
  ub[68] = 1.2;
  //parameter[69]
  lb[69] = 1.15;
  ub[69] = 1.2;
  //parameter[70]
  lb[70] = 1.15;
  ub[70] = 1.2;
  //parameter[71]
  lb[71] = 1.15;
  ub[71] = 1.2;
  
  //parameter[72]
  lb[72] = 1.;
  ub[72] = 1.11;
  //parameter[73]
  lb[73] = 1.;
  ub[73] = 1.11;
  //parameter[74]
  lb[74] = 1.;
  ub[74] = 1.11;
  //parameter[75]
  lb[75] = 1.;
  ub[75] = 1.11;
  //parameter[76]
  lb[76] = 1.;
  ub[76] = 1.11;
  //parameter[77]
  lb[77] = 1.;
  ub[77] = 1.11;
  //parameter[78]
  lb[78] = 1.;
  ub[78] = 1.11;
  //parameter[79]
  lb[79] = 1.;
  ub[79] = 1.11;
  
  
  //parameter[80]
  lb[80] = -0.00001;
  ub[80] = 0.6;
  //parameter[81]
  lb[81] = -0.00001;
  ub[81] = 0.3;
    //parameter[82]
  lb[82] = -0.00001;
  ub[82] = 0.6;
  //parameter[83]
  lb[83] = -0.00001;
  ub[83] = 0.3;
    //parameter[84]
  lb[84] = -0.00001;
  ub[84] = 0.6;
  //parameter[85]
  lb[85] = -0.00001;
  ub[85] = 0.3;
    //parameter[86]
  lb[86] = -0.00001;
  ub[86] = 0.6;
  //parameter[87]
  lb[87] = -0.00001;
  ub[87] = 0.3;
    //parameter[88]
  lb[88] = -0.00001;
  ub[88] = 0.6;
  //parameter[89]
  lb[89] = -0.00001;
  ub[89] = 0.3;
    //parameter[90]
  lb[90] = -0.00001;
  ub[90] = 0.6;
  //parameter[91]
  lb[91] = -0.00001;
  ub[91] = 0.3;
    //parameter[92]
  lb[92] = -0.00001;
  ub[92] = 0.6;
  //parameter[93]
  lb[93] = -0.00001;
  ub[93] = 0.3;
    //parameter[94]
  lb[94] = -0.00001;
  ub[94] = 0.6;
  //parameter[95]
  lb[95] = -0.00001;
  ub[95] = 0.3;
    
  //parameter[96]
  lb[96] = 1.;
  ub[96] = 4.5772;
//  parameter[97]
  lb[97] = -7.9068;
  ub[97] = -3.;
    //parameter[98]
  lb[98] = 1.;
  ub[98] = 4.5772;
//  parameter[99]
  lb[99] = -7.9068;
  ub[99] = -3.;
    //parameter[100]
  lb[100] = 1.;
  ub[100] = 4.5772;
//  parameter[101]
  lb[101] = -7.9068;
  ub[101] = -3.;
    //parameter[102]
  lb[102] = 1.;
  ub[102] = 4.5772;
//  parameter[103]
  lb[103] = -7.9068;
  ub[103] = -3.;
    //parameter[104]
  lb[104] = 1.;
  ub[104] = 4.5772;
//  parameter[105]
  lb[105] = -7.9068;
  ub[105] = -3.;
    //parameter[106]
  lb[106] = 1.;
  ub[106] = 4.5772;
//  parameter[107]
  lb[107] = -7.9068;
  ub[107] = -3.;
    //parameter[108]
  lb[108] = 1.;
  ub[108] = 4.5772;
//  parameter[109]
  lb[109] = -7.9068;
  ub[109] = -3.;
    //parameter[110]
  lb[110] = 1.;
  ub[110] = 4.5772;
//  parameter[111]
  lb[111] = -7.9068;
  ub[111] = -3.;
  
  
  //parameter[112]
  lb[112] = 7.52290;
  ub[112] = 20.52299;
  //parameter[113]
  lb[113] = -20.4609;
  ub[113] = -10.4608;
  //parameter[114]
  lb[114] = 7.52290;
  ub[114] = 20.52299;
  //parameter[115]
  lb[115] = -20.4609;
  ub[115] = -10.4608;
  //parameter[116]
  lb[116] = 7.52290;
  ub[116] = 20.52299;
  //parameter[117]
  lb[117] = -20.4609;
  ub[117] = -10.4608;
  //parameter[118]
  lb[118] = 7.52290;
  ub[118] = 20.52299;
  //parameter[119]
  lb[119] = -20.4609;
  ub[119] = -10.4608;
  //parameter[120]
  lb[120] = 7.52290;
  ub[120] = 20.52299;
  //parameter[121]
  lb[121] = -20.4609;
  ub[121] = -10.4608;
  //parameter[122]
  lb[122] = 7.52290;
  ub[122] = 20.52299;
  //parameter[123]
  lb[123] = -20.4609;
  ub[123] = -10.4608;
  //parameter[124]
  lb[124] = 7.52290;
  ub[124] = 20.52299;
  //parameter[125]
  lb[125] = -20.4609;
  ub[125] = -10.4608;
  //parameter[126]
  lb[126] = 7.52290;
  ub[126] = 20.52299;
  //parameter[127]
  lb[127] = -20.4609;
  ub[127] = -10.4608;
   
  //parameter[128]
  lb[128] = 37.5224;
  ub[128] = 57.5226;
  //parameter[129]
  lb[129] = -37.1068;
  ub[129] = -17.1066;
  //parameter[130]
  lb[130] = 37.5224;
  ub[130] = 57.5226;
  //parameter[131]
  lb[131] = -37.1068;
  ub[131] = -17.1066;;
  //parameter[132]
  lb[132] = 37.5224;
  ub[132] = 57.5226;
  //parameter[133]
  lb[133] = -37.1068;
  ub[133] = -17.1066;
  //parameter[134]
  lb[134] = 37.5224;
  ub[134] = 57.5226;
  //parameter[135]
  lb[135] = -37.1068;
  ub[135] = -17.1066;
  //parameter[136]
  lb[136] = 37.5224;
  ub[136] = 57.5226;
  //parameter[137]
  lb[137] = -37.1068;
  ub[137] = -17.1066;
  //parameter[138]
  lb[138] = 37.5224;
  ub[138] = 57.5226;
  //parameter[139]
  lb[139] = -37.1068;
  ub[139] = -17.1066;
  //parameter[140]
  lb[140] = 37.5224;
  ub[140] = 57.5226;
  //parameter[141]
  lb[141] = -37.1068;
  ub[141] = -17.1066;
  //parameter[142]
  lb[142] = 37.5224;
  ub[142] = 57.5226;
  //parameter[143]
  lb[143] = -37.1068;
  ub[143] = -17.1066;
  
  
  
  
  //parameter[144]
  lb[144] = 85.;
  ub[144] = 105.;
  //parameter[145]
  lb[145] = -50.;
  ub[145] = -28.9;
  //parameter[146]
  lb[146] = 85.;
  ub[146] = 105.;
  //parameter[147]
  lb[147] = -50.;
  ub[147] = -28.9;
  //parameter[148]
  lb[148] = 85.;
  ub[148] = 105.;
  //parameter[149]
  lb[149] = -50.;
  ub[149] = -28.9;
  //parameter[150]
  lb[150] = 85.;
  ub[150] = 105.;
  //parameter[151]
  lb[151] = -50.;
  ub[151] = -28.9;
  //parameter[152]
  lb[152] = 85.;
  ub[152] = 105.;
  //parameter[153]
  lb[153] = -50.;
  ub[153] = -28.9;
  //parameter[154]
  lb[154] = 85.;
  ub[154] = 105.;
  //parameter[155]
  lb[155] = -50.;
  ub[155] = -28.9;
  //parameter[156]
  lb[156] = 85.;
  ub[156] = 105.;
  //parameter[157]
  lb[157] = -50.;
  ub[157] = -28.9;
  //parameter[158]
  lb[158] = 85.;
  ub[158] = 105.;
  //parameter[159]
  lb[159] = -50.;
  ub[159] = -28.9;
  
  //parameter[160]
  lb[160] = 150.;
  ub[160] = 180.;
  //parameter[161]
  lb[161] = -50.;
  ub[161] = -24.;
  //parameter[162]
  lb[162] = 150.;
  ub[162] = 180.;
//parameter[163]
  lb[163] = -50.;
  ub[163] = -24.;
  //parameter[164]
  lb[164] = 150.;
  ub[164] = 180.;
//parameter[165]
  lb[165] = -50.;
  ub[165] = -24.;
  //parameter[166]
  lb[166] = 150.;
  ub[166] = 180.;
//parameter[167]
  lb[167] = -50.;
  ub[167] = -24.;
  //parameter[168]
  lb[168] = 150.;
  ub[168] = 180.;
//parameter[169]
  lb[169] = -50.;
  ub[169] = -24.;
  //parameter[170]
  lb[170] = 150.;
  ub[170] = 180.;
//parameter[171]
  lb[171] = -50.;
  ub[171] = -24.;
  //parameter[172]
  lb[172] = 150.;
  ub[172] = 180.;
//parameter[173]
  lb[173] = -50.;
  ub[173] = -24.;
 //parameter[174]
  lb[174] = 150.;
  ub[174] = 180.;
  //parameter[175]
  lb[175] = -50.;
  ub[175] = -24.;
  
  
  //parameter[176]
  lb[176] = -45.;
  ub[176] = -20.;
  //parameter[177]
  lb[177] = -45.;
  ub[177] = -20.;
  //parameter[178]
  lb[178] = -45.;
  ub[178] = -20.;
  //parameter[179]
  lb[179] = -45.;
  ub[179] = -20.;
  //parameter[180]
  lb[180] = -45.;
  ub[180] = -20.;
  //parameter[181]
  lb[181] = -45.;
  ub[181] = -20.;
  //parameter[182]
  lb[182] = -45.;
  ub[182] = -20.;
  //parameter[183]
  lb[183] = -45.;
  ub[183] = -20.; 
      
      /***********************************************************************
       **************************** TUB LIMITS ****************************
       ***********************************************************************/
      /*
      //parameter[0]
      lb[0] = 1.1;
      ub[0] = 12.;
      //parameter[1]
      lb[1] = 1.1;
      ub[1] = 12.;
      //parameter[2]
      lb[2] = 1.1;
      ub[2] = 12.;
      //parameter[3]
      lb[3] = 1.1;
      ub[3] = 12.;
      //parameter[4]
      lb[4] = 1.;
      ub[4] = 6.;
      //parameter[5]
      lb[5] = 1.;
      ub[5] = 6.;
      //parameter[6]
      lb[6] = 1.;
      ub[6] = 6.;
      //parameter[7]
      lb[7] = 1.;
      ub[7] = 6.;
      //parameter[8]
      lb[8] = 1.1;
      ub[8] = 15.;
      //parameter[9]
      lb[9] = 1.1;
      ub[9] = 15.;
      //parameter[10]
      lb[10] = 1.1;
      ub[10] = 15.;
      //parameter[11]
      lb[11] = 1.1;
      ub[11] = 15.;
      //parameter[12]
      lb[12] = 1.1;
      ub[12] = 14.;
      //parameter[13]
      lb[13] = 1.1;
      ub[13] = 14.;
      //parameter[14]
      lb[14] = 1.1;
      ub[14] = 14.;
      //parameter[15]
      lb[15] = 1.1;
      ub[15] = 14.;
      //parameter[16]
      lb[16] = 4.;
      ub[16] = 14.;
      //parameter[17]
      lb[17] = 4.;
      ub[17] = 14.;
      //parameter[18]
      lb[18] = 4.;
      ub[18] = 14.;
      //parameter[19]
      lb[19] = 4.;
      ub[19] = 14.;
      //parameter[20]
      lb[20] = 4.;
      ub[20] = 12.;
      //parameter[21]
      lb[21] = 4.;
      ub[21] = 12.;
      //parameter[22]
      lb[22] = 4.;
      ub[22] = 12.;
      //parameter[23]
      lb[23] = 4.;
      ub[23] = 12.;
      //parameter[24]
      lb[24] = 1.1;
      ub[24] = 10.;
      //parameter[25]
      lb[25] = 1.1;
      ub[25] = 10.;
      //parameter[26]
      lb[26] = 1.1;
      ub[26] = 10.;
      //parameter[27]
      lb[27] = 1.1;
      ub[27] = 10.;
      //parameter[28]
      lb[28] = 1.1;
      ub[28] = 10.;
      //parameter[29]
      lb[29] = 1.1;
      ub[29] = 10.;
      //parameter[30]
      lb[30] = 1.1;
      ub[30] = 10.;
      //parameter[31]
      lb[31] = 1.1;
      ub[31] = 10.;

      //parameter[32]
      lb[32] = 1.1;
      ub[32] = 8.;

      //parameter[33]
      lb[33] = 1.1;
      ub[33] = 8.;
      //parameter[34]
      lb[34] = 1.1;
      ub[34] = 8.;
      //parameter[35]
      lb[35] = 1.1;
      ub[35] = 8.;
      //parameter[36]
      lb[36] = 1.;
      ub[36] = 8.;
      //parameter[37]
      lb[37] = 1.;
      ub[37] = 8.;
      //parameter[38]
      lb[38] = 1.;
      ub[38] = 8.;
      //parameter[39]
      lb[39] = 1.;
      ub[39] = 8.;
      //parameter[40]
      lb[40] = -1.;
      ub[40] = 0.0051;
      //parameter[41]
      lb[41] = -0.1;
      ub[41] = 0.1;
      //parameter[42]
      lb[42] = -1.;
      ub[42] = 0.0051;
      //parameter[43]
      lb[43] = -0.1;
      ub[43] = 0.1;
      //parameter[44]
      lb[44] = -1.;
      ub[44] = 0.0051;
      //parameter[45]
      lb[45] = -0.1;
      ub[45] = 0.1;
      //parameter[46]
      lb[46] = -1.;
      ub[46] = 0.0051;
      //parameter[47]
      lb[47] = -0.1;
      ub[47] = 0.1;
      //parameter[48]
      lb[48] = 1.;
      ub[48] = 10.;
      //  parameter[49]
      lb[49] = -10.;
      ub[49] = -1.;

      //parameter[50]
      lb[50] = 1.;
      ub[50] = 10.;
      //  parameter[51]
      lb[51] = -10.;
      ub[51] = -1.;
      //parameter[52]
      lb[52] = 1.;
      ub[52] = 10.;
      //  parameter[53]
      lb[53] = -10.;
      ub[53] = -1.;
      //parameter[54]
      lb[54] = 1.;
      ub[54] = 10.;
      //  parameter[55]
      lb[55] = -10.;
      ub[55] = -1.;
      //parameter[56]
      lb[56] = 6.;
      ub[56] = 20.;

      //parameter[57]
      lb[57] = -20.;
      ub[57] = -10.;
      //parameter[58]
      lb[58] = 6.;
      ub[58] = 20.;
      //parameter[59]
      lb[59] = -20.;
      ub[59] = -10.;
      //parameter[60]
      lb[60] = 6.;
      ub[60] = 20.;
      //parameter[61]
      lb[61] = -20.;
      ub[61] = -10.;
      //parameter[62]
      lb[62] = 6.;
      ub[62] = 20.;
      //parameter[63]
      lb[63] = -20.;
      ub[63] = -10.;
      //parameter[64]
      lb[64] = 42.;
      ub[64] = 58.;
      //parameter[65]
      lb[65] = -35.;
      ub[65] = -17.;
      //parameter[66]
      lb[66] = 42.;
      ub[66] = 58.;
      //parameter[67]
      lb[67] = -35.;
      ub[67] = -17.;
      //parameter[68]
      lb[68] = 42.;
      ub[68] = 58.;
      //parameter[69]
      lb[69] = -35.;
      ub[69] = -17.;
      //parameter[70]
      lb[70] = 42.;
      ub[70] = 58.;
      //parameter[71]
      lb[71] = -35.;
      ub[71] = -17.;
      //parameter[72]
      lb[72] = 80.;
      ub[72] = 110.;
      //parameter[73]
      lb[73] = -50.;
      ub[73] = -30.;
      //parameter[74]
      lb[74] = 80.;
      ub[74] = 110.;
      //parameter[75]
      lb[75] = -50.;
      ub[75] = -30.;
      //parameter[76]
      lb[76] = 80.;
      ub[76] = 110.;
      //parameter[77]
      lb[77] = -50.;
      ub[77] = -30.;
      //parameter[78]
      lb[78] = 80.;
      ub[78] = 110.;
      //parameter[79]
      lb[79] = -50.;
      ub[79] = -30.;
      //parameter[80]
      lb[80] = 150.;
      ub[80] = 180.;
      //parameter[81]
      lb[81] = -50.;
      ub[81] = -30.;
      //parameter[82]
      lb[82] = 150.;
      ub[82] = 180.;
      //parameter[83]
      lb[83] = -50.;
      ub[83] = -30.;
      //parameter[84]
      lb[84] = 150.;
      ub[84] = 180.;
      //parameter[85]
      lb[85] = -50.;
      ub[85] = -30.;
      //parameter[86]
      lb[86] = 150.;
      ub[86] = 180.;
      //parameter[87]
      lb[87] = -50.;
      ub[87] = -30.;
      //parameter[88]
      lb[88] = -39.2;
      ub[88] = -38.9;
      //parameter[89]
      lb[89] = -39.2;
      ub[89] = -38.9;
      //parameter[90]
      lb[90] = -39.2;
      ub[90] = -38.9;
      //parameter[91]
      lb[91] = -39.2;
      ub[91] = -38.9;
      */
//      for(i = 0; i < n; i++)
//      {
//        cout << i << ":" << (lb[i] < x[i] && x[i] < ub[i]) << endl;
//      }

      /**********************************************************************************************
       **********************************START lBFGSb************************************************
       **********************************************************************************************/

      cout << "############## Optimization with lBFGSb ##############" << endl;
      int test=0;
      vector<Standard_Real> x_new(n);

      std::ofstream os;
      string filename = mgopt.CasePath + "/optimisation/optimisationBFGSIterations.txt";
      os.open(filename,ios::app);
      os.precision(10);
      int setulbCnt = 0;
      double maxMeshDeform;
      do{
        // call bfgs
        setulb_(&n,&m,x,lb,ub,nbd,&target,gradient,&factr,&pgtol,wa,iwa,task,&info,csave,lsave,isave,dsave);

        os << "setulb_ call : " << setulbCnt << endl;

        os << "x_new:\t";
        for(i = 0; i < n; i++){
          x_new[i] = x[i];
          os << x_new[i].getValue() << " ";
        }
        os << endl;

        // what to do?
        if (strncmp(task,"FG",2)==0)
        {
          os << "FG flag" << endl;
          // the minimization routine has returned to request the function f and gradient g values at the current x.
          if(setulbCnt != 0)
          {
            maxMeshDeform = DesignStep(x_new);
            cout << "maxMeshDeform = " << maxMeshDeform;

            target = CFDCostFunction(restart);
            g = CFDCADGradientUsingADTraceMode(restart);
          }
          grad_norm = 0.;
          os << "gradients:\t";
          for(i = 0; i < n; i++)
          {
            gradient[i] = g[i];
            grad_norm += g[i]*g[i];
            os << gradient[i] << " ";
          }
          os << endl;
          grad_norm = sqrt(grad_norm);
          os << "target:\t" << target << endl;
          os << "g_norm:\t" << grad_norm << endl;
          cout << "////////////////////////////////////////" << endl;
          cout << "GRAD_NORM: " << grad_norm << endl;
        }
        else
        {
          iterationNumber++;
          if(strncmp(task,"NEW_X",5)==0)
          {
            os << "NEW_X flag" << endl;
            //the minimization routine has returned with a new iterate, and we have opted to continue the iteration.
            fprintf(stdout,"%e %e %d  XXX NEW_X\n",grad_norm,target,isave[29]);

            iterationStepFile = mgopt.CasePath +"/optimisation/CAD/currentDesign_" + std::to_string(iterationNumber) + ".stp";
            occt.StepWriter(iterationStepFile.c_str());
            mgopt.BackupSolution("/optimisation/CFD","iteration_"+to_string(iterationNumber));

            OptimisationState optimisationState(mgopt.CasePath);
            optimisationState.iteration=iterationNumber;
            optimisationState.CADSurfaces=occt.facemap.Extent();
            optimisationState.CostFunction=target;
            optimisationState.GradNorm=norm(g);
            optimisationState.params=x_new;
            optimisationState.primalConvergence=mgopt.GetPrimalCovergence();
            optimisationState.adjointConvergence=mgopt.GetAdjointConvergence();
            optimisationState.Gradient=g;
            optimisationState.EffectiveParametersIndexes = maxNElementsAbsVector(5,g);
            optimisationState.MaxMeshPerturbation = maxMeshDeform;

            optimisationState.WriteState();
            optimisationState.WriteCostData();
            optimisationState.WriteParamsData();
          }
          else
          {
            //If task is neither FG nor NEW_X we terminate execution.
            test = 1;
          }
        }
        os << "===============================================================================" << endl;
        setulbCnt++;
      }while(test==0);

      os.close();

      delete gradient;
      delete wa; delete iwa; delete task; delete csave;
      delete x;
      delete lb; delete ub; delete dsave; delete isave;
  }


    //Testing Methods

    void FindAndWriteProjections(){
        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshes(meshTable, designTable);
        WriteProjectionsData();
    }

    void WriteProjectionsData(){
        vector<vector<double>> parametricTable(2,vector<double>(occt.meshpointsParametric.size()));
        vector<double> gaps(occt.meshpointsParametric.size());

        for (int i=0;i<occt.meshpointsParametric.size();i++) {
            parametricTable[0][i] = occt.meshpointsParametric[i].X().getValue();
            parametricTable[1][i] = occt.meshpointsParametric[i].Y().getValue();
            gaps[i] = occt.meshpointsGap[i].getValue();
        }

        cout << "populating meshpointsParametric" << endl;
        mgopt.PopulateTableByName(parametricTable,"meshpointsParametric");
        cout << "populating meshpointFaceIndex" << endl;
        mgopt.PopulateVectorByName(occt.meshpointsFaceIndex, "meshpointFaceIndex");
        cout << "populating meshpointsGap" << endl;
        mgopt.PopulateVectorByName(gaps,"meshpointsGap");
        cout << "populating meshPointsOnFace" << endl;
        mgopt.PopulateVectorByName(occt.meshpointsFaceIndex,"meshPointsOnFace");
    }

    void ReadProjectionData(){

        cout << " Read Data Projections" << endl;
        auto parametricTable = mgopt.GetTableByName<double>("meshpointsParametric");
        auto gaps = mgopt.GetVectorByName<double>("meshpointsGap");
        int meshpointsSize = parametricTable[0].size();
        occt.meshpointsParametric=vector<gp_Pnt2d>(meshpointsSize);
        occt.meshpointsGap=vector<Standard_Real>(meshpointsSize);
//        occt.meshpointsFaceIndex=vector<int>(meshpointsSize);

        for (int i=0;i<meshpointsSize;i++){
            occt.meshpointsParametric[i]=gp_Pnt2d(parametricTable[0][i],parametricTable[1][i]);
            occt.meshpointsGap[i] = gaps[i];
        }

        occt.meshpointsFaceIndex = mgopt.GetVectorByName<int>("meshpointFaceIndex");
        occt.meshPointsOnFace = mgopt.GetVectorByName<int>("meshPointsOnFace");

    }

    void ReadProjectionData(string projectionFilePath){

        auto mgoptCasepathOld = mgopt.CasePath;
        mgopt.CasePath = projectionFilePath;
        cout << " Read Data Projections" << endl;
        auto parametricTable = mgopt.GetTableByName<double>("meshpointsParametric");
        auto gaps = mgopt.GetVectorByName<double>("meshpointsGap");
        int meshpointsSize = parametricTable[0].size();
        occt.meshpointsParametric=vector<gp_Pnt2d>(meshpointsSize);
        occt.meshpointsGap=vector<Standard_Real>(meshpointsSize);
//        occt.meshpointsFaceIndex=vector<int>(meshpointsSize);

        for (int i=0;i<meshpointsSize;i++){
            occt.meshpointsParametric[i]=gp_Pnt2d(parametricTable[0][i],parametricTable[1][i]);
            occt.meshpointsGap[i] = gaps[i];
        }

        occt.meshpointsFaceIndex = mgopt.GetVectorByName<int>("meshpointFaceIndex");
        occt.meshPointsOnFace = mgopt.GetVectorByName<int>("meshPointsOnFace");

        mgopt.CasePath = mgoptCasepathOld;

    }

    void WriteCadSensitivityInVectorMode()
    {
#if !REVERSE_MODE
      OSD_Timer aTimer;

      aTimer.Start();
      auto designTable = mgopt.GetDesignTable();
      auto meshTable = mgopt.GetMeshTable();
      occt.SetMeshes(meshTable, designTable);
      cout << "occt.meshpointsParametric.size(): " << occt.meshpointsParametric.size() << endl;
      aTimer.Stop();
      cout << "Total time for finding mesh projections: " << aTimer.ElapsedTime() << endl;
      aTimer.Reset();

      aTimer.Start();
      vector<vector<Standard_Real>> matrixAD = occt.GetCompleteDesignSensitivityUsingVectorMode();
      aTimer.Stop();
      cout << "Full Matrix AD Succeeded in time: " << aTimer.ElapsedTime() << endl;

      for(int paramIndex = 0; paramIndex < occt.nParams; paramIndex++)
      {
        vector<vector<double>> table(3,vector<double>(occt.meshpointsParametric.size()));
        for(int i = 0; i < occt.meshpointsParametric.size(); i++)
        {
          table[0][i] = matrixAD[0][i].getADValue(paramIndex);
          table[1][i] = matrixAD[1][i].getADValue(paramIndex);
          table[2][i] = matrixAD[2][i].getADValue(paramIndex);
        }
        string name = "CAD_Sensitivity_"+to_string(paramIndex);
        mgopt.PopulateTableByName(table,name);
      }
#else
      cout << "REVERSE MODE IS ACTIVE, PROJECTIONS NOT WRITTEN!" << endl;
#endif
    }

    void WriteCADSensitivityToTextFile(int paramIndex)
    {
#if !REVERSE_MODE
      OSD_Timer aTimer;
      aTimer.Start();
      auto designTable = mgopt.GetDesignTable();
      auto meshTable = mgopt.GetMeshTable();
      //read projections
      Settings_Ubend_Parametric settings_ubend;
      string projectionFile = settings_ubend.projectionFile;
      occt.SetMeshesNoProjection(meshTable, designTable);
      ReadProjectionData(projectionFile);
      cout << "Projection data read successfully" << endl;
      cout << "occt.meshpointsParametric.size(): " << occt.meshpointsParametric.size() << endl;
      aTimer.Stop();
      cout << "Total time for loading mesh projections: " << aTimer.ElapsedTime() << endl;
      aTimer.Reset();

      aTimer.Start();
      vector<vector<double>> sens = occt.GetDesignSensitivity(paramIndex);
      vector<vector<double>> fdsens = occt.GetDesignSensitivityFD(paramIndex);
      aTimer.Stop();
      cout << "Total time for evaluating AD and FD sensitivities: " << aTimer.ElapsedTime() << endl;
      std::ofstream os;
      string filename = "CAD_AD_vs_FD_sensitivities.txt";
      os.open(filename,ios::app);
      os.precision(10);
      int counter = 0;
      double maxdiff = 0.;
      double currentdiff;
      for(int i = 0; i < occt.meshpointsParametric.size(); i++)
      {
        if(sens[0][i] > 10E-10 || sens[1][i] > 10E-10 || sens[2][i] > 10E-10)
        {
          os << "[0][" << i << "]\t AD: " << sens[0][i] << "\tFD: " << fdsens[0][i];
          if(fdsens[0][i] != 0)
          {
            currentdiff = abs(sens[0][i] - fdsens[0][i]);
            if(currentdiff > maxdiff)
              maxdiff = currentdiff;
            os << "\tDIFF: " << currentdiff << endl;
          }
          else
            os << endl;
          os << "[1][" << i << "]\t AD: " << sens[1][i] << "\tFD: " << fdsens[1][i];
          if(fdsens[1][i] != 0)
          {
            currentdiff = abs(sens[1][i] - fdsens[1][i]);
            if(currentdiff > maxdiff)
              maxdiff = currentdiff;
            os << "\tDIFF: " << currentdiff << endl;
          }
          else
            os << endl;
          os << "[2][" << i << "]\t AD: " << sens[2][i] << "\tFD: " << fdsens[2][i];
          if(fdsens[2][i] != 0)
          {
            currentdiff = abs(sens[2][i] - fdsens[2][i]);
            if(currentdiff > maxdiff)
              maxdiff = currentdiff;
            os << "\tDIFF: " << currentdiff << endl << endl;
          }
          else
            os << endl << endl;
          counter++;
        }
      }
      os.close();
      cout << "Writing to File Done" << endl;
      cout << "Total counter: " << counter << endl;
      cout << "maxdiff: " << maxdiff << endl;
#endif
    }

    void WriteCADSensitivityUsingADTraceMode()
    {
#if REVERSE_MODE
      OSD_Timer aTimer;
      aTimer.Start();
      auto designTable = mgopt.GetDesignTable();
      auto meshTable = mgopt.GetMeshTable();
      bool projectMesh = false;
      if (projectMesh) {
          occt.SetMeshes(meshTable, designTable);
      }
      else
      {
          string projectionFile = "/home/mbanovic/IODA/Development/Ubend/2016-09-22";
          occt.SetMeshesNoProjection(meshTable,designTable);
          ReadProjectionData(projectionFile);
          cout << "Projection data read successfully" << endl;
      }
      cout << "occt.meshpointsParametric.size(): " << occt.meshpointsParametric.size() << endl;
      aTimer.Stop();
      cout << "Total time for finding mesh projections: " << aTimer.ElapsedTime() << endl;
      aTimer.Reset();

      aTimer.Start();
      // ----------- GENERATE AD TRACE --------
      occt.GenerateADTrace();
      aTimer.Stop();
      cout << "Total time for tracing: " << aTimer.ElapsedTime() << endl;

      aTimer.Reset();
      aTimer.Start();
      for(int paramIndex = 0; paramIndex < occt.nParams; paramIndex++)
      {
        vector<vector<double>> table = occt.GetDesignSensitivityByEvaluatingADTrace(paramIndex);
        string name = "";
        if(paramIndex < 10)
          name = "CAD_Sensitivity_0"+to_string(paramIndex);
        else
          name = "CAD_Sensitivity_"+to_string(paramIndex);
        mgopt.PopulateTableByName(table,name);
      }
      aTimer.Stop();
      cout << "Total sensitivity calculation time: " << aTimer.ElapsedTime() << endl;
#else
      cout << "REVERSE MODE IS DEACTIVATED, PROJECTIONS NOT WRITTEN!" << endl;
#endif
    }

    void WriteCadSensitivity(int index){
#if !REVERSE_MODE
        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();


        occt.SetMeshes(meshTable, designTable);

        auto table = occt.GetDesignSensitivity(index);

        string name = "CAD_Sensitivity_"+to_string(index);
        mgopt.PopulateTableByName(table,name);
#else
#endif
    }

    void WriteCadSensitivity(vector<int> indexes){
#if !REVERSE_MODE
        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();


        occt.SetMeshes(meshTable, designTable);
        for (int i=0;i<indexes.size(); i++){
            auto index = indexes[i];
            auto table = occt.GetDesignSensitivity(index);
            string name = "CAD_Sensitivity_"+to_string(index);

            cout << name << endl;
            cout << "norm X=" << norm(table[0]) <<endl;
            cout << "norm Y=" << norm(table[1]) <<endl;
            cout << "norm Z=" << norm(table[2]) <<endl;
            // mgopt.PopulateTableByName(table,name);
        }
#else
#endif
    }

    void WriteCadSensitivitiesUsingFD()
    {
      auto designTable = mgopt.GetDesignTable();
      auto meshTable = mgopt.GetMeshTable();
      OSD_Timer aTimer;
      aTimer.Start();
      occt.SetMeshes(meshTable, designTable);
      occt.UpdateMeshOnCAD();
      aTimer.Stop();
      cout << "Total time for finding mesh projections: " << aTimer.ElapsedTime() << endl;
      aTimer.Reset();

      for(int paramIndex = 0; paramIndex < occt.nParams; paramIndex++)
      {
        vector<vector<double>> matrixFD = occt.GetDesignSensitivityFD(paramIndex);
        string name = "FD_Sensitivity_"+to_string(paramIndex);
        mgopt.PopulateTableByName(matrixFD,name);
        cout << "FD FOR PARAMETER INDEX: " << paramIndex << " DONE!" << endl;
      }
    }

    void CompareCAD_AD_FD(int paramindex){
#if !REVERSE_MODE
        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshes(meshTable, designTable);
        occt.UpdateMeshOnCAD();

        auto matrixAD = occt.GetDesignSensitivity(paramindex);
        cout << "Matrix AD Succeeded" << endl;
        auto matrixFD = occt.GetDesignSensitivityFD(paramindex);
        cout << "Matrix FD Succeeded" << endl;

        int indexNotEqual = 0;
        int indexNotZero = 0;

        for (int i = 0; i<matrixAD[0].size();i++){
            for (int j = 0;j<3;j++){
                if (abs(matrixAD[j][i]-matrixFD[j][i]) >10E-8)
                    indexNotEqual++;
                if (abs(matrixAD[j][i])>0.0001)
                    indexNotZero++;
                //  cout << i << " " << j << endl;
            }

        }
        cout << "Out of Matrix " << endl;

        cout << indexNotEqual << endl;
        cout << indexNotZero << endl;
#else
#endif
    }

    void CompareFullLoopAD_FD(int paramindex){

        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshes(meshTable, designTable);
        occt.UpdateMeshOnCAD();

        auto notperturbedCostFunction = CFDCostFunction();
//        auto g = CFDCADGradient();
//
//        double ad_grad = g[paramindex];


        // Finding FD full CFD CAD value
        double paramstep = 4*10E-9;

        vector<Standard_Real> x = occt.designParameters;
        x[paramindex] +=paramstep;

        double maxPerturb = DesignStep(x);

        auto perturbedCostFunction = CFDCostFunction();
        double fd_grad = (perturbedCostFunction - notperturbedCostFunction)/paramstep;

        cout.precision(15);
        cout << "cost unpert = "  << notperturbedCostFunction << endl;
        cout << "cost perturbed = " << perturbedCostFunction << endl;

        // cout << "AD GRAD=" << ad_grad << endl;
        cout << "FD GRAD=" << fd_grad << endl;

        cout << "MaxPerturbation=" << maxPerturb << endl;
    }

    void CompareFullLoopAD_FD_Central(int paramindex){

        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshes(meshTable, designTable);
        DesignStepMeshToCAD();

        auto notperturbedCostFunction = CFDCostFunction();
        auto g = CFDCADGradient();

        double ad_grad = g[paramindex];
        //  double ad_grad = -2.50961039133989;

        // Finding FD full CFD CAD value
        double paramstep = 10E-8;

        vector<Standard_Real> x = occt.designParameters;
        x[paramindex] +=paramstep;

        double maxPerturbPlus = DesignStep(x);

        auto perturbedCostFunctionPlus = CFDCostFunction();


        x[paramindex] -=2*paramstep;
        double maxPerturbMinus = DesignStep(x);

        auto perturbedCostFunctionMinus = CFDCostFunction();

        cout.precision(15);
        cout << "cost unpert = "  << notperturbedCostFunction << endl;
        cout << "cost perturbed Plus = " << perturbedCostFunctionPlus << endl;
        cout << "cost perturbed Minus = " << perturbedCostFunctionPlus << endl;


        double fd_Plus = (perturbedCostFunctionPlus-notperturbedCostFunction)/paramstep;
        double fd_Minus = (notperturbedCostFunction - perturbedCostFunctionMinus)/paramstep;
        double fd_central = (perturbedCostFunctionPlus-perturbedCostFunctionMinus)/(2*paramstep);

        cout << "AD GRAD=" << ad_grad << endl;
        cout << "FD Plus=" << fd_Plus << endl;
        cout << "FD Minus" << fd_Minus << endl;
        cout << "FD Central" << fd_central << endl;

        cout << "MaxPerturbation Plus=" << maxPerturbPlus << endl;
        cout << "MaxPerturbation Minus=" << maxPerturbPlus << endl;
    }

    void CompareFullLoopAD_FD_Central(vector<int> paramindexes) {

        cout << "Compare Full loop AD _FD Central" << endl;
        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshesNoProjection(meshTable, designTable);
        ReadProjectionData();
        DesignStepMeshToCAD();

        auto notperturbedCostFunction = CFDCostFunction();
        auto g = CFDCADGradient(paramindexes);


        vector<Standard_Real> x = occt.designParameters;
        for (int i = 0; i < paramindexes.size(); i++) {

            int paramindex = paramindexes[i];


            double ad_grad = g[paramindex];
            //  double ad_grad = -2.50961039133989;

            // Finding FD full CFD CAD value
            double paramstep = 10E-6;


            x[paramindex] += paramstep;

            double maxPerturbPlus = DesignStep(x);
            auto perturbedCostFunctionPlus = CFDCostFunction();

            x[paramindex] -= 2 * paramstep;
            double maxPerturbMinus = DesignStep(x);

            auto perturbedCostFunctionMinus = CFDCostFunction();


            cout << "=============CAD index =" << paramindex <<"===============" << endl;

            cout.precision(15);
            cout << "cost unpert = " << notperturbedCostFunction << endl;
            cout << "cost perturbed Plus = " << perturbedCostFunctionPlus << endl;
            cout << "cost perturbed Minus = " << perturbedCostFunctionPlus << endl;


            double fd_Plus = (perturbedCostFunctionPlus - notperturbedCostFunction) / paramstep;
            double fd_Minus = (notperturbedCostFunction - perturbedCostFunctionMinus) / paramstep;
            double fd_central = (perturbedCostFunctionPlus - perturbedCostFunctionMinus) / (2 * paramstep);

            cout << "AD GRAD=" << ad_grad << endl;
            cout << "FD Plus=" << fd_Plus << endl;
            cout << "FD Minus" << fd_Minus << endl;
            cout << "FD Central" << fd_central << endl;

            cout << "MaxPerturbation Plus=" << maxPerturbPlus << endl;
            cout << "MaxPerturbation Minus=" << maxPerturbPlus << endl;
            cout << "=================CAD index =" << paramindex <<" FINISHED ===========" << endl;
            //unperturbing again
            x[paramindex] += paramstep;
//            DesignStep(x);
//            CFDCostFunction();

        }
    }

    void CompareMgoptSurfaceSensitivityAD_FD(){

        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshes(meshTable, designTable);
//        //occt.UpdateMeshOnCAD();
//
        auto notperturbedCostFunction = CFDCostFunction();
        // auto notperturbedCostFunction= 101.59142606678731;
//
        mgopt.RunSolverByMode(3);//run adjoint
        mgopt.RunSolverByMode(4);//run adjoint
        mgopt.RunSolverByMode(4);//run adjoint
        mgopt.RunSolverByMode(4);//run adjoint
        mgopt.RunSolverByMode(4);//run adjoint

        mgopt.RunSolverByMode(6);//calculate flow sensitivities (Matrix 3*Meshpointisze)
//
//        auto cfdSensAD = mgopt.GetSensitivity();

        mgopt.BackupSolution("/optimisation/CFD","CompareMgopt4TimesAdjoint");

//       // petrub 1 meshpoint
//        vector<vector<double>> displacement(3,vector<double>(occt.meshpointsParametric.size()));
//        for (int i=0; i<occt.meshpointsParametric.size(); i++){
//            for (int j=0;j<3; j++){
//                displacement[j][i]=0.0;
//            }
//        }
//
//        cout << "meshpointParametric Size = " << occt.meshpointsParametric.size() << endl;
//
//       int  index100Meshpoint = 250;
//
////        for (int i=0;i<occt.meshpointsParametric.size();i++){
////             if (index100Meshpoint>210 && occt.isDesignPointByMeshIndex(i)){
////                index100Meshpoint = i;
////                break;
////            }
////            index100Meshpoint++;
////        }
//
//        index100Meshpoint--;
//         cout << "index100MeshPoint = " << index100Meshpoint << endl;
//        int meshIndexPerturbed = index100Meshpoint;
//        int Component = 1; //Perturbing Z component
//
//        cout << occt.meshpointsCartesian[meshIndexPerturbed].Coord(Component+1);
//        double step =10E-6;
//
//        displacement[Component][meshIndexPerturbed]=step;
//        mgopt.PopulateDisplacementsTable(displacement);
//
//        mgopt.RunSolverByMode(7);
//
//        //Dont forget to update your mesh!!!!
//        //occt.UpdateMeshFromPerturbed();
//
//        auto perturbedCostFunction = CFDCostFunction();
//
//
//
//            cout << occt.meshpointsCartesian[meshIndexPerturbed].Coord(Component+1);
//          //  cout <<"AD Sens = " << cfdSensAD[Component][meshIndexPerturbed] << endl;
//            cout <<"FD Sens = " << (perturbedCostFunction-notperturbedCostFunction)/step << endl;
//        cout <<"indexNumber = " << index100Meshpoint << endl;

    }

    void CompareMgoptVolumeSensitivityAD_FD(){
//        auto designTable = mgopt.GetDesignTable();
//        auto meshTable = mgopt.GetMeshTable();
//
//        occt.SetMeshes(meshTable, designTable);
//        DesignStepMeshToCAD();
////
//        auto notperturbedCostFunction = CFDCostFunction();
//        mgopt.RunSolverByMode(3);
//        mgopt.RunSolverByMode(4);
//        mgopt.RunSolverByMode(6);


        auto notperturbedCostFunction= 106.77437138623856;
//        vector<vector<double>> displacement(3,vector<double>(occt.meshpointsParametric.size()));
//        for (int i=0; i<occt.meshpointsParametric.size(); i++){
//            for (int j=0;j<3; j++){
//                displacement[j][i]=0.0;
//            }
//        }

//
//        int meshIndexPerturbed = 0;
//        int Component = 0; //Perturbing X component
//
//        cout << occt.meshpointsCartesian[meshIndexPerturbed].Coord(Component+1);

////
////        displacement[Component][meshIndexPerturbed]=step;
////        mgopt.PopulateDisplacementsTable(displacement);
//
////        mgopt.RunSolverByMode(7);
////
////        //Dont forget to update your mesh!!!!
////        //occt.UpdateMeshFromPerturbed();
////

        double step = -10E-7;
        auto perturbedCostFunction = CFDCostFunction();
////
////
////        //cout <<"AD Sens = " << cfdSensAD[Component][meshIndexPerturbed] << endl;
        cout <<"FD Sens = " << (perturbedCostFunction-notperturbedCostFunction)/step << endl;
////
        cout.precision(15);
        cout << "cost unpert = "  << notperturbedCostFunction << endl;
        cout << "cost perturbed = " << perturbedCostFunction << endl;

    }

    void DesignStepForwardTest(vector<int> paramindexList){

        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshes(meshTable, designTable);
        occt.UpdateMeshOnCAD();
        mgopt.BackupSolution("/optimisation/CFD","initial_Mesh");

        vector<Standard_Real> x = occt.designParameters;
        vector<Standard_Real> y = occt.designParameters;

        double paramstep = 0.012;
        for (int i = 0; i < paramindexList.size(); i++) {
                auto paramindex = paramindexList[i];
                cout << "designParam beginning = " << occt.designParameters[paramindex] << endl;
                x[paramindex] += paramstep;
        }

        DesignStepIterations(x,10, "u-bend-auto-deformations.json");
        mgopt.BackupSolution("/optimisation/CFD","iteration_Forward_Scale0");
//        DesignStepIterations(y,1, "u-bend-auto-deformations.json");
//        mgopt.BackupSolution("/optimisation/CFD","iteration_Back_Scale0");
//        for (int j = 0; j < 75; ++j) {
//
//
//            vector<Standard_Real> x = occt.designParameters;
//            for (int i = 0; i < paramindexList.size(); i++) {
//                auto paramindex = paramindexList[i];
//                cout << "designParam beginning = " << occt.designParameters[paramindex] << endl;
//                x[paramindex] += paramstep;
//
//            }
//
//            DesignStep(x);
//            mgopt.BackupSolution("/optimisation/CFD/MeshPerturbation", "iteration_DesignFront_45Scale_"+std::to_string(j));
//            mgopt.RunSolverByMode(1);
//            mgopt.RunSolverByMode(3);
//            mgopt.RunSolverByMode(6);

         }

    void DesignStepForwardAndBack(vector<int> paramindexList){

        auto designTable = mgopt.GetDesignTable();
        auto meshTable = mgopt.GetMeshTable();

        occt.SetMeshes(meshTable, designTable);
        occt.UpdateMeshOnCAD();
        mgopt.BackupSolution("/optimisation/CFD","initial_Mesh");
        string iterationStepFile = mgopt.CasePath +"/optimisation/CAD/Design_Changes_Initial.stp";
        occt.StepWriter(iterationStepFile.c_str());

        vector<Standard_Real> x = occt.designParameters;
        vector<Standard_Real> y = occt.designParameters;

        double paramstep = 0.02;
        for (int i = 0; i < paramindexList.size(); i++) {
            auto paramindex = paramindexList[i];
            cout << "designParam beginning = " << occt.designParameters[paramindex] << endl;
            x[paramindex] += paramstep;
        }

        DesignStep(x);
        mgopt.BackupSolution("/optimisation/CFD","iteration_DesignFront");
        iterationStepFile = mgopt.CasePath +"/optimisation/CAD/Design_Changes_Front.stp";
        occt.StepWriter(iterationStepFile.c_str());

        cout << "==================================================" << endl;
        cout << "==================================================" << endl;
        cout << "==================================================" << endl;

        mgopt.RunSolverByMode(1);
        mgopt.RunSolverByMode(3);
        mgopt.RunSolverByMode(6);


        DesignStep(y);
        mgopt.BackupSolution("/optimisation/CFD","iteration_DesignBack");
        iterationStepFile = mgopt.CasePath +"/optimisation/CAD/Design_Changes_Back.stp";
        occt.StepWriter(iterationStepFile.c_str());
    }


    void MoveToNewParams(){
        string params = "-0.0459286 0.0417994 -0.0519148 0.0471592 -0.0515549 0.04901 -0.0451757 0.0442648 -0.0189221 0.0436181 -0.0180698 0.0443564 -0.0179715 0.0453112 -0.0186847 0.0499872 0.018938 0.0436172 0.0180762 0.0443719 0.0179711 0.0453206 0.018688 0.049984 0.0459101 0.0417488 0.0519148 0.0469404 0.0515356 0.0489468 0.0451724 0.0442877 0.0626468 0.0172985 0.0652193 0.0182303 0.0534695 0.0188117 0.0421246 0.0191384 0.0625454 -0.0207856 0.0622501 -0.0193477 0.0561361 -0.0214838 0.0418081 -0.0196949 0.0335877 -0.0222383 0.042128 -0.0205401 0.0589435 -0.052151 0.0407269 -0.0437865 0.0221141 -0.0367257 0.0227053 -0.0359293 0.0203184 -0.0733225 0.019158 -0.0548483 -0.022116 -0.036724 -0.0227047 -0.0359266 -0.0203164 -0.0733153 -0.0191573 -0.0548484 -0.0335826 -0.022224 -0.0421219 -0.0205407 -0.0589454 -0.0521537 -0.0407222 -0.0437902 -0.0625546 -0.0207853 -0.0622536 -0.019347 -0.0561339 -0.0214862 -0.0418025 -0.0196955 -0.0626671 0.0172976 -0.0652263 0.0182301 -0.0534706 0.0188124 -0.0421252 0.0191384";
        istringstream iss(params);
        vector<string> tokens{istream_iterator<string>{iss},
                              istream_iterator<string>{}};

        auto design = vector<Standard_Real>(tokens.size());

        for (int i = 0; i< tokens.size();i++)
            design[i] = Standard_Real(stod(tokens[i]));

        DesignStep(design);

        string iterationStepFile = mgopt.CasePath +"/optimisation/CAD/initialDesign_InverseStart.stp";
        occt.StepWriter(iterationStepFile.c_str());
        mgopt.BackupSolution("/optimisation/CFD","iteration_00_InverseStart");

    }
private:
    //Testing



    //    void WriteCadSensitivity(int index);
    //    void WriteCadSensitivity(vector<int> indexes);
    //    void CompareCAD_AD_FD(int paramindex);
    //    void CompareFullLoopAD_FD(int paramindex);
    //    void CompareFullLoopAD_FD_Central(int paramindex);
    //    void CompareMgoptSurfaceSensitivityAD_FD();
    //    void CompareMgoptVolumeSensitivityAD_FD();

};


#endif //U_BENDOPTIMISATION_CADCFDOPTIMISATION_H


