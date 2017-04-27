//#include "CADCFDOptimisation.h"
//
//void CADCFDOptimisation::WriteCadSensitivity(int index){
//
//    auto designTable = mgopt.GetDesignTable();
//    auto meshTable = mgopt.GetMeshTable();
//
//
//    occt.SetMeshes(meshTable, designTable);
//
//    auto table = occt.GetDesignSensitivity(index);
//
//    string name = "CAD_Sensitivity_"+to_string(index);
//    mgopt.PopulateTableByName(table,name);
//}
//
//void CADCFDOptimisation::WriteCadSensitivity(vector<int> indexes){
//    auto designTable = mgopt.GetDesignTable();
//    auto meshTable = mgopt.GetMeshTable();
//
//
//    occt.SetMeshes(meshTable, designTable);
//    for (int i=0;i<indexes.size(); i++){
//        auto index = indexes[i];
//        auto table = occt.GetDesignSensitivity(index);
//        string name = "CAD_Sensitivity_"+to_string(index);
//
//        cout << name << endl;
//        cout << "norm X=" << norm(table[0]) <<endl;
//        cout << "norm Y=" << norm(table[1]) <<endl;
//        cout << "norm Z=" << norm(table[2]) <<endl;
//        // mgopt.PopulateTableByName(table,name);
//    }
//}
//
//void CADCFDOptimisation::CompareCAD_AD_FD(int paramindex){
//    auto designTable = mgopt.GetDesignTable();
//    auto meshTable = mgopt.GetMeshTable();
//
//    occt.SetMeshes(meshTable, designTable);
//    occt.UpdateMeshOnCAD();
//
//    auto matrixAD = occt.GetDesignSensitivity(paramindex);
//    cout << "Matrix AD Succeeded" << endl;
//    auto matrixFD = occt.GetDesignSensitivityFD(paramindex);
//    cout << "Matrix FD Succeeded" << endl;
//
//    int indexNotEqual = 0;
//    int indexNotZero = 0;
//
//    for (int i = 0; i<matrixAD[0].size();i++){
//        for (int j = 0;j<3;j++){
//            if (abs(matrixAD[j][i]-matrixFD[j][i]) >10E-8)
//                indexNotEqual++;
//            if (abs(matrixAD[j][i])>0.0001)
//                indexNotZero++;
//            //  cout << i << " " << j << endl;
//        }
//
//    }
//    cout << "Out of Matrix " << endl;
//
//    cout << indexNotEqual << endl;
//    cout << indexNotZero << endl;
//}
//
//void CADCFDOptimisation::CompareFullLoopAD_FD(int paramindex){
//
//    auto designTable = mgopt.GetDesignTable();
//    auto meshTable = mgopt.GetMeshTable();
//
//    occt.SetMeshes(meshTable, designTable);
//    occt.UpdateMeshOnCAD();
//
//    auto notperturbedCostFunction = CFDCostFunction();
////        auto g = CFDCADGradient();
////
////        double ad_grad = g[paramindex];
//
//
//    // Finding FD full CFD CAD value
//    double paramstep = 4*10E-9;
//
//    vector<Standard_Real> x = occt.designParameters;
//    x[paramindex] +=paramstep;
//
//    double maxPerturb = DesignStep(x);
//
//    auto perturbedCostFunction = CFDCostFunction();
//    double fd_grad = (perturbedCostFunction - notperturbedCostFunction)/paramstep;
//
//    cout.precision(15);
//    cout << "cost unpert = "  << notperturbedCostFunction << endl;
//    cout << "cost perturbed = " << perturbedCostFunction << endl;
//
//    // cout << "AD GRAD=" << ad_grad << endl;
//    cout << "FD GRAD=" << fd_grad << endl;
//
//    cout << "MaxPerturbation=" << maxPerturb << endl;
//}
//
//void CADCFDOptimisation::CompareFullLoopAD_FD_Central(int paramindex){
//
//    auto designTable = mgopt.GetDesignTable();
//    auto meshTable = mgopt.GetMeshTable();
//
//    occt.SetMeshes(meshTable, designTable);
//    occt.UpdateMeshOnCAD();
//
//    auto notperturbedCostFunction = CFDCostFunction();
//    auto g = CFDCADGradient();
//
//    double ad_grad = g[paramindex];
//
//
//    // Finding FD full CFD CAD value
//    double paramstep = 3*10E-7;
//
//    vector<Standard_Real> x = occt.designParameters;
//    x[paramindex] +=paramstep;
//
//    double maxPerturbPlus = DesignStep(x);
//
//    auto perturbedCostFunctionPlus = CFDCostFunction();
//
//
//    x[paramindex] -=2*paramstep;
//    double maxPerturbMinus = DesignStep(x);
//
//    auto perturbedCostFunctionMinus = CFDCostFunction();
//
//    cout.precision(15);
//    cout << "cost unpert = "  << notperturbedCostFunction << endl;
//    cout << "cost perturbed Plus = " << perturbedCostFunctionPlus << endl;
//    cout << "cost perturbed Minus = " << perturbedCostFunctionPlus << endl;
//
//
//    double fd_Plus = (perturbedCostFunctionPlus-notperturbedCostFunction)/paramstep;
//    double fd_Minus = (notperturbedCostFunction - perturbedCostFunctionMinus)/paramstep;
//    double fd_central = (perturbedCostFunctionPlus-perturbedCostFunctionMinus)/(2*paramstep);
//
//    cout << "AD GRAD=" << ad_grad << endl;
//    cout << "FD Plus=" << fd_Plus << endl;
//    cout << "FD Minus" << fd_Minus << endl;
//    cout << "FD Central" << fd_central << endl;
//
//    cout << "MaxPerturbation Plus=" << maxPerturbPlus << endl;
//    cout << "MaxPerturbation Minus=" << maxPerturbPlus << endl;
//}
//
//void CADCFDOptimisation::CompareMgoptSurfaceSensitivityAD_FD(){
//
//    auto designTable = mgopt.GetDesignTable();
//    auto meshTable = mgopt.GetMeshTable();
//
//    occt.SetMeshes(meshTable, designTable);
////        //occt.UpdateMeshOnCAD();
////
//    auto notperturbedCostFunction = CFDCostFunction();
//    // auto notperturbedCostFunction= 101.59142606678731;
////
//    mgopt.RunSolverByMode(3);//run adjoint
//    mgopt.RunSolverByMode(4);//run adjoint
//    mgopt.RunSolverByMode(4);//run adjoint
//    mgopt.RunSolverByMode(4);//run adjoint
//    mgopt.RunSolverByMode(4);//run adjoint
//
//    mgopt.RunSolverByMode(6);//calculate flow sensitivities (Matrix 3*Meshpointisze)
////
////        auto cfdSensAD = mgopt.GetSensitivity();
//
//    mgopt.BackupSolution("/optimisation/CFD","CompareMgopt4TimesAdjoint");
//
////       // petrub 1 meshpoint
////        vector<vector<double>> displacement(3,vector<double>(occt.meshpointsParametric.size()));
////        for (int i=0; i<occt.meshpointsParametric.size(); i++){
////            for (int j=0;j<3; j++){
////                displacement[j][i]=0.0;
////            }
////        }
////
////        cout << "meshpointParametric Size = " << occt.meshpointsParametric.size() << endl;
////
////       int  index100Meshpoint = 250;
////
//////        for (int i=0;i<occt.meshpointsParametric.size();i++){
//////             if (index100Meshpoint>210 && occt.isDesignPointByMeshIndex(i)){
//////                index100Meshpoint = i;
//////                break;
//////            }
//////            index100Meshpoint++;
//////        }
////
////        index100Meshpoint--;
////         cout << "index100MeshPoint = " << index100Meshpoint << endl;
////        int meshIndexPerturbed = index100Meshpoint;
////        int Component = 1; //Perturbing Z component
////
////        cout << occt.meshpointsCartesian[meshIndexPerturbed].Coord(Component+1);
////        double step =10E-6;
////
////        displacement[Component][meshIndexPerturbed]=step;
////        mgopt.PopulateDisplacementsTable(displacement);
////
////        mgopt.RunSolverByMode(7);
////
////        //Dont forget to update your mesh!!!!
////        //occt.UpdateMeshFromPerturbed();
////
////        auto perturbedCostFunction = CFDCostFunction();
////
////
////
////            cout << occt.meshpointsCartesian[meshIndexPerturbed].Coord(Component+1);
////          //  cout <<"AD Sens = " << cfdSensAD[Component][meshIndexPerturbed] << endl;
////            cout <<"FD Sens = " << (perturbedCostFunction-notperturbedCostFunction)/step << endl;
////        cout <<"indexNumber = " << index100Meshpoint << endl;
//
//}
//
//void CADCFDOptimisation::CompareMgoptVolumeSensitivityAD_FD(){
////        auto designTable = mgopt.GetDesignTable();
////        auto meshTable = mgopt.GetMeshTable();
////
////        occt.SetMeshes(meshTable, designTable);
////        DesignStepMeshToCAD();
//////
////        auto notperturbedCostFunction = CFDCostFunction();
////        mgopt.RunSolverByMode(3);
////        mgopt.RunSolverByMode(4);
////        mgopt.RunSolverByMode(6);
//
//
//    auto notperturbedCostFunction= 106.77437138623856;
////        vector<vector<double>> displacement(3,vector<double>(occt.meshpointsParametric.size()));
////        for (int i=0; i<occt.meshpointsParametric.size(); i++){
////            for (int j=0;j<3; j++){
////                displacement[j][i]=0.0;
////            }
////        }
//
////
////        int meshIndexPerturbed = 0;
////        int Component = 0; //Perturbing X component
////
////        cout << occt.meshpointsCartesian[meshIndexPerturbed].Coord(Component+1);
//
//////
//////        displacement[Component][meshIndexPerturbed]=step;
//////        mgopt.PopulateDisplacementsTable(displacement);
////
//////        mgopt.RunSolverByMode(7);
//////
//////        //Dont forget to update your mesh!!!!
//////        //occt.UpdateMeshFromPerturbed();
//////
//
//    double step = -10E-7;
//    auto perturbedCostFunction = CFDCostFunction();
//////
//////
//////        //cout <<"AD Sens = " << cfdSensAD[Component][meshIndexPerturbed] << endl;
//    cout <<"FD Sens = " << (perturbedCostFunction-notperturbedCostFunction)/step << endl;
//////
//    cout.precision(15);
//    cout << "cost unpert = "  << notperturbedCostFunction << endl;
//    cout << "cost perturbed = " << perturbedCostFunction << endl;
//
//}void CADCFDOptimisation::MoveToNewParams(::CADCFDOptimisation::string basic_string){

//}
