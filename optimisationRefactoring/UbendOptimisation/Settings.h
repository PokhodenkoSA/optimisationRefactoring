//
// Created by orestmykhaskiv on 03/09/16.
//

#ifndef U_BENDOPTIMISATION_SETTINGS_H
#define U_BENDOPTIMISATION_SETTINGS_H

#include <Standard_CString.hxx>

#define REVERSE_MODE 0

using namespace std;


///!!! Setting Halfcylinder NSPCC
class Settings_Halfcylinder{
public:
   Settings_Halfcylinder(){};
    Standard_CString nspcc__cadfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/halfcylinder_sewed.stp";
    Standard_CString nspcc__cadfile_updated = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/halfcylinder_sewed_CPMovements.stp";

   // Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/halfcylinder_sewed_surf.dat";
    Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/halfcylinder_coarse_mesh.dat";

    Standard_CString nspcc__constraintfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/halfcylinder_constraint.dat";
    //Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/surf_coords_halfcylinder.dat";
    Standard_CString nspcc__projections_meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/surf_coords_halfcylinder_projections.xdmf.h5";
    Standard_CString nspcc__projections_meshfile2 = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/surf_coords_halfcylinder_projections2_coarse.xdmf.h5";
    vector<int> nspcc_designFaces = {1,2};

    string nspcc_dir ="/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/";
    Standard_CString gradientMatrices = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/halfcylinder/gradientMatrices_vector.xdmf.h5";
};

//!!!! Settings Stator NSPCC
class Settings_Stator{
public:
    Settings_Stator(){};
    Standard_CString nspcc__cadfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/baseline_stator_NX.stp";
    Standard_CString nspcc__cadfile_sewed = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/baseline_stator_NX_sewed.stp";
    Standard_CString nspcc__constraintfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/stator_constraint.dat";
//    Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/stator_coarse_mesh.dat";
//    Standard_CString nspcc__projections_meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/surf_coords_stator_projections.xdmf.h5";

    Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/stator_fine_mesh.dat";
    Standard_CString nspcc__projections_meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/fine_surf_coords_stator_projections.xdmf.h5";

    Standard_CString nspcc__cadfile_updated = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/stator_sewed_CPMovements";

    string nspcc_dir = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/";

    vector<int> nspcc_designFaces = {1,2,3,4};
    Standard_CString cfdSensitivityFile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/base_sens_vector_ploss.dat";
    Standard_CString gradientMatrices =  "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/gradientMatrices.xdmf.h5";
    string updateMeshFile =  "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/stator_ilias/perturbedMesh.dat";
};







//!!!! Settings UBEND ///ECCOMAS
class Settings_Ubend_Parametric{
public:
    Settings_Ubend_Parametric(){};
    //New Json Filee!!!!
    string mgoptFolder = "/home/salvo/IODA/refactoringOptimisation_compiled/CFD/mgopt/bin";
    // //string mgoptCase="/home/orestmykhaskiv/QMUL/mgoptCustomCases/ubend12_10_fixelegs";
    //  string mgoptCase="/home/orestmykhaskiv/QMUL/mgoptCustomCases/05_04_16_ubend_Mateusz";
    // string mgoptCase ="/home/orestmykhaskiv/QMUL/mgoptCustomCases/05_18_16_SmallUbend_WithLegs";
    string mgoptCase ="/home/salvo/IODA/refactoringOptimisation_compiled/testCase_oxford";
    //  string mgoptCase="/home/orestmykhaskiv/QMUL/mgoptCustomCases/Scale15tarbal";

    // string mgoptCase="/home/orestmykhaskiv/QMUL/mgoptCustomCases/ubend12_10Scale8";
   // string mgoptJsonFile="u-bend-auto-old-Single.json";
    string mgoptJsonFile="u-bend-Mladen-initial.json";
    string ExternalLibraryFolder = "/home/salvo/IODA/refactoringOptimisation_compiled";
    //string RestartSettingsFile = "u-bend-auto-old-Single-Restart.json";
    string RestartSettingsFile = "u-bend-Mladen-initial.json";
    string projectionFile = "/home/salvo/IODA/refactoringOptimisation_compiled/testCase_oxford";
};

//!!!! Settings TUB Parametric
class Settings_TUB_Parametric{
public:
    Settings_TUB_Parametric(){};
    string mgoptFolder = "/home/salvo/IODA/refactoringOptimisation_compiled/CFD/mgopt/bin";
    string mgoptCase ="/home/salvo/IODA/refactoringOptimisation_compiled/testCase_TUB";
    string mgoptJsonFile="TUB.json";
    string ExternalLibraryFolder = "/home/salvo/IODA/refactoringOptimisation_compiled";
    string RestartSettingsFile = "TUB.json";
    string projectionFile = "/home/salvo/IODA/refactoringOptimisation_compiled/testCase_TUB";
};


class Settings_Ubend_NSPCC {
public:
    Settings_Ubend_NSPCC(){};
    //CAD PART
    Standard_CString nspcc__cadfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/U-bend/ubend_from_parametric.stp";
    Standard_CString nspcc__cadfile_updated = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/U-bend/ubend_from_parametric_CPMovements.stp";

    Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/U-bend/meshWithIndex.dat";
    Standard_CString nspcc__projections_meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/U-bend/projected20/solution.xdmf_field.h5";

    vector<int> nspcc_designFaces = {1,2,3,4};

    Standard_CString nspcc__constraintfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/wingfaces/2faces_constraint.dat";
    vector<int> sensitivityKnotIndexes = {/*15350*/33187};




    //CFD PART
    string mgoptFolder = "/home/orestmykhaskiv/QMUL/mgopt/bin";
    string mgoptCase ="/home/orestmykhaskiv/QMUL/mgoptCustomCases/06_28_16_UBend_ONEMoving";
    string mgoptJsonFile="ubend-Rejish.json";
    string ExternalLibraryFolder = "/home/orestmykhaskiv";
    //string RestartSettingsFile = "u-bend-auto-old-Single-Restart.json";
    string RestartSettingsFile = "ubend-Rejish.json";
    string projectionFile = "/home/orestmykhaskiv/QMUL/mgoptCustomCases/06_28_16_UBend_ONEMoving/readyRejishSettings/projected20Rejish";
};



class Settings_Optimisation_UbendX_NSPCC {
public:
    Settings_Optimisation_UbendX_NSPCC(){};
    //CAD PART
    Standard_CString nspcc__cadfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/ubend-nspcc-opt/ubend-x-sew.stp";
    Standard_CString nspcc__cadfile_updated = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/ubend-nspcc-opt/ubend-x-sew-updated.stp";

 //   Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/U-bend/meshWithIndex.dat";
    Standard_CString nspcc__projections_meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/ubend-nspcc-opt/forMgopt/initialSolution_Projected/solution.xdmf_field.h5";
    vector<int> nspcc_designFaces = {1,2,3,4};

    Standard_CString nspcc__constraintfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/ubend-nspcc-opt/ubend-constraint.dat";
  //  vector<int> sensitivityKnotIndexes = {/*15350*/33187};




    //CFD PART
    string mgoptFolder = "/home/orestmykhaskiv/QMUL/mgopt/bin";
    string mgoptCase ="/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/ubend-nspcc-opt/forMgopt";
    string mgoptJsonFile="ubend.json";
    string ExternalLibraryFolder = "/home/orestmykhaskiv";
    //string RestartSettingsFile = "u-bend-auto-old-Single-Restart.json";
    string RestartSettingsFile = "ubend.json";
    string projectionFile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/ubend-nspcc-opt/forMgopt/initialSolution_Projected/solution.xdmf_field.h5";
    string nspcc_dir_faces = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/ubend-nspcc-opt/forMgopt/optimisation/CAD/faces/";
};


class Settings_Optimisation_TUB_NSPCC {
public:
    Settings_Optimisation_TUB_NSPCC(){};
    //CAD PART
    Standard_CString nspcc__cadfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/tuberlin-nspcc-opt/TUBstatorMladenDP_knotted.stp";
    Standard_CString nspcc__cadfile_updated = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/tuberlin-nspcc-opt/TUBstatorMladenDP_knotted_Imroved2.stp";

 //   Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/U-bend/meshWithIndex.dat";
    Standard_CString nspcc__projections_meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/tuberlin-nspcc-opt/forMgopt/initialSolution_Projected/solution.xdmf_field.h5";
    vector<int> nspcc_designFaces = {1,2};

    Standard_CString nspcc__constraintfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/tuberlin-nspcc-opt/tuberlin-constraint.dat";
    vector<int> sensitivityKnotIndexes = {4728,4729,4799,4874,5466,5467,7753,7759};




    //CFD PART
    string mgoptFolder = "/home/orestmykhaskiv/QMUL/mgopt/bin";
    string mgoptCase ="/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/tuberlin-nspcc-opt/forMgopt";
    string mgoptJsonFile="TUB.json";
    string ExternalLibraryFolder = "/home/orestmykhaskiv";
    //string RestartSettingsFile = "u-bend-auto-oldTUB-Single-Restart.json";
    string RestartSettingsFile = "TUB.json";
    string projectionFile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/tuberlin-nspcc-opt/forMgopt/initialSolution_Projected/solution.xdmf_field.h5";
    string nspcc_dir_faces = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/tuberlin-nspcc-opt/forMgopt/optimisation/CAD/faces/";
};



///!!! Settings Wingfaces NSPCC
class Settings_WingFaces{
public:
    Settings_WingFaces(){};
    Standard_CString nspcc__cadfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/wingfaces/2faces.stp";
    Standard_CString nspcc__cadfile_sewed = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/wingfaces/2faces.stp";
    Standard_CString nspcc__constraintfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/wingfaces/2faces_constraint.dat";
    Standard_CString nspcc__meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/wingfaces/2faces_surface_mesh.dat";
    Standard_CString nspcc__projections_meshfile = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/wingfaces/projection.xdmf.h5";

    Standard_CString nspcc__cadfile_updated = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/wingfaces/2faces_CPMovements.stp";

    string nspcc_dir = "/home/orestmykhaskiv/QMUL/programs/NSPCC_testcases/wingfaces/";

    vector<int> nspcc_designFaces = {1,2};
};

#endif //U_BENDOPTIMISATION_SETTINGS_H
