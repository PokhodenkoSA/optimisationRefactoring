//
// Created by orestmykhaskiv on 03/02/16.
//

#ifndef U_BENDOPTIMISATION_MGOPTSOLVER_H
#define U_BENDOPTIMISATION_MGOPTSOLVER_H

#include <string>
using namespace std;

#include <iostream>
#include <fstream>
#include <limits>
#include <string>
#include <sstream>
#include <stdlib.h>
#include "hdmf5IO.h"

std::istream& ignoreline(std::ifstream& in, std::ifstream::pos_type& pos)
{
    pos = in.tellg();
    return in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

std::string getLastLine(std::ifstream& in)
{
    std::ifstream::pos_type pos = in.tellg();

    std::ifstream::pos_type lastPos;
    while (in >> std::ws && ignoreline(in, lastPos))
        pos = lastPos;

    in.clear();
    in.seekg(pos);

    std::string line;
    std::getline(in, line);
    return line;
}

string getSecondLine(std::ifstream &infile){

    string sLine;

    getline(infile, sLine);//dummy first line
    getline(infile, sLine);

    infile.close();
    return sLine;


}

vector<string> GetLastIterationInfo(string filename)
{
    vector<string> iterationInfo(18);
    std::ifstream file(filename);

    if (file)
    {
        std::string line = getLastLine(file);
        int k=0;
        for (int i=0; i<line.length();i++){
            if (line[i]!=' '){
                iterationInfo[k]+=line[i];
                if(line[i+1]==' ')
                    k++;
            }

        }
    }
    else {
        //std::cout << "Error in parsing file" << file;
    }
    return iterationInfo;
}

string GetLastLine(string filename){
    std::ifstream file(filename);
    if (file)
    {
       return getLastLine(file);
    }
    return "";
}

string GetSecondLine(string filename){
    std::ifstream file(filename);
    if (file){
        return getSecondLine(file);
    }
    return "";
}



class MgoptSolver{
public:
    string SolverPath;
    string CasePath;
    string SettingsFile;
    string RestartSettingsFile;
    string ExternalLibraryFolder;

    string SolutionMeshFile = "/solution.xdmf_mesh.h5";
    string SolutionFieldFile = "/solution.xdmf_field.h5";

    MgoptSolver(string solverPath, string casePath, string settingsFile, string externalLibrary){
        SolverPath=solverPath;
        CasePath=casePath;
        SettingsFile = settingsFile;
        ExternalLibraryFolder = externalLibrary;

    }
    MgoptSolver(){

    }

    void RunSolverByMode(int mode, string settingsFile = ""){

        string command = (string)"export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"+ ExternalLibraryFolder + "/hdf5-gnu/lib:" + ExternalLibraryFolder +"/hdf5-intel/lib" +" &&";
        command  +=" cd "+CasePath +" && ";

        command += SolverPath +"/mgopt";
        if (settingsFile=="")
            settingsFile=SettingsFile;
        command +=" "+settingsFile;
        command +=" "+std::to_string(mode);

        cout <<command<<endl;

        system(command.c_str());
    }

    double GetCostFunctionValue(){
       auto iterationInfo = GetLastIterationInfo(CasePath + "/primal.cnv");
       double costFunction = stod(iterationInfo[13]);
       return costFunction;
    }

    double GetXMomentumResidual(){
        auto iterationInfo = GetLastIterationInfo(CasePath + "/primal.cnv");
        double costFunction = stod(iterationInfo[3]);
        return costFunction;
    }

    double GetXAdjointMomentumResidual(){
        auto iterationInfo = GetLastIterationInfo(CasePath + "/adjoint.cnv");
        double costFunction = stod(iterationInfo[3]);
        return costFunction;
    }

    string GetPrimalCovergence(){
        string first = GetSecondLine(CasePath + "/primal.cnv");
        string last = GetLastLine(CasePath + "/primal.cnv");
        string spaces11 = "           ";
        return first+"\n" +spaces11+last;
    }

    string GetAdjointConvergence(){
        string first = GetSecondLine(CasePath + "/adjoint.cnv");
        string last = GetLastLine(CasePath + "/adjoint.cnv");
        string spaces11 = "           ";
        return first+"\n" +spaces11+last;
    }

    vector<vector<int>> GetDesignTable(){
        string hdmffile = CasePath +SolutionFieldFile;
       auto table = getTable<int>(hdmffile.c_str(),"/freeVariableMarker");
        table = GetHd5TransposeGroupByRow<int>(table);
       return table;

    }

    vector<vector<double>> GetSensitivity(){
        string hdmffile = CasePath +SolutionFieldFile;
        auto table = getTable<double>(hdmffile.c_str(),"/sensitivity");
        table = GetHd5TransposeGroupByRow<double>(table);
        return table;

    }

    vector<vector<double>> GetMeshTable(){
        string hdmffile = CasePath +SolutionMeshFile;
        auto table = getTable<double>(hdmffile.c_str(),"/XYZ");
         //return table;
        return GetHd5TransposeGroupByRow<double>(table);


    }

    void PopulateDisplacementsTable(vector<vector<double>> displacements, vector<vector<int>> isDesignVariable){
        //nulify non-design variables
        for (int i=0;i<isDesignVariable.size();i++){
            for (int j=0;j<isDesignVariable[0].size();j++){
                if (isDesignVariable[i][j]!=1)
                    displacements[i][j] = 0;
            }
        }

        string hdmffile = CasePath + SolutionFieldFile;
        displacements=SetHd5TransposeGroupByRow<double>(displacements);
        int status = CreateAndWriteTable(displacements, hdmffile.c_str(), "/dX");
    }

    void PopulateDisplacementsTable(vector<vector<double>> displacements){

        string hdmffile = CasePath + SolutionFieldFile;
        displacements = SetHd5TransposeGroupByRow<double>(displacements);

        int status = CreateAndWriteTable(displacements, hdmffile.c_str(), "/dX");
    }

    void PopulateTableByName(vector<vector<double>> table, string name){

        string hdmffile = CasePath + SolutionFieldFile;
        table = SetHd5TransposeGroupByRow<double>(table);

        int status = CreateAndWriteTable(table, hdmffile.c_str(), name.c_str());
    }

    template<typename T>
    vector<vector<T>> GetTableByName(string name){
        string hdmffile = CasePath + SolutionFieldFile;
        auto table = getTable<T>(hdmffile.c_str(),name.c_str());
        table = GetHd5TransposeGroupByRow<T>(table);
        return table;
    }

    template<typename T>
    vector<T> GetVectorByName(string name){
        string hdmffile = CasePath + SolutionFieldFile;
        auto table = getVector<T>(hdmffile.c_str(),name.c_str());
        return table;
    }

    template<typename T>
    void PopulateVectorByName(vector<T> table, string name){

        string hdmffile = CasePath + SolutionFieldFile;
        int status = CreateAndWriteVector(table, hdmffile.c_str(), name.c_str());
    }

    void hd5TestTranspose(){
        vector<vector<double>> original(3,vector<double>(10));
        for (int i=0;i<3;i++) {
            int j = 0;
            while (j < 10) {
                original[i][j]=10*i+j;
                j++;
            }
        }

        for (int i=0;i<3;i++)
        {
            for (int j=0;j<10;j++){
                cout << original[i][j]<< " ";
            }
            cout << endl;
        }

        auto neworiginal = GetHd5TransposeGroupByRow<double>(original);
        cout << "Transposed original" << endl;
        for (int i=0;i<3;i++)
        {
            for (int j=0;j<10;j++){
                cout << neworiginal[i][j]<< " ";
            }
            cout << endl;
        }

        auto oldoriginal = SetHd5TransposeGroupByRow<double>(neworiginal);
        cout << "Reverse Transposed original" << endl;
        for (int i=0;i<3;i++)
        {
            for (int j=0;j<10;j++){
                cout << oldoriginal[i][j]<< " ";
            }
            cout << endl;
        }
    }

    void BackupSolution(string folderPath, string folderName){

        string filesToCopy="adjoint.cnv primal.cnv solution.xdmf solution.xdmf_mesh.h5 solution.xdmf_field.h5 ubend-cyz.1.msh ubend-cyz.2.msh ubend-test.0.msh ubend-test.1.msh ubend.1.msh ubend.2.msh orest-stator.0.msh sbend_C.0.msh";
        //  string filesToCopy = "adjoint.cnv";

        string gotoCase = " cd "+CasePath + folderPath;
        string command  = gotoCase +" && mkdir "+ folderName;
        cout << command << endl;
        system(command.c_str());

        command = " cd "+CasePath;
        string destination = CasePath +folderPath +"/"+folderName;
        command += " && cp " + filesToCopy +" "+destination;
        cout << command << endl;
        system(command.c_str());

    }



private:


    template<typename T>
    vector<vector<T>> GetHd5TransposeGroupByRow(vector<vector<T>> original){
        int nRows = original.size();
        int nColumns = original[0].size();

        vector<vector<T>> reordered(nRows,vector<T>(nColumns));
        int currentCoord = 0;
        int currentIndex  = 0;
        for (int i=0;i<nRows;i++) {
            for (int j = 0; j < nColumns; j++) {
                reordered[currentCoord][currentIndex] = original[i][j];
                currentCoord++;
                if (currentCoord==nRows){
                    currentCoord=0;
                    currentIndex++;
                }
            }
        }
        return reordered;
        }

    template<typename T>
    vector<vector<T>> SetHd5TransposeGroupByRow(vector<vector<T>> original){
        int nRows = original.size();
        int nColumns = original[0].size();

        vector<vector<T>> reordered(nRows,vector<T>(nColumns));
        int currentCoord = 0;
        int currentIndex  = 0;
        for (int i=0;i<nRows;i++) {
            for (int j = 0; j < nColumns; j++) {
                reordered[i][j] = original[currentCoord][currentIndex];
                currentCoord++;
                if (currentCoord==nRows){
                    currentCoord=0;
                    currentIndex++;
                }
            }
        }
        return reordered;
    }






};
#endif //U_BENDOPTIMISATION_MGOPTSOLVER_H
