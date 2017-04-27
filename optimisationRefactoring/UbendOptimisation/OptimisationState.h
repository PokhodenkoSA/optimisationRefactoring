//
// Created by orestmykhaskiv on 06/04/16.
//

#ifndef U_BENDOPTIMISATION_OPTIMISATIONSTATE_H
#define U_BENDOPTIMISATION_OPTIMISATIONSTATE_H
class OptimisationState{
public:
    int iteration;
    double CostFunction;
    double GradNorm;
    vector<double> Gradient;
    vector<int> EffectiveParametersIndexes;
    vector<Standard_Real> params;
    int CADSurfaces;
    string primalConvergence;
    string adjointConvergence;
    double MaxMeshPerturbation;
    double MaxParamPerturbation;
    double ArmijoStepsPerformed;
    double StepSize;

    string filename;
    OptimisationState(string par_filename){
        filename=par_filename;
    }

    OptimisationState(MgoptSolver mgopt, OCCTDataProvider occt, int k, double obj,vector<double> g,vector<Standard_Real> x ){
        filename = mgopt.CasePath;
        iteration = k;
        CADSurfaces = occt.facemap.Extent();
        CostFunction = obj;
        GradNorm = norm(g);
        params = x;
        primalConvergence = mgopt.GetPrimalCovergence();
        adjointConvergence = mgopt.GetAdjointConvergence();
        Gradient = g;
        EffectiveParametersIndexes = maxNElementsAbsVector(5, g);
        ArmijoStepsPerformed = 0;
        StepSize = 0;
    }

    // Insertion operator
    std::ostream& operator<<( std::ostream& os)
    {
        // write out individual members of s with an end of line between each one
        os << "Iteration "<< iteration << '\n';
        os << "Params: ";
        for (int i=0;i<params.size();i++)
            os << params[i] << " | ";

        os << endl;
        os << "Cost Function =" << CostFunction << endl;
        os << "Grad Norm =" << GradNorm <<endl;
        os << "Number of Surfaces=" << CADSurfaces << endl;
        os << "Primal:" << endl << primalConvergence <<endl;
        os << "Adjoint:" << endl << adjointConvergence << endl;
        os << "===========================================================";

        return os;
    }



    void WriteState(){

        string myfilename= filename + fileOptimisationIterationVerbose;
        std::ofstream os;
        os.open(myfilename,ios::app);
        os.precision(10);
        // write out individual members of s with an end of line between each one

        os << "Iteration "<< iteration << '\n';
        os << "Params: ";
        for (int i=0;i<params.size();i++)
            os << params[i].getValue() << " | ";

        os << endl;

        os << "Cost Function =" << CostFunction << endl;
        os << "Grad Norm =" << GradNorm <<endl;

				os.precision(14);
        if (!Gradient.empty()){
            os << "Gradient:";
            for (int i=0;i<Gradient.size();i++)
                os << i <<" => "<< Gradient[i] << " | ";
               os<< endl;
        }

        if (!EffectiveParametersIndexes.empty()){
            os << "Effective params:";
            for (int i=0;i<EffectiveParametersIndexes.size();i++)
                os << EffectiveParametersIndexes[i] << " | ";
                os<< endl;

        }

        if (MaxMeshPerturbation > 0)
            os << "MaxMeshPerturbation: " << MaxMeshPerturbation << endl;

        os << "Number of ArmijoSteps ="<< ArmijoStepsPerformed << endl;
        os << "ScaleFactor = " << StepSize << endl;
        os << "Number of Surfaces=" << CADSurfaces << endl;
        os << "Primal:" << endl << primalConvergence <<endl;
        os << "Adjoint:" << endl << adjointConvergence << endl;
        os << "===========================================================" << endl;
        os.close();

    }
    void WriteParamsData(bool withHeader = false){
        string myfilename= filename + fileOptimisationDataParams;
        std::ofstream os;
        os.open(myfilename,ios::app);
        // write out individual members of s with an end of line between each one

        if (withHeader)
        os << "# Iteration     Params x "<< Gradient.size() << endl;

        os << iteration << " " ;
        for (int i=0;i<params.size();i++)
            os << params[i].getValue() << " ";

        os << endl;
        os.close();
    }
    
    void WriteCostData(bool withHeader = false){

        string myfilename= filename + fileOptimisationDataCost;
        std::ofstream os;
        os.open(myfilename,ios::app);
        // write out individual members of s with an end of line between each one

        if (withHeader)
            os << "# Iteration   CostFunction  GradNorm MaxParamDeformation MaxMeshDeformation ArmijoStepsPerformed StepFactor EffectiveParamIndexes x"<< EffectiveParametersIndexes.size() << endl;

        os << iteration << " " ;
        int defaultPrecision = os.precision(14);
        os << CostFunction << " ";
        os.precision(defaultPrecision);
        os << GradNorm << " " << MaxParamPerturbation << " "<< MaxMeshPerturbation << " " << " " << ArmijoStepsPerformed <<  " " << StepSize << " ";

            for (int i=0;i<EffectiveParametersIndexes.size();i++)
                os << EffectiveParametersIndexes[i] << " ";

        os << endl;
        os.close();

    }


private:
    string fileOptimisationIterationVerbose ="/optimisation/optimisationIterations.txt";
    string fileOptimisationDataParams ="/optimisation/optimisationParams.cnv";
    string fileOptimisationDataCost ="/optimisation/optimisationCost.cnv";

};
#endif //U_BENDOPTIMISATION_OPTIMISATIONSTATE_H
