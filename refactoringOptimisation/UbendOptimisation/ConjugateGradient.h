//
// Created by orestmykhaskiv on 01/02/16.
//

#ifndef U_BENDOPTIMISATION_CONJUGATEGRADIENT_H
#define U_BENDOPTIMISATION_CONJUGATEGRADIENT_H

#include <stdio.h>
#include "math.h"


#include <iostream>
#include <vector>
#include <Standard_TypeDef.hxx>
#include <bits/stl_algo.h>
#include <queue>
#include "Settings.h"

using namespace std;
static double norm(vector<double> x);


static double func(double* x) {
    return 100 * pow(x[1] *x[1] - x[2],2)  + pow((x[1] - 1) , 2);
}



static double vectorScalarMultiplication(double *x, double *y){
    unsigned int Array_Length( sizeof( x ) / sizeof( x[ 0 ] ) );
    double sum=0;
    for (int i=0; i<Array_Length;i++)
        sum+=x[i]*y[i];

}

static double* vectorSum(double *x, double *y){
    unsigned int Array_Length( sizeof( x ) / sizeof( x[ 0 ] ) );
    double sum[Array_Length];
    for (int i=0; i<Array_Length;i++)
        sum[i]=x[i]+y[i];
}

static double* scaleVector(double scaleFactor, double*x){
    unsigned int Array_Length( sizeof( x ) / sizeof( x[ 0 ] ) );
    double sum[Array_Length];
    for (int i=0; i<Array_Length;i++)
        sum[i]=x[i]*scaleFactor;
}

static double norm(double *x){
    unsigned int Array_Length( sizeof( x ) / sizeof( x[ 0 ] ) );
    double sum;
    for (int i=0; i<Array_Length;i++)
        sum+=x[i]*x[i];
    return sqrt(sum);
}

double maxDeformation(vector<vector<double>> SurfaceMesh){
    double maxdeform = 0;
    int meshSize = SurfaceMesh[0].size();
    for (int i=0;i<meshSize;i++){
        vector<double> columm = {SurfaceMesh[0][i],SurfaceMesh[1][i], SurfaceMesh[2][i]};
        auto normC = norm(columm);
        if (normC>maxdeform)
            maxdeform=normC;


    }
    return maxdeform;
}

static double scalarMatrixMultiplication(vector<vector<double>> a, vector<vector<double>> b){
    int n=a.capacity();
    int m=a[0].capacity();
    double sum = 0;

    for (int i=0;i<n; i++)
        for (int j=0; j<m;j++){
           sum+=a[i][j]*b[i][j];
        }
    return sum;
}

static double scalarMatrixMultiplicationMinus(vector<vector<double>> a, vector<vector<double>> b){
    int n=a.capacity();
    int m=a[0].capacity();
    double sum = 0;

    for (int i=0;i<n; i++)
        for (int j=0; j<m;j++){
            sum+=a[i][j]*(-b[i][j]);
        }
    return sum;
}

static vector<int> maxNElementsAbsVector(int k, vector<double> a){
    vector<int> indexes(k);
    indexes.assign(k,-1);

    std::for_each(a.begin(), a.end(), [](double& d) { d=abs(d);});

    std::priority_queue<std::pair<double, int>> q;
    for (int i = 0; i < a.size(); ++i) {
        q.push(std::pair<double, int>(a[i], i));
    }
    for (int i = 0; i < k; ++i) {
        int ki = q.top().second;
        indexes[i]=ki;
        q.pop();
    }
    return indexes;
}

static double func(vector<double> alpha) {
    //return 100 * pow(x[1] *x[1] - x[2],2)  + pow((x[1] - 1) , 2);
  //  return  100*pow(alpha[0] + 3, 4) + pow(alpha[1] - 3, 4);
    return (sin(alpha[0])+sin(alpha[1]));
  //  return x[0]*x[0] +x[1]*x[1];
}

static vector<double> grad(vector<double> alpha){
    vector<double> y(alpha.capacity());

   // y[0] = 100*(2*(pow(x[1],2)-x[2]))*2*x[1] + 2*(x[1]-1);
    //y[1] = 100*(-2*(pow(x[1],2)-x[2]));

    //  y[0]=400*pow(alpha[0]+3,3);
    //   y[1]=4*pow(alpha[1]-3,3);

    y[0]=cos(alpha[0]);
    y[1]=cos(alpha[1]);

//    y[0] = 2*x[0];
//    y[1] = 2*x[1];
    return y;
}



static double vectorTransposeMultiplication(vector<double> x, vector<double> y){
    unsigned int Array_Length = x.size();
    double sum=0;
    for (int i=0; i<Array_Length;i++)
        sum+=x[i]*y[i];
    return sum;
}

static vector<double> vectorSum(vector<double> x, vector<double> y){
    unsigned int Array_Length=x.size();
    vector<double> sum(Array_Length);
    for (int i=0; i<Array_Length;i++)
        sum[i]=x[i]+y[i];
    return sum;
}


static vector<Standard_Real> vectorSum(vector<Standard_Real> x, vector<double> y){
    unsigned int Array_Length=x.size();
    vector<Standard_Real> sum(Array_Length);
    for (int i=0; i<Array_Length;i++) {
        sum[i] = x[i].getValue() + y[i];
        //TODO
#if !REVERSE_MODE
        sum[i].setADValue(x[i].getADValue());
#else
//        cout << "a part of vectorSum not working with reverse mode" << endl;
#endif
    }
    return sum;
}

static vector<double> scaleVector(double scaleFactor, vector<double> x){
    unsigned int Array_Length = x.size();
    vector<double> sum(Array_Length);
    for (int i=0; i<Array_Length;i++)
        sum[i]=x[i]*scaleFactor;

    return sum;
}

static double norm(vector<double> x){
    unsigned int Array_Length = x.size();
    double sum = 0;
    for (int i=0; i<Array_Length;i++)
        sum+=x[i]*x[i];
    return sqrt(sum);
}


static double maxabs(vector<double> x){
    unsigned int Array_Length = x.size();
    double max = 0;
    for (int i=0; i<Array_Length;i++){
        if (abs(x[i])>max)
            max = abs(x[i]);
    }
    return max;
}



static void RunOptimisation(vector<double> x, bool Armijo=true) {

//Armijo stepsize rule parameters
    double sigma = 10e-3;
    double beta = .5;
    auto obj = func(x);
    auto g = grad(x);
    int k = 0;           //k = # iterations
    int nf = 1;        // nf = # function eval.
    int armijosteps = 0;

    while (norm(g) > 1e-3) {
        auto d = scaleVector(-1, g);                   // steepest     descent direction
        double a = 1;
        auto newobj = func(vectorSum(x, scaleVector(a, d)));
        nf = nf + 1;
        armijosteps = 0;
        while (((newobj - obj) / a) > (sigma * (vectorTransposeMultiplication(g, d)))) {
            armijosteps++;
            a = a * beta;
            newobj = func(vectorSum(x, scaleVector(a, d)));
            nf = nf + 1;

        }



        x = vectorSum(x, scaleVector(a, d));

        cout << "================" << endl;
        cout <<"GC iter = "<<  k << "  Armijo steps:=" << armijosteps << " Stepsize after Armijo = "<< a << " xcureent = " ;
        for (int j=0; j< x.capacity();j++)
            cout << x[j] << " ";
        cout << endl;

        obj = newobj;
        g = grad(x);
        k = k + 1;
    }

    cout <<"Final results ===============" << endl;
    cout << "Function evaluations=" <<nf <<endl;
    for (int j=0; j< x.capacity();j++)
        cout << x[j] << " ";

}

static void RunOptimisationGCConstantStep(vector<double> x){
    double sigma = .1;
    double beta = .5;
    auto obj = func(x);
    auto g = grad(x);
    int k = 0;           //k = # iterations
    int nf = 1;        // nf = # function eval.
    int armijosteps = 0;

    while (norm(g) > 1e-3 && k <10) {
        auto normg = norm(g);
        auto d = scaleVector(-1/norm(g), g);                   // steepest     descent direction
        double a = 0.1;

        x = vectorSum(x, scaleVector(a, d));

        cout << k <<"." << endl;
        for (int j=0; j< x.size();j++)
            cout << x[j] << " ";

       cout << " Objective Function = " << func(x) <<"  Step size=" << 1/normg<<  endl;
        cout << endl;


        g = grad(x);
        k = k + 1;
    }

    cout <<"Final results ===============" << endl;
    cout << "Function evaluations=" <<nf <<endl;
    for (int j=0; j< x.capacity();j++)
        cout << x[j] << " ";
}

#endif //U_BENDOPTIMISATION_CONJUGATEGRADIENT_H


