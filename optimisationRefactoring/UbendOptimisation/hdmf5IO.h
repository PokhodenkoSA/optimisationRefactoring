#ifndef U_BENDOPTIMISATION_HDMF5IO_H
#define U_BENDOPTIMISATION_HDMF5IO_H

#include <iostream>
#include <string>
#include <vector>
#include "hdf5.h"
#include "Eigen/Dense"
#include <exception>

using Eigen::MatrixXd;

using namespace std;
template<typename T>
vector<vector<T>> getTable(const char* FILE, const char* dataset){
    hid_t       file_id, dataset_id, group_id, dspace;  /* identifiers */
    herr_t      status;
    int         i, j;

    /* Open an existing file. */
    try {

    file_id = H5Fopen(FILE, H5F_ACC_RDWR, H5P_DEFAULT);

    status = H5Gget_objinfo (file_id, dataset, 0, NULL);


    if (status!=0) {
        cout << "The dataset " << dataset << " either does NOT exist\n or some other error occurred.\n";
        throw "The group either does NOT exist\n or some other error occurred.\n";
    }

//    /* Open an existing dataset. */
    dataset_id = H5Dopen2(file_id, dataset, H5P_DEFAULT);
    dspace = H5Dget_space(dataset_id);

    const int ndims = H5Sget_simple_extent_ndims(dspace);
    if (ndims>2){
        cout << "Error dataset" << dataset << "more than 2 dimesnional";
        throw std::exception( );
    }
    hsize_t dims[ndims];
    H5Sget_simple_extent_dims(dspace, dims, NULL);


    T data[dims[0]][dims[1]];
//  Get Data




    if (std::is_same<T, double>::value) {
    status = H5Dread(dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                     data);
    }

    if (std::is_same<T, int>::value) {
        status = H5Dread(dataset_id, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
    }

    vector<vector<T>> datavec(dims[0], vector<T>(dims[1]));
    for (int i=0;i<dims[0]; i++){
        for (int j=0; j<dims[1]; j++){
            datavec[i][j]=data[i][j];
        }
    }


    /* Close the dataset. */
    status = H5Dclose(dataset_id);

    status = H5Fclose(file_id);
    return datavec;
}
    catch (string e){
         cout << e << endl;
    }
}

template <typename T>
vector<T> getVector(const char* FILE, const char* dataset){
    hid_t       file_id, dataset_id, group_id, dspace;  /* identifiers */
    herr_t      status;
    int         i, j;

    /* Open an existing file. */
    try {

        file_id = H5Fopen(FILE, H5F_ACC_RDWR, H5P_DEFAULT);

        status = H5Gget_objinfo (file_id, dataset, 0, NULL);


        if (status!=0) {
            cout << "The dataset " << dataset << " either does NOT exist\n or some other error occurred.\n";
            throw "The group either does NOT exist\n or some other error occurred.\n";
        }

//    /* Open an existing dataset. */
        dataset_id = H5Dopen2(file_id, dataset, H5P_DEFAULT);
        dspace = H5Dget_space(dataset_id);

        const int ndims = H5Sget_simple_extent_ndims(dspace);
        if (ndims>2){
            cout << "Error dataset" << dataset << "more than 2 dimesnional";
            throw std::exception( );
        }
        hsize_t dims[ndims];
        H5Sget_simple_extent_dims(dspace, dims, NULL);


        T data[dims[0]][dims[1]];
//  Get Data




        if (std::is_same<T, double>::value) {
            status = H5Dread(dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                             data);
        }

        if (std::is_same<T, int>::value) {
            status = H5Dread(dataset_id, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
        }

        vector<vector<T>> datavec(dims[0], vector<T>(dims[1]));
        for (int i=0;i<dims[0]; i++){
            for (int j=0; j<dims[1]; j++){
                datavec[i][j]=data[i][j];
            }
        }


        /* Close the dataset. */
        status = H5Dclose(dataset_id);

        status = H5Fclose(file_id);
        return datavec[0];
    }
    catch (string e){
        cout << e << endl;
    }
}

template <typename T>
int CreateAndWriteTable(vector<vector<T>> data, const char *FILE, const char *dataset_name){
    hid_t       file_id, dataset_id, group_id, dspace, dataspace_id;  /* identifiers */
    herr_t      status;
    int         i, j, globalStaus;
    hsize_t dims[2];
    globalStaus=1;
    try {
        /* Open an existing file. */
        file_id = H5Fopen(FILE, H5F_ACC_RDWR, H5P_DEFAULT);

        //check wheter dataset exist
      //  status = H5Gget_objinfo(file_id, dataset_name, 0, NULL);


        dims[0] = data.size();
        dims[1] = data[0].size();
        T dset_data[dims[0]][dims[1]];

        //create new dataset
        /* Initialize the first dataset. */
        for (i = 0; i < dims[0]; i++)
            for (j = 0; j < dims[1]; j++)
                dset_data[i][j] = data[i][j];

      //  if (status != 0) {


            /* Create the data space for the first dataset. */

            dataspace_id = H5Screate_simple(2, dims, NULL);

            if (std::is_same<T, double>::value) {
                /* Create a dataset */
                dataset_id = H5Dcreate(file_id, dataset_name, H5T_NATIVE_DOUBLE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT,
                                       H5P_DEFAULT);
            }

            if (std::is_same<T, int>::value) {
                dataset_id = H5Dcreate(file_id, dataset_name, H5T_NATIVE_INT, dataspace_id, H5P_DEFAULT, H5P_DEFAULT,
                                       H5P_DEFAULT);
            }
       // }

        /* Write the first dataset. */
        if (std::is_same<T, double>::value) {
        status = H5Dwrite(dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                          dset_data);
        globalStaus*=status;}

        if (std::is_same<T, int>::value) {
            status = H5Dwrite(dataset_id, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                              dset_data);
            globalStaus*=status;}

        /* Close the data space for the first dataset. */
        status = H5Sclose(dataspace_id);
        globalStaus*=status;
        /* Close the first dataset. */
        status = H5Dclose(dataset_id);




        /* Close the file. */
        status = H5Fclose(file_id);
        return globalStaus;
    }
    catch(int e){
        cout << "Exception in Creating Table" << endl;
    }
}


template <typename T>
int CreateAndWriteVector(vector<T> data, const char *FILE, const char *dataset_name){
    vector<vector<T>> data2(1, vector<T>(data.size()));
    for (int i=0;i<data.size();i++)
        data2[0][i]=data[i];
    CreateAndWriteTable(data2,FILE,dataset_name);
}

template <typename T>
int CreateAndWriteTable(MatrixXd data, const char *FILE, const char *dataset_name){
    hid_t       file_id, dataset_id, group_id, dspace, dataspace_id;  /* identifiers */
    herr_t      status;
    int         i, j, globalStaus;
    hsize_t dims[2];
    globalStaus=1;
    try {
        /* Open an existing file. */
        file_id = H5Fopen(FILE, H5F_ACC_RDWR, H5P_DEFAULT);

        //check wheter dataset exist
        //  status = H5Gget_objinfo(file_id, dataset_name, 0, NULL);


        dims[0] = data.rows();
        dims[1] = data.cols();
        double dset_data[dims[0]][dims[1]];

        //create new dataset
        /* Initialize the first dataset. */
        for (i = 0; i < dims[0]; i++)
            for (j = 0; j < dims[1]; j++)
                dset_data[i][j] = data(i,j);

        //  if (status != 0) {


        /* Create the data space for the first dataset. */

        dataspace_id = H5Screate_simple(2, dims, NULL);

        /* Create a dataset */
        dataset_id = H5Dcreate(file_id, dataset_name, H5T_NATIVE_DOUBLE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT,
                               H5P_DEFAULT);

        /* Write the first dataset. */

        status = H5Dwrite(dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                          dset_data);
        globalStaus*=status;


        /* Close the data space for the first dataset. */
        status = H5Sclose(dataspace_id);
        globalStaus*=status;
        /* Close the first dataset. */
        status = H5Dclose(dataset_id);

        /* Close the file. */
        status = H5Fclose(file_id);
        return globalStaus;
    }
    catch(int e){
        cout << "Exception in Creating Table" << endl;
    }
}

template<typename T>
MatrixXd getTable_Eigen(const char* FILE, const char* dataset){
    hid_t       file_id, dataset_id, group_id, dspace;  /* identifiers */
    herr_t      status;
    int         i, j;

    /* Open an existing file. */
    try {

        file_id = H5Fopen(FILE, H5F_ACC_RDWR, H5P_DEFAULT);

        status = H5Gget_objinfo (file_id, dataset, 0, NULL);


        if (status!=0) {
            cout << "The dataset " << dataset << " either does NOT exist\n or some other error occurred.\n";
            throw "The group either does NOT exist\n or some other error occurred.\n";
        }

//    /* Open an existing dataset. */
        dataset_id = H5Dopen2(file_id, dataset, H5P_DEFAULT);
        dspace = H5Dget_space(dataset_id);

        const int ndims = H5Sget_simple_extent_ndims(dspace);
        if (ndims>2){
            cout << "Error dataset" << dataset << "more than 2 dimesnional";
            throw std::exception( );
        }
        hsize_t dims[ndims];
        H5Sget_simple_extent_dims(dspace, dims, NULL);


        double data[dims[0]][dims[1]];
//  Get Data


            status = H5Dread(dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                             data);



        MatrixXd datavec;
        datavec.resize(dims[0], dims[1]);

        for (int i=0;i<dims[0]; i++){
            for (int j=0; j<dims[1]; j++){
                datavec(i,j)=data[i][j];
            }
        }


        /* Close the dataset. */
        status = H5Dclose(dataset_id);

        status = H5Fclose(file_id);
        return datavec;
    }
    catch (string e){
        cout << e << endl;
    }
}



//int run() {
//    const char* FILE = "/home/orestmykhaskiv/QMUL/mgopt/cases/regressionChecks/cube/solution.xdmf_field.h5";
//    hid_t       file_id, dataset_id, group_id, dspace;  /* identifiers */
//    herr_t      status;
//    int         i, j;
//    double dset_data[3][64];
//
//    /* Initialize the dataset. */
//    for (i = 0; i < 3; i++)
//        for (j = 0; j < 64; j++)
//            dset_data[i][j] = 0;
//
//    /* Open an existing file. */
//    file_id = H5Fopen(FILE, H5F_ACC_RDWR, H5P_DEFAULT);
//
////    /* Open an existing dataset. */
//    dataset_id = H5Dopen2(file_id, "/sensitivity", H5P_DEFAULT);
//    dspace = H5Dget_space(dataset_id);
//    const int ndims = H5Sget_simple_extent_ndims(dspace);
//    cout << ndims << endl;
//
//    hsize_t dims[ndims];
//    H5Sget_simple_extent_dims(dspace, dims, NULL);
//    for (int i=0; i<ndims; i++)
//        cout  << dims[i] << endl;
//
//
////
//    status = H5Dread(dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT,
//                     dset_data);
////
//
//
//    /* Close the file. */
//
//    for (i = 0; i < 3; i++){
//        for (j = 0; j < 64; j++){
//           cout << dset_data[i][j] << " "  ;
//        }
//        cout << endl;}
//
//    dset_data[0][0] = 100;
//
//       /* Write the dataset. */
//    status = H5Dwrite(dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT,
//                      dset_data);
//
//    /* Close the dataset. */
//    status = H5Dclose(dataset_id);
//
//    status = H5Fclose(file_id);
//}

#endif //U_BENDOPTIMISATION_HDMF5IO_H
