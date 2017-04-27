cmake_minimum_required(VERSION 3.3)
project(U-bendOptimisation CXX Fortran)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

ADD_LIBRARY(hdmf5lib STATIC IMPORTED)
SET_TARGET_PROPERTIES( hdmf5lib PROPERTIES  IMPORTED_LOCATION /home/salvo/IODA/refactoringOptimisation_compiled/CFD/mgopt-third-party/lib/libhdf5.a)

#TRACELESS
include_directories(/home/salvo/IODA/refactoringOptimisation_compiled/CFD/mgopt-third-party/include/ /home/salvo/IODA/refactoringOptimisation_compiled/differentiatedCAD/occt-min-topo-install/inc /home/salvo/IODA/refactoringOptimisation_compiled/differentiatedCAD/adolc_base/include /Eigen)
link_directories(/home/salvo/IODA/refactoringOptimisation_compiled/differentiatedCAD/occt-min-topo-install/lin64/gcc/libd /home/salvo/IODA/refactoringOptimisation_compiled/differentiatedCAD/adolc_base/lib64)

#TRACE(REVERSE)
#include_directories(/home/salvo/IODA/CFD/mgopt-third-party/hdf5-gnu/include/ /home/salvo/IODA/CAD/170207_OCCT_diff_TraceMode/occt-min-topo-install/inc /home/salvo/IODA/CAD/170206_OCCT_diff_TL/AD/adolc_base/include /home/salvo/IODA/CFD/tests/TUBstator/170207/U-bendOptimisation-App/U-bendOptimisation/Eigen)
#link_directories(/home/salvo/IODA/CAD/170207_OCCT_diff_TraceMode/occt-min-topo-install/lin64/gcc/libd /home/salvo/IODA/CAD/170206_OCCT_diff_TL/AD/adolc_base/lib64)
set(SOURCE_FILES main.cpp hdmf5IO.h ConjugateGradient.h MgoptSolver.h OCCTDataProvider.h OCCTDataProvider.cxx OptimisationState.h CADCFDOptimisation.h CADCFDOptimisation.cxx NSPCC.h NSPCC.cpp NSPCC_Cases_Run.h CADCFDOptimisation_NSPCC.h CADCFDOptimisation_NSPCC.cpp)
add_executable(U-bendOptimisation ${SOURCE_FILES})# lBFGSb/routines.o)


target_link_libraries(U-bendOptimisation -Wl,--no-as-needed -lz -ldl -lgfortran hdmf5lib -ladolc -lTKMath -lTKernel -lTKService -lTKV3d -lTKOpenGl -lTKBRep -lTKIGES -lTKSTL -lTKVRML -lTKSTEP -lTKSTEPAttr -lTKSTEP209 -lTKSTEPBase -lTKGeomBase -lTKGeomAlgo -lTKG3d -lTKG2d -lTKXSBase -lTKShHealing -lTKHLR -lTKTopAlgo -lTKMesh -lTKPrim -lTKCDF -lTKBool -lTKBO -lTKFillet -lTKOffset -lboost_system)
