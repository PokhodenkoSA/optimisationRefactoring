project(TKTopAlgo)

set (TOOLKIT_MODULES
  IntCurvesFace
  MAT
  MAT2d
  Bisector
  BRepMAT2d
  BRepCheck
  BRepBndLib
  BRepExtrema
  BRepClass
  BRepClass3d
  BRepLib
  BRepGProp
  BRepIntCurveSurface
  BRepTopAdaptor
  BRepBuilderAPI
  BRepApprox
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
