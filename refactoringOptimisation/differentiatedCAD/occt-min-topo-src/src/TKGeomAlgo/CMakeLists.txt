project(TKGeomAlgo)

set (TOOLKIT_MODULES
  Hatch
  GeomInt
  IntStart
  IntWalk
  IntImp
  IntCurveSurface
  IntSurf
  IntPatch
  Geom2dInt
  IntImpParGen
  IntRes2d
  IntCurve
  TopTrans
  Intf
  ApproxInt
  GccAna
  GccEnt
  GccInt
  HatchGen
  Geom2dHatch
  Law
  AppBlend
  Plate
  GeomPlate
  LocalAnalysis
  GeomAPI
  GeomFill
  Geom2dAPI
  Geom2dGcc
  FairCurve
  NLPlate
  IntPolyh
  TopClass
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
