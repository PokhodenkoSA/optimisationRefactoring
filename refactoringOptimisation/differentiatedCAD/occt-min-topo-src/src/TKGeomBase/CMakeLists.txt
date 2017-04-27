project(TKGeomBase)

set (TOOLKIT_MODULES
  ProjLib
  GeomProjLib
  GCPnts
  CPnts
  Approx
  AppParCurves
  FEmTool
  AppCont
  Extrema
  IntAna
  IntAna2d
  GeomConvert
  AdvApp2Var
  GeomLib
  Geom2dConvert
  Hermit
  BndLib
  AppDef
  GeomTools
  GC
  GCE2d
  gce
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
