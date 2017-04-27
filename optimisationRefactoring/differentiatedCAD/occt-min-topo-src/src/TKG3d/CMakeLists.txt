project(TKG3d)

set (TOOLKIT_MODULES
  Geom
  TColGeom
  GeomAdaptor
  AdvApprox
  GeomLProp
  Adaptor3d
  LProp3d
  TopAbs
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
