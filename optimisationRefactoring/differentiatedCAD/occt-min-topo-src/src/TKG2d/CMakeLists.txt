project(TKG2d)

set (TOOLKIT_MODULES
  Geom2d
  LProp
  TColGeom2d
  Adaptor2d
  Geom2dLProp
  Geom2dAdaptor
  GProp
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
