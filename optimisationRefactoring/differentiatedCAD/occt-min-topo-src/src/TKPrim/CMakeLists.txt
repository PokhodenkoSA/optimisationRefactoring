project(TKPrim)

set (TOOLKIT_MODULES
  BRepPrim
  BRepSweep
  Sweep
  BRepPrimAPI
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
