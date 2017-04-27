project(TKMesh)

set (TOOLKIT_MODULES
  BRepMesh
)

if(USE_TBB)
  list( APPEND USED_LIBS tbb )
  list( APPEND USED_LIBS tbbmalloc )
endif()

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
