project(TKBO)

set (TOOLKIT_MODULES
  IntTools
  BRepAlgoAPI
  BOPCol
  BOPDS
  BOPAlgo
  BOPTools
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
