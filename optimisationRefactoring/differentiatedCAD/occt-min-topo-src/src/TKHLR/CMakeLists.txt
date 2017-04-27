project(TKHLR)

set (TOOLKIT_MODULES
  HLRTopoBRep
  HLRBRep
  HLRAlgo
  HLRAppli
  Intrv
  TopBas
  TopCnx
  Contap
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
