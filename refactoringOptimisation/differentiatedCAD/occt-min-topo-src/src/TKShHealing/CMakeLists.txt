project(TKShHealing)

set (TOOLKIT_MODULES
  ShapeBuild
  ShapeExtend
  ShapeConstruct
  ShapeCustom
  ShapeAnalysis
  ShapeFix
  ShapeUpgrade
  ShapeAlgo
  ShapeProcess
  ShapeProcessAPI
)

if (WIN32)
  list( APPEND USED_LIBS wsock32.lib )
endif()

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
