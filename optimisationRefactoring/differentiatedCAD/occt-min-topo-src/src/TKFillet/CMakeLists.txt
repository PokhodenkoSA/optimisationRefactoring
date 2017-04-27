project(TKFillet)

set (TOOLKIT_MODULES
  ChFiDS
  ChFi2d
  ChFi3d
  ChFiKPart
  Blend
  BRepBlend
  BlendFunc
  BRepFilletAPI
  FilletSurf
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
