project(TKOffset)

set (TOOLKIT_MODULES
  BRepOffsetAPI
  Draft
  BRepOffset
  BiTgte
)

if (USE_ADOLC)
  list( APPEND USED_LIBS adolc boost_system)
endif()

OCCT_INCLUDE_CMAKE_FILE (adm/cmake/occt_toolkit)
