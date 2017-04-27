# ADOL-C directory
if (NOT DEFINED 3RDPARTY_ADOLC_DIR)
  set (3RDPARTY_ADOLC_DIR "${CMAKE_SOURCE_DIR}/../adolc_base" CACHE PATH "The directory containing ADOL-C")
endif()

# ADOL-C include directory
if (NOT DEFINED 3RDPARTY_ADOLC_INCLUDE_DIR)
  find_path( 3RDPARTY_ADOLC_INCLUDE_DIR NAMES adolc/adtl.h PATHS ${3RDPARTY_ADOLC_DIR}/include NO_DEFAULT_PATH)
  #set (3RDPARTY_ADOLC_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../adolc_base/include" CACHE FILEPATH "The directory containing headers of ADOL-C")
endif()

# ADOL-C library file (with absolute path)
if (NOT DEFINED 3RDPARTY_ADOLC_LIBRARY OR NOT 3RDPARTY_ADOLC_LIBRARY_DIR)
  find_library(3RDPARTY_ADOLC_LIBRARY NAMES adolc PATHS ${3RDPARTY_ADOLC_DIR}/lib64 ${3RDPARTY_ADOLC_DIR}/lib NO_DEFAULT_PATHS)
  #set (3RDPARTY_ADOLC_LIBRARY "" CACHE FILEPATH "ADOL-C library" FORCE)
endif()

# ADOL-C library directory
if (NOT DEFINED 3RDPARTY_ADOLC_LIBRARY_DIR)
  get_filename_component(3RDPARTY_ADOLC_LIBRARY_DIR ${3RDPARTY_ADOLC_LIBRARY} DIRECTORY CACHE)
  #set (3RDPARTY_ADOLC_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/../adolc_base/lib64" CACHE FILEPATH "The directory containing ADOL-C library")
endif()

# ADOL-C shared library (with absolute path)
#if (WIN32)
#  if (NOT DEFINED 3RDPARTY_ADOLC_DLL OR NOT 3RDPARTY_ADOLC_DLL_DIR)
#    set (3RDPARTY_ADOLC_DLL "" CACHE FILEPATH "ADOL-C shared library" FORCE)
#  endif()
#endif()

# ADOL-C shared library directory
#if (WIN32 AND NOT DEFINED 3RDPARTY_ADOLC_DLL_DIR)
#  set (3RDPARTY_ADOLC_DLL_DIR "" CACHE FILEPATH "The directory containing ADOL-C shared library")
#endif()

# include occt macros. compiler_bitness, os_wiht_bit, compiler
OCCT_INCLUDE_CMAKE_FILE ("adm/cmake/occt_macros")

# search for product directory inside 3RDPARTY_DIR directory
if (NOT 3RDPARTY_ADOLC_DIR AND 3RDPARTY_DIR)
  FIND_PRODUCT_DIR ("${3RDPARTY_DIR}" "ADOLC" ADOLC_DIR_NAME)
  if (ADOLC_DIR_NAME)
    message (STATUS "Info: ADOL-C: ${ADOLC_DIR_NAME} folder is used")
    set (3RDPARTY_ADOLC_DIR "${3RDPARTY_DIR}/${ADOLC_DIR_NAME}" CACHE PATH "The directory containing ADOL-C" FORCE)
  endif()
endif()

OCCT_MAKE_COMPILER_BITNESS()

# search for include directory in defined 3rdparty directory
if (NOT 3RDPARTY_ADOLC_INCLUDE_DIR OR NOT EXISTS "${3RDPARTY_ADOLC_INCLUDE_DIR}")
  set (3RDPARTY_ADOLC_INCLUDE_DIR "3RDPARTY_ADOLC_INCLUDE_DIR-NOTFOUND" CACHE FILEPATH "The directory containing the headers of ADOL-C" FORCE)
  find_path (3RDPARTY_ADOLC_INCLUDE_DIR adolc/adolc.h PATHS "${3RDPARTY_ADOLC_DIR}/include" NO_DEFAULT_PATH)
endif()

if (NOT 3RDPARTY_ADOLC_INCLUDE_DIR OR NOT EXISTS "${3RDPARTY_ADOLC_INCLUDE_DIR}")
  set (3RDPARTY_ADOLC_INCLUDE_DIR "" CACHE FILEPATH "The directory containing the headers of ADOL-C" FORCE)
endif()

OCCT_MAKE_COMPILER_SHORT_NAME()

# include found paths to common variables
if (3RDPARTY_ADOLC_INCLUDE_DIR AND EXISTS "${3RDPARTY_ADOLC_INCLUDE_DIR}")
  list (APPEND 3RDPARTY_INCLUDE_DIRS "${3RDPARTY_ADOLC_INCLUDE_DIR}")
else()
  list (APPEND 3RDPARTY_NOT_INCLUDED 3RDPARTY_ADOLC_INCLUDE_DIR)
endif()

if (3RDPARTY_ADOLC_LIBRARY AND EXISTS "${3RDPARTY_ADOLC_LIBRARY}")
  list (APPEND 3RDPARTY_LIBRARY_DIRS "${3RDPARTY_ADOLC_LIBRARY_DIR}")
else()
  list (APPEND 3RDPARTY_NOT_INCLUDED 3RDPARTY_ADOLC_LIBRARY_DIR)
endif()

#if (WIN32)
#  if (3RDPARTY_ADOLC_DLL OR EXISTS "${3RDPARTY_ADOLC_DLL}")
#    list (APPEND 3RDPARTY_DLL_DIRS "${3RDPARTY_ADOLC_DLL_DIR}")
#  else()
#    list (APPEND 3RDPARTY_NOT_INCLUDED 3RDPARTY_ADOLC_DLL_DIR)
#  endif()
#endif()

# the library directory for using by the executable
#if (WIN32)
#  set (USED_3RDPARTY_ADOLC_DIR ${3RDPARTY_ADOLC_DLL_DIR})
#else()
  set (USED_3RDPARTY_ADOLC_DIR ${3RDPARTY_ADOLC_LIBRARY_DIR})
#endif()

mark_as_advanced (3RDPARTY_ADOLC_LIBRARY 3RDPARTY_ADOLC_DLL)
