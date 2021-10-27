cmake_minimum_required (VERSION 2.8.11)

project (scanline_viewer CXX)

include (FindPkgConfig)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "Release")
endif ()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
  set (CXX_FLAGS "-std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers $ENV{CXXFLAGS}")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 ${CXX_FLAGS}")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1 ${CXX_FLAGS}")
  set (MATH_LIBRARIES m)
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
  set (CXX_FLAGS "/std:c++latest /wd4244 /wd4305 $ENV{CXXFLAGS}")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CXX_FLAGS}")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CXX_FLAGS}")
else ()
  message (FATAL_ERROR "Unsupported compiler ${CMAKE_CXX_COMPILER_ID}")
endif ()

#if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
#  set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#  set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#  set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#endif ()

#if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
#  set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#  set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#  set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#endif ()

#if (NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
#  set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#  set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#  set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#endif ()

# Build without console application.
set(CMAKE_CXX_FLAGS "-mwindows")

message("CMAKE_CURRENT_SOURCE_DIR = " ${CMAKE_CURRENT_SOURCE_DIR})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/source")

if( WIN32 )
        set (QTLIBS Qt5::Core Qt5::Widgets Qt5::Gui Qt5::PrintSupport)
else()
        set (QTLIBS QtGui QtCore QtWidgets QtPrintSupport)
endif()

#find_package (Qt5 REQUIRED)
find_package(Qt5 COMPONENTS Gui Core Widgets PrintSupport REQUIRED)
# qt4_wrap_cpp (MOC_SOURCES ${HEADERS})
#include(${QT_USE_FILE})

#include (findeigen3)
#find_package (Eigen3 REQUIRED)
#include_directories (${EIGEN3_INCLUDE_DIR})
SET(Eigen_DIR D:/soft/developerTools/github/eigen)
include_directories (${Eigen_DIR})
#pkg_search_module(Eigen3 REQUIRED eigen3)

link_directories ("${CMAKE_SOURCE_DIR}/bin")

set (ADDON_LIBRARIES ${ADDON_LIBRARIES})

add_definitions (-DG_LOG_DOMAIN="ScanlineViewer")
#add_subdirectory ("custom_plot")
add_subdirectory ("source")
#add_subdirectory ("tests")
