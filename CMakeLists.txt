cmake_minimum_required( VERSION 3.6 )
project ( myslam )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
# set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11 -march=native -O3")
endif()
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
# Note: Eigen >= 3.3.0 would cause double free error when coorporating w/ PCL
# Other solution: include Eigen directory after the PCL's
include_directories( "/usr/local/include/eigen3" )
# OpenCV
# find_package( OpenCV 3.1 REQUIRED )
set( OpenCV_STATIC OFF )
find_package( OpenCV REQUIRED )
# Required Viz Module
if( OpenCV_FOUND )
  if( NOT "opencv_viz" IN_LIST OpenCV_LIBS )
    message( FATAL_ERROR "not found opencv_viz module." )
  endif()
endif()
if( OpenCV_STATIC AND MSVC )
  foreach( flag_var
      CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
      CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO )
    if( ${flag_var} MATCHES "/MD" )
      string( REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}" )
    endif()
  endforeach()
endif()

# PCL
find_package(PCL 1.7.2 REQUIRED)

# Sophus 
find_package( Sophus REQUIRED )
# G2O
find_package( G2O REQUIRED )
# Thread
find_package( Threads REQUIRED )
# Find Package Boost
# set( BOOST_ROOT "C:/Program Files/boost" )
set( Boost_USE_STATIC_LIBS ON ) # Static Link Libraries ( libboost_* )
set( Boost_USE_MULTITHREADED ON ) # Multi Thread Libraries ( *-mt-* )
set( Boost_USE_STATIC_RUNTIME OFF ) # Static Runtime Libraries ( *-s* )
find_package( Boost COMPONENTS system date_time regex )
# If Direct Capture from Sensors, VelodyneCapture are required Boost.Asio.
# Please Define HAVE_BOOST in Preprocessor.
set( HAVE_BOOST )
if( Boost_FOUND )
  set( HAVE_BOOST "-DHAVE_BOOST" )
endif()
# PCAP
set( ENV{PCAP_DIR} "/usr" )
find_package( PCAP )
# If Capture from PCAP Files, VelodyneCapture are required PCAP.
# Please Define HAVE_PCAP in Preprocessor.
set( HAVE_PCAP )
if( PCAP_FOUND )
  set( HAVE_PCAP "-DHAVE_PCAP" )
endif()

# Add Definitions
add_definitions( ${Boost_LIB_DIAGNOSTIC_DEFINITIONS} )
add_definitions( ${HAVE_BOOST} )
add_definitions( ${HAVE_PCAP} )
add_definitions( ${PCL_DEFINITIONS})

set( THIRD_PARTY_LIBS 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${PCAP_LIBRARIES}
)
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories( ${G2O_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${PCAP_INCLUDE_DIRS} )
############### source and test ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )