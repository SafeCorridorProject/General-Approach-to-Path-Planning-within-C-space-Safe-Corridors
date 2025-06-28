#--------------------------------------------------------------
# 
# Example of CMake configuration file to build an external 
# project depending on Chrono and on optional Chrono modules.
# 
# This minimal sample project can be used as a template for a
# user project.  Modify sections 1, 2, and 3 below as appropriate.
# 
#--------------------------------------------------------------
 

cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)
#--------------------------------------------------------------
# === 1 === 
# Set the project name
#--------------------------------------------------------------

project(PathPlanning_project)

#--------------------------------------------------------------
# === 2 ===
# Find the Chrono package and any REQUIRED or OPTIONAL modules
# by invoking the find_package function in CONFIG mode:
#    find_package(Chrono
#                 COMPONENTS req_module1 req_module1 ...
#                 OPTIONAL_COMPONENTS opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono modules can be requested (case insensitive):
#   Cascade, Cosimulation, Irrlicht, OpenGL, Matlab, Multicore, Gpu,
#   PardisoMKL, PardisoProject, Postprocess, Python, Vehicle,
#   VehicleCosimm, VSG.
# A component can be requested either as required or optional
# (see the CMake documentation for find_package).
# 
# Note that you will have to set the variable Chrono_DIR to 
# specify the location of the chrono-config.cmake script, if
# it is not in its default install location.
# Chrono_DIR can be either a Chrono build tree or a Chrono install tree.
# 
# The following variables are set and can be used further down:
# Chrono_FOUND
#   set to true if Chrono and all required components were found
# CHRONO_C_FLAGS
# CHRONO_CXX_FLAGS
#   C and C++ compilation flags
# CHRONO_INCLUDE_DIRS
#   additional paths for included headers
# CHRONO_LIBRARIES
#   list of required libraries (with full path)
# CHRONO_LINKER_FLAGS
#   additional linker flags
# CHRONO_DATA_DIR
#   path to the Chrono data make_directory
# 
# In addition, for each requested component [COMPONENT], the
# following variable is set to true (ON) or false (OFF):
# CHRONO_[COMPONENT]_FOUND
# 
# In this example, we only request the Irrlicht module (required)
# and, for demonstration purposes, the PardisoMKL module (optional)
#--------------------------------------------------------------

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Irrlicht
             OPTIONAL_COMPONENTS PardisoMKL
             CONFIG)
find_package(osqp REQUIRED)
set(osqp_DIR "C:/Program Files (x86)/osqp/cmake") # C:\Program Files (x86)\osqp\cmake")
#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Important! To ensure ABI compatibility, use the same C++ standard
# as the one used to build the Chrono libraries.
#--------------------------------------------------------------

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
endif()

#--------------------------------------------------------------
# === 3 ===
# Add the executable from your project and specify all C++ 
# files in your project. 
#--------------------------------------------------------------

add_executable(my_demo
                       Example.cpp
                       ChJoint.cpp ChJoint.h
                       ChPlanner.cpp ChPlanner.h
                       ChPlanModel.cpp ChPlanModel.h
                       GbpfAlgorithmCube.h GbpfAlgorithmCube.cpp
                       #CAD_EXPORT/comau_racer_export_2.h CAD_EXPORT/comau_racer_export_2.cpp
                       ChRobot_6dof_CAD.h ChRobot_6dof_CAD.cpp
                       ChRobot.h ChRobot.cpp
                       ChRobot_6dof.h ChRobot_6dof.cpp
                       #CAD_EXPORT2/UR5e.h CAD_EXPORT2/UR5e.cpp                       #this for blocks
                       #CAD_EXPORT3/UR5e_env_1.h CAD_EXPORT3/UR5e_env_1.cpp           #this for shelf
                       #CAD_EXPORT4/Env3.h CAD_EXPORT4/Env3.cpp
                       #CAD_export_final/export_final.h CAD_export_final/export_final.cpp
                       #CAD_export_375/375.h CAD_export_375/375.cpp
                       #CAD_export_400/400.h CAD_export_400/400.cpp
                       #CAD_export_400_2/400_2.h CAD_export_400_2/400_2.cpp             
                       #CAD_export_try/try.h CAD_export_try/try.cpp                     #this for blocks with new end effector
                       #CAD_export_coll/coll.h CAD_export_coll/coll.cpp                  #this for env 3 with no compenetration between coll shape
                       ChRobot_6dof_CAD_free.h ChRobot_6dof_CAD_free.cpp
                       ChRobot_free.h ChRobot_free.cpp
                       ChRobot_6dof_free.h ChRobot_6dof_free.cpp
                       CAD_export_new/new_coll.h CAD_export_new/new_coll.cpp
                       GbpfAlgorithmCuboid.h GbpfAlgorithmCuboid.cpp)
#--------------------------------------------------------------
# Set properties for your executable target
# 
# Note that here we define a macro CHRONO_DATA_DIR which will
# contain the path to the Chrono data directory, either in its
# source tree (if using a build version of Chrono), or in its
# install tree (if using an installed version of Chrono).
#--------------------------------------------------------------

target_compile_definitions(my_demo PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(my_demo PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(my_demo PUBLIC ${CHRONO_LINKER_FLAGS})
target_include_directories(my_demo PUBLIC "C:/Program Files (x86)/osqp/include") # C:\Program Files (x86)\osqp\include

#--------------------------------------------------------------
# Link to Chrono libraries and dependency libraries
#--------------------------------------------------------------

target_link_libraries(my_demo ${CHRONO_LIBRARIES} osqp::osqp)

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# DLLs will be copied into ${PROJECT_BINARY_DIR}/${config} by default
# or in ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${config} if only CMAKE_RUNTIME_OUTPUT_DIRECTORY is set
# or to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG>} if the specific CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG> has been set
#--------------------------------------------------------------

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "<mycustompathforrelease>")

add_DLL_copy_command()




