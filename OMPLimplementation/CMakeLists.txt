
cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

# set(CMAKE_TOOLCHAIN_FILE "C:/Users/ricca/vcpkg/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(
  ChronoOMPL
  VERSION 1.0
  LANGUAGES CXX)


SET(ompl_FOUND OFF)
# find_package(ompl PATHS "C:/Users/ricca/vcpkg/vcpkg/packages/ompl_x64-windows/share/ompl" CONFIG REQUIRED)
set(ompl_DIR "C:/workspace/libraries/ompl_install/share/ompl/cmake" CACHE PATH "Path to ompl installation")
find_package(ompl PATHS ${ompl_DIR} CONFIG REQUIRED)

if (NOT ompl_FOUND)
 message(ERROR "Could not find ompl")
 return()
else()
 message(STATUS "Found ompl")
endif()

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Irrlicht
             CONFIG)

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
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface
endif()

########################################
add_executable(ChronoOMPL
                  main.cpp
                  ChPlannerOMPL.h ChPlannerOMPL.cpp
                  ChPlanner.h ChPlanner.cpp
                  ChPlanModel.h ChPlanModel.cpp
                  ChJoint.h ChJoint.cpp

                  ChRobot_6dof_CAD.h ChRobot_6dof_CAD.cpp
                  ChRobot.h ChRobot.cpp
                  ChRobot_6dof.h ChRobot_6dof.cpp
                  ChRobot_6dof_CAD_free.h ChRobot_6dof_CAD_free.cpp
                  ChRobot_free.h ChRobot_free.cpp
                  ChRobot_6dof_free.h ChRobot_6dof_free.cpp
                  #CAD_EXPORT3/UR5e_env_1.h CAD_EXPORT3/UR5e_env_1.cpp   // FOR CAD EXPORT OF ENV 2 (SHELVES)
                  CAD_export_coll/coll.h CAD_export_coll/coll.cpp        // FOR CAD EXPORT OF ENV 3 (LASER)
                  #CAD_EXPORT2/UR5e.h CAD_EXPORT2/UR5e.cpp               // FOR CAD EXPORT OF ENV 1 (BLOCKS)
                )

target_compile_definitions(ChronoOMPL PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(ChronoOMPL PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(ChronoOMPL PUBLIC ${CHRONO_LINKER_FLAGS})
# target_include_directories(ChronoOMPL PUBLIC ${OMPL_INCLUDE_DIRS})
target_link_libraries(ChronoOMPL ompl::ompl ${CHRONO_LIBRARIES})

# target_link_libraries(ChronoOMPL PRIVATE
#             ompl
#             Eigen3::Eigen
#             Boost::filesystem
#             Boost::system)


add_DLL_copy_command()