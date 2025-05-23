# Install script for directory: /home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/local_planner

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/build/local_planner/catkin_generated/installspace/local_planner.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_planner/cmake" TYPE FILE FILES
    "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/build/local_planner/catkin_generated/installspace/local_plannerConfig.cmake"
    "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/build/local_planner/catkin_generated/installspace/local_plannerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_planner" TYPE FILE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/local_planner/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/localPlanner" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/localPlanner")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/localPlanner"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/local_planner" TYPE EXECUTABLE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/devel/lib/local_planner/localPlanner")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/localPlanner" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/localPlanner")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/localPlanner"
         OLD_RPATH "/usr/local/lib:/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/localPlanner")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/pathFollower" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/pathFollower")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/pathFollower"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/local_planner" TYPE EXECUTABLE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/devel/lib/local_planner/pathFollower")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/pathFollower" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/pathFollower")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/pathFollower"
         OLD_RPATH "/usr/local/lib:/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/local_planner/pathFollower")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_planner/launch" TYPE DIRECTORY FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/local_planner/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_planner/paths" TYPE DIRECTORY FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/local_planner/paths/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_planner/rviz" TYPE DIRECTORY FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/local_planner/rviz/")
endif()

