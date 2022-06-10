# Install script for directory: /home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/online_pose_graph

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/build/online_pose_graph/catkin_generated/installspace/online_pose_graph.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/online_pose_graph/cmake" TYPE FILE FILES
    "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/build/online_pose_graph/catkin_generated/installspace/online_pose_graphConfig.cmake"
    "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/build/online_pose_graph/catkin_generated/installspace/online_pose_graphConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/online_pose_graph" TYPE FILE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/online_pose_graph/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/onlineMapSmoother" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/onlineMapSmoother")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/onlineMapSmoother"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph" TYPE EXECUTABLE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/devel/lib/online_pose_graph/onlineMapSmoother")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/onlineMapSmoother" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/onlineMapSmoother")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/onlineMapSmoother"
         OLD_RPATH "/usr/local/lib:/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/onlineMapSmoother")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/multiRobotMapSmoother" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/multiRobotMapSmoother")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/multiRobotMapSmoother"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph" TYPE EXECUTABLE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/devel/lib/online_pose_graph/multiRobotMapSmoother")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/multiRobotMapSmoother" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/multiRobotMapSmoother")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/multiRobotMapSmoother"
         OLD_RPATH "/usr/local/lib:/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/multiRobotMapSmoother")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/dummyMapSmoother" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/dummyMapSmoother")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/dummyMapSmoother"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph" TYPE EXECUTABLE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/devel/lib/online_pose_graph/dummyMapSmoother")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/dummyMapSmoother" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/dummyMapSmoother")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/dummyMapSmoother"
         OLD_RPATH "/usr/local/lib:/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/dummyMapSmoother")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/transformMaintainer2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/transformMaintainer2")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/transformMaintainer2"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph" TYPE EXECUTABLE FILES "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/devel/lib/online_pose_graph/transformMaintainer2")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/transformMaintainer2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/transformMaintainer2")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/transformMaintainer2"
         OLD_RPATH "/usr/local/lib:/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/online_pose_graph/transformMaintainer2")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/online_pose_graph" TYPE FILE FILES
    "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/online_pose_graph/online_pose_graph.launch"
    "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/online_pose_graph/online_pose_graph_offline.launch"
    "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/online_pose_graph/online_pose_graph.rviz"
    "/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/src/online_pose_graph/LICENSE"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/shum/Videos/mini_crusher_rocky_for_jean/mini-crusher-backup/Sept_23_crusher_JI_tuned/ground_based_autonomy/build/online_pose_graph/isamlib/cmake_install.cmake")

endif()

