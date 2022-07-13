# Install script for directory: /home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/src/phantomx_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/build/phantomx_gazebo/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/build/phantomx_gazebo/catkin_generated/installspace/phantomx_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phantomx_gazebo/cmake" TYPE FILE FILES
    "/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/build/phantomx_gazebo/catkin_generated/installspace/phantomx_gazeboConfig.cmake"
    "/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/build/phantomx_gazebo/catkin_generated/installspace/phantomx_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phantomx_gazebo" TYPE FILE FILES "/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/src/phantomx_gazebo/package.xml")
endif()

