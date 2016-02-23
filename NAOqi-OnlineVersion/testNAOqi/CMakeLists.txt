cmake_minimum_required(VERSION 2.8)

# Give a name to the project.
project(testNAOqi)

# You need this to find the qiBuild CMake framework
find_package(qibuild)

# I add this to fix inclusion problem of opencv2 (last line of this cmake) by antonio
FIND_PACKAGE( OpenCV REQUIRED ) ##############################################

# Create a list of source files
set(_srcs
    main.cpp
    cJoystick.cpp	)

# Create a executable named mybroker
# with the source file: main.cpp
qi_create_bin(testNAOqi ${_srcs})


# Tell CMake that mybroker depends on ALCOMMON.
# This will set the libraries to link mybroker with, the include paths, and so on
qi_use_lib(testNAOqi ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)
