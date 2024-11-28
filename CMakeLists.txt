FILE(GLOB_RECURSE SOURCES_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
FILE(GLOB_RECURSE INCLUDE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

SET(GRASPTELEOP_INCLUDE ${INCLUDE_FILES})
SET(GRASPTELEOP_SOURCES ${SOURCES_FILES})

#Look for VISP
FIND_PACKAGE(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

#Look for EIGEN3
SET( EIGEN3_INCLUDE "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT EIGEN3_INCLUDE )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()

#tinyXML2
SET( TINYXML2_INCLUDE "C:/ThirdParty/tinyxml2/include" )
FIND_LIBRARY(TINYXML2 "tinyxml2" HINTS "C:/ThirdParty/tinyxml2/lib")

#KDL 
SET(KDL_INCLUDE "C:/ThirdParty/KDL/include")
FIND_LIBRARY(KDL "orocos-kdl" HINTS "C:/ThirdParty/KDL/lib")
FIND_LIBRARY(KDL_MODEL "orocos-kdl-models" HINTS "C:/ThirdParty/KDL/lib")

#SFML
SET(SFML_INCLUDE "C:/ThirdParty/SFML/include")
FILE(GLOB_RECURSE SFML_LIBS "C:/ThirdParty/SFML/lib/*.lib")

#Boost
ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ALL_DYN_LINK OFF)
CMAKE_POLICY(SET CMP0074 NEW) ##TODO - Hack to avoid warning
FIND_PACKAGE(Boost REQUIRED )

#PCL
FIND_PACKAGE(PCL 1.3 REQUIRED)
ADD_DEFINITIONS(${PCL_DEFINITIONS} -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

#Generate Executable
ADD_EXECUTABLE(AppGraspTeleop ${GRASPTELEOP_SOURCES} ${GRASPTELEOP_INCLUDE})
INCLUDE_DIRECTORIES(AppGraspTeleop PUBLIC ${EIGEN3_INCLUDE} ${KDL_INCLUDE} 
                                          ${TINYXML2_INCLUDE} ${SFML_INCLUDE} 
                                          ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
                                          "${CMAKE_SOURCE_DIR}/modules/Robot" 
                                          "${CMAKE_SOURCE_DIR}/modules/Kin/include"
                                          "${CMAKE_SOURCE_DIR}/modules/Trajectory/include"
                                          "${CMAKE_SOURCE_DIR}/modules/Grasp/include"
                                          "${CMAKE_SOURCE_DIR}/Core/Sys/include")

#Finding libraries (To improve)
FIND_LIBRARY(KukaIIWA_LIB "KukaIIWA" HINTS "${CMAKE_SOURCE_DIR}/lib")
FIND_LIBRARY(SchunkPG70_LIB "SchunkPG70" HINTS "${CMAKE_SOURCE_DIR}/lib")
FIND_LIBRARY(Virtuose6D_LIB "Virtuose6D" HINTS "${CMAKE_SOURCE_DIR}/lib")
FIND_LIBRARY(Kin_LIB "Kin" HINTS "${CMAKE_SOURCE_DIR}/lib")
FIND_LIBRARY(Virtuose_LIB "virtuoseDLL" HINTS "${CMAKE_SOURCE_DIR}/lib")
FIND_LIBRARY(Grasp_LIB "Grasp" HINTS "${CMAKE_SOURCE_DIR}/lib")
FIND_LIBRARY(Trajectory_LIB "Trajectory" HINTS "${CMAKE_SOURCE_DIR}/lib")
#Link
TARGET_LINK_LIBRARIES(AppGraspTeleop ${KukaIIWA_LIB} ${SchunkPG70_LIB} 
                                      ${Virtuose6D_LIB} ${Virtuose_LIB} 
                                      ${Kin_LIB} ${VISP_LIBRARIES} ${KDL_LIB} 
                                      ${KDL_MODEL_LIB} ${Grasp_LIB} ${Trajectory_LIB}
                                      ${TINYXML2} ${KDL} ${KDL_MODEL} 
                                      ${SFML_LIBS} ${PCL_LIBRARIES} 
                                      ${Boost_LIBRARIES}
                                      ws2_32 legacy_stdio_definitions)

#Set the folders Properties
SET_PROPERTY(TARGET AppGraspTeleop PROPERTY PROJECT_LABEL "GraspTeleop")
SET_PROPERTY(TARGET AppGraspTeleop PROPERTY FOLDER "App/GraspTeleop")
SOURCE_GROUP("Include Files" FILES ${GRASPTELEOP_INCLUDE})

# Copy the .dll to the output folder
SET(VirtuoseRuntime "${CMAKE_SOURCE_DIR}/modules/Robot/dxVirtuose6D/lib/win/bin/VC2015/x64/Release/virtuoseAPI.dll")
FILE(COPY "${VirtuoseRuntime}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/Release")
