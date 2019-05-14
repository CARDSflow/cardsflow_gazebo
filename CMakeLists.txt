cmake_minimum_required(VERSION 2.8.3)
project(cardsflow_gazebo)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -g ")

SET(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rosbag
		std_msgs geometry_msgs
		message_generation
		common_utilities
		roboy_middleware_msgs
		eigen_conversions
)

catkin_package( DEPENDS GAZEBO CATKIN_DEPENDS roboy_middleware_msgs common_utilities
        INCLUDE_DIRS include)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

#find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
#find_package(nlohmann_json REQUIRED)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
message(STATUS ${GAZEBO_INCLUDE_DIRS})

# libxml++
find_package(LibXML++ REQUIRED)
include_directories(${LibXML++_INCLUDE_DIRS})
include_directories(
		include/
        ${catkin_INCLUDE_DIRS}
#		${Qt5Widgets_INCLUDE_DIRS}
#		${Qt5Core_INCLUDE_DIRS}
#		${Qt5Gui_INCLUDE_DIRS}
)

#roboticslibrary
find_package(rl REQUIRED)
include_directories(${RL_INCLUDE_DIRS})

#==============================================================================
# Application
#==============================================================================

add_library(cardsflow_gazebo_plugin SHARED
		src/cardsflow_gazebo.cpp
		src/eccerobotMuscle/physics/Actuator.cpp
		src/muscle/IMuscle.cpp
		src/muscle/ISee.cpp
		src/muscle/IActuator.cpp
		src/muscle/IViaPoints.cpp
		src/muscle/CylindricalWrapping.cpp
		src/muscle/SphericalWrapping.cpp
		src/muscle/MuscPID.cpp
        #	src/muscle/MeshWrapping.cpp
        include/cardsflow_gazebo/cardsflow_gazebo.hpp
        include/cardsflow_gazebo/muscle/MuscPID.hpp
		include/cardsflow_gazebo/muscle/IMuscle.hpp
		include/cardsflow_gazebo/muscle/CylindricalWrapping.hpp
		include/cardsflow_gazebo/muscle/IActuator.hpp
		include/cardsflow_gazebo/muscle/ISee.hpp
		include/cardsflow_gazebo/muscle/IViaPoints.hpp
		#	include/cardsflow_gazebo/muscle/MeshWrapping.hpp
		include/cardsflow_gazebo/muscle/SphericalWrapping.hpp
		include/cardsflow_gazebo/muscle/StateMachine.hpp
		include/cardsflow_gazebo/eccerobotMuscle/physics/Actuator.h
		include/cardsflow_gazebo/eccerobotMuscle/physics/IActuator.h
		)

add_dependencies( cardsflow_gazebo_plugin roboy_middleware_msgs_gencpp roboy_simulation_msgs_gencpp )
target_link_libraries(cardsflow_gazebo_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${RL_LIBRARIES} nlohmann_json)

add_executable(testMuscle src/testMuscle.cpp
		src/eccerobotMuscle/physics/Actuator.cpp
		include/cardsflow_gazebo/eccerobotMuscle/physics/Actuator.h
		include/cardsflow_gazebo/eccerobotMuscle/physics/IActuator.h)

add_dependencies(testMuscle ${RL_LIBRARIES})
target_link_libraries(testMuscle ${RL_LIBRARIES})


add_executable(testZenerModel
		src/eccerobotMuscle/physics/Actuator.cpp
		src/eccerobotMuscle/physics/ZenerModel.cpp
		src/testZenerModel.cpp
		include/cardsflow_gazebo/eccerobotMuscle/physics/ISeriesElasticElement.h
		include/cardsflow_gazebo/eccerobotMuscle/physics/IZenerModel.h
		include/cardsflow_gazebo/eccerobotMuscle/physics/ZenerModel.h)

add_dependencies(testZenerModel ${RL_LIBRARIES})
target_link_libraries(testZenerModel ${RL_LIBRARIES})

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
		cardsflow_gazebo_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
	    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
        package.xml
		cardsflow_gazebo_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
		include/cardsflow_gazebo/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.hpp"
		)
