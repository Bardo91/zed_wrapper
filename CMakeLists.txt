###############################################################
######                                                   ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(zed_wrapper)

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(src)

#########################################
###### Loading 3rd party libraries.######
#########################################
find_package(catkin REQUIRED COMPONENTS
			geometry_msgs
			rgbd_tools  
			cv_bridge 
			image_transport
			sensor_msgs)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs rgbd_tools cv_bridge image_transport sensor_msgs
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#########################################
######			Executable		   ######
#########################################
IF(UNIX)
	add_definitions(-std=c++11)
ENDIF()

file(GLOB_RECURSE SRC_FILES "src/*.cpp" "src/*.h" "src/*.inl" )
GroupSources(src)

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} rgbd_tools )
