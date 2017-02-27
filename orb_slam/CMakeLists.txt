cmake_minimum_required( VERSION 2.8 )
project( slam )

set( CMAKE_BUILD_TYPE Release )
#set( CMAKE_BUILD_TYPE Debug )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )
#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread -g" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( PCL REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(
	${PCL_DEFINITIONS}
    -DEIGEN_DONT_VECTORIZE
    -DEIGEN_DISABLE_UNALIGNED_ASSERT
    )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR} )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

include_directories(
    ${PROJECT_SOURCE_DIR}/3rd_part/dbow2/DUtils/
    ${PROJECT_SOURCE_DIR}/3rd_part/dbow2/
    ${PROJECT_SOURCE_DIR}/inc/
    ${EIGEN3_INCLUDE_DIR}
    )

add_subdirectory( ${PROJECT_SOURCE_DIR}/3rd_part/dbow2/ )
add_subdirectory( ${PROJECT_SOURCE_DIR}/src/ )
add_subdirectory( ${PROJECT_SOURCE_DIR}/test/ )

