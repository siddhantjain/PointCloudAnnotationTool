cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(point_cloud_annotation)

find_package(PCL 1.5 REQUIRED)

set(srcs 
         BNModel.cpp
 		 BNView.cpp
 		 BNSegmentator.cpp
        )

set(incs
         "../includes/BNModel.h"
 		 "../includes/BNView.h"
 		 "../includes/BNSegmentator.h"
 		 "../includes/common.h"
         )

set(BN_LIB "bluenotebook")
add_library("${BN_LIB}" ${srcs})
target_link_libraries("${BN_LIB}" ${PCL_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS} ${incs})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (point_cloud_annotation main.cpp)
target_link_libraries (point_cloud_annotation ${PCL_LIBRARIES} ${BN_LIB} )
#target_link_libraries (point_cloud_annotation ${PCL_LIBRARIES})

