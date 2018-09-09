cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(point_cloud_annotation)

find_package(PCL 1.5 REQUIRED)

set(srcs 
         ../src/BNModel.cpp
 		 ../src/BNView.cpp
 		 ../src/BNSegmentator.cpp
 		 ../src/BNState.cpp
 		 ../src/BNLabelStore.cpp
 		 ../src/BNConfigReader.cpp
 		 ../src/BNPointCloudReader.cpp
        )

set(incs
		 "../src/includes/BNState.h"
         "../src/includes/BNModel.h"
 		 "../src//includes/BNView.h"
 		 "../src/includes/BNSegmentator.h"
 		 "../src/includes/common.h"
 		 "../src/includes/BNLabelStore.h"
 		 "../src/includes/BNLabel.h"
 		 "../src/includes/BNConfigReader.h"
 		 "../src/includes/BNPointCloudReader.h"
         )

set(BN_LIB "bluenotebook")
add_library("${BN_LIB}" ${srcs})
target_link_libraries("${BN_LIB}" ${PCL_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS} ${incs})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (point_cloud_annotation ../src/main.cpp)
target_link_libraries (point_cloud_annotation ${PCL_LIBRARIES} ${BN_LIB} )
#target_link_libraries (point_cloud_annotation ${PCL_LIBRARIES})

