cmake_minimum_required(VERSION 3.15)
project(egocylindrical)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# ament_cmake dependencies
set(dependencies
      cv_bridge
      geometry_msgs
      image_geometry
      image_transport
      egocylindrical_msgs
      # nodelet
      pcl_ros
      rclcpp
      rclpy
      sensor_msgs
      std_msgs
      # tf
      tf2
      tf2_msgs
      tf2_ros
      tf2_sensor_msgs
      tf2_geometry_msgs
      rosidl_default_generators
      # message_generation
      # dynamic_reconfigure
      # benchmarking_tools #Probably just temporary to avoid adding any custom dependencies to this package.
)

# Dependencies
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED)
find_package(PCL    REQUIRED)
find_package(OpenMP)

if(0)
    set(SHOW_VECTORIZE_INFO "-fopt-info-vec-all=vectorized.txt")
else()
    set(SHOW_VECTORIZE_INFO "")
endif()


if(0) #if on arm...
    set(PLATFORM_FLAGS "PIPS_ON_ARM=arm")
else()
    set(PLATFORM_FLAGS "")
endif()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder

# generate_dynamic_reconfigure_options(
#   cfg/Propagator.cfg
#   cfg/RangeImageGenerator.cfg
#   cfg/Filter.cfg
#   cfg/RangeImageInflatorGenerator.cfg
#   cfg/FloorImageGenerator.cfg
#   )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES egocylindrical_propagator range_image_core can_image_core floor_image_core point_cloud_core projected_point_cloud_core semantic_point_cloud_core normal_point_cloud_core normal_point_cloud_floor_only_core point_transformer depth_image_remapper range_image_converter depth_image_inserter laser_scan_inserter sensors ecwrapper_buffer
#   CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport roscpp rospy sensor_msgs std_msgs tf tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs 
#  DEPENDS system_lib
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

###############
## Libraries ##
###############

# RANGE_IMAGE_CORE
add_library(range_image_core 
            src/range_image_core.cpp)
ament_target_dependencies(range_image_core 	
                           ${dependencies}
                           ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_compile_options(range_image_core PRIVATE -ftree-vectorize -fopt-info-loop-optimized=vec_range_image_core.txt -funsafe-math-optimizations -fno-math-errno -ffinite-math-only #-ffinite-math-only and -fno-trapping-math.
   )
target_link_libraries(range_image_core
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

# CAN_IMAGE_CORE
add_library(can_image_core 
            src/can_image_core.cpp)
ament_target_dependencies(can_image_core 
                  ${dependencies}
                  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_compile_options(can_image_core PRIVATE -ftree-vectorize -fopt-info-loop-optimized=vec_can_image_core.txt -funsafe-math-optimizations -fno-math-errno -ffinite-math-only #-ffinite-math-only and -fno-trapping-math.
   )
target_link_libraries(can_image_core
   ${OpenCV_LIBRARIES}
   ${OpenMP_CXX_FLAGS}
   )

#############
## Install ##
#############


install(
  TARGETS range_image_core can_image_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)


# install(
#   TARGETS 
#       DESTINATION lib/${PROJECT_NAME}
# )


# install(DIRECTORY include/ 
#   DESTINATION include/${PROJECT_NAME}
# )

# install(DIRECTORY cfg launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# ament_export_dependencies(${dependencies})  
# ament_export_include_directories("include/${PROJECT_NAME}")
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()