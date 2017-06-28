cmake_minimum_required(VERSION 2.8.3)

## enable c++11 mode
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

project(hanp_to_iri_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hanp_msgs
  iri_perception_msgs
)

# find_package(Boost REQUIRED COMPONENTS thread)

## declare catkin package
catkin_package(
#  INCLUDE_DIRS
#    include
#   LIBRARIES
#     hanp_to_iri_converter
  CATKIN_DEPENDS
    roscpp
    hanp_msgs
    iri_perception_msgs
#   DEPENDS
#     system_lib
)



## build ##

# include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   include
# )

# declare a c++ executable
add_executable(${PROJECT_NAME}
  src/hanp_to_iri_converter.cpp
)

# cmake target dependencies of the c++ executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ executable against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



## install ##

# executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# cpp-header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# other files for installation (launch, data, bag files)
foreach(dir launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
