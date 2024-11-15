cmake_minimum_required(VERSION 3.10)
project(legged_ti5_hw)
#add_definitions(-DTIMEPRINT=TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
        tf
        can_control
        )
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
        INCLUDE_DIRS    
        include
        CATKIN_DEPENDS
        can_control
        roscpp
        legged_hw
)

###########
## Build ##
###########

# add
# add_subdirectory(third-party)
# add_subdirectory(common)
# add_subdirectory(user)
# find_package(Boost REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${Eigen3_INCLUDE_DIRS}
        include/legged_ti5_hw
        ${CMAKE_CURRENT_SOURCE_DIR}/../can_control/include # 添加 can_control 包的头文件路径
        ${CMAKE_CURRENT_SOURCE_DIR}/../can_control/motor # 添加 can_control 包的头文件路径
)

## Declare cpp executables
add_executable(legged_ti5_hw 
        src/legged_ti5_hw.cpp
        src/ti5HW.cpp
        )

add_library(motor ${SOURCES} 
src/utilities.cpp)

## Specify libraries to link executable targets against
target_link_libraries(legged_ti5_hw
        ${catkin_LIBRARIES}
        motor
        ${CMAKE_CURRENT_SOURCE_DIR}/../can_control/lib/libcontrolcan.so
        #${CMAKE_CURRENT_SOURCE_DIR}/../can_control/lib/libmylibscan.so
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../devel/lib/libcan_control.so
        ) 

target_compile_options(legged_ti5_hw PRIVATE -g)



# set(CMAKE_CXX_STANDARD 17)
#总结来说，这段CMake代码的作用是创建一个名为motor的库，该库包含了多个源文件和头文件，以及一个额外的源文件src/utilities.cpp。
#然后，它将这个库与另一个名为soem的库链接起来，以便在编译时将soem库的功能集成到motor库中。
#这样，其他项目或模块就可以使用motor库来访问这些功能。
# set(SOURCES app/motor_control.c
#             app/motor_control.h
#             app/math_ops.c
#             app/math_ops.h
#             app/transmit.cpp
#             app/transmit.h
#             app/config.h
#             app/queue.h 
#             app/command.cpp 
#             app/command.h)

# add_library(motor ${SOURCES}
#         src/utilities.cpp)

#target_link_libraries(motor soem)

# add_executable(master_stack_test main.cpp)
#target_link_libraries(legged_ti5_hw motor soem cpp-readline)
# target_link_options(legged_ti5_hw PRIVATE -g)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " legged_ti5_hw)
    add_clang_tooling(
            TARGETS legged_ti5_hw
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_ti5_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_ti5_hw
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation

install(TARGETS legged_ti5_hw
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


