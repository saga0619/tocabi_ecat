cmake_minimum_required(VERSION 3.0.2)
project(tocabi_ecat)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O)

# ADD_DEFINITIONS(-DTIME_CHECK)
# ADD_DEFINITIONS(-DECAT_DC)

execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RES OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- kernel version : " ${UNAME_RES})
execute_process(COMMAND xeno-config --posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

execute_process(COMMAND xeno-config --posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
message(" xeno flags : " ${XENO_CFLAGS})
string(FIND ${UNAME_RES} "xenomai" wsloc)



if(${wsloc} EQUAL -1 )
  message(" simulation only compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_SIMULATION)
  set(COMPILE_SIM TRUE)
else()
  message(" xenomai detected realrobot compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_REALROBOT)
  set(COMPILE_REAL TRUE)
endif() 


find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(tocabi_lib COMPONENTS)


catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${tocabi_lib_INCLUDE_DIRS}
  /usr/local/include/soem
)

if(${COMPILE_REAL})



link_directories(
)

add_executable(tocabi_ecat src/master_alone.cpp src/ecat_master.cpp  src/ecat_settings.cpp)
add_executable(tocabi_ecat_lower 
src/master_lower.cpp src/ecat_master.cpp src/ecat_settings.cpp)
add_executable(tocabi_ecat_upper 
src/master_upper.cpp src/ecat_master.cpp src/ecat_settings.cpp)

# add_executable(red_test src/red_test.c)

add_executable(watcher src/watcher.cpp)

# add_executable(watcher2 src/watcher_status.cpp)


target_link_libraries(tocabi_ecat
-lsoem
-lpthread

  ${XENO_CFLAGS}
  ${XENO_LDFLAGS}
)
target_link_libraries(tocabi_ecat_lower
  -lsoem
  -lpthread
  ${XENO_CFLAGS}
  ${XENO_LDFLAGS}
)
target_link_libraries(tocabi_ecat_upper
  -lsoem 
  -lpthread
  ${XENO_CFLAGS}
  ${XENO_LDFLAGS}
)



endif()

if(${COMPILE_SIM})

add_executable(watcher src/watcher.cpp)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
