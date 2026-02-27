cmake_minimum_required(VERSION 3.5)
project(serial)
#Find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#If Linux, add rt and pthread
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)


#Sources

set(serial_SRCS
src/serial.cc
include/serial/serial.h
include/serial/v8stdint.h
)
#If ubuntu

if(UNIX AND NOT APPLE)
list(APPEND serial_SRCS src/impl/unix.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
endif()
#Add serial library
include_directories(include)
add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)
#Uncomment for example

add_executable(serial_example examples/serial_example.cc)
add_dependencies(serial_example ${PROJECT_NAME})
target_link_libraries(serial_example ${PROJECT_NAME})
#Include headers
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

#Install executable
install(
    DIRECTORY include/serial
    DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
EXPORT  ${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)
#Install headers

install(FILES include/serial/serial.h include/serial/v8stdint.h
DESTINATION include/serial)
#Tests

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_package()
