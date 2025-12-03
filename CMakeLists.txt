cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(uart_async_rxtxtasks)

set(COMPONENT_DIRS components config)
set(EXTRA_COMPONENT_DIRS components)


set(COMPONENTS 
    main
    servo_driver
    uart_comm
    icm20948
    other
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/config)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/components)

set(EXTRA_COMPONENT_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/components
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)