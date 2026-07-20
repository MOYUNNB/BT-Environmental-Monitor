cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME main)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# BSP library - Board Support Package (sensor drivers + peripheral drivers)
file(GLOB_RECURSE BSP_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Src/*.c
)
add_library(BSP OBJECT)
target_sources(BSP PRIVATE ${BSP_Src})
target_include_directories(BSP PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
)
target_link_libraries(BSP PUBLIC stm32cubemx)

# APP library - Application layer (business logic)
file(GLOB_RECURSE APP_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/*.c
)
if(APP_Src)
    add_library(APP OBJECT)
    target_sources(APP PRIVATE ${APP_Src})
    target_include_directories(APP PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
    )
    target_link_libraries(APP PUBLIC stm32cubemx BSP)
else()
    add_library(APP INTERFACE)
    target_include_directories(APP INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
    )
    target_link_libraries(APP INTERFACE stm32cubemx)
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    BSP
    APP
    # Add user defined libraries
)

# 启用 printf 浮点数支持 (newlib nano 默认禁用 %f)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,-u,_printf_float)
