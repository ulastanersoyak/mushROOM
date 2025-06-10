cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

set(CMAKE_C_COMPILER /opt/stm32cubeclt/GNU-tools-for-STM32/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER /opt/stm32cubeclt/GNU-tools-for-STM32/bin/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER /opt/stm32cubeclt/GNU-tools-for-STM32/bin/arm-none-eabi-gcc)

# Include toolchain file before project()
include("cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_PROJECT_NAME mushROOM)
project(${CMAKE_PROJECT_NAME})

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Core project settings
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

set(BME280_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/Peripherals/bme280/Src/bme280.c
)

# Add sources to executable
target_sources(${PROJECT_NAME} PRIVATE
        ${BME280_SOURCES}
)

# Add include paths
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Peripherals/bme280/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)