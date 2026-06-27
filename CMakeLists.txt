cmake_minimum_required(VERSION 3.22)

set(CMAKE_PROJECT_NAME C_Board_Framework_Old)

enable_language(C ASM)

project(${CMAKE_PROJECT_NAME})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

set(ROBOT_BOARD "GIMBAL" CACHE STRING "Firmware role: GIMBAL or CHASSIS")
set_property(CACHE ROBOT_BOARD PROPERTY STRINGS GIMBAL CHASSIS)

if(ROBOT_BOARD STREQUAL "GIMBAL")
    set(ROBOT_BOARD_DEFINE ROBOT_BOARD=ROBOT_BOARD_GIMBAL)
    set(ROBOT_BOARD_OUTPUT_NAME "${CMAKE_PROJECT_NAME}_GIMBAL")
elseif(ROBOT_BOARD STREQUAL "CHASSIS")
    set(ROBOT_BOARD_DEFINE ROBOT_BOARD=ROBOT_BOARD_CHASSIS)
    set(ROBOT_BOARD_OUTPUT_NAME "${CMAKE_PROJECT_NAME}_CHASSIS")
else()
    message(FATAL_ERROR "ROBOT_BOARD must be GIMBAL or CHASSIS")
endif()
message("Robot board: " ${ROBOT_BOARD})

add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "${ROBOT_BOARD_OUTPUT_NAME}")

add_subdirectory(cmake/stm32cubemx)
add_subdirectory(Middlewares/CMSIS)
add_subdirectory(Middlewares/SEGGER_RTT)
add_subdirectory(User)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        USE_HAL_DRIVER ARM_MATH_CM4 __FPU_PRESENT=1U
        HAL_ASSERT_DISABLE=1 ARM_MATH_MATRIX_CHECK=0 ARM_MATH_ROUNDING=0
        ${ROBOT_BOARD_DEFINE}
        User_Release
        $<$<CONFIG:Debug>:DEBUG_MODE> $<$<CONFIG:Release>:RELEASE_MODE>
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        ${CPU_FLAGS}
        -Wall -Wextra -Wno-comment -Wno-unused-parameter -Wno-unused-but-set-variable
        -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
        -fdata-sections -ffunction-sections
        $<$<CONFIG:Debug>:-Og -g3 -fno-omit-frame-pointer -fno-lto>
        $<$<CONFIG:Release>:-O3 -ffast-math -funroll-loops>
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        ${CPU_FLAGS}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,--relax
        -Wl,--sort-common=descending
        $<$<CONFIG:Release>:-flto=auto>
)

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        m
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${ROBOT_BOARD_OUTPUT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${ROBOT_BOARD_OUTPUT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${HEX_FILE}"
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${BIN_FILE}"
        COMMENT "Generating Output Files..."
)
