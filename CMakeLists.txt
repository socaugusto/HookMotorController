cmake_minimum_required(VERSION 3.15)

project(MotorControl C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(vendor/stm deps/vendor/stm)

add_executable(Hook_Motor
    startup/startup_stm32f031c6tx.s
    src/main.c
    src/stm32f0xx_hal_msp.c
    src/stm32f0xx_it.c
    src/stm32f0xx_mc_it.c
    src/system_stm32f0xx.c

    src/user/syscalls.c
    src/user/sysmem.c

    src/encoding_checksum.cpp

    src/aspep.c
    src/mc_config.c
    src/mcp_config.c
    src/mc_configuration_registers.c
    src/mc_hook_remote_config.c
    src/mc_interface.c
    src/mc_math.c
    src/mc_tasks.c
    src/mcp_config.c
    src/motorcontrol.c
    src/register_interface.c
    src/stlink_mcptl.c
    src/usart_aspep_driver.c
)

target_compile_options(Hook_Motor
    PRIVATE
        -Wextra
        -Wpedantic
        -Wall
        -Werror
        -Wno-missing-field-initializers
        #-Wmissing-declarations
)

# target_compile_definitions(Hook_Motor
#     PRIVATE
#         -DMC_HAL_IS_USED
# )

target_include_directories(Hook_Motor
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/
)

target_include_directories(driver_stm32f0xx
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/
)

target_include_directories(driver_st_mcsdk
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/
)

target_compile_definitions(driver_stm32f0xx
    PUBLIC
        -DUSE_HAL_DRIVER
        -DARM_MATH_CM0
        -DSTM32F031x6
)

target_link_libraries(driver_st_mcsdk
    PRIVATE
        driver_stm32f0xx
)

target_link_libraries(Hook_Motor
    PRIVATE
        driver_st_mcsdk
        driver_stm32f0xx
)

set_target_properties(Hook_Motor PROPERTIES SUFFIX ".elf")

add_custom_target(Hook_Motor_Flash
    COMMAND pushd ${CMAKE_BINARY_DIR}
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -c "program Hook_Motor.elf verify reset exit"
    DEPENDS Hook_Motor
)