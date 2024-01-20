set(STM32F0xx_Driver  ${CMAKE_CURRENT_SOURCE_DIR}/STM32F0xx_HAL_Driver)

add_library(driver_stm32f0xx EXCLUDE_FROM_ALL OBJECT
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_adc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_adc_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_can.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_cec.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_comp.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_cortex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_crc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_crc_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_dac.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_dac_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_dma.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_exti.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_flash.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_flash_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_gpio.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_i2c.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_i2c_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_i2s.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_irda.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_iwdg.c
    #${STM32F0xx_Driver}/Src/stm32f0xx_hal_msp_template.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_pcd.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_pcd_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_pwr.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_pwr_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_rcc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_rcc_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_rtc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_rtc_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_smartcard.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_smartcard_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_smbus.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_spi.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_spi_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_tim.c
    #${STM32F0xx_Driver}/Src/stm32f0xx_hal_timebase_rtc_alarm_template.c
    #${STM32F0xx_Driver}/Src/stm32f0xx_hal_timebase_rtc_wakeup_template.c
    #${STM32F0xx_Driver}/Src/stm32f0xx_hal_timebase_tim_template.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_tim_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_tsc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_uart.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_uart_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_usart.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_usart_ex.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_hal_wwdg.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_adc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_comp.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_crc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_crs.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_dac.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_dma.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_exti.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_gpio.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_i2c.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_pwr.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_rcc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_rtc.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_spi.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_tim.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_usart.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_usb.c
    ${STM32F0xx_Driver}/Src/stm32f0xx_ll_utils.c
)

target_include_directories(driver_stm32f0xx
    SYSTEM PUBLIC
        ${STM32F0xx_Driver}/Inc
        ${STM32F0xx_Driver}/Inc/Legacy
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/DSP/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Device/ST/STM32F0xx/Include
)

set(MCSDK_FOLDER  ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK)

add_library(driver_st_mcsdk EXCLUDE_FROM_ALL OBJECT
    # ${MCSDK_FOLDER}/MCLib/Any/Src/bus_voltage_sensor.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/circle_limitation.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/digital_output.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/encoder_speed_pos_fdbk.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/enc_align_ctrl.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/fast_div.c
    # #${MCSDK_FOLDER}/MCLib/Any/Src/feed_forward_ctrl.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/flux_weakening_ctrl.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/gap_gate_driver_ctrl.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/hall_speed_pos_fdbk.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/inrush_current_limiter.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/max_torque_per_ampere.c
    # #${MCSDK_FOLDER}/MCLib/Any/Src/mcp.c
    ${MCSDK_FOLDER}/MCLib/Any/Src/mcpa.c
    # #${MCSDK_FOLDER}/MCLib/Any/Src/mcptl.c
    ${MCSDK_FOLDER}/MCLib/Any/Src/motor_control_protocol_sixstep.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/motor_power_measurement.c
    # #${MCSDK_FOLDER}/MCLib/Any/Src/ntc_temperature_sensor.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/open_loop.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/pid_regulator.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/pqd_motor_power_measurement.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/pwm_common.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/pwm_curr_fdbk_ovm.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/ramp_ext_mngr.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/revup_ctrl.c
    # #${MCSDK_FOLDER}/MCLib/Any/Src/r_divider_bus_voltage_sensor.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/speed_pos_fdbk.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/speed_torq_ctrl.c
    ${MCSDK_FOLDER}/MCLib/Any/Src/state_machine.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/sto_cordic_speed_pos_fdbk.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/sto_pll_speed_pos_fdbk.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/trajectory_ctrl.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/virtual_bus_voltage_sensor.c
    # ${MCSDK_FOLDER}/MCLib/Any/Src/virtual_speed_sensor.c

    # ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_align_and_go.c
    ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_PID.c
    # ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_SetFunctions_3pwm.c
    ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_SetFunctions_6pwm.c
    # ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_SetFunctions_cm.c
    # ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_SetFunctions_fd.c
    # ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_SetFunctions_qs.c
    ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_SetFunctions_std.c
    ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_SetFunctions_vm.c
    ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_synch.c
    ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_synch_hall.c
    # ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_synch_sl.c
    # ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_table_3pwm.c
    ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_table_6pwm.c
    # ${MCSDK_FOLDER}/MCLib6Step/Any/Src/6step_user_config.c

    ${MCSDK_FOLDER}/MCLib6Step/F0xx/Src/6step_service_f0xx.c
)

target_include_directories(driver_st_mcsdk
    SYSTEM PUBLIC
        ${MCSDK_FOLDER}/MCLib/Any/Inc
        ${MCSDK_FOLDER}/MCLib/F0xx/Inc
        ${MCSDK_FOLDER}/MCLib6Step/Any/Inc
        ${MCSDK_FOLDER}/MCLib6Step/F0xx/Inc
)