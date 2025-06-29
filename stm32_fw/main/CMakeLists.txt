#############################################################################################################################
# file:  CMakeLists.txt
# brief: Template "CMakeLists.txt" for building of executables and static libraries.
#
# usage: Edit "VARIABLES"-section to suit project requirements.
#        For debug build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
#          make -C Debug VERBOSE=1
#        For release build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Release -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
#          make -C Release VERBOSE=1
#############################################################################################################################
cmake_minimum_required(VERSION 3.20)

###################### CONSTANTS ######################################
set (PROJECT_TYPE_EXECUTABLE          "exe")
set (PROJECT_TYPE_STATIC_LIBRARY      "static-lib")
set (MCPU_CORTEX_M0				      "-mcpu=cortex-m0")
set (MCPU_CORTEX_M0PLUS				  "-mcpu=cortex-m0plus")
set (MCPU_CORTEX_M3				      "-mcpu=cortex-m3")
set (MCPU_CORTEX_M4				      "-mcpu=cortex-m4")
set (MCPU_CORTEX_M7				      "-mcpu=cortex-m7")
set (MCPU_CORTEX_M33				  "-mcpu=cortex-m33")
set (MCPU_CORTEX_M55				  "-mcpu=cortex-m55")
set (MCPU_CORTEX_M85				  "-mcpu=cortex-m85")
set (MFPU_FPV4_SP_D16                 "-mfpu=fpv4-sp-d16")
set (MFPU_FPV5_D16                    "-mfpu=fpv5-d16")
set (RUNTIME_LIBRARY_REDUCED_C        "--specs=nano.specs")
set (RUNTIME_LIBRARY_STD_C            "")
set (RUNTIME_LIBRARY_SYSCALLS_MINIMAL "--specs=nosys.specs")
set (RUNTIME_LIBRARY_SYSCALLS_NONE    "")
set (MFLOAT_ABI_SOFTWARE              "-mfloat-abi=soft")
set (MFLOAT_ABI_HARDWARE              "-mfloat-abi=hard")
set (MFLOAT_ABI_MIX                   "-mfloat-abi=softfp")
set (CUBE_DIR                         "/media/stanislav/projects/projects/STM32CubeG4")
set (FREERTOS_DIR                     "/media/stanislav/projects/projects/FreeRTOS/FreeRTOS")
set (MEMS_DIR                         "/media/stanislav/projects/projects/X-CUBE-MEMS1")

#######################################################################

###################### VARIABLES ######################################
set (PROJECT_NAME             "morbo_b")
set (PROJECT_TYPE             "exe")
set (LINKER_SCRIPT            "../STM32G431KBTX_FLASH.ld")
set (MCPU                     "-mcpu=Cortex-M4")
set (MFPU                     "-mfpu=fpv4-sp-d16")
set (MFLOAT_ABI               "-mfloat-abi=hard")
set (RUNTIME_LIBRARY          "--specs=nano.specs")
set (RUNTIME_LIBRARY_SYSCALLS "--specs=nosys.specs")
set (MFX_LIB                   ${MEMS_DIR}/Middlewares/ST/STM32_MotionFX_Library/Lib/MotionFX_CM4F_wc32_ot_hard.a)

set (PROJECT_SOURCES
	# LIST SOURCE FILES HERE
	Startup/startup_stm32g431kbtx.s
	Sources/platform/main.c
	Sources/platform/syscalls.c
	Sources/platform/sysmem.c
	Sources/platform/stm32g4xx_it.c
	Sources/platform/system_stm32g4xx.c
	Sources/platform/stm32g4xx_hal_msp.c
	Sources/drivers/serial.c
	Sources/drivers/i2c.c
	# Sources/drivers/adc.c
	# Sources/drivers/spi.c
	# Sources/drivers/timer.c
	Sources/drivers/system.c
	Sources/drivers/gpio.c
	Sources/drivers/pca9685.c
	Sources/app/controller.c
	# Sources/app/monitor.c
	Sources/app/host_interface.c
	# Sources/app/imu_reader.c
	Sources/app/mhelpers.c
	Sources/app/engine_control.c
	Sources/app/scenarios.c
	Sources/app/calibs.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c
	# ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
	# ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_crc.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_crc_ex.c
	# ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
	# ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
	# ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
	# ${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
	${CUBE_DIR}/Drivers/BSP/STM32G4xx_Nucleo/stm32g4xx_nucleo.c
	${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F/port.c	
	${FREERTOS_DIR}/Source/list.c
	${FREERTOS_DIR}/Source/stream_buffer.c
	${FREERTOS_DIR}/Source/tasks.c
	${FREERTOS_DIR}/Source/timers.c
	${FREERTOS_DIR}/Source/queue.c
	#${FREERTOS_DIR}/Source/portable/MemMang/heap_4.c
	${MEMS_DIR}/Drivers/BSP/Components/lsm6dsox/lsm6dsox.c
	${MEMS_DIR}/Drivers/BSP/Components/lsm6dsox/lsm6dsox_reg.c
	)

set (PROJECT_DEFINES
	# LIST COMPILER DEFINITIONS HERE
	)

set (PROJECT_INCLUDES
	# LIST INCLUDE DIRECTORIES HERE
	${CUBE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
	${CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
	${CUBE_DIR}/Drivers/CMSIS/Include
	${CUBE_DIR}/Drivers/BSP/STM32G4xx_Nucleo
	Sources
	Sources/platform
	Sources/drivers
	Sources/app
	${FREERTOS_DIR}/Source/include
	${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F
	${MEMS_DIR}/Middlewares/ST/STM32_MotionFX_Library/Inc
	${MEMS_DIR}/Middlewares/ST/STM32_MotionAC_Library/Inc
	${MEMS_DIR}/Drivers/BSP/Components/lsm6dsox
	)

############ MODIFY ACCORDING TO REQUIREMENTS) ########################

#######################################################################

################## PROJECT SETUP ######################################
project(${PROJECT_NAME})
enable_language(ASM)

if (${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_EXECUTABLE})
  add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
  add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
  target_link_libraries(${PROJECT_NAME} ${MFX_LIB})
elseif (${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_STATIC_LIBRARY})
  add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
endif()

add_compile_definitions (${PROJECT_DEFINES})
include_directories (${PROJECT_INCLUDES})

set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -DSTM32G431xx -DUSE_HAL_DRIVER -DUSE_STM32G4XX_NUCLEO_32 -DUSE_NUCLEO_32 -Os -mthumb -Wall -Werror")
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
