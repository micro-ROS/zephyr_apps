include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "Linux")

# set(CMAKE_SYSROOT /workspaces/uros_ws_olimexzephyr/uros_ws/firmware/olimex_e407_extensions/microros/../..)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# External transports
# set(EXTERNAL_TRANSPORT_HEADER "@EXTERNAL_TRANSPORT_HEADER@" CACHE PATH "" FORCE)
# set(EXTERNAL_TRANSPORT_SRC "@EXTERNAL_TRANSPORT@" CACHE PATH "" FORCE) 

SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)
# SET(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")

# Makefile flags
#set(CROSSDEV @CROSS_COMPILE_PREFIX@)
#set(ARCH_CPU_FLAGS @ARCH_CPU_FLAGS@)
#set(ARCH_OPT_FLAGS @ARCH_OPT_FLAGS@)


set(CMAKE_C_COMPILER /workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyr-sdk-0.11.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc)
set(CMAKE_CXX_COMPILER /workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyr-sdk-0.11.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-g++)

set(CMAKE_C_FLAGS_INIT "-I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/build/zephyr/include/generated -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/soc/arm/st_stm32/stm32f4 -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/lib/libc/newlib/include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyr-sdk-0.11.1/arm-zephyr-eabi/arm-zephyr-eabi/include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/drivers -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/ext/hal/cmsis/Core/Include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/modules/hal/st/sensor/vl53l1x/api/core/inc -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/drivers/sensor/vl53l1x -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/Legacy -DKERNEL -D__ZEPHYR__=1 -D_FORTIFY_SOURCE=2 -DBUILD_VERSION=zephyr-v2.1.0-1635-g53abcc64ccb6 -D__LINUX_ERRNO_EXTENSIONS__ -D__PROGRAM_START -DSTM32F407xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DCORE_CM4 -DHSE_VALUE=12000000 -Os -imacros/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/build/zephyr/include/generated/autoconf.h -ffreestanding -fno-common -g -mthumb -mcpu=cortex-m4 -imacros/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/include/toolchain/zephyr_stdint.h -Wall -Wformat -Wformat-security -Wno-format-zero-length -Wno-main -Wno-address-of-packed-member -Wno-pointer-sign -Wpointer-arith -Wno-unused-but-set-variable -Werror=implicit-int -fno-asynchronous-unwind-tables -fno-pie -fno-pic -fno-strict-overflow -fno-reorder-functions -fno-defer-pop -fmacro-prefix-map=/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/olimex_e407_extensions=CMAKE_SOURCE_DIR -fmacro-prefix-map=/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr=ZEPHYR_BASE -ffunction-sections -fdata-sections -mabi=aapcs -march=armv7e-m -std=c99  -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/olimex_e407_extensions/microros/../../zephyrproject/zephyr/include/posix" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/build/zephyr/include/generated -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/soc/arm/st_stm32/stm32f4 -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/lib/libc/newlib/include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyr-sdk-0.11.1/arm-zephyr-eabi/arm-zephyr-eabi/include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/drivers -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/ext/hal/cmsis/Core/Include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/modules/hal/st/sensor/vl53l1x/api/core/inc -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/drivers/sensor/vl53l1x -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/Legacy -DKERNEL -D__ZEPHYR__=1 -D_FORTIFY_SOURCE=2 -DBUILD_VERSION=zephyr-v2.1.0-1635-g53abcc64ccb6 -D__LINUX_ERRNO_EXTENSIONS__ -D__PROGRAM_START -DSTM32F407xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DCORE_CM4 -DHSE_VALUE=12000000 -Os -fcheck-new -std=c++17 -Wno-register -fno-exceptions -fno-rtti -imacros/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/build/zephyr/include/generated/autoconf.h -ffreestanding -fno-common -g -mthumb -mcpu=cortex-m4 -imacros/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr/include/toolchain/zephyr_stdint.h -Wall -Wformat -Wformat-security -Wno-format-zero-length -Wno-main -Wno-address-of-packed-member -Wpointer-arith -Wno-unused-but-set-variable -fno-asynchronous-unwind-tables -fno-pie -fno-pic -fno-strict-overflow -fno-reorder-functions -fno-defer-pop -fmacro-prefix-map=/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/olimex_e407_extensions=CMAKE_SOURCE_DIR -fmacro-prefix-map=/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/zephyrproject/zephyr=ZEPHYR_BASE -ffunction-sections -fdata-sections -mabi=aapcs -march=armv7e-m -I/workspaces/uros_ws_olimexzephyr/uros_ws/firmware/olimex_e407_extensions/microros/../../zephyrproject/zephyr/include/posix" CACHE STRING "" FORCE)
    
set(__BIG_ENDIAN__ 0)