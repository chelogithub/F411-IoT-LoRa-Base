################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/mggarcia/OneDrive\ -\ Universidad\ Nacional\ de\ la\ Matanza/xx-DATA\ ELECTRONICA/STM32/LIBRERIAS\ C/ModBUS/ModBUS_Chelo.c 

OBJS += \
./ModBUS/ModBUS_Chelo.o 

C_DEPS += \
./ModBUS/ModBUS_Chelo.d 


# Each subdirectory must supply rules for building sources it contributes
ModBUS/ModBUS_Chelo.o: C:/Users/mggarcia/OneDrive\ -\ Universidad\ Nacional\ de\ la\ Matanza/xx-DATA\ ELECTRONICA/STM32/LIBRERIAS\ C/ModBUS/ModBUS_Chelo.c ModBUS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mggarcia/OneDrive - Universidad Nacional de la Matanza/xx-DATA ELECTRONICA/STM32/LIBRERIAS C/ESP8266" -I"C:/Users/mggarcia/OneDrive - Universidad Nacional de la Matanza/xx-DATA ELECTRONICA/STM32/LIBRERIAS C/HTTP" -I"C:/Users/mggarcia/OneDrive - Universidad Nacional de la Matanza/xx-DATA ELECTRONICA/STM32/LIBRERIAS C/ModBUS" -I"C:/Users/mggarcia/OneDrive - Universidad Nacional de la Matanza/xx-DATA ELECTRONICA/STM32/LIBRERIAS C/STM32_ETH_W5100" -I"C:/Users/mggarcia/OneDrive - Universidad Nacional de la Matanza/xx-DATA ELECTRONICA/STM32/LIBRERIAS C/STRING" -I"C:/Users/mggarcia/OneDrive - Universidad Nacional de la Matanza/xx-DATA ELECTRONICA/STM32/LIBRERIAS C/LoRa" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ModBUS

clean-ModBUS:
	-$(RM) ./ModBUS/ModBUS_Chelo.d ./ModBUS/ModBUS_Chelo.o ./ModBUS/ModBUS_Chelo.su

.PHONY: clean-ModBUS

