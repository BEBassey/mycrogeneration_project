################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/Src/bh1750.c \
../Components/Src/bh1750_config.c \
../Components/Src/bmp2.c \
../Components/Src/bmp280.c \
../Components/Src/bmp280_config.c \
../Components/Src/bmp2_config.c \
../Components/Src/btn.c \
../Components/Src/btn_config.c \
../Components/Src/digital_output.c \
../Components/Src/encoder.c \
../Components/Src/encoder_config.c \
../Components/Src/lamp.c \
../Components/Src/lamp_config.c \
../Components/Src/lcd.c \
../Components/Src/lcd_config.c \
../Components/Src/led_config.c \
../Components/Src/led_rgb.c \
../Components/Src/led_rgb_config.c \
../Components/Src/pwm_output.c 

OBJS += \
./Components/Src/bh1750.o \
./Components/Src/bh1750_config.o \
./Components/Src/bmp2.o \
./Components/Src/bmp280.o \
./Components/Src/bmp280_config.o \
./Components/Src/bmp2_config.o \
./Components/Src/btn.o \
./Components/Src/btn_config.o \
./Components/Src/digital_output.o \
./Components/Src/encoder.o \
./Components/Src/encoder_config.o \
./Components/Src/lamp.o \
./Components/Src/lamp_config.o \
./Components/Src/lcd.o \
./Components/Src/lcd_config.o \
./Components/Src/led_config.o \
./Components/Src/led_rgb.o \
./Components/Src/led_rgb_config.o \
./Components/Src/pwm_output.o 

C_DEPS += \
./Components/Src/bh1750.d \
./Components/Src/bh1750_config.d \
./Components/Src/bmp2.d \
./Components/Src/bmp280.d \
./Components/Src/bmp280_config.d \
./Components/Src/bmp2_config.d \
./Components/Src/btn.d \
./Components/Src/btn_config.d \
./Components/Src/digital_output.d \
./Components/Src/encoder.d \
./Components/Src/encoder_config.d \
./Components/Src/lamp.d \
./Components/Src/lamp_config.d \
./Components/Src/lcd.d \
./Components/Src/lcd_config.d \
./Components/Src/led_config.d \
./Components/Src/led_rgb.d \
./Components/Src/led_rgb_config.d \
./Components/Src/pwm_output.d 


# Each subdirectory must supply rules for building sources it contributes
Components/Src/%.o Components/Src/%.su Components/Src/%.cyclo: ../Components/Src/%.c Components/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Windows 10/Downloads/finaltouch/NUCLEO-F746ZG-Examples-27b39c1fd4a9187e2e2c8fc9769d54475c6aa7e4/Components/Inc" -O0 -ffunction-sections -fdata-sections -Wall -Wextra -pedantic -Wmissing-include-dirs -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-Src

clean-Components-2f-Src:
	-$(RM) ./Components/Src/bh1750.cyclo ./Components/Src/bh1750.d ./Components/Src/bh1750.o ./Components/Src/bh1750.su ./Components/Src/bh1750_config.cyclo ./Components/Src/bh1750_config.d ./Components/Src/bh1750_config.o ./Components/Src/bh1750_config.su ./Components/Src/bmp2.cyclo ./Components/Src/bmp2.d ./Components/Src/bmp2.o ./Components/Src/bmp2.su ./Components/Src/bmp280.cyclo ./Components/Src/bmp280.d ./Components/Src/bmp280.o ./Components/Src/bmp280.su ./Components/Src/bmp280_config.cyclo ./Components/Src/bmp280_config.d ./Components/Src/bmp280_config.o ./Components/Src/bmp280_config.su ./Components/Src/bmp2_config.cyclo ./Components/Src/bmp2_config.d ./Components/Src/bmp2_config.o ./Components/Src/bmp2_config.su ./Components/Src/btn.cyclo ./Components/Src/btn.d ./Components/Src/btn.o ./Components/Src/btn.su ./Components/Src/btn_config.cyclo ./Components/Src/btn_config.d ./Components/Src/btn_config.o ./Components/Src/btn_config.su ./Components/Src/digital_output.cyclo ./Components/Src/digital_output.d ./Components/Src/digital_output.o ./Components/Src/digital_output.su ./Components/Src/encoder.cyclo ./Components/Src/encoder.d ./Components/Src/encoder.o ./Components/Src/encoder.su ./Components/Src/encoder_config.cyclo ./Components/Src/encoder_config.d ./Components/Src/encoder_config.o ./Components/Src/encoder_config.su ./Components/Src/lamp.cyclo ./Components/Src/lamp.d ./Components/Src/lamp.o ./Components/Src/lamp.su ./Components/Src/lamp_config.cyclo ./Components/Src/lamp_config.d ./Components/Src/lamp_config.o ./Components/Src/lamp_config.su ./Components/Src/lcd.cyclo ./Components/Src/lcd.d ./Components/Src/lcd.o ./Components/Src/lcd.su ./Components/Src/lcd_config.cyclo ./Components/Src/lcd_config.d ./Components/Src/lcd_config.o ./Components/Src/lcd_config.su ./Components/Src/led_config.cyclo ./Components/Src/led_config.d ./Components/Src/led_config.o ./Components/Src/led_config.su ./Components/Src/led_rgb.cyclo ./Components/Src/led_rgb.d ./Components/Src/led_rgb.o ./Components/Src/led_rgb.su ./Components/Src/led_rgb_config.cyclo ./Components/Src/led_rgb_config.d ./Components/Src/led_rgb_config.o ./Components/Src/led_rgb_config.su ./Components/Src/pwm_output.cyclo ./Components/Src/pwm_output.d ./Components/Src/pwm_output.o ./Components/Src/pwm_output.su

.PHONY: clean-Components-2f-Src

