# Имя проекта
#-------------------------------------------------------------------------------
TARGET  = target

# Используемые модули библиотеки периферии
#-------------------------------------------------------------------------------
PERIPHDRIVERS += misc
# PERIPHDRIVERS += stm32f10x_adc
# PERIPHDRIVERS += stm32f10x_bkp
# PERIPHDRIVERS += stm32f10x_can
# PERIPHDRIVERS += stm32f10x_cec
# PERIPHDRIVERS += stm32f10x_crc
# PERIPHDRIVERS += stm32f10x_dbgmcu
PERIPHDRIVERS += stm32f10x_dma
PERIPHDRIVERS += stm32f10x_exti
# PERIPHDRIVERS += stm32f10x_flash
# PERIPHDRIVERS += stm32f10x_fsmc
PERIPHDRIVERS += stm32f10x_gpio
# PERIPHDRIVERS += stm32f10x_i2c
# PERIPHDRIVERS += stm32f10x_iwdg
# PERIPHDRIVERS += stm32f10x_pwr
PERIPHDRIVERS += stm32f10x_rcc
# PERIPHDRIVERS += stm32f10x_rtc
# PERIPHDRIVERS += stm32f10x_sdio
# PERIPHDRIVERS += stm32f10x_spi
PERIPHDRIVERS += stm32f10x_tim
PERIPHDRIVERS += stm32f10x_usart
# PERIPHDRIVERS += stm32f10x_wwdg
# PERIPHDRIVERS += stm32f10x_it

# Дефайны
#-------------------------------------------------------------------------------
DEFINES += USE_STDPERIPH_DRIVER
# DEFINES += STM32F10X_LD # STM32 Low density devices
DEFINES += STM32F10X_LD_VL # STM32 Low density Value Line devices
# DEFINES += STM32F10X_MD # STM32 Medium density devices
# DEFINES += STM32F10X_MD_VL # STM32 Medium density Value Line devices
# DEFINES += STM32F10X_HD # STM32 High density devices
# DEFINES += STM32F10X_HD_VL # STM32 High density value line devices
# DEFINES += STM32F10X_XL # STM32 XL-density devices
# DEFINES += STM32F10X_CL # STM32 Connectivity line devices

DEFINES += GCC_ARMCM3
DEFINES += VECT_TAB_FLASH

# startup файл
#-------------------------------------------------------------------------------
STARTUPDIR = $(LIBS_PATH)/startup

# STARTUP = $(STARTUPDIR)/startup_stm32f10x_md_vl.s
STARTUP = $(STARTUPDIR)/startup_stm32f10x_ld_vl.s

# Скрипт линкера
#-------------------------------------------------------------------------------
LDSCR_PATH = $(LIBS_PATH)/ld-scripts
LDSCRIPT   = stm32f100rb.ld
# LDSCRIPT   = stm32f100c4.ld

# Инструменты
#-------------------------------------------------------------------------------
AS = arm-none-eabi-gcc
CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size
OD = arm-none-eabi-objdump
RM = rm

# Общий путь библиотек
#-------------------------------------------------------------------------------
LIBS_PATH          = libs

# Пути к CMSIS, StdPeriph Lib
#-------------------------------------------------------------------------------
CMSIS_PATH         = $(LIBS_PATH)/cmsis
STDPERIPH_INC_PATH = $(LIBS_PATH)/stdperiph/inc
STDPERIPH_SRC_PATH = $(LIBS_PATH)/stdperiph/src

# Пути к FreeRTOS
#-------------------------------------------------------------------------------
FREERTOS_PATH      = $(LIBS_PATH)/frtos
FREERTOS_INC_PATH  = $(FREERTOS_PATH)/include
FREERTOS_INC_PATH += $(FREERTOS_PATH)/portable/GCC/ARM_CM3
FREERTOS_SRC_PATH  = $(FREERTOS_PATH)
FREERTOS_PORT_PATH = $(FREERTOS_PATH)/portable/GCC/ARM_CM3
FREERTOS_MEMMANG_PATH = $(FREERTOS_PATH)/portable/MemMang
FREERTOS_MEMMANG_VERS = heap_2

# Используемые модули FreeRTOS
#-------------------------------------------------------------------------------
#~ FREERTOS_USE += event_groups
#~ FREERTOS_USE += croutine
#~ FREERTOS_USE += event_groups
FREERTOS_USE += list
FREERTOS_USE += queue
FREERTOS_USE += tasks
#~ FREERTOS_USE += timers


# Пути поиска исходных файлов
#-------------------------------------------------------------------------------
PROJECT_SRC = src
DRIVERS_SRC = $(PROJECT_SRC)/drivers
SOURCEDIRS = $(PROJECT_SRC)
SOURCEDIRS += $(DRIVERS_SRC)
SOURCEDIRS += $(CMSIS_PATH)
#SOURCEDIRS += $(STDPERIPH_SRC_PATH) не используем т.к. PERIPHDRIVERS
#SOURCEDIRS += $(FREERTOS_SRC_PATH) не используем т.к. FREERTOS_USE
SOURCEDIRS += $(FREERTOS_PORT_PATH)

# Пути поиска хидеров
#-------------------------------------------------------------------------------
INCLUDES += .
INCLUDES += $(SOURCEDIRS)
INCLUDES += $(STDPERIPH_INC_PATH)
INCLUDES += $(FREERTOS_INC_PATH)

# Библиотеки
#-------------------------------------------------------------------------------
LIBPATH +=
LIBS    +=

# Настройки компилятора
#-------------------------------------------------------------------------------
CFLAGS += -mthumb -mcpu=cortex-m3				# архитектура и система комманд
CFLAGS += -std=gnu99							# стандарт языка С
CFLAGS += -Wall -pedantic						# Выводить все предупреждения
CFLAGS += -Os									# Оптимизация
CFLAGS += -ggdb									# Генерировать отладочную информацию для gdb
CFLAGS += -fno-builtin
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections

CFLAGS += $(addprefix -I, $(INCLUDES))
CFLAGS += $(addprefix -D, $(DEFINES))

# Настройки линкера
#-------------------------------------------------------------------------------
LDFLAGS += -mcpu=cortex-m3 -mthumb -g	#важные диррективы
LDFLAGS += -nostartfiles
LDFLAGS += -nostdlib
LDFLAGS += -Xlinker --gc-sections
LDFLAGS += -L$(LDSCR_PATH)
LDFLAGS += -T$(LDSCR_PATH)/$(LDSCRIPT)
LDFLAGS += $(addprefix -L, $(LIBPATH))
LDFLAGS += $(LIBS)

# Настройки ассемблера
#-------------------------------------------------------------------------------
AFLAGS += -mcpu=cortex-m3 -mthumb -g
#-ahls -mapcs-32 не работает

# Список объектных файлов
#-------------------------------------------------------------------------------
OBJS += $(patsubst %.c, %.o, $(wildcard  $(addsuffix /*.c, $(SOURCEDIRS))))
OBJS += $(addprefix $(STDPERIPH_SRC_PATH)/, $(addsuffix .o, $(PERIPHDRIVERS)))
OBJS += $(addprefix $(FREERTOS_MEMMANG_PATH)/, $(addsuffix .o, $(FREERTOS_MEMMANG_VERS)))
OBJS += $(addprefix $(FREERTOS_SRC_PATH)/, $(addsuffix .o, $(FREERTOS_USE)))
OBJS += $(patsubst %.s, %.o, $(STARTUP))

# Пути поиска make
#-------------------------------------------------------------------------------
VPATH := $(SOURCEDIRS)

# Список файлов к удалению командой "make clean"
#-------------------------------------------------------------------------------
TOREMOVE += *.elf *.hex *.bin *.lst
TOREMOVE += $(addsuffix /*.o, $(SOURCEDIRS))
TOREMOVE += $(addsuffix /*.d, $(SOURCEDIRS))
TOREMOVE += $(STDPERIPH_SRC_PATH)/*.o
TOREMOVE += $(STDPERIPH_SRC_PATH)/*.d
TOREMOVE += $(STARTUPDIR)/*.o
TOREMOVE += $(TARGET)
TOREMOVE += $(FREERTOS_MEMMANG_PATH)/*.o
TOREMOVE += $(FREERTOS_MEMMANG_PATH)/*.d
TOREMOVE += $(FREERTOS_SRC_PATH)/*.o
TOREMOVE += $(FREERTOS_SRC_PATH)/*.d
TOREMOVE += $(FREERTOS_PORT_PATH)/*.o
TOREMOVE += $(FREERTOS_PORT_PATH)/*.d

.PHONY: clean load install all

# Собрать все
#-------------------------------------------------------------------------------
all: $(TARGET).hex $(TARGET).bin $(TARGET).lst size


# Очистка
#-------------------------------------------------------------------------------
clean:
	@$(RM) -f $(TOREMOVE)

# Создание .hex файла
#-------------------------------------------------------------------------------
$(TARGET).hex: $(TARGET).elf
	@$(CP) -Oihex $(TARGET).elf $(TARGET).hex

# Создание .bin файла
#-------------------------------------------------------------------------------
$(TARGET).bin: $(TARGET).elf
	@$(CP) -Obinary $(TARGET).elf $(TARGET).bin

# Создание .lst файла листинга
#-------------------------------------------------------------------------------
$(TARGET).lst: $(TARGET).elf
	@$(OD) -D $(TARGET).elf > $(TARGET).lst

# Показываем размер
#-------------------------------------------------------------------------------
size:
	@echo "---------------------------------------------------"
	@$(SZ) $(TARGET).elf

# Линковка
#-------------------------------------------------------------------------------
$(TARGET).elf: $(OBJS)
	$(LD) $(LDFLAGS) $^ -o $@

# Компиляция
#-------------------------------------------------------------------------------
%.o: %.c
	$(CC) $(CFLAGS) -MD -c $< -o $@

%.o: %.s
	$(AS) $(AFLAGS) -c $< -o $@

# Сгенерированные gcc зависимости
#-------------------------------------------------------------------------------
include $(wildcard $(PROJECT_SRC)/*.d)
include $(wildcard $(DRIVERS_SRC)/*.d)

#Адрес начала прошивки
ADDR=0x8000000

load: $(TARGET).bin
	./stlink/st-flash --reset write $(TARGET).bin $(ADDR)

install: load
