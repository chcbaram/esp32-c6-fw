
file(GLOB SRC_FILES 
  ap/*.c
  ap/*.cpp
  bsp/*.c
  hw/*.c
  hw/driver/*.c
  hw/driver/touch/*.c
  hw/driver/hangul/*.c
  hw/driver/lvgl/*.c
  hw/driver/lcd/*.c
)

file(GLOB_RECURSE SRC_FILES_RECURSE
  common/*.c 
  lib/lvgl/*.c
)

set(requires 
  "spi_flash" 
  "esp_timer"
  "vfs"
  "nvs_flash"
  )

idf_component_register(    
    SRCS "main.cpp"
    
    SRCS ${SRC_FILES}
    SRCS ${SRC_FILES_RECURSE}
    
    INCLUDE_DIRS ""
    INCLUDE_DIRS "ap"
    INCLUDE_DIRS "bsp"
    INCLUDE_DIRS "common"
    INCLUDE_DIRS "common/core"
    INCLUDE_DIRS "common/hw/include"
    INCLUDE_DIRS "hw"
    # INCLUDE_DIRS "lib" 

    PRIV_REQUIRES soc driver freertos
    
    REQUIRES "${requires}"
    )