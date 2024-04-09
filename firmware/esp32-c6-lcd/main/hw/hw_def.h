/*
 * hw_def.h
 *
 *  Created on: 2021. 1. 9.
 *      Author: baram
 */

#ifndef MAIN_HW_HW_DEF_H_
#define MAIN_HW_HW_DEF_H_


#include "def.h"
#include "bsp.h"



#define _DEF_FIRMWATRE_VERSION    "V240409R1"
#define _DEF_BOARD_NAME           "ESP32-C6-LCD"



#define _HW_DEF_RTOS_THREAD_PRI_CLI           5
#define _HW_DEF_RTOS_THREAD_PRI_BUTTON        5
#define _HW_DEF_RTOS_THREAD_PRI_LCD           5


#define _HW_DEF_RTOS_THREAD_MEM_CLI           (8*1024)
#define _HW_DEF_RTOS_THREAD_MEM_BUTTON        (1*1024)
#define _HW_DEF_RTOS_THREAD_MEM_LCD           (2*1024)


#define _USE_HW_RTOS
#define _USE_HW_CDC
#define _USE_HW_NVS

#define _USE_HW_UART
#define      HW_UART_MAX_CH         2
#define      HW_UART_CH_USB         _DEF_UART1
#define      HW_UART_CH_CLI         _DEF_UART2

#define _USE_HW_CLI
#define      HW_CLI_CMD_LIST_MAX    32
#define      HW_CLI_CMD_NAME_MAX    16
#define      HW_CLI_LINE_HIS_MAX    8
#define      HW_CLI_LINE_BUF_MAX    64

#define _USE_HW_CLI_GUI
#define      HW_CLI_GUI_WIDTH       80
#define      HW_CLI_GUI_HEIGHT      24

#define _USE_HW_LOG
#define      HW_LOG_CH              HW_UART_CH_CLI
#define      HW_LOG_BOOT_BUF_MAX    1024
#define      HW_LOG_LIST_BUF_MAX    1024

#define _USE_HW_GPIO
#define      HW_GPIO_MAX_CH         3

#define _USE_HW_LCD
#define      HW_LCD_SWAP_RGB        1
#define      HW_LCD_LVGL            1
#define      HW_LCD_LOGO            1
#define _USE_HW_ST7735
#define      HW_LCD_WIDTH           80
#define      HW_LCD_HEIGHT          160


#define _PIN_GPIO_LCD_BLK          -1
#define _PIN_GPIO_LCD_DC            1
#define _PIN_GPIO_LCD_CS           -1
#define _PIN_GPIO_LCD_RST           2


#endif 
