#ifndef ST7735_H_
#define ST7735_H_

#include "hw_def.h"


#ifdef _USE_HW_ST7735

#include "lcd.h"
#include "st7735_regs.h"



bool st7735Init(void);
bool st7735InitDriver(lcd_driver_t *p_driver);
void st7735SetWindow(int32_t x, int32_t y, int32_t w, int32_t h);
bool st7735SetCallBack(void (*p_func)(void));


uint16_t st7735GetWidth(void);
uint16_t st7735GetHeight(void);

void st7735FillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);

#endif

#endif 