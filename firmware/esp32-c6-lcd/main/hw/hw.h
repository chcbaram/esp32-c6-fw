#ifndef HW_H_
#define HW_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#include "uart.h"
#include "cdc.h"
#include "cli.h"
#include "log.h"
#include "lcd.h"
#include "nvs.h"
#include "gpio.h"

bool hwInit(void);


#ifdef __cplusplus
}
#endif

#endif 