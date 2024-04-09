#include "hw.h"



static void bootMsg(void);




bool hwInit(void)
{
  bspInit();


  cliInit();
  logInit();
  gpioInit();
  cdcInit();
  uartInit();
  for (int i=0; i<HW_UART_MAX_CH; i++)
  {
    uartOpen(i, 115200);
  }

  logOpen(HW_LOG_CH, 115200);

  bootMsg();
  logPrintf("\r\n[ Firmware Begin... ]\r\n");
  logPrintf("Booting..Name \t\t: %s\r\n", _DEF_BOARD_NAME);
  logPrintf("Booting..Ver  \t\t: %s\r\n", _DEF_FIRMWATRE_VERSION);  
  logPrintf("\n");

  nvsInit();
  lcdInit();
  
  return true;
}

void bootMsg(void)
{
  logPrintf("\r\n[ ESP32-C6 Info ]\r\n");
  logPrintf("ESP32-C6 !\n");

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  logPrintf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
          CONFIG_IDF_TARGET,
          chip_info.cores,
          (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
          (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  logPrintf("silicon revision %d\n", chip_info.revision);

  uint32_t flash_size = 0;
  esp_flash_get_size(NULL, &flash_size);
  logPrintf("SPI FLASH : %uMB %s flash\n", flash_size / (1024 * 1024),
          (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");


  logPrintf("Free heap : %ld KB\n", esp_get_free_heap_size()/1024);
  logPrintf("Free Heapi: %d KB\n", esp_get_free_internal_heap_size()/1024);
  logPrintf("CPU Freq  : %lu Mhz\n", bspGetCpuFreqMhz());
}
