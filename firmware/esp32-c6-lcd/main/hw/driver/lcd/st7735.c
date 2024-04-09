#include "lcd/st7735.h"
#include "lcd/st7735_regs.h"
#include "gpio.h"
#include "cli.h"


#ifdef _USE_HW_ST7735
#include "driver/spi_master.h"
#include "driver/gpio.h"


#define _PIN_DEF_BLK    _PIN_GPIO_LCD_BLK
#define _PIN_DEF_DC     _PIN_GPIO_LCD_DC
#define _PIN_DEF_CS     _PIN_GPIO_LCD_CS
#define _PIN_DEF_RST    _PIN_GPIO_LCD_RST


#define MADCTL_MY       0x80
#define MADCTL_MX       0x40
#define MADCTL_MV       0x20
#define MADCTL_ML       0x10
#define MADCTL_RGB      0x00
#define MADCTL_BGR      0x08
#define MADCTL_MH       0x04


typedef struct 
{
  uint8_t dc;
  bool    q_req;  
  uint8_t q_max;
  uint8_t q_index;
} cb_data_t;


static void writecommand(uint8_t c);
static void writedata(uint8_t d);
static void st7735InitRegs(void);
static void st7735SetRotation(uint8_t m);
static bool st7735Reset(void);
static bool st7735SpiInit(void);
static bool st7735SendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms);


static spi_device_handle_t spi_ch;

static const int32_t _width  = HW_LCD_WIDTH;
static const int32_t _height = HW_LCD_HEIGHT;
static void (*frameCallBack)(void) = NULL;
volatile static bool  is_write_frame = false;
static cb_data_t cb_data;

#if HW_ST7735_MODEL == 0
const uint32_t colstart = 26;
const uint32_t rowstart = 1;
#else
const uint32_t colstart = 0;
const uint32_t rowstart = 0;
#endif



#ifdef _USE_HW_CLI
static void cliCmd(cli_args_t *args);
#endif

static void transferDoneISR(void)
{
  if (is_write_frame == true)
  {
    is_write_frame = false;    

    if (frameCallBack != NULL)
    {
      frameCallBack();
    }
  }
}





bool st7735Init(void)
{
  bool ret = true;

  cb_data.q_req = false;

  ret &= st7735SpiInit();
  ret &= st7735Reset();

#ifdef _USE_HW_CLI
  cliAdd("st7735", cliCmd);
#endif

  return ret;
}

bool st7735InitDriver(lcd_driver_t *p_driver)
{
  p_driver->init = st7735Init;
  p_driver->reset = st7735Reset;
  p_driver->setWindow = st7735SetWindow;
  p_driver->getWidth = st7735GetWidth;
  p_driver->getHeight = st7735GetHeight;
  p_driver->setCallBack = st7735SetCallBack;
  p_driver->sendBuffer = st7735SendBuffer;
  return true;
}

void IRAM_ATTR st7735PreCallback(spi_transaction_t *t)
{
  cb_data_t *p_data = (cb_data_t *)t->user;

  gpioPinWrite(_PIN_DEF_DC, p_data->dc);
}

void IRAM_ATTR st7735PostCallback(spi_transaction_t *t)
{
  cb_data_t *p_data = (cb_data_t *)t->user;

  if (p_data->q_req == true)
  {
    p_data->q_index++;
    if (p_data->q_index >= p_data->q_max)
    {
      transferDoneISR();
      p_data->q_req = false;
    }
  }
}

bool st7735SpiInit(void)
{
  esp_err_t ret;

  spi_bus_config_t buscfg = 
  {
    .miso_io_num = -1,
    .mosi_io_num = GPIO_NUM_6,
    .sclk_io_num = GPIO_NUM_7,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 64*1024
  };
  spi_device_interface_config_t devcfg = 
  {
    .clock_speed_hz = 40*1000*1000,          // Clock out at 40 MHz
    .mode = 0,                               // SPI mode 0
    .spics_io_num = GPIO_NUM_0,              // CS pin
    .queue_size = 16,                        // We want to be able to queue 8 transactions at a time
    .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_RETURN_RESULT, 
    .pre_cb = st7735PreCallback,             // Specify pre-transfer callback to handle D/C line
    .post_cb = st7735PostCallback
  };

  //Initialize the SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK)
  {
    return false;
  }

  //Attach the LCD to the SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_ch);
  if (ret != ESP_OK)
  {
    return false;
  }

  return true;
}

bool st7735Reset(void)
{
  if (_PIN_DEF_BLK >= 0) gpioPinWrite(_PIN_DEF_BLK, _DEF_LOW);
  if (_PIN_DEF_DC >= 0)  gpioPinWrite(_PIN_DEF_DC,  _DEF_HIGH);
  if (_PIN_DEF_CS >= 0)  gpioPinWrite(_PIN_DEF_CS,  _DEF_HIGH);
  // if (_PIN_DEF_RST >= 0) gpioPinWrite(_PIN_DEF_RST, _DEF_LOW);
  // delay(10);
  // if (_PIN_DEF_RST >= 0) gpioPinWrite(_PIN_DEF_RST, _DEF_HIGH);

  st7735InitRegs();

  st7735SetRotation(0);
  st7735FillRect(0, 0, HW_LCD_WIDTH, HW_LCD_HEIGHT, black);
  if (_PIN_DEF_BLK >= 0) gpioPinWrite(_PIN_DEF_BLK, _DEF_LOW);
  
  return true;
}

uint16_t st7735GetWidth(void)
{
  return _width;
}

uint16_t st7735GetHeight(void)
{
  return _height;
}

void writecommand(uint8_t c)
{
  esp_err_t ret;
  spi_transaction_t t;


  memset(&t, 0, sizeof(t));     // Zero out the transaction
  t.length = 8;                 // Command is 8 bits
  t.tx_buffer = &c;             // The data is the cmd itself
  cb_data.dc = 0;
  cb_data.q_max = 1;
  t.user = (void*)&cb_data;     // D/C needs to be set to 0
  ret = spi_device_polling_transmit(spi_ch, &t);  //Transmit!
  if (ret != ESP_OK)
  {
    logPrintf("writecommand fail\n");
  }
}

void writedata(uint8_t d)
{
  esp_err_t ret;
  spi_transaction_t t;

  
  memset(&t, 0, sizeof(t));     // Zero out the transaction
  t.length = 8;                 // Command is 8 bits
  t.tx_buffer = &d;             // The data is the cmd itself
  cb_data.dc = 1;
  cb_data.q_max = 1;
  t.user = (void*)&cb_data;     // D/C needs to be set to 0
  ret = spi_device_polling_transmit(spi_ch, &t);  //Transmit!  
  if (ret != ESP_OK)
  {
    logPrintf("writedata fail\n");
  }
}

void st7735InitRegs(void)
{
  writecommand(ST7735_SWRESET); //  1: Software reset, 0 args, w/delay
  delay(10);

  writecommand(ST7735_SLPOUT);  //  2: Out of sleep mode, 0 args, w/delay
  delay(10);

  writecommand(ST7735_FRMCTR1); //  3: Frame rate ctrl - normal mode, 3 args:
  writedata(0x01);              //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
  writedata(0x2C);
  writedata(0x2D);

  writecommand(ST7735_FRMCTR2); //  4: Frame rate control - idle mode, 3 args:
  writedata(0x01);              //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
  writedata(0x2C);
  writedata(0x2D);

  writecommand(ST7735_FRMCTR3); //  5: Frame rate ctrl - partial mode, 6 args:
  writedata(0x01);              //     Dot inversion mode
  writedata(0x2C);
  writedata(0x2D);
  writedata(0x01);              //     Line inversion mode
  writedata(0x2C);
  writedata(0x2D);

  writecommand(ST7735_INVCTR);  //  6: Display inversion ctrl, 1 arg, no delay:
  writedata(0x07);              //     No inversion

  writecommand(ST7735_PWCTR1);  //  7: Power control, 3 args, no delay:
  writedata(0xA2);
  writedata(0x02);              //     -4.6V
  writedata(0x84);              //     AUTO mode

  writecommand(ST7735_PWCTR2);  //  8: Power control, 1 arg, no delay:
  writedata(0xC5);              //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD

  writecommand(ST7735_PWCTR3);  //  9: Power control, 2 args, no delay:
  writedata(0x0A);              //     Opamp current small
  writedata(0x00);              //     Boost frequency

  writecommand(ST7735_PWCTR4);  // 10: Power control, 2 args, no delay:
  writedata(0x8A);              //     BCLK/2, Opamp current small & Medium low
  writedata(0x2A);

  writecommand(ST7735_PWCTR5);  // 11: Power control, 2 args, no delay:
  writedata(0x8A);
  writedata(0xEE);

  writecommand(ST7735_VMCTR1);  // 12: Power control, 1 arg, no delay:
  writedata(0x0E);

#if HW_ST7735_MODEL == 0
  writecommand(ST7735_INVON);   // 13: Don't invert display, no args, no delay
#else
  writecommand(ST7735_INVOFF);  // 13: Don't invert display, no args, no delay
#endif

  writecommand(ST7735_MADCTL);  // 14: Memory access control (directions), 1 arg:
  writedata(0xC8);              //     row addr/col addr, bottom to top refresh

  writecommand(ST7735_COLMOD);  // 15: set color mode, 1 arg, no delay:
  writedata(0x05);              //     16-bit color


  writecommand(ST7735_CASET);   //  1: Column addr set, 4 args, no delay:
  writedata(0x00);
  writedata(0x00);              //     XSTART = 0
  writedata(0x00);
  writedata(HW_LCD_WIDTH-1);    //     XEND = 159

  writecommand(ST7735_RASET);   //  2: Row addr set, 4 args, no delay:
  writedata(0x00);
  writedata(0x00);              //     XSTART = 0
  writedata(0x00);
  writedata(HW_LCD_HEIGHT-1);   //     XEND = 79


  writecommand(ST7735_NORON);   //  3: Normal display on, no args, w/delay
  delay(10);
  writecommand(ST7735_DISPON);  //  4: Main screen turn on, no args w/delay
  delay(10);
}

void st7735SetRotation(uint8_t mode)
{
  writecommand(ST7735_MADCTL);

  switch (mode)
  {
   case 0:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
     break;

   case 1:
     writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
     break;

  case 2:
    writedata(MADCTL_RGB);
    break;

   case 3:
     writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
     break;
  }
}

void st7735SetWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  writecommand(ST7735_CASET); // Column addr set
  writedata(0x00);
  writedata(x0+colstart);     // XSTART
  writedata(0x00);
  writedata(x1+colstart);     // XEND

  writecommand(ST7735_RASET); // Row addr set
  writedata(0x00);
  writedata(y0+rowstart);     // YSTART
  writedata(0x00);
  writedata(y1+rowstart);     // YEND

  writecommand(ST7735_RAMWR); // write to RAM
}

void st7735FillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
  uint16_t line_buf[w];

  // Clipping
  if ((x >= _width) || (y >= _height)) return;

  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }

  if ((x + w) > _width)  w = _width  - x;
  if ((y + h) > _height) h = _height - y;

  if ((w < 1) || (h < 1)) return;


  st7735SetWindow(x, y, x + w - 1, y + h - 1);


  spi_transaction_t trans;

  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.length = 8*w*2;               
  trans.tx_buffer = line_buf;        
  trans.user = (void*)&cb_data;     

  is_write_frame = true;
  cb_data.q_req = true;
  cb_data.q_max = h;
  cb_data.q_index = 0;
  cb_data.dc = 1;
 
  for (int i=0; i<w; i++)
  {
    line_buf[i] = color;
  }
  for (int i=0; i<h; i++)
  {  
    spi_device_polling_transmit(spi_ch, &trans);   
  }
}

bool st7735SendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms)
{
  const uint32_t block_cnt = 4;
  static spi_transaction_t trans[4];
  uint32_t block_len_bytes;
  esp_err_t ret;


  // LCD 프레임 버퍼를 block_cnt 갯수만큼 나눠서 SPI DMA로 전송한다. 
  // 따라서 트랜젝션 데이터도 block_cnt 갯수만큼 필요하다.
  // 1픽셀은 2바이트이기 때문에 2를 곱해야 한다. 
  //
  block_len_bytes = (length/block_cnt) * 2; // bytes

  for (int i=0; i<block_cnt; i++)
  {
    memset(&trans[i], 0, sizeof(spi_transaction_t));
    trans[i].length = block_len_bytes * 8; // bits               
    trans[i].tx_buffer = &p_data[block_len_bytes * i];        
    trans[i].user = (void*)&cb_data;     
  }

  is_write_frame = true;
  cb_data.q_req = true;
  cb_data.q_max = block_cnt;
  cb_data.q_index = 0;
  cb_data.dc = 1;

  for (int i=0; i<block_cnt; i++) 
  {
    ret = spi_device_queue_trans(spi_ch, &trans[i], 10);
    if (ret != ESP_OK)
    {
      // ToDo
    }
  }

  return true;
}

bool st7735SetCallBack(void (*p_func)(void))
{
  frameCallBack = p_func;

  return true;
}


#ifdef _USE_HW_CLI

static void frameISR(void)
{
  static int i = 0;

  cliPrintf("done %d\n", i++);
}

void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info"))
  {
    cliPrintf("Width  : %d\n", _width);
    cliPrintf("Heigth : %d\n", _height);
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "test"))
  {
    uint32_t pre_time;
    uint32_t pre_time_draw;
    uint16_t color_tbl[3] = {red, green, blue};
    uint8_t color_index = 0;
    void (*cb_func_pre)(void);

    cb_func_pre = frameCallBack;

    st7735SetCallBack(frameISR);

    pre_time = millis();
    while(cliKeepLoop())
    {
      if (millis()-pre_time >= 500)
      {
        pre_time = millis();

        if (color_index == 0) cliPrintf("draw red\n");
        if (color_index == 1) cliPrintf("draw green\n");
        if (color_index == 2) cliPrintf("draw blue\n");
        
        pre_time_draw = micros();
        st7735FillRect(0, 0, HW_LCD_WIDTH, HW_LCD_HEIGHT, color_tbl[color_index]);
        color_index = (color_index + 1)%3;

        cliPrintf("draw time : %d us, %d ms\n", micros()-pre_time_draw, (micros()-pre_time_draw)/1000);
      }
      delay(1);
    }

    st7735SetCallBack(cb_func_pre);
    ret = true;
  }

  if (ret == false)
  {
    cliPrintf("st7735 info\n");
    cliPrintf("st7735 test\n");
  }
}


#endif

#endif