
/// Adafruit_LEDBackpack.h

#ifndef NRF_bicolor_h
#define NRF_bicolor_h

#include <stdint.h>
#include <stdio.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include <stdlib.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include <string.h>

extern nrf_drv_twi_t twi_instance;

typedef struct {
    uint16_t displaybuffer[8];
    uint8_t i2c_addr;
    
    uint8_t rotation;
    int16_t cursor_y;
    int16_t cursor_x;
} NRF_bicolor;

extern const uint8_t NRF_bicolor_smile_bmp[];
extern const uint8_t NRF_bicolor_neutral_bmp[];
extern const uint8_t NRF_bicolor_frown_bmp[];

#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3

void NRF_bicolor_TWI_init(void);
void NRF_bicolor_begin(NRF_bicolor* t, uint8_t _addr /* 0x70 */);
void NRF_bicolor_setBrightness(NRF_bicolor * t, uint8_t b);
void NRF_bicolor_blinkRate(NRF_bicolor * t, uint8_t b);
void NRF_bicolor_clear(NRF_bicolor* t);
void NRF_bicolor_drawBitmap(NRF_bicolor* t, uint8_t x, uint8_t y, const uint8_t *bitmap, uint16_t color);
void NRF_bicolor_writeDisplay(NRF_bicolor* t);


#endif // NRF_bicolor_h
