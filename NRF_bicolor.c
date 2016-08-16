
#include "NRF_bicolor.h"

#ifndef _BV
  #define _BV(bit) (1<<(bit))
#endif

#ifndef _swap_int16_t
  #define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

// Gotten from John X. Liu on http://www.keil.com/forum/1064/  
#define LongToBin(n) \
    (\
    ((n >> 21) & 0x80) | \
    ((n >> 18) & 0x40) | \
    ((n >> 15) & 0x20) | \
    ((n >> 12) & 0x10) | \
    ((n >>  9) & 0x08) | \
    ((n >>  6) & 0x04) | \
    ((n >>  3) & 0x02) | \
    ((n      ) & 0x01)   \
    )
#define Bin(n) LongToBin(0x##n##l)
  
const uint8_t NRF_bicolor_smile_bmp[] = { 
        Bin(00111100), 
        Bin(01000010), 
        Bin(10100101), 
        Bin(10000001), 
        Bin(10100101), 
        Bin(10011001), 
        Bin(01000010), 
        Bin(00111100),
};
const uint8_t NRF_bicolor_neutral_bmp[] = { 
        Bin(00111100), 
        Bin(01000010), 
        Bin(10100101), 
        Bin(10000001), 
        Bin(10111101), 
        Bin(10000001), 
        Bin(01000010), 
        Bin(00111100),
};
const uint8_t NRF_bicolor_frown_bmp[] = { 
        Bin(00111100), 
        Bin(01000010), 
        Bin(10100101), 
        Bin(10000001), 
        Bin(10011001), 
        Bin(10100101), 
        Bin(01000010), 
        Bin(00111100),
};


#define LED_ON 1
#define LED_OFF 0

#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3
#define HT16K33_CMD_BRIGHTNESS 0xE0  
#define HT16K33_CMD_WRITE 0x00
// I could not get these global variables to work properly. They were not set correctly.
//uint8_t const bicolor_oscillator_cmd_on = 0x21; 
//uint8_t const bicolor_screen_off = 0x80;
//uint8_t const bicolor_screen_on = 0x81;
  
// This needs to match the settings in nrf_drv_cfg.h for enabling the TWI, the SCL and SDA does not need to match,
// These are set in the init function called when initializing the object
nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(0);
#define BICOLOR_SCL_PIN 3
#define BICOLOR_SDA_PIN 4

/**
 * @brief TWI events handler.
 */
uint8_t device_address = 0; // Address used to temporarily store the current address being checked
bool device_found = false; 
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            // If EVT_DONE (event done) is received a device is found and responding on that particular address
            printf("\r\n!****************************!\r\nDevice found at 7-bit address: %#x!\r\n!****************************!\r\n\r\n", device_address);
            //printf("\r\n!****************************!\r\nEVT_DONE received\r\n");
            device_found = true;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            //printf("No address ACK on address: %#x!\r\n", device_address);
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            //printf("No data ACK on address: %#x!\r\n", device_address);
            break;
        default:
            break;        
    }   
}

/**
 * @brief TWI initialization.
 */
void twi_init () {
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_config = {
       .scl                = BICOLOR_SCL_PIN,
       .sda                = BICOLOR_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&twi_instance, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&twi_instance);
}

void NRF_bicolor_TWI_init(void) {
    // This funciton should only be called once to initialize the TWI module!
    twi_init();
    //nrf_delay_ms(100);  // Delay to allow everything to start up
    
    // Might wanna check if device is actually found on bus here!
//    uint8_t dummy_data = 0x55;
//    // Itterate through all possible 7-bit TWI addresses
//    for(uint8_t i = 0; i <= 0x7F; i++)
//    {
//        device_address = i;
//        // Send dummy data. If a device is present on this particular address a TWI EVT_DONE event is 
//        // received in the twi event handler and a message is printed to UART
//        nrf_drv_twi_tx(&twi_instance, i, &dummy_data, 1, false);
//        // Delay 10 ms to allow TWI transfer to complete and UART to print messages before starting new transfer
//        nrf_delay_ms(10);
//    }
//    if device not found -> fail!
}

void NRF_bicolor_setBrightness(NRF_bicolor * t, uint8_t b) {
    if (b > 15) {
        b = 15;  // Turn fully on if invalid input
    }
    uint8_t const bicolor_brightness_cmd = HT16K33_CMD_BRIGHTNESS | b;
    while(nrf_drv_twi_tx(&twi_instance, t->i2c_addr, &bicolor_brightness_cmd, 1, false) == NRF_ERROR_BUSY);
    nrf_delay_ms(1);
}

void NRF_bicolor_blinkRate(NRF_bicolor * t, uint8_t b) {
    if (b > 3) {
        b = 0;  // Turn off if level invalid
    }
    uint8_t const bicolor_blink_cmd = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1);
    while(nrf_drv_twi_tx(&twi_instance, t->i2c_addr, &bicolor_blink_cmd, 1, false) == NRF_ERROR_BUSY);
    nrf_delay_ms(1);
}

void NRF_bicolor_begin(NRF_bicolor* t, uint8_t _addr /* 0x70 */) {
    t->i2c_addr = _addr;
    t->rotation = 0;
    t->cursor_y = 0;
    t->cursor_x = 0;
    
    // Clear dataBuffer
    for (uint8_t i = 0; i < 8; i++) {
        t->displaybuffer[i] = 0x0;
    }
    
    // Turn on ocillator
    uint8_t const bicolor_oscillator_on = 0x21;  // I could not get this to work being a global variable
    while (nrf_drv_twi_tx(&twi_instance, _addr, &bicolor_oscillator_on, 1, false) == NRF_ERROR_BUSY);
    nrf_delay_ms(1);
    // Turn off blink
    NRF_bicolor_blinkRate(t, HT16K33_BLINK_OFF);
    // Set brightness to max fully lit
    NRF_bicolor_setBrightness(t, 15);
}

void NRF_bicolor_writeDisplay(NRF_bicolor* t) {
    // Insert start write cmd at beginning
    uint8_t twi_write_data[17] = {HT16K33_CMD_WRITE};
    // Insert data from displaybuffer into TWI write array
    for (uint8_t i = 0; i < 8; i++) {
        twi_write_data[(i*2)+1] = t->displaybuffer[i] & 0xFF;
        twi_write_data[(i*2)+2] = t->displaybuffer[i] >> 8;
    }
    nrf_drv_twi_tx(&twi_instance, t->i2c_addr, twi_write_data, 17, false);  // Might want to add while error check?
    nrf_delay_ms(1);
}

void NRF_bicolor_clear(NRF_bicolor* t) {
    for (uint8_t i = 0; i < 8; i++) {
        t->displaybuffer[i] = 0;
    }
}

uint8_t NRF_bicolor_getRotation(NRF_bicolor* t) {
    return t->rotation;
}

void NRF_bicolor_setRotation(NRF_bicolor* t, uint8_t x) {
    t->rotation = (x & 3);
}

void NRF_bicolor_setCursor(NRF_bicolor* t, int16_t x, int16_t y) {
    t->cursor_x = x;
    t->cursor_y = y;
}

void NRF_bicolor_drawPixel(NRF_bicolor* t, int16_t x, int16_t y, uint16_t color) {
    if ((y < 0) || (y >= 8)) return;
    if ((x < 0) || (x >= 8)) return;
    
    switch(NRF_bicolor_getRotation(t)) {
    case 1:
        _swap_int16_t(x, y);
        x = 8 - x - 1;
        break;
    case 2:
        x = 8 - x - 1;
        y = 8 - y - 1;
        break;
    case 3:
        _swap_int16_t(x, y);
        y = 8 - y - 1;
        break;
    }
    
    if (color == LED_GREEN) {
        // Turn on green LED
        t->displaybuffer[y] |= 1 << x;
        // Turn off red LED.
        t->displaybuffer[y] &= ~(1 << (x+8));
    } else if (color == LED_RED) {
        // Turn on red LED.
        t->displaybuffer[y] |= 1 << (x+8);
        // Turn off green LED.
        t->displaybuffer[y] &= ~(1 << x);
    } else if (color == LED_YELLOW) {
        // Turn on green and red LED.
        t->displaybuffer[y] |= (1 << (x+8)) | (1 << x);
    } else if (color == LED_OFF) {
        // Turn off green and red LED.
        t->displaybuffer[y] &= ~(1 << x) & ~(1 << (x+8));
    }
}

void NRF_bicolor_drawBitmap(NRF_bicolor* t, uint8_t x, uint8_t y, const uint8_t *bitmap, uint16_t color) {
    int16_t i, j, byteWidth = (8 + 7) / 8;
    uint8_t byte;
    
    for (j = 0; j < 8; j++) {
        for (i = 0; i < 8; i++) {
            if (i & 7) byte <<= 1;
            else byte = bitmap[j * byteWidth + i / 8];
            if (byte & 0x80) NRF_bicolor_drawPixel(t, x+i, y+j, color);
        }
    }
}

void NRF_bicolor_drawLine(NRF_bicolor* t, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }
    
    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }
    
    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);
    
    int16_t err = dx / 2;
    int16_t ystep;
    
    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }
    
    for (; x0<=x1; x0++) {
        if (steep) {
            NRF_bicolor_drawPixel(t, y0, x0, color);
        } else {
            NRF_bicolor_drawPixel(t, x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void NRF_bicolor_drawRect(NRF_bicolor* t, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    NRF_bicolor_drawLine(t, x, y, x, y+h, color);
    NRF_bicolor_drawLine(t, x, y, x+w, y, color);
    NRF_bicolor_drawLine(t, x+w, y, x+w, y+h, color);
    NRF_bicolor_drawLine(t, x, y+h, x+w, y+h, color);
}

void NRF_bicolor_fillRect(NRF_bicolor* t, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    for (int16_t i=x;  i<x+w; i++) {
        NRF_bicolor_drawLine(t, i, y, i, y+h-1, color);
    }
}

static const uint8_t font[] = {
    Bin(01111100), // 0
    Bin(10000010),
    Bin(10000010),
    Bin(10000010),
    Bin(01111100),
    
    Bin(00000000), // 1
    Bin(10001100),
    Bin(11111110),
    Bin(10000000),
    Bin(00000000),
    
    Bin(01101100), // 2
    Bin(10010010),
    Bin(10010010),
    Bin(10010010),
    Bin(10001100),
    
    Bin(01000100), // 3
    Bin(10000010),
    Bin(10010010),
    Bin(10010010),
    Bin(01101100),
    
    Bin(11111111), // 4 not impl
    Bin(11111111),
    Bin(11111111),
    Bin(11111111),
    Bin(11111111),
};

void NRF_bicolor_drawChar(NRF_bicolor* t, int16_t x, int16_t y, uint8_t c, uint16_t textcolor, uint16_t bgcolor) {
    if ((x >= 8)                ||  // Clip right
        (y >= 8)                ||  // Clip bottom
        ((x + 6 - 1) < 0)   ||  // Clip left
        ((y + 8 - 1) < 0))      // Clip top
        return;
    
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t line;
        if (i < 5)  line = font[(c - '0')*5 + i];
        else        line = 0x0;
        for (uint8_t j = 0; j < 8; j++, line >>= 1) {
            if (line & 0x1) {
                NRF_bicolor_drawPixel(t, x+i, y+j, textcolor);
            }
            else {
                NRF_bicolor_drawPixel(t, x+i, y+j, bgcolor);
            }
        }
    }
}

uint8_t NRF_bicolor_write(NRF_bicolor* t, uint8_t c, uint16_t textcolor, uint16_t bgcolor) {
    NRF_bicolor_drawChar(t, t->cursor_x, t->cursor_y, c, textcolor, bgcolor);
    t->cursor_x += 6;
    return 1;
}

uint8_t NRF_bicolor_write_string(NRF_bicolor* t, const uint8_t *buffer, uint8_t size, uint16_t textcolor, uint16_t bgcolor) {
    uint8_t n = 0;
    while(size--) {
        if (NRF_bicolor_write(t, *buffer++, textcolor, bgcolor)) n++;
        else break;
    }
    return n;
}

/// Adafruit LEDBackpack.cpp
//static const uint8_t numbertable[] = {
//	0x3F, /* 0 */
//	0x06, /* 1 */
//	0x5B, /* 2 */
//	0x4F, /* 3 */
//	0x66, /* 4 */
//	0x6D, /* 5 */
//	0x7D, /* 6 */
//	0x07, /* 7 */
//	0x7F, /* 8 */
//	0x6F, /* 9 */
//	0x77, /* a */
//	0x7C, /* b */
//	0x39, /* C */
//	0x5E, /* d */
//	0x79, /* E */
//	0x71, /* F */
//};

//static const uint16_t alphafonttable[] =  {

//0x1,
//0x2,
//0x4,
//0x8,
//0x10,
//0x20,
//0x40,
//0x80,
//0x100,
//0x200,
//0x400,
//0x800,
//0x1000,
//0x2000,
//0x4000,
//0x8000,
//0x0,
//0x0,
//0x0,
//0x0,
//0x0,
//0x0,
//0x0,
//0x0,
//0x12c9,
//0x15c0,
//0x12f9,
//0xe3,
//0x530,
//0x12c8,
//0x3a00,
//0x1700,
//0x0, //
//0x6, // !
//0x220, // "
//0x12ce, // #
//0x12ed, // $
//0xc24, // %
//0x235d, // &
//0x400, // '
//0x2400, // (
//0x900, // )
//0x3fc0, // *
//0x12c0, // +
//0x800, // ,
//0xc0, // -
//0x0, // .
//0xc00, // /
//0xc3f, // 0
//0x6, // 1
//0xdb, // 2
//0x8f, // 3
//0xe6, // 4
//0x2069, // 5
//0xfd, // 6
//0x7, // 7
//0xff, // 8
//0xef, // 9
//0x1200, // :
//0xa00, // ;
//0x2400, // <
//0xc8, // =
//0x900, // >
//0x1083, // ?
//0x2bb, // @
//0xf7, // A
//0x128f, // B
//0x39, // C
//0x120f, // D
//0xf9, // E
//0x71, // F
//0xbd, // G
//0xf6, // H
//0x1200, // I
//0x1e, // J
//0x2470, // K
//0x38, // L
//0x536, // M
//0x2136, // N
//0x3f, // O
//0xf3, // P
//0x203f, // Q
//0x20f3, // R
//0xed, // S
//0x1201, // T
//0x3e, // U
//0xc30, // V
//0x2836, // W
//0x2d00, // X
//0x1500, // Y
//0xc09, // Z
//0x39, // [
//0x2100, //
//0xf, // ]
//0xc03, // ^
//0x8, // _
//0x100, // `
//0x1058, // a
//0x2078, // b
//0xd8, // c
//0x88e, // d
//0x858, // e
//0x71, // f
//0x48e, // g
//0x1070, // h
//0x1000, // i
//0xe, // j
//0x3600, // k
//0x30, // l
//0x10d4, // m
//0x1050, // n
//0xdc, // o
//0x170, // p
//0x486, // q
//0x50, // r
//0x2088, // s
//0x78, // t
//0x1c, // u
//0x2004, // v
//0x2814, // w
//0x28c0, // x
//0x200c, // y
//0x848, // z
//0x949, // {
//0x1200, // |
//0x2489, // }
//0x520, // ~
//0x3fff,

//};
