/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using ADC.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include <string.h>

//#include "nrf_drv_twi.h"
#include "NRF_bicolor.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1   /**< UART RX buffer size. */

////#warning "!**** ARE YOU ABSOLUTELY SURE YOU HAVE CHOSEN THE CORRECT SCL AND SDA PINS? ****!"
//#define DEVICE_SCL_PIN 3
//#define DEVICE_SDA_PIN 4

//nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(0);

//uint8_t device_address = 0; // Address used to temporarily store the current address being checked
//bool device_found = false; 

#define SAMPLES_IN_BUFFER 128
volatile uint8_t state = 1;

static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;

//static float testInput_f32_9khz[SAMPLES_IN_BUFFER * 2];
//static float testOutput[SAMPLES_IN_BUFFER];
static uint32_t testOutputArduinoAttmpt[SAMPLES_IN_BUFFER/2];  // Only the first half of bins are usefull
static int16_t nrf_analog_samples_to_send[SAMPLES_IN_BUFFER];
static float       magnitudes[SAMPLES_IN_BUFFER] = {0};
static float       samples_nrf[SAMPLES_IN_BUFFER*2]; // = {
static float       samples_nrf_uno[SAMPLES_IN_BUFFER] = {
178,
23,
-158,
-288,
-268,
-143,
14,
197,
307,
254,
114,
-49,
-230,
-308,
-230,
-90,
80,
250,
297,
197,
47,
-130,
-279,
-291,
-177,
-25,
157,
291,
273,
145,
-14,
-197,
-304,
-257,
-121,
43,
220,
301,
224,
82,
-91,
-258,
-304,
-207,
-61,
117,
274,
289,
172,
20,
-160,
-293,
-272,
-146,
11,
193,
304,
252,
115,
-49,
-226,
-305,
-228,
-86,
86,
257,
307,
207,
58,
-117,
-271,
-286,
-172,
-19,
161,
295,
274,
145,
-12,
-192,
-303,
-253,
-119,
45,
222,
303,
229,
84,
-86,
-256,
-302,
-205,
-60,
118,
274,
291,
176,
21,
-158,
-297,
-276,
-150,
8,
188,
299,
248,
111,
-52,
-233,
-312,
-238,
-96,
73,
241,
296,
197,
50,
-125,
-278,
-290,
-176,
-25,
158,
297,
280,
149,
-9,
-191};

// This is low-level noise that's subtracted from each FFT output column: Modified for 52 output
static const uint16_t noise[64] = {
//    8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
//    2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
//    3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
//    2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4
    500,150,150,100,75,90,90,90,75,90,90,75,65,75,75,90,
    65,50,65,50,75,65,75,65,50,65,75,50,65,75,90,90,
    75,65,65,65,65,65,65,50,75,65,65,65,65,65,65,65,
    65,65,65,65,65,65,65,65,65,65,65,65,65,75,75,90
};

  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
static const uint8_t eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 };

// When filtering down to 8 columns, these tables contain indexes
  // and weightings of the FFT spectrum output values to use.  Not all
  // buckets are used -- the bottom-most and several at the top are
  // either noisy or out of range or generally not good for a graph.
static const uint8_t col0data[] = {  2,  1,     // # of spectrum bins to merge, index of first
                            111,   8 };         // Weights for each bin    
static const uint8_t col1data[] = {  4,  1,  // 4 bins, starting at index 1
     19, 186,  38,   2 }; // Weights for 4 bins.  Got it now?
  static const uint8_t col2data[] = {  5,  2,
     11, 156, 118,  16,   1 };
  static const uint8_t col3data[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 };
  static const uint8_t col4data[] = { 11,  5,
      3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 };
  static const uint8_t col5data[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 };
  static const uint8_t col6data[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 };
  static const uint8_t col7data[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 };
static const uint8_t * const colData[] = {col0data,col1data,col2data,col3data,col4data,col5data,col6data,col7data};
      
//extern float testInput_f32_10khz[SAMPLES_IN_BUFFER * 2];  // For testing

      
/**
 * @brief TWI events handler.
 */
//void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
//{   
//    switch(p_event->type)
//    {
//        case NRF_DRV_TWI_EVT_DONE:
//            // If EVT_DONE (event done) is received a device is found and responding on that particular address
//            printf("\r\n!****************************!\r\nDevice found at 7-bit address: %#x!\r\n!****************************!\r\n\r\n", device_address);
//            device_found = true;
//            break;
//        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
//            printf("No address ACK on address: %#x!\r\n", device_address);
//            break;
//        case NRF_DRV_TWI_EVT_DATA_NACK:
//            printf("No data ACK on address: %#x!\r\n", device_address);
//            break;
//        default:
//            break;        
//    }   
//}

//void twi_init (void)
//{
//    ret_code_t err_code;
//    
//    const nrf_drv_twi_config_t twi_config = {
//       .scl                = DEVICE_SCL_PIN,
//       .sda                = DEVICE_SDA_PIN,
//       .frequency          = NRF_TWI_FREQ_100K,
//       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
//    };
//    
//    err_code = nrf_drv_twi_init(&twi_instance, &twi_config, twi_handler, NULL);
//    APP_ERROR_CHECK(err_code);
//    
//    nrf_drv_twi_enable(&twi_instance);
//}

/**
 * @brief UART events handler.
 */
void uart_events_handler(app_uart_evt_t * p_event)
{
}

/**
 * @brief UART initialization.
 */
void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

void timer_handler(nrf_timer_event_t event_type, void* p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 1ms */
    //uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    //uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 10);
    //uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer, 104);  // Every 104 uS should give roughly 9615 Hz
    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer, 62);  // Oversampling 2x, halfing
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

uint8_t samples_ready = false;
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        //ret_code_t err_code;

        //printf("ADC event number: %d\r\n",(int)m_adc_evt_counter);
        samples_ready = true;
//        for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
//        {
//            printf("%d:%d\r\n", i, p_event->data.done.p_buffer[i]);
//        }
        m_adc_evt_counter++;
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
    //        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    {                                                      
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,         
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,         
    .gain       = NRF_SAADC_GAIN1_4,                   
    .reference  = NRF_SAADC_REFERENCE_VDD4,        
    .acq_time   = NRF_SAADC_ACQTIME_10US,              
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,         
    .pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN0),          
    .pin_n      = NRF_SAADC_INPUT_DISABLED            
    };
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    
    //err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    //APP_ERROR_CHECK(err_code);
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = (float)9600 / (float)128;
  return (int)(frequency / binFrequency);
}

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1; i < 128/2; ++i) {
      if (i >= lowBin && i <= highBin) {
        *windowMean += magnitudes[i];
      }
      else {
        *otherMean += magnitudes[i];
      }
    }
    *windowMean /= (highBin - lowBin) + 1;
    *otherMean /= (128 / 2 - (highBin - lowBin));
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    //float maxValue;
    //uint32_t testIndex = 0;
    
    uint8_t nBins = 0;
    uint8_t binNum = 0;
    uint8_t i = 0;
    uint8_t j = 0;
    //uint8_t peak[8] = {0};  // Peak level of each column; used for falling dots
    //uint8_t dotCount = 0;  // Frame counter for delaying dot-falling speed
    uint8_t colCount = 0;  // Frame counter for storing past column data
    int col[8][10];  // Columns levels for the prior 10 frames
    memset(col , 0, sizeof(col));
    int minLvlAvg[8];  // For dynamic adjustment of low & high ends of graph
    int maxLvlAvg[8];  // pseudo rolling averages for the prior few frames.
    int colDiv[8];  // Used when filtering FFT output to 8 columns
    uint8_t * data;
    float frequencyWindow[8+1];
    
    // Setup
    for(i = 0; i < 8; i++) {
        minLvlAvg[i] = 0;
        maxLvlAvg[i] = 512;
        data = (uint8_t *)colData[i];
        nBins = data[0] + 2;
        binNum = data[1];
        for (colDiv[i] = 0; j < nBins; j++) {
            colDiv[i] += data[j];
        }
    } 
    for(i = 0; i < 8+1; i++) {        
        float windowSize = (9600 / 2.0) / 8;
        frequencyWindow[i] = i*windowSize;
    }
    
    uart_config();
    
    NRF_bicolor matrix;
    NRF_bicolor_TWI_init();
    NRF_bicolor_begin(&matrix, 0x70);
    
    NRF_bicolor_blinkRate(&matrix, 1);
    nrf_delay_ms(1000);
    NRF_bicolor_blinkRate(&matrix, 0);
    
    NRF_bicolor_clear(&matrix);
    NRF_bicolor_drawBitmap(&matrix, 0, 0, NRF_bicolor_frown_bmp, LED_RED);
    NRF_bicolor_writeDisplay(&matrix);
    nrf_delay_ms(500);
    
    NRF_bicolor_clear(&matrix);
    NRF_bicolor_drawBitmap(&matrix, 0, 0, NRF_bicolor_neutral_bmp, LED_YELLOW);
    NRF_bicolor_writeDisplay(&matrix);
    nrf_delay_ms(500);
    
    NRF_bicolor_clear(&matrix);
    NRF_bicolor_drawBitmap(&matrix, 0, 0, NRF_bicolor_smile_bmp, LED_GREEN);
    NRF_bicolor_writeDisplay(&matrix);
    nrf_delay_ms(500);
//    NRF_bicolor_blinkRate(&matrix, 1);
//    nrf_delay_ms(2000);
//    NRF_bicolor_blinkRate(&matrix, 2);
//    nrf_delay_ms(2000);
//    NRF_bicolor_blinkRate(&matrix, 0);

    //printf("\n\rSAADC HAL simple example.\r\n");
    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();

    uint8_t c = ' ';
    uint32_t strIndex = 0;
    uint8_t str[64];
    #define FFT_SIZE 128
    while(1)
    {
        uint8_t ret = app_uart_get(&c);
        if (ret != NRF_ERROR_NOT_FOUND) {
            // Check for overflow
            if (strIndex >= sizeof(str) - 1) {
              strIndex = 0;  
            }
            //printf("%c", c);
            // Check for end of command
            if (c != ';') {
                str[strIndex] = c;
                strIndex++;
            }
            else {
                // Parse the command because end of command encountered
                if (strcmp((char*)str, "GET FFT_SIZE") == 0  ||  strcmp((char*)str, "F") == 0) {
                //if (strcmp((char*)str, "F") == 0) {
                    printf("%d\r\n", FFT_SIZE);
                }
                if (strcmp((char*)str, "GET SAMPLE_RATE_HZ") == 0) {
                //if (strcmp((char*)str, "H") == 0) {
                    printf("%d\r\n", 9600);
                }
                if (strcmp((char*)str, "GET MAGNITUDES") == 0  ||  strcmp((char*)str, "M") == 0) {
                //if (strcmp((char*)str, "M") == 0) {
                    for (int i = 0; i < FFT_SIZE; i++) {
                        char array[100];
                        int numWritten = 0;
                        numWritten = sprintf(array, "%f\r\n", magnitudes[i]);
                        for (uint32_t i = 0; i < numWritten; i++) {
                            while(app_uart_put(array[i]) != NRF_SUCCESS);
                        }
                        //printf("%.2f\r\n", magnitudes[i]);
                    }
                }
                if (strcmp((char*)str, "GET SAMPLES") == 0  ||  strcmp((char*)str, "S") == 0) {
                    for (int i = 0; i < FFT_SIZE; i++) {
                        char array[100] = {0};
                        int numWritten = 0;
                        numWritten = sprintf(array, "%d\r\n", nrf_analog_samples_to_send[i]);
                        for (uint32_t i = 0; i < numWritten; i++) {
                            while(app_uart_put(array[i]) != NRF_SUCCESS);
                        }
                    }
                }
//                if (strcmp((char*)str, "GET SAMPLE_RATE_HZ") == 0) {
//                    printf("%d\n", 9600);
//                    start_sequence_done += 1;
//                }
                // Clear the command buffer
                memset(str, 0, sizeof(str));
                strIndex = 0;
            }
        }
        //__WFE();
        if (samples_ready) {
            //printf("\n\rSamples are ready to be used.\r\n");
            static const int16_t noiseThreshold = 4;
            for (i = 0; i < SAMPLES_IN_BUFFER; i++) {
                //printf("%d\r\n", m_buffer_pool[0][i]);
                if (m_buffer_pool[0][i] > (512 - noiseThreshold) && m_buffer_pool[0][i] < (512 + noiseThreshold)) {
                    samples_nrf[i*2] = 0;
                    nrf_analog_samples_to_send[i] = 0;
                } else {
                    samples_nrf[i*2] = (float)(m_buffer_pool[0][i] - 512);  // Sign-convert for FFT; -512 to 512
                    nrf_analog_samples_to_send[i] = m_buffer_pool[0][i] - 512;
                }
                //printf("%f\r\n", samples_nrf[i*2]);
                //testInput_f32_9khz[i*2] = (float)m_buffer_pool[0][i];
                //testInput_f32_9khz[i*2] = (i % 2) ? 800 : 400;
                //testInput_f32_9khz[i*2 + 1] = 0;
                samples_nrf[i*2 + 1] = 0;
            }
            //printf("Print magnitudes below\r\n\n");
            
            arm_cfft_f32(&arm_cfft_sR_f32_len128, samples_nrf, /* ifftFlag */ 0, /* doBitReverse */ 1);
            //arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_f32_10khz, /* ifftFlag */ 0, /* doBitReverse */ 1);
//            for (int i = 0; i < SAMPLES_IN_BUFFER; i++) {
////                printf("%f\r\n", testInput_f32_9khz[i*2]);
////            }
////            printf("\r\n\n\n");
//            
            arm_cmplx_mag_f32(samples_nrf, magnitudes, /* fftsize */ 128);
            //arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, /* fftsize */ 1024);
//            arm_max_f32(testOutput, 128, &maxValue, &testIndex);
            //printf("\n\rMax Value: %f, at index: %d.\r\n", maxValue, testIndex);
            
            uint32_t err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
            APP_ERROR_CHECK(err_code);
            samples_ready = false;
            
            // Remove noise and apply EQ levels to testOutputArduinoAttmpt
            uint16_t L = 0;
            for (i = 1; i < SAMPLES_IN_BUFFER/2; i++) {
//                // Make it match arduino output
//                //printf("%f\r\n", testOutput[i]);
//                //printf("%f\r\n", magnitudes[i]);
//                testOutputArduinoAttmpt[i] = (uint32_t)((uint32_t)magnitudes[i]/100);
//                //printf("%d:%d\r\n", i, testOutputArduinoAttmpt[i]);
//                //printf("%d\r\n", testOutputArduinoAttmpt[i]);
//                
                L = noise[i];
                magnitudes[i] = (magnitudes[i] <= L) ? 0 : magnitudes[i];
//                testOutputArduinoAttmpt[i] = (testOutputArduinoAttmpt[i] <= L) ? 0 :
//                    ((testOutputArduinoAttmpt[i] - L) * (256 - eq[i])) >> 8;
            } 
            
            // Fill background with colors, then idle parts of columns will erase
            NRF_bicolor_fillRect(&matrix, 0, 0, 8, 3, LED_RED);
            NRF_bicolor_fillRect(&matrix, 0, 3, 8, 2, LED_YELLOW);
            NRF_bicolor_fillRect(&matrix, 0, 5, 8, 3, LED_GREEN);
            
            // Calculate intensity and remove pixels from matrix
            #define SPECTRUM_MIN_DB 40.0
            #define SPECTRUM_MAX_DB 80.0
            float intensity, otherMean;
            for (i = 0; i < 8; i++) {
                windowMean(magnitudes, 
                    frequencyToBin(frequencyWindow[i]),
                    frequencyToBin(frequencyWindow[i+1]),
                    &intensity,
                    &otherMean);
                // Convert intensity to decibels.
                intensity = 20.0*log10(intensity);
                // Scale the intensity and clamp between 0 and 1.0.
                intensity -= SPECTRUM_MIN_DB;
                intensity = intensity < 0.0 ? 0.0 : intensity;
                intensity /= (SPECTRUM_MAX_DB-SPECTRUM_MIN_DB);
                intensity = intensity > 1.0 ? 1.0 : intensity;
                //printf("%f\r\n", intensity);
                NRF_bicolor_drawLine(&matrix, i, 0, i, 8 - (int)(8*intensity), LED_OFF);
                //pixels.setPixelColor(i, pixelHSVtoRGBColor(hues[i], 1.0, intensity));
            }
            //printf("\r\n\n\n");
            //nrf_delay_ms(100);
            
            NRF_bicolor_writeDisplay(&matrix);
            nrf_delay_ms(1);
            
            // Downsample spectrum output to 8 columns:
//            uint8_t sum = 0;
//            uint8_t x = 0;
//            uint16_t minLvl = 0;
//            uint16_t maxLvl = 0;
//            uint8_t weighting = 0;
//            uint8_t c = 0;
//            int y = 0;
//            int level = 0;
//            for (x = 0; x < 8; x++) {
//                //NRF_bicolor_writeDisplay(&matrix); // DEBUG
//                
//                data = (uint8_t *)colData[x];
//                nBins = data[0] + 2;
//                binNum = data[1];
//                for (sum = 0, i = 0; i < nBins; i++) {
//                    sum += testOutputArduinoAttmpt[binNum++] * data[i]; // Weighted
//                }
//                col[x][colCount] = sum / colDiv[x];                    // Average
//                minLvl = maxLvl = col[x][0];
//                for(i=1; i<10; i++) { // Get range of prior 10 frames
//                    if(col[x][i] < minLvl)      minLvl = col[x][i];
//                    else if(col[x][i] > maxLvl) maxLvl = col[x][i];
//                }
//                // minLvl and maxLvl indicate the extents of the FFT output, used
//                // for vertically scaling the output graph (so it looks interesting
//                // regardless of volume level).  If they're too close together though
//                // (e.g. at very low volume levels) the graph becomes super coarse
//                // and 'jumpy'...so keep some minimum distance between them (this
//                // also lets the graph go to zero when no sound is playing):
//                if((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
//                minLvlAvg[x] = (minLvlAvg[x] * 7 + minLvl) >> 3; // Dampen min/max levels
//                maxLvlAvg[x] = (maxLvlAvg[x] * 7 + maxLvl) >> 3; // (fake rolling average)
//                
//                // Second fixed-point scale based on dynamic min/max levels:
//                level = 10L * (col[x][colCount] - minLvlAvg[x]) /
//                  (long)(maxLvlAvg[x] - minLvlAvg[x]);
//                
//                // Clip output and convert to byte:
//                if(level < 0L)      c = 0;
//                else if(level > 10) c = 10; // Allow dot to go a couple pixels off top
//                else                c = (uint8_t)level;
//                
//                if(c > peak[x]) peak[x] = c; // Keep dot on top
//                
//                if(peak[x] <= 0) { // Empty column?
//                    NRF_bicolor_drawLine(&matrix, x, 0, x, 7, LED_OFF);
//                    continue;
//                } else if(c < 8) { // Partial column?
//                    NRF_bicolor_drawLine(&matrix, x, 0, x, 7 - c, LED_OFF);
//                }
//                
//                // The 'peak' dot color varies, but doesn't necessarily match
//                // the three screen regions...yellow has a little extra influence.
//                y = 8 - peak[x];
//                if(y < 2)      NRF_bicolor_drawPixel(&matrix, x, y, LED_RED);
//                else if(y < 6) NRF_bicolor_drawPixel(&matrix, x, y, LED_YELLOW);
//                else           NRF_bicolor_drawPixel(&matrix, x, y, LED_GREEN);
//            }
            //NRF_bicolor_drawPixel(&matrix, dotCount, 0, LED_GREEN);
            
            //NRF_bicolor_writeDisplay(&matrix);
            //nrf_delay_ms(1000);
            
//            // Every third frame, make the peak pixels drop by 1:
//            if(++dotCount >= 3) {
//                dotCount = 0;
//                for(x=0; x<8; x++) {
//                    if(peak[x] > 0) peak[x]--;
//                }
//            }
            
            //if(++colCount >= 10) colCount = 0;
        }
    }
}


/** @} */
