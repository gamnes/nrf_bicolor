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

static float testInput_f32_9khz[SAMPLES_IN_BUFFER * 2];
static float testOutput[SAMPLES_IN_BUFFER];
static uint32_t testOutputArduinoAttmpt[SAMPLES_IN_BUFFER/2];  // Only the first half of bins are usefull

// This is low-level noise that's subtracted from each FFT output column:
static const uint8_t noise[64] = {
    8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
    2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
    3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4
};

  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
static const uint8_t eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 };

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
        UART_BAUDRATE_BAUDRATE_Baud1M
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
    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer, 104);  // Every 104 uS should give roughly 9615 Hz
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
        ret_code_t err_code;
     
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

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
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for main application entry.
 */
uint8_t const bicolor_oscillator_cmdd_on = 0x21;
int main(void)
{
    float maxValue;
    uint32_t testIndex = 0;
    
    uart_config();
    
    NRF_bicolor matrix;
    NRF_bicolor_TWI_init();
    NRF_bicolor_begin(&matrix, 0x70);
    NRF_bicolor_blinkRate(&matrix, 1);
    nrf_delay_ms(2000);
    NRF_bicolor_blinkRate(&matrix, 2);
    nrf_delay_ms(2000);
    NRF_bicolor_blinkRate(&matrix, 0);

    printf("\n\rSAADC HAL simple example.\r\n");
    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();

    while(1)
    {
        __WFE();
        if (samples_ready) {
            nrf_drv_timer_pause(&m_timer);
            printf("\n\rSamples are ready to be used.\r\n");
            static const int16_t noiseThreshold = 4;
            for (int i = 0; i < SAMPLES_IN_BUFFER; i++) {
                if (m_buffer_pool[0][i] > (512 - noiseThreshold) && m_buffer_pool[0][i] < (512 + noiseThreshold)) {
                    testInput_f32_9khz[i*2] = 0;
                } else {
                    testInput_f32_9khz[i*2] = (float)(m_buffer_pool[0][i] - 512);  // Sign-convert for FFT; -512 to 512
                }
                //testInput_f32_9khz[i*2] = (float)m_buffer_pool[0][i];
                //testInput_f32_9khz[i*2] = (i % 2) ? 800 : 400;
                testInput_f32_9khz[i*2 + 1] = 0;
            }
//            
            arm_cfft_f32(&arm_cfft_sR_f32_len128, testInput_f32_9khz, /* ifftFlag */ 0, /* doBitReverse */ 1);
//            //arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_f32_10khz, /* ifftFlag */ 0, /* doBitReverse */ 1);
            arm_cmplx_mag_f32(testInput_f32_9khz, testOutput, /* fftsize */ 128);
//            //arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, /* fftsize */ 1024);
            arm_max_f32(testOutput, 128, &maxValue, &testIndex);
            printf("\n\rMax Value: %f, at index: %d.\r\n", maxValue, testIndex);
            
            // Remove noise and apply EQ levels to testOutputArduinoAttmpt
            uint8_t L = 0;
            for (int i = 0; i < SAMPLES_IN_BUFFER/2; i++) {
                // Make it match arduino output
                testOutputArduinoAttmpt[i] = (uint32_t)((uint32_t)testOutput[i]/100);
                //printf("%d:%d\r\n", i, testOutputArduinoAttmpt[i]);
                
                L = noise[i];
                testOutputArduinoAttmpt[i] = (testOutputArduinoAttmpt[i] <= L) ? 0 :
                    ((testOutputArduinoAttmpt[i] - L) * (256 - eq[i])) >> 8;
                printf("%d:%d\r\n", i, testOutputArduinoAttmpt[i]);
            }
//            
//            nrf_delay_ms(100);
            samples_ready = false;
            nrf_drv_timer_resume(&m_timer);
        }
    }
}


/** @} */
