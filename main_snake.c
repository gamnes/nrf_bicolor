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

#include "arm_math.h"
#include "arm_const_structs.h"

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1   /**< UART RX buffer size. */

#define SAMPLES_IN_BUFFER 128
volatile uint8_t state = 1;

static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;

static float testInput_f32_9khz[SAMPLES_IN_BUFFER * 2];
static float testOutput[SAMPLES_IN_BUFFER];

//extern float testInput_f32_10khz[SAMPLES_IN_BUFFER * 2];  // For testing


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
    //uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 2);
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
        //for (int i = 30; i < SAMPLES_IN_BUFFER; i++)
        //{
        //    printf("%d:%d\r\n", i, p_event->data.done.p_buffer[i]);
        //}
        m_adc_evt_counter++;
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
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
// Borrowed code from http://cplus.about.com/od/learningc/a/Game-Programming-In-C-Tutorial-Four-Snake.htm
#define HEIGHT 8
#define WIDTH 8
#define SNHEADWEST 2
#define SNHEADNORTH 3
#define SNHEADEAST 4
#define SNHEADSOUTH 5
#define SNTAILWEST 6
#define SNTAILNORTH 7
#define SNTAILEAST 8
#define SNTAILSOUTH 9
#define SNAKE 1
#define l(X,Y)(Y*WIDTH)+X  // Used for determining int index of snake array matching x,y or board

uint8_t board[WIDTH][HEIGHT] = {0};
// Only Headindex, segment next to headindex, and tailindex change each time it moves
uint8_t headindex = 1;
uint8_t tailindex = 0;
uint8_t atefood = 0;
// Store every location of snake in a ring buffer. Every location is a single int coordinate
// Data is pushed on the front of the queue, and taken off the back.
// If the front hits the end of block, wraps around
uint8_t snake[WIDTH*HEIGHT];
uint8_t dir[WIDTH*HEIGHT];

void initSnake() {
    headindex = 1;
    tailindex = 0;
    atefood = 0;
    memset(snake, 0, sizeof(snake));
    memset(dir, 0, sizeof(dir));
    snake[headindex] = l(5,5);
    snake[tailindex] = l(4,5);
    dir[snake[headindex]] = 0;
}

void drawSnake(void) {
    board[0][0] = SNAKE;
}

void renderScreen(void) {
    drawSnake();
}

void print_board(void) {
    for(uint8_t y = 0; y < 8; y++) {
        printf("|");
        for(uint8_t x = 0; x < 8; x++) {
           if (board[x][y] == SNAKE) printf("O");
        }
        printf("|\r\n");
    }
    printf("----------\r\n");
    nrf_delay_ms(100);
}
int main(void)
{
    float maxValue;
    uint32_t testIndex = 0;
    
    uart_config();

    printf("\n\rSAADC HAL simple example.\r\n");
    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();

    initSnake();
    while(1)
    {
        print_board();
    }
}


/** @} */
