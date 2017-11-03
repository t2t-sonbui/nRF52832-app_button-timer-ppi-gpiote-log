/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup bsp_example_main main.c
 * @{
 * @ingroup bsp_example
 * @brief BSP Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_error.h"

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "app_button.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "nrf_delay.h"
#include "nrf_drv_ppi.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SUCCESS (0)
#define ERROR   (1)

#define GPIO_BUTTON_PIN_PPI BUTTON_1
#define GPIO_OUT_PIN_PPI LED_1

#define PIN_IN_GPIOTE BUTTON_3
#define PIN_OUT_GPIOTE LED_3

#define BUTTON_PIN_APP BUTTON_2
#define GPIO_OUT_PIN_APP LED_2  


#define BUTTON_DELAY (10)

APP_TIMER_DEF(s_timer_id);

const nrf_drv_timer_t timer2 = NRF_DRV_TIMER_INSTANCE(2);
const nrf_drv_timer_t timer3 = NRF_DRV_TIMER_INSTANCE(3);
static nrf_ppi_channel_t ppi_channel_button;
static nrf_ppi_channel_t ppi_channel_timer;
// Timer even handler. Not used since timer is used only for PPI.
void timer_event_handler(nrf_timer_event_t event_type, void * p_context){}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_out_toggle(PIN_OUT_GPIOTE);
}
void in_pin_handler_no_thing(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  
}

static void timer3_init(void)
{
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.mode = NRF_TIMER_MODE_TIMER;
		timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
		timer_cfg.bit_width=NRF_TIMER_BIT_WIDTH_32;
    ret_code_t err_code = nrf_drv_timer_init(&timer3, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);
	//uint32_t time_ticks = nrf_drv_timer_us_to_ticks(&timer1, TIMER_TIMEOUT);
	//nrf_drv_timer_extended_compare(&timer1, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
}
static void timer2_init(void)
{
    // Generate interrupt/event when half of time before the timer overflows has past, that is at 1,3,5,7... seconds from start.
    // Check TIMER1 configuration for details
    // now the overflow occurs every 0xFFFF/(SysClk/2^PRESCALER)
    // = 65535/31250 = 2.097 sec */
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    ret_code_t err_code = nrf_drv_timer_init(&timer2, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);
		uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&timer2, 1000);
    nrf_drv_timer_extended_compare(&timer2, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT_GPIOTE, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN_GPIOTE, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN_GPIOTE, true);
}

/**@brief Function for initializing low frequency clock.
 */
void clock_initialization()
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}
static void button_press_handler(uint8_t pin_no, uint8_t button_action)
{
    if (button_action == APP_BUTTON_PUSH) {
				nrf_gpio_pin_set(GPIO_OUT_PIN_APP);
    } else {
        // APP_BUTTON_RELEASE
				nrf_gpio_pin_clear(GPIO_OUT_PIN_APP);

    }
}

static void button_timeout_handler(void *p_context)
{
	 NRF_LOG_INFO("Timeout!");
}


static void test_app_button_init(void)
{
	uint32_t err_code ;
		nrf_gpio_cfg_output(GPIO_OUT_PIN_APP);
// Button config.
    static app_button_cfg_t s_p_button_cfg[2];
    s_p_button_cfg[0] = (app_button_cfg_t) {
        .pin_no = BUTTON_PIN_APP,
        .active_state = APP_BUTTON_ACTIVE_LOW,
        .pull_cfg = NRF_GPIO_PIN_PULLUP,
        .button_handler = button_press_handler
    };
//			s_p_button_cfg[1] = (app_button_cfg_t) {
//        .pin_no = GPIO_BUTTON_PIN_PPI,
//        .active_state = APP_BUTTON_ACTIVE_LOW,
//        .pull_cfg = NRF_GPIO_PIN_PULLUP,
//        .button_handler = button_press_handler
//		};
			 // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&s_timer_id, APP_TIMER_MODE_SINGLE_SHOT, button_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_init(
        s_p_button_cfg, sizeof s_p_button_cfg / sizeof s_p_button_cfg[0],
        BUTTON_DELAY
    );
    APP_ERROR_CHECK(err_code);
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}
static void ppi_init(void)
{
    uint32_t err_code = NRF_SUCCESS;
	
		timer3_init();
		timer2_init();
	
		if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
				APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_ppi_init();
	  APP_ERROR_CHECK(err_code);
	
	  nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
	  err_code = nrf_drv_gpiote_out_init(GPIO_OUT_PIN_PPI, &out_config);
    APP_ERROR_CHECK(err_code);
	
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrf_drv_gpiote_in_init(GPIO_BUTTON_PIN_PPI, &in_config, in_pin_handler_no_thing);
    APP_ERROR_CHECK(err_code);

	
	
		uint32_t	gpiote_task_addr = nrf_drv_gpiote_out_task_addr_get(GPIO_OUT_PIN_PPI);
		uint32_t gpiote_event_addr = nrf_drv_gpiote_in_event_addr_get(GPIO_BUTTON_PIN_PPI);
	
		
		uint32_t timer_task_capture_addr =  nrf_drv_timer_task_address_get(&timer3, NRF_TIMER_TASK_CAPTURE0);//nrf_drv_timer_capture_task_address_get(&timer3, NRF_TIMER_CC_CHANNEL0);
		
		uint32_t timer_task_clear_addr =  nrf_drv_timer_task_address_get(&timer2, NRF_TIMER_TASK_CLEAR);
		uint32_t timer_even_timeout_addr = nrf_drv_timer_event_address_get(&timer2, NRF_TIMER_EVENT_COMPARE0);
		uint32_t timer_task_start_addr =  nrf_drv_timer_task_address_get(&timer2, NRF_TIMER_TASK_START);
		uint32_t timer_task_stop_addr =  nrf_drv_timer_task_address_get(&timer2, NRF_TIMER_TASK_STOP);

    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_timer);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(ppi_channel_timer,timer_even_timeout_addr,timer_task_capture_addr);//When timer 3 timeout-> capture value of timer 3(us)
    APP_ERROR_CHECK(err_code);
 //Enable configured PPI channels
    err_code = nrf_drv_ppi_channel_enable(ppi_channel_timer);
    APP_ERROR_CHECK(err_code);
		
		
	
    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_button);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(ppi_channel_button,gpiote_event_addr,gpiote_task_addr);
    APP_ERROR_CHECK(err_code);
// //Enable configured PPI channels
		err_code = nrf_drv_ppi_channel_enable(ppi_channel_button);
		

		APP_ERROR_CHECK(err_code);
	 	nrf_drv_gpiote_out_task_enable(GPIO_OUT_PIN_PPI);
		nrf_drv_gpiote_in_event_enable(GPIO_BUTTON_PIN_PPI, false);
		

		nrf_drv_timer_enable(&timer3);
		nrf_drv_timer_enable(&timer2);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
	
   clock_initialization();//App button need

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Example started.");
	
		gpio_init();
		test_app_button_init();
		ppi_init();
	
//    while (true)
//    {
//        NRF_LOG_FLUSH();
//        __SEV();
//        __WFE();
//        __WFE();
//        // no implementation needed
//			
//    }
		while (true)
    {
       //NRF_LOG_INFO("Current count: %lu",nrf_timer_cc_read(timer1.p_reg, NRF_TIMER_CC_CHANNEL0));
			 NRF_LOG_INFO("Current count: %lu",(uint32_t)timer3.p_reg->CC[NRF_TIMER_CC_CHANNEL0]);
       nrf_delay_ms(100);
       NRF_LOG_FLUSH();
			
    }
}


/** @} */
