#include "app/ledif.hpp"
#include "eint.h"
#include "io.hpp"

#include <stdio.h>

#define TASK_NAME "LEDIF"
#define STACK_SIZE 512U

#define ASSERT_LED led_pin->setLow();
#define DEASSERT_LED led_pin->setHigh();

#define CMD_LED_ASSERT led_out->setHigh();
#define CMD_LED_DEASSERT led_out->setLow();

#define EINT_PORT2_BTN 2U


LedIf LEDIF(PRIORITY_HIGH);

static void _btn_cb_wrapper(void);
static void _timer_cb(TimerHandle_t pxTimer);


LedIf::LedIf(uint8_t priority) : scheduler_task(TASK_NAME, STACK_SIZE, priority)
{
    // Pass
}

bool LedIf::init(void) 
{
    _init_gpio();
    assert_led = false;
    btn_timer = xTimerCreate("btn_timer", 500U, pdFALSE, 0, (TimerCallbackFunction_t)_timer_cb);
    btn_sem = xSemaphoreCreateBinary();
    return true;
}


bool LedIf::run(void *p)
{
    if (xSemaphoreTake(btn_sem, 0U)) {
        xTimerReset(btn_timer, 0U);
        CMD_LED_ASSERT;
    }
    else {
        if (!xTimerIsTimerActive(btn_timer)) {
            CMD_LED_DEASSERT;
        }
    }

    if (led_in->read()) {
        ASSERT_LED;
    }
    else {
        DEASSERT_LED;
    }

    return true;
}


void LedIf::_init_gpio(void)
{
    btn_in = new GPIO(P2_2);
    btn_in->setAsInput();
    btn_in->enablePullDown();

    led_pin = new GPIO(P2_1);
    led_pin->setAsOutput();
    DEASSERT_LED;

    led_out = new GPIO(P2_3);
    led_out->setAsOutput();
    CMD_LED_DEASSERT;

    led_in = new GPIO(P2_0);
    led_in->setAsInput();
    led_in->enablePullDown();
}

void LedIf::btn_cb(void)
{
    xSemaphoreGiveFromISR(btn_sem, 0);
}




void LEDIF_init_eint(void)
{
    eint3_enable_port2(EINT_PORT2_BTN, eint_rising_edge, (void_func_t)_btn_cb_wrapper);
}

void _btn_cb_wrapper(void)
{
    LEDIF.btn_cb();
}

void _timer_cb(TimerHandle_t pxTimer)
{
    // Pass
}
