#ifndef L5_APPLICATION_APP_LEDIF_HPP_
#define L5_APPLICATION_APP_LEDIF_HPP_

#include <stdbool.h>
#include <stdint.h>

#include "gpio.hpp"
#include "scheduler_task.hpp"

// RTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#define LEDIF_EINT_PORT2_BTN 2U


class LedIf : public scheduler_task
{
    public:
        LedIf(uint8_t priority);
        void btn_cb(void);
        bool init(void);
        bool run(void *p);

        SemaphoreHandle_t btn_sem;

    protected:

    private:
        bool assert_led;
        TimerHandle_t btn_timer;
        GPIO *led_out;  // Command external source to assert LED
        GPIO *led_in;  // Receive command to assert own LED
        GPIO *btn_in;  // Recieve command to assert LED of external source
        GPIO *led_pin;  // Drive onboard LED

        void _init_gpio(void);
};


extern LedIf LEDIF;

void LEDIF_init_eint(void);

#endif
