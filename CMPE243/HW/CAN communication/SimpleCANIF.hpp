#ifndef SIMPLECANIF_HPP
#define SIMPLECANIF_HPP

#include <stdbool.h>

#include "can.h"
#include "gpio.hpp"


class SimpleCANIF
{
    public:
        static SimpleCANIF* getInstance(void);
        void task_10hz(void);

    private:
        SimpleCANIF();
        void tx_btn_state(void);
        void update_led_state(void);

        can_t can_handle;
        uint32_t recovery_timer;
        bool recovery_mode;
        GPIO* btn_pin;
};


#endif
