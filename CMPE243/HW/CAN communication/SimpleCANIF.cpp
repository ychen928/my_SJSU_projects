#include "app/SimpleCANIF.hpp"
#include <cstddef>
#include <cstring>

#include "io.hpp"
#include <stdio.h>

#define CAN_HANDLE can1
#define CAN_BAUD 250U
#define RX_QUEUE_SIZE 100U
#define TX_QUEUE_SIZE 50U

#define CHECK_10HZ_TIMER_AT_1S(timer) (timer % 10 == 0)
#define CHECK_10HZ_TIMER_5S_ELAPSED(timer) (timer > 50U)

#define GPIO_BTN_PIN P2_0

//void bus_off_cb(uint32_t b)
//{
//    CAN_reset_bus(can1);
//}

SimpleCANIF::SimpleCANIF()
{
    this->can_handle = CAN_HANDLE;
    (void)CAN_init(this->can_handle, CAN_BAUD, RX_QUEUE_SIZE, TX_QUEUE_SIZE, 0, 0);

    CAN_bypass_filter_accept_all_msgs();
    CAN_reset_bus(this->can_handle);

    this->recovery_timer = 0U;
    this->recovery_mode = false;

    this->btn_pin = new GPIO(GPIO_BTN_PIN);
    this->btn_pin->setAsInput();
    this->btn_pin->enablePullUp();
}


SimpleCANIF* SimpleCANIF::getInstance(void)
{
    static SimpleCANIF* instance = NULL;
    if (instance == NULL) {
        instance = new SimpleCANIF;
    }

    return instance;
}


void SimpleCANIF::task_10hz(void)
{
    if (CAN_is_bus_off(this->can_handle)) {
        CAN_reset_bus(this->can_handle);
        this->recovery_timer = 0U;
        this->recovery_mode = true;
    }

    if (!(this->recovery_mode)) {
        this->tx_btn_state();
    }
    else {
        this->recovery_timer++;
        if (CHECK_10HZ_TIMER_AT_1S(this->recovery_timer)) {
            this->tx_btn_state();
        }
        if (CHECK_10HZ_TIMER_5S_ELAPSED(this->recovery_timer)) {
            this->recovery_timer = 0U;
            this->recovery_mode = false;
        }
    }

    this->update_led_state();
}


void SimpleCANIF::tx_btn_state(void)
{
    can_msg_t msg;
    std::memset(&msg, 0, sizeof(msg));
    
    msg.msg_id = 0x123;
    msg.frame_fields.is_rtr = 0U;
    msg.frame_fields.is_29bit = 0U;
    if (this->btn_pin->read() == false) {
        msg.data.bytes[0] = 0xAA;
        puts("tick");
    }
    else {
        msg.data.bytes[0] = 0x00;
    }
    msg.frame_fields.data_len = 1U;

    (void)CAN_tx(this->can_handle, &msg, 0U);
//    printf("value:   %d\r\n", value);
}


void SimpleCANIF::update_led_state(void)
{
    can_msg_t msg;
    if (CAN_rx(this->can_handle, &msg, 0U)) {
//        puts("received");
//        printf("msg_id: %x  data: %x\r\n", msg.msg_id, msg.data.bytes[0]);
        switch (msg.msg_id) {
            case 0x123:
            {
                if (msg.frame_fields.data_len > 0) {
                    if (msg.data.bytes[0] == 0xAA) {
                        LE.on(1U);
                    }
                    else if (msg.data.bytes[0] == 0x00) {
                        LE.off(1U);
                    }
                    else {
                        // Pass
                    }
                }
            }
        }
    }
}
