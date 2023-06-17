#ifndef LED_H
#define LED_H
#include "stm32h7xx_hal.h"
#include "stm32h723xx.h"
namespace STM32LED
{
    enum class state
    {
        on,
        off
    };

    class LED
    {
        public:
            LED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin);
            void set(state s);
            void toggle();
            state LED_state;

        protected:
            GPIO_TypeDef* gpio_port;
            uint16_t gpio_pin;

        
    }
    extern LED* led1;
}

#endif
