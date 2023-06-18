#include "LED.h"

namespace STM32LED
{
    LED* LED2;
    LED* LED3;

    LED::LED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin)
    {
        this->gpio_port = gpio_port;
        this->gpio_pin = gpio_pin;
        this->LED_state = state::off;   //default state is off
    }

    void LED::set(state s)
    {
        return;
    }

    void LED::toggle()
    {
       return;
    }

}