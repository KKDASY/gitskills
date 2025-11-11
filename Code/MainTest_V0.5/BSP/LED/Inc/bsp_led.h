#ifndef RGB_H
#define RGB_H

#include "stdbool.h"
#include "main.h"

typedef union
{
    uint8_t raw_value;
    struct
    {
        uint8_t LED_R : 1;
        uint8_t LED_G : 1;
        uint8_t LED_B : 1;
        uint8_t LED_Flash : 1;
    }bits;
}RGBState_t;

void LED_RGB_Controller(RGBState_t *state);

#define FLASH_ON    led_state.bits.LED_Flash = 1;
#define FLASH_OFF   led_state.bits.LED_Flash = 0;

void RED_ON(void);
void GREEN_ON(void);
void YELLOW_ON(void);


#endif






