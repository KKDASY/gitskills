#include "bsp_led.h"

RGBState_t led_state;

void LED_RGB_Controller(RGBState_t *state)
{
    if(state->bits.LED_Flash)
    {
        if(state->bits.LED_R)
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        if(state->bits.LED_G)
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        if(state->bits.LED_B)
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }
    else
    {
        if(state->bits.LED_R)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        if(state->bits.LED_G)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        if(state->bits.LED_B)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        }
    }
}

void RED_ON(void)
{
    led_state.bits.LED_R = 1;
    led_state.bits.LED_Flash = 0;
    led_state.bits.LED_B = 0;
    led_state.bits.LED_G = 0;
}

void GREEN_ON(void)
{
    led_state.bits.LED_G = 1;
    led_state.bits.LED_Flash = 0;
    led_state.bits.LED_R = 0;
    led_state.bits.LED_B = 0;
}

void YELLOW_ON(void)
{
    led_state.bits.LED_B = 1;
    led_state.bits.LED_Flash = 0;
    led_state.bits.LED_R = 0;
    led_state.bits.LED_G = 0;
}

