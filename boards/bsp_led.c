#include "bsp_led.h"
#include "main.h"

//Ö÷°åÌáÊ¾µÆ
void LED_on(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
}

void LED_off(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
}

