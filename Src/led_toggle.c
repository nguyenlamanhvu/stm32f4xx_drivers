#include "stm32f407xx.h"
#include <stdint.h>

void Delay(void)
{
	for(int i=0; i< 1000000; i++);
}

int main(void)
{

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GpioLed);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		Delay();
	}
}
