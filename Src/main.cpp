#include"stm32f446xx.h"
#include "GPIOHandlet.h"

#include<stdint.h>

int main()
{
	GPIO_Handle_t GPIO_class;

	GPIO_class.pGPIOx = GPIOA;
	GPIO_class.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_class.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_class.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_class.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_class.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_class.GPIO_PeriClkCntrl(GPIOA, ENABLE);
	GPIO_class.GPIO_Init(&GPIO_class);

	while(1)
	{
		GPIO_class.GPIO_ToggleOPin(GPIOA, GPIO_PIN_5);

		for(uint32_t i=0; i<300000; i++);
	}
}
