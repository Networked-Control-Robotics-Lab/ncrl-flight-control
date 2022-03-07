#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <timer.h>

void init_GPIOE()
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType =GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_Init(GPIOE, &GPIO_InitStruct);
}

int main()
{
	init_GPIOE();
	timer3_init();
	int a = 400;	
	while(1) {
		
		int timerValue = TIM_GetCounter(TIM3);
		/*
		if(timerValue == 0){
			a--;
			if(a == 0){
			GPIO_ToggleBits(GPIOE,GPIO_Pin_12);
			a =400;
			}
		}*/
	}

	return 0;
}
