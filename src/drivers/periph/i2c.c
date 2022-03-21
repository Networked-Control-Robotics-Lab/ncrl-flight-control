#include <stm32f4xx.h>
#include <sys_time.h>

void Init_I2C()
{
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* start I2C2 RCC clock*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* set I2C2  pin SDA & SCL */
	/* PB10 = SCL ，PB11 = SDA */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Connect GPIO pins to peripheral
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	/* set I2C2 */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 100000 ; // set I2C clockspeed as 100K
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0; // STM32 own I2C address
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C2, &I2C_InitStructure);

	/* start I2C */
	I2C_Cmd(I2C2,ENABLE);
}

int I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint32_t timeout)
{
	// wait until I2C1 is not busy anymore
	uint32_t start_tick = get_sys_tick_ms();
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) {
		if( (get_sys_tick_ms()-start_tick) > timeout  ) {
			return 1;
		}
	};

	// Send I2Cx START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2Cx EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {
		if( (get_sys_tick_ms()-start_tick) > timeout  ) {
			return 1;
		}
	};

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	* either Slave has acknowledged Master transmitter or
	* Master receiver mode, depending on the transmission
	* direction
	*/
	if(direction == I2C_Direction_Transmitter) {
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			if( (get_sys_tick_ms()-start_tick) > timeout  ) {
				return 1;
			}
		};
	} else if(direction == I2C_Direction_Receiver) {
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			if( (get_sys_tick_ms()-start_tick) > timeout  ) {
				return 1;
			}
		};
	}
	return 0;
}

int I2C_write(I2C_TypeDef* I2Cx, uint8_t data, uint32_t timeout)
{
	uint32_t start_tick = get_sys_tick_ms();
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if( (get_sys_tick_ms()-start_tick) > timeout  ) {
			return 1;
		}
	};
	return 0;
}

void I2C_stop(I2C_TypeDef* I2Cx)
{
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}