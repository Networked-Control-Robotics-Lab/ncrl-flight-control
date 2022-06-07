#include <stm32f4xx.h>
#include <sys_time.h>

void i2c2_init(void)
{
	/* rcc initialization */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* gpio initialization:   *
	 * pb10 = scl，pb11 = sda */
	GPIO_InitTypeDef GPIO_InitStructure= {
		.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11,
		.GPIO_Speed = GPIO_Speed_100MHz,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_OD,
		.GPIO_PuPd  = GPIO_PuPd_NOPULL
	};
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* connect gpi pins to peripheral */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	/* i2c initialization */
	I2C_InitTypeDef I2C_InitStructure = {
		.I2C_Mode = I2C_Mode_I2C,
		.I2C_ClockSpeed = 100000, //set i2c clockspeed as 100kHz
		.I2C_DutyCycle = I2C_DutyCycle_2,
		.I2C_OwnAddress1 = 0, //stm32 own i2c address
		.I2C_Ack = I2C_Ack_Enable,
		.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit
	};
	I2C_Init(I2C2, &I2C_InitStructure);

	/* enable i2c */
	I2C_Cmd(I2C2,ENABLE);
}

int i2c_start(I2C_TypeDef* i2c, uint8_t address, uint8_t direction, float timeout)
{
	/* wait until i2c is not busy anymore */
	float start_time = get_sys_time_ms();
	while(I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY)) {
		if((get_sys_time_ms() - start_time) > timeout) {
			return 1;
		}
	};

	/* Send i2c start condition */
	I2C_GenerateSTART(i2c, ENABLE);

	/* wait for i2c ev5 --> slave has acknowledged start condition */
	while(!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT)) {
		if((get_sys_time_ms() - start_time) > timeout) {
			return 1;
		}
	};

	/* send slave address for write */
	I2C_Send7bitAddress(i2c, address, direction);

	/* wait for i2c ev6, check if either slave has acknowledged master transmitter or
	   master receiver mode, depending on the transmission direction */
	if(direction == I2C_Direction_Transmitter) {
		while(!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			if((get_sys_time_ms() - start_time) > timeout) {
				return 1;
			}
		};
	} else if(direction == I2C_Direction_Receiver) {
		while(!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			if((get_sys_time_ms() - start_time) > timeout) {
				return 1;
			}
		};
	}
	return 0;
}

int i2c_write(I2C_TypeDef* i2c, uint8_t data, float timeout)
{
	float start_time = get_sys_time_ms();
	I2C_SendData(i2c, data);

	/* wait until i2c byte transmission is complete */
	while(!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if((get_sys_time_ms() - start_time) > timeout) {
			return 1;
		}
	};
	return 0;
}

void i2c_stop(I2C_TypeDef* i2c)
{
	/* send i2c1 stop condition */
	I2C_GenerateSTOP(i2c, ENABLE);
}
