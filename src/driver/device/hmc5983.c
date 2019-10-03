#include "stm32f4xx_conf.h"

#include "spi.h"
#include "hmc5983.h"

#include "vector.h"

#define HMC5893_MAG_SCALE HMC5983_SCALE_1

void hmc5983_read(uint8_t register_address, uint8_t *data, int data_count)
{
	hmc5983_chip_select();

	//Write the register address
	spi_read_write(SPI1, register_address | 0xC0); //Continuous byte read

	//Read the data
	int i;
	for(i = 0; i < data_count; i++)
		data[i] = spi_read_write(SPI1, 0x00);

	hmc5983_chip_deselect();
}

void hmc5983_write(uint8_t register_address, uint8_t data)
{
	hmc5983_chip_select();

	//Write the register address
	spi_read_write(SPI1, register_address);
	//Write the data
	spi_read_write(SPI1, data);

	hmc5983_chip_deselect();
}

int hmc5983_read_identification()
{
	uint8_t data[3] = {0}; //Identification register A B and C
	hmc5983_read(HMC5983_ID_A, data, 3);

	/* If can't correctly read the hmc5983 identification register then return 1 */
	if(data[0] != 0x48 || data[1] != 0x34 || data[2] != 0x33) return 1;

	return 0;
}

int hmc5983_init()
{
	/* Check HMC5983 device is alive or not */
	if(hmc5983_read_identification() == 1) return 1;

	//HMC5983 configuration A :
	//(1)Enable temperature sensor compensation
	//(2)samples averaged : 8 per measurement output
	//(3)Typical data output rate : 220Hz
	hmc5983_write(HMC5983_CONF_A, 0xFC);

	//MC5983 configuration B : highest resolution
	hmc5983_write(HMC5983_CONF_B, 0x00);

	//HMC5983 mode : continuous-measurement mode
	hmc5983_write(HMC5983_MODE, 0x00);

	return 0;
}

void hmc5983_read_unscaled_data(vector3d_16_t *mag_unscaled_data)
{
	uint8_t buffer[6];

	/* Get the new data */
	hmc5983_read(0x03, buffer, 6);

	mag_unscaled_data->x = (buffer[0] << 8) | buffer[1];
	mag_unscaled_data->y = (buffer[4] << 8) | buffer[5];
	mag_unscaled_data->z = (buffer[2] << 8) | buffer[3];
}

void hmc5983_mag_convert_to_scale(vector3d_16_t *mag_unscaled_data,
                                  vector3d_f_t *mag_scaled_data)
{
	mag_scaled_data->x = mag_unscaled_data->x * HMC5893_MAG_SCALE;
	mag_scaled_data->y = mag_unscaled_data->y * HMC5893_MAG_SCALE;
	mag_scaled_data->z = mag_unscaled_data->z * HMC5893_MAG_SCALE;
}
