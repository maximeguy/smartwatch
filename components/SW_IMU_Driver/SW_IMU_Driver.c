/**
 * @file SW_IMU_Driver.h
 *
 * @author Majdi Rhim
 * @brief Inertial Measurement unit  IMU driver
 * @version 0.1
 * @date 2023-03-16
 *
 */


#include "SW_IMU_Driver.h"


lin_t lin_hum;
lin_t lin_temp;

/**
 * @brief Initializes the SW MEMS interface.
 * 
 * @param handle The handle of the interface.
 * @param Mems_type Type of Sensor interface being initialized (0=>Lsm6dso 1=>Lis2mdl 2=>lps22hh 3=>hts221)
 * @return stmdev_ctx_t Returns the STM device context.
 * 
 * @see platform_write()
 * @see platform_read()
 * @see platform_delay()
 */

stmdev_ctx_t SW_Mems_Interface_Init(uint8_t handle,uint8_t Mems_type){
	stmdev_ctx_t dev_ctx;
	if(Mems_type==0){
		/* Initialize mems driver interface */
		dev_ctx.write_reg = Lsm6dso_platform_write; // @suppress("Field cannot be resolved")
		dev_ctx.read_reg = Lsm6dso_platform_read; // @suppress("Field cannot be resolved")
		dev_ctx.handle = handle; // @suppress("Field cannot be resolved")
		return dev_ctx;
	}else if(Mems_type==1){
		/* Initialize Lis2mdl mems driver interface */
			dev_ctx.write_reg = Lis2mdl_platform_write;// @suppress("Field cannot be resolved")
			dev_ctx.read_reg = Lis2mdl_platform_read;// @suppress("Field cannot be resolved")
			dev_ctx.handle = handle;// @suppress("Field cannot be resolved")
	}else if(Mems_type==2){
		/* Initialize Lis2mdl mems driver interface */
			dev_ctx.write_reg = lps22hh_platform_write;// @suppress("Field cannot be resolved")
			dev_ctx.read_reg = lps22hh_platform_read;// @suppress("Field cannot be resolved")
			dev_ctx.handle = handle;// @suppress("Field cannot be resolved")
	}else if(Mems_type==3){
		/* Initialize Lis2mdl mems driver interface */
			dev_ctx.write_reg = hts221_platform_write;// @suppress("Field cannot be resolved")
			dev_ctx.read_reg = hts221_platform_read;// @suppress("Field cannot be resolved")
			dev_ctx.handle = handle;// @suppress("Field cannot be resolved")
	}
	/* Wait sensor boot time */
	platform_delay(10);
	return dev_ctx;
}

/**
 * @brief Initializes and configures the LSM6DSO sensor.
 * 
 * @param dev_ctx The device context for communication with the LSM6DSO sensor.
 * 
 * @note Enables interrupt generation on the INT1 pin for (Activity/Inactivity)
 */
void SW_Lsm6dso6_Init_Config(stmdev_ctx_t dev_ctx){
	uint8_t whoamI = 0, rst;
	lsm6dso_pin_int1_route_t int1_route;
	/* Check device ID */
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LSM6DSO_ID)
		while (1)
			SW_SafePrint(&UART_Jeton,"Device lsm6dso Not found %d\n\r",whoamI);

	/* Restore default configuration */
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lsm6dso_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Disable I3C interface */
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
	/* Set XL and Gyro Output Data Rate */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_208Hz);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_104Hz);
	/* Set 2g full XL scale and 250 dps full Gyro */
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_250dps);
	/* Set duration for Activity detection to 9.62 ms (= 2 * 1 / ODR_XL) */
	lsm6dso_wkup_dur_set(&dev_ctx, 0x02);
	/* Set duration for Inactivity detection to 4.92 s (= 2 * 512 / ODR_XL) */
	lsm6dso_act_sleep_dur_set(&dev_ctx, 0x02);
	/* Set Activity/Inactivity threshold to 125 mg */

	/*********** 1LSB = FS_XL/2^(6) ==> 5LSB = (2g/64)*5 * 10^(3) mg = 156.25 mg**************/
	lsm6dso_wkup_threshold_set(&dev_ctx, 0x10);

	/* Inactivity configuration: XL to 12.5 in LP, gyro to Power-Down */
	lsm6dso_act_mode_set(&dev_ctx, LSM6DSO_XL_12Hz5_GY_PD);
	/* Enable interrupt generation on Inactivity INT1 pin */
	lsm6dso_pin_int1_route_get(&dev_ctx, &int1_route);
	int1_route.sleep_change = PROPERTY_ENABLE;
	lsm6dso_pin_int1_route_set(&dev_ctx, int1_route);
}



/**
 * @brief Initializes and configures the Lis2mdl sensor.
 *
 * @param dev_ctx The device context for communication with the Lis2mdl sensor.
 *
 */
void SW_Lis2mdl_Init_Config(stmdev_ctx_t dev_ctx){
	uint8_t whoamI = 0;
	uint8_t rst;
	/* Check device ID */
	lis2mdl_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != LIS2MDL_ID)
		while (1)
			SW_SafePrint(&UART_Jeton,"Device LIS2MDL Not found %d\n\r",whoamI);

	/* Restore default configuration */
	lis2mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lis2mdl_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lis2mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lis2mdl_data_rate_set(&dev_ctx, LIS2MDL_ODR_10Hz);
	/* Set / Reset sensor mode */
	lis2mdl_set_rst_mode_set(&dev_ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
	/* Enable temperature compensation */
	lis2mdl_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Low-pass bandwidth to ODR/4 */
	//lis2mdl_low_pass_bandwidth_set(&dev_ctx, LIS2MDL_ODR_DIV_4);
	/* Set device in continuous mode */
	lis2mdl_operating_mode_set(&dev_ctx, LIS2MDL_CONTINUOUS_MODE);
	/* Enable interrupt generation on new data ready */
	lis2mdl_drdy_on_pin_set(&dev_ctx, PROPERTY_ENABLE);

}




/**
 * @brief Initializes and configures the Hts221 sensor.
 *
 * @param dev_ctx The device context for communication with the Hts221 sensor.
 *
 */

void SW_Hts221_Init_Config(stmdev_ctx_t dev_ctx){
	uint8_t whoamI = 0;
	hts221_device_id_get(&dev_ctx, &whoamI);

	if ( whoamI != HTS221_ID )
		while (1)
			SW_SafePrint(&UART_Jeton,"Device HTS221_ID Not found %d\n\r",whoamI);/*manage here device not found */

	/* Read humidity calibration coefficient */

	hts221_hum_adc_point_0_get(&dev_ctx, &lin_hum.x0);
	hts221_hum_rh_point_0_get(&dev_ctx, &lin_hum.y0);
	hts221_hum_adc_point_1_get(&dev_ctx, &lin_hum.x1);
	hts221_hum_rh_point_1_get(&dev_ctx, &lin_hum.y1);
	/* Read temperature calibration coefficient */

	hts221_temp_adc_point_0_get(&dev_ctx, &lin_temp.x0);
	hts221_temp_deg_point_0_get(&dev_ctx, &lin_temp.y0);
	hts221_temp_adc_point_1_get(&dev_ctx, &lin_temp.x1);
	hts221_temp_deg_point_1_get(&dev_ctx, &lin_temp.y1);
	/* Enable Block Data Update */
	hts221_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	hts221_data_rate_set(&dev_ctx, HTS221_ODR_1Hz);
	/* Device power on */
	hts221_power_on_set(&dev_ctx, PROPERTY_ENABLE);
}

/**
 * @brief Initializes and configures the Lps22hh sensor.
 *
 * @param dev_ctx The device context for communication with the Lps22hh sensor.
 *
 */

void SW_Lps22hh_Init_Config(stmdev_ctx_t dev_ctx){
	/* Check device ID */
	uint8_t whoamI = 0;
	uint8_t rst;
	lps22hh_device_id_get(&dev_ctx, &whoamI);

	if ( whoamI != LPS22HH_ID )
		while (1)
			SW_SafePrint(&UART_Jeton,"Device Lps22hh Not found %d\n\r",whoamI);/*manage here device not found */

	/* Restore default configuration */
	lps22hh_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lps22hh_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lps22hh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lps22hh_data_rate_set(&dev_ctx, LPS22HH_10_Hz_LOW_NOISE);
}


/**
 * @brief Determines the north direction relative to the Lis2mdl sensor
 *
 * @return North direction in degrees
 */
double SW_North_Dir(int16_t* data_raw_magnetic){
	float magnetic_mG[3];
	double norm_x,norm_y,angle,angle_degree;
	magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[0]);
	magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[1]);
	magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[2]);
	norm_x=magnetic_mG[0]/sqrt(magnetic_mG[0]*magnetic_mG[0] + magnetic_mG[1]*magnetic_mG[1]+magnetic_mG[2]*magnetic_mG[2]); //Normalize
	norm_y=magnetic_mG[1]/sqrt(magnetic_mG[0]*magnetic_mG[0] + magnetic_mG[1]*magnetic_mG[1]+magnetic_mG[2]*magnetic_mG[2]); //Normalize
	angle = atan2(norm_x,norm_y);
	angle_degree = angle*(180/M_PI)-1.1333; // 1.1333 poitiers-declination
	return angle_degree;
}



static float linear_interpolation(lin_t *lin, int16_t x)
{
	return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
			(lin->x0 * lin->y1)))
			/ (lin->x1 - lin->x0);
}
/**
 * @brief get temperature/humidity/Pressure
 *
 * @return struct Weather
 */

Weather SW_Get_Weather(stmdev_ctx_t hts221_dev_ctx,stmdev_ctx_t lps22hh_dev_ctx){
	Weather  weather;
	int16_t data_raw_humidity=0;
	int16_t data_raw_temperature=0;
	float humidity_perc=0;
	lin_t lin_hum;
	float temperature_degC=0.0;
	uint32_t data_raw_pressure=0;
	float pressure_hPa=0.0;
	lps22hh_reg_t reg_Pressure;
	/* Read output only if new value is available */
	hts221_reg_t reg_hts221;
	hts221_status_get(&hts221_dev_ctx, &reg_hts221.status_reg);

	if (reg_hts221.status_reg.h_da) {
		/* Read humidity data */
		memset(&data_raw_humidity, 0x00, sizeof(int16_t));
		hts221_humidity_raw_get(&hts221_dev_ctx, &data_raw_humidity);
		humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity);
	}
	if (humidity_perc < 0) {
		humidity_perc = 0;
		weather.Humdity=humidity_perc;
	}

	if (humidity_perc > 100) {
		humidity_perc = 100;
		weather.Humdity=humidity_perc;
	}
	if (reg_hts221.status_reg.t_da) {
		/* Read temperature data */
		memset(&data_raw_temperature, 0x00, sizeof(int16_t));
		hts221_temperature_raw_get(&hts221_dev_ctx, &data_raw_temperature);
		temperature_degC = linear_interpolation(&lin_temp,data_raw_temperature);
		weather.Temperature=temperature_degC;
	}
	/* Read output only if new value is available */
	lps22hh_read_reg(&lps22hh_dev_ctx, LPS22HH_STATUS, (uint8_t *)&reg_Pressure, 1);

	if (reg_Pressure.status.p_da) {
		memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
		lps22hh_pressure_raw_get(&lps22hh_dev_ctx, &data_raw_pressure);
		pressure_hPa = lps22hh_from_lsb_to_hpa( data_raw_pressure);
		weather.Pressure=pressure_hPa;
	}
	return weather;
}


int32_t Lsm6dso_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len)
{
	return SW_Thread_Safe_I2c_Write(&I2c_Jeton, handle, LSM6DSO_ADDR_7BITS, reg, bufp, len);
}



int32_t Lsm6dso_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{
	return  SW_Thread_Safe_I2c_Read(&I2c_Jeton, handle, LSM6DSO_ADDR_7BITS, reg, bufp, len);
}




int32_t Lis2mdl_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len)
{
	/* Write multiple command */
	reg |= 0x80;
	return SW_Thread_Safe_I2c_Write(&I2c_Jeton, handle, LIS2MDL_ADDR_7BITS, reg, bufp, len);
}
int32_t Lis2mdl_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{
	/* Read multiple command */
	reg |= 0x80;
	return  SW_Thread_Safe_I2c_Read(&I2c_Jeton, handle, LIS2MDL_ADDR_7BITS, reg, bufp, len);
}


int32_t lps22hh_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len)
{
	/* Write multiple command */
	reg |= 0x80;
	return SW_Thread_Safe_I2c_Write(&I2c_Jeton, handle, LPS22hh_ADDR_7BITS, reg, bufp, len);
}
int32_t lps22hh_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{
	/* Read multiple command */
	reg |= 0x80;
	return  SW_Thread_Safe_I2c_Read(&I2c_Jeton, handle, LPS22hh_ADDR_7BITS, reg, bufp, len);
}


int32_t hts221_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len)
{
	/* Write multiple command */
	reg |= 0x80;
	return SW_Thread_Safe_I2c_Write(&I2c_Jeton, handle, HTS221_ADDR_7BITS, reg, bufp, len);
}
int32_t hts221_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{
	/* Read multiple command */
	reg |= 0x80;
	return  SW_Thread_Safe_I2c_Read(&I2c_Jeton, handle, HTS221_ADDR_7BITS, reg, bufp, len);
}


void platform_delay(uint32_t ms){
	vTaskDelay(ms/portTICK_PERIOD_MS);
}
