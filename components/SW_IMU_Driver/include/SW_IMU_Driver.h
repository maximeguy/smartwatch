/**
 * @file SW_IMU_Driver.h
 * @author Majdi Rhim
 * @brief LSM6DSO sensor
 * @version 0.1
 * @date 2023-03-16
 *
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "SW_I2c_Driver.h"
#include "lsm6dso_reg.h"
#include "lis2mdl_reg.h"
#include "lps22hh_reg.h"
#include "hts221_reg.h"
#include "Weather_struct.h"
#include "string.h"
#define LSM6DSO_ADDR_7BITS 0x6B //7bit lsm6dso address

#define LIS2MDL_ADDR_7BITS 0x1E //7bit lis2mdl address

#define LPS22hh_ADDR_7BITS 0x5D //7bit lps22hh address

#define HTS221_ADDR_7BITS 0x5F //7bit hts221 address


/**
 * @note Use SW_SafePrint for thread safe UART communication
 */
extern SemaphoreHandle_t UART_Jeton;

/*
 * Use this Token for all I2c0 Transactions
 */
extern SemaphoreHandle_t I2c_Jeton ;


/*
 *  Function used to apply coefficient ==> hts221
 */
typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;


esp_err_t SW_SafePrint(SemaphoreHandle_t* Jeton,const char* fmt, ...);

/**
 * @brief Initializes the SW MEMS interface.
 *
 * @param handle The handle of the interface.
 * @return stmdev_ctx_t Returns the STM device context.
 *
 * @see platform_write()
 * @see platform_read()
 * @see platform_delay()
 */
stmdev_ctx_t SW_Mems_Interface_Init(uint8_t handle,uint8_t Mems_type);

/**
 * @brief Initializes and configures the LSM6DSO sensor.
 *
 * @param dev_ctx The device context for communication with the LSM6DSO sensor.
 *
 * @note Enables interrupt generation on the INT1 pin for (Activity/Inactivity)
 */

void SW_Lsm6dso6_Init_Config(stmdev_ctx_t dev_ctx);
/**
 * @brief Initializes and configures the Lis2mdl sensor.
 *
 * @param dev_ctx The device context for communication with the Lis2mdl sensor.
 *
 */

void SW_Lis2mdl_Init_Config(stmdev_ctx_t dev_ctx);


/**
 * @brief Initializes and configures the Hts221 sensor.
 *
 * @param dev_ctx The device context for communication with the Hts221 sensor.
 *
 */

void SW_Hts221_Init_Config(stmdev_ctx_t dev_ctx);

/**
 * @brief Initializes and configures the Lps22hh sensor.
 *
 * @param dev_ctx The device context for communication with the Lps22hh sensor.
 *
 */

void SW_Lps22hh_Init_Config(stmdev_ctx_t dev_ctx);



/**
 * @brief Determines the north direction relative to the Lis2mdl sensor
 *
 * @return North direction in degrees
 */
double SW_North_Dir(int16_t* data_raw_magnetic);


/**
 * @brief get temperature/humidity/Pressure
 *
 * @return struct Weather
 */
Weather SW_Get_Weather(stmdev_ctx_t hts221_dev_ctx,stmdev_ctx_t lps22hh_dev_ctx);



int32_t Lsm6dso_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t Lsm6dso_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

int32_t Lis2mdl_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t Lis2mdl_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

int32_t lps22hh_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);

int32_t lps22hh_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);

int32_t hts221_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);

int32_t hts221_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);


void platform_delay(uint32_t ms);
