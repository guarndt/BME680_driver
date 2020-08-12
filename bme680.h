/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	bme680.h
 * @date	19 Jun 2018
 * @version	3.5.9
 * @brief
 *
 */
/*! @file bme680.h
 @brief Sensor driver for BME680 sensor */
/*!
 * @defgroup BME680 SENSOR API
 * @{*/
#ifndef BME680_H_
#define BME680_H_

#include "bme680_defs.h"

namespace sensors::bme680 {

struct bme680_dev {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Device Id */
	uint8_t dev_id;
	/*! SPI/I2C interface */
	enum bme680_intf intf;
	/*! Memory page used */
	uint8_t mem_page;
	/*! Ambient temperature in Degree C */
	int8_t amb_temp;
	/*! Sensor calibration data */
	struct bme680_calib_data calib;
	/*! Sensor settings */
	struct bme680_tph_sett tph_sett;
	/*! Gas Sensor settings */
	struct bme680_gas_sett gas_sett;
	/*! Sensor power modes */
	uint8_t power_mode;
	/*! New sensor fields */
	uint8_t new_fields;
	/*! Store the info messages */
	uint8_t info_msg;
	/*! Communication function result */
	int8_t com_rslt;

	/*! Bus read function  */
	virtual int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) = 0;
	/*! Bus write function */
	virtual int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) = 0;
	/*! delay function */
	virtual void delay_ms(uint32_t period) = 0;

	virtual ~bme680_dev() = default;

	/* function prototype declarations */
	/*!
	 *  @brief This API is the entry point.
	 *  It reads the chip-id and calibration data from the sensor.
	 *
	 *  @param[in,out] dev : Structure instance of bme680_dev
	 *
	 *  @return Result of API execution status
	 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t bme680_init();

	/*!
	 * @brief This API writes the given data to the register address
	 * of the sensor.
	 *
	 * @param[in] reg_addr : Register address from where the data to be written.
	 * @param[in] reg_data : Pointer to data buffer which is to be written
	 * in the sensor.
	 * @param[in] len : No of bytes of data to write..
	 * @param[in] dev : Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len);

	/*!
	 * @brief This API reads the data from the given register address of the sensor.
	 *
	 * @param[in] reg_addr : Register address from where the data to be read
	 * @param[out] reg_data : Pointer to data buffer to store the read data.
	 * @param[in] len : No of bytes of data to be read.
	 * @param[in] dev : Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

	/*!
	 * @brief This API performs the soft reset of the sensor.
	 *
	 * @param[in] dev : Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
	 */
	int8_t bme680_soft_reset();

	/*!
	 * @brief This API is used to set the power mode of the sensor.
	 *
	 * @param[in] dev : Structure instance of bme680_dev
	 * @note : Pass the value to bme680_dev.power_mode structure variable.
	 *
	 *  value	|	mode
	 * -------------|------------------
	 *	0x00	|	BME680_SLEEP_MODE
	 *	0x01	|	BME680_FORCED_MODE
	 *
	 * * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t bme680_set_sensor_mode();

	/*!
	 * @brief This API is used to get the power mode of the sensor.
	 *
	 * @param[in] dev : Structure instance of bme680_dev
	 * @note : bme680_dev.power_mode structure variable hold the power mode.
	 *
	 *  value	|	mode
	 * ---------|------------------
	 *	0x00	|	BME680_SLEEP_MODE
	 *	0x01	|	BME680_FORCED_MODE
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t bme680_get_sensor_mode();

	/*!
	 * @brief This API is used to set the profile duration of the sensor.
	 *
	 * @param[in] dev	   : Structure instance of bme680_dev.
	 * @param[in] duration : Duration of the measurement in ms.
	 *
	 * @return Nothing
	 */
	void bme680_set_profile_dur(uint16_t duration);

	/*!
	 * @brief This API is used to get the profile duration of the sensor.
	 *
	 * @param[in] dev	   : Structure instance of bme680_dev.
	 * @param[in] duration : Duration of the measurement in ms.
	 *
	 * @return Nothing
	 */
	void bme680_get_profile_dur(uint16_t *duration) const;

	/*!
	 * @brief This API reads the pressure, temperature and humidity and gas data
	 * from the sensor, compensates the data and store it in the bme680_data
	 * structure instance passed by the user.
	 *
	 * @param[out] data: Structure instance to hold the data.
	 * @param[in] dev : Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t bme680_get_sensor_data(struct bme680_field_data *data);

	/*!
	 * @brief This API is used to set the oversampling, filter and T,P,H, gas selection
	 * settings in the sensor.
	 *
	 * @param[in] dev : Structure instance of bme680_dev.
	 * @param[in] desired_settings : Variable used to select the settings which
	 * are to be set in the sensor.
	 *
	 *	 Macros	                   |  Functionality
	 *---------------------------------|----------------------------------------------
	 *	BME680_OST_SEL             |    To set temperature oversampling.
	 *	BME680_OSP_SEL             |    To set pressure oversampling.
	 *	BME680_OSH_SEL             |    To set humidity oversampling.
	 *	BME680_GAS_MEAS_SEL        |    To set gas measurement setting.
	 *	BME680_FILTER_SEL          |    To set filter setting.
	 *	BME680_HCNTRL_SEL          |    To set humidity control setting.
	 *	BME680_RUN_GAS_SEL         |    To set run gas setting.
	 *	BME680_NBCONV_SEL          |    To set NB conversion setting.
	 *	BME680_GAS_SENSOR_SEL      |    To set all gas sensor related settings
	 *
	 * @note : Below are the macros to be used by the user for selecting the
	 * desired settings. User can do OR operation of these macros for configuring
	 * multiple settings.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
	 */
	int8_t bme680_set_sensor_settings(uint16_t desired_settings);

	/*!
	 * @brief This API is used to get the oversampling, filter and T,P,H, gas selection
	 * settings in the sensor.
	 *
	 * @param[in] dev : Structure instance of bme680_dev.
	 * @param[in] desired_settings : Variable used to select the settings which
	 * are to be get from the sensor.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
	 */
	int8_t bme680_get_sensor_settings(uint16_t desired_settings);

	/*!
	 * @brief This internal API is used to read the calibrated data from the sensor.
	 *
	 * This function is used to retrieve the calibration
	 * data from the image registers of the sensor.
	 *
	 * @note Registers 89h  to A1h for calibration data 1 to 24
	 *        from bit 0 to 7
	 * @note Registers E1h to F0h for calibration data 25 to 40
	 *        from bit 0 to 7
	 * @param[in] dev	:Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status.
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t get_calib_data();

	/*!
	 * @brief This internal API is used to set the gas configuration of the sensor.
	 *
	 * @param[in] dev	:Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status.
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t set_gas_config();

	/*!
	 * @brief This internal API is used to get the gas configuration of the sensor.
	 * @note heatr_temp and heatr_dur values are currently register data
	 * and not the actual values set
	 *
	 * @param[in] dev	:Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status.
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t get_gas_config();

	/*!
	 * @brief This internal API is used to calculate the temperature value.
	 *
	 * @param[in] dev	:Structure instance of bme680_dev.
	 * @param[in] temp_adc	:Contains the temperature ADC value .
	 *
	 * @return uint32_t calculated temperature.
	 */
	int16_t calc_temperature(uint32_t temp_adc);

	/*!
	 * @brief This internal API is used to calculate the pressure value.
	 *
	 * @param[in] dev	:Structure instance of bme680_dev.
	 * @param[in] pres_adc	:Contains the pressure ADC value .
	 *
	 * @return uint32_t calculated pressure.
	 */
	uint32_t calc_pressure(uint32_t pres_adc) const;

	/*!
	 * @brief This internal API is used to calculate the humidity value.
	 *
	 * @param[in] dev	:Structure instance of bme680_dev.
	 * @param[in] hum_adc	:Contains the humidity ADC value.
	 *
	 * @return uint32_t calculated humidity.
	 */
	uint32_t calc_humidity(uint16_t hum_adc);

	/*!
	 * @brief This internal API is used to calculate the Gas Resistance value.
	 *
	 * @param[in] dev		:Structure instance of bme680_dev.
	 * @param[in] gas_res_adc	:Contains the Gas Resistance ADC value.
	 * @param[in] gas_range		:Contains the range of gas values.
	 *
	 * @return uint32_t calculated gas resistance.
	 */
	uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range) const;

	/*!
	 * @brief This internal API is used to calculate the Heat Resistance value.
	 *
	 * @param[in] dev	: Structure instance of bme680_dev
	 * @param[in] temp	: Contains the target temperature value.
	 *
	 * @return uint8_t calculated heater resistance.
	 */
	uint8_t calc_heater_res(uint16_t temp) const;

	/*!
	 * @brief This internal API is used to calculate the field data of sensor.
	 *
	 * @param[out] data :Structure instance to hold the data
	 * @param[in] dev	:Structure instance of bme680_dev.
	 *
	 *  @return int8_t result of the field data from sensor.
	 */
	int8_t read_field_data(struct bme680_field_data *data);

	/*!
	 * @brief This internal API is used to set the memory page
	 * based on register address.
	 *
	 * The value of memory page
	 *  value  | Description
	 * --------|--------------
	 *   0     | BME680_PAGE0_SPI
	 *   1     | BME680_PAGE1_SPI
	 *
	 * @param[in] dev	:Structure instance of bme680_dev.
	 * @param[in] reg_addr	:Contains the register address array.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t set_mem_page(uint8_t reg_addr);

	/*!
	 * @brief This internal API is used to get the memory page based
	 * on register address.
	 *
	 * The value of memory page
	 *  value  | Description
	 * --------|--------------
	 *   0     | BME680_PAGE0_SPI
	 *   1     | BME680_PAGE1_SPI
	 *
	 * @param[in] dev	:Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t get_mem_page();

	/*!
	 * @brief This internal API is used to check the boundary
	 * conditions.
	 *
	 * @param[in] value	:pointer to the value.
	 * @param[in] min	:minimum value.
	 * @param[in] max	:maximum value.
	 * @param[in] dev	:Structure instance of bme680_dev.
	 *
	 * @return Result of API execution status
	 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
	 */
	int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max);

};
#endif /* BME680_H_ */
/** @}*/
}
