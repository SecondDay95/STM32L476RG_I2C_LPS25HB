#include "lps25hb.h"
#include "i2c.h"

#define LPS25HB_ADDR        		0xBA

#define LPS25HB_WHO_AM_I 			0x0F
#define LPS25HB_CTRL_REG1 			0x20
#define LPS25HB_CTRL_REG2 			0x21
#define LPS25HB_CTRL_REG3 			0x22
#define LPS25HB_CTRL_REG4 			0x23
#define LPS25HB_PRESS_OUT_XL 		0x28
#define LPS25HB_PRESS_OUT_L 		0x29
#define LPS25HB_PRESS_OUT_H 		0x2A
#define LPS25HB_TEMP_OUT_L 			0x2B
#define LPS25HB_TEMP_OUT_H 			0x2C
#define LPS25HB_RPDS_L 				0x39
#define LPS25HB_RPDS_H 				0x3A

#define TIMEOUT                 	100

#define LPS25HB_CTRL_REG1_PD 		0x80
#define LPS25HB_CTRL_REG1_ODR2 		0x40
#define LPS25HB_CTRL_REG1_ODR1 		0x20
#define LPS25HB_CTRL_REG1_ODR0 		0x10
#define LPS25HB_CTRL_REG1_RESET_AZ 	0x02

//Funkcja odczytujaca zawartosc rejestru WHO_AM_I do inicjalizacji czujnika:
static uint8_t lps_read_reg_init(void) {

	uint8_t value = 0;
	HAL_I2C_Mem_Read(&hi2c1, LPS25HB_ADDR, LPS25HB_WHO_AM_I, 1, &value, sizeof(value), TIMEOUT);

	return value;

}

//Funkcja odczytujaca zawartosc rejestru z czujnika:
static uint8_t lps_read_reg(uint8_t reg) {

	uint8_t value = 0;
	if(HAL_I2C_Mem_Read(&hi2c1, LPS25HB_ADDR, reg, 1, &value, sizeof(value), TIMEOUT) != HAL_OK)
		Error_Handler();

	return value;

}

//Funkcja zapisujaca wartosc w rejestrze czujnika:
static void lps_write_reg(uint8_t reg, uint8_t value) {

	if(HAL_I2C_Mem_Write(&hi2c1, LPS25HB_ADDR, reg, 1, &value, sizeof(value), TIMEOUT) != HAL_OK)
		Error_Handler();

}

HAL_StatusTypeDef lps25hb_init(void) {

	if(lps_read_reg_init() != 0xBD) {
	//if(lps_read_reg(LPS25HB_WHO_AM_I) != 0xBD) {

		return HAL_ERROR;

	}

	lps_write_reg(LPS25HB_CTRL_REG1, LPS25HB_CTRL_REG1_PD | LPS25HB_CTRL_REG1_ODR2);
	return HAL_OK;

}

float lps25hp_read_temp(void) {

	  //Odczyt temperatury z czujnika LPS25HB:
	  int16_t temp;
	  uint8_t temp_lsb;
	  uint8_t temp_msb;
	  temp_lsb = lps_read_reg(LPS25HB_TEMP_OUT_L);
	  temp_msb = lps_read_reg(LPS25HB_TEMP_OUT_H);
	  temp = (temp_msb << 8) | temp_lsb;

	  return 42.5f + temp / 480.0f;

}

float lps25hb_read_pressure(void) {

	  //Odczyt ciśnienia bezwzględnego z czujnika LPS25HB:
	  int32_t pressure = 0;
	  uint8_t pressure_l;
	  uint8_t pressure_h;
	  uint8_t pressure_xl;
	  pressure_l = lps_read_reg(LPS25HB_PRESS_OUT_L);
	  pressure_h = lps_read_reg(LPS25HB_PRESS_OUT_H);
	  pressure_xl = lps_read_reg(LPS25HB_PRESS_OUT_XL);
	  pressure = (pressure_h << 16) | (pressure_l << 8) | pressure_xl;

	  return pressure / 4096.0f;

}

void lps25hb_set_calib(int16_t value) {

	//Przeslanie mniej znaczacych 8 bitow do rejestru kalibrujacego RPDS_L
	lps_write_reg(LPS25HB_RPDS_L, value);
	//Przeslanie bardziej znaczacych 8 bitow do rejestru kalibrujacego RPDS_H
	lps_write_reg(LPS25HB_RPDS_H, value >> 8);

}

