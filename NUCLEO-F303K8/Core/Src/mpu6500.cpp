#include "mpu6500.hpp"
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

MPU6500::MPU6500(int cs) :
		cs(cs){
}
uint8_t MPU6500::read_byte(uint8_t reg, int cs) {
	uint8_t rx_data[2];
	uint8_t tx_data[2];

	tx_data[0] = reg | 0x80;
	tx_data[1] = 0x00;  // dummy

	if (cs == 1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	} else if (cs == 2) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}

	return rx_data[1];
}

void MPU6500::write_byte(uint8_t reg, uint8_t data, int cs) {
	uint8_t rx_data[2];
	uint8_t tx_data[2];

	tx_data[0] = reg & 0x7F;
//   tx_data[0] = reg | 0x00;
	tx_data[1] = data;  // write data

	if (cs == 1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	} else if (cs == 2) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
}

void MPU6500::MPU6500_Init() {
	uint8_t who_am_i;

	HAL_Delay(100);             // wait start up
	who_am_i = read_byte(WHO_AM_I, cs); // read who am i
	printf("who_am_i_%d = 0x%x\r\n", cs, who_am_i); // check who am i value
//	HAL_UART_Transmit(&huart2, (uint8_t*) who_am_i, sizeof(who_am_i), 3000);
	HAL_Delay(10);
	who_am_i = read_byte(WHO_AM_I, cs); // read who am i
		printf("who_am_i_%d = 0x%x\r\n", cs, who_am_i); // check who am i value
	//	HAL_UART_Transmit(&huart2, (uint8_t*) who_am_i, sizeof(who_am_i), 3000);
		HAL_Delay(10);
	if (who_am_i != 0x70) {
		printf("mpu6500_%d error \r\n", cs);
	}

	HAL_Delay(50);
	write_byte(PWR_MGMT_1, 0x00, cs); // set pwr_might (20MHz)
	HAL_Delay(50);
	write_byte(CONFIG, 0x00, cs); // set config (FSYNCはNC)
	HAL_Delay(50);
	write_byte(GYRO_CONFIG, 0x18, cs); // set gyro config (2000dps)
	HAL_Delay(50);
	write_byte(ACCEL_CONFIG, 0x00, cs); // set accel config (2g)
	HAL_Delay(50);
}

void MPU6500::MPU6500_OffsetCalc() {
	int16_t ax_raw, ay_raw, az_raw;
	int16_t gx_raw, gy_raw, gz_raw;
	float ax_sum = 0;
	float ay_sum = 0;
	float az_sum = 0;
	float gx_sum = 0;
	float gy_sum = 0;
	float gz_sum = 0;

	for (int i = 0; i < 1000; i++) {
		// H:8bit shift, Link h and l
		ax_raw = (int16_t) ((uint16_t) (read_byte(ACCEL_XOUT_H, cs) << 8)
				| (uint16_t) read_byte(ACCEL_XOUT_L, cs));
		ay_raw = (int16_t) ((uint16_t) (read_byte(ACCEL_YOUT_H, cs) << 8)
				| (uint16_t) read_byte(ACCEL_YOUT_L, cs));
		az_raw = (int16_t) ((uint16_t) (read_byte(ACCEL_ZOUT_H, cs) << 8)
				| (uint16_t) read_byte(ACCEL_ZOUT_L, cs));
		gx_raw = (int16_t) ((uint16_t) (read_byte(GYRO_XOUT_H, cs) << 8)
				| (uint16_t) read_byte(GYRO_XOUT_L, cs));
		gy_raw = (int16_t) ((uint16_t) (read_byte(GYRO_YOUT_H, cs) << 8)
				| (uint16_t) read_byte(GYRO_YOUT_L, cs));
		gz_raw = (int16_t) ((uint16_t) (read_byte(GYRO_ZOUT_H, cs) << 8)
				| (uint16_t) read_byte(GYRO_ZOUT_L, cs));

		ax_sum += (float) (ax_raw / ACCEL_FACTOR);
		ay_sum += (float) (ay_raw / ACCEL_FACTOR);
		az_sum += (float) (az_raw / ACCEL_FACTOR);
		gx_sum += (float) (gx_raw / GYRO_FACTOR); // dps to deg/sec
		gy_sum += (float) (gy_raw / GYRO_FACTOR);
		gz_sum += (float) (gz_raw / GYRO_FACTOR);
		HAL_Delay(1);
	}
	ax_offset = ax_sum / 1000.0;
	ay_offset = ay_sum / 1000.0;
	az_offset = az_sum / 1000.0;
	gx_offset = gx_sum / 1000.0;
	gy_offset = gy_sum / 1000.0;
	gz_offset = gz_sum / 1000.0;
}

void MPU6500::Get_MPU6500_Data() {
	int16_t ax_raw, ay_raw, az_raw;
	int16_t gx_raw, gy_raw, gz_raw;

	// H:8bit shift, Link h and l
	ax_raw = (int16_t) ((uint16_t) (read_byte(ACCEL_XOUT_H, cs) << 8)
			| (uint16_t) read_byte(ACCEL_XOUT_L, cs));
	ay_raw = (int16_t) ((uint16_t) (read_byte(ACCEL_YOUT_H, cs) << 8)
			| (uint16_t) read_byte(ACCEL_YOUT_L, cs));
	az_raw = (int16_t) ((uint16_t) (read_byte(ACCEL_ZOUT_H, cs) << 8)
			| (uint16_t) read_byte(ACCEL_ZOUT_L, cs));
	gx_raw = (int16_t) ((uint16_t) (read_byte(GYRO_XOUT_H, cs) << 8)
			| (uint16_t) read_byte(GYRO_XOUT_L, cs));
	gy_raw = (int16_t) ((uint16_t) (read_byte(GYRO_YOUT_H, cs) << 8)
			| (uint16_t) read_byte(GYRO_YOUT_L, cs));
	gz_raw = (int16_t) ((uint16_t) (read_byte(GYRO_ZOUT_H, cs) << 8)
			| (uint16_t) read_byte(GYRO_ZOUT_L, cs));

	ax = (float) (ax_raw / ACCEL_FACTOR) - ax_offset;
	ay = (float) (ay_raw / ACCEL_FACTOR) - ay_offset;
	az = (float) (az_raw / ACCEL_FACTOR) - az_offset;
	gx = (float) (gx_raw / GYRO_FACTOR) - gx_offset;
	gy = (float) (gy_raw / GYRO_FACTOR) - gy_offset;
	gz = (float) (gz_raw / GYRO_FACTOR) - gz_offset;
}
