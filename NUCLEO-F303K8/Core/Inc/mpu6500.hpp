#ifndef _MPU6500_HPP_
#define _MPU6500_HPP_
#include "main.h"

#define ADDRESS           0x68
#define WHO_AM_I          0x75

#define PWR_MGMT_1        0x6B
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C

#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40

#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

#define GYRO_FACTOR       16.4
#define ACCEL_FACTOR	  16384.0

class MPU6500 {
private:
	float ax_offset, ay_offset, az_offset;
	float gx_offset, gy_offset, gz_offset;

public:
	MPU6500(int cs);

	int cs;
	float ax, ay, az;
	float gx, gy, gz;

	uint8_t read_byte(uint8_t reg, int cs);
	void write_byte(uint8_t reg, uint8_t data, int cs);
	void MPU6500_Init();
	void MPU6500_OffsetCalc();
	void Get_MPU6500_Data();
};

#endif // _MPU6500_HPP_

