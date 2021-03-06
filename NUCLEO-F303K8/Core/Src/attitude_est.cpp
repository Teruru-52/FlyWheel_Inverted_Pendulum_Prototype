#include "attitude_est.hpp"

Attitude::Attitude() :
		mpu1(1), mpu2(2), init_flag(false) {
}

void Attitude::Init() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	mpu1.MPU6500_Init();
	mpu2.MPU6500_Init();
	mpu1.MPU6500_OffsetCalc();
	mpu2.MPU6500_OffsetCalc();
	mpu1.Get_MPU6500_Data();
	mpu2.Get_MPU6500_Data();

	mx = mpu1.az - mu * mpu2.az;
	my = mpu1.ay - mu * mpu2.ay;
	theta_b = atan(-mx / my);
	kalman.setAngle(theta_b);

	init_flag = true;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

std::array<float, 2> Attitude::GetEstAttitude() {
	mpu1.Get_MPU6500_Data();
	mpu2.Get_MPU6500_Data();

	mx = mpu1.az - mu * mpu2.az;
	my = mpu1.ay - mu * mpu2.ay;
	theta_b = atan(-mx / my);
	dot_theta_b = 0.5 * (mpu1.gx + mpu2.gx);
//	est_attitude[0] = kalman.getAngle(theta_b, dot_theta_b, 0.01);
//	est_attitude[1] = kalman.getRate();
	est_attitude[0] = mpu1.az;
	est_attitude[1] = mpu2.az;

	return est_attitude;
}
