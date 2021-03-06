#ifndef _ATTITDE_EST_HPP_
#define _ATTITDE_EST_HPP_
#include "main.h"
#include "mpu6500.hpp"
#include "kalman.hpp"
#include <array>

class Attitude{
private:
	Kalman kalman;
	MPU6500 mpu1;
	MPU6500 mpu2;

	float mu = 9.0;
	float mx, my;
	float theta_b, dot_theta_b;
	std::array<float, 2> est_attitude;

public:
	Attitude();

	bool init_flag;

	void Init();
	std::array<float, 2> GetEstAttitude();

};

#endif // _ATTITDE_EST_HPP_
