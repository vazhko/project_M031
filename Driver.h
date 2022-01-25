#ifndef MOTOR_DRIVER__H
#define MOTOR_DRIVER__H

#include <cstdint>

namespace Motor
{
	namespace Driver
	{
		using namespace std;

		void Init();
		void SetPWM(int pwm);
		int32_t GetPosition();
		void ResetPosition();

		void UpdateTimer();
	}
}

#endif
