#include "nuTimer.h"
#include "timer.h"
#include "NuMicro.h"

namespace Timer
{
	void Init()
	{

              TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1);
              TIMER_Start(TIMER0);

	}
	void Delay_us(unsigned long delay)
	{
		Timer timer;
		while (timer.Elapsed() < delay);
	}
	void Delay_ms(unsigned long delay)
	{
		Delay_us((unsigned long long)delay * 1000);
	}

	unsigned long Get()
	{
		unsigned long temp = TIMER_GetCounter(TIMER0);
		return temp;
	}

	bool Timer::FirstInstance = true;
	Timer::Timer()
	{
		if (FirstInstance)
		{
			FirstInstance = false;
			Init();
		}
		Running = true;
		Reset();
	}
	void Timer::Reset()
	{
		StartCountdown = Get();
		EndCountdown = Get();
		Counter = 0;
	}
	void Timer::Update()
	{
		EndCountdown = Get();

		if (Running == false) StartCountdown = EndCountdown;

		unsigned long long elapsed;
		if(StartCountdown <= EndCountdown) elapsed = EndCountdown - StartCountdown;
		else elapsed = (0xFFFF) - (StartCountdown - EndCountdown) + 1;
		Counter += elapsed;

		StartCountdown = EndCountdown;
	}
	void Timer::Pause()
	{
		Update();
		Running = false;
	}
	void Timer::Resume()
	{
		Update();
		Running = true;
	}
	unsigned long long Timer::Elapsed()
	{
		Update();
		unsigned long long temp = Counter;
		return temp;
	}
}
