#ifndef TIMER__H__
#define TIMER__H__

namespace Timer
{
	void Delay_us(unsigned long delay);
	void Delay_ms(unsigned long delay);
	
	class Timer
	{
	private:
		static bool FirstInstance;
		unsigned long long StartCountdown;
		unsigned long long EndCountdown;
		unsigned long long Counter;
		bool Running;
	public:
		Timer();
		void Reset();
		void Update();
		void Pause();
		void Resume();
		unsigned long long Elapsed();
	};
}
#endif
