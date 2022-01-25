#include "Led.h"
#include "NuMicro.h"
#include "nuTimer.h"


namespace Led
{
	void LedRed( bool on )
	{
		static bool init = true;
		if (init)
		{
			GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
		}

                        PB14 = 1;
	}

	Timer::Timer LedTimer;
	_RedState RedLedState = _Off;
	uint8_t RedCounter = 0;
	bool RedState = false;

	void Init()
	{
		LedRed( false );
	}
	void Module()
	{
		if (LedTimer.Elapsed() > 1000*10)
		{
			LedTimer.Reset();

			if( RedLedState == _On ) LedRed( true );
			if( RedLedState == _Off ) LedRed( false );
			if( RedLedState == _1Hz || RedLedState == _3Hz )
			{
				if( RedState )
				{
					if( RedCounter < 1 )
					{
						if( RedLedState == _1Hz) RedCounter = 50;
						if( RedLedState == _3Hz) RedCounter = 15;
						RedState = false;
					}
					RedCounter--;
				}
				else
				{
					if( RedCounter < 1 )
					{
						if( RedLedState == _1Hz) RedCounter = 50;
						if( RedLedState == _3Hz) RedCounter = 15;
						RedState = true;
					}
					RedCounter--;
				}
				LedRed( RedState );
			}

		}

	}
	void Red( _RedState state )
	{
		RedLedState = state;
	}

}

